//! Live 1 Hz multi-constellation tracker that OWNS the radio.
//!
//! Same engine as live_track, but streams from the HackRF Pro directly via
//! rs-hackrf (vendored, extended with open_by_serial + clock correction).
//! Owning the device handle is what makes the DISCIPLINE LOOP possible:
//! hackrf_open is exclusive, so when hackrf_transfer held the radio no other
//! process could touch the clock-correction register (the v1 loop's actuator
//! was unreachable behind the tracker). Here the same process streams and
//! steers — control transfers interleave with bulk reads on one handle.
//!
//! Discipline v2 (the v1 runaway's lessons built in):
//!   - feedback is UNSTEERED: mean Doppler of locked SBAS/WAAS GEO channels
//!     (geographic ~zero-Doppler, GPS-disciplined transmitters), never the
//!     ATSC phase tracker (whose own integrator absorbs ramps — feeding on
//!     it sent v1 open-loop to +1.9 ppm)
//!   - slew, never leap: <= STEP_MAX ppm per cycle
//!   - excursion clamp at +/- CLAMP ppm
//!   - stall detector (coarse mode only): 3 steps without >= 20%
//!     improvement -> stop + alarm
//!   - residual-plausibility gate (discipline::PlausibilityGate): no write
//!     while the locked-channel count collapses, while a feeding WAAS/GEO
//!     channel is freshly relocked, on an impossible residual slew vs the
//!     last accepted residual, or on stale inputs — the 2026-08-25 bogus
//!     -0.37 ppm class (docs/p0c_clock_continuity.md)
//!
//! stdout: one JSON line per tracked PRN per second (same as live_track),
//! plus {"discipline": {...}} once per cycle. usage: live_radio [serial]

use hackrf_gnss::discipline::PlausibilityGate;
use hackrf_gnss::live::{Engine, Sys};
use rs_hackrf::HackRf;
use std::io::{self, BufWriter, Write};
use std::time::{SystemTime, UNIX_EPOCH};

const FS: f64 = 16.0e6;
const FC: u64 = 1_568_250_000;
const F_L1: f64 = 1575.42e6;
const PRO: &str = "0000000000000000977c64de2b557213";
const CACHE: &str = "/Volumes/Radiator 8TB/gnss/observations/tracker_seed_cache.json";
const CORR_CACHE: &str = "/Volumes/Radiator 8TB/gnss/observations/tracker_corr_cache.json";
const EPH_CACHE: &str = "/Volumes/Radiator 8TB/gnss/observations/tracker_eph.json";
const TICK_STATE: &str = "/Volumes/Radiator 8TB/gnss/observations/state.tick.json";

const DISC_EVERY_S: f64 = 60.0;
const STEP_MAX_PPM: f64 = 0.1;
const CLAMP_PPM: f64 = 2.0;
const DEADBAND_PPM: f64 = 0.01;
const STALL_AFTER: usize = 3;

fn now_f64() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// Declare the consumer thread real-time to the Mach scheduler: 8 ms of
/// CPU guaranteed every 16 ms period. This is the CoreAudio mechanism —
/// without it, any concurrent batch load (the other session's acquisition
/// jobs) deschedules the consumer for seconds, the USB stream overflows,
/// and every channel's phase dies in the gap. With it, the scheduler
/// preempts batch work for us instead.
#[cfg(target_os = "macos")]
fn set_realtime() {
    #[repr(C)]
    struct TimeConstraint {
        period: u32,
        computation: u32,
        constraint: u32,
        preemptible: u32,
    }
    unsafe extern "C" {
        fn mach_thread_self() -> u32;
        fn thread_policy_set(tid: u32, flavor: i32, info: *const TimeConstraint, count: u32) -> i32;
        fn mach_timebase_info(info: *mut TimebaseInfo) -> i32;
    }
    #[repr(C)]
    struct TimebaseInfo {
        numer: u32,
        denom: u32,
    }
    const THREAD_TIME_CONSTRAINT_POLICY: i32 = 2;
    const COUNT: u32 = 4; // THREAD_TIME_CONSTRAINT_POLICY_COUNT
    // Values are in mach absolute-time ticks (NOT ns — the first attempt
    // assumed ns and the kernel rejected the 606 ms period, kr=4). Convert
    // via the timebase: ns_per_tick = numer/denom, so ticks_per_ms =
    // 1e6 * denom/numer (first attempt had it flipped — kr=4 again).
    let mut tb = TimebaseInfo { numer: 0, denom: 0 };
    unsafe { mach_timebase_info(&mut tb) };
    let ticks_per_ms = (1e6 * tb.denom as f64 / tb.numer.max(1) as f64) as u32;
    let p = TimeConstraint {
        period: 16 * ticks_per_ms,
        computation: 8 * ticks_per_ms,
        constraint: 12 * ticks_per_ms,
        preemptible: 1,
    };
    let kr = unsafe {
        thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY, &p, COUNT)
    };
    eprintln!("live_radio: real-time thread policy set, kern_return {kr}");
}

#[cfg(not(target_os = "macos"))]
fn set_realtime() {}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let serial = a.get(1).map(|s| s.as_str()).unwrap_or(PRO);
    let epoch0 = now_f64();
    set_realtime();
    // Discipline actuation gate. The 2026-08-25 retro analysis of the
    // telemetry archive showed every observable correction write (122/122,
    // down to +-0.01 ppm dither steps) followed by a tracker-wide lock
    // collapse and ~1 min relock: the firmware write path disables SGPIO,
    // reprograms Si5351 MS0/MS1 and resets PLL-A (radio.c/clock_gen.c), and
    // note_clock_step cannot save the loops through it. So the loop runs
    // SHADOW by default — residuals and corrections are computed, logged and
    // published, but the hardware is never written. HACKRF_GNSS_ACTUATE=1
    // re-enables actuation once a capture-boundary-safe write path exists.
    let actuate =
        std::env::var("HACKRF_GNSS_ACTUATE").map(|v| v == "1").unwrap_or(false);
    if !actuate {
        eprintln!(
            "live_radio: discipline loop in SHADOW mode (set HACKRF_GNSS_ACTUATE=1 to actuate)"
        );
    }

    // Radio thread: the ASYNC streaming reader keeps 24 bulk transfers
    // queued (~190 ms of device-side slack) — the synchronous read_sync
    // leaves an inter-transfer gap with no host read pending, and the
    // HackRF's FIFO overflowed in that gap (rhythmic sample loss). 8
    // transfers (64 ms) proved too thin under host build load: cargo/
    // nextpnr stalls of ~115 ms overflowed the queue every few minutes,
    // and each overflow forced a full channel realign — the churn that
    // kept anchors from maturing. Corrections go through the control
    // handle on the main thread, interleaved between transfers.
    let (tx, rx) = crossbeam_channel::bounded::<Vec<u8>>(512); // ~8 s of stream
    let drops = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
    let drops_r = drops.clone();
    let serial_owned = serial.to_string();
    let dev = match HackRf::open_by_serial(&serial_owned) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("live_radio: open {serial_owned} failed: {e}");
            std::process::exit(2);
        }
    };
    let r = (|| -> rs_hackrf::error::Result<()> {
        dev.set_sample_rate(FS as u32)?;
        dev.set_freq(FC)?;
        dev.set_lna_gain(40)?;
        dev.set_vga_gain(46)?;
        dev.set_amp_enable(false)?;
        dev.set_antenna_enable(true)?; // AA.250 dual-stage LNA needs bias
        // CLKOUT ownership follows radio ownership: this process holds the
        // Pro 24/7, and the One is CLKIN-slaved to its 10 MHz. Radio config
        // (and any flash) can drop CLKOUT — re-assert it on every startup.
        dev.set_clkout_enable(true)?;
        Ok(())
    })();
    if let Err(e) = r {
        eprintln!("live_radio: radio config failed: {e}");
        std::process::exit(2);
    }
    let stream = match dev.into_streaming_reader(24, 262144) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("live_radio: start streaming failed: {e}");
            std::process::exit(2);
        }
    };
    let radio_ctrl = stream.control_handle();
    eprintln!("live_radio: streaming 16 Msps @ 1568.25 MHz from {serial_owned}");
    std::thread::spawn(move || {
        set_realtime(); // the reader must also preempt batch load
        let mut acc: Vec<u8> = Vec::with_capacity(512 * 1024);
        while let Some(chunk) = stream.recv() {
            match chunk {
                Ok(data) => {
                    acc.extend_from_slice(&data);
                    if acc.len() >= 512 * 1024 {
                        let full =
                            std::mem::replace(&mut acc, Vec::with_capacity(512 * 1024));
                        let l = full.len() as u64;
                        if tx.try_send(full).is_err() {
                            drops_r.fetch_add(l, std::sync::atomic::Ordering::Relaxed);
                        }
                    }
                }
                Err(e) => {
                    eprintln!("live_radio: stream error: {e} — exiting for restart");
                    std::process::exit(3);
                }
            }
        }
        eprintln!("live_radio: stream closed — exiting");
        std::process::exit(3);
    });

    let mut eng = Engine::new(FS, FC as f64, epoch0);
    // Seed cache: skip the blind all-sky seed after a restart.
    if let Ok(text) = std::fs::read_to_string(CACHE) {
        if let Ok(v) = serde_json::from_str::<serde_json::Value>(&text) {
            if v["epoch"].as_f64().map(|e| epoch0 - e < 600.0).unwrap_or(false) {
                let (mut l1, mut b1i) = (Vec::new(), Vec::new());
                for s in v["sats"].as_array().into_iter().flatten() {
                    let (Some(sys), Some(prn), Some(dopp)) = (
                        s["sys"].as_str().and_then(Sys::from_name),
                        s["prn"].as_u64().map(|p| p as usize),
                        s["dopp"].as_f64(),
                    ) else { continue };
                    match sys {
                        Sys::Beidou => b1i.push((sys, prn, dopp)),
                        _ => l1.push((sys, prn, dopp)),
                    }
                }
                eprintln!(
                    "live_radio: seed cache — {} L1 + {} B1I candidates",
                    l1.len(),
                    b1i.len()
                );
                eng.l1_band.prime_candidates(l1);
                eng.b1i_band.prime_candidates(b1i);
            }
        }
    }

    let stdout = io::stdout();
    let mut out = BufWriter::new(stdout.lock());
    let mut skip_bytes = (FS * 2.0) as u64; // 1 s startup transient
    let mut bytes_in = 0u64;
    let mut last_log = 0u64;
    let mut last_drops = 0u64;
    let mut last_hw_drops = 0u64;
    let mut max_proc_ms = 0.0f64;
    let mut last_cache_write = 0.0f64;

    // discipline state
    // the register survives process restarts (only a reflash zeroes it),
    // so the model must too — otherwise the loop re-learns the offset over
    // several cycles
    let mut corr: f64 = std::fs::read_to_string(CORR_CACHE)
        .ok()
        .and_then(|t| t.trim().parse().ok())
        .unwrap_or(0.0);
    // Sign: resid := doppler_meas/f, and downconversion negates the clock
    // error (f_bb = f_rf - f_lo), so resid = -(clock err) and the register
    // drives resid_new = resid - corr_step. Zero-seeking is corr += resid.
    // Verified live: corr 0->+0.4 marched resid -0.37->-0.61 (slope -1).
    // NB: the Iridium/tick estimators carry the OPPOSITE sign convention.
    // sign = +1 hardcoded: established empirically by the corr/resid march
    // (see above). The lock-noise made poke-probe verdicts inconclusive;
    // the stall detector + clamp are the protections now.
    let sign = 1.0f64;
    let mut steps: Vec<f64> = Vec::new(); // residuals at each applied step
    let mut stalled = false;
    let mut last_disc = 0.0f64;
    let mut last_corr_written = corr;
    // residual-plausibility gate: no correction write on a dying tracker's
    // measurement (collapse / fresh relock / impossible slew / stale inputs)
    let mut gate = PlausibilityGate::new();
    // wall time of the newest 1 Hz channel report (the gate's staleness ref)
    let mut last_report_wall = now_f64();

    // Tick counter sampling: the in-process replacement for sync_producer
    // (an external 2-s SPI poller can never open the radio while we own it
    // 24/7 — its attempts were pure USB contention). 5 s cadence, 200-sample
    // ring, published as state.tick.json in the shape the panel/series
    // producers already consume.
    let mut tick_hist: std::collections::VecDeque<(f64, f64)> =
        std::collections::VecDeque::new();
    let mut last_tick: Option<(f64, u64)> = None;
    let mut last_tick_read = 0.0f64;
    let mut last_tick_write = 0.0f64;
    let mut tick_ref_rate: Option<f64> = None;

    while let Ok(chunk) = rx.recv() {
        let iter_t0 = std::time::Instant::now();
        if skip_bytes > 0 {
            let n = skip_bytes.min(chunk.len() as u64) as usize;
            skip_bytes -= n as u64;
            if n == chunk.len() {
                continue;
            }
            bytes_in += (chunk.len() - n) as u64;
            let reports = eng.push_i8(&chunk[n..]);
            if !reports.is_empty() {
                last_report_wall = now_f64();
            }
            emit(reports, &mut out);
            continue;
        }
        bytes_in += chunk.len() as u64;
        let reports = eng.push_i8(&chunk);
        if !reports.is_empty() {
            last_report_wall = now_f64();
        }
        emit(reports, &mut out);
        let proc_ms = iter_t0.elapsed().as_secs_f64() * 1e3;
        if proc_ms > max_proc_ms {
            max_proc_ms = proc_ms;
        }

        let d_soft = drops.load(std::sync::atomic::Ordering::Relaxed);
        // device-side overflow: transfers the streaming thread couldn't
        // queue because WE didn't drain it (each chunk = one transfer)
        let hw = radio_ctrl.dropped_chunks();
        let hw_bytes = (hw - last_hw_drops) * 262144;
        last_hw_drops = hw;
        let d = d_soft + hw_bytes;
        if d != last_drops {
            let gap_bytes = d - last_drops;
            last_drops = d;
            // One dropped batch = a 16 ms hole: the loops coast through
            // that (PLL re-settles in ~50 ms, code drift ~0.02 chips).
            // force_reseed on ANY drop was the gap-storm amplifier: a
            // single 512 KiB USB clump-drop killed every channel, and the
            // reseeds made the next storm worse. Only a BIG gap (>= ~100 ms
            // of stream) actually invalidates the phases.
            const BIG_GAP: u64 = 3_200_000; // ~100 ms at 32 MB/s
            if gap_bytes < BIG_GAP {
                eng.note_gap_bytes(gap_bytes); // keep stream time honest
            } else {
                eprintln!(
                    "live_radio: big gap {:.1} ms — realigning channels at the edge",
                    gap_bytes as f64 / 32e6 * 1e3
                );
                let mut total = gap_bytes;
                while let Ok(stale) = rx.try_recv() {
                    total += stale.len() as u64;
                }
                eng.note_gap_bytes(total);
                if eng.l1_band.seeded() || eng.b1i_band.seeded() {
                    eng.request_reseed();
                }
            }
        }

        let now_wall = now_f64();
        // persist the seed cache every 30 s while anything is tracked
        if now_wall - last_cache_write > 30.0
            && (!eng.l1_band.channels.is_empty() || !eng.b1i_band.channels.is_empty())
        {
            last_cache_write = now_wall;
            let mut sats = Vec::new();
            for (sys, prn, dopp) in eng
                .l1_band
                .candidates_snapshot()
                .into_iter()
                .chain(eng.b1i_band.candidates_snapshot())
            {
                sats.push(serde_json::json!({"sys": sys.name(), "prn": prn, "dopp": dopp}));
            }
            let doc = serde_json::json!({"epoch": now_wall, "sats": sats});
            let tmp = format!("{CACHE}.tmp");
            if std::fs::write(&tmp, doc.to_string()).is_ok() {
                let _ = std::fs::rename(&tmp, CACHE);
            }
            // self-decoded ephemerides (live LNAV subframes 1-3)
            let ephs = eng.ephemerides();
            if !ephs.is_empty() {
                let doc = serde_json::json!({"epoch": now_wall, "ephemeris": ephs});
                let tmp = format!("{EPH_CACHE}.tmp");
                if std::fs::write(&tmp, doc.to_string()).is_ok() {
                    let _ = std::fs::rename(&tmp, EPH_CACHE);
                }
            }
        }

        // ---- discipline cycle ----
        if now_wall - last_disc >= DISC_EVERY_S {
            last_disc = now_wall;
            // (doppler, lock age) of the locked WAAS/GEO channels
            let waas: Vec<(f64, f64)> = eng
                .l1_band
                .channels
                .iter()
                .filter(|c| c.sys == Sys::Sbas && c.lock_s > 5.0)
                .map(|c| (c.debug_dopp(), c.lock_s))
                .collect();
            gate.observe_locked(
                eng.l1_band
                    .channels
                    .iter()
                    .chain(eng.b1i_band.channels.iter())
                    .filter(|c| c.sys == Sys::Sbas && c.lock_s > 0.0)
                    .count(),
            );
            let mut note = String::new();
            let mut resid: Option<f64> = None;
            if stalled {
                note = "STALLED — residual did not improve over 3 steps; holding".into();
            } else if waas.is_empty() {
                note = "no locked WAAS GEO — holding".into();
            } else {
                let mean = waas.iter().map(|w| w.0).sum::<f64>() / waas.len() as f64;
                let r = mean / F_L1 * 1e6; // unsteered clock residual
                resid = Some(r);
                let waas_locks: Vec<f64> = waas.iter().map(|w| w.1).collect();
                // measurement context untrustworthy: suppress the write and
                // say why (the note rides the discipline line into
                // state.tracker.json, so the panel shows the gate working)
                if let Err(reason) = gate.check(r, &waas_locks, now_wall - last_report_wall) {
                    note = format!("gate: {reason} — suppressed write (residual {r:+.4} ppm)");
                    eprintln!("live_radio: {note}");
                } else if gate.recovered {
                    note = format!("GATE RECOVERY — slew reference re-anchored after latch-up (residual {r:+.4} ppm)");
                    eprintln!("live_radio: {note}");
                } else if r.abs() < DEADBAND_PPM {
                    note = format!("in deadband ({r:+.4} ppm) — loop closed");
                } else {
                    // two modes: coarse steps converge fast, fine steps stop
                    // the dither-oscillation around zero (0.1 steps vs 0.01
                    // deadband oscillated and tripped the stall detector)
                    let fine = r.abs() < 0.15;
                    let step_limit = if fine { 0.03 } else { STEP_MAX_PPM };
                    let target = (corr + sign * r).clamp(-CLAMP_PPM, CLAMP_PPM);
                    let step = (target - corr).clamp(-step_limit, step_limit);
                    let new_corr = ((corr + step) * 1e4).round() / 1e4;
                    if !actuate {
                        // shadow: compute and log the would-be correction,
                        // never touch the hardware (see the actuation gate at
                        // startup); corr stays at the value the radio
                        // actually holds
                        note = format!(
                            "SHADOW: would apply {new_corr:+.4} ppm (residual {r:+.4}) — actuation disabled"
                        );
                    // Only advance the software bookkeeping if the hardware
                    // actually accepted the write + retune — otherwise the
                    // loop's belief and the radio's state diverge silently.
                    } else if let Err(e) = radio_ctrl.set_clock_corr_ppm(new_corr) {
                        note = format!("clock-corr write FAILED ({e}) — software state not advanced");
                        eprintln!("live_radio: {note}");
                    } else if let Err(e) = radio_ctrl.tune(FC) {
                        // the correction WRITE already landed in hardware —
                        // software must adopt it even though the retune
                        // failed, or the loop's belief diverges from the
                        // radio (review round 6). Log loudly; the failed
                        // retune only means the LO may not have re-synced.
                        corr = new_corr;
                        note = format!("retune FAILED ({e}) after successful write — adopted {corr:+.4} ppm to match hardware");
                        eprintln!("live_radio: {note}");
                    } else {
                        corr = new_corr;
                        // Firmware truth (radio.c): mid-stream, the correction
                        // only re-programs the AFE/sample clock. The LO synth
                        // is NOT re-programmed unless a frequency update runs
                        // (its `freq_lo != applied_lo` guard skips when only
                        // the correction changed) — so the RX path sees the
                        // correction ONLY at config time. A same-freq retune
                        // forces it; note_clock_step keeps the loops on the
                        // signal through the sub-ms relock.
                        eng.note_clock_step(step); // keep the loops on the signal
                        steps.push(r);
                        if note.is_empty() {
                            note = format!("applied {corr:+.4} ppm (residual {r:+.4})");
                        }
                    // stall detector only in coarse mode: near zero the
                    // loop dithers within measurement noise by design
                    if !fine && steps.len() > STALL_AFTER {
                        let improved =
                            r.abs() < 0.8 * steps[steps.len() - 1 - STALL_AFTER].abs();
                        if !improved {
                            stalled = true;
                            note = format!(
                                "STALLED — |resid| {r:+.4} not improving over {STALL_AFTER} steps; holding at {corr:+.4} ppm"
                            );
                        }
                    }
                    }
                }
            }
            if corr != last_corr_written {
                last_corr_written = corr;
                let _ = std::fs::write(CORR_CACHE, format!("{corr}"));
            }
            let line = serde_json::json!({"discipline": {
                "epoch": now_wall,
                "residual_ppm": resid,
                "correction_ppm": corr,
                "step_limit_ppm": STEP_MAX_PPM,
                "clamp_ppm": CLAMP_PPM,
                "sign": sign,
                "stalled": stalled,
                "waas_locked": waas.len(),
                "note": note,
            }});
            let _ = writeln!(out, "{}", line);
            let _ = out.flush();
        }

        // ---- tick counter sample (in-process sync_producer replacement) ----
        if now_wall - last_tick_read >= 5.0 {
            last_tick_read = now_wall;
            if let Some(ticks) = radio_ctrl.ts_read_now() {
                if let Some((pt, pticks)) = last_tick {
                    let dt = now_wall - pt;
                    let rate = (ticks.wrapping_sub(pticks)) as f64 / dt;
                    if dt > 0.5 && (30e6..42e6).contains(&rate) {
                        // drop transport glitches; first valid rate is the
                        // session reference for the drift line
                        tick_ref_rate.get_or_insert(rate);
                        tick_hist.push_back((now_wall, rate));
                        if tick_hist.len() > 200 {
                            tick_hist.pop_front();
                        }
                    }
                }
                last_tick = Some((now_wall, ticks));
            }
            if now_wall - last_tick_write >= 30.0 && !tick_hist.is_empty() {
                last_tick_write = now_wall;
                let rate = tick_hist.back().unwrap().1;
                let ref_rate = tick_ref_rate.unwrap_or(rate);
                let recent: Vec<serde_json::Value> = tick_hist
                    .iter()
                    .map(|&(t, r)| serde_json::json!([t, r]))
                    .collect();
                let doc = serde_json::json!({
                    "epoch": now_wall, "ttl_s": 120,
                    "clock": {
                        "live_tick_hz": rate,
                        "tick_rate_drift_ppm_vs_session_ref": (rate / ref_rate - 1.0) * 1e6,
                        "samples": tick_hist.len(),
                        "note": "in-process read via the live_radio control handle; PC-read jitter applies",
                        "recent": recent,
                    },
                });
                let tmp = format!("{TICK_STATE}.tmp");
                if std::fs::write(&tmp, doc.to_string()).is_ok() {
                    let _ = std::fs::rename(&tmp, TICK_STATE);
                }
            }
        }

        if bytes_in / (2 * FS as u64 * 2) != last_log {
            last_log = bytes_in / (2 * FS as u64 * 2);
            eprintln!(
                "live_radio: {:.1} s in, l1 {} chans [{}], b1i {} chans [{}], queue {}, corr {:+.4}, maxproc {:.0} ms",
                bytes_in as f64 / 2.0 / FS,
                eng.l1_band.channels.len(),
                eng.l1_band.status(),
                eng.b1i_band.channels.len(),
                eng.b1i_band.status(),
                rx.len(),
                corr,
                max_proc_ms
            );
            max_proc_ms = 0.0;
        }
    }
    let _ = out.flush();
    eprintln!("live_radio: stream ended — exiting");
}

fn emit(mut reports: Vec<hackrf_gnss::live::SatReport>, out: &mut BufWriter<io::StdoutLock>) {
    if reports.is_empty() {
        return;
    }
    let now = now_f64();
    for r in reports.iter_mut() {
        r.epoch = now;
        if let Ok(line) = serde_json::to_string(r) {
            let _ = writeln!(out, "{}", line);
        }
    }
    let _ = out.flush();
}
