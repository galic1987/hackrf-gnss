//! Live 1 Hz multi-constellation tracker that OWNS the radio.
//!
//! Same engine as live_track, but streams from the HackRF Pro directly via
//! rs-hackrf (vendored, extended with open-by-serial and timestamp access).
//! Owning the device handle makes sample/timestamp telemetry deterministic:
//! hackrf_open is exclusive, so a second capture cannot share the stream.
//! Production discipline is SHADOW-ONLY. The historical correction-write
//! path collapsed all tracker locks on 122/122 observed writes and has been
//! removed from this binary; a runtime environment variable cannot restore it.
//!
//! Discipline v2 (the v1 runaway's lessons built in):
//!   - feedback is UNSTEERED: mean Doppler of locked SBAS/WAAS GEO channels
//!     (geographic ~zero-Doppler, GPS-disciplined transmitters), never the
//!     ATSC phase tracker (whose own integrator absorbs ramps — feeding on
//!     it sent v1 open-loop to +1.9 ppm)
//!   - proposed slew <= STEP_MAX ppm per cycle
//!   - proposed excursion clamp at +/- CLAMP ppm
//!   - residual-plausibility gate (discipline::PlausibilityGate): no proposal
//!     while the locked-channel count collapses, while a feeding WAAS/GEO
//!     channel is freshly relocked, on an impossible residual slew vs the
//!     last accepted residual, or on stale inputs — the 2026-08-25 bogus
//!     -0.37 ppm class (docs/p0c_clock_continuity.md)
//!
//! stdout: one JSON line per tracked PRN per second (same as live_track),
//! plus {"discipline": {...}} once per cycle. Production usage is through
//! tracker_producer.py; direct launches are rejected unless the child can
//! prove the wrapper's token-owned Pro lease.

use hackrf_gnss::discipline::PlausibilityGate;
use hackrf_gnss::live::{Engine, Sys};
use rs_hackrf::HackRf;
use std::io::{self, BufWriter, Write};
use std::time::{SystemTime, UNIX_EPOCH};

const FS: f64 = 16.0e6;
const FC: u64 = 1_568_250_000;
const F_L1: f64 = 1575.42e6;
/// Production radio: Pro#2. Pro#1 (…977c64de2b557213) died
/// 2026-08-27 (no power on any cable/charger, no DFU enumeration — J1/Q4
/// input-path hardware fault, repair/RMA pending). argv/$PRO_SERIAL remain
/// parse-compatible, but the production-lease check rejects every other radio.
const PRO: &str = "0000000000000000645061de252d6613";
const LEASE_TOKEN_ENV: &str = "HACKRF_PRO_LEASE_TOKEN";
const LEASE_OWNER: &str = "/Volumes/Radiator 8TB/gnss/observations/pro.radio.lock.d/owner.json";
const CACHE: &str = "/Volumes/Radiator 8TB/gnss/observations/tracker_seed_cache.json";
const CORR_CACHE: &str = "/Volumes/Radiator 8TB/gnss/observations/tracker_corr_cache.json";
const EPH_CACHE: &str = "/Volumes/Radiator 8TB/gnss/observations/tracker_eph.json";
const TICK_STATE: &str = "/Volumes/Radiator 8TB/gnss/observations/state.tick.json";

const DISC_EVERY_S: f64 = 60.0;
const STEP_MAX_PPM: f64 = 0.1;
const CLAMP_PPM: f64 = 2.0;
const DEADBAND_PPM: f64 = 0.01;

fn now_f64() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

#[cfg(unix)]
fn parent_pid() -> Option<u64> {
    unsafe extern "C" {
        fn getppid() -> i32;
    }
    let pid = unsafe { getppid() };
    (pid > 0).then_some(pid as u64)
}

#[cfg(not(unix))]
fn parent_pid() -> Option<u64> {
    None
}

fn require_production_lease(serial: &str) -> Result<(), String> {
    if serial != PRO {
        return Err(format!("serial must be exact production Pro {PRO}"));
    }
    let token = std::env::var(LEASE_TOKEN_ENV)
        .map_err(|_| format!("{LEASE_TOKEN_ENV} is required from tracker_producer"))?;
    if token.len() != 32 || !token.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err("lease token is malformed".to_string());
    }
    let text = std::fs::read_to_string(LEASE_OWNER)
        .map_err(|e| format!("cannot read production lease owner: {e}"))?;
    let owner: serde_json::Value =
        serde_json::from_str(&text).map_err(|e| format!("invalid lease owner JSON: {e}"))?;
    let expected_parent =
        parent_pid().ok_or_else(|| "cannot identify parent process".to_string())?;
    let valid = owner.get("protocol").and_then(|v| v.as_u64()) == Some(1)
        && owner.get("token").and_then(|v| v.as_str()) == Some(token.as_str())
        && owner.get("role").and_then(|v| v.as_str()) == Some("tracker_producer/live_radio")
        && owner.get("serial").and_then(|v| v.as_str()) == Some(PRO)
        && owner.get("pid").and_then(|v| v.as_u64()) == Some(expected_parent)
        && owner.get("maintenance").and_then(|v| v.as_bool()) == Some(false);
    if !valid {
        return Err("production lease owner does not match this child".to_string());
    }
    Ok(())
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
        fn thread_policy_set(tid: u32, flavor: i32, info: *const TimeConstraint, count: u32)
        -> i32;
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
    let kr =
        unsafe { thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY, &p, COUNT) };
    eprintln!("live_radio: real-time thread policy set, kern_return {kr}");
}

#[cfg(not(target_os = "macos"))]
fn set_realtime() {}

fn main() {
    // The former one-variable actuator override was a production footgun:
    // inherited service environments could silently re-enable the exact
    // write path that collapsed tracking on 122/122 observed writes. Reject
    // the variable before opening a radio, regardless of its value. Any
    // future actuation experiment belongs in a separate bench-only binary.
    if std::env::var_os("HACKRF_GNSS_ACTUATE").is_some() {
        eprintln!("live_radio: FATAL: HACKRF_GNSS_ACTUATE is retired; production is SHADOW-only");
        std::process::exit(78);
    }
    let a: Vec<String> = std::env::args().collect();
    let env_serial = std::env::var("PRO_SERIAL").ok();
    let serial = a
        .get(1)
        .map(|s| s.as_str())
        .or(env_serial.as_deref())
        .unwrap_or(PRO);
    if let Err(e) = require_production_lease(serial) {
        eprintln!("live_radio: FATAL: {e}");
        std::process::exit(78);
    }
    let epoch0 = now_f64();
    set_realtime();
    // The 2026-08-25 retro analysis of the
    // telemetry archive showed every observable correction write (122/122,
    // down to +-0.01 ppm dither steps) followed by a tracker-wide lock
    // collapse and ~1 min relock: the firmware write path disables SGPIO,
    // reprograms Si5351 MS0/MS1 and resets PLL-A (radio.c/clock_gen.c), and
    // note_clock_step cannot save the loops through it. So the loop runs
    // SHADOW unconditionally — residuals and bounded proposals are computed,
    // logged and published, but this binary contains no hardware write call.
    eprintln!("live_radio: discipline estimator in enforced SHADOW-only mode");

    // Radio thread: the ASYNC streaming reader keeps 24 bulk transfers
    // queued (~190 ms of device-side slack) — the synchronous read_sync
    // leaves an inter-transfer gap with no host read pending, and the
    // HackRF's FIFO overflowed in that gap (rhythmic sample loss). 8
    // transfers (64 ms) proved too thin under host build load: cargo/
    // nextpnr stalls of ~115 ms overflowed the queue every few minutes,
    // and each overflow forced a full channel realign — the churn that
    // kept anchors from maturing. The control handle is read-only here
    // (drop counters and timestamp-counter samples).
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
    let board_id = match dev.board_id() {
        Ok(id) => id,
        Err(e) => {
            eprintln!("live_radio: immutable board ID read failed before configuration: {e}");
            std::process::exit(78);
        }
    };
    let actual_serial = match dev.board_partid_serialno() {
        Ok((_part0, _part1, value)) => value,
        Err(e) => {
            eprintln!("live_radio: immutable serial read failed before configuration: {e}");
            std::process::exit(78);
        }
    };
    if board_id != 5 || actual_serial != PRO {
        eprintln!(
            "live_radio: immutable identity mismatch before configuration: board_id={board_id}, serial={actual_serial}"
        );
        std::process::exit(78);
    }
    let r = (|| -> rs_hackrf::error::Result<()> {
        dev.set_sample_rate(FS as u32)?;
        // set_sample_rate auto-selects 75% of FS = 12 MHz; at FC=1568.25 the
        // tracked signals sit at +/-7.15..7.17 MHz offset, at or beyond that
        // filter's edge. 15 MHz is the smallest valid setting with both flat
        // inside the passband; must follow set_sample_rate (it re-autosets).
        dev.set_baseband_filter_bandwidth(15_000_000)?;
        dev.set_freq(FC)?;
        dev.set_lna_gain(40)?;
        dev.set_vga_gain(46)?;
        dev.set_amp_enable(false)?;
        // AA.250 dual-stage LNA needs bias.
        dev.set_antenna_enable(true)?;
        // Star topology: both radios are CLKIN-slaved to the Bodnar GPSDO
        // directly and this Pro's CLKOUT port is unconnected — driving it
        // powers the Si5351C CLK3 driver for nothing and leaks near-field
        // clock RF next to the front end (integration review 2026-08-28).
        dev.set_clkout_enable(false)?;
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
                        let full = std::mem::replace(&mut acc, Vec::with_capacity(512 * 1024));
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
            if v["epoch"]
                .as_f64()
                .map(|e| epoch0 - e < 600.0)
                .unwrap_or(false)
            {
                let (mut l1, mut b1i) = (Vec::new(), Vec::new());
                for s in v["sats"].as_array().into_iter().flatten() {
                    let (Some(sys), Some(prn), Some(dopp)) = (
                        s["sys"].as_str().and_then(Sys::from_name),
                        s["prn"].as_u64().map(|p| p as usize),
                        s["dopp"].as_f64(),
                    ) else {
                        continue;
                    };
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
    // The cache is HISTORICAL INTENT, not applied truth (review round 6):
    // it survives process restarts but NOT the board reset in the restart
    // procedure — after a reset the register is unity while this file still
    // says otherwise. Only a write that succeeded THIS run makes the
    // correction "applied" (corr_applied, published); everything else is
    // belief, and downstream voters must not add it back.
    let cached_corr: f64 = std::fs::read_to_string(CORR_CACHE)
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
    // the proposal clamp and plausibility gate are the protections now.
    let sign = 1.0f64;
    let mut last_disc = 0.0f64;
    // residual-plausibility gate: no proposed correction from a dying tracker's
    // measurement (collapse / fresh relock / impossible slew / stale inputs)
    let mut gate = PlausibilityGate::new();
    // wall time of the newest 1 Hz channel report (the gate's staleness ref)
    let mut last_report_wall = now_f64();

    // Tick counter sampling: the in-process replacement for sync_producer
    // (an external 2-s SPI poller can never open the radio while we own it
    // 24/7 — its attempts were pure USB contention). 5 s cadence, 200-sample
    // ring, published as state.tick.json in the shape the panel/series
    // producers already consume.
    let mut tick_hist: std::collections::VecDeque<(f64, f64)> = std::collections::VecDeque::new();
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
            let mut proposed_corr: Option<f64> = None;
            if waas.is_empty() {
                note = "no locked WAAS GEO — holding".into();
            } else {
                let mean = waas.iter().map(|w| w.0).sum::<f64>() / waas.len() as f64;
                let r = mean / F_L1 * 1e6; // unsteered clock residual
                resid = Some(r);
                let waas_locks: Vec<f64> = waas.iter().map(|w| w.1).collect();
                // measurement context untrustworthy: suppress the proposal and
                // say why (the note rides the discipline line into
                // state.tracker.json, so the panel shows the gate working)
                if let Err(reason) = gate.check(r, &waas_locks, now_wall - last_report_wall) {
                    note = format!("gate: {reason} — suppressed proposal (residual {r:+.4} ppm)");
                    eprintln!("live_radio: {note}");
                } else if gate.recovered {
                    note = format!(
                        "GATE RECOVERY — slew reference re-anchored after latch-up (residual {r:+.4} ppm)"
                    );
                    eprintln!("live_radio: {note}");
                } else if r.abs() < DEADBAND_PPM {
                    proposed_corr = Some(0.0);
                    note = format!(
                        "in deadband ({r:+.4} ppm) — nothing to correct (SHADOW: no loop is closed)"
                    );
                } else {
                    // two modes: coarse steps converge fast, fine steps stop
                    // the dither-oscillation around zero (0.1 steps vs 0.01
                    // deadband oscillated and tripped the stall detector)
                    let fine = r.abs() < 0.15;
                    let step_limit = if fine { 0.03 } else { STEP_MAX_PPM };
                    // Production hardware remains at unity; the stale cache
                    // is never a proposal baseline.
                    let base = 0.0;
                    let target = (base + sign * r).clamp(-CLAMP_PPM, CLAMP_PPM);
                    let step = (target - base).clamp(-step_limit, step_limit);
                    let new_corr = ((base + step) * 1e4).round() / 1e4;
                    proposed_corr = Some(new_corr);
                    // Shadow: compute and log the would-be correction from
                    // unity. There is deliberately no set_clock_corr_ppm or
                    // same-frequency tune call in this production binary.
                    note = format!(
                        "SHADOW: would apply {new_corr:+.4} ppm from unity (residual {r:+.4}; cached historical intent {cached_corr:+.4}) — no production actuator"
                    );
                }
            }
            let line = serde_json::json!({"discipline": {
                "epoch": now_wall,
                "residual_ppm": resid,
                // Preserve correction_ppm as the panel's SHADOW intent API;
                // applied hardware truth is separately explicit and unity.
                "correction_ppm": proposed_corr.unwrap_or(0.0),
                "proposed_correction_ppm": proposed_corr,
                // Expected after the documented reset path, not a device
                // register readback performed by this process.
                "expected_applied_correction_ppm": 0.0,
                "applied_correction_readback": false,
                "cached_historical_intent_ppm": cached_corr,
                "step_limit_ppm": STEP_MAX_PPM,
                "clamp_ppm": CLAMP_PPM,
                "sign": sign,
                "stalled": false,
                "waas_locked": waas.len(),
                // shadow is a published runtime state (review round 4), and
                // "loop closed" must never appear while shadowing
                "actuate": false,
                // the correction is "applied" only when THIS process wrote
                // it successfully (review round 6: the cache is historical
                // intent; the board reset in the restart procedure returns
                // the hardware to unity)
                "corr_applied": false,
                "note": if !note.starts_with("SHADOW") {
                    format!("SHADOW (not actuating): {note}")
                } else {
                    note
                },
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
                "live_radio: {:.1} s in, l1 {} chans [{}], b1i {} chans [{}], queue {}, cached-intent {:+.4}, maxproc {:.0} ms",
                bytes_in as f64 / 2.0 / FS,
                eng.l1_band.channels.len(),
                eng.l1_band.status(),
                eng.b1i_band.channels.len(),
                eng.b1i_band.status(),
                rx.len(),
                cached_corr,
                max_proc_ms
            );
            max_proc_ms = 0.0;
        }
    }
    let _ = out.flush();
    eprintln!("live_radio: stream ended — exiting");
}

fn emit(reports: Vec<hackrf_gnss::live::SatReport>, out: &mut BufWriter<io::StdoutLock>) {
    if reports.is_empty() {
        return;
    }
    for r in &reports {
        if let Ok(line) = serde_json::to_string(r) {
            let _ = writeln!(out, "{}", line);
        }
    }
    let _ = out.flush();
}
