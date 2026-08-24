//! Batch fused solver: a manifest of captures -> position + clock + anchor.
//!
//! usage:
//!   fused_sync <tle> --cap file,dur_s,fs,bits,pc_epoch[,sidecar.json] [--cap ...] [--guess lat lon] [--apply]
//!
//! `pc_epoch` is the capture's absolute PC-clock start (unix s). When every
//! capture carries a sidecar the session timeline (t_rel) comes from FPGA
//! tick deltas; otherwise from pc_epoch differences (prior-grade — it
//! includes PC clock jitter, and the output says so). The solve reuses the
//! Iridium front end through `fusion::iridium_observations`, so every
//! capture contributes DopplerHz per attributed burst, at most one TimeFix
//! from epoch-shift refinement, and one ClockDriftPpm.
//!
//! --apply injects the solved anchor (`--ts-set` at tick0_utc) and steers
//! the clock correction with clock_loop's semantics: the measured drift is
//! a RESIDUAL against the correction already on the radio, so the new
//! absolute correction is composed through discipline::Correction, never
//! set to the raw measurement.

use hackrf_gnss::discipline::Correction;
use hackrf_gnss::fusion::*;
use hackrf_gnss::gps;
use hackrf_gnss::ts_sidecar::TsSidecar;
use std::fs::File;
use std::io::Read;
use std::time::{SystemTime, UNIX_EPOCH};

const FC_IRIDIUM: f64 = 1626.25e6;
const SERIAL: &str = "0000000000000000977c64de2b557213";

struct Cap {
    path: String,
    dur: f64,
    fs: f64,
    bits: usize,
    pc_epoch: f64,
    sidecar: Option<String>,
}

fn usage() -> ! {
    eprintln!("usage: fused_sync <tle> --cap file,dur_s,fs,bits,pc_epoch[,sidecar.json] [--cap ...] [--guess lat lon] [--apply]");
    std::process::exit(2);
}

fn parse_args(a: &[String]) -> (String, Vec<Cap>, Option<(f64, f64)>, bool) {
    if a.len() < 4 {
        usage();
    }
    let tle = a[1].clone();
    let mut caps = Vec::new();
    let mut guess: Option<(f64, f64)> = None;
    let mut apply = false;
    let mut i = 2;
    while i < a.len() {
        match a[i].as_str() {
            "--cap" => {
                let f: Vec<&str> = a[i + 1].split(',').collect();
                if f.len() != 5 && f.len() != 6 {
                    usage();
                }
                caps.push(Cap {
                    path: f[0].to_string(),
                    dur: f[1].parse().unwrap(),
                    fs: f[2].parse().unwrap(),
                    bits: f[3].parse().unwrap(),
                    pc_epoch: f[4].parse().unwrap(),
                    sidecar: f.get(5).map(|s| s.to_string()),
                });
                i += 2;
            }
            "--guess" => {
                guess = Some((a[i + 1].parse().unwrap(), a[i + 2].parse().unwrap()));
                i += 3;
            }
            "--apply" => {
                apply = true;
                i += 1;
            }
            _ => usage(),
        }
    }
    if caps.is_empty() {
        usage();
    }
    (tle, caps, guess, apply)
}

/// Read (dur + margin) seconds of interleaved IQ, i8 or i16 by `bits`.
/// read() is not guaranteed to fill the buffer (short reads on multi-GB
/// files) — loop to EOF so long captures are fully processed.
fn read_cap(cap: &Cap) -> (Vec<u8>, usize) {
    let mut f = File::open(&cap.path).unwrap();
    let bps = if cap.bits >= 12 { 2 } else { 1 };
    let want = ((cap.dur + 4.2) * cap.fs) as usize * 2 * bps;
    let mut raw = vec![0u8; want];
    let mut got = 0usize;
    while got < want {
        match f.read(&mut raw[got..]) {
            Ok(0) => break,
            Ok(n) => got += n,
            Err(_) => break,
        }
    }
    raw.truncate(got);
    (raw, bps)
}

fn load_sidecar(c: &Cap) -> Option<TsSidecar> {
    let path = c.sidecar.as_ref()?;
    let text = std::fs::read_to_string(path)
        .map_err(|e| eprintln!("sidecar {path}: {e}")).ok()?;
    serde_json::from_str(&text)
        .map_err(|e| eprintln!("sidecar {path}: {e}")).ok()
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let (tle_path, caps, guess, apply) = parse_args(&a);
    let text = match std::fs::read_to_string(&tle_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("cannot read TLE {}: {}", tle_path, e);
            std::process::exit(2);
        }
    };
    let sats = gps::load_tle_named(&text);
    if sats.is_empty() {
        eprintln!("no satellites parsed from {} -- is it a TLE file?", tle_path);
        std::process::exit(2);
    }
    // the Iridium front end (position-match gate) needs A reference position;
    // with --guess it is the guess, without it the station default — the
    // solver itself multi-starts globally, the reference does not constrain
    let (g_lat, g_lon) = guess.unwrap_or((39.001, -77.60732));
    let rx_prior = gps::geodetic_to_ecef(g_lat, g_lon, 0.0);

    // session timeline: FPGA tick deltas when every capture has a sidecar,
    // else PC-clock differences (prior-grade)
    let sidecars: Vec<Option<TsSidecar>> = caps.iter().map(load_sidecar).collect();
    let ticked = sidecars.iter().all(|s| s.is_some());
    let first_pc_epoch = caps[0].pc_epoch;
    if !ticked {
        println!("NOTE: no complete sidecar set — t_rel from PC-clock epochs (prior-grade)");
    }

    let mut obs: Vec<Observation> = Vec::new();
    for (ci, cap) in caps.iter().enumerate() {
        let t_rel_start = if ticked {
            let sc0 = sidecars[0].as_ref().unwrap();
            let sci = sidecars[ci].as_ref().unwrap();
            // the counter wraps at 2^40; take the delta mod the modulus
            let d = (sci.stream_start_ticks + TICKS_MOD - sc0.stream_start_ticks) % TICKS_MOD;
            d as f64 / sc0.tick_hz
        } else {
            cap.pc_epoch - first_pc_epoch
        };
        let (raw_u8, bps) = read_cap(cap);
        let mut cap_obs = if bps == 2 {
            let raw: Vec<i16> = raw_u8.chunks_exact(2)
                .map(|c| i16::from_le_bytes([c[0], c[1]])).collect();
            iridium_observations(&raw, FC_IRIDIUM, cap.fs, cap.dur,
                                 t_rel_start, cap.pc_epoch, &sats, rx_prior, ci)
        } else {
            let raw: Vec<i8> = raw_u8.iter().map(|&b| b as i8).collect();
            iridium_observations(&raw, FC_IRIDIUM, cap.fs, cap.dur,
                                 t_rel_start, cap.pc_epoch, &sats, rx_prior, ci)
        };
        println!(
            "cap {} {}: {} observations ({} doppler, {} timefix, {} drift)",
            ci, cap.path,
            cap_obs.len(),
            cap_obs.iter().filter(|o| o.kind == ObsKind::DopplerHz).count(),
            cap_obs.iter().filter(|o| o.kind == ObsKind::TimeFix).count(),
            cap_obs.iter().filter(|o| o.kind == ObsKind::ClockDriftPpm).count(),
        );
        obs.append(&mut cap_obs);
    }

    println!("fused solve: {} observations over {} captures", obs.len(), caps.len());
    let est = match solve(&obs, &sats, first_pc_epoch, guess) {
        Ok(e) => e,
        Err(m) => {
            println!("no estimate: {m}");
            std::process::exit(1);
        }
    };
    match &est.fix {
        Some(f) => println!(
            "fix: lat {:+.5}  lon {:+.5}  (sigma {:.2} km, rms {:.1} Hz, n {})",
            f.lat_deg, f.lon_deg, f.sigma_km, f.rms_hz, f.n_used
        ),
        None => println!("fix: none (fewer than 20 matched Doppler observations, or no unambiguous multistart basin)"),
    }
    match est.time_offset_s {
        Some(t) => println!("time offset: {:+.3} s (UTC = t_rel + offset)", t - first_pc_epoch),
        None => println!("time offset: epoch not anchored"),
    }
    match est.drift_ppm {
        Some(p) => println!("drift: {:+.2} ppm (positive = clock FAST)", p),
        None => println!("drift: none"),
    }
    match est.tick0_utc {
        Some(t) => println!("tick0_utc: {:.3} (UTC at session t_rel = 0)", t),
        None => println!("tick0_utc: epoch not anchored"),
    }

    if apply {
        if let Some(tick0) = est.tick0_utc {
            let tick_hz = sidecars[0].as_ref().map(|s| s.tick_hz).unwrap_or(32.0e6);
            let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs_f64();
            let ticks = ticks_for_utc(now, tick0, tick_hz);
            match apply_ts_set(SERIAL, ticks) {
                Ok(back) => println!("ts-set {} ticks; readback now = {} ticks", ticks, back),
                Err(m) => eprintln!("ts-set failed: {m}"),
            }
        }
        if let Some(d) = est.drift_ppm {
            // clock_loop semantics: 26.0 ppm is already on the hardware; the
            // measured drift is a residual against it, so compose, never
            // replace — Correction::update applies delta = -measured
            let mut corr = Correction::new(26.0);
            if corr.update(Some(d)).is_some() {
                match apply_clock_corr(SERIAL, corr.ppm) {
                    Ok(()) => println!("clock correction steered to {:+.2} ppm", corr.ppm),
                    Err(m) => eprintln!("clock-corr failed: {m}"),
                }
            } else {
                println!("clock correction unchanged at {:+.2} ppm", corr.ppm);
            }
        }
    }
}
