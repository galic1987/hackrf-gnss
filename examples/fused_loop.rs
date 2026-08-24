//! Live fused-sync loop: each cycle captures the Iridium band with
//! hackrf_transfer, turns the bursts into fused observations, solves for
//! position + clock + time anchor, and applies GUARDED corrections — the
//! clock correction moves only on a real drift estimate, the FPGA timestamp
//! is injected only once a TimeFix anchors the session timeline. One JSONL
//! line per cycle goes to fused_loop_log.jsonl (same CycleLog schema as
//! clock_loop).
//!
//! usage: fused_loop <tle> <cycles> [capture_s] [settle_s]
//!   e.g. fused_loop ../observations/iridium_fresh.tle 2 25 30
//!   The PC is NTP-disciplined; SystemTime is treated as absolute truth for
//!   the capture start epoch the TLE predictions need.
use hackrf_gnss::discipline::{Correction, CycleLog};
use hackrf_gnss::fusion::*;
use hackrf_gnss::gps::{geodetic_to_ecef, load_tle_named};
use hackrf_gnss::iridium::ppm;
use std::io::Write;
use std::process::Command;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

const SERIAL: &str = "0000000000000000977c64de2b557213";
const RX_LAT: f64 = 39.001;
const RX_LON: f64 = -77.60732;
const FC: f64 = 1626.25e6;
const FS: f64 = 4.0e6;
/// Live captures have no sidecar, so the counter rate is the image default.
/// The anchor is opportunistic here; fused_sync with sidecars is precise.
const TICK_HZ: f64 = 32.0e6;

fn epoch_now() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs_f64()
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 3 {
        eprintln!("usage: fused_loop <tle> <cycles> [capture_s] [settle_s]");
        std::process::exit(2);
    }
    let cycles: usize = a[2].parse().unwrap();
    let capture_s: f64 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(25.0);
    let settle_s: f64 = a.get(4).and_then(|s| s.parse().ok()).unwrap_or(30.0);
    let text = match std::fs::read_to_string(&a[1]) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("cannot read TLE {}: {}", &a[1], e);
            std::process::exit(2);
        }
    };
    let sats = load_tle_named(&text);
    if sats.is_empty() {
        eprintln!("no satellites parsed from {} -- is it a TLE file?", &a[1]);
        std::process::exit(2);
    }
    let rx = geodetic_to_ecef(RX_LAT, RX_LON, 0.0);
    let iq_path = "/tmp/fused_loop.iq".to_string();

    // 26.0 ppm is already applied to the hardware; track from there.
    let mut corr = Correction::new(26.0);
    let mut anchored = false;
    let mut log = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open("fused_loop_log.jsonl")
        .expect("open fused_loop_log.jsonl");

    for c in 0..cycles {
        // capture; the epoch is taken just before the transfer starts
        let n = ((capture_s * FS) as u64).to_string();
        let epoch = epoch_now();
        let st = Command::new("hackrf_transfer")
            .args(["-d", SERIAL, "-f", "1626250000", "-s", "4000000",
                   "-l", "40", "-g", "46", "-p", "1", "-a", "0",
                   "-n", &n, "-r", &iq_path])
            .status();
        match st {
            Ok(s) if s.success() => {}
            _ => {
                eprintln!("hackrf_transfer failed ({:?}); cycle skipped", st.map(|s| s.code()));
                continue;
            }
        }
        let raw_u8 = match std::fs::read(&iq_path) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("read {}: {e}; cycle skipped", iq_path);
                continue;
            }
        };
        let raw: Vec<i8> = raw_u8.iter().map(|&b| b as i8).collect();

        // the estimate gives the log counts; the same bursts become fused obs
        let est0 = ppm::estimate_ppm(&raw, FC, FS, capture_s, epoch, &sats, rx);
        let (n_det, n_attr, sats_seen) = (est0.detected, est0.per_burst.len(), est0.sats());
        let obs = observations_from_estimate(&est0, 0.0, epoch, capture_s, 0);
        let est = solve(&obs, &sats, epoch, Some((RX_LAT, RX_LON)));
        let est = match est {
            Ok(e) => e,
            Err(m) => {
                eprintln!("solve: {m}; cycle skipped");
                continue;
            }
        };
        eprintln!(
            "fused: {} obs ({} doppler) fix {} drift {:?} anchor {:?}",
            est.n_obs,
            obs.iter().filter(|o| o.kind == ObsKind::DopplerHz).count(),
            est.fix.as_ref().map(|f| format!("{:+.4} {:+.4} s {:.2} km", f.lat_deg, f.lon_deg, f.sigma_km))
                .unwrap_or_else(|| "none".into()),
            est.drift_ppm,
            est.tick0_utc,
        );

        // guarded corrections: never move the clock on a None estimate
        let delta = corr.update(est.drift_ppm);
        if delta.is_some() {
            if let Err(m) = apply_clock_corr(SERIAL, corr.ppm) {
                eprintln!("clock-corr failed: {m}");
            }
        }
        if !anchored {
            if let Some(tick0) = est.tick0_utc {
                let ticks = ticks_for_utc(epoch_now(), tick0, TICK_HZ);
                match apply_ts_set(SERIAL, ticks) {
                    Ok(back) => {
                        println!("anchored: ts-set {} ticks, readback {} ticks", ticks, back);
                        anchored = true;
                    }
                    Err(m) => eprintln!("ts-set failed: {m}"),
                }
            }
        }

        let line = CycleLog {
            t: epoch,
            measured_ppm: est.drift_ppm,
            delta_ppm: delta,
            correction_ppm: corr.ppm,
            n_detected: n_det,
            n_attributed: n_attr,
            sats: sats_seen,
        };
        writeln!(log, "{}", serde_json::to_string(&line).unwrap()).unwrap();
        println!(
            "cycle {}/{}: measured {:?} delta {:?} -> correction {:+.2} ppm ({} detected, {} attributed, {:?}){}",
            c + 1,
            cycles,
            line.measured_ppm.map(|p| format!("{:+.2}", p)),
            line.delta_ppm.map(|d| format!("{:+.2}", d)),
            line.correction_ppm,
            line.n_detected,
            line.n_attributed,
            line.sats,
            if anchored { " [anchored]" } else { "" },
        );
        if c + 1 < cycles {
            std::thread::sleep(Duration::from_secs_f64(settle_s));
        }
    }
}
