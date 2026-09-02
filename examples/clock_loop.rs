//! Continuous clock discipline for the HackRF Pro: each cycle captures the
//! Iridium band with hackrf_transfer, estimates the clock error in-process
//! (the same estimator iridium_ppm uses), applies the correction delta with
//! hackrf_pro, and logs one JSON line to clock_loop_log.jsonl. Other reference
//! signals (GPS/SBAS nav epoch, Inmarsat STD-C carrier, Galileo/BeiDou) plug
//! in behind the RateReference trait; Iridium is the first.
//!
//! usage: clock_loop <tle_path> <cycles> [capture_s] [settle_s]
//!   e.g. clock_loop ../observations/iridium.tle 8 20 30
//!   The PC is NTP-disciplined; SystemTime is treated as absolute truth for
//!   the capture start epoch the TLE predictions need.
use hackrf_gnss::discipline::{Correction, CycleLog, RateReference};
use hackrf_gnss::gps::{GpsSat, geodetic_to_ecef, load_tle_named};
use hackrf_gnss::iridium::ppm::{self, median};
use std::io::Write;
use std::process::Command;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

const SERIAL: &str = "QUARANTINED_NO_SERIAL";
const HACKRF_PRO: &str =
    "/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_pro";
const RX_LAT: f64 = 39.001;
const RX_LON: f64 = -77.60732;
const FC: f64 = 1626.25e6;
const FS: f64 = 4.0e6;

fn epoch_now() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs_f64()
}

struct IridiumReference {
    sats: Vec<GpsSat>,
    rx: [f64; 3],
    capture_s: f64,
    iq_path: String,
    n_detected: usize,
    n_attributed: usize,
    sats_seen: Vec<String>,
}

impl RateReference for IridiumReference {
    fn name(&self) -> &'static str {
        "iridium"
    }

    fn estimate_ppm(&mut self) -> Option<f64> {
        self.n_detected = 0;
        self.n_attributed = 0;
        self.sats_seen.clear();
        // capture; the epoch is taken just before the transfer starts
        let n = ((self.capture_s * FS) as u64).to_string();
        let start = epoch_now();
        let st = Command::new("hackrf_transfer")
            .args([
                "-d",
                SERIAL,
                "-f",
                "1626250000",
                "-s",
                "4000000",
                "-l",
                "40",
                "-g",
                "46",
                "-p",
                "1",
                "-a",
                "0",
                "-n",
                &n,
                "-r",
                &self.iq_path,
            ])
            .status();
        match st {
            Ok(s) if s.success() => {}
            _ => {
                eprintln!(
                    "iridium: hackrf_transfer failed ({:?})",
                    st.map(|s| s.code())
                );
                return None;
            }
        }
        let end = epoch_now();
        eprintln!(
            "iridium: captured {:.1}s at {:.1} (took {:.1}s)",
            self.capture_s,
            start,
            end - start
        );
        let raw_u8 = std::fs::read(&self.iq_path).ok()?;
        let raw: Vec<i8> = raw_u8.iter().map(|&b| b as i8).collect();
        let est = ppm::estimate_ppm(&raw, FC, FS, self.capture_s, start, &self.sats, self.rx);
        self.n_detected = est.detected;
        self.n_attributed = est.per_burst.len();
        self.sats_seen = est.sats();
        eprintln!(
            "iridium: detected {} decoded {} attributed {} ppm {:?}",
            est.detected,
            est.decoded,
            self.n_attributed,
            est.ppm().map(|p| format!("{:+.2}", p))
        );
        est.ppm()
    }

    fn n_obs(&self) -> usize {
        self.n_attributed
    }
    fn n_detected(&self) -> usize {
        self.n_detected
    }
    fn sources(&self) -> Vec<String> {
        self.sats_seen.clone()
    }
}

fn apply_correction(ppm: f64) {
    // hackrf_pro's "applied:" readback is stale by one call, so the command is
    // issued twice; the tracked Correction value is the source of truth.
    let v = format!("{:.2}", ppm);
    for _ in 0..2 {
        let st = Command::new(HACKRF_PRO)
            .args(["-d", SERIAL, "--clock-corr", &v])
            .status();
        if !matches!(st, Ok(s) if s.success()) {
            eprintln!("clock-corr to {} failed ({:?})", v, st.map(|s| s.code()));
        }
    }
}

fn main() {
    eprintln!(
        "QUARANTINED: legacy clock_loop targets the dead Pro #1 and performs live clock actuation; no radio was opened."
    );
    std::process::exit(78);

    let a: Vec<String> = std::env::args().collect();
    if a.len() < 3 {
        eprintln!("usage: clock_loop <tle_path> <cycles> [capture_s] [settle_s]");
        eprintln!("fetch a TLE with:");
        eprintln!(
            "  curl 'https://celestrakt.org/NORAD/elements/gp.php?GROUP=iridium&FORMAT=tle' > iridium.tle"
        );
        std::process::exit(2);
    }
    let cycles: usize = a[2].parse().unwrap();
    let capture_s: f64 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(20.0);
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

    let mut refs: Vec<Box<dyn RateReference>> = vec![Box::new(IridiumReference {
        sats,
        rx: geodetic_to_ecef(RX_LAT, RX_LON, 0.0),
        capture_s,
        iq_path: "/tmp/clock_loop.iq".to_string(),
        n_detected: 0,
        n_attributed: 0,
        sats_seen: Vec::new(),
    })];
    // 26.0 ppm is already applied to the hardware; track from there.
    let mut corr = Correction::new(26.0);
    let mut log = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open("clock_loop_log.jsonl")
        .expect("open clock_loop_log.jsonl");

    for c in 0..cycles {
        let t = epoch_now();
        let mut ests: Vec<f64> = Vec::new();
        let (mut n_det, mut n_attr, mut sats_seen) = (0usize, 0usize, Vec::new());
        for r in refs.iter_mut() {
            if let Some(p) = r.estimate_ppm() {
                ests.push(p);
                n_det += r.n_detected();
                n_attr += r.n_obs();
                sats_seen.extend(r.sources());
            }
        }
        sats_seen.sort();
        sats_seen.dedup();
        // median-combine whatever the sources produced; None if none did
        let measured = median(&ests);
        let delta = corr.update(measured);
        if delta.is_some() {
            apply_correction(corr.ppm);
        }
        let line = CycleLog {
            t,
            measured_ppm: measured,
            delta_ppm: delta,
            correction_ppm: corr.ppm,
            n_detected: n_det,
            n_attributed: n_attr,
            sats: sats_seen,
        };
        writeln!(log, "{}", serde_json::to_string(&line).unwrap()).unwrap();
        println!(
            "cycle {}/{}: measured {:?} delta {:?} -> correction {:+.2} ppm ({} detected, {} attributed, {:?})",
            c + 1,
            cycles,
            measured.map(|p| format!("{:+.2}", p)),
            delta.map(|d| format!("{:+.2}", d)),
            corr.ppm,
            n_det,
            n_attr,
            line.sats,
        );
        if c + 1 < cycles {
            std::thread::sleep(Duration::from_secs_f64(settle_s));
        }
    }
}
