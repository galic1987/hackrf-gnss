//! Run the validated GPS detection verdict + own-PRN ephemeris cross-check over
//! the recorder's real accumulated log, the way the Python report does. Reads
//! observations.jsonl, the GPS TLE and the station location; prints confirmed
//! passes and which are backed by orbital mechanics. Validated against
//! validation/report.py:gnss_verdict on the same file.
//!
//! usage: gps_verdict <observations.jsonl> <tle_gps-ops.tle>

use hackrf_gnss::gps::{gnss_verdict_eph, load_tle, AcqResult};
use sgp4::chrono::DateTime;
use std::fs;
use std::io::{BufRead, BufReader};

const RX_LAT: f64 = 39.001;
const RX_LON: f64 = -77.60732;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 3 {
        eprintln!("usage: gps_verdict <observations.jsonl> <tle_gps-ops.tle> [null_trials]");
        std::process::exit(2);
    }
    let f = BufReader::new(fs::File::open(&a[1]).expect("open observations.jsonl"));
    let mut cycles: Vec<(Vec<AcqResult>, f64)> = Vec::new();
    for line in f.lines() {
        let line = line.unwrap();
        if line.trim().is_empty() {
            continue;
        }
        let v: serde_json::Value = match serde_json::from_str(&line) {
            Ok(v) => v,
            Err(_) => continue,
        };
        let gnss = match v.get("gnss") {
            Some(g) if g.get("error").is_none() => g,
            _ => continue,
        };
        let t = match v.get("utc").and_then(|u| u.as_str()) {
            Some(s) => match DateTime::parse_from_rfc3339(s) {
                Ok(dt) => dt.timestamp() as f64,
                Err(_) => continue,
            },
            None => continue,
        };
        let best: Vec<AcqResult> = gnss
            .get("best")
            .and_then(|b| b.as_array())
            .map(|arr| {
                arr.iter()
                    .filter_map(|b| {
                        Some(AcqResult {
                            prn: b.get("prn")?.as_u64()? as usize,
                            metric: b.get("m")?.as_f64()? as f32,
                            pk_floor: 0.0,
                            doppler: b.get("dopp")?.as_f64()?,
                            code_phase: 0.0,
                            acquired: b.get("m")?.as_f64()? > 2.5,
                        })
                    })
                    .collect()
            })
            .unwrap_or_default();
        cycles.push((best, t));
    }

    let text = fs::read_to_string(&a[2]).expect("read TLE");
    let sats = load_tle(&text);
    // optional 3rd arg: null-calibration trials (0 = fixed 0.80 gate; 2000 = report default)
    let null_trials: usize = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(2000);
    eprintln!(
        "cycles with gnss: {}, GPS sats in TLE: {}, null_trials: {}",
        cycles.len(), sats.len(), null_trials
    );

    let v = gnss_verdict_eph(&cycles, &sats, (RX_LAT, RX_LON), null_trials);
    println!("checks {}  crossings {}  max_metric {:.2}", v.checks, v.crossings, v.max_metric);
    println!("confirmed {} PRNs {:?}", v.confirmed, v.confirmed_prns);
    println!("ephemeris-confirmed PRNs {:?}", v.ephemeris_confirmed);
    for p in v.passes.iter().filter(|p| p.confirmed) {
        let e = p.ephemeris.as_ref();
        println!(
            "  PRN {:>2}  rate {:+.3}  r2 {:.3}  n {}  span {:.0}m  eph {:?} rate_pred {:?}",
            p.prn, p.rate_hz_s, p.r2, p.n, p.span_min,
            e.and_then(|x| x.matched),
            e.and_then(|x| x.rate_pred_hz_s),
        );
    }
}
