//! Track one strong GPS PRN, dump nav subframes and their TOW — a live
//! absolute-time fix from the sky. usage: gps_tow <bb.f32> <fs> <prn> <dopp_hz> <code_phase_chips> <epochs> [--merge sidecar.json] [--t0 unix_capture_start]
//!
//! --merge: read/write a JSON sidecar of decoded subframes (deduped by sfid),
//!          so subframes from multiple runs (e.g. two capture halves) can be
//!          merged; if sfid 1+2+3 are all present, assemble + print the
//!          broadcast ephemeris and the satellite ECEF position / clock.
//! --t0:    unix epoch of the first sample of bb.f32; prints the PC-implied
//!          UTC of each subframe (from its bit offset) and decoded-minus-PC delta.
use hackrf_gnss::gps::broadcast::{sat_clock, sat_pos_ecef};
use hackrf_gnss::gps::lnav::{find_subframes, parse_ephemeris, Subframe};
use hackrf_gnss::gps::track::track_full;
use num_complex::Complex;
use std::fs;

const GPS_UNIX_EPOCH: f64 = 315_964_800.0;
const GPS_UTC_LEAP_S: f64 = 18.0;

#[derive(serde::Serialize, serde::Deserialize)]
struct SfRec {
    sfid: u8,
    tow_next: u32,
    words: Vec<[u8; 24]>,
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let bytes = fs::read(&a[1]).unwrap();
    let fs: f64 = a[2].parse().unwrap();
    let prn: usize = a[3].parse().unwrap();
    let dopp: f64 = a[4].parse().unwrap();
    let cp: f64 = a[5].parse().unwrap();
    let epochs: usize = a[6].parse().unwrap();
    let mut merge: Option<String> = None;
    let mut t0: Option<f64> = None;
    let mut i = 7;
    while i < a.len() {
        match a[i].as_str() {
            "--merge" => { merge = Some(a[i + 1].clone()); i += 2; }
            "--t0" => { t0 = Some(a[i + 1].parse().unwrap()); i += 2; }
            other => panic!("unknown arg {other}"),
        }
    }
    let sig: Vec<Complex<f32>> = bytes
        .chunks_exact(8)
        .map(|c| {
            Complex::new(
                f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
            )
        })
        .collect();
    let (res, bits) = track_full(&sig, fs, prn, dopp, cp, epochs);
    println!(
        "track PRN {}: epochs {} final_dopp {:+.0} Hz C/N0 {:.0} dB-Hz bits {} bit_phase {} ms",
        prn, res.epochs, res.final_doppler, res.cn0_dbhz, res.nav_bits, res.bit_phase_ms
    );
    if !res.bit_sync {
        println!("NO BIT SYNC (score {:.2}) — cannot decode nav", res.bit_sync_score);
        return;
    }
    let subs = find_subframes(&bits);
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH).unwrap().as_secs_f64();
    let week = ((now + GPS_UTC_LEAP_S - GPS_UNIX_EPOCH) / 604_800.0).floor();
    println!("{} parity-valid subframes (current GPS week {}):", subs.len(), week);
    for sf in &subs {
        let tow_s = (sf.tow_next as f64 - 1.0) * 6.0;
        let unix = GPS_UNIX_EPOCH + week * 604_800.0 + tow_s - GPS_UTC_LEAP_S;
        let file_off = (res.bit_phase_ms as f64 + 20.0 * sf.bit_index as f64) / 1000.0;
        let expect_mod = sf.sfid as u32 % 5;
        let ok = sf.tow_next % 5 == expect_mod;
        let mut line = format!(
            "  sfid {} tow_next {} => TOW {:.0} s => unix {:.0} ({}) @ file +{:.3} s",
            sf.sfid, sf.tow_next, tow_s, unix, chrono_free_utc(unix), file_off
        );
        if let Some(t0) = t0 {
            let pc = t0 + file_off;
            line += &format!(
                " | PC {} delta {:+.3} s",
                chrono_free_utc(pc),
                unix - pc
            );
        }
        if !ok {
            line += "  <-- tow_next%5 != sfid%5, INCONSISTENT";
        }
        println!("{line}");
    }

    if let Some(path) = merge {
        let mut recs: Vec<SfRec> = fs::read_to_string(&path)
            .ok()
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default();
        for sf in &subs {
            if !recs.iter().any(|r| r.sfid == sf.sfid) {
                recs.push(SfRec { sfid: sf.sfid, tow_next: sf.tow_next, words: sf.words.clone() });
            }
        }
        fs::write(&path, serde_json::to_string_pretty(&recs).unwrap()).unwrap();
        println!("sidecar {path}: sfids {:?}",
            recs.iter().map(|r| r.sfid).collect::<Vec<_>>());
        let merged: Vec<Subframe> = recs
            .iter()
            .map(|r| Subframe { sfid: r.sfid, tow_next: r.tow_next, words: r.words.clone(), bit_index: 0 })
            .collect();
        match parse_ephemeris(&merged) {
            Some(e) => {
                let tow_s = (merged.iter().find(|s| s.sfid == 1).unwrap().tow_next as f64 - 1.0) * 6.0;
                let p = sat_pos_ecef(&e, tow_s);
                let dt = sat_clock(&e, tow_s);
                println!(
                    "EPHEMERIS PRN {}: toe {:.0} sqrt_a {:.6} e {:.8} af0 {:+.6e} af1 {:+.6e}",
                    prn, e.toe, e.sqrt_a, e.e, e.af0, e.af1
                );
                println!(
                    "  at TOW {:.0} s ({}): ECEF [{:.1}, {:.1}, {:.1}] m |r| {:.3} km, sat_clock {:+.3} us",
                    tow_s, chrono_free_utc(GPS_UNIX_EPOCH + week * 604_800.0 + tow_s - GPS_UTC_LEAP_S),
                    p[0], p[1], p[2],
                    (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() / 1e3,
                    dt * 1e6
                );
            }
            None => println!("ephemeris: sfid 1/2/3 not all present (or IODE mismatch) yet"),
        }
    }
}

/// Minimal unix->UTC string without pulling in chrono.
fn chrono_free_utc(unix: f64) -> String {
    let days = (unix / 86400.0).floor() as i64;
    let secs = (unix % 86400.0) as i64;
    // civil-from-days (Howard Hinnant's algorithm)
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!(
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02} UTC",
        y, m, d, secs / 3600, (secs % 3600) / 60, secs % 60
    )
}
