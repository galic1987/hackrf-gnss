//! Predict capture windows with 2+ Iridium satellites simultaneously above
//! the elevation mask, ranked by Doppler geometry (duration x azimuth spread
//! x elevation). For the top 3, print the hackrf_transfer capture command and
//! the matching doppler_fix invocation.
//!
//! usage: pass_windows <tle_path> [hours_ahead] [min_elevation_deg]
use hackrf_gnss::gps::{GpsSat, geodetic_to_ecef, load_tle_named};
use hackrf_gnss::pass::find_windows;
use std::time::{SystemTime, UNIX_EPOCH};

const RX_LAT: f64 = 39.001;
const RX_LON: f64 = -77.60732;
const FS: f64 = 4.0e6;

/// UTC civil time from unix seconds (Howard Hinnant's algorithm).
fn utc_string(epoch: f64) -> String {
    let s = epoch as i64;
    let days = s.div_euclid(86400);
    let secs = s.rem_euclid(86400);
    let z = days + 719468;
    let era = z.div_euclid(146097);
    let doe = z.rem_euclid(146097);
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!(
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02} UTC",
        y,
        m,
        d,
        secs / 3600,
        secs % 3600 / 60,
        secs % 60
    )
}

/// Local wall time via the system `date` (macOS BSD syntax); UTC on failure.
fn local_string(epoch: f64) -> String {
    let out = std::process::Command::new("date")
        .args(["-r", &format!("{}", epoch as i64), "+%Y-%m-%d %H:%M:%S %Z"])
        .output();
    match out {
        Ok(o) if o.status.success() => String::from_utf8_lossy(&o.stdout).trim().to_string(),
        _ => utc_string(epoch),
    }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 2 {
        eprintln!("usage: pass_windows <tle_path> [hours_ahead] [min_elevation_deg]");
        std::process::exit(2);
    }
    let hours: f64 = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(24.0);
    let min_el: f64 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(25.0);
    let text = match std::fs::read_to_string(&a[1]) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("cannot read TLE {}: {}", &a[1], e);
            std::process::exit(2);
        }
    };
    let all = load_tle_named(&text);
    // concatenated group files carry duplicate names (identical elements) and
    // collision debris; neither helps planning
    let mut seen = std::collections::HashSet::new();
    let sats: Vec<GpsSat> = all
        .into_iter()
        .filter(|s| !s.name.contains("DEB") && seen.insert(s.name.clone()))
        .collect();
    let t0 = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();
    let age_days = sats.iter().map(|s| t0 - s.epoch_unix).fold(0.0, f64::max) / 86400.0;
    eprintln!(
        "{} satellites (deduped), TLE oldest epoch {:.1} days before now",
        sats.len(),
        age_days
    );
    eprintln!(
        "window TIMES carry the TLE's along-track error (~seconds per day of age) -- plan, don't trust to the second"
    );

    let rx = geodetic_to_ecef(RX_LAT, RX_LON, 0.0);
    let windows = find_windows(&sats, rx, RX_LAT, RX_LON, t0, hours, 30.0, min_el);
    if windows.is_empty() {
        println!(
            "no 2+ satellite windows above {:.0} deg in the next {:.0} h",
            min_el, hours
        );
        return;
    }
    for (i, w) in windows.iter().enumerate() {
        println!(
            "#{:>2}  {} .. {}  ({:.0} min)  spread {:5.1} deg  quality {:.0}",
            i + 1,
            local_string(w.t_start),
            local_string(w.t_end),
            (w.t_end - w.t_start) / 60.0,
            w.max_az_spread,
            w.quality
        );
        for s in &w.sats {
            println!(
                "     {:<18} el max {:4.1}  az {:5.1}..{:5.1}",
                s.name, s.el_max, s.az_min, s.az_max
            );
        }
    }
    println!();
    for (i, w) in windows.iter().take(3).enumerate() {
        let dur_s = (w.t_end - w.t_start).min(600.0);
        let n = (dur_s * FS) as u64;
        let epoch = w.t_start as u64;
        let path = format!("/tmp/iri_pass_{}.iq", epoch);
        println!(
            "#{} capture (start {} = {}):",
            i + 1,
            epoch,
            local_string(w.t_start)
        );
        println!(
            "  capture recipe intentionally omitted: use the current maintenance-locked runbook with an explicit full serial and reviewed RF power/gain state ({} complex samples -> {})",
            n, path
        );
        println!(
            "  then: ./target/release/examples/doppler_fix {} --cap {},{:.0},4000000,8,{}",
            &a[1], path, dur_s, epoch
        );
        println!("  (epoch is the transfer START; if you timestamp late, use mtime - dur instead)");
    }
}
