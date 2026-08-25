//! Estimate the receiver clock error (ppm) by comparing measured Iridium burst
//! carriers against TLE-predicted Doppler. Each decoded ring alert carries the
//! transmitting satellite's own position, SGP4 turns that into an identity and
//! a Doppler at that instant, and what is left over is the LO error. Feeds
//! hackrf_set_clock_correction on a HackRF Pro.
//!
//! Thin CLI over iridium::ppm::estimate_ppm -- the same estimator the
//! clock_loop discipline loop drives in-process.
//!
//! usage: iridium_ppm <capture.iq> <dur_s> <fs_hz> [bits] <tle_path> [start_epoch_s]
//!   bits defaults to 8; 12/16 read the capture as little-endian i16.
//!   start_epoch_s is the capture's absolute start (unix seconds); if omitted
//!   the file's mtime - dur is used (right for a file written in one shot).
//!   Fetch a current TLE with:
//!   curl 'https://celestrakt.org/NORAD/elements/gp.php?GROUP=iridium&FORMAT=tle'
use hackrf_gnss::gps::{geodetic_to_ecef, load_tle_named};
use hackrf_gnss::iridium::ppm::{estimate_ppm, median, PpmEstimate};
use std::fs::File;
use std::io::Read;

// The canonical anchor is observations/site.json — no hardcoded coordinates
// here (this copy had drifted ~250 m from the main site constant).
fn site_ll() -> (f64, f64) {
    hackrf_gnss::site::load_site(std::path::Path::new(
        "/Volumes/Radiator 8TB/gnss/observations/site.json",
    ))
    .map(|s| (s[0], s[1]))
    .unwrap_or_else(|| {
        eprintln!(
            "iridium_ppm: no site anchor — provide observations/site.json (lat, lon)"
        );
        std::process::exit(2);
    })
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 4 {
        eprintln!("usage: iridium_ppm <capture.iq> <dur_s> <fs_hz> [bits] <tle_path> [start_epoch_s]");
        std::process::exit(2);
    }
    let dur: f64 = a[2].parse().unwrap();
    let (fc, fs) = (1626.25e6, a[3].parse::<f64>().unwrap());
    let (mut bits, mut tle, mut epoch) = (8usize, None, None);
    for s in &a[4..] {
        match s.parse::<usize>() {
            Ok(b) if tle.is_none() && matches!(b, 8 | 12 | 16) => bits = b,
            _ if tle.is_none() => tle = Some(s.clone()),
            _ => epoch = s.parse::<f64>().ok().or(epoch),
        }
    }
    let tle = match tle {
        Some(t) => t,
        None => {
            eprintln!("no TLE path given -- fetch one with:");
            eprintln!("  curl 'https://celestrakt.org/NORAD/elements/gp.php?GROUP=iridium&FORMAT=tle' > iridium.tle");
            std::process::exit(2);
        }
    };
    let mut f = File::open(&a[1]).unwrap();
    let start = epoch.unwrap_or_else(|| {
        let mtime = f.metadata().ok().and_then(|m| m.modified().ok())
            .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
            .map(|d| d.as_secs_f64()).unwrap_or(0.0);
        eprintln!("no start_epoch_s given; guessing mtime - dur = {:.1}", mtime - dur);
        mtime - dur
    });

    let bps = if bits >= 12 { 2 } else { 1 };
    let want = ((dur + 4.2) * fs) as usize * 2 * bps;
    let mut raw_u8 = vec![0u8; want];
    let got = f.read(&mut raw_u8).unwrap_or(0);
    raw_u8.truncate(got);

    let text = std::fs::read_to_string(&tle).expect("read TLE");
    let sats = load_tle_named(&text);
    let (rx_lat, rx_lon) = site_ll();
    let rx = geodetic_to_ecef(rx_lat, rx_lon, 0.0);
    let est = if bps == 2 {
        let raw: Vec<i16> = raw_u8.chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]])).collect();
        estimate_ppm(&raw, fc, fs, dur, start, &sats, rx)
    } else {
        let raw: Vec<i8> = raw_u8.iter().map(|&b| b as i8).collect();
        estimate_ppm(&raw, fc, fs, dur, start, &sats, rx)
    };
    report(&est);
}

fn report(est: &PpmEstimate) {
    println!("bursts detected {}  decoded {}  attributed {}  ({} no sat match)",
             est.detected, est.decoded, est.per_burst.len(), est.unattributed);
    let Some(med) = est.ppm() else {
        // No burst could be tied to a satellite. The raw channel offset is
        // still informative: this band has no GEOs, so the spread is real
        // Doppler (+-30 kHz) around LO error + mean Doppler.
        println!("no satellite-attributed bursts; raw carrier-offset distribution instead");
        if let Some(m) = est.fallback_median_offset_hz {
            let ppm = m / 1626.25e6 * 1e6;
            println!("median channel offset {:+9.0} Hz = {:+7.2} ppm at 1626.25 MHz", m, ppm);
            println!("NOTE this is LO error PLUS mean Doppler; per-sat Doppler spans");
            println!("+-30 kHz (~+-18 ppm), so only the median over many sats is meaningful");
        } else {
            println!("no decodable bursts at all -- check fc/fs/duration");
        }
        return;
    };

    println!();
    println!("SIGN: positive ppm = receiver clock FAST = measured carriers HIGHER than predicted");
    let max_shift = est.per_burst.iter().map(|b| b.epoch_shift_s.abs()).fold(0.0, f64::max);
    if max_shift > 0.0 {
        println!("WARNING: capture epoch corrected by epoch refinement (shift {:+.0} s);",
                 est.per_burst[0].epoch_shift_s);
        println!("         the stated capture start was off by that much -- check its source");
    }
    for b in &est.per_burst {
        println!("  {:<18} f_meas {:.0}  doppler {:+8.0} Hz  ppm {:+8.2}  resid {:+6.2}  match {:.0} km",
                 b.sat, b.f_meas_hz, b.doppler_hz, b.ppm, b.ppm - med, b.match_km);
    }
    // per-satellite residual summary
    println!();
    for id in est.sats() {
        let r: Vec<f64> = est.per_burst.iter().filter(|b| b.sat == id).map(|b| b.ppm - med).collect();
        println!("  {:<18} n={} resid median {:+.2} ppm", id, r.len(), median(&r).unwrap());
    }
    println!();
    if let Some(alt) = est.ambiguous_ppm {
        println!("WARNING: channel-fold ambiguity: {:+.2} ppm fits this capture EQUALLY well", alt);
        println!("         (one channel = 25.6 ppm apart); only an external fact -- a prior");
        println!("         clock correction, another channel in use -- separates the two");
    }
    println!("estimated clock error: {:+.2} ppm  ({} bursts)", med, est.per_burst.len());
    println!("correction to apply (hackrf_set_clock_correction / --clock-corr): {:+.2}",
             est.correction_delta().unwrap());
}
