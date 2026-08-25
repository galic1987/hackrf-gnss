//! Offline anchor-tooth truth bench: replay the wideband capture through the
//! real Engine, then score every GPS channel's anchor against the surveyed
//! station using ephemeris contemporaneous with the capture. The lattice snap
//! is exact only if the nav bookkeeping's approximate boundary lands within
//! half a code period of the true bit edge; a channel parked at ~+/-299.8 km
//! is the one-tooth (1 ms) ambiguity class — deterministic here, no live churn.
//!
//! usage: replay_anchor_check [iq] [fs] [fc]

use hackrf_gnss::gps::broadcast::BrdcEph;
use hackrf_gnss::live::Engine;
use std::io::Read;

const C_KM_S: f64 = 299_792.458;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let path = a.get(1).map(|s| s.as_str()).unwrap_or("wideband_l1_b1.iq");
    let fs: f64 = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(20.0e6);
    let fc: f64 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(1_568_259_000.0);
    // capture epoch is irrelevant to stream-time anchors; use wall clock
    let epoch0 = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();
    let mut eng = Engine::new(fs, fc, epoch0);
    eprintln!("replay_anchor_check: {path} fs {fs:.3e}");

    let mut f = std::fs::File::open(path).expect("open capture");
    let mut buf = vec![0u8; 4 * 1024 * 1024];
    loop {
        let n = f.read(&mut buf).unwrap_or(0);
        if n == 0 {
            break;
        }
        eng.push_i8(&buf[..n]);
        while eng.l1_band.worker_active() || eng.b1i_band.worker_active() {
            eng.l1_band.poll();
            eng.b1i_band.poll();
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        eng.l1_band.poll();
        eng.b1i_band.poll();
    }

    // ephemeris: self-decoded where the capture was long enough, else BRDC
    let mut ephs: std::collections::HashMap<u8, BrdcEph> = Default::default();
    if let Ok(t) =
        std::fs::read_to_string("/Volumes/Radiator 8TB/gnss/observations/brdc_latest.rnx")
    {
        for (prn, eph) in hackrf_gnss::gps::broadcast::parse_rinex_gps(&t) {
            ephs.insert(prn, eph);
        }
    }
    let site =
        hackrf_gnss::gps::ephemeris::geodetic_to_ecef(39.0042, -77.6095, 0.020); // km

    let mut raw: Vec<(usize, f64)> = Vec::new();
    for ch in &eng.l1_band.channels {
        if ch.sys != hackrf_gnss::live::Sys::Gps {
            continue;
        }
        let Some((t_bit, t_tx)) = ch.anchor else { continue };
        let eph = ch.eph.as_ref().or_else(|| ephs.get(&(ch.prn as u8)));
        let Some(eph) = eph else {
            println!("PRN {:2}: anchored, no ephemeris", ch.prn);
            continue;
        };
        // reception-anchored light-time solve (sat_at_txtime's tow is a GPS
        // RECEPTION time; passing the SV-clock t_tx evaluates the satellite
        // ~70 ms from emission — up to ~60 m of range-rate artifact)
        let site_m = site.map(|x| x * 1000.0);
        let (_, dt0, _) = hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, t_tx, site_m);
        let mut a = t_tx - dt0 + 0.075;
        let mut dt_sv = dt0;
        let mut rng_m = 0.0;
        for _ in 0..2 {
            let (_, d, r) = hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, a, site_m);
            dt_sv = d;
            rng_m = r;
            a = t_tx - d + r / 299_792_458.0;
        }
        let geom = rng_m / 1000.0;
        let rho_km = (t_bit - t_tx) * 299.792_458 + dt_sv * C_KM_S;
        raw.push((ch.prn, rho_km - geom));
        println!(
            "PRN {:2}: t_tx {:.0} bits {} eph {} raw {:+.3} km",
            ch.prn,
            t_tx,
            ch.nav_bits.len(),
            if ch.eph.is_some() { "self" } else { "brdc" },
            rho_km - geom
        );
    }
    if raw.len() >= 2 {
        let mut s: Vec<f64> = raw.iter().map(|&(_, r)| r).collect();
        s.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let clock = s[s.len() / 2];
        println!("\nresiduals vs survey (median removed):");
        for (prn, r) in &raw {
            let resid = r - clock;
            let teeth = resid / 299.792_458; // km per 1 ms tooth
            println!(
                "  PRN {prn:2}: {resid:+12.3} km  ({teeth:+.2} teeth){}",
                if resid.abs() > 150.0 { "  <== TOOTH AMBIGUITY" } else { "" }
            );
        }
    }
}
