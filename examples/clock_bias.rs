//! 1 Hz receiver clock-bias series for the sub-ns precision claim (Leg 1).
//!
//! Reads state.tracker.json once per second (the tracker owns the radio; we
//! never touch it), Hatch-smooths each GPS satellite's pseudorange against its
//! published carrier (reset on slip), solves PVT weighted AND unweighted
//! (paired elevation-weighting A/B), and appends one row to
//! observations/clock_bias.jsonl. Rows:
//! {epoch, clock_ns, clock_ns_uw, tdop, n_sat, gdop, residual_rms_m,
//!  residual_rms_m_uw, n_smoothed, slips, source}

use hackrf_gnss::gps::broadcast::{parse_rinex_gps, wrap_tk, BrdcEph};
use hackrf_gnss::gps::hatch::Hatch;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::{fs, thread, time::Duration};

const STATE: &str = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json";
const OUT: &str = "/Volumes/Radiator 8TB/gnss/observations/clock_bias.jsonl";
const RINEX: &str = "/Volumes/Radiator 8TB/gnss/observations/brdc_latest.rnx";
const TRACKER_EPH: &str = "/Volumes/Radiator 8TB/gnss/observations/tracker_eph.json";
const SITE_JSON: &str = "/Volumes/Radiator 8TB/gnss/observations/site.json";
const LAM_L1: f64 = 299_792_458.0 / 1_575_420_000.0;
const WINDOW_S: f64 = 100.0;
/// BRDC refresh cadence (position_producer refetches the file hourly).
const EPH_REFRESH_S: f64 = 900.0;

fn unix_now() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64()
}

/// Canonical ephemeris path, copied from live_fix.rs:237-240 + :272-313 (GPS
/// branch): BRDC from the RINEX file position_producer refreshes, with live
/// self-decoded LNAV ephemerides taking precedence ONLY when they are the
/// newer valid issue (health belt + wrap-aware toe compare — the tracker
/// keeps its first decode per channel and re-stamps the envelope every
/// write, so envelope freshness says nothing about issue freshness).
fn load_ephs() -> HashMap<u8, BrdcEph> {
    let mut ephs = match fs::read_to_string(RINEX) {
        Ok(t) => parse_rinex_gps(&t),
        Err(_) => Default::default(),
    };
    if let Ok(t) = fs::read_to_string(TRACKER_EPH) {
        if let Ok(v) = serde_json::from_str::<Value>(&t) {
            for e in v["ephemeris"].as_array().into_iter().flatten() {
                if let Ok(eph) = serde_json::from_value::<BrdcEph>(e.clone()) {
                    if eph.sys != 0 {
                        continue; // GPS only in this example
                    }
                    // health belt: never merge a KNOWN-unhealthy self-decode
                    if let Some(h) = eph.health.filter(|&h| h != 0) {
                        eprintln!(
                            "clock_bias: self-decoded ephemeris PRN {} rejected — SV health {}",
                            eph.prn, h
                        );
                        continue;
                    }
                    if ephs.get(&eph.prn).is_some_and(|c| wrap_tk(eph.toe - c.toe) <= 0.0) {
                        continue; // the held record is the newer (or same) issue
                    }
                    ephs.insert(eph.prn, eph);
                }
            }
        }
    }
    ephs
}

/// Canonical site anchor (observations/site.json) — NO hardcoded
/// coordinates, copied from live_fix.rs:218-223.
fn site_lla() -> [f64; 3] {
    hackrf_gnss::site::load_site(std::path::Path::new(SITE_JSON)).unwrap_or_else(|| {
        eprintln!("clock_bias: no site anchor — provide {SITE_JSON}");
        std::process::exit(2);
    })
}

/// live_fix's solve guess: site LLA -> ECEF (km).
fn site_guess() -> [f64; 3] {
    let lla = site_lla();
    hackrf_gnss::gps::ephemeris::geodetic_to_ecef(lla[0], lla[1], lla[2] / 1000.0)
}

fn build_meas(
    prn: u8,
    rho_m: f64,
    t_tx: f64,
    ephs: &HashMap<u8, BrdcEph>,
    site_m: [f64; 3],
) -> Option<hackrf_gnss::gps::pvt::Meas> {
    // live_fix.rs:574-585 + :678-682 (GPS branch), verbatim except the SBAS
    // terms: this example harvests no SBAS corrections, so prc / iono_m /
    // daf0 are 0 — the (dt_sv + daf0) SV-clock structure is kept.
    let eph = ephs.get(&prn)?;
    let (_, dt0, _) = hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, t_tx, site_m);
    let mut a = t_tx - dt0 + 0.075;
    let (mut sat_m, mut dt_sv) = ([0.0; 3], dt0);
    for _ in 0..2 {
        let (s, d, r) = hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, a, site_m);
        sat_m = s;
        dt_sv = d;
        a = t_tx - d + r / 299_792_458.0;
    }
    let (prc, iono_m, daf0) = (0.0, 0.0, 0.0); // no SBAS corrections here
    Some(hackrf_gnss::gps::pvt::Meas {
        sat: [sat_m[0] / 1000.0, sat_m[1] / 1000.0, sat_m[2] / 1000.0],
        pseudorange: (rho_m + prc - iono_m) / 1000.0 + (dt_sv + daf0) * 299_792.458, // sat clock removed
        clock_free: false,
    })
}

fn main() {
    // ephemeris: same loader live_fix uses (BRDC); refresh every 15 min
    let mut ephs = load_ephs();
    let mut eph_loaded = unix_now();
    // light-time anchor for sat_at_txtime_pub (metres), site-anchored —
    // live_fix's site_m, built from the canonical anchor here
    let site_m = site_guess().map(|x| x * 1000.0);
    let mut smoothers: HashMap<u8, Hatch> = HashMap::new();
    let mut last_epoch = 0.0_f64;
    loop {
        thread::sleep(Duration::from_millis(500));
        let txt = match fs::read_to_string(STATE) { Ok(t) => t, Err(_) => continue };
        let st: Value = match serde_json::from_str(&txt) { Ok(v) => v, Err(_) => continue };
        let epoch = st["epoch"].as_f64().unwrap_or(0.0);
        if epoch <= last_epoch { continue; }
        last_epoch = epoch;
        let sats = match st["tracker"]["sats"].as_array() { Some(s) => s, None => continue };

        if ephs.is_empty() || unix_now() - eph_loaded > EPH_REFRESH_S {
            ephs = load_ephs();
            eph_loaded = unix_now();
        }

        let mut meas = Vec::new();
        let mut slips = 0u32;
        let mut n_smoothed = 0u32;
        for s in sats {
            if s["sys"].as_str() != Some("gps") { continue; }
            if s["cn0_proxy"].as_f64().unwrap_or(0.0) < 30.0 { continue; }
            if s["lock_s"].as_f64().unwrap_or(0.0) < 20.0 { continue; }
            let (rho, t_tx) = match (s["rho_m"].as_f64(), s["t_tx"].as_f64()) {
                (Some(a), Some(b)) => (a, b), _ => continue };
            let prn = s["prn"].as_u64().unwrap_or(0) as u8;
            let slip = s["slip"].as_bool().unwrap_or(false);
            if slip { slips += 1; }
            let carr = s["carrier_cycles"].as_f64().unwrap_or(0.0);
            let h = smoothers.entry(prn).or_insert_with(|| Hatch::new(WINDOW_S));
            let rho_s = h.update(rho, carr, LAM_L1, slip);
            n_smoothed += 1;
            // sat position + SV clock correction: live_fix.rs:574-585 and
            // :678-682 (sat_at_txtime_pub, dt_sv + daf0 term)
            meas.push(build_meas(prn, rho_s, t_tx, &ephs, site_m)); // Option<Meas>, filtered below
        }
        let meas: Vec<_> = meas.into_iter().flatten().collect();
        if meas.len() < 5 { continue; }   // redundancy required; exact 4-sat solves are unverifiable
        let g = site_guess();
        let fw = hackrf_gnss::gps::pvt::solve(&meas, g);             // weighted (default)
        let fu = hackrf_gnss::gps::pvt::solve_unweighted(&meas, g);  // paired A/B
        if let (Some(a), Some(b)) = (fw, fu) {
            let row = json!({
                "epoch": epoch,
                "clock_ns": a.clock_km * 1e9 / 299_792.458,
                "clock_ns_uw": b.clock_km * 1e9 / 299_792.458,
                "tdop": a.tdop, "n_sat": a.n_sat, "gdop": a.gdop,
                "residual_rms_m": a.residual_rms_m,
                "residual_rms_m_uw": b.residual_rms_m,
                "n_smoothed": n_smoothed, "slips": slips,
                "source": "clock_bias",
            });
            use std::io::Write;
            let mut f = fs::OpenOptions::new().create(true).append(true).open(OUT).unwrap();
            writeln!(f, "{}", row).unwrap();
        }
    }
}
