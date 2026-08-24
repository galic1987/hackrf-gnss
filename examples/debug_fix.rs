//! Debug: compare the live tracker's per-PRN code phases against the
//! ephemeris-predicted sub-ms arrival fractions at the surveyed site.
//! Prints per-sat observed-vs-predicted under BOTH sign conventions so the
//! tracker/solver phase-convention mismatch is directly visible.

use hackrf_gnss::gps::broadcast::parse_rinex_gps;
use hackrf_gnss::gps::snapshot::Obs;

const TRACKER_STATE: &str = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json";
const RINEX: &str = "/Volumes/Radiator 8TB/gnss/observations/brdc_latest.rnx";
const APPROX_LLA: [f64; 3] = [40.65, -73.80, 20.0];
const GPS_UNIX_EPOCH: f64 = 315_964_800.0;
const LEAP_S: f64 = 18.0;

fn main() {
    let ephs = parse_rinex_gps(&std::fs::read_to_string(RINEX).unwrap());
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(TRACKER_STATE).unwrap()).unwrap();
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();
    let tow = (now - GPS_UNIX_EPOCH + LEAP_S) % 604_800.0;

    let mut obs = Vec::new();
    for s in v["tracker"]["sats"].as_array().into_iter().flatten() {
        if s["sys"].as_str() != Some("gps") {
            continue;
        }
        let (Some(prn), Some(cp)) = (s["prn"].as_u64().map(|p| p as u8), s["code_phase"].as_f64())
        else {
            continue;
        };
        obs.push(Obs { prn, code_phase: cp, doppler: 0.0 });
    }
    println!("{:<6} {:>10} {:>10} {:>10} {:>10}", "prn", "obs_chips", "pred_frac", "match+", "match-");
    for o in &obs {
        let Some(e) = ephs.get(&o.prn) else {
            println!("{:>3} no eph", o.prn);
            continue;
        };
        // predicted sub-ms arrival fraction at the approx site
        let g = hackrf_gnss::gps::ephemeris::geodetic_to_ecef(APPROX_LLA[0], APPROX_LLA[1], APPROX_LLA[2] / 1000.0);
        let rx_m = [g[0] * 1000.0, g[1] * 1000.0, g[2] * 1000.0];
        let (_, dt, rng) = hackrf_gnss::gps::snapshot::sat_at_txtime_pub(e, tow, rx_m);
        let pr_m = rng - dt * 299_792_458.0;
        let pred = ((pr_m / 299_792.458) % 1.0 + 1.0) % 1.0; // fraction of ms
        let obs_frac = (o.code_phase / 1023.0).rem_euclid(1.0);
        let d_pos = (obs_frac - pred).rem_euclid(1.0).min(1.0 - (obs_frac - pred).rem_euclid(1.0));
        let comp = (1.0 - obs_frac).rem_euclid(1.0);
        let d_neg = (comp - pred).rem_euclid(1.0).min(1.0 - (comp - pred).rem_euclid(1.0));
        println!(
            "{:>3} {:>10.1} {:>10.4} {:>10.4} {:>10.4}",
            o.prn, o.code_phase, pred, d_pos, d_neg
        );
    }
}
