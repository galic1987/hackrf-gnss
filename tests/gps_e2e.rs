//! End-to-end GPS chain on real IQ: acquire the L1 signal out of a simulated
//! capture, feed the measured code phases to the coarse-time snapshot solver
//! with broadcast ephemeris, and confirm it recovers the KNOWN receiver site.
//!
//! This is the Rust counterpart of the Python Tier-3 test (which hit 30 m). The
//! baseband fixture is the first 12 ms of `sim.iq` (an IS-GPS L1 simulation, 6
//! sats @ 45 dB-Hz, true site 47.4979 N 19.0402 E) with the IF removed; the nav
//! fixture is that simulation's own ephemeris re-emitted as RINEX-3.

use hackrf_gnss::gps::{acquire, parse_rinex_gps, snapshot_fix, Obs};
use num_complex::Complex;

const BB: &[u8] = include_bytes!("fixtures/sim_l1_bb.f32");
const NAV: &str = include_str!("fixtures/sim_nav.rnx");

// ground truth from sim.truth.json
const TRUE_LAT: f64 = 47.4979;
const TRUE_LON: f64 = 19.0402;
const TOW_START: f64 = 345597.337; // GPS time-of-week at the start of the capture
const FS: f64 = 8_000_000.0;

fn load_baseband() -> Vec<Complex<f32>> {
    BB.chunks_exact(8)
        .map(|c| {
            let i = f32::from_le_bytes([c[0], c[1], c[2], c[3]]);
            let q = f32::from_le_bytes([c[4], c[5], c[6], c[7]]);
            Complex::new(i, q)
        })
        .collect()
}

#[test]
fn acquire_then_snapshot_recovers_the_true_site() {
    let sig = load_baseband();
    assert!(sig.len() >= 80_000, "fixture too short: {} samples", sig.len());

    // acquisition: residual Doppler is near 0 (IF already removed)
    let dopplers: Vec<f64> = (-6000..=6000).step_by(200).map(|x| x as f64).collect();
    let prns: Vec<usize> = vec![5, 9, 10, 16, 20, 26];
    let res = acquire(&sig, FS, &prns, &dopplers, 10, 2.5);

    let obs: Vec<Obs> = res
        .iter()
        .filter(|r| r.metric >= 2.5)
        .map(|r| Obs { prn: r.prn as u8, code_phase: r.code_phase, doppler: r.doppler })
        .collect();
    assert!(obs.len() >= 4, "acquired only {} of 6 sats (need >=4)", obs.len());

    // snapshot fix from the sim's own broadcast ephemeris, seeded ~80 km off
    let ephs = parse_rinex_gps(NAV);
    assert!(ephs.len() >= 4, "parsed only {} sim ephemerides", ephs.len());
    let fix = snapshot_fix(&obs, &ephs, [TRUE_LAT + 0.5, TRUE_LON - 0.6, 0.0], TOW_START)
        .expect("snapshot fix should solve with >=4 sats");

    let err_lat = (fix.lat - TRUE_LAT) * 111_000.0;
    let err_lon = (fix.lon - TRUE_LON) * 111_000.0 * TRUE_LAT.to_radians().cos();
    let err = (err_lat * err_lat + err_lon * err_lon).sqrt();
    println!(
        "e2e fix: {:.5} N {:.5} E  ({} sats, rms {:.0} m)  error {:.0} m",
        fix.lat, fix.lon, fix.n_sat, fix.residual_rms_m, err
    );
    assert!(err < 5000.0, "e2e fix error {err:.0} m (expected < 5 km)");
}
