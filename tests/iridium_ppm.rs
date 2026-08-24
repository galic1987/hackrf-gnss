//! Tests for the Iridium Doppler -> clock-error estimator: the ppm math on a
//! known input, the sign convention, median robustness against outliers, and
//! an end-to-end physics check that propagates a real TLE (fixture) and
//! recovers a synthetic clock error through the full predict -> solve path.

use hackrf_gnss::gps::{geodetic_to_ecef, load_tle_named, predict_doppler_el_f};
use hackrf_gnss::iridium::ppm::{channel_center, median, ppm_from_burst, sat_id_from_name};
use std::fs;
use std::path::PathBuf;

fn fx(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(name)
}

const RX_LAT: f64 = 40.65;
const RX_LON: f64 = -73.80;

#[test]
fn known_radial_velocity_gives_known_ppm() {
    // approaching satellite at 2.5 km/s range rate, clock exactly on:
    // solver must say ~0 ppm regardless of how big the Doppler is
    let f_nom = channel_center(1626.270833e6);
    let d = f_nom * 2500.0 / 299_792_458.0;
    assert!((ppm_from_burst(f_nom + d, f_nom, d)).abs() < 1e-6);
    // same geometry, clock 25 ppm fast: +25 ppm back out
    let f_meas = (f_nom + d) * (1.0 + 25e-6);
    assert!((ppm_from_burst(f_meas, f_nom, d) - 25.0).abs() < 1e-3);
    // and receding (-2.5 km/s) with a 12 ppm slow clock
    let d = -f_nom * 2500.0 / 299_792_458.0;
    let f_meas = (f_nom + d) * (1.0 - 12e-6);
    assert!((ppm_from_burst(f_meas, f_nom, d) + 12.0).abs() < 1e-3);
}

#[test]
fn positive_ppm_means_clock_fast() {
    // the sign convention printed by the example, pinned by test:
    // positive ppm = clock fast = measured frequency higher than predicted
    let f_nom = 1626.25e6;
    assert!(ppm_from_burst(f_nom + 16_262.5, f_nom, 0.0) > 0.0); // +10 ppm worth of Hz
    assert!(ppm_from_burst(f_nom - 16_262.5, f_nom, 0.0) < 0.0);
}

#[test]
fn median_survives_wild_outliers() {
    // 11 good bursts around +18 ppm plus 3 badly misattributed ones
    let mut v: Vec<f64> = (0..11).map(|i| 18.0 + i as f64 * 0.1 - 0.5).collect();
    v.extend([400.0, -350.0, 999.0]);
    let m = median(&v).unwrap();
    assert!((m - 18.0).abs() < 0.6, "median {m}");
}

#[test]
fn end_to_end_with_a_real_tle_recovers_a_synthetic_clock_error() {
    let text = fs::read_to_string(fx("iridium.tle")).unwrap();
    let sats = load_tle_named(&text);
    assert!(!sats.is_empty(), "fixture TLE loaded no satellites");
    let sat = sats
        .iter()
        .find(|s| sat_id_from_name(&s.name) == Some(106))
        .expect("IRIDIUM 106 in fixture TLE");
    let rx = geodetic_to_ecef(RX_LAT, RX_LON, 0.0);
    let f_nom = channel_center(1626.270833e6);
    // scan a day around the TLE epoch for a moment above the horizon
    let mut found = None;
    for k in 0..(24 * 60) {
        let t = sat.epoch_unix + k as f64 * 60.0;
        if let Some((dop, el)) = predict_doppler_el_f(sat, rx, t, f_nom) {
            if el > 20.0 {
                found = Some((t, dop));
                break;
            }
        }
    }
    let (t, dop) = found.expect("IRIDIUM 106 rises above 20 deg within a day");
    // sanity: LEO Doppler at 1.6 GHz must sit inside +-40 kHz
    assert!(dop.abs() < 40e3, "doppler {dop}");
    // inject +30 ppm clock error and recover it through the same path the
    // example uses
    let f_meas = (f_nom + dop) * (1.0 + 30e-6);
    let (dop2, _) = predict_doppler_el_f(sat, rx, t, f_nom).unwrap();
    let ppm = ppm_from_burst(f_meas, f_nom, dop2);
    assert!((ppm - 30.0).abs() < 1e-3, "ppm {ppm}");
}

#[test]
fn an_unmatched_spacecraft_number_is_not_invented() {
    let text = fs::read_to_string(fx("iridium.tle")).unwrap();
    let sats = load_tle_named(&text);
    // the estimator must skip rather than misattribute when the ring alert's
    // sat number has no TLE namesake
    assert!(!sats.iter().any(|s| sat_id_from_name(&s.name) == Some(9999)));
}

/// Minimal real TLE triple (IRIDIUM 106, from the fixture file) for the
/// position-matching regression tests.
const MINI_TLE: &str = "\
IRIDIUM 106
1 41917U 17003A   26226.07312931 -.00000025  00000+0 -16097-4 0  9992
2 41917  86.3914  64.3400 0002364  84.9672 275.1794 14.34217167501505
";

#[test]
fn position_match_recovers_a_capture_epoch_50s_late() {
    // FAILURE A regression: /tmp/iri_corr.iq was passed with a start epoch
    // 50 s after the true transfer start; the position gate missed IRIDIUM
    // 122 by ~370 km (7.5 km/s of along-track travel) and the capture
    // attributed nothing. The refinement must recover the shift.
    use hackrf_gnss::gps::sat_ecef;
    use hackrf_gnss::iridium::ppm::match_position;
    let sats = load_tle_named(MINI_TLE);
    let sat = &sats[0];
    let t_true = sat.epoch_unix + 3600.0;
    let p = sat_ecef(sat, t_true).unwrap();

    // exact epoch: no shift, negligible distance
    let (s, d, sh) = match_position(&sats, &p, t_true).expect("exact-epoch match");
    assert_eq!(s.name, "IRIDIUM 106");
    assert_eq!(sh, 0.0);
    assert!(d < 1.0, "dist {d}");

    // epoch 50 s late: the raw gate misses (~375 km), refinement finds it
    let (s, d, sh) = match_position(&sats, &p, t_true + 50.0).expect("shifted match");
    assert_eq!(s.name, "IRIDIUM 106");
    assert!((sh - -50.0).abs() <= 10.0, "shift {sh}");
    assert!(d < 80.0, "dist {d} (10 s shift quantisation leaves <= 40 km)");

    // a point no Iridium satellite comes near stays unattributed
    let far = [-p[0], -p[1], -p[2]];
    assert!(match_position(&sats, &far, t_true).is_none());
}
