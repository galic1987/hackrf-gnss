//! Cross-language parity of the Rust ephemeris port against the Python oracle
//! (`validation/identify.py` + `report.py:_ephemeris_check`). The fixture embeds
//! one satellite's exact TLE plus the L1 Doppler and elevation Python's SGP4 +
//! geometry predict at eight epochs. The Rust SGP4/geometry must reproduce those
//! numbers, and the own-PRN match must confirm real geometry while rejecting a
//! spur that merely ramps.

use hackrf_gnss::gps::{ephemeris_match, geodetic_to_ecef, load_tle, predict_doppler_el};
use std::fs;
use std::path::PathBuf;

fn fixture() -> serde_json::Value {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/ephemeris.json");
    serde_json::from_str(&fs::read_to_string(p).unwrap()).unwrap()
}

#[test]
fn rust_doppler_prediction_matches_python() {
    let f = fixture();
    let prn = f["prn"].as_u64().unwrap() as u16;
    let tle = f["tle"].as_array().unwrap();
    let text = format!(
        "{}\n{}\n{}\n",
        tle[0].as_str().unwrap(),
        tle[1].as_str().unwrap(),
        tle[2].as_str().unwrap()
    );
    let sats = load_tle(&text);
    assert_eq!(sats.len(), 1, "embedded TLE should parse to one sat");
    let sat = &sats[0];
    assert_eq!(sat.prn, prn);

    let rx = geodetic_to_ecef(f["rx_lat"].as_f64().unwrap(), f["rx_lon"].as_f64().unwrap(), 0.0);
    let times: Vec<f64> = f["times"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
    let fd_py: Vec<f64> = f["fd_l1"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
    let el_py: Vec<f64> = f["el_deg"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();

    for i in 0..times.len() {
        let (fd, el) = predict_doppler_el(sat, rx, times[i]).expect("SGP4 ok");
        assert!(
            (fd - fd_py[i]).abs() < 25.0,
            "epoch {i}: rust fd {fd:.1} vs python {:.1}",
            fd_py[i]
        );
        assert!(
            (el - el_py[i]).abs() < 0.5,
            "epoch {i}: rust el {el:.2} vs python {:.2}",
            el_py[i]
        );
    }
}

#[test]
fn ephemeris_match_confirms_real_geometry_and_rejects_spur() {
    let f = fixture();
    let prn = f["prn"].as_u64().unwrap() as u16;
    let tle = f["tle"].as_array().unwrap();
    let text = format!(
        "{}\n{}\n{}\n",
        tle[0].as_str().unwrap(),
        tle[1].as_str().unwrap(),
        tle[2].as_str().unwrap()
    );
    let sats = load_tle(&text);
    let rx_ll = (f["rx_lat"].as_f64().unwrap(), f["rx_lon"].as_f64().unwrap());
    let rx = geodetic_to_ecef(rx_ll.0, rx_ll.1, 0.0);
    let times: Vec<f64> = f["times"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
    let fd_py: Vec<f64> = f["fd_l1"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();

    // real geometry + a constant LO offset -> matched
    let real: Vec<(f64, f64)> = times.iter().zip(&fd_py).map(|(&t, &fd)| (t, fd + 2200.0)).collect();
    let m = ephemeris_match(prn, &real, &sats, rx, 0.0, 0.15, 2500.0);
    assert_eq!(m.matched, Some(true), "real geometry should match: {m:?}");

    // spur: a clean ramp whose rate is nothing like this satellite -> rejected
    let t0 = times[0];
    let spur: Vec<(f64, f64)> = times.iter().map(|&t| (t, fd_py[0] + 2200.0 + 3.0 * (t - t0))).collect();
    let ms = ephemeris_match(prn, &spur, &sats, rx, 0.0, 0.15, 2500.0);
    assert_eq!(ms.matched, Some(false), "spur should be rejected: {ms:?}");

    // a PRN not in the TLE set cannot be checked
    let none = ephemeris_match(31, &real, &sats, rx, 0.0, 0.15, 2500.0);
    assert_eq!(none.matched, None);
}
