//! Edge tests for TLE loading and `ephemeris_match` rejection paths, using the
//! embedded single-satellite fixture (tests/fixtures/ephemeris.json) whose
//! Doppler/elevation values are Python-oracle validated (tests/gps_ephemeris.rs).

use hackrf_gnss::gps::{ephemeris_match, geodetic_to_ecef, load_tle, load_tle_named};
use std::fs;
use std::path::PathBuf;

fn fixture() -> serde_json::Value {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/ephemeris.json");
    serde_json::from_str(&fs::read_to_string(p).unwrap()).unwrap()
}

fn fixture_tle_text(f: &serde_json::Value) -> String {
    let tle = f["tle"].as_array().unwrap();
    format!(
        "{}\n{}\n{}\n",
        tle[0].as_str().unwrap(),
        tle[1].as_str().unwrap(),
        tle[2].as_str().unwrap()
    )
}

fn fx(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(name)
}

#[test]
fn too_few_crossings_cannot_be_checked() {
    let f = fixture();
    let prn = f["prn"].as_u64().unwrap() as u16;
    let sats = load_tle(&fixture_tle_text(&f));
    let rx = geodetic_to_ecef(f["rx_lat"].as_f64().unwrap(), f["rx_lon"].as_f64().unwrap(), 0.0);
    let times: Vec<f64> = f["times"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
    let fd: Vec<f64> = f["fd_l1"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();

    // a Doppler rate needs at least three points; two must be "cannot check",
    // never a fabricated verdict
    let two: Vec<(f64, f64)> = times[..2].iter().zip(&fd[..2]).map(|(&t, &d)| (t, d)).collect();
    let m = ephemeris_match(prn, &two, &sats, rx, 0.0, 0.15, 2500.0);
    assert_eq!(m.matched, None, "two crossings gave a verdict: {m:?}");
    assert_eq!(m.reason.as_deref(), Some("too few crossings"));
    assert!(m.rate_obs_hz_s.is_none());
}

#[test]
fn a_satellite_below_the_horizon_is_a_contradiction_not_a_match() {
    let f = fixture();
    let prn = f["prn"].as_u64().unwrap() as u16;
    let sats = load_tle(&fixture_tle_text(&f));
    let rx = geodetic_to_ecef(f["rx_lat"].as_f64().unwrap(), f["rx_lon"].as_f64().unwrap(), 0.0);
    let times: Vec<f64> = f["times"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
    let fd: Vec<f64> = f["fd_l1"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
    let real: Vec<(f64, f64)> = times.iter().zip(&fd).map(|(&t, &d)| (t, d)).collect();

    // an impossible 90-degree elevation gate: even perfect Doppler geometry
    // cannot match a satellite that never rises. The verdict must be
    // Some(false) -- a physical contradiction -- and must REPORT the maximum
    // elevation it actually computed, not drop it.
    let m = ephemeris_match(prn, &real, &sats, rx, 90.0, 0.15, 2500.0);
    assert_eq!(m.matched, Some(false), "below-horizon pass matched: {m:?}");
    let el = m.el_deg.expect("the max elevation seen must be reported");
    assert!(el < 90.0, "el {el}");
    assert_eq!(m.norad.as_deref(), Some(sats[0].name.as_str()));
    assert!(m.reason.is_some());
    // and the Doppler statistics must NOT be filled in for a rejected pass
    assert!(m.rate_obs_hz_s.is_none());
}

#[test]
fn malformed_tle_text_is_skipped_not_fatal() {
    let f = fixture();
    let good = fixture_tle_text(&f);
    // garbage before, between and after the one valid triple; a broken second
    // triple whose element lines are not TLE lines at all
    let text = format!(
        "GARBAGE HEADER\n{good}not a name\n1 broken\n2 broken\nTRAILING JUNK WITHOUT TRIPLE\n"
    );
    let sats = load_tle(&text);
    assert_eq!(sats.len(), 1, "expected exactly the one valid satellite: {}", sats.len());
    assert_eq!(sats[0].prn, f["prn"].as_u64().unwrap() as u16);
    // completely unusable input parses to nothing, without panicking
    assert!(load_tle("hello\nworld\n").is_empty());
    assert!(load_tle("").is_empty());
}

#[test]
fn load_tle_keeps_only_prn_titles_while_load_tle_named_keeps_all() {
    // the Iridium fixture titles carry no "(PRN NN)": the GPS loader must
    // drop every one of them, the generic loader must keep them all with
    // prn == 0 -- the two loaders differ ONLY in that filter
    let text = fs::read_to_string(fx("iridium.tle")).unwrap();
    assert!(load_tle(&text).is_empty(), "Iridium titles must not load as GPS sats");
    let named = load_tle_named(&text);
    assert!(named.len() > 10, "fixture should hold dozens of Iridium sats");
    assert!(named.iter().all(|s| s.prn == 0));
    assert!(named.iter().all(|s| s.name.starts_with("IRIDIUM")));
}
