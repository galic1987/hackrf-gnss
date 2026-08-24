//! Tests for the PpmEstimate aggregation semantics (median ppm, correction
//! sign, satellite roll-up) and for the full estimator pipeline running on the
//! real captured burst fixture: detection and decode must succeed, but a burst
//! that cannot be attributed to a satellite must yield NO clock estimate.

use hackrf_gnss::gps::geodetic_to_ecef;
use hackrf_gnss::iridium::ppm::{estimate_ppm, BurstEst, PpmEstimate, CHANNEL_WIDTH};
use std::fs;
use std::path::PathBuf;

fn fx(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(name)
}

fn burst(sat: &str, ppm: f64) -> BurstEst {
    BurstEst {
        sat: sat.into(),
        ppm,
        doppler_hz: -12_000.0,
        f_meas_hz: 1_626_270_800.0,
        f_nom_hz: 1_626_270_833.33,
        t_epoch: 1_787_000_000.0,
        confidence: 90,
        match_km: 42.0,
        epoch_shift_s: 0.0,
    }
}

#[test]
fn ppm_is_the_median_and_the_correction_is_its_negative() {
    let est = PpmEstimate {
        per_burst: vec![
            burst("IRIDIUM 106", -26.1),
            burst("IRIDIUM 104", -25.9),
            burst("IRIDIUM 106", -25.8),
        ],
        ..PpmEstimate::default()
    };
    let p = est.ppm().unwrap();
    assert!((p - -25.9).abs() < 1e-12, "median ppm {p}");
    // hackrf_set_clock_correction takes the NEGATIVE of the measured error
    let d = est.correction_delta().unwrap();
    assert!((d - 25.9).abs() < 1e-12, "correction delta {d}");
}

#[test]
fn an_empty_estimate_has_no_ppm_and_no_correction() {
    let est = PpmEstimate::default();
    assert!(est.ppm().is_none());
    assert!(est.correction_delta().is_none());
    assert!(est.sats().is_empty());
}

#[test]
fn sats_are_sorted_and_deduplicated() {
    let est = PpmEstimate {
        per_burst: vec![
            burst("IRIDIUM 106", 1.0),
            burst("IRIDIUM 104", 2.0),
            burst("IRIDIUM 106", 3.0),
        ],
        ..PpmEstimate::default()
    };
    assert_eq!(est.sats(), vec!["IRIDIUM 104".to_string(), "IRIDIUM 106".to_string()]);
}

#[test]
fn estimator_on_a_real_burst_never_invents_a_clock() {
    // tests/fixtures/iridium_burst.iq is one clean simplex burst. Through the
    // full estimator pipeline the burst is DETECTED, and -- because the burst
    // window find_bursts2 cuts from this short capture does not survive full
    // demodulation -- it lands in `detected_fine`: the preamble tone still
    // yields a fine carrier (that path exists precisely so an undecodable
    // burst is not silently dropped). With no decodable frame there is no
    // satellite attribution, so the estimate must carry NO ppm: a made-up
    // clock correction is worse than none.
    let meta: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(fx("iridium_burst.json")).unwrap()).unwrap();
    let raw: Vec<i8> = fs::read(fx("iridium_burst.iq"))
        .unwrap()
        .into_iter()
        .map(|b| b as i8)
        .collect();
    let fc = meta["fc"].as_f64().unwrap();
    let fs_hz = meta["fs"].as_f64().unwrap();
    let dur = raw.len() as f64 / 2.0 / fs_hz;
    let tle = fs::read_to_string(fx("iridium.tle")).unwrap();
    let sats = hackrf_gnss::gps::load_tle_named(&tle);
    let rx = geodetic_to_ecef(40.65, -73.80, 0.0);

    let est = estimate_ppm(&raw, fc, fs_hz, dur, 1_787_000_000.0, &sats, rx);

    assert!(est.detected >= 1, "the fixture burst must be detected");
    // the burst is accounted for somewhere: decoded or fine-carrier-only
    assert!(
        est.decoded + est.detected_fine.len() >= 1,
        "a detected burst vanished without a trace: {est:?}"
    );
    // no attribution -> no clock estimate, on every output path
    assert!(est.per_burst.is_empty(), "unattributable burst attributed: {:?}", est.per_burst);
    assert!(est.ppm().is_none(), "no attribution must mean no ppm");
    assert!(est.correction_delta().is_none());
    assert!(est.ambiguous_ppm.is_none(), "no fold alternative without bursts");
    // every reported carrier is internally consistent: its snapped channel
    // centre is within half a channel of the measurement
    for db in est.decoded_bursts.iter().chain(est.detected_fine.iter()) {
        assert!(
            (db.f_meas_hz - db.f_nom_hz).abs() <= CHANNEL_WIDTH / 2.0 + 1.0,
            "carrier {} snapped to {} ({} Hz off grid)",
            db.f_meas_hz,
            db.f_nom_hz,
            db.f_meas_hz - db.f_nom_hz
        );
    }
    // the fine carrier lands inside the simplex band the estimator searched
    for db in &est.detected_fine {
        assert!(
            (db.f_meas_hz - 1626.25e6).abs() < 500e3,
            "fine carrier {} outside the searched band",
            db.f_meas_hz
        );
        assert_eq!(db.confidence, 0, "an undecoded burst carries no confidence");
    }
}
