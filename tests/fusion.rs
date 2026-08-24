//! Tests for the fused multi-satellite sync engine (`src/fusion.rs`):
//! the common Observation currency, the FPGA tick/anchor math, and the
//! per-constellation adapters.

use hackrf_gnss::fusion::*;

#[test]
fn observation_serde_roundtrip() {
    let o = Observation {
        source: Constellation::Iridium,
        sat_id: "IRIDIUM 106".into(),
        t_rel_s: 12.5,
        kind: ObsKind::DopplerHz,
        value: 1_626_270_833.3,
        aux: 1_626_270_833.0,
        sigma: 25.0,
        capture_id: 2,
    };
    let s = serde_json::to_string(&o).unwrap();
    let back: Observation = serde_json::from_str(&s).unwrap();
    assert_eq!(o, back);
}

#[test]
fn ticks_for_utc_wraps_mod_2_40() {
    // 1 s after anchor at 32 MHz
    assert_eq!(ticks_for_utc(1000.0, 999.0, 32.0e6), 32_000_000);
    // before anchor by half a wrap: negative time wraps into the 40-bit range
    let t = ticks_for_utc(1000.0 - 17179.869184, 1000.0, 32.0e6); // −2^39 ticks
    assert_eq!(t, 1u64 << 39);
}

#[test]
fn parse_ts_read_happy_and_missing_line() {
    assert_eq!(parse_ts_read("ts.now = 123456 ticks\n"), Some(123456));
    assert_eq!(parse_ts_read("ts.start = 42 ticks\n"), Some(42));
    // hackrf_pro exits 0 even on failure: missing ts. line MUST be a failure
    assert_eq!(parse_ts_read("hackrf_open() failed\n"), None);
    assert_eq!(parse_ts_read(""), None);
}

// ---- Iridium adapter -----------------------------------------------------

use hackrf_gnss::iridium::ppm::{BurstEst, PpmEstimate};
use std::path::PathBuf;

fn fx(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(name)
}

fn burst_est(sat: &str, ppm: f64, conf: u32, t_epoch: f64, shift: f64) -> BurstEst {
    BurstEst {
        sat: sat.into(),
        ppm,
        doppler_hz: -12_000.0,
        f_meas_hz: 1_626_270_800.0,
        f_nom_hz: 1_626_270_833.33,
        t_epoch,
        confidence: conf,
        match_km: 42.0,
        epoch_shift_s: shift,
    }
}

#[test]
fn iridium_adapter_maps_an_estimate_to_observations() {
    let pc_epoch = 1_787_000_000.0;
    let est = PpmEstimate {
        detected: 2,
        decoded: 2,
        per_burst: vec![
            burst_est("IRIDIUM 106", -26.0, 100, pc_epoch + 1.25, 0.0),
            burst_est("IRIDIUM 104", -24.0, 20, pc_epoch + 9.5, -30.0),
        ],
        ..PpmEstimate::default()
    };
    let obs = observations_from_estimate(&est, 100.0, pc_epoch, 22.0, 3);

    // one DopplerHz per attributed burst
    let dopp: Vec<&Observation> = obs.iter().filter(|o| o.kind == ObsKind::DopplerHz).collect();
    assert_eq!(dopp.len(), 2);
    assert_eq!(dopp[0].sat_id, "IRIDIUM 106");
    assert_eq!(dopp[0].value, 1_626_270_800.0);
    assert_eq!(dopp[0].aux, 1_626_270_833.33);
    assert_eq!(dopp[0].t_rel_s, 100.0 + 1.25);
    // sigma scales inversely with confidence, floored at 20% confidence
    assert_eq!(dopp[0].sigma, 50.0 / 1.0);
    assert_eq!(dopp[1].sigma, 50.0 / 0.2);
    assert!(dopp
        .iter()
        .all(|o| o.source == Constellation::Iridium && o.capture_id == 3));

    // one TimeFix per capture from the median nonzero epoch shift
    let tf: Vec<&Observation> = obs.iter().filter(|o| o.kind == ObsKind::TimeFix).collect();
    assert_eq!(tf.len(), 1);
    assert_eq!(tf[0].value, pc_epoch - 30.0);
    assert_eq!(tf[0].sigma, 5.0);
    assert_eq!(tf[0].t_rel_s, 100.0);

    // one ClockDriftPpm from the estimate's median ppm, stamped mid-capture
    let cd: Vec<&Observation> = obs
        .iter()
        .filter(|o| o.kind == ObsKind::ClockDriftPpm)
        .collect();
    assert_eq!(cd.len(), 1);
    // ppm::median takes s[len/2] of the sorted values: [-26, -24] -> -24
    assert_eq!(cd[0].value, -24.0);
    assert_eq!(cd[0].t_rel_s, 100.0 + 11.0);
    assert_eq!(cd[0].sigma, 0.1);

    // no TimeFix when no burst carries a nonzero epoch shift
    let est0 = PpmEstimate {
        per_burst: vec![burst_est("IRIDIUM 106", -26.0, 90, pc_epoch + 1.0, 0.0)],
        ..PpmEstimate::default()
    };
    let obs0 = observations_from_estimate(&est0, 0.0, pc_epoch, 22.0, 0);
    assert!(!obs0.iter().any(|o| o.kind == ObsKind::TimeFix));

    // nothing attributed -> no observations at all
    let obs1 = observations_from_estimate(&PpmEstimate::default(), 0.0, pc_epoch, 22.0, 0);
    assert!(obs1.is_empty());
}

#[test]
fn iridium_adapter_never_invents_observations_from_an_unattributable_burst() {
    // tests/fixtures/iridium_burst.iq is one clean simplex burst, but its
    // frame classifies as "unknown" (not a ring alert or broadcast), so no
    // satellite attribution is possible and the adapter must emit nothing --
    // a missing observation is honest, a made-up one is not.
    let raw: Vec<i8> = std::fs::read(fx("iridium_burst.iq"))
        .unwrap()
        .into_iter()
        .map(|b| b as i8)
        .collect();
    let meta: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(fx("iridium_burst.json")).unwrap()).unwrap();
    let (fc, fs) = (meta["fc"].as_f64().unwrap(), meta["fs"].as_f64().unwrap());
    let tle = std::fs::read_to_string(fx("iridium.tle")).unwrap();
    let sats = hackrf_gnss::gps::load_tle_named(&tle);
    let rx = hackrf_gnss::gps::geodetic_to_ecef(40.65, -73.80, 0.0);
    let dur = raw.len() as f64 / 2.0 / fs;
    let obs = iridium_observations(&raw, fc, fs, dur, 0.0, 1_787_000_000.0, &sats, rx, 0);
    assert!(obs.is_empty(), "unattributable burst yielded {obs:?}");
}

// ---- GPS time adapter ----------------------------------------------------

use hackrf_gnss::gps::lnav::Subframe;

#[test]
fn tow_to_unix_accounts_for_week_and_leap() {
    // GPS week 0 TOW 0 = 1980-01-06 00:00:00 UTC = 315964800 unix; leap 18 s
    assert_eq!(tow_to_unix(0, 18.0), GPS_UNIX_EPOCH);
    assert_eq!(tow_to_unix(1, 18.0), GPS_UNIX_EPOCH + 604_800.0);
    assert_eq!(current_gps_week(GPS_UNIX_EPOCH + 18.0), 0);
    assert_eq!(current_gps_week(GPS_UNIX_EPOCH + 604_800.0 + 18.0), 1);
}

#[test]
fn gps_timefix_from_subframe_fields() {
    // construct a Subframe directly (parity-valid bitstream synthesis is out of
    // scope here; lnav's own tests cover find_subframes)
    let sf = Subframe { sfid: 1, tow_next: 100, words: vec![[0u8; 24]; 10], bit_index: 0 };
    let pc_now = GPS_UNIX_EPOCH + 604_800.0 * 2.0; // week 2
    let o = gps_time_observation_from(&sf, 30.0, pc_now, 0).unwrap();
    assert_eq!(o.kind, ObsKind::TimeFix);
    // current subframe TOW = (100-1)*6 = 594 s into the week
    assert_eq!(o.value, tow_to_unix(2, 594.0));
    assert_eq!(o.t_rel_s, 30.0);
}

#[test]
fn gps_timefix_matches_the_live_decode() {
    // 2026-08-22 outdoor session, examples/gps_tow.rs: a capture stamped with
    // PC epoch 1787433256 decoded tow_next 99147 on sfid 2 (validated live
    // against NTP to ~0.14 s across 6 PRNs). The adapter must reproduce that:
    // current subframe TOW = (99147-1)*6 = 594876 s into GPS week 2432.
    let sf = Subframe { sfid: 2, tow_next: 99147, words: vec![[0u8; 24]; 10], bit_index: 0 };
    let pc_now = 1_787_433_256.0;
    let o = gps_time_observation_from(&sf, 30.0, pc_now, 0).unwrap();
    assert_eq!(o.value, tow_to_unix(2432, 594_876.0));
    assert!(
        (o.value - pc_now).abs() < 60.0,
        "TOW-derived UTC {} vs PC epoch {}",
        o.value,
        pc_now
    );
}

// ---- generic carrier adapter ----------------------------------------------

#[test]
fn carrier_adapter_measures_a_synthetic_tone() {
    use num_complex::Complex;
    let fs = 4.0e6;
    let n = 1 << 20;
    let f_nom = 1_543_000_000.0;          // e.g. an Inmarsat carrier, RF
    let f_carr = 1_541_000_000.0;          // tuned centre
    let true_err_ppm = -26.0;
    let f_true = f_nom * (1.0 + true_err_ppm * 1e-6);
    let f_bb = f_true - f_carr;            // appears at baseband
    let sig: Vec<Complex<f32>> = (0..n)
        .map(|i| {
            let ph = 2.0 * std::f64::consts::PI * f_bb * i as f64 / fs;
            Complex::new(ph.cos() as f32, ph.sin() as f32)
        })
        .collect();
    // NB: -26 ppm at 1.543 GHz is -40.1 kHz, so the search must span more
    // than the plan's 20 kHz or the true peak falls outside its own window
    // (verified: with 20 kHz the adapter locks a skirt bin and reads -13 ppm).
    let o = carrier_observation(&sig, fs, f_carr, f_nom, 50_000.0, 0.0,
                                Constellation::Inmarsat, "IOR", 0).unwrap();
    assert_eq!(o.kind, ObsKind::ClockDriftPpm);
    assert!((o.value - true_err_ppm).abs() < 0.2, "got {} ppm", o.value);
    // noise-only input must yield None
    let noise: Vec<Complex<f32>> = (0..n).map(|i| {
        let mut x = (i as u64).wrapping_mul(0x9E3779B97F4A7C15);
        x ^= x >> 33; x = x.wrapping_mul(0xFF51AFD7ED558CCD); x ^= x >> 33;
        Complex::new((x as f32 / u64::MAX as f32) - 0.5, 0.0)
    }).collect();
    assert!(carrier_observation(&noise, fs, f_carr, f_nom, 50_000.0, 0.0,
                                Constellation::Inmarsat, "IOR", 0).is_none());
}

// ---- solver ----------------------------------------------------------------

/// Horizontal miss distance, km (ECEF chord — same pattern as
/// tests/doppler_fix.rs; iridium::geo exposes no distance helper).
fn err_km(fix_lat: f64, fix_lon: f64, la: f64, lo: f64) -> f64 {
    let a = hackrf_gnss::gps::geodetic_to_ecef(fix_lat, fix_lon, 0.0);
    let b = hackrf_gnss::gps::geodetic_to_ecef(la, lo, 0.0);
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

#[test]
fn solve_recovers_truth_from_synthetic_observations() {
    use hackrf_gnss::gps::{load_tle_named, predict_doppler_el_f, geodetic_to_ecef};
    let tle = std::fs::read_to_string(fx("iridium.tle")).unwrap();
    let sats = load_tle_named(&tle);
    let (la, lo) = (40.65f64, -73.80f64);
    let rx = geodetic_to_ecef(la, lo, 0.0);
    // the plan's fixed t0 (1_787_000_000) yields only 18 visible-satellite
    // epochs from the fixture TLE, and its take(12) never exceeds 50 at any
    // hour — so scan all 40 fixture sats for a 10-minute window with enough
    // (best: 83 obs / 4 sats), the same thing synth_pass does in
    // tests/doppler_fix.rs
    let mut t0 = 0.0f64;
    for h in 0..(24 * 6) {
        let cand = sats[0].epoch_unix + h as f64 * 600.0;
        let mut n = 0usize;
        for s in &sats {
            for k in 0..30 {
                let t = cand + k as f64 * 20.0;
                if let Some((_, el)) = predict_doppler_el_f(s, rx, t, 1_626_250_000.0) {
                    if el >= 8.0 { n += 1; }
                }
            }
        }
        if n > 50 { t0 = cand; break; }
    }
    assert!(t0 > 0.0, "no window with >50 synthetic obs in a day of the fixture TLE");
    let clock_ppm = -26.0e-6f64;
    let mut obs = Vec::new();
    let mut lcg = 0x12345u64;
    let mut noise = move |span: f64| { lcg ^= lcg << 13; lcg ^= lcg >> 7; lcg ^= lcg << 17;
        ((lcg >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * span };
    let mut n_dopp = 0;
    for s in &sats {
        for k in 0..30 {
            let t = t0 + k as f64 * 20.0;
            let Some((dopp, el)) = predict_doppler_el_f(s, rx, t, 1_626_250_000.0) else { continue };
            if el < 8.0 { continue; }
            let f_nom = 1_626_250_000.0;
            let f_meas = f_nom + dopp + f_nom * clock_ppm + noise(20.0);
            obs.push(Observation { source: Constellation::Iridium, sat_id: s.name.clone(),
                t_rel_s: t - t0, kind: ObsKind::DopplerHz, value: f_meas, aux: f_nom,
                sigma: 25.0, capture_id: 0 });
            n_dopp += 1;
        }
    }
    assert!(n_dopp > 50, "fixture should give >50 synthetic obs, got {n_dopp}");
    // a TimeFix 12 s off the PC prior
    obs.push(Observation { source: Constellation::Iridium, sat_id: "iridium-epoch".into(),
        t_rel_s: 0.0, kind: ObsKind::TimeFix, value: t0 + 12.0, aux: 0.0, sigma: 5.0, capture_id: 0 });
    let est = solve(&obs, &sats, t0, Some((la + 0.5, lo + 0.5))).unwrap();
    let fix = est.fix.expect("position should solve");
    let err = err_km(fix.lat_deg, fix.lon_deg, la, lo);
    assert!(err < 50.0, "position error {err} km");
    assert!((est.time_offset_s.unwrap() - (t0 + 12.0)).abs() < 6.0);
    assert!(est.tick0_utc.is_some());
}

// ---- hardware apply helpers (arg construction only — no radio in tests) ----

#[test]
fn hw_args_are_well_formed() {
    let a = ts_set_args("SERIAL", 42);
    assert_eq!(a, vec!["-d", "SERIAL", "--ts-set", "42"]);
    let c = clock_corr_args("SERIAL", 26.5);
    assert_eq!(c, vec!["-d", "SERIAL", "--clock-corr", "26.5"]);
}

// ---- GLONASS FDMA adapter --------------------------------------------------

#[test]
fn glonass_adapter_attributes_a_synthetic_channel() {
    use hackrf_gnss::glonass::{glonass_code, l1_freq, CHIP_RATE};
    use hackrf_gnss::gps::{geodetic_to_ecef, load_tle_named, predict_doppler_el_f};
    use num_complex::Complex;
    let tle = std::fs::read_to_string(fx("glonass.tle")).unwrap();
    let sats = load_tle_named(&tle);
    let rx = geodetic_to_ecef(39.001, -77.60732, 0.0);
    // ch +1 carried COSMOS 2544 in today's survey; the fixture TLE holds only
    // 2544 and 2584, so attribution must pick 2544 or nothing
    let f_nom = l1_freq(1);
    let fc = 1602.0e6;
    let sat = sats.iter().find(|s| s.name.contains("2544")).unwrap();
    let other = sats.iter().find(|s| s.name.contains("2584")).unwrap();
    // a moment with 2544 well up and 2584 down (keeps the margin rule trivially
    // satisfiable — the test pins the Doppler match, not the geometry)
    let mut found = None;
    for m in 0..(24 * 60) {
        let t = sat.epoch_unix + m as f64 * 60.0;
        let Some((d, el)) = predict_doppler_el_f(sat, rx, t, f_nom) else { continue };
        if el < 20.0 { continue; }
        let el2 = predict_doppler_el_f(other, rx, t, f_nom).map(|(_, e)| e).unwrap_or(-90.0);
        if el2 < 5.0 {
            found = Some((t, d));
            break;
        }
    }
    let (t_abs, dopp) = found.expect("a window with 2544 up and 2584 down within a day");
    assert!(dopp.abs() < 10_000.0, "LEO Doppler inside the acq grid: {dopp}");

    // 0.1 s at 2 Msps (q=1: no decimation, and the Nyquist guard skips all
    // channels but k=-1/0/+1 — keeps the debug-mode test at seconds, not
    // minutes): the shared 511-chip code (1 ms period, so every integration
    // block sees the same code phase) on a carrier at the channel IF plus
    // the true Doppler, in xorshift noise (Task 5's LCG pattern)
    let fs = 2.0e6;
    let n = (0.1 * fs) as usize;
    let code = glonass_code();
    let ifhz = f_nom - fc + dopp;
    let mut lcg = 0x9E3779B9u64;
    let mut noise = move || {
        lcg ^= lcg << 13; lcg ^= lcg >> 7; lcg ^= lcg << 17;
        (lcg >> 11) as f64 / (1u64 << 53) as f64 - 0.5
    };
    let amp = 0.12f64; // strong: ~58 dB-Hz per component variance
    let sig: Vec<Complex<f32>> = (0..n)
        .map(|i| {
            let t = i as f64 / fs;
            let chip = code[(t * CHIP_RATE) as usize % code.len()] as f64;
            let ph = 2.0 * std::f64::consts::PI * ifhz * t;
            Complex::new((amp * chip * ph.cos() + noise()) as f32,
                         (amp * chip * ph.sin() + noise()) as f32)
        })
        .collect();
    let obs = glonass_observations(&sig, fs, fc, t_abs, 0.0, &sats, rx, 0);
    let glo: Vec<&Observation> =
        obs.iter().filter(|o| o.source == Constellation::Glonass).collect();
    assert_eq!(glo.len(), 1, "exactly one channel must attribute: {glo:?}");
    assert!(glo[0].sat_id.contains("2544"), "attributed to {}", glo[0].sat_id);
    assert!((glo[0].value - (f_nom + dopp)).abs() < 1000.0,
            "value {} vs truth {}", glo[0].value, f_nom + dopp);
    assert_eq!(glo[0].aux, f_nom);
    assert_eq!(glo[0].sigma, 500.0);
    assert_eq!(glo[0].capture_id, 0);
}
