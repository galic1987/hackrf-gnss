//! Tests for the Doppler geolocation solver: synthetic end-to-end with a real
//! TLE (noisy and noiseless), outlier rejection, and the degenerate-geometry
//! failure mode. Synthesis goes through the INDEPENDENTLY validated
//! predict_doppler_el_f path, so a frame/sign error in the solver's own
//! predict() shows up as position error rather than cancelling out.

use hackrf_gnss::gps::{geodetic_to_ecef, load_tle_named, predict_doppler_el_f, GpsSat};
use hackrf_gnss::iridium::geo::{solve_fix, Obs};
use std::fs;
use std::path::PathBuf;

const RX_LAT: f64 = 40.65;
const RX_LON: f64 = -73.80;
const E_TRUE: f64 = -0.92e-6; // the station's disciplined clock error, ppm-scale

fn fx(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(name)
}

/// Deterministic pseudo-noise in [-1, 1] (LCG; reproducibility over realism).
struct Noise(u64);
impl Noise {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 33) as f64 / (1u64 << 31) as f64) - 1.0
    }
}

/// Build a synthetic 8-minute observation set: every satellite in the fixture
/// TLE that is above 15 deg during the window contributes a measurement every
/// 20 s, synthesized through the validated Doppler path plus uniform noise of
/// `noise_hz` amplitude.
fn synth_pass(sats: &[GpsSat], noise_hz: f64) -> (Vec<(usize, f64, f64, f64)>, f64) {
    // find an 8-min window with >= 2 satellites up, starting within a day of
    // the TLE epoch
    let rx = geodetic_to_ecef(RX_LAT, RX_LON, 0.0);
    let f_nom = 1626.270833e6;
    for h in 0..(24 * 6) {
        let t0 = sats[0].epoch_unix + h as f64 * 600.0;
        let mut obs = Vec::new();
        for (si, s) in sats.iter().enumerate() {
            for k in 0..24 {
                let t = t0 + k as f64 * 20.0;
                let Some((fd, el)) = predict_doppler_el_f(s, rx, t, f_nom) else { continue };
                if el > 15.0 {
                    obs.push((si, t, fd, f_nom));
                }
            }
        }
        if obs.len() >= 12 && obs.iter().map(|o| o.0).collect::<std::collections::HashSet<_>>().len() >= 2 {
            let mut nz = Noise(0xDEADBEEF);
            let out = obs
                .iter()
                .map(|&(si, t, fd, f)| {
                    let f_meas = (f + fd) * (1.0 + E_TRUE) + noise_hz * nz.next();
                    (si, t, f_meas, f)
                })
                .collect();
            return (out, t0);
        }
    }
    panic!("no 8-min window with 2+ visible satellites in the fixture TLE");
}

fn to_obs<'a>(synth: &[(usize, f64, f64, f64)], sats: &'a [GpsSat]) -> Vec<Obs<'a>> {
    synth
        .iter()
        .map(|&(si, t, f_meas, f_nom)| Obs { t, f_meas, f_nom, sat: &sats[si], conf: 90, w_scale: 1.0, cap: 0 })
        .collect()
}

fn err_km(fix_lat: f64, fix_lon: f64) -> f64 {
    let a = geodetic_to_ecef(fix_lat, fix_lon, 0.0);
    let b = geodetic_to_ecef(RX_LAT, RX_LON, 0.0);
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

#[test]
fn synthetic_pass_recovers_position_and_clock() {
    let sats = load_tle_named(&fs::read_to_string(fx("iridium.tle")).unwrap());
    let (synth, _t0) = synth_pass(&sats, 20.0); // +-20 Hz, realistic burst noise
    eprintln!("synthetic pass: {} observations", synth.len());
    let obs = to_obs(&synth, &sats);
    // initial guess ~110 km off, clock guess 0
    let fix = solve_fix(&obs, RX_LAT + 1.0, RX_LON - 1.0, &[0.0], None, None).expect("should converge");
    let err = err_km(fix.lat_deg, fix.lon_deg);
    eprintln!(
        "noisy: err {:.2} km, clock {:+.3} ppm (true {:+.3}), rms {:.1} Hz, sigma {:.2} km, n {}",
        err, fix.clock_ppm[0], E_TRUE * 1e6, fix.rms_hz, fix.sigma_km, fix.n_used
    );
    assert!(err < 5.0, "position error {err} km");
    assert!((fix.clock_ppm[0] - E_TRUE * 1e6).abs() < 0.5, "clock {} ppm", fix.clock_ppm[0]);
    assert!(fix.rms_hz < 60.0, "rms {} Hz", fix.rms_hz);
}

#[test]
fn noiseless_synthesis_is_recovered_near_exactly() {
    let sats = load_tle_named(&fs::read_to_string(fx("iridium.tle")).unwrap());
    let (synth, _) = synth_pass(&sats, 0.0);
    let obs = to_obs(&synth, &sats);
    let fix = solve_fix(&obs, RX_LAT + 1.0, RX_LON - 1.0, &[0.0], None, None).expect("should converge");
    let err = err_km(fix.lat_deg, fix.lon_deg);
    eprintln!("noiseless: err {:.3} m, clock {:+.5} ppm, rms {:.2} Hz",
              err * 1000.0, fix.clock_ppm[0], fix.rms_hz);
    assert!(err < 0.2, "position error {err} km");
    assert!((fix.clock_ppm[0] - E_TRUE * 1e6).abs() < 0.05, "clock {} ppm", fix.clock_ppm[0]);
}

#[test]
fn a_5khz_outlier_is_rejected() {
    let sats = load_tle_named(&fs::read_to_string(fx("iridium.tle")).unwrap());
    let (mut synth, _) = synth_pass(&sats, 20.0);
    synth[3].2 += 5000.0; // one wildly wrong carrier
    let obs = to_obs(&synth, &sats);
    let fix = solve_fix(&obs, RX_LAT + 1.0, RX_LON - 1.0, &[0.0], None, None).expect("should converge");
    let err = err_km(fix.lat_deg, fix.lon_deg);
    eprintln!("outlier: err {:.2} km, n_used {} of {}", err, fix.n_used, synth.len());
    assert_eq!(fix.n_used, synth.len() - 1, "the outlier should be the only drop");
    assert!(err < 5.0, "position error {err} km");
}

#[test]
fn a_single_short_arc_fails_cleanly() {
    let sats = load_tle_named(&fs::read_to_string(fx("iridium.tle")).unwrap());
    // 4 measurements over 60 s of one satellite: geometry cannot constrain
    // (lat, lon, e) — must fail with a message, not garbage coordinates
    let (synth, _) = synth_pass(&sats, 20.0);
    let one_sat = synth[0].0;
    let short: Vec<(usize, f64, f64, f64)> =
        synth.iter().copied().filter(|o| o.0 == one_sat).take(4).collect();
    let obs = to_obs(&short, &sats);
    match solve_fix(&obs, RX_LAT + 1.0, RX_LON - 1.0, &[0.0], None, None) {
        Err(m) => eprintln!("degenerate case rejected: {m}"),
        Ok(fix) => panic!(
            "a 60-second single-satellite arc must not yield a trustworthy fix, got \
             ({:+.3}, {:+.3}) sigma {:.1} km",
            fix.lat_deg, fix.lon_deg, fix.sigma_km
        ),
    }
}

#[test]
fn too_few_observations_is_an_error_not_a_fix() {
    let sats = load_tle_named(&fs::read_to_string(fx("iridium.tle")).unwrap());
    let (synth, _) = synth_pass(&sats, 0.0);
    let obs = to_obs(&synth[..3], &sats);
    assert!(solve_fix(&obs, RX_LAT, RX_LON, &[0.0], None, None).is_err());
}

#[test]
fn single_capture_gauge_fix_puts_the_common_mode_in_the_clock() {
    // With free per-satellite bias states the (clock, mean-bias) direction is
    // degenerate: f_pred = f(1+e) + b_s is invariant under e -> e + m/f,
    // b_s -> b_s - m. The ridge prior is too weak to pin it in practice (the
    // 00:12 solve printed +18.4 ppm of clock with all biases ~ -31.8 kHz).
    // The gauge fix must pin mean(bias) = 0 so the clock owns the common mode.
    let sats = load_tle_named(&fs::read_to_string(fx("iridium.tle")).unwrap());
    let (synth, _t0) = synth_pass(&sats, 20.0);
    // inject per-satellite biases with a deliberate 1.5 kHz common mode
    let bias_of = |si: usize| 1500.0 + ((si * 7 % 5) as f64 - 2.0) * 300.0;
    let mut mean_inj = 0.0;
    let mut seen: Vec<usize> = Vec::new();
    let obs: Vec<Obs> = synth
        .iter()
        .map(|&(si, t, f_meas, f_nom)| {
            if !seen.contains(&si) {
                seen.push(si);
                mean_inj += bias_of(si);
            }
            let b = bias_of(si);
            Obs { t, f_meas: f_meas + b, f_nom, sat: &sats[si], conf: 90, w_scale: 1.0, cap: 0 }
        })
        .collect();
    mean_inj /= seen.len() as f64;
    let f_bar = obs.iter().map(|o| o.f_nom).sum::<f64>() / obs.len() as f64;
    let fix = solve_fix(&obs, RX_LAT + 1.0, RX_LON - 1.0, &[0.0], Some(2.0e3), None)
        .expect("should converge");
    let mean_bias = fix.per_sat.iter().map(|p| p.3).sum::<f64>() / fix.per_sat.len() as f64;
    let want_clock = E_TRUE * 1e6 + mean_inj / f_bar * 1e6;
    eprintln!(
        "gauge: clock {:+.3} ppm (want {:+.3}), mean bias {:+.2} Hz, err {:.2} km",
        fix.clock_ppm[0],
        want_clock,
        mean_bias,
        err_km(fix.lat_deg, fix.lon_deg)
    );
    assert!(mean_bias.abs() < 1.0, "mean bias {mean_bias} Hz — gauge not pinned");
    assert!(
        (fix.clock_ppm[0] - want_clock).abs() < 0.2,
        "clock {} ppm, want {} (common mode not in the clock)",
        fix.clock_ppm[0],
        want_clock
    );
    assert!(err_km(fix.lat_deg, fix.lon_deg) < 5.0, "gauge fix must not hurt the position");
}
