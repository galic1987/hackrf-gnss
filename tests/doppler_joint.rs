//! Multi-capture joint solve: two short single-satellite arcs, each with its
//! own clock error, must refuse alone and converge together — with both
//! per-capture clock errors recovered independently.

use hackrf_gnss::gps::{geodetic_to_ecef, load_tle_named, predict_doppler_el_f, GpsSat};
use hackrf_gnss::iridium::geo::{solve_fix, Obs};
use std::fs;
use std::path::PathBuf;

const RX_LAT: f64 = 40.65;
const RX_LON: f64 = -73.80;

fn fx(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(name)
}

fn err_km(fix_lat: f64, fix_lon: f64) -> f64 {
    let a = geodetic_to_ecef(fix_lat, fix_lon, 0.0);
    let b = geodetic_to_ecef(RX_LAT, RX_LON, 0.0);
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// A short arc: 4 measurements, 20 s apart, from `sat` starting at the first
/// time after `t_from` where it is above 25 deg, synthesized through the
/// validated Doppler path at the given clock error (plus deterministic noise).
fn short_arc<'a>(
    sat: &'a GpsSat,
    t_from: f64,
    e_true: f64,
    noise_hz: f64,
    cap: usize,
    seed: u64,
) -> (Vec<Obs<'a>>, f64) {
    arc(sat, t_from, e_true, noise_hz, cap, seed, 4)
}

/// Same, with `count` observations spanning the pass.
#[allow(clippy::too_many_arguments)]
fn arc<'a>(
    sat: &'a GpsSat,
    t_from: f64,
    e_true: f64,
    noise_hz: f64,
    cap: usize,
    seed: u64,
    count: usize,
) -> (Vec<Obs<'a>>, f64) {
    let rx = geodetic_to_ecef(RX_LAT, RX_LON, 0.0);
    let f_nom = 1626.270833e6;
    for m in 0..(24 * 60) {
        let t0 = t_from + m as f64 * 60.0;
        let Some((_, el)) = predict_doppler_el_f(sat, rx, t0 + 30.0, f_nom) else { continue };
        if el < 25.0 {
            continue;
        }
        let mut state = seed;
        let mut out = Vec::new();
        for k in 0..count {
            let t = t0 + k as f64 * 20.0;
            let (fd, _) = predict_doppler_el_f(sat, rx, t, f_nom).unwrap();
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let nz = ((state >> 33) as f64 / (1u64 << 31) as f64) - 1.0;
            out.push(Obs {
                t,
                f_meas: (f_nom + fd) * (1.0 + e_true) + noise_hz * nz,
                f_nom,
                sat,
                conf: 90,
                w_scale: 1.0,
                cap,
            });
        }
        return (out, t0);
    }
    panic!("no pass found for {}", sat.name);
}

#[test]
fn two_short_arcs_converge_jointly_where_each_refuses_alone() {
    let text = fs::read_to_string(fx("iridium.tle")).unwrap();
    let sats = load_tle_named(&text);
    let (sa, sb) = (&sats[0], &sats[1]);
    let t_epoch = sats[0].epoch_unix;
    // capture A: virgin radio, -26 ppm; capture B (a day later): corrected, -0.92 ppm
    let (mut obs_a, ta) = short_arc(sa, t_epoch, -26.0e-6, 20.0, 0, 7);
    let (obs_b, tb) = short_arc(sb, t_epoch + 5400.0, -0.92e-6, 20.0, 1, 11);

    // each alone must refuse (single short arc, degenerate geometry)
    assert!(solve_fix(&obs_a, RX_LAT + 1.0, RX_LON - 1.0, &[0.0], None, None).is_err(), "arc A alone solved");
    assert!(solve_fix(&obs_b, RX_LAT + 1.0, RX_LON - 1.0, &[0.0], None, None).is_err(), "arc B alone solved");

    obs_a.extend(obs_b);
    let fix = solve_fix(&obs_a, RX_LAT + 1.0, RX_LON - 1.0, &[0.0, 0.0], None, None)
        .expect("joint solve should converge");
    let err = err_km(fix.lat_deg, fix.lon_deg);
    eprintln!(
        "joint: err {:.2} km, clocks [{:+.2}, {:+.2}] ppm (true -26.00, -0.92), rms {:.1} Hz, sigma {:.2} km, arcs at t+{:.0}s/{:.0}s",
        err, fix.clock_ppm[0], fix.clock_ppm[1], fix.rms_hz, fix.sigma_km, ta - t_epoch, tb - t_epoch
    );
    assert!(err < 5.0, "position error {err} km");
    assert!((fix.clock_ppm[0] + 26.0).abs() < 0.5, "cap 0 clock {} ppm", fix.clock_ppm[0]);
    assert!((fix.clock_ppm[1] + 0.92).abs() < 0.5, "cap 1 clock {} ppm", fix.clock_ppm[1]);
}

/// The bias state for `sat_name` in a fix, looked up by name.
fn bias_of(fix: &hackrf_gnss::iridium::geo::Fix, name: &str) -> f64 {
    fix.per_sat
        .iter()
        .find(|(n, _, _, _, _)| n == name)
        .map(|(_, _, _, b, _)| *b)
        .unwrap_or(f64::NAN)
}

#[test]
fn an_injected_per_sat_bias_is_absorbed_not_fit_into_position() {
    // Capture B holds TWO satellites so its clock e_1 is pinned by the
    // unbiased one; the +1.2 kHz injected on the other must land in its bias
    // state, not in the position or the clock.
    let text = fs::read_to_string(fx("iridium.tle")).unwrap();
    let sats = load_tle_named(&text);
    let (sa, sb, sc) = (&sats[0], &sats[1], &sats[2]);
    let t_epoch = sats[0].epoch_unix;
    let (mut obs_a, _) = short_arc(sa, t_epoch, -26.0e-6, 20.0, 0, 7);
    let (mut obs_b, _) = short_arc(sb, t_epoch + 5400.0, -0.92e-6, 20.0, 1, 11);
    let (obs_c, _) = short_arc(sc, t_epoch + 10800.0, -0.92e-6, 20.0, 1, 13);
    for o in &mut obs_b {
        o.f_meas += 1200.0; // satellite B's TLE is "wrong": constant along-track offset
    }
    let n_total = obs_a.len() + obs_b.len() + obs_c.len();
    obs_a.extend(obs_b);
    obs_a.extend(obs_c);

    let plain = solve_fix(&obs_a, RX_LAT + 1.0, RX_LON - 1.0, &[0.0, 0.0], None, None);
    match &plain {
        Ok(f) => eprintln!("plain on biased data: rms {:.0} Hz, n_used {} of {}", f.rms_hz, f.n_used, n_total),
        Err(m) => eprintln!("plain on biased data refused: {m}"),
    }

    let fix = solve_fix(&obs_a, RX_LAT + 1.0, RX_LON - 1.0, &[0.0, 0.0], Some(2.0e3), None)
        .expect("biased solve should converge");
    let err = err_km(fix.lat_deg, fix.lon_deg);
    let (b_a, b_b, b_c) = (bias_of(&fix, &sa.name), bias_of(&fix, &sb.name), bias_of(&fix, &sc.name));
    eprintln!(
        "with biases: err {:.2} km, clocks [{:+.2}, {:+.2}] ppm, biases [{:+.0}, {:+.0}, {:+.0}] Hz, rms {:.1} Hz, sigma {:.2} km, n {}",
        err, fix.clock_ppm[0], fix.clock_ppm[1], b_a, b_b, b_c, fix.rms_hz, fix.sigma_km, fix.n_used
    );
    assert!(err < 5.0, "position error {err} km");
    assert!((fix.clock_ppm[0] + 26.0).abs() < 0.5, "cap 0 clock {}", fix.clock_ppm[0]);
    assert!(b_a.abs() < 400.0, "single-sat capture A bias {b_a} Hz");
    // The ridge splits a shared component 50/50 between the clock and the
    // biases (minimum prior-norm), so e_1 and the b's individually are NOT
    // identifiable — the per-satellite totals e_1*f_nom + b_s and the
    // differential bias are. Those must be right.
    let f_nom = 1626.270833e6;
    let tot = |clk_ppm: f64, b: f64| clk_ppm * 1e-6 * f_nom + b;
    let true_e1_hz = -0.92e-6 * f_nom;
    let (tot_b, tot_c) = (tot(fix.clock_ppm[1], b_b), tot(fix.clock_ppm[1], b_c));
    eprintln!("per-sat totals: B {:+.0} Hz (true {:+.0}), C {:+.0} Hz (true {:+.0})",
              tot_b, true_e1_hz + 1200.0, tot_c, true_e1_hz);
    assert!((tot_b - (true_e1_hz + 1200.0)).abs() < 200.0, "sat B total {tot_b} Hz");
    assert!((tot_c - true_e1_hz).abs() < 200.0, "sat C total {tot_c} Hz");
    assert!((b_b - b_c - 1200.0).abs() < 300.0, "differential bias {} Hz", b_b - b_c);
    assert_eq!(fix.n_used, n_total, "no observation should need screening");
    // the plain solve must cope visibly worse: refuse, drop bursts, or noisier
    let worse = match &plain {
        Err(_) => true,
        Ok(f) => f.rms_hz > 5.0 * fix.rms_hz || f.n_used < n_total - 1,
    };
    assert!(worse, "plain solve coped as well as the biased one — the bias state is not earning its keep");
}

#[test]
fn biases_without_injection_stay_near_zero_and_do_not_degrade() {
    let text = fs::read_to_string(fx("iridium.tle")).unwrap();
    let sats = load_tle_named(&text);
    let (sa, sb) = (&sats[0], &sats[1]);
    let t_epoch = sats[0].epoch_unix;
    let (mut obs_a, _) = short_arc(sa, t_epoch, -26.0e-6, 20.0, 0, 7);
    let (obs_b, _) = short_arc(sb, t_epoch + 5400.0, -0.92e-6, 20.0, 1, 11);
    obs_a.extend(obs_b);
    let fix = solve_fix(&obs_a, RX_LAT + 1.0, RX_LON - 1.0, &[0.0, 0.0], Some(2.0e3), None)
        .expect("should converge");
    let err = err_km(fix.lat_deg, fix.lon_deg);
    let b_a = bias_of(&fix, &sa.name);
    let b_b = bias_of(&fix, &sb.name);
    eprintln!("no-injection: err {:.2} km, biases [{:+.0}, {:+.0}] Hz, sigma {:.2} km",
              err, b_a, b_b, fix.sigma_km);
    assert!(err < 5.0, "position error {err} km (unbiased solve: 4.62 km)");
    assert!(b_a.abs() < 400.0 && b_b.abs() < 400.0, "biases [{b_a}, {b_b}] Hz should stay ~0");
}

#[test]
fn a_single_sat_capture_with_bias_does_not_blow_up() {
    // e and b are nearly collinear here (one capture, one satellite): the
    // ridge must keep the normal matrix invertible and the sigma guard must
    // still refuse the fix — no panic, no garbage coordinates
    let text = fs::read_to_string(fx("iridium.tle")).unwrap();
    let sats = load_tle_named(&text);
    let (obs, _) = short_arc(&sats[0], sats[0].epoch_unix, -26.0e-6, 20.0, 0, 7);
    match solve_fix(&obs, RX_LAT + 1.0, RX_LON - 1.0, &[0.0], Some(2.0e3), None) {
        Err(m) => eprintln!("single-sat biased case refused cleanly: {m}"),
        Ok(fix) => panic!("single-sat single-capture must stay untrusted, got ({:+.3}, {:+.3}) sigma {:.1} km",
                          fix.lat_deg, fix.lon_deg, fix.sigma_km),
    }
}

fn rate_of(fix: &hackrf_gnss::iridium::geo::Fix, name: &str) -> f64 {
    fix.per_sat
        .iter()
        .find(|(n, _, _, _, _)| n == name)
        .map(|(_, _, _, _, r)| *r)
        .unwrap_or(f64::NAN)
}

#[test]
fn an_injected_drift_is_absorbed_by_the_rate_state() {
    // The shape measured on the real evening capture: a constant bias PLUS an
    // ~80 Hz/s drift on one satellite (TLE rate error). A constant bias alone
    // cannot absorb it; the rate state must.
    let text = fs::read_to_string(fx("iridium.tle")).unwrap();
    let sats = load_tle_named(&text);
    let (sa, sb, sc) = (&sats[0], &sats[1], &sats[2]);
    let t_epoch = sats[0].epoch_unix;
    let (mut obs_a, _) = arc(sa, t_epoch, -26.0e-6, 20.0, 0, 7, 20);
    let (mut obs_b, _) = arc(sb, t_epoch + 5400.0, -0.92e-6, 20.0, 1, 11, 20);
    let (obs_c, _) = arc(sc, t_epoch + 10800.0, -0.92e-6, 20.0, 1, 13, 20);
    let t_ref_b = obs_b.iter().map(|o| o.t).sum::<f64>() / obs_b.len() as f64;
    for o in &mut obs_b {
        o.f_meas += 1200.0 + 80.0 * (o.t - t_ref_b);
    }
    obs_a.extend(obs_b);
    obs_a.extend(obs_c);

    // constant biases only: the drift has nowhere to go
    let bias_only = solve_fix(&obs_a, RX_LAT + 1.0, RX_LON - 1.0, &[0.0, 0.0], Some(2.0e3), None);
    match &bias_only {
        Ok(f) => eprintln!("bias-only on drifting data: rms {:.0} Hz, err {:.2} km", f.rms_hz, err_km(f.lat_deg, f.lon_deg)),
        Err(m) => eprintln!("bias-only on drifting data refused: {m}"),
    }

    let fix = solve_fix(&obs_a, RX_LAT + 1.0, RX_LON - 1.0, &[0.0, 0.0], Some(2.0e3), Some(100.0))
        .expect("drift solve should converge");
    let err = err_km(fix.lat_deg, fix.lon_deg);
    let r_b = rate_of(&fix, &sb.name);
    let r_c = rate_of(&fix, &sc.name);
    eprintln!(
        "with rate: err {:.2} km, clocks [{:+.2}, {:+.2}] ppm, drift B {:+.1} C {:+.1} Hz/s, rms {:.1} Hz, sigma {:.2} km",
        err, fix.clock_ppm[0], fix.clock_ppm[1], r_b, r_c, fix.rms_hz, fix.sigma_km
    );
    assert!(err < 5.0, "position error {err} km");
    assert!((r_b - 80.0).abs() < 40.0, "drift B {r_b} Hz/s (true +80)");
    assert!(r_c.abs() < 40.0, "drift C {r_c} Hz/s (true 0)");
    let bias_only_err = bias_only.as_ref().map(|f| err_km(f.lat_deg, f.lon_deg)).unwrap_or(f64::MAX);
    let bias_only_rms = bias_only.as_ref().map(|f| f.rms_hz).unwrap_or(f64::MAX);
    assert!(bias_only_err > 2.0 * err || bias_only_rms > 5.0 * fix.rms_hz,
            "bias-only coped too well: err {bias_only_err} rms {bias_only_rms}");
}
