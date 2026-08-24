//! Tests for Doppler-based burst attribution (geo::doppler_attribute) and the
//! e-bootstrap wiring: correct-sat attribution, ambiguity rejection, and
//! recovery from a wrong initial clock estimate.

use hackrf_gnss::gps::{geodetic_to_ecef, load_tle_named, predict_doppler_el_f, GpsSat};
use hackrf_gnss::iridium::geo::doppler_attribute;
use hackrf_gnss::iridium::ppm::{median, ppm_from_burst, DecodedBurst};
use std::fs;
use std::path::PathBuf;

const RX_LAT: f64 = 40.65;
const RX_LON: f64 = -73.80;

fn fx(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(name)
}

fn load() -> Vec<GpsSat> {
    load_tle_named(&fs::read_to_string(fx("iridium.tle")).unwrap())
}

fn rx() -> [f64; 3] {
    geodetic_to_ecef(RX_LAT, RX_LON, 0.0)
}

/// Find a time when at least two fixture satellites are well above the
/// horizon; returns (t, [(sat_idx, doppler, f_nom)]).
fn two_sat_window(sats: &[GpsSat]) -> (f64, Vec<(usize, f64, f64)>) {
    let f_nom = 1626.270833e6;
    for h in 0..(24 * 12) {
        let t = sats[0].epoch_unix + h as f64 * 300.0;
        let up: Vec<(usize, f64, f64)> = sats
            .iter()
            .enumerate()
            .filter_map(|(i, s)| {
                predict_doppler_el_f(s, rx(), t, f_nom)
                    .and_then(|(fd, el)| (el > 20.0).then_some((i, fd, f_nom)))
            })
            .collect();
        if up.len() >= 2 {
            return (t, up);
        }
    }
    panic!("no two-satellite window in the fixture TLE");
}

fn burst(t: f64, f_nom: f64, fd: f64, e: f64) -> DecodedBurst {
    DecodedBurst {
        t_epoch: t,
        f_meas_hz: (f_nom + fd) * (1.0 + e),
        f_nom_hz: f_nom,
        confidence: 90,
    }
}

#[test]
fn a_burst_at_a_sats_predicted_offset_attributes_to_it() {
    let sats = load();
    let (t, up) = two_sat_window(&sats);
    let e = -0.92e-6;
    let (si, fd, f_nom) = up[0];
    let b = burst(t, f_nom, fd, e);
    let attrs = doppler_attribute(&[b], &sats, rx(), e, 0.5e-6, 2.5e3);
    assert_eq!(attrs.len(), 1, "should attribute exactly once");
    assert_eq!(attrs[0].sat, sats[si].name);
    assert!(attrs[0].score_hz < 100.0, "score {}", attrs[0].score_hz);
}

#[test]
fn an_ambiguous_burst_stays_unattributed() {
    let sats = load();
    let (t, up) = two_sat_window(&sats);
    // burst exactly BETWEEN two satellites' predictions: best score is half
    // the gap and the margin to the second is 1x, not 2x -> no attribution
    let (s0, d0, f_nom) = up[0];
    let (_, d1, _) = up[1];
    let mid = (d0 + d1) / 2.0;
    let gap = (d0 - d1).abs();
    let b = DecodedBurst { t_epoch: t, f_meas_hz: f_nom + mid, f_nom_hz: f_nom, confidence: 90 };
    let attrs = doppler_attribute(&[b], &sats, rx(), 0.0, 0.5e-6, gap.max(3e3));
    if gap > 100.0 {
        assert!(attrs.is_empty(),
                "midpoint burst attributed to {} despite a {:.0} Hz twin",
                attrs.first().map(|a| a.sat.as_str()).unwrap_or("-"), gap);
        // and the midpoint must not attribute to either of the pair even at a
        // generous tolerance, because the margin rule rejects it
        assert!(!attrs.iter().any(|a| a.sat == sats[s0].name));
    }
}

#[test]
fn a_wrong_initial_clock_is_recovered_before_attribution() {
    // The bootstrap path: position-attributed bursts give the first-pass e;
    // with 5 ppm injected error the median recovers it, and attribution at
    // the recovered e then works.
    let sats = load();
    let (t, up) = two_sat_window(&sats);
    let e_true = 5.0e-6;
    // first pass: "position-attributed" bursts at three sats/times
    let mut es = Vec::new();
    for &(_si, fd, f_nom) in up.iter().take(3) {
        let b = burst(t, f_nom, fd, e_true);
        es.push(ppm_from_burst(b.f_meas_hz, b.f_nom_hz, fd));
    }
    let e0 = median(&es).unwrap() * 1e-6;
    assert!((e0 - e_true).abs() < 0.1e-6, "first-pass e {e0}");
    // second pass: an anonymous burst from another sat attributes correctly
    // at the recovered e, and NOT at e = 0 (5 ppm = ~8 kHz, way past 2.5 kHz)
    let &(si, fd, f_nom) = up.last().unwrap();
    let b = burst(t, f_nom, fd, e_true);
    let at_zero = doppler_attribute(std::slice::from_ref(&b), &sats, rx(), 0.0, 0.5e-6, 2.5e3);
    assert!(
        at_zero.iter().all(|a| a.sat != sats[si].name) ,
        "at e=0 the burst must not land on the true satellite"
    );
    let at_est = doppler_attribute(&[b], &sats, rx(), e0, 0.5e-6, 2.5e3);
    assert_eq!(at_est.len(), 1);
    assert_eq!(at_est[0].sat, sats[si].name);
}

#[test]
fn undecoded_bursts_attribute_with_the_lowest_weight() {
    use hackrf_gnss::iridium::geo::{W_DOPPLER_DECODED, W_UNDECODED};
    let sats = load();
    let (t, up) = two_sat_window(&sats);
    let e = -0.92e-6;
    let (si, fd, f_nom) = up[0];
    // a detected-only burst (confidence 0 sentinel) at sat 0's offset
    let b0 = DecodedBurst { confidence: 0, ..burst(t, f_nom, fd, e) };
    let attrs = doppler_attribute(&[b0], &sats, rx(), e, 0.5e-6, 2.5e3);
    assert_eq!(attrs.len(), 1);
    assert_eq!(attrs[0].sat, sats[si].name);
    assert_eq!(attrs[0].w_scale, W_UNDECODED);
    // the same burst decoded (real confidence) gets the decoded weight
    let b1 = burst(t, f_nom, fd, e);
    let attrs = doppler_attribute(&[b1], &sats, rx(), e, 0.5e-6, 2.5e3);
    assert_eq!(attrs[0].w_scale, W_DOPPLER_DECODED);
    assert!(W_UNDECODED < W_DOPPLER_DECODED);
}

#[test]
fn an_off_frequency_burst_is_rejected_regardless_of_class() {
    let sats = load();
    let (t, up) = two_sat_window(&sats);
    let (_, fd, f_nom) = up[0];
    // 40 kHz past every prediction: no satellite can own this
    let b = DecodedBurst { t_epoch: t, f_meas_hz: (f_nom + fd) + 40e3, f_nom_hz: f_nom, confidence: 0 };
    assert!(doppler_attribute(&[b], &sats, rx(), 0.0, 0.5e-6, 2.5e3).is_empty());
}

#[test]
fn the_weight_reaches_the_formal_sigma() {
    // same obs set, but the undecoded-class obs at 0.25 vs 1.0 weight must
    // move the formal uncertainty: higher weight -> tighter sigma
    use hackrf_gnss::iridium::geo::{solve_fix, Obs};
    let sats = load();
    let (t, up) = two_sat_window(&sats);
    let f_nom = 1626.270833e6;
    let e = -0.92e-6;
    let rx_ecef = rx();
    let mut base: Vec<(usize, f64, f64)> = Vec::new();
    // one obs per visible sat every 20 s over 8 min at both sats' times
    for &(si, _, _) in &up {
        for k in 0..24 {
            let tk = t + k as f64 * 20.0;
            if let Some((fd, el)) = predict_doppler_el_f(&sats[si], rx_ecef, tk, f_nom) {
                if el > 15.0 {
                    base.push((si, tk, (f_nom + fd) * (1.0 + e)));
                }
            }
        }
    }
    let build = |w: f64| -> Vec<Obs> {
        base.iter()
            .map(|&(si, tk, fm)| Obs { t: tk, f_meas: fm, f_nom, sat: &sats[si], conf: 90, w_scale: w, cap: 0 })
            .collect()
    };
    let lo = solve_fix(&build(0.25), RX_LAT + 1.0, RX_LON - 1.0, &[0.0], None, None).unwrap();
    let hi = solve_fix(&build(1.0), RX_LAT + 1.0, RX_LON - 1.0, &[0.0], None, None).unwrap();
    eprintln!("sigma w=0.25: {:.3} km, w=1.0: {:.3} km", lo.sigma_km, hi.sigma_km);
    assert!(hi.sigma_km < lo.sigma_km, "weight did not reach sigma");
    // noiseless data: both land on truth regardless of weight
    assert!(lo.lat_deg - RX_LAT < 0.01 && hi.lat_deg - RX_LAT < 0.01);
}
