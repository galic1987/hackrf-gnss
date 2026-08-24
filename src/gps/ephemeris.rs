//! Ephemeris cross-check: does a confirmed pass's OWN PRN actually reproduce the
//! observed Doppler track? Ported from `validation/report.py:_ephemeris_check`
//! (SGP4 propagation + range-rate Doppler) and `validation/identify.py`
//! (TEME->ECEF, velocity frame conversion).
//!
//! The statistical verdict only tests exchangeability; a slowly drifting spur
//! can pass it. The decisive test is physical: the same PRN, propagated from
//! published elements for this station, must be above the horizon at those
//! moments and produce a Doppler ramp of the observed rate. This does not change
//! the `confirmed` decision — it annotates it.

use sgp4::chrono::NaiveDateTime;
use sgp4::{Constants, Elements, MinutesSinceEpoch};

const C: f64 = 299_792_458.0;
pub const F_L1: f64 = 1575.42e6;
const OMEGA_E: f64 = 7.2921159e-5; // Earth rotation rate, rad/s

#[derive(Debug, Clone, serde::Serialize)]
pub struct EphemMatch {
    /// Some(true)=own PRN backs it, Some(false)=contradicted, None=cannot check
    pub matched: Option<bool>,
    pub norad: Option<String>,
    pub rate_obs_hz_s: Option<f64>,
    pub rate_pred_hz_s: Option<f64>,
    pub resid_hz: Option<f64>,
    pub lo_offset_hz: Option<f64>,
    pub el_deg: Option<f64>,
    pub sign: Option<i8>,
    pub reason: Option<String>,
}

impl EphemMatch {
    fn none(reason: &str) -> Self {
        EphemMatch {
            matched: None,
            norad: None,
            rate_obs_hz_s: None,
            rate_pred_hz_s: None,
            resid_hz: None,
            lo_offset_hz: None,
            el_deg: None,
            sign: None,
            reason: Some(reason.to_string()),
        }
    }
}

/// A GPS satellite with its PRN parsed from the TLE title `(PRN NN)`.
pub struct GpsSat {
    pub prn: u16,
    pub name: String,
    pub epoch_unix: f64,
    pub constants: Constants,
}

fn epoch_unix(dt: &NaiveDateTime) -> f64 {
    dt.and_utc().timestamp_millis() as f64 / 1000.0
}

/// Parse a GPS TLE file (name / line1 / line2 triples). Only satellites whose
/// title contains `(PRN NN)` are kept, keyed by that PRN.
pub fn load_tle(text: &str) -> Vec<GpsSat> {
    let lines: Vec<&str> = text.lines().map(|l| l.trim_end()).filter(|l| !l.is_empty()).collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i + 3 <= lines.len() {
        let (name, l1, l2) = (lines[i], lines[i + 1], lines[i + 2]);
        if !l1.starts_with("1 ") || !l2.starts_with("2 ") {
            i += 1;
            continue;
        }
        i += 3;
        let prn = match parse_prn(name) {
            Some(p) => p,
            None => continue,
        };
        let elements = match Elements::from_tle(
            Some(name.to_string()),
            l1.as_bytes(),
            l2.as_bytes(),
        ) {
            Ok(e) => e,
            Err(_) => continue,
        };
        let constants = match Constants::from_elements(&elements) {
            Ok(c) => c,
            Err(_) => continue,
        };
        out.push(GpsSat {
            prn,
            name: name.trim().to_string(),
            epoch_unix: epoch_unix(&elements.datetime),
            constants,
        });
    }
    out
}

/// Parse a TLE file keeping EVERY satellite (not just GPS `(PRN NN)` names).
/// Used for constellations like Iridium whose titles carry no PRN. `prn` is 0.
pub fn load_tle_named(text: &str) -> Vec<GpsSat> {
    let lines: Vec<&str> = text.lines().map(|l| l.trim_end()).filter(|l| !l.is_empty()).collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i + 3 <= lines.len() {
        let (name, l1, l2) = (lines[i], lines[i + 1], lines[i + 2]);
        if !l1.starts_with("1 ") || !l2.starts_with("2 ") {
            i += 1;
            continue;
        }
        i += 3;
        let elements =
            match Elements::from_tle(Some(name.to_string()), l1.as_bytes(), l2.as_bytes()) {
                Ok(e) => e,
                Err(_) => continue,
            };
        let constants = match Constants::from_elements(&elements) {
            Ok(c) => c,
            Err(_) => continue,
        };
        out.push(GpsSat {
            prn: 0,
            name: name.trim().to_string(),
            epoch_unix: epoch_unix(&elements.datetime),
            constants,
        });
    }
    out
}

fn parse_prn(name: &str) -> Option<u16> {
    let idx = name.find("PRN")?;
    let rest = &name[idx + 3..];
    let digits: String = rest.chars().skip_while(|c| !c.is_ascii_digit()).take_while(|c| c.is_ascii_digit()).collect();
    digits.parse().ok()
}

fn gmst(jd: f64) -> f64 {
    let t = (jd - 2451545.0) / 36525.0;
    let g = 280.46061837 + 360.98564736629 * (jd - 2451545.0) + 0.000387933 * t * t
        - t * t * t / 38710000.0;
    g.rem_euclid(360.0).to_radians()
}

fn teme_to_ecef(p: [f64; 3], jd: f64) -> [f64; 3] {
    let th = gmst(jd);
    let (s, c) = th.sin_cos();
    [c * p[0] + s * p[1], -s * p[0] + c * p[1], p[2]]
}

fn teme_vel_to_ecef(v: [f64; 3], pos_ecef: [f64; 3], jd: f64) -> [f64; 3] {
    let vr = teme_to_ecef(v, jd);
    // ECEF is a rotating frame: subtract omega x r
    [
        vr[0] - (-OMEGA_E * pos_ecef[1]),
        vr[1] - (OMEGA_E * pos_ecef[0]),
        vr[2],
    ]
}

pub fn geodetic_to_ecef(lat_deg: f64, lon_deg: f64, h_km: f64) -> [f64; 3] {
    let a = 6378.137;
    let f = 1.0 / 298.257223563;
    let e2 = f * (2.0 - f);
    let (la, lo) = (lat_deg.to_radians(), lon_deg.to_radians());
    let n = a / (1.0 - e2 * la.sin() * la.sin()).sqrt();
    [
        (n + h_km) * la.cos() * lo.cos(),
        (n + h_km) * la.cos() * lo.sin(),
        (n * (1.0 - e2) + h_km) * la.sin(),
    ]
}

fn jd_of(unix: f64) -> f64 {
    unix / 86400.0 + 2440587.5
}

/// ECEF position (km) of `sat` at unix time `t`. None if SGP4 fails.
pub fn sat_ecef(sat: &GpsSat, t: f64) -> Option<[f64; 3]> {
    sat_ecef_pv(sat, t).map(|pv| pv.0)
}

/// ECEF position (km) AND velocity (km/s, Earth-rotation removed) of `sat` at
/// unix time `t`. None if SGP4 fails. Used by the Doppler geolocation solver,
/// which needs the satellite velocity, not just a scalar Doppler.
pub fn sat_ecef_pv(sat: &GpsSat, t: f64) -> Option<([f64; 3], [f64; 3])> {
    let t_min = (t - sat.epoch_unix) / 60.0;
    let pred = sat.constants.propagate(MinutesSinceEpoch(t_min)).ok()?;
    let jd = jd_of(t);
    let pos = teme_to_ecef(pred.position, jd);
    let vel = teme_vel_to_ecef(pred.velocity, pos, jd);
    Some((pos, vel))
}

/// Predicted GPS L1 Doppler (Hz) and elevation (deg) of `sat` seen from
/// `rx_ecef` at unix time `t`. None if SGP4 fails.
pub fn predict_doppler_el(sat: &GpsSat, rx_ecef: [f64; 3], t: f64) -> Option<(f64, f64)> {
    predict_doppler_el_f(sat, rx_ecef, t, F_L1)
}

/// As `predict_doppler_el` but at an arbitrary carrier frequency `f_hz` --
/// the geometry (range rate, elevation) is frequency-independent; only the
/// Hz scaling changes. Used by the Iridium clock-error estimator.
pub fn predict_doppler_el_f(sat: &GpsSat, rx_ecef: [f64; 3], t: f64, f_hz: f64) -> Option<(f64, f64)> {
    let (pos, vel) = sat_ecef_pv(sat, t)?;
    let rel = [pos[0] - rx_ecef[0], pos[1] - rx_ecef[1], pos[2] - rx_ecef[2]];
    let rng = (rel[0] * rel[0] + rel[1] * rel[1] + rel[2] * rel[2]).sqrt();
    let rr = (rel[0] * vel[0] + rel[1] * vel[1] + rel[2] * vel[2]) / rng; // km/s
    let fd = -f_hz * rr * 1000.0 / C; // Hz
    let rn = (rx_ecef[0].powi(2) + rx_ecef[1].powi(2) + rx_ecef[2].powi(2)).sqrt();
    let up = [rx_ecef[0] / rn, rx_ecef[1] / rn, rx_ecef[2] / rn];
    let el = ((rel[0] * up[0] + rel[1] * up[1] + rel[2] * up[2]) / rng)
        .clamp(-1.0, 1.0)
        .asin()
        .to_degrees();
    Some((fd, el))
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    if v.is_empty() { 0.0 } else { v[v.len() / 2] }
}

fn polyfit_slope(t: &[f64], d: &[f64]) -> f64 {
    let n = t.len() as f64;
    let mt = t.iter().sum::<f64>() / n;
    let md = d.iter().sum::<f64>() / n;
    let (mut sxx, mut sxy) = (0.0, 0.0);
    for i in 0..t.len() {
        let dt = t[i] - mt;
        sxx += dt * dt;
        sxy += dt * (d[i] - md);
    }
    if sxx > 1e-12 { sxy / sxx } else { 0.0 }
}

/// Annotate one confirmed pass. `crossings` = (unix_t, observed_doppler_hz) of
/// the pass, `sats` the loaded TLE set, `rx` the station ECEF.
pub fn ephemeris_match(
    prn: u16,
    crossings: &[(f64, f64)],
    sats: &[GpsSat],
    rx: [f64; 3],
    min_el: f64,
    tol_rate: f64,
    tol_resid: f64,
) -> EphemMatch {
    let sat = match sats.iter().find(|s| s.prn == prn) {
        Some(s) => s,
        None => return EphemMatch::none("PRN not in TLE set"),
    };
    if crossings.len() < 3 {
        return EphemMatch::none("too few crossings");
    }
    let te: Vec<f64> = crossings.iter().map(|c| c.0).collect();
    let dobs: Vec<f64> = crossings.iter().map(|c| c.1).collect();
    let mut fdp = Vec::with_capacity(te.len());
    let mut elp = Vec::with_capacity(te.len());
    for &t in &te {
        match predict_doppler_el(sat, rx, t) {
            Some((fd, el)) => {
                fdp.push(fd);
                elp.push(el);
            }
            None => {
                return EphemMatch {
                    matched: Some(false),
                    norad: Some(sat.name.clone()),
                    reason: Some("SGP4 failed".into()),
                    ..EphemMatch::none("SGP4 failed")
                }
            }
        }
    }
    let el_max = elp.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if el_max <= min_el {
        return EphemMatch {
            matched: Some(false),
            norad: Some(sat.name.clone()),
            el_deg: Some((el_max * 10.0).round() / 10.0),
            reason: Some("PRN below the horizon at these times".into()),
            ..EphemMatch::none("below horizon")
        };
    }
    let t0 = te[0];
    let tt: Vec<f64> = te.iter().map(|t| t - t0).collect();
    let rate_obs = polyfit_slope(&tt, &dobs);
    // sign convention not pinned: try both, remove a constant offset, keep the
    // sign that best fits the rate. The rate is the discriminating quantity.
    let mut best: Option<(f64, f64, f64, i8)> = None; // (resid, rate_pred, off, sign)
    for sgn in [1.0f64, -1.0] {
        let pr: Vec<f64> = fdp.iter().map(|f| sgn * f).collect();
        let rate_pred = polyfit_slope(&tt, &pr);
        let diff: Vec<f64> = dobs.iter().zip(&pr).map(|(o, p)| o - p).collect();
        let off = median(&mut diff.clone());
        let mut resid: Vec<f64> = diff.iter().map(|x| (x - off).abs()).collect();
        let resid_med = median(&mut resid);
        let better = match &best {
            None => true,
            Some((_, rp, _, _)) => (rate_pred - rate_obs).abs() < (rp - rate_obs).abs(),
        };
        if better {
            best = Some((resid_med, rate_pred, off, sgn as i8));
        }
    }
    let (resid, rate_pred, off, sign) = best.unwrap();
    let matched = (rate_pred - rate_obs).abs() < tol_rate && resid < tol_resid;
    EphemMatch {
        matched: Some(matched),
        norad: Some(sat.name.clone()),
        rate_obs_hz_s: Some((rate_obs * 1000.0).round() / 1000.0),
        rate_pred_hz_s: Some((rate_pred * 1000.0).round() / 1000.0),
        resid_hz: Some(resid.round()),
        lo_offset_hz: Some(off.round()),
        el_deg: Some((el_max * 10.0).round() / 10.0),
        sign: Some(sign),
        reason: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_prn_from_title() {
        assert_eq!(parse_prn("GPS BIIR-5  (PRN 22)"), Some(22));
        assert_eq!(parse_prn("GPS BIII-7  (PRN 01)"), Some(1));
        assert_eq!(parse_prn("SOME JUNK"), None);
    }

    #[test]
    fn geodetic_matches_known_point() {
        // New York-ish site; magnitude near Earth radius
        let e = geodetic_to_ecef(40.65, -73.80, 0.0);
        let r = (e[0] * e[0] + e[1] * e[1] + e[2] * e[2]).sqrt();
        assert!((r - 6369.0).abs() < 10.0, "|ecef|={r}");
    }
}
