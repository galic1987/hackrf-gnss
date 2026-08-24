//! Receiver positioning from Iridium Doppler tracks — the receiver's location is
//! SOLVED, not assumed. Each track gives a time of closest approach and a
//! measured Doppler RATE (Hz/s); only a narrow band of ground positions can
//! explain all the tracks against the known Iridium orbits at once. A coarse
//! lat/lon grid search (then a refinement) finds it. Ported from
//! `validation/identify.py`.
//!
//! The Doppler rate is carrier-frequency scaled: `gps::predict_doppler_el`
//! returns Doppler at L1, so we finite-difference it for the rate and scale by
//! the Iridium/L1 carrier ratio (Doppler is exactly linear in carrier).

use crate::gps::ephemeris::{geodetic_to_ecef, predict_doppler_el, GpsSat};

const F_L1: f64 = 1_575.42e6;
const F_IRIDIUM: f64 = 1_626.1e6;
const SCALE: f64 = F_IRIDIUM / F_L1;

/// One observed Doppler track: unix time of closest approach + measured rate.
#[derive(Clone, Copy)]
pub struct Track {
    pub t_unix: f64,
    pub rate_hz_s: f64,
}

#[derive(Debug, Clone)]
pub struct LocateResult {
    pub lat: f64,
    pub lon: f64,
    pub hits: usize,
    pub n_tracks: usize,
    pub mean_resid_hz_s: f64,
}

/// Predicted Iridium Doppler rate (Hz/s) and elevation (deg) for one satellite
/// at an observer and time. `None` if SGP4 fails at any of the three epochs.
fn doppler_rate(sat: &GpsSat, rx_ecef: [f64; 3], t: f64) -> Option<(f64, f64)> {
    let dt = 1.0;
    let (fdp, _) = predict_doppler_el(sat, rx_ecef, t + dt)?;
    let (fdm, _) = predict_doppler_el(sat, rx_ecef, t - dt)?;
    let (_, el) = predict_doppler_el(sat, rx_ecef, t)?;
    let fdd_l1 = (fdp - fdm) / (2.0 * dt);
    Some((fdd_l1 * SCALE, el))
}

/// Score a candidate location: for each track, find the visible satellite whose
/// predicted rate best matches, and count it a hit within `tol`. Returns
/// (hits, summed residual over the hits).
pub fn score_location(
    lat: f64,
    lon: f64,
    tracks: &[Track],
    sats: &[GpsSat],
    min_el: f64,
    tol: f64,
) -> (usize, f64) {
    let rx = geodetic_to_ecef(lat, lon, 0.0);
    let mut hits = 0;
    let mut resid = 0.0;
    for tr in tracks {
        let mut best: Option<f64> = None;
        for sat in sats {
            if let Some((fdd, el)) = doppler_rate(sat, rx, tr.t_unix) {
                if el < min_el {
                    continue;
                }
                let d = (fdd - tr.rate_hz_s).abs();
                if best.map_or(true, |b| d < b) {
                    best = Some(d);
                }
            }
        }
        if let Some(b) = best {
            if b < tol {
                hits += 1;
                resid += b;
            }
        }
    }
    (hits, resid)
}

fn better(a: (usize, f64), b: (usize, f64)) -> bool {
    // more matches first, then lower mean residual
    a.0 > b.0 || (a.0 == b.0 && a.1 / (a.0.max(1) as f64) < b.1 / (b.0.max(1) as f64))
}

/// Coarse grid search over North America (then a local refinement) for the
/// receiver location that best explains the Doppler tracks. `None` if no tracks.
pub fn locate(tracks: &[Track], sats: &[GpsSat], min_el: f64, tol: f64) -> Option<LocateResult> {
    if tracks.is_empty() || sats.is_empty() {
        return None;
    }
    let mut best: Option<(usize, f64, f64, f64)> = None; // (hits, resid, lat, lon)
    let mut lat = 20.0;
    while lat <= 56.0 {
        let mut lon = -130.0;
        while lon <= -58.0 {
            let (h, r) = score_location(lat, lon, tracks, sats, min_el, tol);
            if best.map_or(true, |(bh, br, _, _)| better((h, r), (bh, br))) {
                best = Some((h, r, lat, lon));
            }
            lon += 2.0;
        }
        lat += 2.0;
    }
    let (mut bh, mut br, mut bl, mut bo) = best?;
    for &step in &[1.0, 0.5, 0.25, 0.1] {
        let (cl, co) = (bl, bo);
        let mut la = cl - 2.0 * step;
        while la <= cl + 2.0 * step + 1e-9 {
            let mut lo = co - 2.0 * step;
            while lo <= co + 2.0 * step + 1e-9 {
                let (h, r) = score_location(la, lo, tracks, sats, min_el, tol);
                if better((h, r), (bh, br)) {
                    bh = h;
                    br = r;
                    bl = la;
                    bo = lo;
                }
                lo += step;
            }
            la += step;
        }
    }
    Some(LocateResult {
        lat: bl,
        lon: bo,
        hits: bh,
        n_tracks: tracks.len(),
        mean_resid_hz_s: br / (bh.max(1) as f64),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gps::ephemeris::load_tle_named;

    const IRIDIUM_TLE: &str = include_str!("../../tests/fixtures/iridium.tle");

    // Synthesize Doppler tracks for a KNOWN site from the real Iridium orbits,
    // then confirm the grid search recovers that site. This is the closed-loop
    // analogue of the live positioning that reaches ~30-60 km on real data.
    #[test]
    fn recovers_a_known_site_from_doppler_tracks() {
        let sats = load_tle_named(IRIDIUM_TLE);
        assert!(sats.len() >= 6, "parsed {} Iridium sats", sats.len());
        let (true_lat, true_lon) = (40.65, -73.80);
        let rx = geodetic_to_ecef(true_lat, true_lon, 0.0);

        // anchor to the TLE epoch so SGP4 is not propagated far from it
        let t0 = sats[0].epoch_unix;
        // for each sat, find the moment of steepest Doppler while it is well up,
        // and record (time, true rate) as a track
        let mut tracks = Vec::new();
        for sat in &sats {
            let mut best: Option<(f64, f64)> = None; // (|rate|, (t,rate))
            let mut steep = (0.0f64, 0.0f64);
            let mut t = t0;
            while t < t0 + 12.0 * 3600.0 {
                if let Some((rate, el)) = doppler_rate(sat, rx, t) {
                    if el > 15.0 && rate.abs() > best.map_or(0.0, |(m, _)| m) {
                        best = Some((rate.abs(), 0.0));
                        steep = (t, rate);
                    }
                }
                t += 30.0;
            }
            if best.is_some() {
                tracks.push(Track { t_unix: steep.0, rate_hz_s: steep.1 });
            }
        }
        assert!(tracks.len() >= 4, "only {} usable tracks", tracks.len());

        let fix = locate(&tracks, &sats, 8.0, 25.0).expect("locate");
        let err_km = ((fix.lat - true_lat) * 111.0).hypot(
            (fix.lon - true_lon) * 111.0 * true_lat.to_radians().cos(),
        );
        println!(
            "iridium locate: {:.2} N {:.2} E  {}/{} tracks  resid {:.1} Hz/s  err {:.0} km",
            fix.lat, fix.lon, fix.hits, fix.n_tracks, fix.mean_resid_hz_s, err_km
        );
        assert!(err_km < 120.0, "iridium locate error {err_km:.0} km");
    }

    #[test]
    fn empty_inputs_return_none() {
        let sats = load_tle_named(IRIDIUM_TLE);
        assert!(locate(&[], &sats, 8.0, 25.0).is_none());
        assert!(locate(&[Track { t_unix: 0.0, rate_hz_s: 0.0 }], &[], 8.0, 25.0).is_none());
    }
}
