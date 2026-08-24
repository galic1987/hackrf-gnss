//! Pass-window prediction for multi-satellite Doppler geolocation captures.
//!
//! The joint fix needs 2+ satellites above a decent elevation at once, and —
//! the part that actually constrains position — with their lines of sight
//! well separated in azimuth (the Doppler slopes must DISAGREE). This module
//! scans the TLE set forward in time and reports the windows where that
//! holds, ranked by duration x azimuth spread x elevation.
//!
//! Timing accuracy: a TLE a few days old shifts pass TIMES by seconds to tens
//! of seconds (along-track error), which is negligible for capture planning;
//! report windows with that caveat, not as second-precise truth.

use crate::gps::{sat_ecef, GpsSat};

/// Elevation/azimuth of `sat_ecef` (km) seen from a station at geodetic
/// (lat_deg, lon_deg) with ECEF position `rx_ecef` (km). Azimuth is degrees
/// clockwise from north, [0, 360).
pub fn el_az(sat_ecef: [f64; 3], rx_ecef: [f64; 3], lat_deg: f64, lon_deg: f64) -> (f64, f64) {
    let rel = [
        sat_ecef[0] - rx_ecef[0],
        sat_ecef[1] - rx_ecef[1],
        sat_ecef[2] - rx_ecef[2],
    ];
    let (la, lo) = (lat_deg.to_radians(), lon_deg.to_radians());
    let east = [-lo.sin(), lo.cos(), 0.0];
    let north = [-la.sin() * lo.cos(), -la.sin() * lo.sin(), la.cos()];
    let up = [la.cos() * lo.cos(), la.cos() * lo.sin(), la.sin()];
    let e = rel[0] * east[0] + rel[1] * east[1] + rel[2] * east[2];
    let n = rel[0] * north[0] + rel[1] * north[1] + rel[2] * north[2];
    let u = rel[0] * up[0] + rel[1] * up[1] + rel[2] * up[2];
    let rng = (e * e + n * n + u * u).sqrt();
    let el = (u / rng).clamp(-1.0, 1.0).asin().to_degrees();
    let az = e.atan2(n).to_degrees().rem_euclid(360.0);
    (el, az)
}

/// Circular difference of two azimuths, degrees in [0, 180].
pub fn az_sep(a: f64, b: f64) -> f64 {
    let d = (a - b).abs().rem_euclid(360.0);
    d.min(360.0 - d)
}

/// One satellite's track through a window.
#[derive(Debug, Clone)]
pub struct WindowSat {
    pub name: String,
    pub el_max: f64,
    pub az_min: f64,
    pub az_max: f64,
}

/// A window with >= 2 satellites simultaneously above the elevation mask.
#[derive(Debug, Clone)]
pub struct Window {
    pub t_start: f64,
    pub t_end: f64,
    pub sats: Vec<WindowSat>,
    /// max over the window of the largest pairwise azimuth separation among
    /// visible satellites — the Doppler geometry metric; >60 deg is good
    pub max_az_spread: f64,
    /// mean elevation of the visible satellites at the best-spread instant
    pub mean_el: f64,
    /// duration_min x max_az_spread x mean_el
    pub quality: f64,
}

/// Merge runs separated by gaps shorter than `max_gap_s`. Runs are
/// (start, end) pairs, sorted by start.
pub fn merge_runs(runs: &[(f64, f64)], max_gap_s: f64) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = Vec::new();
    for &(s, e) in runs {
        if let Some(last) = out.last_mut() {
            if s - last.1 <= max_gap_s {
                last.1 = last.1.max(e);
                continue;
            }
        }
        out.push((s, e));
    }
    out
}

/// Scan `hours` ahead of `t_start` in `step_s` steps; return windows with
/// >= 2 satellites above `min_el` (gaps < 2 min merged), best quality first.
pub fn find_windows(
    sats: &[GpsSat],
    rx: [f64; 3],
    lat_deg: f64,
    lon_deg: f64,
    t_start: f64,
    hours: f64,
    step_s: f64,
    min_el: f64,
) -> Vec<Window> {
    let nsteps = (hours * 3600.0 / step_s) as usize;
    // per-step visible set: (sat index, el, az)
    let mut good: Vec<Option<Vec<(usize, f64, f64)>>> = Vec::with_capacity(nsteps);
    for k in 0..nsteps {
        let t = t_start + k as f64 * step_s;
        let vis: Vec<(usize, f64, f64)> = sats
            .iter()
            .enumerate()
            .filter_map(|(i, s)| {
                let p = sat_ecef(s, t)?;
                let (el, az) = el_az(p, rx, lat_deg, lon_deg);
                (el >= min_el).then_some((i, el, az))
            })
            .collect();
        good.push((vis.len() >= 2).then_some(vis));
    }
    // runs of good steps, merged across < 2 min dips
    let mut runs: Vec<(f64, f64)> = Vec::new();
    let mut k = 0;
    while k < nsteps {
        if good[k].is_some() {
            let mut j = k;
            while j + 1 < nsteps && good[j + 1].is_some() {
                j += 1;
            }
            runs.push((t_start + k as f64 * step_s, t_start + (j + 1) as f64 * step_s));
            k = j + 1;
        } else {
            k += 1;
        }
    }
    let merged = merge_runs(&runs, 120.0);
    let mut out = Vec::new();
    for (ws, we) in merged {
        let k0 = ((ws - t_start) / step_s) as usize;
        let k1 = ((we - t_start) / step_s) as usize;
        let mut by_sat: std::collections::HashMap<usize, WindowSat> = std::collections::HashMap::new();
        let (mut best_spread, mut best_mean_el) = (0.0f64, 0.0f64);
        for k in k0..=k1.min(nsteps - 1) {
            let Some(vis) = &good[k] else { continue };
            for &(i, el, az) in vis {
                let w = by_sat.entry(i).or_insert(WindowSat {
                    name: sats[i].name.clone(),
                    el_max: el,
                    az_min: az,
                    az_max: az,
                });
                w.el_max = w.el_max.max(el);
                w.az_min = w.az_min.min(az);
                w.az_max = w.az_max.max(az);
            }
            let mut spread = 0.0f64;
            for a in 0..vis.len() {
                for b in (a + 1)..vis.len() {
                    spread = spread.max(az_sep(vis[a].2, vis[b].2));
                }
            }
            if spread > best_spread {
                best_spread = spread;
                best_mean_el = vis.iter().map(|v| v.1).sum::<f64>() / vis.len() as f64;
            }
        }
        let dur_min = (we - ws) / 60.0;
        let mean_el_max = if by_sat.is_empty() {
            0.0
        } else {
            by_sat.values().map(|s| s.el_max).sum::<f64>() / by_sat.len() as f64
        };
        let mut sats: Vec<WindowSat> = by_sat.into_values().collect();
        sats.sort_by(|a, b| a.name.cmp(&b.name));
        out.push(Window {
            t_start: ws,
            t_end: we,
            sats,
            max_az_spread: best_spread,
            mean_el: best_mean_el,
            quality: dur_min * best_spread * mean_el_max,
        });
    }
    out.sort_by(|a, b| b.quality.partial_cmp(&a.quality).unwrap_or(std::cmp::Ordering::Equal));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gps::load_tle_named;

    #[test]
    fn a_dense_constellation_offers_a_two_sat_window_in_24h() {
        // sanity on the propagation + scan path: if NO pair of Iridiums is
        // jointly above 25 deg in a day, something is broken, not quiet
        let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/iridium.tle")).unwrap();
        let sats = load_tle_named(&text);
        let rx = crate::gps::geodetic_to_ecef(40.65, -73.80, 0.0);
        let ws = find_windows(&sats, rx, 40.65, -73.80, sats[0].epoch_unix, 24.0, 60.0, 25.0);
        assert!(!ws.is_empty(), "no 2-satellite window in 24 h");
        let w = &ws[0];
        assert!(w.t_end > w.t_start);
        assert!(w.sats.len() >= 2);
        assert!(w.max_az_spread > 0.0);
        eprintln!("best window: {:.0} min, {} sats, spread {:.0} deg, quality {:.0}",
                  (w.t_end - w.t_start) / 60.0, w.sats.len(), w.max_az_spread, w.quality);
    }

    #[test]
    fn run_merging_bridges_short_gaps_only() {
        // two runs 90 s apart merge; a 3 min gap does not
        let runs = [(100.0, 200.0), (290.0, 400.0), (580.0, 700.0)];
        let m = merge_runs(&runs, 120.0);
        assert_eq!(m, vec![(100.0, 400.0), (580.0, 700.0)]);
        assert_eq!(merge_runs(&[], 120.0), vec![]);
        assert_eq!(merge_runs(&[(5.0, 6.0)], 120.0), vec![(5.0, 6.0)]);
    }

    #[test]
    fn azimuth_separation_is_circular() {
        assert!((az_sep(350.0, 10.0) - 20.0).abs() < 1e-9);
        assert!((az_sep(90.0, 270.0) - 180.0).abs() < 1e-9);
        assert!(az_sep(45.0, 45.0) == 0.0);
    }
}
