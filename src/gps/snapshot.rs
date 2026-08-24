//! Coarse-time GPS snapshot positioning. A snapshot capture yields only a
//! SUB-millisecond code phase per satellite (0..1 C/A period) plus Doppler — not
//! a full pseudorange. Given broadcast ephemeris, an approximate position
//! (within ~150 km) and time (within ~1 min), this resolves the integer
//! millisecond ambiguity by the reference-satellite method (range DIFFERENCES
//! are insensitive to the common receiver-clock offset), reconstructs
//! pseudoranges, and hands them to `pvt::solve`.
//!
//! `pvt::solve` works in km and applies no Earth-rotation (Sagnac) correction,
//! so this layer rotates each satellite into the reception frame and converts to
//! km when building the measurements. Ported from `validation/satcatch_gps_fix.py`.

use std::collections::HashMap;

use super::broadcast::{sat_clock, sat_pos_ecef, BrdcEph, C_LIGHT, EARTH_RATE};
use super::ephemeris::geodetic_to_ecef;
use super::pvt::{self, Fix, Meas};

const MS_M: f64 = C_LIGHT / 1000.0; // metres of range per millisecond of light

/// One satellite observation from acquisition.
#[derive(Clone, Copy)]
pub struct Obs {
    pub prn: u8,
    pub code_phase: f64, // chips, 0..1023
    pub doppler: f64,    // Hz (carried through; not used by the position solve)
}

/// Satellite ECEF (metres) at transmit time, Sagnac-rotated into the reception
/// frame, plus its clock correction (s) and geometric range (m) to `rx_m`.
fn sat_at_txtime(e: &BrdcEph, tow: f64, rx_m: [f64; 3]) -> ([f64; 3], f64, f64) {
    sat_at_txtime_impl(e, tow, rx_m)
}

/// Public wrapper for diagnostics tooling (debug_fix example).
pub fn sat_at_txtime_pub(e: &BrdcEph, tow: f64, rx_m: [f64; 3]) -> ([f64; 3], f64, f64) {
    sat_at_txtime_impl(e, tow, rx_m)
}

fn sat_at_txtime_impl(e: &BrdcEph, tow: f64, rx_m: [f64; 3]) -> ([f64; 3], f64, f64) {    let mut tau = 0.075;
    let mut s = [0.0f64; 3];
    for _ in 0..2 {
        let s0 = sat_pos_ecef(e, tow - tau);
        let th = EARTH_RATE * tau;
        let (ct, st) = (th.cos(), th.sin());
        s = [s0[0] * ct + s0[1] * st, -s0[0] * st + s0[1] * ct, s0[2]];
        let d = [s[0] - rx_m[0], s[1] - rx_m[1], s[2] - rx_m[2]];
        tau = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() / C_LIGHT;
    }
    let dt = sat_clock(e, tow - tau);
    let d = [s[0] - rx_m[0], s[1] - rx_m[1], s[2] - rx_m[2]];
    let rng = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    (s, dt, rng)
}

fn solve_one(
    obs: &[Obs],
    ephs: &HashMap<u8, BrdcEph>,
    approx_lla: [f64; 3],
    tow: f64,
    sign: f64,
) -> Option<Fix> {
    let prns: Vec<u8> = obs.iter().map(|o| o.prn).filter(|p| ephs.contains_key(p)).collect();
    if prns.len() < 4 {
        return None;
    }
    // sub-ms code phase in the receiver frame, chosen sign convention
    let phi = |prn: u8| -> f64 {
        let o = obs.iter().find(|o| o.prn == prn).unwrap();
        (sign * o.code_phase / 1023.0).rem_euclid(1.0)
    };
    // approx receiver, metres
    let g = geodetic_to_ecef(approx_lla[0], approx_lla[1], approx_lla[2] / 1000.0);
    let mut rx = [g[0] * 1000.0, g[1] * 1000.0, g[2] * 1000.0];
    let mut fix: Option<Fix> = None;

    for _ in 0..5 {
        let mut sat_m = Vec::with_capacity(prns.len());
        let mut dtsv = Vec::with_capacity(prns.len());
        let mut predms = Vec::with_capacity(prns.len());
        for &prn in &prns {
            let (s, dt, rng) = sat_at_txtime(&ephs[&prn], tow, rx);
            sat_m.push(s);
            dtsv.push(dt);
            predms.push((rng / C_LIGHT) * 1000.0 - dt * 1000.0);
        }
        let phi0 = phi(prns[0]);
        let pred0 = predms[0];
        let meas: Vec<Meas> = (0..prns.len())
            .map(|k| {
                let ph = phi(prns[k]);
                // integer ms from range differences vs the reference sat
                let n = ((predms[k] - pred0) - (ph - phi0)).round();
                let rho_m = (n + ph) * MS_M; // arbitrary common offset -> absorbed by clock
                let corrected_m = rho_m + dtsv[k] * C_LIGHT; // remove sat clock
                Meas {
                    sat: [sat_m[k][0] / 1000.0, sat_m[k][1] / 1000.0, sat_m[k][2] / 1000.0],
                    pseudorange: corrected_m / 1000.0,
                    clock_free: false,
                }
            })
            .collect();
        let guess = [rx[0] / 1000.0, rx[1] / 1000.0, rx[2] / 1000.0];
        let f = pvt::solve(&meas, guess)?;
        let newrx = [f.ecef[0] * 1000.0, f.ecef[1] * 1000.0, f.ecef[2] * 1000.0];
        let step = ((newrx[0] - rx[0]).powi(2) + (newrx[1] - rx[1]).powi(2) + (newrx[2] - rx[2]).powi(2)).sqrt();
        rx = newrx;
        fix = Some(f);
        if step < 0.01 {
            break;
        }
    }
    fix
}

/// Coarse-time snapshot fix. The code-phase sign convention differs between
/// simulators and acquirers, so both are tried and the lower post-fit residual
/// (the self-consistent one) is kept. `approx_lla` = [lat_deg, lon_deg, h_m];
/// `tow` = approximate GPS time-of-week at the START of the capture.
pub fn snapshot_fix(
    obs: &[Obs],
    ephs: &HashMap<u8, BrdcEph>,
    approx_lla: [f64; 3],
    tow: f64,
) -> Option<Fix> {
    let mut best: Option<Fix> = None;
    for &sign in &[1.0f64, -1.0f64] {
        if let Some(f) = solve_one(obs, ephs, approx_lla, tow, sign) {
            if best.as_ref().map_or(true, |b| f.residual_rms_m < b.residual_rms_m) {
                best = Some(f);
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gps::broadcast::sat_clock as _clk;

    // Real broadcast ephemeris (BKG BRDC, DOY 232 2026) as a fixture, so the
    // closed loop has a genuine GPS sky over the test site to solve.
    const BRDC: &str = include_str!("../../tests/fixtures/brdc_gps.rnx");

    fn constellation() -> HashMap<u8, BrdcEph> {
        crate::gps::broadcast::parse_rinex_gps(BRDC)
    }

    fn median_toe(ephs: &HashMap<u8, BrdcEph>) -> f64 {
        let mut toes: Vec<f64> = ephs.values().map(|e| e.toe).collect();
        toes.sort_by(|a, b| a.partial_cmp(b).unwrap());
        toes[toes.len() / 2]
    }

    #[test]
    fn closed_loop_recovers_a_known_site() {
        let ephs = constellation();
        assert!(ephs.len() >= 20, "parsed only {} GPS eph", ephs.len());
        let (lat, lon, h) = (40.65, -73.80, 20.0);
        let tow = median_toe(&ephs);
        let g = geodetic_to_ecef(lat, lon, h / 1000.0);
        let rx_m = [g[0] * 1000.0, g[1] * 1000.0, g[2] * 1000.0];
        let up = {
            let n = (rx_m[0].powi(2) + rx_m[1].powi(2) + rx_m[2].powi(2)).sqrt();
            [rx_m[0] / n, rx_m[1] / n, rx_m[2] / n]
        };
        // synthesize sub-ms code phases (sign convention +1) for sats above 10 deg
        let mut obs = Vec::new();
        for (&prn, e) in ephs.iter() {
            let (s, dt, rng) = sat_at_txtime(e, tow, rx_m);
            let los = [s[0] - rx_m[0], s[1] - rx_m[1], s[2] - rx_m[2]];
            let el = ((los[0] * up[0] + los[1] * up[1] + los[2] * up[2]) / rng).asin().to_degrees();
            if el < 10.0 {
                continue;
            }
            let pr_m = rng - dt * C_LIGHT; // clock-free pseudorange
            let frac = ((pr_m / MS_M) % 1.0 + 1.0) % 1.0;
            obs.push(Obs { prn, code_phase: frac * 1023.0, doppler: 0.0 });
        }
        assert!(obs.len() >= 4, "need >=4 sats above horizon, got {}", obs.len());
        // seed the fix ~80 km off and confirm it snaps back to the true site
        let solved = snapshot_fix(&obs, &ephs, [lat + 0.5, lon - 0.6, 0.0], tow).expect("fix");
        let err_lat = (solved.lat - lat) * 111_000.0;
        let err_lon = (solved.lon - lon) * 111_000.0 * lat.to_radians().cos();
        let err = (err_lat * err_lat + err_lon * err_lon).sqrt();
        assert!(err < 50.0, "closed-loop error {err:.1} m (expected < 50 m)");
    }

    #[test]
    fn refuses_under_four_sats() {
        let ephs = constellation();
        let obs = vec![
            Obs { prn: 1, code_phase: 100.0, doppler: 0.0 },
            Obs { prn: 2, code_phase: 200.0, doppler: 0.0 },
            Obs { prn: 3, code_phase: 300.0, doppler: 0.0 },
        ];
        assert!(snapshot_fix(&obs, &ephs, [40.0, -74.0, 0.0], 345600.0).is_none());
    }

    #[test]
    fn touches_clock_symbol() {
        // keep the sat_clock re-export exercised so the import is meaningful
        let e = BrdcEph { sqrt_a: 5153.6, af0: 1e-4, toc: 0.0, ..Default::default() };
        assert!((_clk(&e, 0.0) - 1e-4).abs() < 1e-9);
    }
}
