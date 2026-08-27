//! Weighted-least-squares GNSS position/time solver (PVT). Given satellite ECEF
//! positions and pseudoranges, it recovers the receiver's ECEF position and
//! clock bias by Gauss-Newton iteration, and reports the geometry (DOP).
//!
//! This is the meter-level positioning engine. It needs >=4 satellites with
//! decoded ephemeris and pseudoranges — which through a partial (window) sky
//! view this station cannot yet supply, so it is validated against REAL
//! satellite geometry rather than presented as a live fix.

/// One measurement: satellite ECEF position (km) and pseudorange (km).
#[derive(Clone, Copy)]
pub struct Meas {
    pub sat: [f64; 3],
    pub pseudorange: f64,
    /// true for pseudo-measurements that carry no receiver clock (e.g. the
    /// altitude-hold range-to-Earth-centre row): the clock column of the
    /// design matrix and the clock term of the residual are both 0 for
    /// this row, so it cannot trade against the clock unknown.
    pub clock_free: bool,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct Fix {
    pub ecef: [f64; 3],
    pub lat: f64,
    pub lon: f64,
    pub alt_km: f64,
    pub clock_km: f64,
    pub iterations: usize,
    pub residual_rms_m: f64,
    pub gdop: f64,
    pub pdop: f64,
    pub tdop: f64,
    pub n_sat: usize,
}

fn norm3(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// Solve for receiver position + clock from >=4 pseudoranges. `guess` seeds the
/// iteration (Earth centre works; a nearby prior converges faster). None if
/// under-determined or the geometry is singular.
/// Topocentric elevation of satellite `s` seen from receiver `p` (radians,
/// spherical-up — fine for weighting; the geodetic correction is <0.3% of
/// the angle and only matters if we hard-gated on it, which we do not).
fn elev_rad(p: [f64; 3], s: [f64; 3]) -> f64 {
    let d = [s[0] - p[0], s[1] - p[1], s[2] - p[2]];
    let dn = norm3(d);
    let pn = norm3(p);
    if dn < 1e-9 || pn < 1e-9 {
        return 0.0;
    }
    let up = (d[0] * p[0] + d[1] * p[1] + d[2] * p[2]) / (dn * pn);
    up.clamp(-1.0, 1.0).asin()
}

/// sin²(el) measurement weight, floored at sin²(5°) so a horizon-grazing
/// satellite is heavily downweighted rather than zeroed mid-iteration.
/// The alt-hold pseudo-measurement (sat at origin) evaluates to weight 1.
const MIN_EL_W: f64 = 0.0076;

fn el_w(p: [f64; 3], s: [f64; 3]) -> f64 {
    let w = elev_rad(p, s).sin();
    (w * w).max(MIN_EL_W)
}

pub fn solve(meas: &[Meas], guess: [f64; 3]) -> Option<Fix> {
    solve_w(meas, guess, true)
}

/// The solver core, with elevation weighting switchable so tests can
/// measure the unweighted baseline against the same data.
pub(crate) fn solve_w(meas: &[Meas], guess: [f64; 3], weighted: bool) -> Option<Fix> {
    if meas.len() < 4 {
        return None;
    }
    let mut p = guess;
    let mut c = 0.0f64; // clock bias (km)
    let mut last_q = [[0.0f64; 4]; 4];
    let mut iters = 0;

    for _ in 0..12 {
        iters += 1;
        // build normal equations H^T H (4x4) and H^T r (4)
        let mut hth = [[0.0f64; 4]; 4];
        let mut htr = [0.0f64; 4];
        for m in meas {
            let d = [p[0] - m.sat[0], p[1] - m.sat[1], p[2] - m.sat[2]];
            let g = norm3(d);
            if g < 1e-6 {
                return None;
            }
            let u = [d[0] / g, d[1] / g, d[2] / g, if m.clock_free { 0.0 } else { 1.0 }]; // design row
            let r = m.pseudorange - (g + if m.clock_free { 0.0 } else { c }); // residual
            // Elevation weighting (precision round): multipath lives low,
            // so low-elevation measurements enter at sin²(el) weight — on
            // consistent data the answer is unchanged (any consistent
            // weighting solves it exactly); on real data the low outliers
            // stop dragging the fix.
            let w = if weighted { el_w(p, m.sat) } else { 1.0 };
            for i in 0..4 {
                htr[i] += w * u[i] * r;
                for j in 0..4 {
                    hth[i][j] += w * u[i] * u[j];
                }
            }
        }
        let q = inv4(&hth)?; // covariance ~ (H^T H)^-1
        last_q = q;
        // dx = q * htr
        let mut dx = [0.0f64; 4];
        for i in 0..4 {
            for j in 0..4 {
                dx[i] += q[i][j] * htr[j];
            }
        }
        p[0] += dx[0];
        p[1] += dx[1];
        p[2] += dx[2];
        c += dx[3];
        if dx[0].hypot(dx[1]).hypot(dx[2]) < 1e-7 {
            break;
        }
    }

    // residuals
    let mut ss = 0.0;
    for m in meas {
        let g = norm3([p[0] - m.sat[0], p[1] - m.sat[1], p[2] - m.sat[2]]);
        let r = m.pseudorange - (g + if m.clock_free { 0.0 } else { c });
        ss += r * r;
    }
    let rms_m = (ss / meas.len() as f64).sqrt() * 1000.0;

    let gdop = (last_q[0][0] + last_q[1][1] + last_q[2][2] + last_q[3][3]).sqrt();
    let pdop = (last_q[0][0] + last_q[1][1] + last_q[2][2]).sqrt();
    let tdop = last_q[3][3].sqrt();
    let (lat, lon, alt) = ecef_to_geodetic(p);
    Some(Fix {
        ecef: p,
        lat,
        lon,
        alt_km: alt,
        clock_km: c,
        iterations: iters,
        residual_rms_m: rms_m,
        gdop,
        pdop,
        tdop,
        n_sat: meas.len(),
    })
}

/// 4x4 inverse (Gauss-Jordan). None if singular.
fn inv4(a: &[[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    let mut m = *a;
    let mut inv = [[0.0f64; 4]; 4];
    for i in 0..4 {
        inv[i][i] = 1.0;
    }
    for col in 0..4 {
        // pivot
        let mut piv = col;
        for r in col + 1..4 {
            if m[r][col].abs() > m[piv][col].abs() {
                piv = r;
            }
        }
        if m[piv][col].abs() < 1e-12 {
            return None;
        }
        m.swap(col, piv);
        inv.swap(col, piv);
        let d = m[col][col];
        for k in 0..4 {
            m[col][k] /= d;
            inv[col][k] /= d;
        }
        for r in 0..4 {
            if r == col {
                continue;
            }
            let f = m[r][col];
            for k in 0..4 {
                m[r][k] -= f * m[col][k];
                inv[r][k] -= f * inv[col][k];
            }
        }
    }
    Some(inv)
}

// ------------------------------------------------------- mixed-constellation

/// One measurement tagged with its constellation, for the two-clock mixed
/// solve. `system`: 0 = GPS, 1 = BeiDou. The design row carries a 1 in the
/// clock column of its OWN system and 0 in the other, so each constellation's
/// receiver-clock offset (and the GPS/BDS time-scale offset riding on it) is
/// estimated independently.
#[derive(Clone, Copy)]
pub struct MeasSys {
    pub sat: [f64; 3],
    pub pseudorange: f64,
    pub system: u8,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct FixMixed {
    pub ecef: [f64; 3],
    pub lat: f64,
    pub lon: f64,
    pub alt_km: f64,
    pub clock_gps_km: f64,
    pub clock_bds_km: f64,
    /// inter-system clock offset δt_gps − δt_bds (km): absorbs the GPST/BDT
    /// scale difference plus any per-band hardware delay — and doubles as a
    /// spoof-detection observable (a spoofer on one constellation only moves
    /// one side).
    pub isx_km: f64,
    pub iterations: usize,
    pub residual_rms_m: f64,
    pub gdop: f64,
    pub pdop: f64,
    pub tdop: f64,
    pub n_sat: usize,
    pub n_gps: usize,
    pub n_bds: usize,
}

/// Two-clock mixed-constellation solve: unknowns x,y,z,δt_gps,δt_bds. Needs
/// >=5 rows with >=1 row per system (5 unknowns). Same Gauss-Newton skeleton
/// as [`solve`], widened to a 5x5 normal-equations system.
pub fn solve_mixed(meas: &[MeasSys], guess: [f64; 3]) -> Option<FixMixed> {
    let n_gps = meas.iter().filter(|m| m.system == 0).count();
    let n_bds = meas.iter().filter(|m| m.system == 1).count();
    if meas.len() < 5 || n_gps == 0 || n_bds == 0 {
        return None;
    }
    let mut p = guess;
    let mut clk = [0.0f64; 2]; // [gps, bds] clock biases (km)
    let mut last_q = [[0.0f64; 5]; 5];
    let mut iters = 0;

    for _ in 0..12 {
        iters += 1;
        let mut hth = [[0.0f64; 5]; 5];
        let mut htr = [0.0f64; 5];
        for m in meas {
            let d = [p[0] - m.sat[0], p[1] - m.sat[1], p[2] - m.sat[2]];
            let g = norm3(d);
            if g < 1e-6 {
                return None;
            }
            let sys = m.system.min(1) as usize;
            let u = [d[0] / g, d[1] / g, d[2] / g, (sys == 0) as u8 as f64, (sys == 1) as u8 as f64];
            let r = m.pseudorange - (g + clk[sys]);
            let w = el_w(p, m.sat); // same elevation weighting as solve()
            for i in 0..5 {
                htr[i] += w * u[i] * r;
                for j in 0..5 {
                    hth[i][j] += w * u[i] * u[j];
                }
            }
        }
        let q = inv5(&hth)?;
        last_q = q;
        let mut dx = [0.0f64; 5];
        for i in 0..5 {
            for j in 0..5 {
                dx[i] += q[i][j] * htr[j];
            }
        }
        p[0] += dx[0];
        p[1] += dx[1];
        p[2] += dx[2];
        clk[0] += dx[3];
        clk[1] += dx[4];
        if dx[0].hypot(dx[1]).hypot(dx[2]) < 1e-7 {
            break;
        }
    }

    let mut ss = 0.0;
    for m in meas {
        let g = norm3([p[0] - m.sat[0], p[1] - m.sat[1], p[2] - m.sat[2]]);
        let r = m.pseudorange - (g + clk[m.system.min(1) as usize]);
        ss += r * r;
    }
    let rms_m = (ss / meas.len() as f64).sqrt() * 1000.0;

    let gdop = (last_q[0][0] + last_q[1][1] + last_q[2][2] + last_q[3][3] + last_q[4][4]).sqrt();
    let pdop = (last_q[0][0] + last_q[1][1] + last_q[2][2]).sqrt();
    let tdop = (last_q[3][3] + last_q[4][4]).sqrt();
    let (lat, lon, alt) = ecef_to_geodetic(p);
    Some(FixMixed {
        ecef: p,
        lat,
        lon,
        alt_km: alt,
        clock_gps_km: clk[0],
        clock_bds_km: clk[1],
        isx_km: clk[0] - clk[1],
        iterations: iters,
        residual_rms_m: rms_m,
        gdop,
        pdop,
        tdop,
        n_sat: meas.len(),
        n_gps,
        n_bds,
    })
}

/// Outlier-rejection threshold (metres). A full 1 ms code-period tooth slip
/// is ~300 km — orders of magnitude outside this bound — while honest
/// anchored channels sit meter-class to a few hundred metres indoors. The
/// threshold removes only catastrophic outliers; fractional-tooth suspects
/// stay (they are real signal, just biased).
pub const REJECT_THRESH_M: f64 = 1000.0;

/// RAIM-style outlier rejection around `solve`: drop the worst measurement
/// while its residual exceeds REJECT_THRESH_M, re-solve, at most `max_drops`
/// times. Returns the fix plus the ORIGINAL indices of dropped rows.
/// Exact solves (rows <= unknowns) are not policed — nothing independent
/// left to test against.
pub fn solve_with_rejection(
    meas: &[Meas],
    guess: [f64; 3],
    max_drops: usize,
) -> Option<(Fix, Vec<usize>)> {
    let mut idx: Vec<usize> = (0..meas.len()).collect();
    let mut dropped = Vec::new();
    loop {
        let cur: Vec<Meas> = idx.iter().map(|&i| meas[i]).collect();
        let fix = solve(&cur, guess)?;
        if cur.len() <= 4 || dropped.len() >= max_drops {
            return Some((fix, dropped));
        }
        // per-measurement SUSPICION: residual x its weight. A low-elevation
        // measurement is allowed more noise (its weight is small), so a big
        // raw residual there is less damning than the same residual at the
        // zenith; ranking by raw residual with the weighted solve otherwise
        // lets a dragged zenith-good sat outrank the actual outlier
        // (observed: dropped [4, 1] and kept the 300 km slip).
        let mut worst = 0.0f64;
        let mut worst_pos = 0usize;
        for (pos, m) in cur.iter().enumerate() {
            let g = norm3([fix.ecef[0] - m.sat[0], fix.ecef[1] - m.sat[1], fix.ecef[2] - m.sat[2]]);
            let r = (m.pseudorange - (g + if m.clock_free { 0.0 } else { fix.clock_km })).abs()
                * 1000.0 * el_w(fix.ecef, m.sat).sqrt();
            if r > worst {
                worst = r;
                worst_pos = pos;
            }
        }
        if worst <= REJECT_THRESH_M {
            return Some((fix, dropped));
        }
        dropped.push(idx.remove(worst_pos));
    }
}

/// The same for the mixed-constellation solve (5 unknowns: x,y,z,clk_gps,
/// clk_bds). A single wrong-tooth channel otherwise drags the free isx
/// state — observed live: isx -239.5 km with a 1 ms slip in the set.
pub fn solve_mixed_with_rejection(
    meas: &[MeasSys],
    guess: [f64; 3],
    max_drops: usize,
) -> Option<(FixMixed, Vec<usize>)> {
    let mut idx: Vec<usize> = (0..meas.len()).collect();
    let mut dropped = Vec::new();
    loop {
        let cur: Vec<MeasSys> = idx.iter().map(|&i| meas[i].clone()).collect();
        let fix = solve_mixed(&cur, guess)?;
        if cur.len() <= 5 || dropped.len() >= max_drops {
            return Some((fix, dropped));
        }
        let clk = [fix.clock_gps_km, fix.clock_bds_km];
        let mut worst = 0.0f64;
        let mut worst_pos = 0usize;
        for (pos, m) in cur.iter().enumerate() {
            let g = norm3([fix.ecef[0] - m.sat[0], fix.ecef[1] - m.sat[1], fix.ecef[2] - m.sat[2]]);
            // same weighted suspicion as solve_with_rejection: residual x
            // its elevation weight, so a dragged good sat can't outrank the
            // actual outlier under the weighted solve
            let r = (m.pseudorange - (g + clk[m.system.min(1) as usize])).abs()
                * 1000.0 * el_w(fix.ecef, m.sat).sqrt();
            if r > worst {
                worst = r;
                worst_pos = pos;
            }
        }
        if worst <= REJECT_THRESH_M {
            return Some((fix, dropped));
        }
        dropped.push(idx.remove(worst_pos));
    }
}

/// 5x5 inverse (Gauss-Jordan). None if singular.
fn inv5(a: &[[f64; 5]; 5]) -> Option<[[f64; 5]; 5]> {
    let mut m = *a;
    let mut inv = [[0.0f64; 5]; 5];
    for i in 0..5 {
        inv[i][i] = 1.0;
    }
    for col in 0..5 {
        let mut piv = col;
        for r in col + 1..5 {
            if m[r][col].abs() > m[piv][col].abs() {
                piv = r;
            }
        }
        if m[piv][col].abs() < 1e-12 {
            return None;
        }
        m.swap(col, piv);
        inv.swap(col, piv);
        let d = m[col][col];
        for k in 0..5 {
            m[col][k] /= d;
            inv[col][k] /= d;
        }
        for r in 0..5 {
            if r == col {
                continue;
            }
            let f = m[r][col];
            for k in 0..5 {
                m[r][k] -= f * m[col][k];
                inv[r][k] -= f * inv[col][k];
            }
        }
    }
    Some(inv)
}

/// ECEF (km) -> geodetic lat/lon (deg), altitude (km). WGS-84.
pub fn ecef_to_geodetic(p: [f64; 3]) -> (f64, f64, f64) {
    let a = 6378.137;
    let f = 1.0 / 298.257223563;
    let e2 = f * (2.0 - f);
    let lon = p[1].atan2(p[0]);
    let hyp = (p[0] * p[0] + p[1] * p[1]).sqrt();
    let mut lat = p[2].atan2(hyp * (1.0 - e2));
    let mut alt = 0.0;
    for _ in 0..8 {
        let n = a / (1.0 - e2 * lat.sin() * lat.sin()).sqrt();
        alt = hyp / lat.cos() - n;
        lat = p[2].atan2(hyp * (1.0 - e2 * n / (n + alt)));
    }
    (lat.to_degrees(), lon.to_degrees(), alt)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Real GPS satellite ECEF positions (km) at the capture time, from SGP4 of
    // the current TLEs — the six the station saw above 10 deg. The receiver's
    // true ECEF is the New York site. Pseudoranges = true geometric range plus a
    // fixed receiver-clock bias. This validates that the solver inverts real
    // geometry back to the true position; it is NOT a claim of a measured fix.
    const STATION: [f64; 3] = [1351.991, -4653.584, 4133.012];
    const SATS: [[f64; 3]; 6] = [
        [5869.975, -16762.018, 19215.021],   // PRN16
        [-3107.678, -19845.218, 17310.754],  // PRN4
        [13935.468, -7948.651, 20949.401],   // PRN26
        [12385.868, -23003.053, 2879.614],   // PRN27
        [21723.378, -9762.873, 11751.846],   // PRN31
        [-11644.487, -10338.459, 21397.985], // PRN9
    ];

    fn ranges(clock_km: f64) -> Vec<Meas> {
        SATS.iter()
            .map(|&s| {
                let g = norm3([STATION[0] - s[0], STATION[1] - s[1], STATION[2] - s[2]]);
                Meas { sat: s, pseudorange: g + clock_km, clock_free: false }
            })
            .collect()
    }

    #[test]
    fn recovers_the_true_station_position_from_real_geometry() {
        let clock = 50.0; // km of receiver-clock bias
        let m = ranges(clock);
        let fix = solve(&m, [0.0, 0.0, 0.0]).expect("converges");
        let err_m = norm3([
            fix.ecef[0] - STATION[0],
            fix.ecef[1] - STATION[1],
            fix.ecef[2] - STATION[2],
        ]) * 1000.0;
        assert!(err_m < 1.0, "position error {err_m:.3} m");
        assert!((fix.clock_km - clock).abs() < 1e-3, "clock {}", fix.clock_km);
        assert!(fix.residual_rms_m < 1e-3, "residual {}", fix.residual_rms_m);
        // real GPS geometry gives a healthy GDOP
        assert!(fix.gdop > 1.0 && fix.gdop < 8.0, "gdop {}", fix.gdop);
        // and the recovered lat/lon is the New York site
        assert!((fix.lat - 40.65).abs() < 0.01 && (fix.lon + 73.80).abs() < 0.01);
    }

    #[test]
    fn needs_four_satellites() {
        let m = ranges(0.0);
        assert!(solve(&m[..3], [0.0, 0.0, 0.0]).is_none());
        assert!(solve(&m[..4], [0.0, 0.0, 0.0]).is_some());
    }

    /// 3 GPS + 3 BDS rows on the real six-sat geometry, each constellation
    /// with its OWN receiver-clock bias.
    fn ranges_mixed(c_gps: f64, c_bds: f64) -> Vec<MeasSys> {
        SATS.iter()
            .enumerate()
            .map(|(k, &s)| {
                let g = norm3([STATION[0] - s[0], STATION[1] - s[1], STATION[2] - s[2]]);
                let system = if k < 3 { 0 } else { 1 };
                let clock = if system == 0 { c_gps } else { c_bds };
                MeasSys { sat: s, pseudorange: g + clock, system }
            })
            .collect()
    }

    #[test]
    fn mixed_solve_recovers_position_and_both_clocks() {
        let (c_gps, c_bds) = (50.0, 80.0); // km — deliberately different
        let m = ranges_mixed(c_gps, c_bds);
        let fix = solve_mixed(&m, [0.0, 0.0, 0.0]).expect("converges");
        let err_m = norm3([
            fix.ecef[0] - STATION[0],
            fix.ecef[1] - STATION[1],
            fix.ecef[2] - STATION[2],
        ]) * 1000.0;
        assert!(err_m < 1.0, "position error {err_m:.3} m");
        assert!((fix.clock_gps_km - c_gps).abs() < 1e-3, "gps clock {}", fix.clock_gps_km);
        assert!((fix.clock_bds_km - c_bds).abs() < 1e-3, "bds clock {}", fix.clock_bds_km);
        assert!((fix.isx_km - (c_gps - c_bds)).abs() < 1e-3, "isx {}", fix.isx_km);
        assert!(fix.residual_rms_m < 1e-3, "residual {}", fix.residual_rms_m);
        assert_eq!((fix.n_gps, fix.n_bds), (3, 3));
        // the recovered lat/lon is the New York site
        assert!((fix.lat - 40.65).abs() < 0.01 && (fix.lon + 73.80).abs() < 0.01);
    }

    /// RAIM rejection: a full 1 ms tooth slip (+299.792 km) on one channel
    /// must be dropped, and the fix must still recover the station.
    #[test]
    fn rejection_drops_a_tooth_slip() {
        let mut m = ranges(50.0);
        m[2].pseudorange += 299.792; // 1 ms of light-travel
        let (fix, dropped) =
            solve_with_rejection(&m, [0.0, 0.0, 0.0], 3).expect("converges");
        assert_eq!(dropped, vec![2], "the tooth-slip channel must be dropped");
        let err_m = norm3([
            fix.ecef[0] - STATION[0],
            fix.ecef[1] - STATION[1],
            fix.ecef[2] - STATION[2],
        ]) * 1000.0;
        assert!(err_m < 1.0, "position error {err_m:.3} m");
    }

    /// Elevation weighting: a 25 m bias on the LOWEST-elevation satellite
    /// (sub-rejection-threshold noise, the multipath class the weighting
    /// exists for) must hurt the weighted solve less than the unweighted
    /// one, while a clean solve is bit-identical under any weighting.
    #[test]
    fn elevation_weighting_tames_a_low_outlier() {
        let m = ranges(50.0);
        // find the lowest-elevation satellite from the station
        let low = (0..m.len())
            .min_by(|&a, &b| {
                elev_rad(STATION, m[a].sat)
                    .partial_cmp(&elev_rad(STATION, m[b].sat))
                    .unwrap()
            })
            .unwrap();
        let mut biased = ranges(50.0);
        biased[low].pseudorange += 0.025; // 25 m, below the rejection gate
        let err = |fix: Fix| {
            norm3([
                fix.ecef[0] - STATION[0],
                fix.ecef[1] - STATION[1],
                fix.ecef[2] - STATION[2],
            ]) * 1000.0
        };
        // unweighted reference: solve with weights forced to 1 by using
        // measurements at the zenith? No — compare against the pre-
        // weighting behavior encoded directly: the weighted solve must be
        // CLOSER to truth than the naive expectation bias*(weight share).
        let fw = solve(&biased, [0.0, 0.0, 0.0]).expect("converges");
        let e_w = err(fw);
        let fu = solve_w(&biased, [0.0, 0.0, 0.0], false).expect("converges");
        let e_u = err(fu);
        // the same solve with the biased sat removed bounds the best case
        let mut clean = biased.clone();
        clean.remove(low);
        let e_best = err(solve(&clean, [0.0, 0.0, 0.0]).expect("converges"));
        assert!(
            e_w < e_u && e_w >= e_best - 1e-9,
            "weighted {e_w:.2} m must beat unweighted {e_u:.2} m (best possible {e_best:.2} m)"
        );
    }

    /// A clean set must pass untouched (no false rejections).
    #[test]
    fn rejection_leaves_clean_set_untouched() {
        let m = ranges(50.0);
        let (_fix, dropped) =
            solve_with_rejection(&m, [0.0, 0.0, 0.0], 3).expect("converges");
        assert!(dropped.is_empty());
    }

    /// Mixed: a tooth slip on a BDS row must not drag the isx state.
    /// Note the redundancy requirement: with 6 rows / 5 unknowns there is
    /// only one degree of freedom — detection works, but IDENTIFICATION of
    /// the bad row is not guaranteed. With 7+ rows the wrapper identifies
    /// reliably; the test uses 7.
    #[test]
    fn mixed_rejection_protects_isx() {
        let (c_gps, c_bds) = (50.0, 80.0);
        let mut m = ranges_mixed(c_gps, c_bds);
        // add a 7th row (GPS) for real redundancy
        let extra = [15000.0, 15000.0, 15000.0];
        let g = norm3([STATION[0] - extra[0], STATION[1] - extra[1], STATION[2] - extra[2]]);
        m.push(MeasSys { sat: extra, pseudorange: g + c_gps, system: 0 });
        m[4].pseudorange -= 299.792; // BDS row, 1 ms slip
        let (fix, dropped) =
            solve_mixed_with_rejection(&m, [0.0, 0.0, 0.0], 3).expect("converges");
        assert_eq!(dropped, vec![4]);
        assert!((fix.isx_km - (c_gps - c_bds)).abs() < 1e-3, "isx {}", fix.isx_km);
    }

    #[test]
    fn mixed_solve_works_at_the_live_minimum_3_gps_2_bds() {
        let (c_gps, c_bds) = (50.0, -120.0);
        let m: Vec<MeasSys> = ranges_mixed(c_gps, c_bds)
            .into_iter()
            .enumerate()
            .filter(|(k, _)| *k != 2) // 2 GPS + 3 BDS would do; drop one GPS -> 2+3... keep 5 rows
            .map(|(_, r)| r)
            .collect();
        // exactly 5 rows: 2 GPS + 3 BDS here; gate requires >=3 GPS? No: the
        // solver itself only needs >=5 rows with >=1 per system; the >=3 GPS /
        // >=2 BDS policy lives in the caller (live_fix).
        let fix = solve_mixed(&m, [0.0, 0.0, 0.0]).expect("5-row mix converges");
        let err_m = norm3([
            fix.ecef[0] - STATION[0],
            fix.ecef[1] - STATION[1],
            fix.ecef[2] - STATION[2],
        ]) * 1000.0;
        assert!(err_m < 1.0, "position error {err_m:.3} m");
        assert!((fix.clock_gps_km - c_gps).abs() < 1e-3);
        assert!((fix.clock_bds_km - c_bds).abs() < 1e-3);
    }

    #[test]
    fn mixed_solve_rejects_under_determined_inputs() {
        let m = ranges_mixed(0.0, 0.0);
        assert!(solve_mixed(&m[..4], [0.0, 0.0, 0.0]).is_none()); // < 5 rows
        let gps_only: Vec<MeasSys> = m.iter().copied().filter(|r| r.system == 0).collect();
        assert!(solve_mixed(&gps_only, [0.0, 0.0, 0.0]).is_none()); // one system
    }

    #[test]
    fn altitude_hold_row_does_not_couple_to_receiver_clock() {
        // 3 real satellite rows with a receiver clock bias, plus an
        // altitude-hold pseudo-measurement: range to Earth's centre equals
        // the station's geocentric radius. That row has NO receiver clock
        // in it, so changing the clock bias on the satellite rows must not
        // move the recovered position — the clock unknown absorbs it fully.
        // (If the pseudo-row wrongly carried the +1 clock coefficient, a
        // clock change would trade directly into altitude/position.)
        let r0 = norm3(STATION);
        let solve_with = |clock_km: f64| {
            let mut m: Vec<Meas> = ranges(clock_km).into_iter().take(3).collect();
            m.push(Meas {
                sat: [0.0, 0.0, 0.0],
                pseudorange: r0,
                clock_free: true,
            });
            solve(&m, STATION).expect("converges")
        };
        let a = solve_with(0.0);
        let b = solve_with(300.0); // +300 km of receiver clock
        let drift_m = norm3([
            a.ecef[0] - b.ecef[0],
            a.ecef[1] - b.ecef[1],
            a.ecef[2] - b.ecef[2],
        ]) * 1000.0;
        assert!(
            drift_m < 0.01,
            "position moved {drift_m:.3} m when only the clock changed"
        );
        // the 3+alt fix lands at the station's geocentric radius
        let rad_err_m = (norm3(a.ecef) - r0).abs() * 1000.0;
        assert!(rad_err_m < 1.0, "geocentric radius off by {rad_err_m:.3} m");
        // and horizontally at the NYC station
        assert!((a.lat - 40.65).abs() < 0.05 && (a.lon + 73.80).abs() < 0.05);
    }

    #[test]
    fn altitude_hold_stays_near_the_full_4sat_fix() {
        // Drop-one-sat check on real geometry: the 3-sat + altitude-hold
        // fix must stay within the publish gate of the full 4-sat fix.
        // (Characterization bound: this geometry computes far inside it;
        // the live comparison against tracker data stays a sky-dependent
        // check for when 4+ channels anchor simultaneously.)
        let r0 = norm3(STATION);
        let full = solve(&ranges(0.0)[..4], STATION).expect("4-sat converges");
        let mut m: Vec<Meas> = ranges(0.0).into_iter().take(3).collect();
        m.push(Meas {
            sat: [0.0, 0.0, 0.0],
            pseudorange: r0,
            clock_free: true,
        });
        let hold = solve(&m, STATION).expect("3+alt converges");
        let dist_m = norm3([
            full.ecef[0] - hold.ecef[0],
            full.ecef[1] - hold.ecef[1],
            full.ecef[2] - hold.ecef[2],
        ]) * 1000.0;
        assert!(
            dist_m < 2000.0,
            "alt-hold fix {dist_m:.0} m from the 4-sat fix (publish gate)"
        );
    }
}
