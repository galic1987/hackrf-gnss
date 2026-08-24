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
pub fn solve(meas: &[Meas], guess: [f64; 3]) -> Option<Fix> {
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
            for i in 0..4 {
                htr[i] += u[i] * r;
                for j in 0..4 {
                    hth[i][j] += u[i] * u[j];
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
