//! Doppler geolocation: solve for the receiver position and clock error from
//! attributed Iridium burst carriers — the inverse of `ppm::estimate_ppm`
//! (known position -> clock error becomes known time -> position + clock).
//!
//! Model per burst i at time t_i:
//!
//! ```text
//! f_i = f_nom_i * (1 - v_r/c) * (1 + e_cap(i)) + noise
//! v_r = <v_sat_ecef, (r_sat - r_rx)/|r_sat - r_rx|>
//! ```
//!
//! with e the fractional LO error. The Earth-rotation term (~350 m/s at 40 N,
//! ~2 ppm-equivalent at L-band) is NOT dropped — it is exactly absorbed:
//! sat_ecef_pv yields the ECEF-frame (rotation-removed) satellite velocity,
//! and <omega x (r_sat - r_rx), n> = 0, so the plain ECEF range rate already
//! is the signal Doppler (see the frame note on `predict`).
//!
//! MULTI-CAPTURE JOINT SOLVE: bursts from several captures share the receiver
//! position but each capture gets its own clock unknown e_k (the correction
//! state differs between days). The state is (lat, lon, e_1..e_N); arcs from
//! different satellites at different times fix the geometry that any single
//! short arc cannot.
//!
//! PER-SATELLITE FREQUENCY BIASES (the classical Transit/ARGOS treatment):
//! TLE along-track error appears as a near-constant offset per satellite per
//! pass (measured live: +0.6/+1.5/-1.8 kHz on one capture's arcs), which
//! otherwise inflates the joint RMS and the formal sigma by 50x. With
//! `bias_prior_hz = Some(s)`, each satellite gets a constant bias state b_s
//! (Hz, init 0) under a ridge prior b_s ~ N(0, s^2) — the bias absorbs the
//! TLE level error while the position information, living in the Doppler
//! SLOPE, is untouched. IDENTIFIABILITY: on a single-satellite capture, e_k
//! and b_s are nearly collinear (b + e*f_nom is one number); the ridge keeps
//! the normal matrix invertible and bounds the split, and the position sigma
//! guard still refuses the fix — tested. The prior biases the formal sigma
//! slightly optimistic; that is documented, not hidden.
//!
//! Solved by Levenberg-Marquardt over (lat, lon, e_1..e_N) with altitude FIXED
//! at 0: for a surface receiver seen from space, altitude trades almost
//! exactly against a tiny range change and is unobservable at burst-noise
//! levels — the normal matrix goes singular if it is included. The position
//! is therefore a surface fix; a receiver at real altitude h sees its fix
//! pulled slightly along the line of sight (documented, tested).
//!
//! Robustness: residuals are outlier-screened between LM rounds by
//! median/IQR with a 50 Hz floor (burst carrier noise is tens of Hz), and the
//! formal 1-sigma position error comes from the weighted normal matrix scaled
//! by the residual RMS — reported honestly, not as a point estimate alone.

use crate::gps::{geodetic_to_ecef, predict_doppler_el_f, sat_ecef_pv, GpsSat};
use crate::iridium::ppm::DecodedBurst;

const C_KMS: f64 = 299_792.458;

/// One attributed burst observation.
#[derive(Clone, Copy)]
pub struct Obs<'a> {
    /// absolute time, unix seconds
    pub t: f64,
    /// measured carrier, Hz
    pub f_meas: f64,
    /// snapped channel centre, Hz
    pub f_nom: f64,
    pub sat: &'a GpsSat,
    /// demodulator confidence, percent
    pub conf: u32,
    /// extra weight multiplier: 1.0 for position-attributed bursts, 0.5 for
    /// Doppler-attributed ones (they carry no independent position anchor)
    pub w_scale: f64,
    /// which capture this burst came from — each capture gets its own clock
    /// error unknown in the joint solve
    pub cap: usize,
}

/// A solved fix.
#[derive(Debug)]
pub struct Fix {
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// always 0: solved with altitude constrained, see module docs
    pub alt_km: f64,
    /// fractional LO error per capture, ppm (positive = clock fast, same
    /// convention as the ppm estimator)
    pub clock_ppm: Vec<f64>,
    /// weighted residual RMS, Hz
    pub rms_hz: f64,
    pub n_used: usize,
    /// formal 1-sigma horizontal position error, km (residual-scaled)
    pub sigma_km: f64,
    /// geometric dilution: sigma per unit of residual RMS
    pub dop: f64,
    pub iters: usize,
    /// (satellite, n, residual RMS Hz, solved bias Hz, solved drift Hz/s —
    /// zeros when those states are off)
    pub per_sat: Vec<(String, usize, f64, f64, f64)>,
}

/// Predicted carrier for one observation. None if SGP4 fails.
///
/// Frame note (the classic bug, pinned here): `sat_ecef_pv` returns the
/// satellite's ECEF-frame velocity (Earth rotation already removed), and the
/// receiver is FIXED in ECEF, so the range rate is simply <v_sat_ecef, n>.
/// The task-level model f = f_nom(1 - v_r/c) with v_r = <v_sat - omega x
/// r_rx, n> assumes an INERTIAL v_sat; applied to an ECEF v_sat it would
/// double-count the rotation. The two forms are identical anyway because
/// <omega x (r_sat - r_rx), n> = 0 — the receiver's rotation velocity cancels
/// the frame-rotation part of the satellite velocity exactly. Do not
/// reintroduce the omega x r_rx term against ECEF velocities.
///
/// Per-satellite state columns for one observation (bias and/or drift rate).
/// `t_ref` decorrelates the two: the bias reads at the arc's mean time.
#[derive(Clone, Copy, Default)]
pub struct SatCols {
    pub bias: Option<usize>,
    pub rate: Option<usize>,
    pub t_ref: f64,
}

/// State layout: x = [lat, lon, e_0, e_1, ..., b_0, b_1, ..., r_0, r_1, ...]
/// — lat/lon in radians, one fractional clock error per capture, one Hz bias
/// and optionally one Hz/s drift rate per satellite (when enabled). `sc` is
/// this observation's per-satellite columns.
fn predict(o: &Obs, x: &[f64], sc: SatCols, dcap: usize) -> Option<f64> {
    let (pos, vel) = sat_ecef_pv(o.sat, o.t)?;
    let rx = geodetic_to_ecef(x[0].to_degrees(), x[1].to_degrees(), 0.0);
    let rel = [pos[0] - rx[0], pos[1] - rx[1], pos[2] - rx[2]];
    let rng = (rel[0] * rel[0] + rel[1] * rel[1] + rel[2] * rel[2]).sqrt();
    let vr = (vel[0] * rel[0] + vel[1] * rel[1] + vel[2] * rel[2]) / rng;
    let b = sc.bias.map(|c| x[c]).unwrap_or(0.0);
    let r = sc.rate.map(|c| x[c] * (o.t - sc.t_ref)).unwrap_or(0.0);
    Some(o.f_nom * (1.0 - vr / C_KMS) * (1.0 + x[2 + dcap]) + b + r)
}

fn weight(conf: u32, w_scale: f64) -> f64 {
    (conf as f64 / 100.0).clamp(0.2, 1.0) * w_scale
}

/// Solve A x = b by Gaussian elimination with partial pivoting. None if
/// singular. Small dense systems only (the normal matrix is (2+N)^2).
fn mat_solve(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let n = b.len();
    let mut m: Vec<Vec<f64>> = a.to_vec();
    let mut x = b.to_vec();
    for col in 0..n {
        let piv = (col..n).max_by(|&i, &j| m[i][col].abs().partial_cmp(&m[j][col].abs()).unwrap())?;
        if m[piv][col].abs() < 1e-300 {
            return None;
        }
        m.swap(col, piv);
        x.swap(col, piv);
        for r in (col + 1)..n {
            let f = m[r][col] / m[col][col];
            for c in col..n {
                m[r][c] -= f * m[col][c];
            }
            x[r] -= f * x[col];
        }
    }
    for r in (0..n).rev() {
        let mut s = x[r];
        for c in (r + 1)..n {
            s -= m[r][c] * x[c];
        }
        x[r] = s / m[r][r];
    }
    Some(x)
}

/// Inverse of a small dense matrix via elimination on identity columns.
fn mat_inverse(a: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let n = a.len();
    let mut inv = vec![vec![0.0; n]; n];
    for col in 0..n {
        let mut e = vec![0.0; n];
        e[col] = 1.0;
        let x = mat_solve(a, &e)?;
        for r in 0..n {
            inv[r][col] = x[r];
        }
    }
    Some(inv)
}

/// One LM solve over the given observation set. `bcols` is parallel to `obs`:
/// the bias column for each observation (None when biases are disabled).
/// `sigma_prior` is the ridge strength on bias states (Hz). Returns
/// (x, (JtWJ)^-1, residual RMS Hz, iterations).
#[allow(clippy::too_many_arguments)]
fn lm(
    obs: &[&Obs],
    dcaps: &[usize],
    scols: &[SatCols],
    prior: Option<(f64, f64)>, // (sigma bias Hz, sigma rate Hz/s; rate off if <= 0)
    lat0: f64,
    lon0: f64,
    e0: &[f64],
    nbias: usize,
    nrate: usize,
) -> Result<(Vec<f64>, Vec<Vec<f64>>, f64, usize), String> {
    let np = 2 + e0.len() + nbias + nrate;
    let mut x: Vec<f64> = std::iter::once(lat0.to_radians())
        .chain(std::iter::once(lon0.to_radians()))
        .chain(e0.iter().copied())
        .chain(std::iter::repeat(0.0).take(nbias + nrate))
        .collect();
    // column scaling keeps the normal matrix conditioned: ~100 m in angle,
    // 1 ppm in clock, 1 kHz in bias, 100 Hz/s in rate
    let scale: Vec<f64> = std::iter::repeat(1e-5)
        .take(2)
        .chain(std::iter::repeat(1e-6).take(e0.len()))
        .chain(std::iter::repeat(1e3).take(nbias))
        .chain(std::iter::repeat(1e2).take(nrate))
        .collect();
    let prior2_b = prior.map(|(s, _)| 1.0 / (s * s));
    let prior2_r = prior.and_then(|(_, s)| (s > 0.0).then_some(1.0 / (s * s)));
    let mut lambda = 1e-3f64;
    let mut cost_prev = f64::MAX;
    let mut iters = 0;
    for it in 0..30 {
        iters = it + 1;
        // residuals and Jacobian (numeric, central differences; an obs touches
        // only lat, lon, its own capture's clock and its satellite's bias —
        // the bias derivative is exactly 1 Hz/Hz and needs no differencing)
        let mut jtj = vec![vec![0.0f64; np]; np];
        let mut jtr = vec![0.0f64; np];
        let mut cost = 0.0f64;
        let mut n = 0usize;
        for ((o, &dc), &sc) in obs.iter().zip(dcaps).zip(scols) {
            let w = weight(o.conf, o.w_scale);
            let Some(f0) = predict(o, &x, sc, dc) else { continue };
            let r = f0 - o.f_meas;
            cost += w * w * r * r;
            n += 1;
            // dynamic columns: lat, lon, clock; then analytic bias (1) and
            // rate (t - t_ref) columns
            let cols = [0, 1, 2 + dc];
            let mut j = [0.0f64; 3];
            for (ci, &p) in cols.iter().enumerate() {
                let h = scale[p];
                let mut xp = x.clone();
                xp[p] += h;
                let mut xm = x.clone();
                xm[p] -= h;
                let (fp, fm) = match (predict(o, &xp, sc, dc), predict(o, &xm, sc, dc)) {
                    (Some(a), Some(b)) => (a, b),
                    _ => return Err("SGP4 failed mid-solve".into()),
                };
                j[ci] = (fp - fm) / (2.0 * h);
            }
            for (ai, &a) in cols.iter().enumerate() {
                jtr[a] += w * w * j[ai] * r;
                for (bi, &b) in cols.iter().enumerate() {
                    jtj[a][b] += w * w * j[ai] * j[bi];
                }
            }
            for (col, deriv) in [(sc.bias, 1.0), (sc.rate, o.t - sc.t_ref)].into_iter().filter_map(|(c, d)| c.map(|c| (c, d))) {
                jtr[col] += w * w * deriv * r;
                for (ai, &a) in cols.iter().enumerate() {
                    jtj[a][col] += w * w * j[ai] * deriv;
                    jtj[col][a] += w * w * j[ai] * deriv;
                }
                jtj[col][col] += w * w * deriv * deriv;
            }
            if let (Some(cb), Some(cr)) = (sc.bias, sc.rate) {
                jtj[cb][cr] += w * w * (o.t - sc.t_ref);
                jtj[cr][cb] += w * w * (o.t - sc.t_ref);
            }
        }
        if n < np {
            return Err(format!("too few usable observations ({n}); need >= {np} for this state"));
        }
        // ridge priors: bias and rate states measure 0 with their sigmas
        if let Some(p2) = prior2_b {
            for bc in (2 + e0.len())..(2 + e0.len() + nbias) {
                jtj[bc][bc] += p2;
                jtr[bc] += p2 * x[bc];
                cost += p2 * x[bc] * x[bc];
            }
        }
        if let Some(p2) = prior2_r {
            for rc in (np - nrate)..np {
                jtj[rc][rc] += p2;
                jtr[rc] += p2 * x[rc];
                cost += p2 * x[rc] * x[rc];
            }
        }
        // condition check: the normal matrix must be invertible
        if mat_inverse(&jtj).is_none() {
            return Err("singular geometry: observations do not constrain (lat, lon, e) — \
                        too short an arc or a single satellite".into());
        }
        // LM step with damping
        let mut stepped = false;
        for _ in 0..12 {
            let mut a = jtj.clone();
            for p in 0..np {
                a[p][p] += lambda * jtj[p][p].max(1e-30);
            }
            let Some(d) = mat_solve(&a, &jtr.iter().map(|v| -v).collect::<Vec<_>>()) else {
                lambda *= 10.0;
                continue;
            };
            let xn: Vec<f64> = x.iter().zip(&d).map(|(a, b)| a + b).collect();
            let mut cnew = 0.0;
            for ((o, &dc), &sc) in obs.iter().zip(dcaps).zip(scols) {
                let w = weight(o.conf, o.w_scale);
                if let Some(f) = predict(o, &xn, sc, dc) {
                    let r = f - o.f_meas;
                    cnew += w * w * r * r;
                }
            }
            if let Some(p2) = prior2_b {
                for bc in (2 + e0.len())..(2 + e0.len() + nbias) {
                    cnew += p2 * xn[bc] * xn[bc];
                }
            }
            if let Some(p2) = prior2_r {
                for rc in (np - nrate)..np {
                    cnew += p2 * xn[rc] * xn[rc];
                }
            }
            if cnew < cost {
                x = xn;
                lambda = (lambda / 10.0).max(1e-9);
                stepped = true;
                break;
            }
            lambda *= 10.0;
        }
        if !stepped || (cost_prev - cost).abs() < 1e-6 * cost.max(1.0) {
            if !stepped && cost_prev == f64::MAX {
                // never improved on the first pass
                return Err("LM made no progress: degenerate geometry".into());
            }
            break;
        }
        cost_prev = cost;
    }
    // final residual RMS (unweighted, Hz, data residuals only) and the
    // inverse at the solution
    let mut rs = Vec::new();
    let mut jtj = vec![vec![0.0f64; np]; np];
    for ((o, &dc), &sc) in obs.iter().zip(dcaps).zip(scols) {
        let w = weight(o.conf, o.w_scale);
        if let Some(f) = predict(o, &x, sc, dc) {
            rs.push(f - o.f_meas);
            let cols = [0, 1, 2 + dc];
            let mut j = [0.0f64; 3];
            for (ci, &p) in cols.iter().enumerate() {
                let h = scale[p];
                let mut xp = x.clone();
                xp[p] += h;
                let mut xm = x.clone();
                xm[p] -= h;
                if let (Some(a), Some(b)) = (predict(o, &xp, sc, dc), predict(o, &xm, sc, dc)) {
                    j[ci] = (a - b) / (2.0 * h);
                }
            }
            for (ai, &a) in cols.iter().enumerate() {
                for (bi, &b) in cols.iter().enumerate() {
                    jtj[a][b] += w * w * j[ai] * j[bi];
                }
            }
            for (col, deriv) in [(sc.bias, 1.0), (sc.rate, o.t - sc.t_ref)].into_iter().filter_map(|(c, d)| c.map(|c| (c, d))) {
                for (ai, &a) in cols.iter().enumerate() {
                    jtj[a][col] += w * w * j[ai] * deriv;
                    jtj[col][a] += w * w * j[ai] * deriv;
                }
                jtj[col][col] += w * w * deriv * deriv;
            }
            if let (Some(cb), Some(cr)) = (sc.bias, sc.rate) {
                jtj[cb][cr] += w * w * (o.t - sc.t_ref);
                jtj[cr][cb] += w * w * (o.t - sc.t_ref);
            }
        }
    }
    if let Some(p2) = prior2_b {
        for bc in (2 + e0.len())..(2 + e0.len() + nbias) {
            jtj[bc][bc] += p2;
        }
    }
    if let Some(p2) = prior2_r {
        for rc in (np - nrate)..np {
            jtj[rc][rc] += p2;
        }
    }
    let inv = mat_inverse(&jtj).ok_or("singular normal matrix at solution")?;
    let rms = (rs.iter().map(|r| r * r).sum::<f64>() / rs.len().max(1) as f64).sqrt();
    Ok((x, inv, rms, iters))
}

/// Median/IQR outlier screen. Keeps indices within max(3*IQR, 50 Hz) of the
/// median residual.
fn screen(resid: &[f64]) -> Vec<bool> {
    let mut s: Vec<f64> = resid.to_vec();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let med = s[s.len() / 2];
    let q1 = s[s.len() / 4];
    let q3 = s[3 * s.len() / 4];
    let tol = (3.0 * (q3 - q1)).max(50.0);
    resid.iter().map(|r| (r - med).abs() <= tol).collect()
}

/// Solve (lat, lon, e_1..e_N, [b_s1..b_sM, [r_s1..r_sM]]) from attributed
/// bursts: shared receiver position, one clock error per capture (`Obs::cap`
/// indexes `e0`), and — when `bias_prior_hz` is Some — one constant Hz bias
/// per satellite under a ridge prior of that sigma, plus — when
/// `rate_prior_hz_s` is also Some — one Hz/s drift per satellite under its
/// own ridge (measured live: TLE rate error drifts the residual up to
/// +-3 kHz across a pass, so a constant bias alone cannot absorb it; the
/// drift eats some Doppler-slope information, which is the honest cost — the
/// prior sigma bounds it). See the module docstring for identifiability.
/// Err with a clear message on degenerate geometry instead of returning
/// garbage.
pub fn solve_fix(
    obs: &[Obs],
    lat0: f64,
    lon0: f64,
    e0: &[f64],
    bias_prior_hz: Option<f64>,
    rate_prior_hz_s: Option<f64>,
) -> Result<Fix, String> {
    solve_fix_inner(obs, lat0, lon0, e0, bias_prior_hz, rate_prior_hz_s, true)
}

/// The solve without the degeneracy guard when `enforce_guard` is false —
/// multi-start needs to SEE the bad basins to rank them, not have them
/// refused upstream.
#[allow(clippy::too_many_arguments)]
pub fn solve_fix_inner(
    obs: &[Obs],
    lat0: f64,
    lon0: f64,
    e0: &[f64],
    bias_prior_hz: Option<f64>,
    rate_prior_hz_s: Option<f64>,
    enforce_guard: bool,
) -> Result<Fix, String> {
    // the state needs a clock slot for every capture index present, even if
    // the caller passed fewer initial values (single-capture solves of a
    // capture that isn't index 0). SPARSE indices (a capture dropped from a
    // joint set) must not leave an empty clock column -- an all-zero column
    // makes the normal matrix singular. Map present caps to dense slots.
    let mut present: Vec<usize> = obs.iter().map(|o| o.cap).collect();
    present.sort_unstable();
    present.dedup();
    let ncaps = present.len();
    let e0: Vec<f64> = present.iter().map(|&k| e0.get(k).copied().unwrap_or(0.0)).collect();
    let cap_dense = |c: usize| present.iter().position(|&p| p == c).unwrap_or(0);
    // bias/rate columns: one per satellite, after the clock slots
    let mut sat_names: Vec<String> = obs.iter().map(|o| o.sat.name.clone()).collect();
    sat_names.sort();
    sat_names.dedup();
    let nbias = if bias_prior_hz.is_some() { sat_names.len() } else { 0 };
    let nrate = if rate_prior_hz_s.is_some() { sat_names.len() } else { 0 };
    let bias_base = 2 + ncaps;
    let rate_base = bias_base + nbias;
    let scol_of = |name: &str| -> (Option<usize>, Option<usize>) {
        let i = sat_names.iter().position(|n| n == name);
        (
            if nbias == 0 { None } else { i.map(|i| bias_base + i) },
            if nrate == 0 { None } else { i.map(|i| rate_base + i) },
        )
    };
    // per-satellite reference time: the arc's mean, so the bias reads there
    // and bias/rate stay decorrelated
    let scols: Vec<SatCols> = obs
        .iter()
        .map(|o| {
            let (b, r) = scol_of(&o.sat.name);
            let ts: Vec<f64> = obs.iter().filter(|q| q.sat.name == o.sat.name).map(|q| q.t).collect();
            let t_ref = ts.iter().sum::<f64>() / ts.len() as f64;
            SatCols { bias: b, rate: r, t_ref }
        })
        .collect();
    let dcaps: Vec<usize> = obs.iter().map(|o| cap_dense(o.cap)).collect();
    let prior = bias_prior_hz.map(|sb| (sb, rate_prior_hz_s.unwrap_or(0.0)));
    let np = 2 + e0.len() + nbias + nrate;
    // without biases every parameter needs data; with them the ridge prior
    // constrains the bias slots
    let min_obs = if nbias > 0 { np } else { np + 1 };
    if obs.len() < min_obs {
        return Err(format!("too few bursts ({}); need >= {min_obs} for this state", obs.len()));
    }
    // up to 3 solve+screen rounds
    let mut keep: Vec<usize> = (0..obs.len()).collect();
    let mut solved = None;
    for _ in 0..3 {
        let cur: Vec<&Obs> = keep.iter().map(|&i| &obs[i]).collect();
        let cur_sc: Vec<SatCols> = keep.iter().map(|&i| scols[i]).collect();
        let cur_dc: Vec<usize> = keep.iter().map(|&i| dcaps[i]).collect();
        let (x, inv, rms, iters) = lm(&cur, &cur_dc, &cur_sc, prior, lat0, lon0, &e0, nbias, nrate)?;
        let resid: Vec<f64> = cur
            .iter()
            .zip(&cur_sc)
            .zip(&cur_dc)
            .filter_map(|((o, &sc), &dc)| predict(o, &x, sc, dc).map(|f| f - o.f_meas))
            .collect();
        let mask = screen(&resid);
        let dropped: Vec<usize> = keep
            .iter()
            .copied()
            .zip(mask.iter().copied())
            .filter(|(_, m)| !m)
            .map(|(i, _)| i)
            .collect();
        solved = Some((x, inv, rms, iters));
        if dropped.is_empty() || keep.len() - dropped.len() < min_obs {
            break;
        }
        keep = keep
            .iter()
            .copied()
            .zip(mask.iter().copied())
            .filter(|(_, m)| *m)
            .map(|(i, _)| i)
            .collect();
    }
    let (mut x, inv, rms, iters) = solved.unwrap();
    // Gauge fix, single capture with free bias states: the model
    // f(1+e) + b_s is invariant under e -> e + m/f, b_s -> b_s - m, and the
    // ridge prior is too weak to pin that direction in practice (the 00:12
    // EDT solve printed +18.4 ppm of clock with every bias ~ -31.8 kHz; the
    // physical clock is the SUM, -1.2 ppm). Pin the gauge the way the prior
    // intends: mean bias = 0 over sats with kept observations, so the clock
    // term owns the common mode. Residuals are unchanged (the transform is
    // exact up to Doppler-leakage ~ Hz). Multi-capture solves are left
    // alone: satellites shared between captures break the degeneracy with
    // data, and re-centring would move off the likelihood optimum.
    if nbias > 0 && ncaps == 1 {
        let mut m = 0.0;
        let mut nn = 0usize;
        for (ni, n) in sat_names.iter().enumerate() {
            if keep.iter().any(|&i| obs[i].sat.name == *n) {
                m += x[bias_base + ni];
                nn += 1;
            }
        }
        if nn > 0 {
            m /= nn as f64;
            let f_bar = keep.iter().map(|&i| obs[i].f_nom).sum::<f64>() / keep.len() as f64;
            x[2] += m / f_bar;
            for ni in 0..sat_names.len() {
                x[bias_base + ni] -= m;
            }
        }
    }
    let lat = x[0];
    // formal uncertainty: sigma^2 per observation ~ rms^2; position block is
    // the lat/lon part of (JtWJ)^-1, converted rad -> km at the surface. With
    // biases enabled the prior is inside JtWJ, so sigma is slightly
    // optimistic — documented in the module docstring.
    let r_earth = 6378.137;
    let s_lat_km = rms * inv[0][0].max(0.0).sqrt() * r_earth;
    let s_lon_km = rms * inv[1][1].max(0.0).sqrt() * r_earth * lat.cos().abs().max(0.2);
    let sigma_km = (s_lat_km * s_lat_km + s_lon_km * s_lon_km).sqrt();
    // Degenerate geometry check: a numerically invertible normal matrix can
    // still be worthless (a 60 s single-satellite arc "solves" with a formal
    // sigma of tens of km and the point estimate is wherever the guess was).
    // Above 10 km 1-sigma this is not a fix; say so instead of printing
    // garbage coordinates.
    if enforce_guard && sigma_km > 10.0 {
        return Err(format!(
            "geometry too weak for a surface fix: formal 1-sigma {:.1} km \
             ({} bursts, residual RMS {:.0} Hz — short arc or single satellite)",
            sigma_km,
            keep.len(),
            rms
        ));
    }
    let dop = sigma_km / rms.max(1e-9) * 1e3; // km of fix per kHz of residual
    // per-satellite residual RMS, solved bias and drift over the kept set
    let mut per_sat: Vec<(String, usize, f64, f64, f64)> = Vec::new();
    for (ni, n) in sat_names.iter().enumerate() {
        let rs: Vec<f64> = keep
            .iter()
            .filter(|&&i| obs[i].sat.name == *n)
            .filter_map(|&i| predict(&obs[i], &x, scols[i], dcaps[i]).map(|f| f - obs[i].f_meas))
            .collect();
        if !rs.is_empty() {
            let r = (rs.iter().map(|r| r * r).sum::<f64>() / rs.len() as f64).sqrt();
            let bias = if nbias > 0 { x[bias_base + ni] } else { 0.0 };
            let rate = if nrate > 0 { x[rate_base + ni] } else { 0.0 };
            per_sat.push((n.clone(), rs.len(), r, bias, rate));
        }
    }
    let (lat_deg, lon_deg) = normalize_lat_lon(lat.to_degrees(), x[1].to_degrees());
    Ok(Fix {
        lat_deg,
        lon_deg,
        alt_km: 0.0,
        // dense clock slots are mapped back to the caller's capture indices;
        // unused capture slots read NaN rather than the wrong clock
        clock_ppm: {
            let n_out = 1 + obs.iter().map(|o| o.cap).max().unwrap_or(0);
            let mut v = vec![f64::NAN; n_out];
            for (dk, &orig) in present.iter().enumerate() {
                v[orig] = x[2 + dk] * 1e6;
            }
            v
        },
        rms_hz: rms,
        n_used: keep.len(),
        sigma_km,
        dop,
        iters,
        per_sat,
    })
}

/// Normalize an LM-wandered (lat, lon) back onto the sphere: lat reflects at
/// the poles, lon wraps to [-180, 180). The solver's parameter space is
/// unwrapped; physical equality must be judged after this map.
pub fn normalize_lat_lon(lat_deg: f64, lon_deg: f64) -> (f64, f64) {
    let mut lat = lat_deg.rem_euclid(360.0);
    let mut lon = lon_deg;
    if lat > 180.0 {
        lat = 360.0 - lat;
        lon += 180.0;
    }
    if lat > 90.0 {
        lat = 180.0 - lat;
        lon += 180.0;
    }
    (lat, (lon + 180.0).rem_euclid(360.0) - 180.0)
}

// ------------------------------------------------------------ multi-start

/// One basin visited by the multi-start sweep.
#[derive(Debug, Clone)]
pub struct Candidate {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub rms_hz: f64,
    pub sigma_km: f64,
    /// the LM solve converged at all (singular/no-progress basins are false)
    pub converged: bool,
}

/// The result of a multi-start sweep over coarse initial guesses.
#[derive(Debug)]
pub struct MultiStart {
    /// lowest-RMS candidate passing the sigma guard; None if none does
    pub best: Option<Fix>,
    /// every converged candidate, best RMS first
    pub candidates: Vec<Candidate>,
    /// winner's RMS is less than 2x better than the runner-up's — the two
    /// basins are not decisively separated and the fix must not be trusted
    /// silently
    pub ambiguous: bool,
}

/// Doppler-only geolocation is multimodal: per-satellite systematics create
/// local minima (mirrors across ground tracks), and on real data a wrong
/// basin is easily 30x worse in RMS — so the sweep ranks basins by residual
/// RMS, applies the sigma guard, and demands the winner beat the runner-up
/// by 2x. `starts` are (lat, lon) degrees; the guard (10 km) is applied at
/// selection, not inside the solves.
///
/// The true basin can be NARROWER than the grid: on the real 8-capture set,
/// all 40 coarse starts converged but none passed the guard, while a start
/// near the truth fixes at sigma 1.5 km. So after the coarse sweep the five
/// best-RMS basins get one refinement hop — a fresh solve from each of their
/// converged positions — which is how a narrow true basin is entered.
#[allow(clippy::too_many_arguments)]
pub fn multistart_fix(
    obs: &[Obs],
    e0: &[f64],
    bias_prior_hz: Option<f64>,
    rate_prior_hz_s: Option<f64>,
    starts: &[(f64, f64)],
) -> MultiStart {
    let solve_at = |la: f64, lo: f64| -> (Candidate, Option<Fix>) {
        match solve_fix_inner(obs, la, lo, e0, bias_prior_hz, rate_prior_hz_s, false) {
            Ok(f) => (
                Candidate {
                    lat_deg: f.lat_deg,
                    lon_deg: f.lon_deg,
                    rms_hz: f.rms_hz,
                    sigma_km: f.sigma_km,
                    converged: true,
                },
                Some(f),
            ),
            Err(_) => (
                Candidate { lat_deg: la, lon_deg: lo, rms_hz: f64::MAX, sigma_km: f64::MAX, converged: false },
                None,
            ),
        }
    };
    let mut cands: Vec<(Candidate, Option<Fix>)> =
        starts.iter().map(|&(la, lo)| solve_at(la, lo)).collect();
    // refinement hop: re-solve from the five best coarse basins
    cands.sort_by(|a, b| a.0.rms_hz.partial_cmp(&b.0.rms_hz).unwrap_or(std::cmp::Ordering::Equal));
    let hops: Vec<(f64, f64)> = cands
        .iter()
        .filter(|(c, _)| c.converged)
        .take(5)
        .map(|(c, _)| (c.lat_deg, c.lon_deg))
        .collect();
    for (la, lo) in hops {
        cands.push(solve_at(la, lo));
    }
    cands.sort_by(|a, b| a.0.rms_hz.partial_cmp(&b.0.rms_hz).unwrap_or(std::cmp::Ordering::Equal));
    // dedupe basins: many grid starts converge to the SAME minimum; comparing
    // the winner against its own twin would fake a thin margin. Two
    // candidates within 100 km are the same basin — keep the better RMS.
    let mut basin: Vec<(Candidate, Option<Fix>)> = Vec::new();
    for (c, f) in cands {
        let dup = basin.iter().any(|(b, _)| {
            let dlat = (b.lat_deg - c.lat_deg).to_radians();
            let dlon_deg = (b.lon_deg - c.lon_deg).rem_euclid(360.0).min(360.0 - (b.lon_deg - c.lon_deg).rem_euclid(360.0));
            let dlon = dlon_deg.to_radians();
            let km = 6371.0 * (dlat * dlat + dlon * dlon * b.lat_deg.to_radians().cos().powi(2)).sqrt();
            km < 100.0
        });
        if !dup {
            basin.push((c, f));
        }
    }
    let mut cands = basin;
    let passing: Vec<usize> = (0..cands.len())
        .filter(|&i| cands[i].0.converged && cands[i].0.sigma_km <= 10.0)
        .collect();
    let best = passing.first().and_then(|&i| cands[i].1.take());
    let ambiguous = match (passing.first(), passing.get(1)) {
        (Some(&a), Some(&b)) => cands[b].0.rms_hz < 2.0 * cands[a].0.rms_hz,
        _ => false,
    };
    MultiStart {
        best,
        candidates: cands.into_iter().map(|(c, _)| c).collect(),
        ambiguous,
    }
}

/// A global mid-latitude start grid: lat -60..60 step 30, lon -180..180 step
/// 45 (40 starts). Polar receivers are out of scope (documented, not hidden).
pub fn global_grid() -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let mut lat = -60.0;
    while lat <= 60.0 {
        let mut lon = -180.0;
        while lon < 180.0 {
            out.push((lat, lon));
            lon += 45.0;
        }
        lat += 30.0;
    }
    out
}

// ------------------------------------------------------- doppler attribution

/// Observation weights by attribution class. Position-matched bursts carry an
/// independent physical anchor (the satellite's self-reported position) and
/// get full weight; Doppler-attributed decoded bursts are verified by the
/// margin rule; detected-only bursts never decoded at all — real, but
/// unverified as Iridium and with a noisier carrier — so they count least.
pub const W_POSITION: f64 = 1.0;
pub const W_DOPPLER_DECODED: f64 = 0.5;
pub const W_UNDECODED: f64 = 0.25;

/// A burst attributed by its carrier offset rather than its payload.
#[derive(Debug, Clone)]
pub struct DopplerAttr {
    pub burst: DecodedBurst,
    pub sat: String,
    /// channel centre (raw snap; accepted bursts provably did not fold)
    pub f_nom_hz: f64,
    /// |f_meas - f_pred| at the accepted assignment, Hz
    pub score_hz: f64,
    /// solve weight by class: W_DOPPLER_DECODED, or W_UNDECODED when the
    /// burst never decoded (confidence 0 is the sentinel for that)
    pub w_scale: f64,
}

/// Attribute decoded-but-payload-anonymous bursts by carrier offset. For each
/// burst, every satellite above 5 deg at the burst's time gets a predicted
/// total offset `D_pred(sat, rx, t) + e * f_nom`; a burst is attributed iff
/// the best score is under `tol_hz + e_tol_frac * f_nom` AND the second-best
/// satellite is at least 2x worse (uniqueness margin — a burst between two
/// equally-plausible satellites stays unattributed rather than poisoning the
/// fix). Tolerance reasoning: the carrier measurement is ~tens of Hz, but the
/// burst time is only known to ~0.1 s and Doppler slews at up to ~40 Hz/s
/// near overhead, plus TLE along-track error moves the prediction by
/// hundreds of Hz — so ~2.5 kHz base, plus the clock-error uncertainty term.
///
/// The channel centre is the RAW snap of the measured carrier, and that is a
/// deliberate, conservative choice: allowing each candidate satellite its own
/// fold of the channel (see the 25.6 ppm fold in `ppm`) lets wrong satellites
/// fold their prediction into tolerance and the uniqueness margin collapses
/// — on the real 8-minute capture that variant attributed 89 of 91 bursts
/// ambiguously. With the raw snap, a burst whose true assignment folded a
/// channel simply fails its tolerance and is LOST (conservative), while
/// accepted bursts carry a correct f_nom by construction. Debris TLE entries
/// are excluded — debris does not transmit ring alerts.
pub fn doppler_attribute(
    decoded: &[DecodedBurst],
    sats: &[GpsSat],
    rx: [f64; 3],
    e_frac: f64,
    e_tol_frac: f64,
    tol_hz: f64,
) -> Vec<DopplerAttr> {
    let mut out = Vec::new();
    // concatenated group files (iridium-next + iridium) list the operational
    // satellites twice with identical elements; a duplicate name predicts an
    // identical offset and would trip the uniqueness margin on every burst
    let mut seen = std::collections::HashSet::new();
    let sats: Vec<&GpsSat> = sats
        .iter()
        .filter(|s| !s.name.contains("DEB") && seen.insert(s.name.as_str()))
        .collect();
    for b in decoded {
        let tol = tol_hz + e_tol_frac * b.f_nom_hz;
        let mut best: Option<(f64, &str)> = None;
        let mut second = f64::MAX;
        for s in &sats {
            let Some((fd, el)) = predict_doppler_el_f(s, rx, b.t_epoch, b.f_nom_hz) else { continue };
            if el < 5.0 {
                continue;
            }
            let f_pred = b.f_nom_hz + fd + e_frac * b.f_nom_hz;
            let score = (b.f_meas_hz - f_pred).abs();
            match &best {
                None => best = Some((score, &s.name)),
                Some((bs, _)) if score < *bs => {
                    second = *bs;
                    best = Some((score, &s.name));
                }
                Some((bs, _)) if score < second => second = score,
                _ => {}
            }
        }
        if let Some((score, name)) = best {
            if score < tol && second >= 2.0 * score.max(1.0) {
                out.push(DopplerAttr {
                    w_scale: if b.confidence == 0 { W_UNDECODED } else { W_DOPPLER_DECODED },
                    burst: b.clone(),
                    sat: name.to_string(),
                    f_nom_hz: b.f_nom_hz,
                    score_hz: score,
                });
            }
        }
    }
    out
}
