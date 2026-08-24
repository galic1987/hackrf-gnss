//! GPS detection verdict, a port of `validation/report.py:_fit_passes` /
//! `gnss_verdict`.
//!
//! A single strong correlation peak proves nothing: near the detection floor,
//! isolated crossings are noise tails. What separates a satellite from noise is
//! physics across time — a real pass sweeps its Doppler monotonically through
//! zero, bounded by ~0.9 Hz/s at L1, and lies on a straight line to high r².
//! Only crossings above the acquisition threshold are fitted.

use std::collections::BTreeMap;

use super::ephemeris::{ephemeris_match, geodetic_to_ecef, EphemMatch, GpsSat};

pub const ACQ_THRESHOLD: f32 = 2.5;
pub const MAX_RATE: f64 = 0.9; // Hz/s, GPS L1 Doppler-rate bound

#[derive(Debug, Clone, Copy)]
pub struct Crossing {
    pub t: f64,
    pub dopp: f64,
    pub metric: f32,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct Pass {
    pub prn: u16,
    pub n: usize,
    pub span_min: f64,
    pub rate_hz_s: f64,
    pub r2: f64,
    pub monotonic: bool,
    pub reversals: usize,
    pub max_m: f32,
    pub t_start: f64,
    pub t_end: f64,
    pub confirmed: bool,
    /// physical cross-check of this pass against its own PRN's ephemeris; set
    /// only by the ephemeris-aware verdict, and never changes `confirmed`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ephemeris: Option<EphemMatch>,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct Verdict {
    pub checks: usize,
    pub crossings: usize,
    pub passes: Vec<Pass>,
    pub confirmed: usize,
    pub confirmed_prns: Vec<u16>,
    /// confirmed PRNs whose own ephemeris also backs them (subset of the above);
    /// empty when the verdict was run without a TLE set.
    pub ephemeris_confirmed: Vec<u16>,
    pub max_metric: f32,
}

/// Least-squares line fit: returns (slope, intercept).
fn polyfit1(t: &[f64], d: &[f64]) -> (f64, f64) {
    let n = t.len() as f64;
    let mt = t.iter().sum::<f64>() / n;
    let md = d.iter().sum::<f64>() / n;
    let mut sxx = 0.0;
    let mut sxy = 0.0;
    for i in 0..t.len() {
        let dt = t[i] - mt;
        sxx += dt * dt;
        sxy += dt * (d[i] - md);
    }
    let slope = if sxx > 1e-12 { sxy / sxx } else { 0.0 };
    (slope, md - slope * mt)
}

#[allow(clippy::too_many_arguments)]
pub fn fit_passes(
    per: &BTreeMap<u16, Vec<Crossing>>,
    min_n: usize,
    pass_gap_s: f64,
    min_span_s: f64,
    max_rate: f64,
    max_violations: usize,
    min_r2: f64,
) -> Vec<Pass> {
    let mut out = Vec::new();
    for (&prn, v) in per {
        if v.is_empty() {
            continue;
        }
        let mut cr = v.clone();
        cr.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(std::cmp::Ordering::Equal));

        // split into passes on time gaps
        let mut groups: Vec<Vec<Crossing>> = Vec::new();
        let mut cur = vec![cr[0]];
        for &x in &cr[1..] {
            if x.t - cur.last().unwrap().t > pass_gap_s {
                groups.push(std::mem::take(&mut cur));
            }
            cur.push(x);
        }
        groups.push(cur);

        for gset in groups {
            if gset.len() < min_n {
                continue;
            }
            let t0 = gset[0].t;
            let t: Vec<f64> = gset.iter().map(|x| x.t - t0).collect();
            let d: Vec<f64> = gset.iter().map(|x| x.dopp).collect();
            let span = *t.last().unwrap();
            if span < min_span_s {
                continue;
            }
            let (slope, ic) = polyfit1(&t, &d);
            let md = d.iter().sum::<f64>() / d.len() as f64;
            let mut ss_res = 0.0;
            let mut ss_tot = 0.0;
            for i in 0..t.len() {
                let pred = slope * t[i] + ic;
                ss_res += (d[i] - pred).powi(2);
                ss_tot += (d[i] - md).powi(2);
            }
            let r2 = 1.0 - ss_res / ss_tot.max(1e-9);
            // reversals: min of up-steps and down-steps
            let mut up = 0usize;
            let mut dn = 0usize;
            for i in 1..d.len() {
                if d[i] - d[i - 1] > 0.0 {
                    up += 1;
                } else if d[i] - d[i - 1] < 0.0 {
                    dn += 1;
                }
            }
            let reversals = up.min(dn);
            let monotonic = reversals <= max_violations;
            let confirmed = slope.abs() < max_rate && r2 > min_r2 && monotonic;
            let max_m = gset.iter().map(|x| x.metric).fold(f32::NEG_INFINITY, f32::max);
            out.push(Pass {
                prn,
                n: gset.len(),
                span_min: (span / 60.0 * 10.0).round() / 10.0,
                rate_hz_s: (slope * 1000.0).round() / 1000.0,
                r2: (r2 * 1000.0).round() / 1000.0,
                monotonic,
                reversals,
                max_m,
                t_start: t0,
                t_end: gset.last().unwrap().t,
                confirmed,
                ephemeris: None,
            });
        }
    }
    out
}

/// Xorshift64 PRNG — a deterministic stand-in for the permutation null. The
/// null is a Monte-Carlo estimate, so it need not (and cannot) bit-match numpy's
/// PCG64; a fixed seed keeps the Rust verdict reproducible run to run.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn shuffle<T>(&mut self, v: &mut [T]) {
        for i in (1..v.len()).rev() {
            let j = (self.next() % (i as u64 + 1)) as usize;
            v.swap(i, j);
        }
    }
}

/// Permutation null: shuffle the Doppler values among all crossings (keeping the
/// PRN and timing pattern) and refit. Every pass that survives is false by
/// construction. Returns, per shuffle, the r² of candidates that cleared every
/// gate except linearity — the input to `calibrate_min_r2`. Port of
/// report.py:_null_candidates.
fn null_candidates(
    per: &BTreeMap<u16, Vec<Crossing>>,
    trials: usize,
    seed: u64,
) -> Vec<Vec<f64>> {
    let mut keys: Vec<u16> = Vec::new();
    let mut times: Vec<f64> = Vec::new();
    let mut dopp: Vec<f64> = Vec::new();
    for (&prn, v) in per {
        for c in v {
            keys.push(prn);
            times.push(c.t);
            dopp.push(c.dopp);
        }
    }
    if trials == 0 || dopp.len() < 3 {
        return Vec::new();
    }
    let mut rng = Rng(seed);
    let mut out = Vec::with_capacity(trials);
    for _ in 0..trials {
        rng.shuffle(&mut dopp);
        let mut per_s: BTreeMap<u16, Vec<Crossing>> = BTreeMap::new();
        for i in 0..keys.len() {
            per_s.entry(keys[i]).or_default().push(Crossing {
                t: times[i],
                dopp: dopp[i],
                metric: 0.0,
            });
        }
        // keep every candidate (min_r2 = -1), then filter as report.py does
        let cands = fit_passes(&per_s, 3, 45.0 * 60.0, 300.0, MAX_RATE, 1, -1.0);
        out.push(
            cands
                .iter()
                .filter(|p| p.monotonic && p.rate_hz_s.abs() < 0.9)
                .map(|p| p.r2)
                .collect(),
        );
    }
    out
}

/// Choose the linearity gate that holds the expected false count fixed, port of
/// report.py:calibrate_min_r2. Can only tighten past `floor`, never loosen.
pub fn calibrate_min_r2(null_lists: &[Vec<f64>], target_false: f64, floor: f64) -> f64 {
    if null_lists.is_empty() {
        return floor;
    }
    let trials = null_lists.len();
    let mut allr: Vec<f64> = null_lists.iter().flatten().copied().collect();
    allr.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal)); // descending
    let budget = (target_false * trials as f64).round() as usize;
    if budget >= allr.len() {
        return floor;
    }
    allr[budget].max(floor)
}

/// Build the verdict from a list of per-cycle acquisition results, each stamped
/// with an epoch. Fixed 0.80 linearity gate (no null calibration).
pub fn gnss_verdict(cycles: &[(Vec<crate::gps::AcqResult>, f64)]) -> Verdict {
    verdict_core(cycles, None, 0)
}

/// As `gnss_verdict`, but also annotates each confirmed pass with whether its
/// own PRN's ephemeris backs it (advisory; does not change `confirmed`), and
/// runs the permutation-null gate calibration when `null_trials > 0` (0.5
/// expected-false budget, matching the Python report; 2000 is the report's
/// default, 0 keeps the fixed 0.80 gate).
pub fn gnss_verdict_eph(
    cycles: &[(Vec<crate::gps::AcqResult>, f64)],
    sats: &[GpsSat],
    rx_latlon: (f64, f64),
    null_trials: usize,
) -> Verdict {
    verdict_core(cycles, Some((sats, rx_latlon)), null_trials)
}

fn verdict_core(
    cycles: &[(Vec<crate::gps::AcqResult>, f64)],
    eph: Option<(&[GpsSat], (f64, f64))>,
    null_trials: usize,
) -> Verdict {
    let mut per: BTreeMap<u16, Vec<Crossing>> = BTreeMap::new();
    let mut crossings = 0usize;
    let mut max_metric = 0.0f32;
    for (best, t) in cycles {
        for b in best {
            max_metric = max_metric.max(b.metric);
            if b.metric > ACQ_THRESHOLD {
                per.entry(b.prn as u16).or_default().push(Crossing {
                    t: *t,
                    dopp: b.doppler,
                    metric: b.metric,
                });
                crossings += 1;
            }
        }
    }
    // Choose the linearity gate from this data's own null so the expected number
    // of false detections stays put as the station keeps running.
    let min_r2 = if null_trials > 0 {
        let lists = null_candidates(&per, null_trials, 20260815);
        calibrate_min_r2(&lists, 0.5, 0.80)
    } else {
        0.80
    };
    let mut passes = fit_passes(&per, 3, 45.0 * 60.0, 300.0, MAX_RATE, 1, min_r2);

    let mut ephemeris_confirmed: Vec<u16> = Vec::new();
    if let Some((sats, (lat, lon))) = eph {
        let rx = geodetic_to_ecef(lat, lon, 0.0);
        for p in passes.iter_mut() {
            if !p.confirmed {
                continue;
            }
            // reconstruct this pass's crossings from `per`, as report.py does
            let cr: Vec<(f64, f64)> = per
                .get(&p.prn)
                .map(|v| {
                    v.iter()
                        .filter(|c| c.t >= p.t_start && c.t <= p.t_end)
                        .map(|c| (c.t, c.dopp))
                        .collect()
                })
                .unwrap_or_default();
            let m = ephemeris_match(p.prn, &cr, sats, rx, 0.0, 0.15, 2500.0);
            if m.matched == Some(true) {
                ephemeris_confirmed.push(p.prn);
            }
            p.ephemeris = Some(m);
        }
    }

    let confirmed: Vec<&Pass> = passes.iter().filter(|p| p.confirmed).collect();
    let mut confirmed_prns: Vec<u16> = confirmed.iter().map(|p| p.prn).collect();
    confirmed_prns.sort_unstable();
    confirmed_prns.dedup();
    ephemeris_confirmed.sort_unstable();
    ephemeris_confirmed.dedup();
    Verdict {
        checks: cycles.len(),
        crossings,
        confirmed: confirmed.len(),
        confirmed_prns,
        ephemeris_confirmed,
        max_metric,
        passes,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn real_pass(_prn: u16, n: usize, rate: f64, d0: f64, dt: f64, t0: f64) -> Vec<Crossing> {
        (0..n)
            .map(|i| Crossing {
                t: t0 + i as f64 * dt,
                dopp: d0 + rate * (i as f64 * dt),
                metric: 3.0,
            })
            .collect()
    }

    fn run(per: BTreeMap<u16, Vec<Crossing>>) -> Vec<Pass> {
        fit_passes(&per, 3, 45.0 * 60.0, 300.0, MAX_RATE, 1, 0.80)
    }

    #[test]
    fn a_real_pass_is_confirmed() {
        let mut per = BTreeMap::new();
        per.insert(7, real_pass(7, 8, -0.55, 2200.0, 120.0, 1.7e9));
        let p = &run(per)[0];
        assert!(p.confirmed && p.monotonic && p.r2 > 0.99);
        assert!((p.rate_hz_s - (-0.55)).abs() < 0.01);
    }

    #[test]
    fn too_few_crossings_is_not_a_detection() {
        let mut per = BTreeMap::new();
        per.insert(7, real_pass(7, 2, -0.55, 2200.0, 120.0, 1.7e9));
        assert!(run(per).is_empty());
    }

    #[test]
    fn too_short_a_span_is_rejected() {
        let mut per = BTreeMap::new();
        per.insert(7, real_pass(7, 8, -0.55, 2200.0, 12.0, 1.7e9)); // 84 s
        assert!(run(per).is_empty());
    }

    #[test]
    fn rate_faster_than_physics_is_rejected() {
        let mut per = BTreeMap::new();
        per.insert(7, real_pass(7, 8, -5.0, 2200.0, 120.0, 1.7e9));
        let p = &run(per)[0];
        assert!(!p.confirmed && p.rate_hz_s.abs() > MAX_RATE);
    }

    #[test]
    fn non_monotonic_doppler_is_rejected() {
        let d = [2000.0, 1000.0, 0.0, -1000.0, 0.0, 1000.0, 2000.0, 3000.0];
        let cr: Vec<Crossing> = d
            .iter()
            .enumerate()
            .map(|(i, &dv)| Crossing { t: 1.7e9 + 120.0 * i as f64, dopp: dv, metric: 3.0 })
            .collect();
        let mut per = BTreeMap::new();
        per.insert(7, cr);
        let p = &run(per)[0];
        assert!(!p.confirmed && !p.monotonic);
    }

    #[test]
    fn scattered_doppler_with_strong_metric_is_rejected() {
        // unrelated Doppler each look, high metric -> must not confirm
        let vals = [4200.0, -1300.0, 900.0, -4800.0, 2600.0, -200.0, 3900.0, -3100.0];
        let cr: Vec<Crossing> = vals
            .iter()
            .enumerate()
            .map(|(i, &dv)| Crossing { t: 1.7e9 + 120.0 * i as f64, dopp: dv, metric: 9.9 })
            .collect();
        let mut per = BTreeMap::new();
        per.insert(7, cr);
        assert!(!run(per)[0].confirmed);
    }

    #[test]
    fn two_separate_passes_are_not_one() {
        let mut per = BTreeMap::new();
        let mut cr = real_pass(9, 6, -0.5, 2200.0, 120.0, 1.7e9);
        cr.extend(real_pass(9, 6, -0.5, 2200.0, 120.0, 1.7e9 + 4.0 * 3600.0));
        per.insert(9, cr);
        let passes = run(per);
        assert_eq!(passes.iter().filter(|p| p.prn == 9).count(), 2);
    }

    #[test]
    fn calibration_never_falls_below_the_floor() {
        assert_eq!(calibrate_min_r2(&[], 0.5, 0.80), 0.80);
        assert_eq!(calibrate_min_r2(&vec![vec![]; 100], 0.5, 0.80), 0.80);
        assert_eq!(calibrate_min_r2(&vec![vec![0.1]; 100], 0.5, 0.80), 0.80);
    }

    #[test]
    fn calibration_tightens_as_candidates_accumulate() {
        let few = vec![vec![0.99, 0.95]; 100];
        let many = vec![vec![0.99, 0.98, 0.97, 0.96, 0.95]; 100];
        let a = calibrate_min_r2(&few, 0.5, 0.5);
        let b = calibrate_min_r2(&many, 0.5, 0.5);
        assert!(b >= a, "more candidates must not loosen the gate: {a} -> {b}");
    }

    fn cycles_from(cr: &[Crossing]) -> Vec<(Vec<crate::gps::AcqResult>, f64)> {
        cr.iter()
            .map(|c| {
                (
                    vec![crate::gps::AcqResult {
                        prn: 1,
                        metric: c.metric,
                        pk_floor: 10.0,
                        doppler: c.dopp,
                        code_phase: 0.0,
                        acquired: true,
                    }],
                    c.t,
                )
            })
            .collect()
    }

    #[test]
    fn verdict_with_ephemeris_and_null_runs_all_paths() {
        // a real GPS TLE (PRN 1) so the ephemeris branch has a satellite to check,
        // plus null_trials>0 to drive the permutation-null calibration path.
        let text = "GPS BIII-7  (PRN 01)\n\
                    1 62339U 24242A   26226.61674491 -.00000103  00000+0  00000+0 0  9996\n\
                    2 62339  54.8202 331.8118 0018431   3.0031 316.8707  2.00572831 12424\n";
        let sats = crate::gps::load_tle(text);
        assert_eq!(sats.len(), 1);
        let cr = real_pass(1, 8, -0.55, 2200.0, 120.0, 1.7e9);
        let cycles = cycles_from(&cr);
        let v = gnss_verdict_eph(&cycles, &sats, (40.65, -73.80), 200);
        // statistical decision still stands, ephemeris annotation is attached
        assert_eq!(v.confirmed, 1);
        let p = v.passes.iter().find(|p| p.confirmed).unwrap();
        assert!(p.ephemeris.is_some(), "ephemeris annotation present");
        // the permutation null ran (calibration used a non-floor threshold or floor)
        let _ = v.ephemeris_confirmed;
    }

    #[test]
    fn verdict_counts_and_confirms() {
        // one real pass spread across 8 cycles -> confirmed once
        let cr = real_pass(7, 8, -0.55, 2200.0, 120.0, 1.7e9);
        let cycles: Vec<(Vec<crate::gps::AcqResult>, f64)> = cr
            .iter()
            .map(|c| {
                (
                    vec![crate::gps::AcqResult {
                        prn: 7,
                        metric: c.metric,
                        pk_floor: 10.0,
                        doppler: c.dopp,
                        code_phase: 0.0,
                        acquired: true,
                    }],
                    c.t,
                )
            })
            .collect();
        let v = gnss_verdict(&cycles);
        assert_eq!(v.confirmed, 1);
        assert_eq!(v.confirmed_prns, vec![7]);
        assert_eq!(v.checks, 8);
    }
}
