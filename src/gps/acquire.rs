//! Parallel-code-phase GPS L1 C/A acquisition, a port of
//! `validation/acquire.py:acquire`.
//!
//! For each Doppler hypothesis the baseband signal is de-rotated, correlated
//! against the local code by circular convolution (FFT * conj(code FFT), IFFT),
//! and the block magnitudes are summed non-coherently. The reported metric is
//! peak / second-highest-peak with the winning peak's +/-1 chip excluded — the
//! same ratio the Python oracle uses, so it is invariant to the (unnormalised)
//! FFT scaling rustfft applies.

use num_complex::Complex;
use rustfft::{Fft, FftPlanner};
use std::f64::consts::PI;
use std::sync::Arc;

use super::ca_code::{gps_ca, resample_code, CHIP_RATE};
use super::sbas_code::sbas_code;

/// GPS/SBAS L1 carrier, used to couple code-rate drift to the Doppler hypothesis.
pub const F_L1: f64 = 1575.42e6;

#[derive(Debug, Clone, serde::Serialize)]
pub struct AcqResult {
    pub prn: usize,
    /// peak / second-peak (the acquisition metric; > ~2.5 is a detection)
    pub metric: f32,
    /// peak / median-floor, a coarser SNR proxy
    pub pk_floor: f32,
    pub doppler: f64,
    /// code phase in chips (0..1023)
    pub code_phase: f64,
    pub acquired: bool,
}

fn median(v: &mut [f32]) -> f32 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    if v.is_empty() {
        0.0
    } else {
        v[v.len() / 2]
    }
}

/// Acquire GPS L1 C/A over `sig` (baseband IQ at `fs`), with code-Doppler
/// compensation on — long non-coherent integration then actually helps instead
/// of smearing the peak across chips.
pub fn acquire(
    sig: &[Complex<f32>],
    fs: f64,
    prns: &[usize],
    dopplers: &[f64],
    nblocks: usize,
    thresh: f32,
) -> Vec<AcqResult> {
    let codes: Vec<(usize, Vec<f32>)> = prns.iter().map(|&p| (p, gps_ca(p))).collect();
    acquire_codes(sig, fs, &codes, CHIP_RATE, dopplers, nblocks, thresh, F_L1, true)
}

/// Acquire SBAS (WAAS/EGNOS/...) L1 birds. Same 1.023 Mcps L1 code family; GEO
/// Doppler is near zero, so a narrow `dopplers` range around the clock offset is
/// enough, and long integration (compensated) pays off because the code phase is
/// stable.
pub fn acquire_sbas(
    sig: &[Complex<f32>],
    fs: f64,
    prns: &[usize],
    dopplers: &[f64],
    nblocks: usize,
    thresh: f32,
) -> Vec<AcqResult> {
    let codes: Vec<(usize, Vec<f32>)> = prns.iter().map(|&p| (p, sbas_code(p))).collect();
    acquire_codes(sig, fs, &codes, CHIP_RATE, dopplers, nblocks, thresh, F_L1, true)
}

/// Acquire an arbitrary set of (PRN, code) replicas. PRNs run in parallel; each
/// reuses its own FFT planner (rustfft planners are not Sync). `f_carrier` and
/// `compensate` control the per-block code-Doppler correction.
#[allow(clippy::too_many_arguments)]
pub fn acquire_codes(
    sig: &[Complex<f32>],
    fs: f64,
    prn_codes: &[(usize, Vec<f32>)],
    chiprate: f64,
    dopplers: &[f64],
    nblocks: usize,
    thresh: f32,
    f_carrier: f64,
    compensate: bool,
) -> Vec<AcqResult> {
    use rayon::prelude::*;
    let ns = (fs / 1000.0).round() as usize;

    prn_codes
        .par_iter()
        .map(|(prn, code)| {
            let mut planner = FftPlanner::<f32>::new();
            let fft = planner.plan_fft_forward(ns);
            let ifft = planner.plan_fft_inverse(ns);
            let (metric, doppler, code_phase, pk_floor) = acquire_one(
                sig, fs, code, chiprate, dopplers, nblocks, &fft, &ifft, ns, f_carrier, compensate,
            );
            AcqResult {
                prn: *prn,
                metric,
                pk_floor,
                doppler,
                code_phase,
                acquired: metric >= thresh,
            }
        })
        .collect()
}

/// Acquire one PRN. `sig` is baseband IQ at `fs`; `dopplers` in Hz.
/// Returns (metric = peak/2nd, winning doppler Hz, code phase chips, peak/floor).
#[allow(clippy::too_many_arguments)]
fn acquire_one(
    sig: &[Complex<f32>],
    fs: f64,
    code: &[f32],
    chiprate: f64,
    dopplers: &[f64],
    nblocks: usize,
    fft: &Arc<dyn Fft<f32>>,
    ifft: &Arc<dyn Fft<f32>>,
    ns: usize,
    f_carrier: f64,
    compensate: bool,
) -> (f32, f64, f64, f32) {
    let lc = resample_code(code, ns, chiprate, fs);
    let mut cf: Vec<Complex<f32>> = lc.iter().map(|&v| Complex::new(v, 0.0)).collect();
    fft.process(&mut cf);
    for c in cf.iter_mut() {
        *c = c.conj();
    }

    let nb = nblocks.min(sig.len() / ns);
    let mut best_peak = f32::NEG_INFINITY;
    let mut best_ci = 0usize;
    let mut best_d = 0.0f64;
    let mut best_row: Vec<f32> = vec![0.0; ns];
    let mut scratch: Vec<Complex<f32>> = vec![Complex::new(0.0, 0.0); ns];

    for &d in dopplers {
        let mut acc = vec![0.0f32; ns];
        // code-Doppler coupling: the code rate is scaled by (1 + d/f_carrier), so
        // over block b the correlation peak drifts by b*ns*d/f_carrier samples.
        // Roll each block's correlation back onto a common code phase before
        // summing, so long integration reinforces the peak instead of smearing it.
        let drift_per_block = if compensate { ns as f64 * d / f_carrier } else { 0.0 };
        for b in 0..nb {
            let blk = &sig[b * ns..(b + 1) * ns];
            for (i, s) in blk.iter().enumerate() {
                let k = (b * ns + i) as f64;
                let ph = -2.0 * PI * d * k / fs;
                scratch[i] = *s * Complex::new(ph.cos() as f32, ph.sin() as f32);
            }
            fft.process(&mut scratch);
            for i in 0..ns {
                scratch[i] *= cf[i];
            }
            ifft.process(&mut scratch);
            let shift = (b as f64 * drift_per_block).round() as isize;
            if shift == 0 {
                for i in 0..ns {
                    acc[i] += scratch[i].norm_sqr();
                }
            } else {
                let n = ns as isize;
                for i in 0..ns {
                    let src = (i as isize - shift).rem_euclid(n) as usize;
                    acc[i] += scratch[src].norm_sqr();
                }
            }
        }
        let (ci, &pk) = acc
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap();
        if pk > best_peak {
            best_peak = pk;
            best_ci = ci;
            best_d = d;
            best_row.copy_from_slice(&acc);
        }
    }

    let excl = (fs / chiprate).ceil() as isize;
    let mut masked: Vec<f32> = Vec::with_capacity(ns);
    for (i, &v) in best_row.iter().enumerate() {
        let mut dd = (i as isize) - (best_ci as isize);
        if dd > ns as isize / 2 {
            dd -= ns as isize;
        }
        if dd < -(ns as isize) / 2 {
            dd += ns as isize;
        }
        if dd.abs() > excl {
            masked.push(v);
        }
    }
    let second = masked.iter().copied().fold(f32::NEG_INFINITY, f32::max).max(1e-30);
    let floor = median(&mut masked).max(1e-30);
    (
        best_peak / second,
        best_d,
        best_ci as f64 * chiprate / fs,
        best_peak / floor,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a clean baseband C/A signal for `prn` at a known Doppler and code
    /// phase, then confirm acquisition recovers both and the wrong PRN does not.
    #[test]
    fn acquires_a_synthetic_prn() {
        let fs = 4.0e6;
        let ns = (fs / 1000.0) as usize;
        let prn = 19;
        let code = gps_ca(prn);
        let true_dopp = 1250.0f64;
        let phase_chips = 300.0f64;
        let phase_samp = (phase_chips * fs / CHIP_RATE) as usize;

        let nblocks = 10;
        let mut sig = vec![Complex::<f32>::new(0.0, 0.0); ns * nblocks];
        let lc = resample_code(&code, ns, CHIP_RATE, fs);
        for k in 0..sig.len() {
            let c = lc[(k + ns - (phase_samp % ns)) % ns]; // shift code by phase
            let ph = 2.0 * PI * true_dopp * (k as f64) / fs;
            let carrier = Complex::new(ph.cos() as f32, ph.sin() as f32);
            sig[k] = Complex::new(c, 0.0) * carrier;
        }

        let dopplers: Vec<f64> = (-3000..=3000).step_by(250).map(|x| x as f64).collect();
        let res = acquire(&sig, fs, &[prn], &dopplers, nblocks, 2.5);
        let r = &res[0];
        assert!(r.metric > 3.0, "clean signal should acquire, metric={}", r.metric);
        assert!((r.doppler - true_dopp).abs() <= 250.0, "doppler {}", r.doppler);
        // code phase recovered within a chip (wrap-aware)
        let dphi = ((r.code_phase - phase_chips + 1023.0) % 1023.0).min(
            (phase_chips - r.code_phase + 1023.0) % 1023.0,
        );
        assert!(dphi <= 1.5, "code phase {} vs {}", r.code_phase, phase_chips);

        // a different PRN must not acquire on the same data
        let wrong = acquire(&sig, fs, &[20], &dopplers, nblocks, 2.5);
        assert!(wrong[0].metric < 2.0, "wrong PRN metric={}", wrong[0].metric);
    }

    #[test]
    fn code_doppler_compensation_helps_long_integration() {
        // synthesize a satellite whose code rate is scaled by (1 + d/f_L1) — the
        // code phase drifts over the record. Over many blocks, compensation off
        // smears the peak; compensation on keeps it aligned.
        let fs = 4.0e6;
        let ns = (fs / 1000.0) as usize;
        let prn = 7;
        let code = gps_ca(prn);
        let d = 5000.0f64; // large Doppler -> significant code drift
        let nblocks = 120;
        let chip_scale = CHIP_RATE * (1.0 + d / F_L1);
        let mut sig = vec![Complex::<f32>::new(0.0, 0.0); ns * nblocks];
        for k in 0..sig.len() {
            // code sampled at the Doppler-stretched chip rate
            let ci = ((k as f64) * chip_scale / fs) as usize % 1023;
            let ph = 2.0 * PI * d * (k as f64) / fs;
            sig[k] = Complex::new(code[ci], 0.0) * Complex::new(ph.cos() as f32, ph.sin() as f32);
        }
        let dopp: Vec<f64> = (4000..=6000).step_by(250).map(|x| x as f64).collect();
        let codes = vec![(prn, code)];
        let off = acquire_codes(&sig, fs, &codes, CHIP_RATE, &dopp, nblocks, 2.5, F_L1, false);
        let on = acquire_codes(&sig, fs, &codes, CHIP_RATE, &dopp, nblocks, 2.5, F_L1, true);
        assert!(
            on[0].metric > off[0].metric,
            "compensation should raise the metric: off {} vs on {}",
            off[0].metric, on[0].metric
        );
    }

    #[test]
    fn pure_noise_stays_below_threshold() {
        let fs = 4.0e6;
        let ns = (fs / 1000.0) as usize;
        let nblocks = 10;
        // deterministic pseudo-noise (no rng dependency)
        let mut state = 0x2545f491_4f6cdd1du64;
        let mut nxt = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            ((state >> 40) as f32 / 8_388_608.0) - 1.0
        };
        let sig: Vec<Complex<f32>> = (0..ns * nblocks)
            .map(|_| Complex::new(nxt(), nxt()))
            .collect();
        let dopplers: Vec<f64> = (-3000..=3000).step_by(250).map(|x| x as f64).collect();
        let res = acquire(&sig, fs, &[5], &dopplers, nblocks, 2.5);
        assert!(res[0].metric < 2.0, "noise metric={}", res[0].metric);
    }
}
