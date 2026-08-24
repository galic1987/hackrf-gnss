//! Minimal GPS L1 C/A tracking loop: a carrier Costas PLL + an early/late DLL,
//! carrier-aided code NCO. The decisive output is BIT SYNC — a genuine GPS
//! signal carries 50 bps nav data whose prompt-correlator sign flips only on
//! 20 ms boundaries. Noise has no such structure, so if the prompt sign
//! transitions cluster at one 20 ms phase, the signal is real GPS.
//!
//! This is not a full receiver (no nav-message decode / PVT); it is the
//! tracking + bit-sync stage that proves a signal is a satellite and is the
//! gateway to LNAV decode.

use num_complex::Complex;
use std::f64::consts::PI;

use super::acquire::F_L1;
use super::ca_code::{gps_ca, CHIP_RATE};


#[derive(Debug, Clone, serde::Serialize)]
pub struct TrackResult {
    pub prn: usize,
    pub epochs: usize,
    /// mean prompt correlation power over noise-floor proxy
    pub cn0_dbhz: f32,
    /// fraction of prompt-sign transitions falling on the winning 20 ms phase
    pub bit_sync_score: f32,
    pub bit_sync: bool,
    /// carrier frequency at the end (Hz) — should stay near the acquired Doppler
    pub final_doppler: f64,
    /// number of nav bits recovered (prompt sign per 20 ms)
    pub nav_bits: usize,
    /// winning 20 ms bit-sync phase (ms): returned nav bit k sits at
    /// (bit_phase_ms + 20*k) ms in the tracked stream
    pub bit_phase_ms: usize,
}

/// Track `prn` in `sig` (baseband IQ at `fs`) starting from an acquired Doppler
/// (Hz) and code phase (chips). Runs `epochs` 1 ms integrations.
pub fn track(
    sig: &[Complex<f32>],
    fs: f64,
    prn: usize,
    dopp0: f64,
    code_phase0_chips: f64,
    epochs: usize,
) -> TrackResult {
    track_full(sig, fs, prn, dopp0, code_phase0_chips, epochs).0
}

/// Same as `track`, but also returns the recovered nav bits (0/1 per 20 ms).
pub fn track_full(
    sig: &[Complex<f32>],
    fs: f64,
    prn: usize,
    dopp0: f64,
    code_phase0_chips: f64,
    epochs: usize,
) -> (TrackResult, Vec<u8>) {
    let code = gps_ca(prn);
    let ns = (fs / 1000.0).round() as usize;

    // loop states
    let mut carrier_phase = 0.0f64; // rad
    let mut carrier_freq = dopp0; // Hz
    let mut code_phase = code_phase0_chips; // chips (0..1023), where in the code we are
    const PDI: f64 = 0.001; // 1 ms integration
    // 2nd-order Costas PLL (Borre coefficients) around the acquired Doppler, plus
    // a 1st-order carrier-aided DLL. A ~10 Hz PLL is a compromise between pull-in
    // and noise tolerance at the marginal C/N0 of a window signal.
    let (pll_t1, pll_t2) = borre(10.0, 0.7, 0.25);
    let carr_basis = dopp0;
    let mut carr_nco = 0.0f64;
    let mut old_carr_err = 0.0f64;
    let dll_k = 1.0; // direct code-phase nudge per epoch

    let spacing = 0.5f64; // early/late correlator spacing in chips
    let mut prompt_i: Vec<f64> = Vec::with_capacity(epochs);
    let mut prompt_pwr = 0.0f64;
    let mut noise_pwr = 0.0f64;

    let n_epoch = epochs.min(sig.len() / ns);
    for _ in 0..n_epoch {
        let (mut ie, mut qe) = (0.0f64, 0.0f64);
        let (mut ip, mut qp) = (0.0f64, 0.0f64);
        let (mut il, mut ql) = (0.0f64, 0.0f64);
        // code chips advanced per sample, carrier-aided (code Doppler = carrier
        // Doppler scaled by chiprate/carrier)
        let code_rate = CHIP_RATE * (1.0 + carrier_freq / F_L1); // chips/s
        let code_step = code_rate / fs; // chips/sample
        let dphi = 2.0 * PI * carrier_freq / fs; // rad/sample

        let base = (prompt_i.len()) * ns;
        for k in 0..ns {
            let s = sig[base + k];
            // wipe carrier
            let ph = carrier_phase + dphi * k as f64;
            let (sinp, cosp) = ph.sin_cos();
            let bi = s.re as f64 * cosp + s.im as f64 * sinp;
            let bq = -(s.re as f64) * sinp + s.im as f64 * cosp;
            // code replicas E/P/L
            let cp = code_phase + code_step * k as f64;
            let ce = chip(&code, cp - spacing);
            let cpr = chip(&code, cp);
            let cl = chip(&code, cp + spacing);
            ie += bi * ce;
            qe += bq * ce;
            ip += bi * cpr;
            qp += bq * cpr;
            il += bi * cl;
            ql += bq * cl;
        }
        // advance code phase / carrier phase across the epoch
        code_phase = (code_phase + code_step * ns as f64).rem_euclid(1023.0);
        carrier_phase = (carrier_phase + dphi * ns as f64).rem_euclid(2.0 * PI);

        // Costas discriminator: Q*sign(I) normalised -> phase error in cycles,
        // bounded (data-bit insensitive). Borre 2nd-order loop filter around the
        // acquired Doppler basis.
        let norm = (ip * ip + qp * qp).sqrt().max(1e-12);
        let carr_err = (qp * ip.signum()) / norm / (2.0 * PI);
        carr_nco += (pll_t2 / pll_t1) * (carr_err - old_carr_err) + carr_err * (PDI / pll_t1);
        old_carr_err = carr_err;
        carrier_freq = carr_basis + carr_nco;

        // DLL: normalized early-minus-late power, nudge the code phase directly
        let e = (ie * ie + qe * qe).sqrt();
        let l = (il * il + ql * ql).sqrt();
        let dll_err = if e + l > 1e-12 { 0.5 * (e - l) / (e + l) } else { 0.0 };
        code_phase = (code_phase - dll_k * dll_err).rem_euclid(1023.0);

        prompt_i.push(ip);
        prompt_pwr += ip * ip + qp * qp;
        noise_pwr += qp * qp; // quadrature ~ noise once locked
    }

    // ---- bit sync: prompt-sign transitions should land on 20 ms boundaries ----
    let mut trans = [0usize; 20];
    let mut ntrans = 0usize;
    for i in 1..prompt_i.len() {
        if prompt_i[i].signum() != prompt_i[i - 1].signum() {
            trans[i % 20] += 1;
            ntrans += 1;
        }
    }
    let (best_phase, best_cnt) = trans
        .iter()
        .copied()
        .enumerate()
        .max_by_key(|&(_, c)| c)
        .unwrap_or((0, 0));
    let bit_sync_score = if ntrans > 0 { best_cnt as f32 / ntrans as f32 } else { 0.0 };
    // real GPS: nearly all transitions on one phase; noise: spread ~uniformly (1/20)
    let bit_sync = ntrans >= 3 && bit_sync_score > 0.5;

    // nav bits: integrated prompt sign per 20 ms block from the synced phase
    let mut bits = Vec::new();
    if bit_sync {
        let mut i = best_phase;
        while i + 20 <= prompt_i.len() {
            let s: f64 = prompt_i[i..i + 20].iter().sum();
            bits.push(if s >= 0.0 { 1u8 } else { 0u8 });
            i += 20;
        }
    }
    let nav_bits = bits.len();

    let mean_p = prompt_pwr / n_epoch.max(1) as f64;
    let mean_n = (noise_pwr / n_epoch.max(1) as f64).max(1e-12);
    let cn0 = 10.0 * (mean_p / mean_n).log10() + 30.0; // rough proxy (1 ms -> dB-Hz)

    (TrackResult {
        prn,
        epochs: n_epoch,
        cn0_dbhz: cn0 as f32,
        bit_sync_score,
        bit_sync,
        final_doppler: carrier_freq,
        nav_bits,
        bit_phase_ms: if bit_sync { best_phase } else { 0 },
    }, bits)
}

/// Linear-interpolated code chip at fractional chip index (code repeats every 1023).
#[inline]
fn chip(code: &[f32], c: f64) -> f64 {
    let idx = c.rem_euclid(1023.0) as usize % 1023;
    code[idx] as f64
}

/// Borre 2nd-order loop-filter time constants (tau1, tau2) for a given noise
/// bandwidth `bn` (Hz), damping `zeta`, and loop gain `k`.
fn borre(bn: f64, zeta: f64, k: f64) -> (f64, f64) {
    let wn = bn * 8.0 * zeta / (4.0 * zeta * zeta + 1.0);
    let tau1 = k / (wn * wn);
    let tau2 = 2.0 * zeta / wn;
    (tau1, tau2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::ca_code::resample_code;

    #[test]
    fn bit_sync_locks_on_a_synthetic_gps_signal_with_nav_data() {
        // build a clean PRN with 50 bps nav bits (sign flips every 20 ms) and a
        // small Doppler; tracking must lock and report bit sync.
        let fs = 4.0e6;
        let ns = (fs / 1000.0) as usize;
        let prn = 11;
        let code = gps_ca(prn);
        let lc = resample_code(&code, ns, CHIP_RATE, fs);
        let dopp = 800.0f64;
        let epochs = 400; // 400 ms -> 20 nav bits
        let mut sig = vec![Complex::<f32>::new(0.0, 0.0); ns * epochs];
        // one nav-bit sign per 20 ms block
        let mut blocks = vec![1.0f32; epochs / 20 + 1];
        let mut s2 = 0x1234567u64;
        for b in blocks.iter_mut() {
            s2 ^= s2 << 13; s2 ^= s2 >> 7; s2 ^= s2 << 17;
            *b = if (s2 >> 63) & 1 == 1 { 1.0 } else { -1.0 };
        }
        for e in 0..epochs {
            let data = blocks[e / 20];
            for k in 0..ns {
                let g = e * ns + k;
                let c = lc[g % ns] * data;
                let ph = 2.0 * PI * dopp * (g as f64) / fs;
                sig[g] = Complex::new(c, 0.0) * Complex::new(ph.cos() as f32, ph.sin() as f32);
            }
        }
        let r = track(&sig, fs, prn, dopp, 0.0, epochs);
        assert!(r.bit_sync, "should bit-sync on clean GPS: score {}", r.bit_sync_score);
        assert!(r.nav_bits >= 15, "should recover ~20 nav bits, got {}", r.nav_bits);
    }
}
