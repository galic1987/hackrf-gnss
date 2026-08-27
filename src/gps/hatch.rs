//! Hatch carrier-smoothing of pseudoranges (sub-ns Leg 1, spec 2026-08-27).
//!
//! One filter per satellite per epoch:
//!   S(k) = (1/n)·P(k) + ((n-1)/n)·[S(k-1) + λ·(Φ(k) - Φ(k-1))],  n → window
//! with S(0) = P(0). Carrier carries the dynamics; code pins the absolute.
//! Any slip/generation change in the carrier series must call update with
//! reset = true — a smoothed value across a discontinuity is worse than raw.

#[derive(Debug, Clone)]
pub struct Hatch {
    n: f64,
    window: f64,
    smoothed: Option<f64>,
    last_carrier: Option<f64>,
}

impl Hatch {
    pub fn new(window_epochs: f64) -> Self {
        Self { n: 0.0, window: window_epochs, smoothed: None, last_carrier: None }
    }

    pub fn update(&mut self, code_m: f64, carrier_cycles: f64, lam: f64, reset: bool) -> f64 {
        if reset || self.smoothed.is_none() || self.last_carrier.is_none() {
            self.n = 1.0;
            self.smoothed = Some(code_m);
            self.last_carrier = Some(carrier_cycles);
            return code_m;
        }
        let d_m = (carrier_cycles - self.last_carrier.unwrap()) * lam;
        self.last_carrier = Some(carrier_cycles);
        self.n = (self.n + 1.0).min(self.window);
        let s = self.smoothed.unwrap() + d_m;
        let out = code_m / self.n + s * (self.n - 1.0) / self.n;
        self.smoothed = Some(out);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const LAM: f64 = 0.190293672798; // L1 wavelength m
    const R0: f64 = 20_000_000.0;

    #[test]
    fn exact_on_consistent_data() {
        // truth drifts 19.029 m/epoch = exactly 100 L1 cycles/epoch
        let mut h = Hatch::new(100.0);
        for k in 0..300u64 {
            let r = R0 + 19.029 * k as f64;
            let carr = (r - R0) / LAM;
            let out = h.update(r, carr, LAM, false);
            assert!((out - r).abs() < 1e-6, "k={k} out={out} r={r}");
        }
    }

    #[test]
    fn tracks_consistent_step_immediately() {
        // truth steps an extra +19.0294 m at k=200; carrier steps with it
        let mut h = Hatch::new(100.0);
        for k in 0..300u64 {
            let step = if k >= 200 { 19.0293672798 } else { 0.0 };
            let r = R0 + 19.029 * k as f64 + step;
            let carr = (r - R0) / LAM;
            let out = h.update(r, carr, LAM, false);
            assert!((out - r).abs() < 1e-6, "k={k} out={out} r={r}");
        }
    }

    #[test]
    fn reset_flushes_state() {
        let mut h = Hatch::new(100.0);
        for k in 0..200u64 {
            let r = R0 + 19.029 * k as f64;
            h.update(r, (r - R0) / LAM, LAM, false);
        }
        // slip: carrier jumps arbitrarily; reset must re-init from code
        let v = h.update(R0 + 500.0, 555_555.0, LAM, true);
        assert_eq!(v, R0 + 500.0);
    }

    #[test]
    fn noise_var_shrinks_vs_raw() {
        // deterministic pseudo-noise on code only; smoother must cut its spread
        let mut h = Hatch::new(100.0);
        let mut raw = Vec::new();
        let mut sm = Vec::new();
        for k in 0..300u64 {
            let noise = (k.wrapping_mul(2654435761) % 2000) as f64 / 100.0 - 10.0;
            let r = R0 + 19.029 * k as f64;
            let out = h.update(r + noise, (r - R0) / LAM, LAM, false);
            if k >= 200 { raw.push(noise); sm.push(out - r); }
        }
        let spread = |v: &[f64]| {
            let m = v.iter().sum::<f64>() / v.len() as f64;
            (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / v.len() as f64).sqrt()
        };
        assert!(spread(&sm) < spread(&raw) * 0.3,
                "raw {} sm {}", spread(&raw), spread(&sm));
    }
}
