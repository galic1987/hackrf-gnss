//! Clock-discipline scaffolding for `examples/clock_loop.rs`: the trait every
//! rate reference implements, the correction accumulator, and the log-line
//! schema. Iridium is the first source; GPS/SBAS nav, Inmarsat STD-C carrier
//! and Galileo/BeiDou plug in behind the same trait later.

/// A signal source that can measure the receiver's clock error.
pub trait RateReference {
    fn name(&self) -> &str;
    /// One measurement cycle. Returns the measured clock error in ppm
    /// (SIGN: positive = receiver clock runs FAST, measured carriers higher
    /// than predicted), or None when the cycle produced no usable observation
    /// -- a None must never move the correction.
    fn estimate_ppm(&mut self) -> Option<f64>;
    /// observations behind the last Some estimate, for logging
    fn n_obs(&self) -> usize {
        0
    }
    /// raw detections behind the last cycle, for logging (not all
    /// detections decode/attribute into observations)
    fn n_detected(&self) -> usize {
        0
    }
    /// identities of the sources behind the last estimate, for logging
    fn sources(&self) -> Vec<String> {
        Vec::new()
    }
}

/// The accumulating clock-correction state.
///
/// SIGN: the correction sent to the radio zeroes the measured error, so
/// `delta = -measured_ppm` and `new_correction = current + delta`. Verified
/// on hardware: a measured -26.00 ppm was zeroed by correction +26.00, and a
/// later residual of -0.92 ppm calls for delta +0.92.
pub struct Correction {
    pub ppm: f64,
}

impl Correction {
    pub fn new(ppm: f64) -> Self {
        Correction { ppm }
    }
    /// Fold one combined measurement into the correction. Returns the delta
    /// applied, or None (state untouched) when the cycle had no estimate.
    pub fn update(&mut self, measured_ppm: Option<f64>) -> Option<f64> {
        let delta = -measured_ppm?;
        self.ppm += delta;
        Some(delta)
    }
}

/// One line of `clock_loop_log.jsonl`.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CycleLog {
    /// cycle start, unix seconds
    pub t: f64,
    /// combined measured clock error this cycle (null if no source produced one)
    pub measured_ppm: Option<f64>,
    /// change applied to the correction this cycle (null if none applied)
    pub delta_ppm: Option<f64>,
    /// correction in effect after this cycle
    pub correction_ppm: f64,
    /// bursts detected this cycle (before decode/attribution)
    pub n_detected: usize,
    /// bursts attributed to a satellite this cycle — the observations
    /// actually behind measured_ppm
    pub n_attributed: usize,
    pub sats: Vec<String>,
}

// ---------------------------------------------------------------------------
// residual-plausibility gate (in-process loop, examples/live_radio.rs)
// ---------------------------------------------------------------------------

/// Window (discipline cycles) the collapse-reference locked-count max is
/// taken over: ~10 min at the 60 s cadence.
pub const GATE_WINDOW: usize = 10;
/// Collapse threshold: a write is suppressed when the current locked-channel
/// count falls below max(2, this fraction of the recent-window max). A
/// tracker whose locks are collapsing produces residuals dominated by dying
/// channels — the 2026-08-24/25 incident stepped -0.1 ppm three times on a
/// bogus -0.37 ppm residual measured exactly then (docs/p0c_clock_continuity.md).
pub const GATE_COLLAPSE_FRAC: f64 = 0.5;
/// Fresh-lock threshold (s): a WAAS/GEO channel younger than this just came
/// out of a collapse/relock; its Doppler must not steer the clock yet.
pub const GATE_FRESH_LOCK_S: f64 = 30.0;
/// Max plausible change vs the previous ACCEPTED residual (ppm per 60 s
/// cycle). The TCXO cannot move 0.3 ppm in minutes indoors; 0.15 is an
/// order of magnitude above the observed thermal wander (the live residual
/// stayed within [-0.48, -0.34] ppm over 6.6 h) yet half the bogus jump.
pub const GATE_MAX_SLEW_PPM: f64 = 0.15;
/// Measurement inputs older than this are stale (channel reports are 1 Hz;
/// a dying tracker stops emitting long before the discipline cycle notices).
pub const GATE_STALE_S: f64 = 10.0;
/// Absolute sanity bound on the residual (ppm), applied to EVERY sample
/// including the first. The firmware clamp is ±1 % (±10 000 ppm), but this
/// TCXO's raw drift lives well inside ±1 ppm — 2 ppm is a generous bound
/// that still rejects NaN-adjacent garbage and unit bugs outright.
pub const GATE_ABS_BOUND_PPM: f64 = 2.0;

/// Residual-plausibility gate for the in-process discipline loop
/// (examples/live_radio.rs): suppresses a correction write when the
/// measurement context is untrustworthy. Added after the 2026-08-24/25
/// incident, where the loop stepped on a bogus residual measured while the
/// tracker was dying, then walked the correction back over ~8 min.
#[derive(Debug, Default)]
pub struct PlausibilityGate {
    /// locked SBAS/WAAS channel counts of the recent cycles (the collapse
    /// reference); recorded EVERY cycle, gated or not — the reference must
    /// track the tracker. SBAS-only because the residual the gate protects
    /// is WAAS-derived; an all-band count would mask a GEO collapse behind
    /// healthy GPS/B1I locks.
    sbas_hist: std::collections::VecDeque<usize>,
    /// previous ACCEPTED residual (the slew reference); a suppressed
    /// residual never becomes the reference, or a bogus value would
    /// legitimize the next bogus one
    prev_resid: Option<f64>,
}

impl PlausibilityGate {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record the locked SBAS/WAAS channel count of the current cycle.
    pub fn observe_locked(&mut self, n: usize) {
        self.sbas_hist.push_back(n);
        while self.sbas_hist.len() > GATE_WINDOW {
            self.sbas_hist.pop_front();
        }
    }

    /// Check the measurement context before a correction write.
    /// `waas_lock_s`: lock ages of the WAAS/GEO channels behind the
    /// residual; `inputs_age_s`: wall-clock age of the newest channel
    /// report. Ok(()) lets the write through and adopts `resid` as the new
    /// slew reference; Err(reason) suppresses it.
    pub fn check(
        &mut self,
        resid: f64,
        waas_lock_s: &[f64],
        inputs_age_s: f64,
    ) -> Result<(), &'static str> {
        if !resid.is_finite() {
            return Err("non-finite residual");
        }
        if resid.abs() > GATE_ABS_BOUND_PPM {
            return Err("residual beyond absolute sanity bound");
        }
        let locked_now = self.sbas_hist.back().copied().unwrap_or(0);
        let win_max = self.sbas_hist.iter().copied().max().unwrap_or(0);
        let floor = (win_max as f64 * GATE_COLLAPSE_FRAC).max(1.0);
        // only a count BELOW the window max can be a collapse — a healthy
        // low-count track (e.g. one GEO all night) is not one
        if locked_now < win_max && (locked_now as f64) < floor {
            return Err("WAAS locked-channel count collapsing");
        }
        if let Some(freshest) = waas_lock_s.iter().copied().reduce(f64::min) {
            if freshest < GATE_FRESH_LOCK_S {
                return Err("WAAS/GEO channel below fresh-lock threshold");
            }
        }
        if let Some(prev) = self.prev_resid {
            if (resid - prev).abs() > GATE_MAX_SLEW_PPM {
                return Err("residual jump beyond plausible TCXO slew");
            }
        }
        if inputs_age_s > GATE_STALE_S {
            return Err("measurement inputs stale");
        }
        self.prev_resid = Some(resid);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A healthy context passes and adopts the residual as slew reference.
    #[test]
    fn gate_passes_healthy_context() {
        let mut g = PlausibilityGate::new();
        for _ in 0..3 {
            g.observe_locked(5);
        }
        assert!(g.check(-0.40, &[45.0, 90.0], 1.0).is_ok());
        // a gentle move vs the accepted reference also passes
        assert!(g.check(-0.44, &[46.0, 91.0], 1.0).is_ok());
    }

    /// (a) locked-channel collapse: current count below max(2, half the
    /// recent-window max) suppresses the write.
    #[test]
    fn gate_suppresses_lock_collapse() {
        let mut g = PlausibilityGate::new();
        for _ in 0..5 {
            g.observe_locked(6);
        }
        g.observe_locked(2); // 6 -> 2: below 0.5 * 6 = 3
        assert_eq!(
            g.check(-0.40, &[45.0], 1.0),
            Err("WAAS locked-channel count collapsing")
        );
        // a gentle fluctuation (6 -> 4) is not a collapse
        let mut g = PlausibilityGate::new();
        for _ in 0..5 {
            g.observe_locked(6);
        }
        g.observe_locked(4);
        assert!(g.check(-0.40, &[45.0], 1.0).is_ok());
        // and a healthy low-count track never trips the floor
        let mut g = PlausibilityGate::new();
        for _ in 0..5 {
            g.observe_locked(1);
        }
        assert!(g.check(-0.40, &[45.0], 1.0).is_ok());
    }

    /// (a) a WAAS/GEO channel younger than the fresh-lock threshold just
    /// came out of a relock; its Doppler must not steer the clock.
    #[test]
    fn gate_suppresses_fresh_lock() {
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert_eq!(
            g.check(-0.40, &[45.0, 12.0], 1.0),
            Err("WAAS/GEO channel below fresh-lock threshold")
        );
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert!(g.check(-0.40, &[45.0, 31.0], 1.0).is_ok());
    }

    /// (b) a residual jump beyond the plausible TCXO slew vs the previous
    /// ACCEPTED residual suppresses the write — and the suppressed value
    /// must not become the new reference.
    #[test]
    fn gate_suppresses_implausible_slew() {
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert!(g.check(-0.40, &[45.0], 1.0).is_ok());
        assert_eq!(
            g.check(-0.05, &[45.0], 1.0),
            Err("residual jump beyond plausible TCXO slew")
        );
        // the suppressed -0.05 did not become the reference: a residual
        // near the old reference still passes
        assert!(g.check(-0.46, &[45.0], 1.0).is_ok());
    }

    /// (c) stale measurement inputs (the tracker stopped reporting)
    /// suppress the write.
    #[test]
    fn gate_suppresses_stale_inputs() {
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert_eq!(
            g.check(-0.40, &[45.0], 15.0),
            Err("measurement inputs stale")
        );
        assert!(g.check(-0.40, &[45.0], 5.0).is_ok());
    }

    /// (d) non-finite residuals are rejected outright.
    #[test]
    fn gate_suppresses_non_finite() {
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert_eq!(g.check(f64::NAN, &[45.0], 1.0), Err("non-finite residual"));
        assert_eq!(
            g.check(f64::INFINITY, &[45.0], 1.0),
            Err("non-finite residual")
        );
    }

    /// (e) the absolute sanity bound applies to EVERY sample, including the
    /// first — a garbage initial residual must not legitimize itself.
    #[test]
    fn gate_suppresses_beyond_absolute_bound() {
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert_eq!(
            g.check(3.7, &[45.0], 1.0),
            Err("residual beyond absolute sanity bound")
        );
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert!(g.check(1.9, &[45.0], 1.0).is_ok());
    }
}
