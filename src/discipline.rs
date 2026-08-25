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
/// First-write lock maturity (s): the first ACCEPTED residual after a
/// process (re)start needs WAAS/GEO locks at least this old — with no slew
/// reference the gate cannot tell a warming tracker's convergence transient
/// from truth (the 2026-08-25 restart mis-step: spurious -0.1 ppm
/// first-cycle steps at 08:44, 09:49, 11:50).
pub const GATE_FIRST_LOCK_S: f64 = 180.0;
/// Cumulative accepted-drift bound vs the oldest retained acceptance (ppm).
/// Per-cycle slew bounds don't stop a slow bogus ramp (2026-08-25 review:
/// <=0.0375 ppm/cycle walks in unbounded — 2.25 ppm/h). True thermal wander
/// is ~0.02 ppm/h (the live residual stayed within [-0.48, -0.34] over
/// 6.6 h), so 0.15 ppm is far above physics and far below the ramp.
pub const GATE_CUM_SLEW_PPM: f64 = 0.15;
/// Consistent cumulative suppressions before re-anchoring: true multi-hour
/// drift must eventually win (latch-up class), but a bogus ramp pays this
/// many quiet cycles (~20 min) per 0.15 ppm adopted.
pub const GATE_CUM_RECOVER_N: usize = 20;

/// Bounded latch-up recovery: after this many CONSECUTIVE slew
/// suppressions whose values agree among themselves, re-anchor the
/// reference to their median — loudly. (2026-08-25 latch-up: 276
/// suppressed writes over 4.6 h after the true residual had legitimately
/// drifted past the slew bound; prev_resid only updated on accepts, so
/// every later sample failed against the stale reference forever.)
pub const GATE_RECOVER_N: usize = 5;

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
    /// recent slew-suppressed residuals (the recovery candidate pool)
    suppressed: std::collections::VecDeque<f64>,
    /// recently ACCEPTED residuals, newest last (the cumulative-drift
    /// window: bounded at GATE_WINDOW entries)
    accepted: std::collections::VecDeque<f64>,
    /// consecutive cumulative-drift suppressions (the slow recovery pool)
    cum_suppressed: std::collections::VecDeque<f64>,
    /// true when this check() recovered the slew reference — the caller
    /// must log it loudly (a recovery means the loop was latched)
    pub recovered: bool,
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
        self.recovered = false;
        if !resid.is_finite() {
            self.suppressed.clear();
            self.cum_suppressed.clear();
            return Err("non-finite residual");
        }
        if resid.abs() > GATE_ABS_BOUND_PPM {
            self.suppressed.clear();
            self.cum_suppressed.clear();
            return Err("residual beyond absolute sanity bound");
        }
        let locked_now = self.sbas_hist.back().copied().unwrap_or(0);
        let win_max = self.sbas_hist.iter().copied().max().unwrap_or(0);
        let floor = (win_max as f64 * GATE_COLLAPSE_FRAC).max(1.0);
        // only a count BELOW the window max can be a collapse — a healthy
        // low-count track (e.g. one GEO all night) is not one
        if locked_now < win_max && (locked_now as f64) < floor {
            self.suppressed.clear();
            self.cum_suppressed.clear();
            return Err("WAAS locked-channel count collapsing");
        }
        let min_lock = waas_lock_s.iter().copied().reduce(f64::min);
        if let Some(freshest) = min_lock {
            if freshest < GATE_FRESH_LOCK_S {
                self.suppressed.clear();
                self.cum_suppressed.clear();
                return Err("WAAS/GEO channel below fresh-lock threshold");
            }
        }
        // staleness precedes the slew/recovery path: a stale input must
        // never re-anchor the reference (2026-08-25 review hole)
        if inputs_age_s > GATE_STALE_S {
            self.suppressed.clear();
            self.cum_suppressed.clear();
            return Err("measurement inputs stale");
        }
        // first write after (re)start: no slew reference exists yet, so
        // demand mature locks (the restart mis-step class)
        if self.prev_resid.is_none() {
            if let Some(freshest) = min_lock {
                if freshest < GATE_FIRST_LOCK_S {
                    self.suppressed.clear();
                    self.cum_suppressed.clear();
                    return Err("first write before WAAS/GEO locks mature");
                }
            }
        }
        if let Some(prev) = self.prev_resid {
            if (resid - prev).abs() > GATE_MAX_SLEW_PPM {
                // latch-up recovery: N consecutive slew suppressions that
                // agree among themselves mean the TRUE residual moved and
                // the reference is stale — re-anchor to their median,
                // loudly (the 4.6 h / 276-write latch-up of 2026-08-25).
                self.suppressed.push_back(resid);
                while self.suppressed.len() > GATE_RECOVER_N {
                    self.suppressed.pop_front();
                }
                if self.suppressed.len() >= GATE_RECOVER_N {
                    let mut pool: Vec<f64> = self.suppressed.iter().copied().collect();
                    pool.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    if pool[pool.len() - 1] - pool[0] <= GATE_MAX_SLEW_PPM {
                        let med = pool[pool.len() / 2];
                        self.prev_resid = Some(med);
                        self.accepted.clear();
                        self.accepted.push_back(med);
                        self.suppressed.clear();
                        self.cum_suppressed.clear();
                        self.recovered = true;
                        return Ok(());
                    }
                }
                return Err("residual jump beyond plausible TCXO slew");
            }
        }
        // cumulative bound: even per-cycle-plausible steps must not walk
        // the reference without limit (the slow-ramp hole)
        if let Some(&oldest) = self.accepted.front() {
            if (resid - oldest).abs() > GATE_CUM_SLEW_PPM {
                self.cum_suppressed.push_back(resid);
                while self.cum_suppressed.len() > GATE_CUM_RECOVER_N {
                    self.cum_suppressed.pop_front();
                }
                // true long-term drift eventually wins, but slowly and
                // loudly: GATE_CUM_RECOVER_N consistent suppressions
                if self.cum_suppressed.len() >= GATE_CUM_RECOVER_N {
                    let mut pool: Vec<f64> = self.cum_suppressed.iter().copied().collect();
                    pool.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    if pool[pool.len() - 1] - pool[0] <= GATE_MAX_SLEW_PPM {
                        let med = pool[pool.len() / 2];
                        self.prev_resid = Some(med);
                        self.accepted.clear();
                        self.accepted.push_back(med);
                        self.suppressed.clear();
                        self.cum_suppressed.clear();
                        self.recovered = true;
                        return Ok(());
                    }
                }
                return Err("cumulative drift beyond plausible TCXO wander");
            }
        }
        self.suppressed.clear();
        self.cum_suppressed.clear();
        self.prev_resid = Some(resid);
        self.accepted.push_back(resid);
        while self.accepted.len() > GATE_WINDOW {
            self.accepted.pop_front();
        }
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
        assert!(g.check(-0.40, &[200.0, 190.0], 1.0).is_ok());
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
        assert!(g.check(-0.40, &[200.0], 1.0).is_ok());
        // and a healthy low-count track never trips the floor
        let mut g = PlausibilityGate::new();
        for _ in 0..5 {
            g.observe_locked(1);
        }
        assert!(g.check(-0.40, &[200.0], 1.0).is_ok());
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
        assert!(g.check(-0.40, &[200.0], 1.0).is_ok()); // mature first write
        assert!(g.check(-0.44, &[45.0, 31.0], 1.0).is_ok());
    }

    /// (b) a residual jump beyond the plausible TCXO slew vs the previous
    /// ACCEPTED residual suppresses the write — and the suppressed value
    /// must not become the new reference.
    #[test]
    fn gate_suppresses_implausible_slew() {
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert!(g.check(-0.40, &[200.0], 1.0).is_ok());
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
        assert!(g.check(-0.40, &[200.0], 5.0).is_ok());
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
        assert!(g.check(1.9, &[200.0], 1.0).is_ok());
    }

    /// (f) latch-up recovery: N consecutive slew suppressions that agree
    /// among themselves re-anchor the reference (the 2026-08-25 latch-up:
    /// 276 suppressed writes over 4.6 h with no recovery path).
    #[test]
    fn gate_recovers_from_latchup() {
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert!(g.check(-0.40, &[200.0], 1.0).is_ok());
        for _ in 0..GATE_RECOVER_N - 1 {
            assert_eq!(
                g.check(-0.70, &[45.0], 1.0),
                Err("residual jump beyond plausible TCXO slew")
            );
            assert!(!g.recovered);
        }
        assert!(g.check(-0.70, &[45.0], 1.0).is_ok());
        assert!(g.recovered);
        // the re-anchored reference accepts values near the new level
        assert!(g.check(-0.72, &[45.0], 1.0).is_ok());
    }

    /// (g) disagreement among suppressed values never recovers.
    #[test]
    fn gate_no_recovery_on_disagreement() {
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert!(g.check(-0.40, &[200.0], 1.0).is_ok());
        for r in [-0.70, -1.30, -0.65, -1.35, -0.75, -1.25] {
            assert!(g.check(r, &[45.0], 1.0).is_err());
            assert!(!g.recovered);
        }
    }

    /// (h) staleness is checked BEFORE the slew/recovery path: a stale
    /// input must never trigger latch-up recovery (2026-08-25 review: the
    /// recovery path ran before the staleness check).
    #[test]
    fn gate_stale_inputs_never_recover() {
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert!(g.check(-0.40, &[200.0], 1.0).is_ok());
        for _ in 0..GATE_RECOVER_N * 2 {
            assert_eq!(
                g.check(-0.70, &[200.0], 30.0),
                Err("measurement inputs stale")
            );
            assert!(!g.recovered);
        }
    }

    /// (i) the FIRST accepted residual after a process (re)start needs
    /// mature WAAS/GEO locks: with no slew reference the gate cannot tell a
    /// warming tracker's convergence transient from a true residual (the
    /// restart mis-step: spurious -0.1 ppm first-cycle steps at 08:44,
    /// 09:49, 11:50 on 2026-08-25).
    #[test]
    fn gate_first_write_needs_mature_locks() {
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert_eq!(
            g.check(-0.40, &[45.0, 90.0], 1.0),
            Err("first write before WAAS/GEO locks mature")
        );
        // one mature + one young lock: the young one still gates the first
        // write (min semantics, same as the fresh-lock check)
        assert_eq!(
            g.check(-0.40, &[45.0, 200.0], 1.0),
            Err("first write before WAAS/GEO locks mature")
        );
        assert!(g.check(-0.40, &[200.0, 240.0], 1.0).is_ok());
        // afterwards the normal per-cycle thresholds apply
        assert!(g.check(-0.44, &[45.0, 46.0], 1.0).is_ok());
    }

    /// (j) slow-ramp walk-in: per-cycle slew is bounded, but a bogus
    /// residual ramping just under the slew bound must not be adopted
    /// forever — cumulative drift over the window is bounded too
    /// (2026-08-25 review: <=0.0375 ppm/cycle walks in unbounded; true
    /// thermal wander is ~0.02 ppm/h, a bogus ramp is 2.25 ppm/h).
    #[test]
    fn gate_bounds_cumulative_drift() {
        let mut g = PlausibilityGate::new();
        g.observe_locked(5);
        assert!(g.check(-0.40, &[200.0], 1.0).is_ok());
        let mut r = -0.40;
        let mut suppressed = false;
        for _ in 0..20 {
            r += 0.04; // under the 0.15 ppm/cycle slew bound every time
            if g.check(r, &[200.0], 1.0).is_err() {
                suppressed = true;
                break;
            }
        }
        assert!(suppressed, "slow ramp was never suppressed");
        // and the walk must stay bounded near the window bound: a residual
        // at the full-ramp value (+0.80 from start) is rejected
        assert!(g.check(0.40, &[200.0], 1.0).is_err());
    }
}
