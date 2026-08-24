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
