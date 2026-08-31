//! Host side of the iCE40 carry-chain TDC: thermometer decoding and
//! code-density calibration.
//!
//! Gateware side: `CarryChainTDC` in firmware/fpga/dsp/tdc.py, integrated
//! in top/timing.py (image 0) with SPI readout at 0x20-0x31.
//!
//! The gateware freezes a raw 48-tap thermometer code per event (the rev3c
//! chain; the 6-byte register block 0x20-0x25 holds taps 0-47, LSB-first).
//! A2 accepts only nonzero tap-0-anchored words. Exact strict prefixes 1..47
//! are interior codes, all ones is composite full scale, and an anchored word
//! with internal holes is bubbled. Preserve a bubbled raw word and its
//! popcount for diagnostics, but do not use that popcount as a timing bin.
//!
//! Code-density calibration: the on-chip ring oscillator is asynchronous to
//! the sampling clock, so selftest event phases are uniform over the adclk
//! period. Code-density input must first exclude bubbled and tap-0-clear raw
//! words; `TdcCal::ingest` cannot perform that validation because it receives
//! only a scalar. Bin widths are also meaningful only relative to the adclk
//! rate recorded with the capture.

/// Descriptive popcount of the frozen raw word.
///
/// `bytes` is the register block at 0x20: 6 bytes / 48 taps in the shipped
/// rev3c image (the legacy 64-tap/16-byte variant works too — a popcount
/// does not care about the block length). This function does not classify A2
/// words; its result must not be ingested as a timing code when the raw word
/// is bubbled or tap 0 is clear.
pub fn popcount_thermo(bytes: &[u8]) -> u32 {
    bytes.iter().map(|b| b.count_ones()).sum()
}

/// Code-density calibration accumulator.
///
/// `hist[pop]` = number of prevalidated strict-prefix codes with that value.
/// Grows on ingest; bins never observed keep count 0 and get zero width.
pub struct TdcCal {
    pub hist: Vec<u64>,
    pub clk_period_s: f64,
}

impl TdcCal {
    pub fn new(clk_period_s: f64) -> Self {
        TdcCal { hist: Vec::new(), clk_period_s }
    }

    /// Record one already-validated strict-prefix code.
    ///
    /// The caller must reject tap-0-clear words and exclude bubbled words;
    /// passing a bubbled raw-word popcount here creates an invalid LUT.
    pub fn ingest(&mut self, pop: u32) {
        let idx = pop as usize;
        if idx >= self.hist.len() {
            self.hist.resize(idx + 1, 0);
        }
        self.hist[idx] += 1;
    }

    /// Total samples ingested.
    pub fn total(&self) -> u64 {
        self.hist.iter().sum()
    }

    /// Per-bin width in picoseconds: each bin's hit share of the clock
    /// period. Unobserved bins get 0.0 (they contribute nothing to the
    /// prefix-sum LUT, keeping it monotone).
    pub fn bin_widths_ps(&self) -> Vec<f64> {
        let total = self.total();
        if total == 0 {
            return vec![0.0; self.hist.len()];
        }
        let period_ps = self.clk_period_s * 1e12;
        self.hist
            .iter()
            .map(|&c| (c as f64 / total as f64) * period_ps)
            .collect()
    }

    /// Phase lookup table in picoseconds: `lut[k]` = phase offset of a code
    /// with popcount k = prefix sum of the bin widths below k.
    pub fn lut_ps(&self) -> Vec<f64> {
        let mut lut = Vec::with_capacity(self.hist.len());
        let mut acc = 0.0;
        for w in self.bin_widths_ps() {
            lut.push(acc);
            acc += w;
        }
        lut
    }

    /// Widest single bin in picoseconds (DNL worst case).
    pub fn widest_bin_ps(&self) -> f64 {
        self.bin_widths_ps()
            .into_iter()
            .fold(0.0_f64, f64::max)
    }
}

/// Absolute phase in nanoseconds: coarse timestamp ticks plus the fine
/// TDC offset. `pop` beyond the LUT clamps to the last entry (a saturated
/// code means the event phase ran past the end of the chain).
pub fn phase_ns(coarse_ticks: u64, tick_hz: f64, pop: u32, lut: &[f64]) -> f64 {
    let coarse_ns = coarse_ticks as f64 / tick_hz * 1e9;
    let fine_ps = match lut.len() {
        0 => 0.0,
        n => lut[(pop as usize).min(n - 1)],
    };
    coarse_ns + fine_ps / 1e3
}
