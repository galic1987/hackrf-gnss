//! GLONASS L1OF / L2OF standard-accuracy ranging code and FDMA channel plan.
//!
//! Unlike CDMA GNSS, every legacy GLONASS satellite transmits the SAME 511-chip
//! maximal-length sequence at 0.511 Mcps; satellites are separated by CARRIER
//! FREQUENCY (FDMA), not by code. So there is no per-PRN code table — one
//! m-sequence plus a frequency-channel search is the whole code layer. The
//! sequence is a 9-stage LFSR with the primitive polynomial x^9 + x^5 + 1,
//! initial state all ones, output tapped at stage 7 (GLONASS ICD).

pub const CODE_LEN: usize = 511;
pub const CHIP_RATE: f64 = 0.511e6;

/// L1 carrier for FDMA channel `k` (k = -7..=6): 1602.0 MHz + k*0.5625 MHz.
pub fn l1_freq(k: i32) -> f64 {
    1602.0e6 + k as f64 * 0.5625e6
}

/// L2 carrier for FDMA channel `k`: 1246.0 MHz + k*0.4375 MHz.
pub fn l2_freq(k: i32) -> f64 {
    1246.0e6 + k as f64 * 0.4375e6
}

/// The 511-chip GLONASS standard-accuracy code as +1.0 / -1.0 (same for every
/// satellite; FDMA separates them).
pub fn glonass_code() -> Vec<f32> {
    // 9-bit register, stages 1..9 held in indices 0..8; initial state all ones.
    let mut reg = [1u8; 9];
    let mut out = vec![0.0f32; CODE_LEN];
    for slot in out.iter_mut() {
        // output is stage 7 (index 6)
        *slot = 1.0 - 2.0 * reg[6] as f32;
        // feedback: x^9 + x^5 + 1  -> XOR of stages 5 and 9 (indices 4 and 8)
        let fb = reg[4] ^ reg[8];
        reg.copy_within(0..8, 1);
        reg[0] = fb;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn code_is_a_maximal_length_sequence() {
        let c = glonass_code();
        assert_eq!(c.len(), CODE_LEN);
        // an m-sequence of length 2^9-1 = 511 is balanced to within one:
        // exactly 256 of one symbol and 255 of the other.
        let ones = c.iter().filter(|&&v| v < 0.0).count();
        assert!(ones == 256 || ones == 255, "m-sequence imbalance: {ones}");
    }

    #[test]
    fn autocorrelation_is_two_valued() {
        // the defining property of an m-sequence: periodic autocorrelation is
        // CODE_LEN at zero lag and exactly -1 at every other lag.
        let c = glonass_code();
        let peak: f32 = c.iter().map(|&v| v * v).sum();
        assert!((peak - CODE_LEN as f32).abs() < 1e-3);
        for lag in 1..CODE_LEN {
            let r: f32 = (0..CODE_LEN)
                .map(|i| c[i] * c[(i + lag) % CODE_LEN])
                .sum();
            assert!((r + 1.0).abs() < 1e-3, "lag {lag}: autocorr {r} (expected -1)");
        }
    }

    #[test]
    fn fdma_channels_are_spaced_correctly() {
        assert!((l1_freq(0) - 1602.0e6).abs() < 1.0);
        assert!((l1_freq(1) - l1_freq(0) - 0.5625e6).abs() < 1.0);
        assert!((l1_freq(-7) - 1598.0625e6).abs() < 1.0);
        assert!((l2_freq(1) - l2_freq(0) - 0.4375e6).abs() < 1.0);
    }
}
