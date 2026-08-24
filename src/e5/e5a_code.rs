//! Galileo E5a-I primary ranging codes: 10230 chips at 10.23 Mcps, two
//! 14-stage LFSRs (base register 1 XOR base register 2), truncated and
//! re-initialised every 10230 chips.
//!
//! Provenance — nothing here is invented or reconstructed from memory:
//!   * Galileo Open Service Signal In Space ICD, Issue 1.3, December 2016:
//!     §3.3 (LFSR generation method, Figure 10), §3.4.1 Table 14 (E5a-I:
//!     register length 14; feedback taps, octal, register 1 = 40503,
//!     register 2 = 50661; register 1 start value all-ones) and Table 15
//!     (per-code register-2 start values in octal + first 24 code chips in
//!     hex, first chip in time = MSB).
//!     https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v1.3.pdf
//!     (retrieved 2026-08-23)
//!   * Cross-checked against PocketSDR `src/sdr_code.c` (`gen_code_E5AI`,
//!     T. Takasu), https://github.com/tomojitakasu/PocketSDR (retrieved
//!     2026-08-23): all 50 register-2 start values identical.
//!
//! The unit test `reproduces_all_50_icd_first24` regenerates every code and
//! matches all 50 published first-24-chip vectors from ICD Table 15, proving
//! the polynomials, tap convention, bit order and start values are all right.

pub const CODE_LEN: usize = 10230;
const STAGES: usize = 14;
// ICD Table 14 feedback tap words include the unused j=0 bit at the LSB and
// the always-1 j=R bit at the MSB; `>> 1` drops the j=0 bit (PocketSDR does
// exactly the same).
const X1_TAPS: u32 = 0o40503 >> 1; // E5a-I register 1 feedback taps
const X2_TAPS: u32 = 0o50661 >> 1; // E5a-I register 2 feedback taps

/// Register-2 start values, ICD Table 15 (octal), PRN 1..=50.
const E5AI_START: [u32; 50] = [
    0o30305, 0o14234, 0o27213, 0o20577, 0o23312, 0o33463, 0o15614, 0o12537,
    0o01527, 0o30236, 0o27344, 0o07272, 0o36377, 0o17046, 0o06434, 0o15405,
    0o24252, 0o11631, 0o24776, 0o00630, 0o11560, 0o17272, 0o27445, 0o31702,
    0o13012, 0o14401, 0o34727, 0o22627, 0o30623, 0o27256, 0o01520, 0o14211,
    0o31465, 0o22164, 0o33516, 0o02737, 0o21316, 0o35425, 0o35633, 0o24655,
    0o14054, 0o27027, 0o06604, 0o31455, 0o34465, 0o25273, 0o20763, 0o31721,
    0o17312, 0o13277,
];

/// First 24 chips of each E5a-I code, ICD Table 15 (hex, first chip = MSB).
/// Used only by the unit tests as golden vectors.
#[cfg(test)]
const E5AI_FIRST24: [u32; 50] = [
    0x3CEA9D, 0x9D8CF1, 0x45D1C8, 0x7A0133, 0x64D423, 0x23300D, 0x91CEF2, 0xAA82DC,
    0xF2A17D, 0x3D84AE, 0x446D38, 0xC514F2, 0x0C0184, 0x8767E0, 0xCB8EFF, 0x93EBCD,
    0x5D55CE, 0xB19B7C, 0x5805FC, 0xF99EA1, 0xB23CE5, 0x8515E8, 0x436822, 0x30F77B,
    0xA7D629, 0x9BFAC7, 0x18A25B, 0x69A39F, 0x39B27D, 0x454598, 0xF2BC62, 0x9DDBC6,
    0x332827, 0x6E2FCA, 0x22C6D5, 0xE881D9, 0x74C4DB, 0x13AB03, 0x119323, 0x594886,
    0x9F4D89, 0x47A3C0, 0xC9ED53, 0x334994, 0x1B2A30, 0x5513F3, 0x7831C1, 0x30B93A,
    0x84D5B4, 0xA5029C,
];

/// Reverse the low `n` bits of `r`. The ICD counts tap/start-value bits from
/// the LSB with cell j at bit j; the generator below emits the LSB first and
/// shifts right (ICD Figure 10 with cell R on the right), so the ICD words
/// enter the register bit-reversed — same transformation as PocketSDR's
/// `rev_reg`.
const fn rev_reg(r: u32, n: usize) -> u32 {
    let mut rr = 0u32;
    let mut i = 0;
    while i < n {
        rr = (rr << 1) | ((r >> i) & 1);
        i += 1;
    }
    rr
}

/// Run a Fibonacci LFSR for `n` chips: output is the LSB, the parity of the
/// tapped cells feeds the MSB, the register shifts right.
fn lfsr(n: usize, init: u32, taps: u32) -> Vec<u8> {
    let mut r = init;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        out.push((r & 1) as u8);
        let fb = (r & taps).count_ones() & 1;
        r = (fb << (STAGES - 1)) | (r >> 1);
    }
    out
}

/// One 1 ms E5a-I primary code (10230 chips) as +1.0 / -1.0. PRN 1..=50.
pub fn e5ai_code(prn: usize) -> Vec<f32> {
    assert!((1..=50).contains(&prn), "E5a-I PRN {prn} out of range (1..=50)");
    let x1 = lfsr(CODE_LEN, 0x3FFF, rev_reg(X1_TAPS, STAGES)); // register 1: all-ones start
    let x2 = lfsr(CODE_LEN, rev_reg(E5AI_START[prn - 1], STAGES), rev_reg(X2_TAPS, STAGES));
    (0..CODE_LEN).map(|k| 1.0 - 2.0 * (x1[k] ^ x2[k]) as f32).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pack `n` chips starting at `from` into a word, first chip = MSB (the
    /// ICD Table 15 convention).
    fn chips_word(c: &[f32], from: usize, n: usize) -> u32 {
        (0..n).fold(0u32, |a, k| a | (((c[from + k] < 0.0) as u32) << (n - 1 - k)))
    }

    // The definitive correctness proof: regenerate all 50 codes and match the
    // published first-24-chip vectors of ICD Table 15.
    #[test]
    fn reproduces_all_50_icd_first24() {
        for prn in 1..=50usize {
            let c = e5ai_code(prn);
            assert_eq!(
                chips_word(&c, 0, 24),
                E5AI_FIRST24[prn - 1],
                "E5a-I PRN {prn} first 24 chips disagree with ICD Table 15"
            );
        }
    }

    #[test]
    fn code_length_and_balance() {
        let c = e5ai_code(1);
        assert_eq!(c.len(), CODE_LEN);
        let ones = c.iter().filter(|&&v| v < 0.0).count() as i64;
        let imbalance = (ones - (CODE_LEN as i64 - ones)).abs();
        assert!(imbalance < 200, "E5a-I PRN1 imbalance {imbalance}");
    }

    #[test]
    fn distinct_prns_are_nearly_orthogonal() {
        let a = e5ai_code(1);
        let b = e5ai_code(7);
        let x: f32 = a.iter().zip(&b).map(|(u, v)| u * v).sum();
        assert!(x.abs() < 0.06 * CODE_LEN as f32, "E5a-I PRN1xPRN7 xcorr {x}");
        let peak: f32 = a.iter().map(|v| v * v).sum();
        assert!((peak - CODE_LEN as f32).abs() < 1.0);
    }
}
