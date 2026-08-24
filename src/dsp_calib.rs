//! Multi-Constellation DSP Calibration Module for HackRF GNSS
//!
//! Provides mathematically rigorous Pseudo-Random Noise (PRN) code generators
//! and DSP correlation routines for:
//! - GPS L1 C/A (1023-chip Gold code sequence, IS-GPS-200)
//! - BeiDou B1I (2046-chip Gold code sequence, BDS-SIS-ICD-B1I)
//!
//! Galileo E1 is deliberately ABSENT. Its E1B/E1C primary codes are memory
//! codes tabulated in the ICD and cannot be produced by an LFSR; an earlier
//! version invented one, which made every Galileo result meaningless.

/// Generates a 1023-chip GPS L1 C/A PRN Gold code for a given PRN ID (1..=32).
///
/// Follows IS-GPS-200 specifications with LFSR registers G1 and G2.
/// Returns a vector of 1023 `f32` values (+1.0 or -1.0).
/// Returns 1023 zeros if `prn` is out of the valid range (1..=32).
pub fn generate_gps_ca_code(prn: usize) -> Vec<f32> {
    // Single source of truth: the C/A generator lives in `gps::ca_code`, where it
    // is validated bit-for-bit against the Python oracle and drives the
    // real-data-validated acquisition. This wrapper keeps the historical
    // out-of-range contract (a zero replica, which acquire_prn treats as the
    // noise floor) that this module's callers and tests rely on.
    if !(1..=32).contains(&prn) {
        return vec![0.0; 1023];
    }
    crate::gps::ca_code::gps_ca(prn)
}

/// Generates a 2046-chip BeiDou B1I PRN code for a given PRN ID (1..=37).
///
/// Follows BDS-SIS-ICD-B1I specifications:
/// - 11-stage LFSR G1 and G2 initialized to `01010101010` (binary)
/// - G1 feedback: 1 + X + X^7 + X^8 + X^9 + X^10 + X^11
/// - G2 feedback: 1 + X + X^2 + X^3 + X^4 + X^5 + X^8 + X^9 + X^11
///
/// Returns a vector of 2046 `f32` values (+1.0 or -1.0).
/// Returns 2046 zeros if `prn` is out of the valid range (1..=37).
pub fn generate_beidou_b1_code(prn: usize) -> Vec<f32> {
    if !(1..=37).contains(&prn) {
        return vec![0.0; 2046];
    }

    // G2 tap selection pairs for BeiDou B1I (PRN 1..=37)
    let tap_pairs: [(usize, usize); 37] = [
        (1, 3),  (1, 4),  (1, 5),  (1, 6),  (1, 8),  (1, 9),  (1, 10), (1, 11), // PRN 1..8
        (2, 7),  (3, 4),  (3, 5),  (3, 6),  (3, 8),  (3, 9),  (3, 10), (3, 11), // PRN 9..16
        (4, 5),  (4, 6),  (4, 8),  (4, 9),  (4, 10), (4, 11),                   // PRN 17..22
        (5, 6),  (5, 8),  (5, 9),  (5, 10), (5, 11),                            // PRN 23..27
        (6, 8),  (6, 9),  (6, 10), (6, 11),                                     // PRN 28..31
        (8, 9),  (8, 10), (8, 11),                                             // PRN 32..34
        (9, 10), (9, 11), (10, 11),                                             // PRN 35..37
    ];

    let (t1, t2) = tap_pairs[prn - 1];

    // G1 and G2 shift registers, length 11. Initial state: 01010101010 (binary)
    let mut g1 = [0i8, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0];
    let mut g2 = [0i8, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0];

    let mut code = Vec::with_capacity(2046);

    for _ in 0..2046 {
        let g1_out = g1[10];
        let g2_out = g2[t1 - 1] ^ g2[t2 - 1];
        let bit = g1_out ^ g2_out;

        code.push(if bit == 0 { 1.0f32 } else { -1.0f32 });

        let g1_fb = g1[0] ^ g1[6] ^ g1[7] ^ g1[8] ^ g1[9] ^ g1[10];
        let g2_fb = g2[0] ^ g2[1] ^ g2[2] ^ g2[3] ^ g2[4] ^ g2[7] ^ g2[8] ^ g2[10];

        for i in (1..11).rev() {
            g1[i] = g1[i - 1];
            g2[i] = g2[i - 1];
        }
        g1[0] = g1_fb;
        g2[0] = g2_fb;
    }

    code
}

/// Computes the normalized cross-correlation between a signal slice and a local code.
// `correlate_code` and `calibrate_dsp_channel` were removed. The latter scored
// a code by peak/mean, which is exactly the metric that reported 30 satellites
// from pure noise: over ~5e5 correlation bins the noise maximum alone reaches
// about 13. Acquisition uses peak-to-second-peak instead (see acquire_prn).
// Nothing called either function; leaving them exported invited their reuse.

#[cfg(test)]
mod tests {
    use super::*;

    /// First 10 chips of each GPS C/A code, in octal, from IS-GPS-200
    /// Table 3-I. This is the whole point of the test: it checks the generator
    /// against the PUBLISHED STANDARD rather than against itself.
    ///
    /// The tests these replaced asserted only that a code was 1023 long, that
    /// every value was +/-1, and that PRN 1 differed from PRN 32. A generator
    /// with the wrong tap pair, the wrong feedback polynomial or the wrong
    /// initial register state satisfies all three, so those tests could not
    /// fail for any bug they were meant to catch.
    const CA_FIRST10_OCTAL: [u16; 32] = [
        0o1440, 0o1620, 0o1710, 0o1744, 0o1133, 0o1455, 0o1131, 0o1454,
        0o1626, 0o1504, 0o1642, 0o1750, 0o1764, 0o1772, 0o1775, 0o1776,
        0o1156, 0o1467, 0o1633, 0o1715, 0o1746, 0o1763, 0o1063, 0o1706,
        0o1743, 0o1761, 0o1770, 0o1774, 0o1127, 0o1453, 0o1625, 0o1712,
    ];

    /// The generator emits +1.0 for chip 0 and -1.0 for chip 1.
    fn chips_to_bits(code: &[f32], n: usize) -> u16 {
        let mut v = 0u16;
        for &c in code.iter().take(n) {
            v = (v << 1) | if c < 0.0 { 1 } else { 0 };
        }
        v
    }

    fn correlate(a: &[f32], b: &[f32], shift: usize) -> i32 {
        let n = a.len();
        let mut acc = 0i32;
        for i in 0..n {
            acc += (a[i] * b[(i + shift) % n]) as i32;
        }
        acc
    }

    #[test]
    fn gps_ca_first_ten_chips_match_is_gps_200() {
        for prn in 1..=32 {
            let code = generate_gps_ca_code(prn);
            assert_eq!(code.len(), 1023, "PRN {} wrong length", prn);
            let got = chips_to_bits(&code, 10);
            let want = CA_FIRST10_OCTAL[prn - 1];
            assert_eq!(
                got, want,
                "PRN {}: first 10 chips are {:#o}, IS-GPS-200 Table 3-I says {:#o}",
                prn, got, want
            );
        }
    }

    #[test]
    fn gps_ca_codes_are_balanced() {
        // A balanced Gold code of period 1023 has exactly 512 ones and 511
        // zeros. A wrong initial state or a maximal-length-sequence bug shows
        // up here immediately.
        for prn in 1..=32 {
            let code = generate_gps_ca_code(prn);
            let ones = code.iter().filter(|&&c| c < 0.0).count();
            assert_eq!(ones, 512, "PRN {} has {} ones, expected 512", prn, ones);
        }
    }

    #[test]
    fn gps_ca_autocorrelation_is_three_valued() {
        // The defining property of a Gold code family: away from zero lag the
        // autocorrelation takes only the values -65, -1 and 63 (for m = 10).
        // Nothing but a correct pair of maximal-length sequences does this.
        for prn in [1usize, 5, 19, 32] {
            let code = generate_gps_ca_code(prn);
            assert_eq!(correlate(&code, &code, 0), 1023);
            for shift in 1..1023 {
                let c = correlate(&code, &code, shift);
                assert!(
                    c == -65 || c == -1 || c == 63,
                    "PRN {} autocorrelation at shift {} is {}, not one of -65/-1/63",
                    prn, shift, c
                );
            }
        }
    }

    #[test]
    fn gps_ca_cross_correlation_is_bounded() {
        // Cross-correlation between distinct PRNs must stay in the same
        // three-valued set. This is what makes CDMA work at all, and what makes
        // a 2.5 peak-to-second-peak threshold meaningful.
        for (a, b) in [(1usize, 2usize), (1, 19), (5, 32), (24, 25)] {
            let ca = generate_gps_ca_code(a);
            let cb = generate_gps_ca_code(b);
            for shift in 0..1023 {
                let c = correlate(&ca, &cb, shift);
                assert!(
                    c == -65 || c == -1 || c == 63,
                    "PRN {} x PRN {} at shift {} is {}, outside the Gold bound",
                    a, b, shift, c
                );
            }
        }
    }

    #[test]
    fn gps_ca_out_of_range_returns_zeros() {
        for prn in [0usize, 33, 100] {
            let code = generate_gps_ca_code(prn);
            assert_eq!(code.len(), 1023);
            assert!(code.iter().all(|&v| v == 0.0), "PRN {} should be all zeros", prn);
        }
    }

    #[test]
    fn beidou_b1_has_the_right_shape_and_low_correlation() {
        // Weaker than the GPS tests above, and deliberately labelled as such:
        // these check structure and correlation rather than published chip
        // values, so they would catch a broken generator but not a subtly wrong
        // one. B1I is a length-2046 Gold code (2047 truncated by one), so its
        // correlation is near-ideal but not exactly three-valued.
        for prn in [1usize, 12, 37] {
            let code = generate_beidou_b1_code(prn);
            assert_eq!(code.len(), 2046);
            assert!(code.iter().all(|&v| v == 1.0 || v == -1.0));
            let dc = code.iter().sum::<f32>().abs();
            assert!(dc < 64.0, "PRN {} has DC imbalance {}", prn, dc);
            assert_eq!(correlate(&code, &code, 0), 2046);
            let worst = (1..2046)
                .map(|s| correlate(&code, &code, s).abs())
                .max()
                .unwrap();
            assert!(
                worst < 300,
                "PRN {} worst off-peak autocorrelation {} is too high for a Gold code",
                prn, worst
            );
        }
        assert_ne!(generate_beidou_b1_code(1), generate_beidou_b1_code(37));
    }

    #[test]
    fn beidou_b1_out_of_range_returns_zeros() {
        for prn in [0usize, 38] {
            let code = generate_beidou_b1_code(prn);
            assert_eq!(code.len(), 2046);
            assert!(code.iter().all(|&v| v == 0.0));
        }
    }
}
