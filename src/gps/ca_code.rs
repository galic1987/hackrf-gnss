//! GPS L1 C/A code generation, a bit-for-bit port of `validation/acquire.py:gps_ca`.
//!
//! Two 10-stage LFSRs (G1, G2). The PRN is selected by which pair of G2 taps is
//! summed to form its output phase; G1 and both feedbacks are fixed. Codes are
//! returned as +1.0 / -1.0 floats, chip order identical to the Python oracle so
//! the two acquisitions can be compared sample for sample.

/// G2 output-phase tap pair (1-based stage numbers) for each PRN 1..=32.
const TAPS: [(usize, usize); 32] = [
    (2, 6), (3, 7), (4, 8), (5, 9), (1, 9), (2, 10), (1, 8), (2, 9),
    (3, 10), (2, 3), (3, 4), (5, 6), (6, 7), (7, 8), (8, 9), (9, 10),
    (1, 4), (2, 5), (3, 6), (4, 7), (5, 8), (6, 9), (1, 3), (4, 6),
    (5, 7), (6, 8), (7, 9), (8, 10), (1, 6), (2, 7), (3, 8), (4, 9),
];

pub const CA_LEN: usize = 1023;
pub const CHIP_RATE: f64 = 1.023e6;

/// The 1023-chip C/A sequence for `prn` (1..=32) as +1.0 / -1.0.
pub fn gps_ca(prn: usize) -> Vec<f32> {
    assert!((1..=32).contains(&prn), "GPS PRN out of range: {prn}");
    let (t1, t2) = TAPS[prn - 1];
    let mut g1 = [1u8; 10];
    let mut g2 = [1u8; 10];
    let mut out = vec![0.0f32; CA_LEN];
    for slot in out.iter_mut() {
        let chip = g1[9] ^ (g2[t1 - 1] ^ g2[t2 - 1]);
        *slot = 1.0 - 2.0 * (chip as f32);
        let fb1 = g1[2] ^ g1[9];
        let fb2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9];
        // shift up by one, feedback into index 0 (matches g1[1:]=g1[:-1]; g1[0]=fb)
        g1.copy_within(0..9, 1);
        g1[0] = fb1;
        g2.copy_within(0..9, 1);
        g2[0] = fb2;
    }
    out
}

/// Nearest-chip resample of a code to `nsamp` samples at `fs`, matching
/// `validation/acquire.py:resample_code` (floor(i*chiprate/fs) % len).
pub fn resample_code(code: &[f32], nsamp: usize, chiprate: f64, fs: f64) -> Vec<f32> {
    let n = code.len();
    (0..nsamp)
        .map(|i| {
            let idx = ((i as f64) * chiprate / fs) as usize % n;
            code[idx]
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The first 10 chips of each PRN's C/A code, expressed as the octal of the
    /// leading bits, are fixed by IS-GPS-200. PRN 1's first 10 chips are the
    /// well-known 1440 octal pattern; check the canonical first-10-chip value.
    #[test]
    fn first_ten_chips_match_python_oracle() {
        // ground truth from validation/acquire.py:gps_ca (chip=1 -> -1.0)
        let want: [(usize, [u8; 10]); 4] = [
            (1, [1, 1, 0, 0, 1, 0, 0, 0, 0, 0]),
            (2, [1, 1, 1, 0, 0, 1, 0, 0, 0, 0]),
            (19, [1, 1, 1, 0, 0, 1, 1, 0, 1, 1]),
            (32, [1, 1, 1, 1, 0, 0, 1, 0, 1, 0]),
        ];
        for (prn, bits) in want {
            let c = gps_ca(prn);
            let got: Vec<u8> = c[..10].iter().map(|&v| if v < 0.0 { 1 } else { 0 }).collect();
            assert_eq!(got, bits.to_vec(), "PRN {prn}");
        }
    }

    #[test]
    fn all_prns_are_balanced_and_full_length() {
        for prn in 1..=32 {
            let c = gps_ca(prn);
            assert_eq!(c.len(), CA_LEN);
            // a Gold code of length 1023 has exactly 512 ones and 511 zeros
            let ones = c.iter().filter(|&&v| v < 0.0).count();
            assert_eq!(ones, 512, "PRN {prn} imbalance");
        }
    }

    #[test]
    fn codes_are_distinct() {
        let a = gps_ca(1);
        let b = gps_ca(2);
        assert_ne!(a, b);
    }

    #[test]
    fn resample_wraps_and_holds_chips() {
        let code = gps_ca(1);
        // at 2x chip rate each chip is held for ~2 samples
        let rs = resample_code(&code, 2046, CHIP_RATE, 2.0 * CHIP_RATE);
        assert_eq!(rs[0], code[0]);
        assert_eq!(rs[1], code[0]);
        assert_eq!(rs[2], code[1]);
    }
}
