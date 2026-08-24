//! GPS L5 ranging codes (IS-GPS-705). L5 is 10230 chips at 10.23 Mcps — ten
//! times L1's length. Each code is XA xor XB of two 13-stage LFSRs:
//!   XA: 1 + x^9 + x^10 + x^12 + x^13, all-ones init, SHORT CYCLED to 8190.
//!   XB: 1 + x + x^3 + x^4 + x^6 + x^7 + x^8 + x^12 + x^13, period 8191, its
//!       starting phase (an advance from all-ones) selects the PRN + component.
//! The I5 (data) and Q5 (pilot) phases form inseparable per-PRN pairs.
//!
//! Ported from `validation/l5.py`. Table 3-I lists, per PRN, both the XB code
//! advance AND the first 13 XB chips — redundant columns, so the test
//! regenerates all 126 published phases (63 PRN x I5/Q5) as a golden-vector
//! proof that the polynomial, taps, bit order and advance are all correct.

pub const CODE_LEN: usize = 10230;
const XA_LEN: usize = 8190; // short cycled one chip before its natural 8191
const XB_LEN: usize = 8191;
const XA_TAPS: [usize; 4] = [9, 10, 12, 13];
const XB_TAPS: [usize; 8] = [1, 3, 4, 6, 7, 8, 12, 13];

/// (I5 advance, Q5 advance, I5 first-13 XB chips, Q5 first-13 XB chips), PRN 1..=63.
/// First-13 chips are stored as in IS-GPS-705: the RIGHTMOST bit is the first chip.
const L5_TABLE: [(u16, u16, u16, u16); 63] = [
    (266, 1701, 0b0101011100100, 0b1001011001100),  // PRN 1
    (365, 323, 0b1100000110101, 0b0100011110110),   // PRN 2
    (804, 5292, 0b0100000001000, 0b1111000100011),  // PRN 3
    (1138, 2020, 0b1011000100110, 0b0011101101010), // PRN 4
    (1509, 5429, 0b1110111010111, 0b0011110110010), // PRN 5
    (1559, 7136, 0b0110011111010, 0b0101010101001), // PRN 6
    (1756, 1041, 0b1010010011111, 0b1111110000001), // PRN 7
    (2084, 5947, 0b1011110100100, 0b0110101101000), // PRN 8
    (2170, 4315, 0b1111100101011, 0b1011101000011), // PRN 9
    (2303, 148, 0b0111111011110, 0b0010010000110),  // PRN 10
    (2527, 535, 0b0000100111010, 0b0001000000101),  // PRN 11
    (2687, 1939, 0b1110011111001, 0b0101011000101), // PRN 12
    (2930, 5206, 0b0001110011100, 0b0100110100101), // PRN 13
    (3471, 5910, 0b0100000100111, 0b1010000111111), // PRN 14
    (3940, 3595, 0b0110101011010, 0b1011110001111), // PRN 15
    (4132, 5135, 0b0001111001001, 0b1101001011111), // PRN 16
    (4332, 6082, 0b0100110001111, 0b1110011001000), // PRN 17
    (4924, 6990, 0b1111000011110, 0b1011011100100), // PRN 18
    (5343, 3546, 0b1100100011111, 0b0011001011011), // PRN 19
    (5443, 1523, 0b0110101101101, 0b1100001110001), // PRN 20
    (5641, 4548, 0b0010000001000, 0b0110110010000), // PRN 21
    (5816, 4484, 0b1110111101111, 0b0010110001110), // PRN 22
    (5898, 1893, 0b1000011111110, 0b1000101111101), // PRN 23
    (5918, 3961, 0b1100010110100, 0b0110111110011), // PRN 24
    (5955, 7106, 0b1101001101101, 0b0100010011011), // PRN 25
    (6243, 5299, 0b1010110010110, 0b0101010111100), // PRN 26
    (6345, 4660, 0b0101011011110, 0b1000011111010), // PRN 27
    (6477, 276, 0b0111101010110, 0b1111101000010),  // PRN 28
    (6518, 4389, 0b0101111100001, 0b0101000100100), // PRN 29
    (6875, 3783, 0b1000010110111, 0b1000001111001), // PRN 30
    (7168, 1591, 0b0001010011110, 0b0101111100101), // PRN 31
    (7187, 1601, 0b0000010111001, 0b1001000101010), // PRN 32
    (7329, 749, 0b1101010000001, 0b1011001000100),  // PRN 33
    (7577, 1387, 0b1101111111001, 0b1111001000100), // PRN 34
    (7720, 1661, 0b1111011011100, 0b0110010110011), // PRN 35
    (7777, 3210, 0b1001011001000, 0b0011110101111), // PRN 36
    (8057, 708, 0b0011010010000, 0b0010011010001),  // PRN 37
    (5358, 4226, 0b0101100000110, 0b1111110011101), // PRN 38
    (3550, 5604, 0b1001001100101, 0b0101010011111), // PRN 39
    (3412, 6375, 0b1100111001010, 0b1000110101010), // PRN 40
    (819, 3056, 0b0111011011001, 0b0010111100100),  // PRN 41
    (4608, 1772, 0b0011101101100, 0b1011000100000), // PRN 42
    (3698, 3662, 0b0011011111010, 0b0011001011001), // PRN 43
    (962, 4401, 0b1001011010001, 0b1000100101000),  // PRN 44
    (3001, 5218, 0b1001010111111, 0b0000001111110), // PRN 45
    (4441, 2838, 0b0111000111101, 0b0000000010011), // PRN 46
    (4937, 6913, 0b0000001000100, 0b0101110011110), // PRN 47
    (3717, 1685, 0b1000101010001, 0b0001001000111), // PRN 48
    (4730, 1194, 0b0011010001001, 0b0011110000100), // PRN 49
    (7291, 6963, 0b1000111110001, 0b0100101011100), // PRN 50
    (2279, 5001, 0b1011100101001, 0b0010100011111), // PRN 51
    (7613, 6694, 0b0100101011010, 0b1101110011001), // PRN 52
    (5723, 991, 0b0000001000010, 0b0011111101111),  // PRN 53
    (7030, 7489, 0b0110001101110, 0b1100100110111), // PRN 54
    (1475, 2441, 0b0000011001110, 0b1001001100110), // PRN 55
    (2593, 639, 0b1110111011110, 0b0100010011001),  // PRN 56
    (2904, 2097, 0b0001000010011, 0b0000000001011), // PRN 57
    (2056, 2498, 0b0000010100001, 0b0000001101111), // PRN 58
    (2757, 6470, 0b0100001100001, 0b0101101101111), // PRN 59
    (3756, 2399, 0b0100101001001, 0b0100100001101), // PRN 60
    (6205, 242, 0b0011110011110, 0b1101100101011),  // PRN 61
    (5053, 3768, 0b1011000110001, 0b1010111000100), // PRN 62
    (6437, 1186, 0b0101111001011, 0b0010001101001), // PRN 63
];

/// One shift of a 13-stage register (stages 1..13 in `s[0..13]`): stage 13 is
/// the output, feedback is the XOR of the tapped stages into stage 1.
fn clock(s: &mut [u8; 13], taps: &[usize]) -> u8 {
    let out = s[12];
    let mut fb = 0u8;
    for &t in taps {
        fb ^= s[t - 1];
    }
    for i in (1..13).rev() {
        s[i] = s[i - 1];
    }
    s[0] = fb;
    out
}

/// Run `taps` for `n` clocks from `state` (all-ones if None), returning the
/// output chips and leaving `state` advanced.
fn run(taps: &[usize], n: usize, state: [u8; 13]) -> (Vec<u8>, [u8; 13]) {
    let mut s = state;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        out.push(clock(&mut s, taps));
    }
    (out, s)
}

/// The register state after `n` clocks from all-ones — a PRN's XB starting phase.
fn advance(taps: &[usize], n: usize) -> [u8; 13] {
    run(taps, n, [1u8; 13]).1
}

fn xa_sequence() -> Vec<u8> {
    run(&XA_TAPS, XA_LEN, [1u8; 13]).0
}

fn xb_sequence(prn: usize, is_q5: bool) -> Vec<u8> {
    let row = L5_TABLE[prn - 1];
    let adv = if is_q5 { row.1 } else { row.0 } as usize;
    run(&XB_TAPS, XB_LEN, advance(&XB_TAPS, adv)).0
}

/// One 1 ms L5 ranging code (10230 chips) as +1.0 / -1.0. `is_q5` selects the
/// Q5 pilot phase (else the I5 data phase). PRN 1..=63.
pub fn l5_code(prn: usize, is_q5: bool) -> Vec<f32> {
    assert!((1..=63).contains(&prn), "L5 PRN {prn} out of range (1..=63)");
    let xa = xa_sequence();
    let xb = xb_sequence(prn, is_q5);
    (0..CODE_LEN)
        .map(|k| {
            let c = xa[k % XA_LEN] ^ xb[k % XB_LEN];
            1.0 - 2.0 * c as f32
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    // The definitive correctness proof: regenerate every published XB starting
    // phase from its code advance and match the printed first-13 chips.
    #[test]
    fn reproduces_all_126_is_gps_705_phases() {
        let mut matched = 0;
        for prn in 1..=63usize {
            for is_q5 in [false, true] {
                let row = L5_TABLE[prn - 1];
                let adv = if is_q5 { row.1 } else { row.0 } as usize;
                let (first13, _) = run(&XB_TAPS, 13, advance(&XB_TAPS, adv));
                // rightmost bit is the first chip -> value = sum(chip[i] << i)
                let got: u16 = first13.iter().enumerate().fold(0u16, |a, (i, &b)| a | ((b as u16) << i));
                let want = if is_q5 { row.3 } else { row.2 };
                if got == want {
                    matched += 1;
                }
            }
        }
        assert_eq!(matched, 126, "only {matched}/126 published XB phases reproduced");
    }

    #[test]
    fn code_length_and_balance() {
        let c = l5_code(1, false);
        assert_eq!(c.len(), CODE_LEN);
        // a good ranging code is near-balanced (|ones - zeros| small vs length)
        let ones = c.iter().filter(|&&v| v < 0.0).count() as i64;
        let imbalance = (ones - (CODE_LEN as i64 - ones)).abs();
        assert!(imbalance < 200, "L5 PRN1 imbalance {imbalance}");
    }

    #[test]
    fn distinct_prns_are_nearly_orthogonal() {
        // cross-correlation of two different L5 codes is small vs the 10230 peak
        let a = l5_code(1, false);
        let b = l5_code(7, false);
        let x: f32 = a.iter().zip(&b).map(|(u, v)| u * v).sum();
        assert!(x.abs() < 0.06 * CODE_LEN as f32, "PRN1xPRN7 xcorr {x}");
        // and the autocorrelation peak is the full length
        let peak: f32 = a.iter().map(|v| v * v).sum();
        assert!((peak - CODE_LEN as f32).abs() < 1.0);
    }

    #[test]
    fn i5_and_q5_are_different_phases() {
        let i5 = l5_code(1, false);
        let q5 = l5_code(1, true);
        assert_ne!(
            i5.iter().map(|&v| (v < 0.0) as u8).collect::<Vec<_>>(),
            q5.iter().map(|&v| (v < 0.0) as u8).collect::<Vec<_>>()
        );
    }
}
