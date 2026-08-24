//! SBAS (WAAS/EGNOS/...) L1 PRN codes, a port of `validation/sbas.py:sbas_code`.
//!
//! SBAS uses the same 1023-chip Gold-code family as GPS C/A at 1.023 Mcps, but a
//! PRN is chosen by a G2-register DELAY (RTCA DO-229) rather than a tap pair:
//!     Code(i) = G1(i) XOR G2(i - delay)
//! The G2[i - delay] direction matters — G2[i + delay] gives the time-mirrored
//! (usually invalid) code, the bug that once broke every SBAS code here.

pub const SBAS_LO: usize = 120;
pub const SBAS_HI: usize = 158;

/// RTCA DO-229 G2 delays for PRN 120..=158.
const G2_DELAY: [u16; 39] = [
    145, 175, 52, 21, 237, 235, 886, 657, 634, 762, 355, 1012, 176, 603, 130,
    359, 595, 68, 386, 797, 456, 499, 883, 307, 127, 211, 121, 118, 163, 628,
    853, 484, 289, 811, 202, 1021, 463, 568, 904,
];

fn base_sequences() -> (Vec<u8>, Vec<u8>) {
    let mut g1 = [1u8; 10];
    let mut g2 = [1u8; 10];
    let mut cg1 = vec![0u8; 1023];
    let mut cg2 = vec![0u8; 1023];
    for i in 0..1023 {
        cg1[i] = g1[9];
        cg2[i] = g2[9];
        let fb1 = g1[2] ^ g1[9];
        let fb2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9];
        g1.copy_within(0..9, 1);
        g1[0] = fb1;
        g2.copy_within(0..9, 1);
        g2[0] = fb2;
    }
    (cg1, cg2)
}

/// The 1023-chip SBAS code for `prn` (120..=158) as +1.0 / -1.0.
pub fn sbas_code(prn: usize) -> Vec<f32> {
    assert!(
        (SBAS_LO..=SBAS_HI).contains(&prn),
        "SBAS PRN out of range: {prn}"
    );
    let d = G2_DELAY[prn - SBAS_LO] as usize;
    let (g1, g2) = base_sequences();
    (0..1023)
        .map(|i| {
            let j = (i + 1023 - d) % 1023;
            1.0 - 2.0 * ((g1[i] ^ g2[j]) as f32)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sbas_codes_are_balanced_gold_codes() {
        // a valid 1023-chip Gold code has exactly 512 ones (the -1.0 chips)
        for prn in SBAS_LO..=SBAS_HI {
            let c = sbas_code(prn);
            assert_eq!(c.len(), 1023);
            let ones = c.iter().filter(|&&v| v < 0.0).count();
            assert_eq!(ones, 512, "PRN {prn} not balanced ({ones} ones)");
        }
    }

    #[test]
    fn first_chips_match_python_oracle() {
        // ground truth from validation/sbas.py:sbas_code, chip=1 -> -1.0
        let want: [(usize, [u8; 12]); 4] = [
            (120, [0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0]),
            (131, [1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1]),
            (133, [0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1]),
            (138, [1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1]),
        ];
        for (prn, bits) in want {
            let c = sbas_code(prn);
            let got: Vec<u8> = c[..12].iter().map(|&v| if v < 0.0 { 1 } else { 0 }).collect();
            assert_eq!(got, bits.to_vec(), "SBAS PRN {prn}");
        }
    }
}
