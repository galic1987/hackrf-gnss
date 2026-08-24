//! BeiDou B3I ranging codes (BeiDou ICD). B3I sits at 1268.52 MHz, 10230 chips
//! at 10.23 Mcps (1 ms). It is a Gold code from two 13-bit LFSRs:
//!   G1: 1 + x + x^10 + x^11 + x^13, all-ones init, SHORT-CYCLED (reset when the
//!       register reaches {0,0,1,1,...,1}) so its period is 8190.
//!   G2: 1 + x + x^3 + x^4 + x^6 + x^7 + x^8 + x^13, its per-PRN initial phase
//!       selecting the satellite.
//! code = G1 xor G2.
//!
//! Polynomial + per-PRN G2 phase table are the authoritative GNSS-SDR values
//! (`beidou_b3i_signal_replica.cc`). The test cross-checks the port against that
//! reference (golden first-24 chips) and the Gold-code structural properties.
//! (BeiDou GEO/IGSO are over Asia — not received at this station; this is the
//! code layer, verified, for completeness / an open-sky capture.)

pub const CODE_LEN: usize = 10230;

/// Per-PRN G2 initial register (bit k = stage k), PRN 1..=63.
const G2_INIT: [u16; 63] = [
    0o12777, 0o17053, 0o13612, 0o17773, 0o14437, 0o11144, 0o17722, 0o16775,
    0o12002, 0o2033, 0o16560, 0o2636, 0o6225, 0o7046, 0o10611, 0o16174,
    0o2305, 0o354, 0o10527, 0o1336, 0o2055, 0o2612, 0o1317, 0o3142,
    0o3510, 0o4451, 0o13323, 0o12742, 0o1365, 0o7777, 0o6617, 0o12611,
    0o11253, 0o14645, 0o15135, 0o17564, 0o2547, 0o16420, 0o15620, 0o15316,
    0o10064, 0o5731, 0o6674, 0o15161, 0o3442, 0o5305, 0o11746, 0o17510,
    0o511, 0o10254, 0o17114, 0o4617, 0o30, 0o10004, 0o3246, 0o13106,
    0o7170, 0o2712, 0o14766, 0o11105, 0o7040, 0o3102, 0o2116,
];

fn load(reg: u16) -> [u8; 13] {
    let mut r = [0u8; 13];
    for (k, slot) in r.iter_mut().enumerate() {
        *slot = ((reg >> k) & 1) as u8;
    }
    r
}

/// One 1 ms B3I ranging code (10230 chips) as +1.0 / -1.0. PRN 1..=63.
pub fn b3i_code(prn: usize) -> Vec<f32> {
    assert!((1..=63).contains(&prn), "B3I PRN {prn} out of range (1..=63)");
    let mut g1 = [1u8; 13];
    let mut g2 = load(G2_INIT[prn - 1]);
    // G1 short-cycle reset state: stages 0,1 low, the rest high
    let reset = {
        let mut r = [1u8; 13];
        r[0] = 0;
        r[1] = 0;
        r
    };
    let mut out = Vec::with_capacity(CODE_LEN);
    for _ in 0..CODE_LEN {
        out.push(1.0 - 2.0 * (g1[0] ^ g2[0]) as f32);
        let fb1 = g1[0] ^ g1[9] ^ g1[10] ^ g1[12];
        let fb2 = g2[0] ^ g2[1] ^ g2[3] ^ g2[4] ^ g2[6] ^ g2[7] ^ g2[8] ^ g2[12];
        for i in 0..12 {
            g1[i] = g1[i + 1];
            g2[i] = g2[i + 1];
        }
        g1[12] = fb1;
        g2[12] = fb2;
        if g1 == reset {
            g1 = [1u8; 13];
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn first24(prn: usize) -> u32 {
        let c = b3i_code(prn);
        let mut v = 0u32;
        for i in 0..24 {
            v = (v << 1) | ((c[i] < 0.0) as u32);
        }
        v
    }

    // Golden first-24 chips from the authoritative GNSS-SDR B3I generator.
    #[test]
    fn matches_reference_first24_chips() {
        for (prn, want) in [
            (1, 0o51340u32),
            (2, 0o12700750),
            (6, 0o66330754),
            (63, 0o43356551),
        ] {
            assert_eq!(first24(prn), want, "PRN {prn} first-24 mismatch");
        }
    }

    #[test]
    fn length_and_near_balance() {
        for prn in [1usize, 19, 63] {
            let c = b3i_code(prn);
            assert_eq!(c.len(), CODE_LEN);
            // a Gold code of this length is near-balanced (within a few %)
            let ones = c.iter().filter(|&&v| v < 0.0).count() as i64;
            let imbalance = (2 * ones - CODE_LEN as i64).abs();
            assert!(imbalance < 300, "PRN {prn} imbalance {imbalance}");
        }
    }

    #[test]
    fn distinct_prns_are_low_cross_correlation() {
        let a = b3i_code(1);
        let b = b3i_code(6);
        let x: f32 = a.iter().zip(&b).map(|(u, v)| u * v).sum();
        assert!(x.abs() < 0.08 * CODE_LEN as f32, "PRN1xPRN6 xcorr {x}");
        let peak: f32 = a.iter().map(|v| v * v).sum();
        assert!((peak - CODE_LEN as f32).abs() < 1.0);
    }
}
