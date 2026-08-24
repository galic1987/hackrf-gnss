//! BeiDou B2a primary ranging codes (data and pilot components): 10230 chips
//! at 10.23 Mcps, Gold codes from two 13-stage LFSRs. Register 1 starts
//! all-ones and is reset after chip 8190 of every period (short-cycled);
//! register 2 starts from the per-PRN value and runs through; both registers
//! reset at the start of each 10230-chip period.
//!
//! Provenance — nothing here is invented or reconstructed from memory:
//!   * BeiDou Navigation Satellite System Signal In Space Interface Control
//!     Document, Open Service Signal B2a (Version 1.0), December 2017, §5.2.1:
//!     data  g1(x) = 1 + x + x^5 + x^11 + x^13,
//!           g2(x) = 1 + x^3 + x^5 + x^9 + x^11 + x^12 + x^13   (eq. 5-1)
//!     pilot g1(x) = 1 + x^3 + x^6 + x^7 + x^13,
//!           g2(x) = 1 + x + x^5 + x^7 + x^8 + x^12 + x^13      (eq. 5-2)
//!     register-2 start values, first-24 and last-24 chips (octal, MSB =
//!     first chip) in Tables 5-2 (data) and 5-3 (pilot).
//!     http://www.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf
//!     (retrieved 2026-08-23)
//!   * Cross-checked against PocketSDR `src/sdr_code.c` (`gen_code_B2AD`,
//!     `gen_code_B2AP`, T. Takasu), https://github.com/tomojitakasu/PocketSDR
//!     (retrieved 2026-08-23): all 126 register-2 start values identical.
//!     Tap words map ICD cell j to bit (13 - j) of the register word below
//!     (e.g. data g1 cells {1,5,11,13} -> bits {12,8,2,0} = 0x1105).
//!
//! The unit tests regenerate every code and match all 2 x 63 published
//! first-24 AND last-24 chip vectors from ICD Tables 5-2/5-3, proving the
//! polynomials, tap/cell mapping, start values and the 8190 short-cycle are
//! all correct.

pub const CODE_LEN: usize = 10230;
const STAGES: usize = 13;
const G1_LEN: usize = 8190; // register 1 is reset after the 8190th chip
const G1_DATA_TAPS: u32 = 0x1105; // data  g1 = 1+x+x^5+x^11+x^13
const G2_DATA_TAPS: u32 = 0x0517; // data  g2 = 1+x^3+x^5+x^9+x^11+x^12+x^13
const G1_PILOT_TAPS: u32 = 0x04C1; // pilot g1 = 1+x^3+x^6+x^7+x^13
const G2_PILOT_TAPS: u32 = 0x1163; // pilot g2 = 1+x+x^5+x^7+x^8+x^12+x^13

/// Register-2 start values, B2a data component, ICD Table 5-2, PRN 1..=63.
const B2AD_INIT: [u32; 63] = [
    0x1025, 0x1034, 0x10AD, 0x114F, 0x1155, 0x11AE, 0x11EE, 0x11FB,
    0x1329, 0x13DA, 0x1435, 0x1444, 0x1455, 0x145B, 0x145C, 0x14A3,
    0x14F7, 0x1501, 0x153E, 0x15AB, 0x15B1, 0x1653, 0x1662, 0x1698,
    0x16B6, 0x16F2, 0x16FF, 0x1712, 0x173C, 0x17A1, 0x17C8, 0x17D4,
    0x17EB, 0x17F3, 0x1851, 0x1894, 0x18B7, 0x1911, 0x1919, 0x19AB,
    0x19B1, 0x19D2, 0x1A55, 0x1A74, 0x1ACB, 0x1B57, 0x1C34, 0x1C83,
    0x1C8B, 0x1CA3, 0x1CA8, 0x1D3B, 0x1D97, 0x1E48, 0x1E94, 0x1E99,
    0x1EDA, 0x1EF8, 0x1EFF, 0x1FB5, 0x0402, 0x1BF5, 0x03D2,
];

/// Register-2 start values, B2a pilot component, ICD Table 5-3, PRN 1..=63.
/// Identical to the data values except PRN 61..=63.
const B2AP_INIT: [u32; 63] = [
    0x1025, 0x1034, 0x10AD, 0x114F, 0x1155, 0x11AE, 0x11EE, 0x11FB,
    0x1329, 0x13DA, 0x1435, 0x1444, 0x1455, 0x145B, 0x145C, 0x14A3,
    0x14F7, 0x1501, 0x153E, 0x15AB, 0x15B1, 0x1653, 0x1662, 0x1698,
    0x16B6, 0x16F2, 0x16FF, 0x1712, 0x173C, 0x17A1, 0x17C8, 0x17D4,
    0x17EB, 0x17F3, 0x1851, 0x1894, 0x18B7, 0x1911, 0x1919, 0x19AB,
    0x19B1, 0x19D2, 0x1A55, 0x1A74, 0x1ACB, 0x1B57, 0x1C34, 0x1C83,
    0x1C8B, 0x1CA3, 0x1CA8, 0x1D3B, 0x1D97, 0x1E48, 0x1E94, 0x1E99,
    0x1EDA, 0x1EF8, 0x1EFF, 0x1FB5, 0x1486, 0x05F8, 0x0355,
];

/// First/last 24 chips of every code (octal, MSB = first chip), ICD Tables
/// 5-2/5-3. Used only by the unit tests as golden vectors.
#[cfg(test)]
const B2AD_FIRST24: [u32; 63] = [
    0o26771056, 0o64771737, 0o22570544, 0o03270060, 0o25270173, 0o42473731, 0o42073211, 0o10070275,
    0o32630236, 0o51032336, 0o24751346, 0o67350347, 0o25350426, 0o11351730, 0o61353105, 0o16553042,
    0o04152767, 0o37653046, 0o40653671, 0o12450445, 0o34450556, 0o15311110, 0o56310431, 0o71511012,
    0o44511144, 0o54112361, 0o00112147, 0o55611514, 0o60611442, 0o36413134, 0o73011377, 0o65011630,
    0o12011007, 0o14012245, 0o35360637, 0o65561423, 0o04561753, 0o35662052, 0o31663710, 0o12463151,
    0o34463042, 0o55063612, 0o25322050, 0o64321071, 0o13121416, 0o05223044, 0o64742223, 0o17543106,
    0o13542644, 0o16542346, 0o72542534, 0o10643011, 0o05440046, 0o73302166, 0o65502351, 0o31502177,
    0o51103567, 0o70101476, 0o00103243, 0o24403035, 0o57754771, 0o24021305, 0o55037136,
];
#[cfg(test)]
const B2AD_LAST24: [u32; 63] = [
    0o42646672, 0o43261240, 0o22122147, 0o37130044, 0o62604441, 0o32223757, 0o75444074, 0o72155517,
    0o23340625, 0o70730557, 0o12470110, 0o43367447, 0o42740075, 0o26275034, 0o77007136, 0o21516371,
    0o57170016, 0o73363551, 0o01726764, 0o65504556, 0o30230153, 0o06600771, 0o10770505, 0o76447734,
    0o05425133, 0o44374741, 0o77505753, 0o30732736, 0o43750131, 0o24525367, 0o41152341, 0o73304761,
    0o01741554, 0o35421025, 0o50337664, 0o44445660, 0o04256075, 0o50515704, 0o53542760, 0o71045216,
    0o24771613, 0o23705725, 0o75623014, 0o54464775, 0o45712211, 0o53232723, 0o57720500, 0o45401000,
    0o46456064, 0o52156646, 0o06245671, 0o42540225, 0o33645207, 0o16264764, 0o00166336, 0o33717324,
    0o23234454, 0o55337366, 0o04145264, 0o66364214, 0o16642116, 0o46402740, 0o06147764,
];
#[cfg(test)]
const B2AP_FIRST24: [u32; 63] = [
    0o26772435, 0o64771100, 0o22573033, 0o03272567, 0o25270312, 0o42471450, 0o42073477, 0o10071171,
    0o32631672, 0o51030525, 0o24752054, 0o67350376, 0o25353643, 0o11350203, 0o61350565, 0o16550214,
    0o04153006, 0o37653767, 0o40650022, 0o12453537, 0o34451342, 0o15311341, 0o56311044, 0o71513035,
    0o44513245, 0o54110251, 0o00112144, 0o55613763, 0o60613513, 0o36410413, 0o73012122, 0o65013702,
    0o12010047, 0o14010654, 0o35362324, 0o65563410, 0o04561575, 0o35663035, 0o31663420, 0o12463063,
    0o34461616, 0o55061754, 0o25322640, 0o64322743, 0o13120015, 0o05223510, 0o64741454, 0o17543717,
    0o13543302, 0o16540127, 0o72541267, 0o10642411, 0o05441614, 0o73300134, 0o65502720, 0o31500435,
    0o51103347, 0o70102511, 0o00102277, 0o24401515, 0o47551324, 0o70057625, 0o25236023,
];
#[cfg(test)]
const B2AP_LAST24: [u32; 63] = [
    0o05133452, 0o32506731, 0o46030461, 0o46247217, 0o25242712, 0o30604612, 0o46162133, 0o01037517,
    0o70661477, 0o11057614, 0o60410454, 0o57214270, 0o60621113, 0o05270220, 0o55150062, 0o30076625,
    0o40344732, 0o46567772, 0o62054544, 0o12272230, 0o71277735, 0o56036234, 0o17154331, 0o43013023,
    0o50115176, 0o56313110, 0o13102726, 0o37225071, 0o24323124, 0o20375533, 0o15635105, 0o67011450,
    0o43522666, 0o41666474, 0o06151354, 0o76525270, 0o20632513, 0o26643303, 0o52433060, 0o04062730,
    0o67067235, 0o47416277, 0o51407764, 0o66451710, 0o75211676, 0o66732705, 0o24716231, 0o43326034,
    0o37156357, 0o35671252, 0o61241434, 0o56632466, 0o13706174, 0o71335154, 0o42104070, 0o07315646,
    0o51233462, 0o46425113, 0o16705351, 0o23126772, 0o77540116, 0o31062540, 0o01076040,
];

/// Run a Fibonacci LFSR for `n` chips: output is the LSB (ICD cell 13), the
/// parity of the tapped cells feeds the MSB (ICD cell 1), register shifts
/// right — the architecture of ICD Figures 5-2/5-3.
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

/// One 1 ms B2a primary code (10230 chips) as +1.0 / -1.0. `pilot` selects
/// the pilot component (else the data component). PRN 1..=63.
pub fn b2a_code(prn: usize, pilot: bool) -> Vec<f32> {
    assert!((1..=63).contains(&prn), "B2a PRN {prn} out of range (1..=63)");
    let (g1_taps, g2_taps, g2_init) = if pilot {
        (G1_PILOT_TAPS, G2_PILOT_TAPS, B2AP_INIT[prn - 1])
    } else {
        (G1_DATA_TAPS, G2_DATA_TAPS, B2AD_INIT[prn - 1])
    };
    let g1 = lfsr(G1_LEN, 0x1FFF, g1_taps); // register 1: all-ones, reset after 8190 chips
    let g2 = lfsr(CODE_LEN, g2_init, g2_taps);
    (0..CODE_LEN)
        .map(|k| 1.0 - 2.0 * (g1[k % G1_LEN] ^ g2[k]) as f32)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pack `n` chips starting at `from` into a word, first chip = MSB (the
    /// ICD Tables 5-2/5-3 convention).
    fn chips_word(c: &[f32], from: usize, n: usize) -> u32 {
        (0..n).fold(0u32, |a, k| a | (((c[from + k] < 0.0) as u32) << (n - 1 - k)))
    }

    // The definitive correctness proof: all 63 data codes and all 63 pilot
    // codes must reproduce the published first-24 AND last-24 chip vectors
    // (the last-24 match also proves the 8190-chip short-cycle of register 1).
    #[test]
    fn reproduces_all_63_icd_data_vectors() {
        for prn in 1..=63usize {
            let c = b2a_code(prn, false);
            assert_eq!(chips_word(&c, 0, 24), B2AD_FIRST24[prn - 1], "B2a data PRN {prn} first 24");
            assert_eq!(
                chips_word(&c, CODE_LEN - 24, 24),
                B2AD_LAST24[prn - 1],
                "B2a data PRN {prn} last 24"
            );
        }
    }

    #[test]
    fn reproduces_all_63_icd_pilot_vectors() {
        for prn in 1..=63usize {
            let c = b2a_code(prn, true);
            assert_eq!(chips_word(&c, 0, 24), B2AP_FIRST24[prn - 1], "B2a pilot PRN {prn} first 24");
            assert_eq!(
                chips_word(&c, CODE_LEN - 24, 24),
                B2AP_LAST24[prn - 1],
                "B2a pilot PRN {prn} last 24"
            );
        }
    }

    #[test]
    fn code_length_and_balance() {
        for pilot in [false, true] {
            let c = b2a_code(1, pilot);
            assert_eq!(c.len(), CODE_LEN);
            let ones = c.iter().filter(|&&v| v < 0.0).count() as i64;
            let imbalance = (ones - (CODE_LEN as i64 - ones)).abs();
            assert!(imbalance < 200, "B2a PRN1 pilot={pilot} imbalance {imbalance}");
        }
    }

    #[test]
    fn distinct_codes_are_nearly_orthogonal() {
        let a = b2a_code(1, false);
        let b = b2a_code(7, false);
        let p = b2a_code(1, true);
        let x: f32 = a.iter().zip(&b).map(|(u, v)| u * v).sum();
        assert!(x.abs() < 0.06 * CODE_LEN as f32, "B2a data PRN1xPRN7 xcorr {x}");
        let y: f32 = a.iter().zip(&p).map(|(u, v)| u * v).sum();
        assert!(y.abs() < 0.06 * CODE_LEN as f32, "B2a PRN1 data x pilot xcorr {y}");
        let peak: f32 = a.iter().map(|v| v * v).sum();
        assert!((peak - CODE_LEN as f32).abs() < 1.0);
    }
}
