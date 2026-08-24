//! GPS LNAV navigation-message decode (IS-GPS-200): locate parity-valid
//! subframes in a nav-bit stream and assemble subframes 1/2/3 into broadcast
//! ephemeris. This is the autonomous path — decode the ephemeris off the air
//! instead of fetching RINEX. Ported from `validation/gps_engine.py`
//! (word_decode / find_subframes / parse_ephemeris) and validated against the
//! simulator's ground-truth nav bits.

use std::collections::HashMap;

use super::broadcast::BrdcEph;

const PREAMBLE: [u8; 8] = [1, 0, 0, 0, 1, 0, 1, 1];
const PAR_PREV: [u8; 6] = [29, 30, 29, 30, 30, 29];
const PAR_IDX: [&[usize]; 6] = [
    &[1, 2, 3, 5, 6, 10, 11, 12, 13, 14, 17, 18, 20, 23],
    &[2, 3, 4, 6, 7, 11, 12, 13, 14, 15, 18, 19, 21, 24],
    &[1, 3, 4, 5, 7, 8, 12, 13, 14, 15, 16, 19, 20, 22],
    &[2, 4, 5, 6, 8, 9, 13, 14, 15, 16, 17, 20, 21, 23],
    &[1, 3, 5, 6, 7, 9, 10, 14, 15, 16, 17, 18, 21, 22, 24],
    &[3, 5, 6, 8, 9, 10, 11, 13, 15, 19, 22, 23, 24],
];
const SC: f64 = std::f64::consts::PI; // semicircles -> radians

/// A parity-valid subframe: its ID (1..=5), the TOW-count of the next
/// subframe (6 s units, from the HOW), and the ten 24-bit source words.
/// `bit_index` is the offset of the preamble's first bit in the input
/// bit stream (each bit = 20 ms of signal).
pub struct Subframe {
    pub sfid: u8,
    pub tow_next: u32,
    pub words: Vec<[u8; 24]>,
    pub bit_index: usize,
}

/// Recover 24 source bits from a 30-bit received word and check its parity.
fn word_decode(word: &[u8], d29s: u8, d30s: u8) -> ([u8; 24], bool) {
    let mut d = [0u8; 24];
    for k in 0..24 {
        d[k] = word[k] ^ d30s;
    }
    for k in 0..6 {
        let mut v = if PAR_PREV[k] == 29 { d29s } else { d30s };
        for &idx in PAR_IDX[k] {
            v ^= d[idx - 1];
        }
        if v != word[24 + k] {
            return (d, false);
        }
    }
    (d, true)
}

fn bu(b: &[u8], pos: usize, n: usize) -> u64 {
    let mut v = 0u64;
    for i in 0..n {
        v = (v << 1) | b[pos - 1 + i] as u64;
    }
    v
}

fn bi(b: &[u8], pos: usize, n: usize) -> i64 {
    let v = bu(b, pos, n) as i64;
    if (v >> (n - 1)) & 1 == 1 {
        v - (1 << n)
    } else {
        v
    }
}

/// Locate every parity-valid subframe in a nav-bit stream (0/1 bytes).
pub fn find_subframes(bits: &[u8]) -> Vec<Subframe> {
    let n = bits.len();
    let mut out = Vec::new();
    let mut i = 2usize;
    while i + 300 <= n {
        let b = &bits[i..i + 8];
        let norm = b == PREAMBLE;
        let inv = b.iter().zip(PREAMBLE).all(|(x, y)| *x == 1 - y);
        if !(norm || inv) {
            i += 1;
            continue;
        }
        let (mut a, mut bb) = (bits[i - 2], bits[i - 1]);
        let mut words = Vec::with_capacity(10);
        let mut ok = true;
        for w in 0..10 {
            let word = &bits[i + 30 * w..i + 30 * w + 30];
            let (d, good) = word_decode(word, a, bb);
            if !good {
                ok = false;
                break;
            }
            a = word[28];
            bb = word[29];
            words.push(d);
        }
        if !ok {
            i += 1;
            continue;
        }
        let how = &words[1];
        let tow_next = bu(how, 1, 17);
        let sfid = bu(how, 20, 3);
        if sfid < 1 || sfid > 5 || tow_next == 0 || tow_next > 100799 {
            i += 1;
            continue;
        }
        out.push(Subframe { sfid: sfid as u8, tow_next: tow_next as u32, words, bit_index: i });
        i += 300;
    }
    out
}

/// Assemble subframes 1/2/3 into a broadcast ephemeris (SI units, radians).
/// Returns None if any of the three are missing or fail the consistency checks.
pub fn parse_ephemeris(subs: &[Subframe]) -> Option<BrdcEph> {
    let mut got: HashMap<u8, &Vec<[u8; 24]>> = HashMap::new();
    for s in subs {
        got.entry(s.sfid).or_insert(&s.words);
    }
    let w1 = got.get(&1)?;
    let w2 = got.get(&2)?;
    let w3 = got.get(&3)?;

    let mut e = BrdcEph::default();
    // subframe 1: clock + week + TGD
    e.week = bu(&w1[2], 1, 10) as f64;
    let iodc = ((bu(&w1[2], 23, 2) << 8) | bu(&w1[7], 1, 8)) as u64;
    e.tgd = bi(&w1[6], 17, 8) as f64 * 2f64.powi(-31);
    e.toc = bu(&w1[7], 9, 16) as f64 * 16.0;
    e.af2 = bi(&w1[8], 1, 8) as f64 * 2f64.powi(-55);
    e.af1 = bi(&w1[8], 9, 16) as f64 * 2f64.powi(-43);
    e.af0 = bi(&w1[9], 1, 22) as f64 * 2f64.powi(-31);

    // subframe 2: orbit part 1
    let iode = bu(&w2[2], 1, 8);
    e.crs = bi(&w2[2], 9, 16) as f64 * 2f64.powi(-5);
    e.delta_n = bi(&w2[3], 1, 16) as f64 * 2f64.powi(-43) * SC;
    let m0 = ((bu(&w2[3], 17, 8) << 24) | bu(&w2[4], 1, 24)) as i64;
    let m0 = if m0 >> 31 == 1 { m0 - (1 << 32) } else { m0 };
    e.m0 = m0 as f64 * 2f64.powi(-31) * SC;
    e.cuc = bi(&w2[5], 1, 16) as f64 * 2f64.powi(-29);
    e.e = ((bu(&w2[5], 17, 8) << 24) | bu(&w2[6], 1, 24)) as f64 * 2f64.powi(-33);
    e.cus = bi(&w2[7], 1, 16) as f64 * 2f64.powi(-29);
    e.sqrt_a = ((bu(&w2[7], 17, 8) << 24) | bu(&w2[8], 1, 24)) as f64 * 2f64.powi(-19);
    e.toe = bu(&w2[9], 1, 16) as f64 * 16.0;

    // subframe 3: orbit part 2
    e.cic = bi(&w3[2], 1, 16) as f64 * 2f64.powi(-29);
    let o0 = ((bu(&w3[2], 17, 8) << 24) | bu(&w3[3], 1, 24)) as i64;
    let o0 = if o0 >> 31 == 1 { o0 - (1 << 32) } else { o0 };
    e.omega0 = o0 as f64 * 2f64.powi(-31) * SC;
    e.cis = bi(&w3[4], 1, 16) as f64 * 2f64.powi(-29);
    let i0 = ((bu(&w3[4], 17, 8) << 24) | bu(&w3[5], 1, 24)) as i64;
    let i0 = if i0 >> 31 == 1 { i0 - (1 << 32) } else { i0 };
    e.i0 = i0 as f64 * 2f64.powi(-31) * SC;
    e.crc = bi(&w3[6], 1, 16) as f64 * 2f64.powi(-5);
    let og = ((bu(&w3[6], 17, 8) << 24) | bu(&w3[7], 1, 24)) as i64;
    let og = if og >> 31 == 1 { og - (1 << 32) } else { og };
    e.omega = og as f64 * 2f64.powi(-31) * SC;
    e.omega_dot = bi(&w3[8], 1, 24) as f64 * 2f64.powi(-43) * SC;
    let iode3 = bu(&w3[9], 1, 8);
    e.idot = bi(&w3[9], 9, 14) as f64 * 2f64.powi(-43) * SC;

    // consistency: the three IODEs agree, and the orbit is a sane GPS orbit
    if iode != iode3 || iode != (iodc & 0xFF) {
        return None;
    }
    if !(5000.0..5500.0).contains(&e.sqrt_a) || !(0.0..0.05).contains(&e.e) {
        return None;
    }
    Some(e)
}

#[cfg(test)]
mod tests {
    use super::*;

    const NAVBITS: &str = include_str!("../../tests/fixtures/sim_navbits.txt");

    // (prn, sqrtA, toe, e) ground truth from sim.truth.json
    const TRUTH: [(u16, f64, f64, f64); 6] = [
        (20, 5153.548027, 345600.0, 0.003738211),
        (5, 5153.504997, 345600.0, 0.002109149),
        (9, 5152.679949, 345600.0, 0.003935247),
        (16, 5152.682881, 345600.0, 0.009670782),
        (10, 5154.493729, 345600.0, 0.009659233),
        (26, 5154.057816, 345600.0, 0.009324108),
    ];

    fn eph_for(prn: u16) -> Option<BrdcEph> {
        for line in NAVBITS.lines() {
            let mut it = line.split_whitespace();
            let p: u16 = it.next()?.parse().ok()?;
            if p != prn {
                continue;
            }
            let bits: Vec<u8> = it.next()?.bytes().map(|c| c - b'0').collect();
            return parse_ephemeris(&find_subframes(&bits));
        }
        None
    }

    #[test]
    fn decodes_every_sim_satellite_to_the_true_ephemeris() {
        for (prn, sqrt_a, toe, ecc) in TRUTH {
            let e = eph_for(prn).unwrap_or_else(|| panic!("PRN {prn} did not decode"));
            assert!((e.sqrt_a - sqrt_a).abs() < 1e-3, "PRN {prn} sqrtA {} vs {sqrt_a}", e.sqrt_a);
            assert!((e.toe - toe).abs() < 1.0, "PRN {prn} toe {} vs {toe}", e.toe);
            assert!((e.e - ecc).abs() < 1e-8, "PRN {prn} e {} vs {ecc}", e.e);
        }
    }

    #[test]
    fn subframes_report_their_bit_offset_in_the_stream() {
        // every PRN line of the fixture: each reported bit_index must point at
        // a preamble (normal or inverted polarity), indices must be strictly
        // increasing, and for a 300-bit-aligned pair the tow step must match
        for line in NAVBITS.lines() {
            let mut it = line.split_whitespace();
            let prn: u16 = match it.next().and_then(|s| s.parse().ok()) {
                Some(p) => p,
                None => continue,
            };
            let bits: Vec<u8> = it.next().unwrap().bytes().map(|c| c - b'0').collect();
            let subs = find_subframes(&bits);
            assert!(!subs.is_empty(), "PRN {prn} has no subframes");
            for sf in &subs {
                let pre = &bits[sf.bit_index..sf.bit_index + 8];
                let norm = pre == [1, 0, 0, 0, 1, 0, 1, 1];
                let inv = pre.iter().zip([1u8, 0, 0, 0, 1, 0, 1, 1]).all(|(&x, y)| x == 1 - y);
                assert!(norm || inv, "PRN {prn} bit_index must point at the preamble");
            }
            for w in subs.windows(2) {
                assert!(w[1].bit_index > w[0].bit_index, "PRN {prn} indices increase");
                let gap = w[1].bit_index - w[0].bit_index;
                if gap % 300 == 0 {
                    assert_eq!(
                        (w[1].tow_next - w[0].tow_next) as usize,
                        gap / 300,
                        "PRN {prn} tow step vs bit gap"
                    );
                }
            }
        }
    }

    #[test]
    fn a_stream_with_no_preamble_yields_no_subframes() {
        let bits = vec![0u8; 900];
        assert!(find_subframes(&bits).is_empty());
        assert!(parse_ephemeris(&[]).is_none());
    }

    #[test]
    fn decoded_ephemeris_places_the_satellite_in_a_gps_orbit() {
        // decode -> sat_pos_ecef must land at ~26,560 km (ties LNAV to broadcast.rs)
        let e = eph_for(20).unwrap();
        let p = super::super::broadcast::sat_pos_ecef(&e, e.toe);
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        assert!(r > 26_000_000.0 && r < 27_100_000.0, "radius {r}");
    }
}
