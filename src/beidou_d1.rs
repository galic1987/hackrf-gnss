//! BeiDou B1I D1 navigation-message decode (BDS-SIS-ICD-B1I; MEO/IGSO
//! satellites — GEO PRNs 1..=5 broadcast D2 at 500 bps, not handled: none are
//! visible from this station), plus the BDS broadcast-orbit/clock model with
//! ICD constants and RINEX-3 C-record parsing.
//!
//! D1 NAV: 50 bps BPSK with the 20-bit Neumann-Hofman secondary code on top
//! (1 ms per NH chip, NH period == nav-bit period, edges aligned with the
//! ranging-code period). One subframe = 300 bits = 6 s = ten 30-bit words.
//! Word 1 is NOT interleaved: [11-bit preamble 11100010010][4 reserved]
//! [one BCH(15,11,1) block: FraID 3 + SOW-msb 8 + parity 4]. Words 2..10 are
//! two bit-interleaved BCH(15,11,1) blocks each (even received bit positions
//! -> block 1, odd -> block 2).
//!
//! Two subtleties baked into the design:
//!   - BCH(15,11,3) is a PERFECT code: every 15-bit block is within one flip
//!     of a valid codeword, so per-word "validity" rejects nothing at all.
//!     Framing therefore rests on the preamble + range checks (FraID, SOW)
//!     AND cross-subframe consistency: a frame is kept only if a neighbour
//!     300 bits away carries SOW +6 and FraID +1 — see [`find_subframes`].
//!   - SOW is the seconds-of-week of BDT at the LEADING EDGE of the current
//!     subframe's preamble (unlike the GPS HOW, which names the NEXT
//!     subframe). BDT = GPST - 14 s (no leap seconds in BDT). All t_tx are
//!     carried in GPST across constellations: t_tx_gpst = (SOW + 14) mod
//!     604800 ([`sow_bdt_to_gpst`]); BDS ephemeris toe/toc get the same +14
//!     so one timescale serves the whole PVT. (Edge case: within 14 s of the
//!     week boundary the two constellations' SOWs sit on opposite sides of
//!     the wrap — a 604800 s pseudorange error for one constellation. Not
//!     handled; the fix would fail the RMS gate for those few seconds.)
//!
//! Bit positions/scales follow RTKLIB `rcvraw.c decode_bds_d1` and
//! GNSS-SDRLIB `sdrnav_bds.c` (they agree); the BCH decoder is cross-checked
//! bit-exact against GNSS-SDRLIB's syndrome table in the tests.

use std::collections::HashMap;

use crate::gps::broadcast::{
    detect_ang_unit, df_opt, df_strict, fld, i0_sane, iparse_strict, AngUnit, BrdcEph, RinexParse,
    C_LIGHT,
};

/// BDS ICD gravitational parameter (m^3/s^2).
pub const MU_BDS: f64 = 3.986004418e14;
/// BDS ICD Earth rotation rate (rad/s).
pub const OMEGA_BDS: f64 = 7.2921150e-5;
/// BDS ICD pi (used for the semicircle->radian conversions).
const PI_BDS: f64 = 3.1415926535898;
/// BDT = GPST - BDT_GPST_OFFSET (BDT has no leap seconds).
pub const BDT_GPST_OFFSET: f64 = 14.0;
const WEEK_S: f64 = 604800.0;

/// GPST seconds-of-week of an event stamped `sow_bdt` in BDT.
pub fn sow_bdt_to_gpst(sow_bdt: f64) -> f64 {
    (sow_bdt + BDT_GPST_OFFSET).rem_euclid(WEEK_S)
}

/// The 20-bit Neumann-Hofman secondary code as +1/-1 (bit 0 -> +1).
/// ICD sequence 00000 10011 01010 11010.
pub const NH20: [i8; 20] = [1, 1, 1, 1, 1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1];

/// The 11-bit D1/D2 subframe preamble (0x712 MSB-first).
const PREAMBLE: [u8; 11] = [1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0];

// ------------------------------------------------------------------ NH20 sync

/// Find the NH20 phase (= the 20 ms nav-bit boundary) in a prompt-I 1 ms
/// series: the offset o maximising the non-coherent NH correlation
/// sum_g |sum_k ms[20g+o+k]*NH20[k]|. At the right offset every 20 ms group
/// integrates coherently (~20x the per-ms amplitude, data-bit sign removed
/// by the |.|); wrong offsets see only the NH sidelobes (max |C(o)| = 4,
/// see the tests) further broken up by data-bit edges. Returns None without
/// a clear 2x margin over the runner-up.
pub fn nh_sync(ms: &[f64]) -> Option<usize> {
    if ms.len() < 400 {
        return None;
    }
    let mut acc = [0.0f64; 20];
    for (o, a) in acc.iter_mut().enumerate() {
        let (mut s, mut n) = (0.0, 0u32);
        let mut g = 0;
        while o + 20 * (g + 1) <= ms.len() {
            let c: f64 = (0..20).map(|k| ms[o + 20 * g + k] * NH20[k] as f64).sum();
            s += c.abs();
            n += 1;
            g += 1;
        }
        *a = s / n.max(1) as f64;
    }
    let best = acc
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .map(|(i, _)| i)
        .unwrap();
    let second = acc
        .iter()
        .enumerate()
        .filter(|&(i, _)| i != best)
        .map(|(_, &v)| v)
        .fold(0.0f64, f64::max);
    if acc[best] > 2.0 * second {
        Some(best)
    } else {
        None
    }
}

/// One 50 bps nav bit from twenty 1 ms prompt-I values (NH wipe + integrate).
pub fn nh_bit(ms20: &[f64]) -> u8 {
    let s: f64 = (0..20).map(|k| ms20[k] * NH20[k] as f64).sum();
    (s > 0.0) as u8
}

// ------------------------------------------------------------------ BCH(15,11)

/// LFSR remainder of the 15-bit block divided by g(x) = x^4 + x + 1, zero
/// initial state, bits fed MSB-first. Verified block-for-block identical to
/// GNSS-SDRLIB's `decodebch_bi1` LFSR (all 32768 inputs).
///
/// ICD codewords do NOT have remainder 0: the encoder initialises the
/// register to all ones, so every valid codeword carries the constant
/// remainder 8 (GNSS-SDRLIB's "no error" syndrome — its errind[8] = 0).
fn bch_lfsr(block: &[u8]) -> u8 {
    let (mut r0, mut r1, mut r2, mut r3) = (0u8, 0u8, 0u8, 0u8);
    for &b in block.iter().take(15) {
        let bit = r3;
        r3 = r2;
        r2 = r1;
        r1 = r0 ^ bit;
        r0 = b ^ bit;
    }
    r0 << 3 | r1 << 2 | r2 << 1 | r3
}

/// Remainder of a valid BCH(15,11,1) codeword (all-ones encoder init).
const BCH_COSET: u8 = 8;

/// Correct a 15-bit BCH block and return its 11 data bits. The code is
/// perfect, so every block lands within one flip of a codeword — there is no
/// "decode failure" to report; multi-bit errors are silently miscorrected
/// (cross-subframe consistency in find_subframes is the real filter).
/// (GNSS-SDRLIB's errind table is inconsistent with its own LFSR and reads
/// out of bounds at syndrome 15 — the error-position map here is built from
/// first principles instead.)
fn bch_decode(block: &[u8]) -> [u8; 11] {
    let mut w = [0u8; 15];
    w.copy_from_slice(&block[..15]);
    let s = bch_lfsr(&w) ^ BCH_COSET; // 0 for a clean codeword
    if s != 0 {
        // single-error syndromes are distinct nonzero (perfect code)
        for p in 0..15 {
            let mut e = [0u8; 15];
            e[p] = 1;
            if bch_lfsr(&e) == s {
                w[p] ^= 1;
                break;
            }
        }
    }
    let mut d = [0u8; 11];
    d.copy_from_slice(&w[..11]);
    d
}

// ------------------------------------------------------------------ bit fields

/// 0-based MSB-first unsigned field over the decoded 300-bit layout.
fn bu(d: &[u8], pos: usize, n: usize) -> u64 {
    let mut v = 0u64;
    for i in 0..n {
        v = (v << 1) | d[pos + i] as u64;
    }
    v
}

/// two's-complement signed field.
fn bi(d: &[u8], pos: usize, n: usize) -> i64 {
    let v = bu(d, pos, n) as i64;
    if (v >> (n - 1)) & 1 == 1 {
        v - (1 << n)
    } else {
        v
    }
}

/// unsigned field split across two word regions (MSB piece first).
fn u2(d: &[u8], p1: usize, l1: usize, p2: usize, l2: usize) -> u64 {
    (bu(d, p1, l1) << l2) | bu(d, p2, l2)
}

/// signed field split across two word regions.
fn s2(d: &[u8], p1: usize, l1: usize, p2: usize, l2: usize) -> i64 {
    let v = u2(d, p1, l1, p2, l2) as i64;
    let n = l1 + l2;
    if (v >> (n - 1)) & 1 == 1 {
        v - (1 << n)
    } else {
        v
    }
}

// ------------------------------------------------------------------ framing

impl D1Subframe {
    /// Diagnostic field dump (b1i_replay): the headline fields of this
    /// subframe, read at the same offsets [`parse_ephemeris`] uses — a
    /// cross-check of the field layout against RINEX on real captures.
    pub fn debug_fields(&self) -> String {
        let d = &self.data;
        match self.frid {
            1 => format!(
                "week {} toc_bdt {} af0 {:.3e} tgd1 {:.2e}",
                bu(d, 60, 13),
                u2(d, 73, 9, 90, 8) * 8,
                s2(d, 225, 7, 240, 17) as f64 * 2f64.powi(-33),
                bi(d, 98, 10) as f64 * 0.1e-9
            ),
            2 => format!(
                "sqrtA {:.1} e {:.5} toe1 {}",
                u2(d, 250, 12, 270, 20) as f64 * 2f64.powi(-19),
                u2(d, 132, 10, 150, 22) as f64 * 2f64.powi(-33),
                bu(d, 290, 2)
            ),
            3 => format!(
                "i0 {:.2} deg omg0 {:.3} omg {:.3} toe2 {}",
                (s2(d, 65, 17, 90, 15) as f64 * 2f64.powi(-31) * PI_BDS).to_degrees(),
                s2(d, 211, 21, 240, 11) as f64 * 2f64.powi(-31) * PI_BDS,
                s2(d, 251, 11, 270, 21) as f64 * 2f64.powi(-31) * PI_BDS,
                u2(d, 42, 10, 60, 5)
            ),
            _ => String::new(),
        }
    }
}

/// A frame-consistent D1 subframe. `data` is the decoded 300-bit layout:
/// word 1 raw (positions 0..30), then each word's 22 BCH-corrected data bits
/// at [30w .. 30w+22) — the same addressing RTKLIB uses, so all field
/// positions below read directly off `data`.
#[derive(Clone)]
pub struct D1Subframe {
    /// subframe ID 1..=5
    pub frid: u8,
    /// seconds of the BDT week at this subframe's preamble leading edge
    pub sow_bdt: u32,
    pub data: [u8; 300],
    /// offset of the preamble's first bit in the input stream (1 bit = 20 ms)
    pub bit_index: usize,
}

/// Decode one 300-bit window starting at a (normalised-polarity) preamble.
/// None on out-of-range FraID/SOW — the only per-frame rejection available,
/// since BCH(15,11,3) being perfect makes per-word validity vacuous.
fn subframe_decode(win: &[u8], bit_index: usize) -> Option<D1Subframe> {
    let mut data = [0u8; 300];
    data[..30].copy_from_slice(&win[..30]);
    // word 1, second half: one non-interleaved BCH block (FraID 3 + SOW-msb 8)
    let w1 = bch_decode(&win[15..30]);
    data[15..26].copy_from_slice(&w1);
    let frid = bu(&data, 15, 3) as u8;
    if !(1..=5).contains(&frid) {
        return None;
    }
    // words 2..10: deinterleave (even bits -> block 1) + BCH-correct each half
    for wd in 1..10 {
        let w = &win[30 * wd..30 * wd + 30];
        let (mut b1, mut b2) = ([0u8; 15], [0u8; 15]);
        for k in 0..15 {
            b1[k] = w[2 * k];
            b2[k] = w[2 * k + 1];
        }
        let d1 = bch_decode(&b1);
        let d2 = bch_decode(&b2);
        data[30 * wd..30 * wd + 11].copy_from_slice(&d1);
        data[30 * wd + 11..30 * wd + 22].copy_from_slice(&d2);
    }
    let sow = (bu(&data, 18, 8) << 12) | bu(&data, 30, 12);
    if sow >= WEEK_S as u64 || sow % 6 != 0 {
        return None;
    }
    Some(D1Subframe { frid, sow_bdt: sow as u32, data, bit_index })
}

/// Locate D1 subframes in a nav-bit stream (0/1 bytes, one per 20 ms).
/// Polarity is unresolved (Costas) — both preamble polarities are tried.
/// A candidate frame is kept only when a frame 300 bits before or after it
/// carries the exactly-consistent SOW (+6 s) and FraID (+1 mod 5): a random
/// preamble passes the per-frame checks with probability ~2^-11, but a
/// random PAIR agreeing to the second on SOW and on FraID is ~2^-24.
pub fn find_subframes(bits: &[u8]) -> Vec<D1Subframe> {
    let cand = find_candidates(bits);
    let consistent = |a: &D1Subframe, b: &D1Subframe| {
        b.bit_index == a.bit_index + 300
            && b.sow_bdt == (a.sow_bdt + 6) % WEEK_S as u32
            && b.frid == a.frid % 5 + 1
    };
    let keep: Vec<bool> = cand
        .iter()
        .map(|a| cand.iter().any(|b| consistent(a, b) || consistent(b, a)))
        .collect();
    cand.into_iter().zip(keep).filter(|&(_, k)| k).map(|(s, _)| s).collect()
}

/// Raw per-position scan: every window whose preamble, FraID and SOW pass
/// the per-frame checks, WITHOUT cross-subframe consistency. Diagnostic
/// surface (b1i_replay); production framing is [`find_subframes`].
pub fn find_candidates(bits: &[u8]) -> Vec<D1Subframe> {
    let n = bits.len();
    let mut cand: Vec<D1Subframe> = Vec::new();
    let mut i = 0;
    while i + 300 <= n {
        let pre = &bits[i..i + 11];
        let norm = pre == PREAMBLE;
        let inv = pre.iter().zip(PREAMBLE).all(|(x, y)| *x == 1 - y);
        if !(norm || inv) {
            i += 1;
            continue;
        }
        let win: Vec<u8> = if inv {
            bits[i..i + 300].iter().map(|b| 1 - b).collect()
        } else {
            bits[i..i + 300].to_vec()
        };
        if let Some(sf) = subframe_decode(&win, i) {
            cand.push(sf);
        }
        i += 1;
    }
    cand
}

// ------------------------------------------------------------------ ephemeris

/// Assemble D1 subframes 1/2/3 into a broadcast ephemeris (SI units, radians
/// per BDS ICD pi; toe/toc stored as GPST-equivalent SOW — see module docs).
/// The three must be consecutive (300-bit spacing, +6 s SOW steps).
/// `health` carries SatH1 (subframe 1 word 2 bit 13 — RTKLIB
/// decode_bds_d1 reads the same offset) and `rx_epoch` the wall-clock decode
/// time (lifecycle honesty: the tracker_eph.json cache envelope is
/// re-stamped on every write and must not be read as issue freshness).
pub fn parse_ephemeris(subs: &[D1Subframe]) -> Option<BrdcEph> {
    for a in subs.iter().filter(|s| s.frid == 1) {
        let b = subs.iter().find(|s| {
            s.frid == 2 && s.bit_index == a.bit_index + 300 && s.sow_bdt == (a.sow_bdt + 6) % WEEK_S as u32
        });
        let c = subs.iter().find(|s| {
            s.frid == 3 && s.bit_index == a.bit_index + 600 && s.sow_bdt == (a.sow_bdt + 12) % WEEK_S as u32
        });
        let (Some(b), Some(c)) = (b, c) else { continue };
        let (d1, d2, d3) = (&a.data, &b.data, &c.data);
        let mut e = BrdcEph { sys: 1, ..Default::default() };
        // subframe 1: week (BDT), clock, TGD1. Word 2 data layout per
        // BDS-SIS-ICD-B1I: [SOW lsb 12][SatH1 1][AODC 5][URAI 4] — SatH1 at
        // bit 42 (RTKLIB decode_bds_d1 reads svh from the same offset).
        e.health = Some(bu(d1, 42, 1) as u8);
        e.week = bu(d1, 60, 13) as f64;
        e.toc = sow_bdt_to_gpst(u2(d1, 73, 9, 90, 8) as f64 * 8.0);
        e.tgd = bi(d1, 98, 10) as f64 * 0.1e-9; // TGD1: B1I group delay
        e.af2 = bi(d1, 214, 11) as f64 * 2f64.powi(-66);
        e.af0 = s2(d1, 225, 7, 240, 17) as f64 * 2f64.powi(-33);
        e.af1 = s2(d1, 257, 5, 270, 17) as f64 * 2f64.powi(-50);
        // subframe 2: orbit part 1
        e.delta_n = s2(d2, 42, 10, 60, 6) as f64 * 2f64.powi(-43) * PI_BDS;
        e.cuc = s2(d2, 66, 16, 90, 2) as f64 * 2f64.powi(-31);
        e.m0 = s2(d2, 92, 20, 120, 12) as f64 * 2f64.powi(-31) * PI_BDS;
        e.e = u2(d2, 132, 10, 150, 22) as f64 * 2f64.powi(-33);
        e.cus = bi(d2, 180, 18) as f64 * 2f64.powi(-31);
        e.crc = s2(d2, 198, 4, 210, 14) as f64 * 2f64.powi(-6);
        e.crs = s2(d2, 224, 8, 240, 10) as f64 * 2f64.powi(-6);
        e.sqrt_a = u2(d2, 250, 12, 270, 20) as f64 * 2f64.powi(-19);
        let toe1 = bu(d2, 290, 2);
        // subframe 3: orbit part 2
        let toe2 = u2(d3, 42, 10, 60, 5);
        e.toe = sow_bdt_to_gpst(((toe1 << 15) | toe2) as f64 * 8.0);
        e.i0 = s2(d3, 65, 17, 90, 15) as f64 * 2f64.powi(-31) * PI_BDS;
        e.cic = s2(d3, 105, 7, 120, 11) as f64 * 2f64.powi(-31);
        e.omega_dot = s2(d3, 131, 11, 150, 13) as f64 * 2f64.powi(-43) * PI_BDS;
        e.cis = s2(d3, 163, 9, 180, 9) as f64 * 2f64.powi(-31);
        e.idot = s2(d3, 189, 13, 210, 1) as f64 * 2f64.powi(-43) * PI_BDS;
        e.omega0 = s2(d3, 211, 21, 240, 11) as f64 * 2f64.powi(-31) * PI_BDS;
        e.omega = s2(d3, 251, 11, 270, 21) as f64 * 2f64.powi(-31) * PI_BDS;
        // sanity: a sane BDS MEO (~27900 km) or IGSO (~42164 km) orbit.
        // (GEO would land in the IGSO range too, but GEOs broadcast D2 — a
        // different layout — and are over Asia anyway: not this station.)
        let meo = (5100.0..5450.0).contains(&e.sqrt_a);
        let igso = (6300.0..6600.0).contains(&e.sqrt_a);
        if !(meo || igso) || !(0.0..0.05).contains(&e.e) {
            continue;
        }
        // lifecycle honesty: when WE decoded it (round-11 review)
        e.rx_epoch = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .ok();
        return Some(e);
    }
    None
}

// ---------------------------------------------------------- orbit/clock model

fn kepler_e(m: f64, ecc: f64) -> f64 {
    let mut ek = m;
    for _ in 0..12 {
        ek -= (ek - ecc * ek.sin() - m) / (1.0 - ecc * ek.cos());
    }
    ek
}

fn wrap_tk(mut tk: f64) -> f64 {
    if tk > WEEK_S / 2.0 {
        tk -= WEEK_S;
    } else if tk < -WEEK_S / 2.0 {
        tk += WEEK_S;
    }
    tk
}

/// Satellite ECEF position (metres) at `t` — same Kepler equations as GPS
/// but with BDS ICD constants (mu, omega_e). `t` is the GPST-equivalent SOW
/// (toe/toc are stored shifted by +14 s, so the difference cancels the
/// BDT/GPST offset). MEO/IGSO only: GEO needs the ICD's +5 deg orbital-plane
/// transform, not implemented (no GEOs at this station).
pub fn sat_pos_ecef_bds(e: &BrdcEph, t: f64) -> [f64; 3] {
    let a = e.sqrt_a * e.sqrt_a;
    let n0 = (MU_BDS / (a * a * a)).sqrt();
    let tk = wrap_tk(t - e.toe);
    let mk = e.m0 + (n0 + e.delta_n) * tk;
    let ek = kepler_e(mk, e.e);
    let (se, ce) = (ek.sin(), ek.cos());
    let vk = ((1.0 - e.e * e.e).sqrt() * se).atan2(ce - e.e);
    let phik = vk + e.omega;
    let (s2, c2) = ((2.0 * phik).sin(), (2.0 * phik).cos());
    let uk = phik + e.cus * s2 + e.cuc * c2;
    let rk = a * (1.0 - e.e * ce) + e.crs * s2 + e.crc * c2;
    let ik = e.i0 + e.cis * s2 + e.cic * c2 + e.idot * tk;
    let (xp, yp) = (rk * uk.cos(), rk * uk.sin());
    let om = e.omega0 + (e.omega_dot - OMEGA_BDS) * tk - OMEGA_BDS * e.toe;
    let (co, so, ci, si) = (om.cos(), om.sin(), ik.cos(), ik.sin());
    [xp * co - yp * ci * so, xp * so + yp * ci * co, yp * si]
}

/// Satellite clock correction dt_sv (seconds), incl. relativity (BDS F = -2
/// sqrt(mu)/c^2 with the BDS mu) and the B1I group delay TGD1.
pub fn sat_clock_bds(e: &BrdcEph, t: f64) -> f64 {
    let f_rel = -2.0 * MU_BDS.sqrt() / (C_LIGHT * C_LIGHT);
    let dt = wrap_tk(t - e.toc);
    let poly = e.af0 + e.af1 * dt + e.af2 * dt * dt;
    let a = e.sqrt_a * e.sqrt_a;
    let n0 = (MU_BDS / (a * a * a)).sqrt();
    let tk = wrap_tk(t - e.toe);
    let mk = e.m0 + (n0 + e.delta_n) * tk;
    let ek = kepler_e(mk, e.e);
    poly + f_rel * e.e * e.sqrt_a * ek.sin() - e.tgd
}

/// Satellite ECEF (metres) at transmit time, Sagnac-rotated into the
/// reception frame with the BDS Earth rate, plus clock correction (s) and
/// geometric range (m) — the BDS twin of `gps::snapshot::sat_at_txtime_pub`.
pub fn sat_at_txtime_bds(e: &BrdcEph, t_tx: f64, rx_m: [f64; 3]) -> ([f64; 3], f64, f64) {
    let mut tau = 0.075;
    let mut s = [0.0f64; 3];
    for _ in 0..2 {
        let s0 = sat_pos_ecef_bds(e, t_tx - tau);
        let th = OMEGA_BDS * tau;
        let (ct, st) = (th.cos(), th.sin());
        s = [s0[0] * ct + s0[1] * st, -s0[0] * st + s0[1] * ct, s0[2]];
        let d = [s[0] - rx_m[0], s[1] - rx_m[1], s[2] - rx_m[2]];
        tau = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() / C_LIGHT;
    }
    let dt = sat_clock_bds(e, t_tx - tau);
    let d = [s[0] - rx_m[0], s[1] - rx_m[1], s[2] - rx_m[2]];
    let rng = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    (s, dt, rng)
}

// ------------------------------------------------------------------ RINEX BDS
//
// The strict field helpers (fld / df_strict / df_opt / iparse_strict), the
// angle-unit machinery (detect_ang_unit / i0_sane / AngUnit) and the parse
// outcome (RinexParse) are shared with the GPS parser — see
// src/gps/broadcast.rs; parser parity between the constellations is the
// point (round-11 review).

fn jdn(y: i64, m: i64, d: i64) -> i64 {
    let a = (14 - m) / 12;
    let yy = y + 4800 - a;
    let mm = m + 12 * a - 3;
    d + (153 * mm + 2) / 5 + 365 * yy + yy / 4 - yy / 100 + yy / 400 - 32045
}

/// BDT seconds-of-week for a BDS RINEX record epoch. RINEX-3 C-record epochs
/// are BDT calendar time (BDT has no leap seconds, so no leap term).
fn bdt_sow(y: i64, mo: i64, d: i64, h: i64, mi: i64, s: i64) -> f64 {
    let bdt_epoch = jdn(2006, 1, 1);
    let days = jdn(y, mo, d) - bdt_epoch;
    let secs = days * 86400 + h * 3600 + mi * 60 + s;
    (secs as f64).rem_euclid(WEEK_S)
}

/// Parse the BeiDou records of a RINEX-3 MIXED navigation file, keeping the
/// newest VALID issue per PRN. Same 8-line record layout as GPS (RINEX-3.05
/// Appendix A14); differences handled here: epochs/toe/toc are BDT (stored
/// +14 s as GPST-equivalent SOW, matching the D1 decode path and the t_tx
/// anchor), line-7 TGD field is TGD1 (the B1I group delay), line-7 field 2
/// is SatH1 -> `health`, `sys` = 1. The BDS line-2 field 1 is AODE and the
/// line-8 field 2 is AODC — different quantities from IODE/fit interval, so
/// `iode`/`iodc`/`fit_h` stay None. Same strict parsing and per-
/// constellation radians-vs-semicircles unit votes as parse_rinex_gps_nav
/// (shared helpers, broadcast.rs).
///
/// Selection: RINEX BDS weeks are continuous (3.05 §4.1.4), so the
/// (week, toe) tuple compare is rollover-exact across the week boundary.
pub fn parse_rinex_bds(text: &str) -> HashMap<u8, BrdcEph> {
    let r = parse_rinex_bds_nav(text);
    if r.rejected > 0 {
        eprintln!(
            "parse_rinex_bds: {} BDS record(s) rejected (unit {:?})",
            r.rejected, r.unit
        );
    }
    r.ephs
}

/// Full BDS-record parse with the rejection ledger (round-11 review).
pub fn parse_rinex_bds_nav(text: &str) -> RinexParse {
    let lines: Vec<&str> = text.lines().collect();
    let mut hdr = 0usize;
    while hdr < lines.len() && !lines[hdr].contains("END OF HEADER") {
        hdr += 1;
    }
    hdr += 1;
    let unit = detect_ang_unit(&lines, hdr, 'C');
    let mut out: HashMap<u8, BrdcEph> = HashMap::new();
    let mut rejected = 0usize;
    let mut i = hdr;
    while i < lines.len() {
        let ln = lines[i];
        if ln.is_empty() || !ln.starts_with('C') || i + 7 >= lines.len() {
            i += 1;
            continue;
        }
        let b: Vec<&str> = (0..7).map(|k| lines[i + 1 + k]).collect();
        match parse_bds_record(ln, &b, unit) {
            Some(e) => {
                out.entry(e.prn)
                    .and_modify(|cur| {
                        if (e.week, e.toe) > (cur.week, cur.toe) {
                            *cur = e.clone();
                        }
                    })
                    .or_insert(e);
            }
            None => rejected += 1,
        }
        i += 8;
    }
    RinexParse { ephs: out, rejected, unit }
}

/// One 8-line BDS nav record -> BrdcEph, STRICT (round-11 review): every
/// consumed field must parse — a blank/malformed core field rejects the
/// record (None) where the old df() silently zero-filled. SatH1 (health) is
/// blank-tolerant but malformed-rejecting. The raw i0 must pass [`i0_sane`]
/// for the constellation's unit verdict — which is what keeps a GEO's tiny
/// inclination (i0 ~ 0.02-0.12 rad, unit-neutral) from flipping the file,
/// while MEO/IGSO i0 (~0.93-1.03 rad vs ~0.31 semicircles) decides it.
fn parse_bds_record(ln: &str, b: &[&str], unit: AngUnit) -> Option<BrdcEph> {
    let ang = unit.factor()?; // Ambiguous fails closed: no guessed unit
    let prn = u8::try_from(iparse_strict(fld(ln, 1, 3))?).ok()?;
    let (y, mo, d) = (
        iparse_strict(fld(ln, 4, 8))?,
        iparse_strict(fld(ln, 9, 11))?,
        iparse_strict(fld(ln, 12, 14))?,
    );
    let (h, mi, s) = (
        iparse_strict(fld(ln, 15, 17))?,
        iparse_strict(fld(ln, 18, 20))?,
        iparse_strict(fld(ln, 21, 23))?,
    );
    // orbit field j on line `l`: 3-space indent, 19-char columns
    let f = |l: usize, j: usize| df_strict(fld(b[l], 4 + j * 19, 4 + (j + 1) * 19));
    let i0_raw = f(3, 0)?;
    if !i0_sane(i0_raw, unit) {
        return None;
    }
    // SatH1 (line 7 field 2): blank -> None, malformed/out of range -> reject
    let health = match df_opt(fld(b[5], 23, 42)).ok()? {
        Some(v) if (0.0..=63.0).contains(&v) => Some(v as u8),
        Some(_) => return None,
        None => None,
    };
    Some(BrdcEph {
        sys: 1,
        prn,
        // BDS nav records carry AODE (line 2 field 1) and AODC (line 8
        // field 2) — different quantities from the GPS IODE/IODC/fit
        // interval, and the SBAS LT gate is GPS-only regardless: all stay
        // None (unverifiable).
        iode: None,
        iodc: None,
        fit_h: None,
        af0: df_strict(fld(ln, 23, 42))?,
        af1: df_strict(fld(ln, 42, 61))?,
        af2: df_strict(fld(ln, 61, 80))?,
        crs: f(0, 1)?,
        delta_n: f(0, 2)? * ang,
        m0: f(0, 3)? * ang,
        cuc: f(1, 0)?,
        e: f(1, 1)?,
        cus: f(1, 2)?,
        sqrt_a: f(1, 3)?,
        toe: sow_bdt_to_gpst(f(2, 0)?),
        cic: f(2, 1)?,
        omega0: f(2, 2)? * ang,
        cis: f(2, 3)?,
        i0: i0_raw * ang,
        crc: f(3, 1)?,
        omega: f(3, 2)? * ang,
        omega_dot: f(3, 3)? * ang,
        idot: f(4, 0)? * ang,
        week: f(4, 2)?, // BDT week (continuous per RINEX-3.05 §4.1.4)
        health,
        tgd: f(5, 2)?, // TGD1: B1I group delay
        toc: sow_bdt_to_gpst(bdt_sow(y, mo, d, h, mi, s)),
        rx_epoch: None, // text-only parser: the caller attaches the file mtime
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------------- BCH(15,11,1) -------------------------------------

    /// Golden cross-check against GNSS-SDRLIB's proven B1I decoder. The
    /// values were extracted by running its actual `decodebch_bi1` LFSR
    /// (sdrnav_bds.c) over every 15-bit block: it is block-for-block
    /// identical to bch_lfsr, and a valid codeword's remainder is 8
    /// (its errind[8] = 0 = "no error"). GOLD[p] is that LFSR's raw
    /// remainder for a single bit-flip at position p of a valid codeword,
    /// i.e. BCH_COSET ^ bch_lfsr(e_p) — pinning polynomial, tap positions,
    /// bit order and packing all at once. (Its errind TABLE itself is
    /// inconsistent with its own LFSR — it would miscorrect — so the table
    /// is not used as the reference; the LFSR behaviour is.)
    #[test]
    fn bch_lfsr_matches_gnss_sdrlib_golden() {
        const GOLD: [u8; 15] = [1, 3, 7, 15, 6, 13, 2, 5, 11, 14, 4, 9, 10, 12, 0];
        for p in 0..15 {
            let mut e = [0u8; 15];
            e[p] = 1;
            assert_eq!(bch_lfsr(&e) ^ BCH_COSET, GOLD[p], "error syndrome bit {p}");
        }
        // and a codeword from the test encoder must carry the coset remainder
        let cw = bch_encode(&[1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1]);
        assert_eq!(bch_lfsr(&cw), BCH_COSET, "codeword remainder");
    }

    /// Test-local systematic encoder, consistent with bch_decode by
    /// construction: parity = the 4 bits giving the ICD coset remainder.
    fn bch_encode(data: &[u8; 11]) -> [u8; 15] {
        let mut w = [0u8; 15];
        w[..11].copy_from_slice(data);
        // find parity p with LFSR([data|p]) == BCH_COSET by linearity over GF(2)
        for p in 0..16u8 {
            let mut cand = w;
            for j in 0..4 {
                cand[11 + j] = (p >> (3 - j)) & 1;
            }
            if bch_lfsr(&cand) == BCH_COSET {
                return cand;
            }
        }
        panic!("no parity found for data");
    }

    #[test]
    fn bch_roundtrip_all_words_and_single_error_correction() {
        // single-error syndromes must be distinct and nonzero
        let mut seen = [false; 16];
        seen[0] = true;
        for p in 0..15 {
            let mut e = [0u8; 15];
            e[p] = 1;
            let s = bch_lfsr(&e) as usize;
            assert!(!seen[s], "syndrome collision at bit {p}");
            seen[s] = true;
        }
        for v in 0..2048u16 {
            let mut data = [0u8; 11];
            for j in 0..11 {
                data[j] = ((v >> (10 - j)) & 1) as u8;
            }
            let code = bch_encode(&data);
            assert_eq!(bch_decode(&code), data, "clean decode {v}");
            for p in 0..15 {
                let mut bad = code;
                bad[p] ^= 1;
                assert_eq!(bch_decode(&bad), data, "flip at {p} of word {v}");
            }
        }
    }

    // ---------------- D1 framing ----------------------------------------

    /// Test-local subframe encoder: data layout -> transmitted 300 bits
    /// (word 1 raw + word-1 BCH block; words 2..10 interleaved BCH pairs).
    fn subframe_encode(data: &[u8; 300]) -> [u8; 300] {
        let mut out = [0u8; 300];
        out[..15].copy_from_slice(&data[..15]);
        let mut w1 = [0u8; 11];
        w1.copy_from_slice(&data[15..26]);
        out[15..30].copy_from_slice(&bch_encode(&w1));
        for wd in 1..10 {
            let mut d1 = [0u8; 11];
            let mut d2 = [0u8; 11];
            d1.copy_from_slice(&data[30 * wd..30 * wd + 11]);
            d2.copy_from_slice(&data[30 * wd + 11..30 * wd + 22]);
            let b1 = bch_encode(&d1);
            let b2 = bch_encode(&d2);
            for k in 0..15 {
                out[30 * wd + 2 * k] = b1[k];
                out[30 * wd + 2 * k + 1] = b2[k];
            }
        }
        out
    }

    /// Build the 300-bit data layout for a subframe with the given FraID/SOW
    /// and filler content in the remaining data fields (parity slots are
    /// filled too but overwritten by subframe_encode).
    fn subframe_data(frid: u8, sow: u32, filler: &mut dyn FnMut(usize) -> u8) -> [u8; 300] {
        let mut d = [0u8; 300];
        for (k, &b) in PREAMBLE.iter().enumerate() {
            d[k] = b;
        }
        // reserved bits 11..15 stay 0
        for j in 0..3 {
            d[15 + j] = ((frid as u64 >> (2 - j)) & 1) as u8;
        }
        for j in 0..8 {
            d[18 + j] = ((sow as u64 >> (12 + 7 - j)) & 1) as u8;
        }
        for j in 0..12 {
            d[30 + j] = ((sow as u64 >> (11 - j)) & 1) as u8;
        }
        // filler everywhere else (word-1 bits 26..30; words 2..10 data bits
        // from 42 on, plus their parity slots)
        for k in 0..300 {
            let structured = k < 26 || (30..42).contains(&k);
            if !structured {
                d[k] = filler(k);
            }
        }
        d
    }

    #[test]
    fn d1_framing_roundtrip_both_polarities() {
        let mut st = 0x1234_5678u64;
        let mut rng = move |_| {
            st ^= st << 13;
            st ^= st >> 7;
            st ^= st << 17;
            (st >> 63) as u8
        };
        let sow0 = 234_558u32; // multiple of 6
        let mut stream: Vec<u8> = Vec::new();
        for k in 0..3u8 {
            let d = subframe_data(k + 1, sow0 + 6 * k as u32, &mut rng);
            stream.extend_from_slice(&subframe_encode(&d));
        }
        for inv in [false, true] {
            let bits: Vec<u8> = if inv {
                stream.iter().map(|b| 1 - b).collect()
            } else {
                stream.clone()
            };
            let subs = find_subframes(&bits);
            assert_eq!(subs.len(), 3, "inv={inv}");
            for (k, s) in subs.iter().enumerate() {
                assert_eq!(s.bit_index, 300 * k);
                assert_eq!(s.frid, k as u8 + 1);
                assert_eq!(s.sow_bdt, sow0 + 6 * k as u32);
            }
        }
    }

    #[test]
    fn d1_framing_rejects_unpaired_candidates() {
        // one isolated frame: no consistent neighbour -> dropped
        let mut st = 0xabcd_ef01u64;
        let mut rng = move |_| {
            st ^= st << 13;
            st ^= st >> 7;
            st ^= st << 17;
            (st >> 63) as u8
        };
        let d = subframe_data(1, 100_002, &mut rng);
        let mut stream: Vec<u8> = (0..600).map(|_| rng(0)).collect();
        stream.extend_from_slice(&subframe_encode(&d));
        stream.extend((0..600).map(|_| rng(0)));
        assert!(find_subframes(&stream).is_empty());
    }

    #[test]
    fn d1_framing_noise_is_empty() {
        let mut st = 0x5555_aaaa_1234u64;
        let bits: Vec<u8> = (0..100_000)
            .map(|_| {
                st ^= st << 13;
                st ^= st >> 7;
                st ^= st << 17;
                (st >> 63) as u8
            })
            .collect();
        assert!(find_subframes(&bits).is_empty());
    }

    #[test]
    fn d1_ephemeris_roundtrip() {
        // encode sf1-3 carrying a realistic BDS MEO ephemeris, decode, compare
        let want = BrdcEph {
            sys: 1,
            prn: 22,
            iode: None,
            iodc: None,
            health: Some(0), // zero filler -> SatH1 = 0
            fit_h: None,
            rx_epoch: None,  // set by the decode; asserted separately
            sqrt_a: 5283.0,
            e: 0.004,
            m0: 1.1,
            delta_n: 4.5e-9,
            omega0: 0.7,
            omega: -1.3,
            i0: 0.96,
            idot: 1.0e-10,
            omega_dot: -8.0e-9,
            cuc: 3.0e-6,
            cus: -4.0e-6,
            crc: 210.0,
            crs: -35.0,
            cic: 2.0e-8,
            cis: -2.5e-8,
            af0: 3.0e-4,
            af1: 1.0e-11,
            af2: 0.0,
            tgd: 2.0e-9,
            toe: sow_bdt_to_gpst(234_552.0),
            toc: sow_bdt_to_gpst(234_552.0),
            week: 900.0,
        };
        let sow0 = 234_600u32; // subframe leading edges after toe
        let mut zero = |_| 0;
        let mut d = [subframe_data(1, sow0, &mut zero), subframe_data(2, sow0 + 6, &mut zero), subframe_data(3, sow0 + 12, &mut zero)];
        let put_u = |d: &mut [u8; 300], pos: usize, n: usize, v: u64| {
            for j in 0..n {
                d[pos + j] = ((v >> (n - 1 - j)) & 1) as u8;
            }
        };
        let put2 = |d: &mut [u8; 300], p1: usize, l1: usize, p2: usize, l2: usize, v: u64| {
            put_u(d, p1, l1, v >> l2);
            put_u(d, p2, l2, v & ((1 << l2) - 1));
        };
        let twos = |v: f64, scale: f64, n: usize| -> u64 {
            let q = (v / scale).round() as i64;
            (q & ((1i64 << n) - 1)) as u64
        };
        // sf1
        put_u(&mut d[0], 60, 13, 900);
        put2(&mut d[0], 73, 9, 90, 8, (234_552 / 8) as u64);
        put_u(&mut d[0], 98, 10, twos(want.tgd / 0.1e-9, 1.0, 10));
        put_u(&mut d[0], 214, 11, 0);
        put2(&mut d[0], 225, 7, 240, 17, twos(want.af0, 2f64.powi(-33), 24));
        put2(&mut d[0], 257, 5, 270, 17, twos(want.af1, 2f64.powi(-50), 22));
        // sf2
        put2(&mut d[1], 42, 10, 60, 6, twos(want.delta_n / PI_BDS, 2f64.powi(-43), 16));
        put2(&mut d[1], 66, 16, 90, 2, twos(want.cuc, 2f64.powi(-31), 18));
        put2(&mut d[1], 92, 20, 120, 12, twos(want.m0 / PI_BDS, 2f64.powi(-31), 32));
        put2(&mut d[1], 132, 10, 150, 22, (want.e / 2f64.powi(-33)).round() as u64);
        put_u(&mut d[1], 180, 18, twos(want.cus, 2f64.powi(-31), 18));
        put2(&mut d[1], 198, 4, 210, 14, twos(want.crc, 2f64.powi(-6), 18));
        put2(&mut d[1], 224, 8, 240, 10, twos(want.crs, 2f64.powi(-6), 18));
        put2(&mut d[1], 250, 12, 270, 20, (want.sqrt_a / 2f64.powi(-19)).round() as u64);
        put_u(&mut d[1], 290, 2, ((234_552 / 8) >> 15) as u64);
        // sf3
        put2(&mut d[2], 42, 10, 60, 5, ((234_552 / 8) & 0x7FFF) as u64);
        put2(&mut d[2], 65, 17, 90, 15, twos(want.i0 / PI_BDS, 2f64.powi(-31), 32));
        put2(&mut d[2], 105, 7, 120, 11, twos(want.cic, 2f64.powi(-31), 18));
        put2(&mut d[2], 131, 11, 150, 13, twos(want.omega_dot / PI_BDS, 2f64.powi(-43), 24));
        put2(&mut d[2], 163, 9, 180, 9, twos(want.cis, 2f64.powi(-31), 18));
        put2(&mut d[2], 189, 13, 210, 1, twos(want.idot / PI_BDS, 2f64.powi(-43), 14));
        put2(&mut d[2], 211, 21, 240, 11, twos(want.omega0 / PI_BDS, 2f64.powi(-31), 32));
        put2(&mut d[2], 251, 11, 270, 21, twos(want.omega / PI_BDS, 2f64.powi(-31), 32));

        let mut stream: Vec<u8> = Vec::new();
        for dd in &d {
            stream.extend_from_slice(&subframe_encode(dd));
        }
        let subs = find_subframes(&stream);
        assert_eq!(subs.len(), 3);
        let e = parse_ephemeris(&subs).expect("ephemeris assembles");
        assert!((e.sqrt_a - want.sqrt_a).abs() < 1e-3, "sqrtA {} vs {}", e.sqrt_a, want.sqrt_a);
        assert!((e.e - want.e).abs() < 1e-9, "e");
        assert!((e.m0 - want.m0).abs() < 1e-9, "m0 {} vs {}", e.m0, want.m0);
        assert!((e.omega0 - want.omega0).abs() < 1e-9, "omg0");
        assert!((e.omega - want.omega).abs() < 1e-9, "omg");
        assert!((e.i0 - want.i0).abs() < 1e-9, "i0");
        assert!((e.toe - want.toe).abs() < 1e-6, "toe {} vs {}", e.toe, want.toe);
        assert!((e.toc - want.toc).abs() < 1e-6, "toc");
        assert!((e.af0 - want.af0).abs() < 1e-9, "af0"); // quantization 2^-33
        assert!((e.crc - want.crc).abs() < 0.1, "crc");
        assert!((e.crs - want.crs).abs() < 0.1, "crs");
        assert_eq!(e.sys, 1);
        assert_eq!(e.health, want.health, "SatH1 carries into the ephemeris");
        assert!(e.rx_epoch.is_some(), "decode stamps its receive epoch");
        // and the orbit it describes is a sane BDS MEO radius
        let p = sat_pos_ecef_bds(&e, e.toe);
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        assert!(r > 27_500_000.0 && r < 28_300_000.0, "MEO radius {r}");
    }

    // ---------------- NH20 -----------------------------------------------

    #[test]
    fn nh20_sidelobes_are_small() {
        // max periodic autocorrelation sidelobe of the NH20 code: this is
        // what a misaligned sync offset sees on a constant data stretch
        let mut worst = 0i32;
        for o in 1..20 {
            let c: i32 = (0..20).map(|k| NH20[k] as i32 * NH20[(k + o) % 20] as i32).sum();
            worst = worst.max(c.abs());
        }
        assert!(worst <= 6, "NH20 max sidelobe {worst} (peak 20)");
    }

    #[test]
    fn nh_sync_recovers_boundary_and_bits() {
        // synthesize 1 ms prompt-I: data bit * NH20 starting at offset o
        for off in [0usize, 3, 7, 13] {
            let mut st = 0x7777_1111u64 + off as u64;
            let mut rng = move || {
                st ^= st << 13;
                st ^= st >> 7;
                st ^= st << 17;
                ((st >> 40) as f64 / 8_388_608.0) - 1.0
            };
            let nbits = 100;
            let bits: Vec<u8> = (0..nbits).map(|_| (rng() > 0.0) as u8).collect();
            let mut ms = Vec::new();
            for _ in 0..off {
                ms.push(rng());
            }
            for &b in &bits {
                let s = if b == 1 { 10.0 } else { -10.0 };
                for k in 0..20 {
                    ms.push(s * NH20[k] as f64 + 1.5 * rng());
                }
            }
            let sync = nh_sync(&ms).expect("syncs");
            assert_eq!(sync, off, "offset {off}");
            let body = &ms[sync..sync + nbits * 20];
            for (g, &b) in bits.iter().enumerate() {
                assert_eq!(nh_bit(&body[g * 20..g * 20 + 20]), b, "bit {g} at offset {off}");
            }
        }
    }

    // ---------------- RINEX -----------------------------------------------

    #[test]
    fn parses_a_minimal_bds_rinex_record() {
        // one synthetic BDS record in RINEX-3 layout (values need not be
        // physical; sqrtA 5283 -> ~27900 km MEO)
        let txt = "\
     3.05           NAVIGATION DATA     MIXED               RINEX VERSION / TYPE
                                                            END OF HEADER
C22 2026 08 20 00 00 00-1.000000000000D-04 0.000000000000D+00 0.000000000000D+00
     1.000000000000D+02-5.000000000000D+00 4.000000000000D-09 3.000000000000D-01
     1.000000000000D-06 4.000000000000D-03 5.000000000000D-06 5.283000000000D+03
     3.456000000000D+05 1.000000000000D-08-2.500000000000D+00 2.000000000000D-08
     3.000000000000D-01 2.000000000000D+02-5.000000000000D-01-8.000000000000D-09
    -2.600000000000D-10 0.000000000000D+00 9.000000000000D+02 0.000000000000D+00
     0.000000000000D+00 0.000000000000D+00-1.000000000000D-08 1.000000000000D+02
     3.456000000000D+05 0.000000000000D+00 0.000000000000D+00 0.000000000000D+00";
        let ephs = parse_rinex_bds(txt);
        assert_eq!(ephs.len(), 1);
        let e = &ephs[&22];
        assert_eq!(e.prn, 22);
        assert_eq!(e.sys, 1);
        // line 7 field 2 is SatH1 (0 in this fixture); BDS records carry
        // AODE/AODC, never IODE/IODC/fit
        assert_eq!(e.health, Some(0));
        assert_eq!(e.iode, None);
        assert_eq!(e.iodc, None);
        assert_eq!(e.fit_h, None);
        assert_eq!(e.rx_epoch, None);
        assert!((e.sqrt_a - 5283.0).abs() < 1e-6);
        // toe stored as GPST-equivalent SOW (BDT + 14)
        assert!((e.toe - sow_bdt_to_gpst(345600.0)).abs() < 1e-6, "toe {}", e.toe);
        assert!((e.af0 - (-1.0e-4)).abs() < 1e-12);
        // angular field converted semicircles -> radians (M0 = 0.3 * pi)
        assert!((e.m0 - 0.3 * std::f64::consts::PI).abs() < 1e-9);
        // BDS MEO radius
        let p = sat_pos_ecef_bds(e, e.toe);
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        assert!(r > 27_500_000.0 && r < 28_300_000.0, "radius {r}");
    }

    #[test]
    fn bds_clock_reduces_to_af0_minus_tgd_at_toc() {
        let e = BrdcEph {
            sys: 1,
            sqrt_a: 5283.0,
            e: 0.0,
            af0: 1.5e-4,
            af1: 0.0,
            af2: 0.0,
            tgd: 2.0e-9,
            toe: 1000.0,
            toc: 1000.0,
            ..Default::default()
        };
        let dt = sat_clock_bds(&e, e.toc);
        assert!((dt - (1.5e-4 - 2.0e-9)).abs() < 1e-12, "dt {dt}");
    }

    /// round-11 review: reusable BDS record builder — `i0` is line 5 field 1
    /// (the unit-detection field). Line 6 fields 2/4 and line 8 fields 3/4
    /// are BLANK, exactly as in the live BKG file (blank unconsumed fields
    /// must not reject). Every written field is exactly 19 columns, first
    /// field at col 4.
    fn bds_rinex_record(prn: u8, week: f64, toe: f64, i0: &str) -> String {
        format!(
            "C{prn:02} 2026 08 20 00 00 00-1.000000000000D-04 0.000000000000D+00 0.000000000000D+00\n\
             \x20    1.000000000000D+02-5.000000000000D+00 4.000000000000D-09 3.000000000000D-01\n\
             \x20    1.000000000000D-06 4.000000000000D-03 5.000000000000D-06 5.283000000000D+03\n\
             \x20   {toe:19.12E} 1.000000000000D-08-2.500000000000D+00 2.000000000000D-08\n\
             \x20   {i0:>19} 2.000000000000D+02-5.000000000000D-01-8.000000000000D-09\n\
             \x20   -2.600000000000D-10                  {week:19.12E}\n\
             \x20    2.000000000000D+00 0.000000000000D+00 4.499999928242D-09 4.500000000000D-09\n\
             \x20   {toe:19.12E} 1.000000000000D+00"
        )
    }

    const RNX_HDR: &str = "\
     3.05           NAVIGATION DATA     MIXED               RINEX VERSION / TYPE
                                                            END OF HEADER
";

    #[test]
    fn bds_malformed_core_field_rejects_the_record() {
        let bad = bds_rinex_record(22, 1077.0, 345600.0, "3.000000000000D-01")
            .replace("5.283000000000D+03", &" ".repeat(18));
        let r = parse_rinex_bds_nav(&format!("{RNX_HDR}{bad}"));
        assert_eq!(r.rejected, 1);
        assert!(r.ephs.is_empty(), "malformed record must not enter the map");
    }

    #[test]
    fn bds_contradictory_unit_content_fails_closed() {
        let sc = bds_rinex_record(22, 1077.0, 345600.0, "3.000000000000D-01");
        let rad = bds_rinex_record(23, 1077.0, 345600.0, "9.600000000000D-01");
        let r = parse_rinex_bds_nav(&format!("{RNX_HDR}{sc}\n{rad}"));
        assert_eq!(r.unit, AngUnit::Ambiguous);
        assert_eq!(r.rejected, 2);
        assert!(r.ephs.is_empty());
    }

    #[test]
    fn bds_geo_records_carry_no_unit_evidence() {
        // a GEO-only constellation: i0 ~ 0.02 is neutral, the spec default
        // (semicircles) applies, and the record is accepted (the GEO Kepler
        // math is unused downstream regardless)
        let geo = bds_rinex_record(1, 1077.0, 345600.0, "2.000000000000D-02");
        let r = parse_rinex_bds_nav(&format!("{RNX_HDR}{geo}"));
        assert_eq!(r.unit, AngUnit::Semicircles);
        assert_eq!(r.rejected, 0);
        assert!((r.ephs[&1].i0 - 0.02 * std::f64::consts::PI).abs() < 1e-12);
        // and a grey-band i0 (0.5: impossible under either unit) rejects
        let grey = bds_rinex_record(2, 1077.0, 345600.0, "5.000000000000D-01");
        let r = parse_rinex_bds_nav(&format!("{RNX_HDR}{grey}"));
        assert_eq!(r.rejected, 1);
        assert!(r.ephs.is_empty());
    }

    #[test]
    fn bds_newest_issue_selection_is_week_rollover_exact() {
        // BDT weeks in RINEX are continuous (3.05 §4.1.4): the fresh
        // next-week issue (small toe) must displace the old one
        let old = bds_rinex_record(22, 1077.0, 604_000.0, "3.000000000000D-01");
        let new = bds_rinex_record(22, 1078.0, 200.0, "3.000000000000D-01");
        let r = parse_rinex_bds_nav(&format!("{RNX_HDR}{old}\n{new}"));
        assert_eq!(r.rejected, 0);
        assert_eq!(r.ephs[&22].week, 1078.0, "the next-week issue must win");
    }

    #[test]
    fn bds_high_inclination_igso_is_accepted() {
        // live regression (BRDC 2026-08-26): C09 is an IGSO at i0 = 1.0523
        // rad = 60.3 deg — inside [0, PI] sanity and rad-like evidence; an
        // earlier 60.1-deg rad cap rejected every one of its records
        let igso = bds_rinex_record(9, 1077.0, 345600.0, "1.052303169428D+00");
        let r = parse_rinex_bds_nav(&format!("{RNX_HDR}{igso}"));
        assert_eq!(r.unit, AngUnit::Radians);
        assert_eq!(r.rejected, 0);
        assert!((r.ephs[&9].i0 - 1.052303169428).abs() < 1e-12);
    }
}
