//! Galileo E1B I/NAV decoder — work package B of
//! docs/superpowers/specs/2026-09-02-galileo-inav-ranging-spec.md.
//!
//! Structural twin of the PROVEN python oracle `scripts/inav_reference.py`
//! (work package A: 61 tests green against the running implementation; its
//! numeric behavior is pinned into tests/fixtures/inav/*.json). Every test in
//! this file asserts bit-exact (bits) or 1e-9-exact (floats; see per-test
//! tolerance notes) agreement with those python-pinned vectors.
//!
//! FEC machinery is REUSED from src/sbas.rs — `crc24q` (identical CRC-24Q
//! polynomial, ICD 5.1.9.4), `conv_encode`/`viterbi` with `invert_g2 = true`
//! (the on-air Galileo convention: K=7, G1=171o, G2=133o with the SECOND
//! branch output inverted, ICD Table 24 / Fig. 13). No FEC code is duplicated
//! here; only the I/NAV-specific layers (30x8 interleaver, page framing,
//! word parse, ephemeris/clock/BGD/GGTO) are new.
//!
//! STATION LAW: this file has NOT been compiled or run — cargo is forbidden
//! while the live tracker runs. The tests below run at the maintenance
//! window; nothing here is claimed tested until then.
//!
//! ICD references: Galileo OS SIS ICD v2.1 (Nov 2023), tables cited inline;
//! all constants were adversarially verified against the ICD PDF per the spec.

use crate::gps::broadcast::{BrdcEph, C_LIGHT};
use crate::sbas::{conv_encode, crc24q, viterbi};

// ---------------------------------------------------------------------------
// constants (ICD-verified per the spec; values mirror inav_reference.py)
// ---------------------------------------------------------------------------

/// Page-part sync pattern, NOT encoded/interleaved (ICD 4.3.2.1).
pub const SYNC: [u8; 10] = [0, 1, 0, 1, 1, 0, 0, 0, 0, 0];

/// Block interleaver: written 30 columns (ICD Table 25) ...
pub const INTER_COLS: usize = 30;
/// ... x 8 rows, read row-by-row.
pub const INTER_ROWS: usize = 8;
/// 240 coded symbols per 1 s page part.
pub const PART_SYMS: usize = INTER_COLS * INTER_ROWS;
/// 120 decoded bits per part (114 information + 6 zero tail, ICD 4.3.2.2).
pub const PART_BITS: usize = 120;
/// Full nominal page: even part + odd part, syncs included (2 s at 250 sym/s).
pub const PAGE_SYMS: usize = 2 * (SYNC.len() + PART_SYMS);
/// Assembled I/NAV word: Data(1/2) 112 + Data(2/2) 16 bits (ICD Table 36).
pub const WORD_BITS: usize = 128;
/// CRC-24Q coverage: even 114 + odd 82 bits (ICD Table 36 note).
pub const CRC_SPAN_BITS: usize = 196;
/// sbas.rs MIN_BLOCK_WEIGHT law applied to the 196-bit CRC span: an all-zero
/// span with all-zero CRC passes arithmetically (Viterbi zero-collapse false
/// positive) — reject degenerate weights.
pub const MIN_SPAN_WEIGHT: usize = 12;
/// live.rs rescan-back for the every-second re-anchor law: one full page +
/// slack (a page needs nothing from the previous page).
pub const GAL_RESCAN_BACK: usize = PAGE_SYMS + 2;

/// Galileo gravitational parameter (ICD Table 66; == beidou MU_BDS, != GPS).
pub const MU_GAL: f64 = 3.986004418e14;
/// Galileo Earth rotation rate (ICD Table 66; GPS value, != BDS omega_e).
pub const OMEGA_E_GAL: f64 = 7.2921151467e-5;
/// Relativistic clock constant -2 sqrt(mu)/c^2 (ICD Eq. 15; GPS uses
/// -4.442807633e-10 — different mu).
pub const F_REL_GAL: f64 = -4.442807309e-10;
const WEEK_S: f64 = 604800.0;
const PI: f64 = std::f64::consts::PI;
/// GST WN 0 == GPS continuous week 1024 (ICD 5.1.2); valid to ~2077.
pub const GST_GPS_WEEK_OFFSET: u32 = 1024;

/// RINEX 3.04 Galileo "Data Sources" bits (gLAB reference; verified on live
/// brdc_latest.rnx: 258 = F/NAV rejected, 513/516/517 = I/NAV accepted).
pub const DS_INAV_E1B: u32 = 1 << 0;
/// I/NAV E5b-I (same message).
pub const DS_INAV_E5B: u32 = 1 << 2;
/// af0-2 are the (E5b,E1) clock pair -> I/NAV clock (the discriminator).
pub const DS_CLOCK_E5B_E1: u32 = 1 << 9;

/// I/NAV record selection gate for RINEX GAL records (spec 5.2): the clock
/// pair bit AND an I/NAV signal bit. F/NAV records carry the (E1,E5a) clock
/// and MUST be rejected for the E1 user equation.
pub fn data_sources_is_inav(ds: u32) -> bool {
    (ds & DS_CLOCK_E5B_E1) != 0 && (ds & (DS_INAV_E1B | DS_INAV_E5B)) != 0
}

// ---------------------------------------------------------------------------
// bit helpers (MSB-first, matching the python reference and the LNAV parsers)
// ---------------------------------------------------------------------------

/// Unsigned integer from `bits[off..off+n]`, MSB first (n <= 32).
pub fn ubits(bits: &[u8], off: usize, n: usize) -> u64 {
    let mut v = 0u64;
    for &b in &bits[off..off + n] {
        v = (v << 1) | (b & 1) as u64;
    }
    v
}

/// Two's-complement integer from `bits[off..off+n]`.
pub fn sbits(bits: &[u8], off: usize, n: usize) -> i64 {
    let v = ubits(bits, off, n);
    if v >= 1u64 << (n - 1) {
        v as i64 - (1i64 << n)
    } else {
        v as i64
    }
}

/// Write `value` (may be negative; masked to n bits) MSB-first (n <= 32).
pub fn put_bits(bits: &mut [u8], off: usize, n: usize, value: i64) {
    let v = (value as u64) & ((1u64 << n) - 1);
    for i in 0..n {
        bits[off + i] = ((v >> (n - 1 - i)) & 1) as u8;
    }
}

/// Pack MSB-first bits into lowercase hex, zero-padded to a byte boundary
/// (matches python bits_to_hex — fixture hex fields use this form).
pub fn bits_to_hex(bits: &[u8]) -> String {
    let nb = (bits.len() + 7) / 8;
    let mut bytes = vec![0u8; nb];
    for (i, &b) in bits.iter().enumerate() {
        if b & 1 != 0 {
            bytes[i / 8] |= 1 << (7 - (i % 8));
        }
    }
    bytes.iter().map(|b| format!("{:02x}", b)).collect()
}

/// Inverse of `bits_to_hex`, truncated to `nbits`.
pub fn hex_to_bits(hex: &str, nbits: usize) -> Vec<u8> {
    let mut bits = Vec::with_capacity(nbits);
    for ch in hex.chars() {
        let v = ch.to_digit(16).expect("hex digit") as u8;
        for i in (0..4).rev() {
            bits.push((v >> i) & 1);
        }
    }
    bits.truncate(nbits);
    bits
}

// ---------------------------------------------------------------------------
// 30x8 block interleaver (ICD Table 25)
// ---------------------------------------------------------------------------
// Transmit: 240 encoded symbols are WRITTEN into a matrix of 30 columns x
// 8 rows column-by-column and READ row-by-row:
//   transmitted[30*row + col] = encoded[8*col + row].
// Receiver inverse (PocketSDR: syms.reshape(8, 30).T.ravel()):
//   decoder_input[8*col + row] = received[30*row + col].

/// Transmit-side interleave (240 symbols). Generic so it serves both hard
/// (u8) test symbols and soft (f32) streams.
pub fn interleave<T: Copy + Default>(sym: &[T]) -> Vec<T> {
    assert_eq!(sym.len(), PART_SYMS);
    let mut out = vec![T::default(); PART_SYMS];
    for row in 0..INTER_ROWS {
        for col in 0..INTER_COLS {
            out[INTER_COLS * row + col] = sym[INTER_ROWS * col + row];
        }
    }
    out
}

/// Receive-side deinterleave (exact inverse of `interleave`).
pub fn deinterleave<T: Copy + Default>(sym: &[T]) -> Vec<T> {
    assert_eq!(sym.len(), PART_SYMS);
    let mut out = vec![T::default(); PART_SYMS];
    for row in 0..INTER_ROWS {
        for col in 0..INTER_COLS {
            out[INTER_ROWS * col + row] = sym[INTER_COLS * row + col];
        }
    }
    out
}

// ---------------------------------------------------------------------------
// page forward chain (ICD Table 36, E1-B column) — for tests + synthetic
// streams; the live receive path uses only the inverse below.
// ---------------------------------------------------------------------------

/// Odd-part filler fields (nonzero on live SVs; never CRC-rejected).
#[derive(Debug, Clone)]
pub struct PageExtras {
    pub osnma: [u8; 40],
    pub sar: [u8; 22],
    pub spare: [u8; 2],
    pub ssp: [u8; 8],
}

impl Default for PageExtras {
    fn default() -> Self {
        PageExtras { osnma: [0; 40], sar: [0; 22], spare: [0; 2], ssp: [0; 8] }
    }
}

/// 128-bit word -> (even 120 bits, odd 120 bits), CRC included.
///
/// Even: E/O=0 | PT | Data(1/2)[112] | tail 6
/// Odd:  E/O=1 | PT | Data(2/2)[16] | OSNMA 40 | SAR 22 | spare 2
///       | CRC 24 | SSP 8 | tail 6
/// CRC-24Q covers even[0..114] ++ odd[0..82] (196 bits); SSP/tails excluded.
pub fn build_page_parts(word: &[u8], extras: &PageExtras, page_type: u8) -> (Vec<u8>, Vec<u8>) {
    assert_eq!(word.len(), WORD_BITS);
    let mut even = Vec::with_capacity(PART_BITS);
    even.push(0);
    even.push(page_type & 1);
    even.extend_from_slice(&word[..112]);
    let mut odd = Vec::with_capacity(PART_BITS);
    odd.push(1);
    odd.push(page_type & 1);
    odd.extend_from_slice(&word[112..]);
    odd.extend_from_slice(&extras.osnma);
    odd.extend_from_slice(&extras.sar);
    odd.extend_from_slice(&extras.spare);
    let mut span = even.clone();
    span.extend_from_slice(&odd);
    debug_assert_eq!(span.len(), CRC_SPAN_BITS);
    let crc = crc24q(&span);
    for i in 0..24 {
        odd.push(((crc >> (23 - i)) & 1) as u8);
    }
    odd.extend_from_slice(&extras.ssp);
    even.extend_from_slice(&[0; 6]);
    odd.extend_from_slice(&[0; 6]);
    debug_assert_eq!(even.len(), PART_BITS);
    debug_assert_eq!(odd.len(), PART_BITS);
    (even, odd)
}

/// 120 bits -> 250 hard symbols (sync ++ interleave(conv_encode)). Each part
/// encodes independently from state 0: the previous part's 6-bit zero tail
/// returns the encoder to state 0 (ICD 4.3.2.2). `invert_g2 = true` is the
/// on-air convention; `false` exists only for the negative-vector tests.
pub fn encode_part(part: &[u8], invert_g2: bool) -> Vec<u8> {
    assert_eq!(part.len(), PART_BITS);
    let mut out = SYNC.to_vec();
    out.extend(interleave(&conv_encode(part, invert_g2, 0)));
    out
}

/// word -> 500 hard symbols (even part then odd part, syncs included).
pub fn encode_page(word: &[u8], extras: &PageExtras, page_type: u8, invert_g2: bool) -> Vec<u8> {
    let (even, odd) = build_page_parts(word, extras, page_type);
    let mut out = encode_part(&even, invert_g2);
    out.extend(encode_part(&odd, invert_g2));
    out
}

// ---------------------------------------------------------------------------
// receive inverse
// ---------------------------------------------------------------------------

/// First sync at/after `start`, trying both polarities (Costas 180-degree
/// ambiguity). Returns (index, polarity) with polarity +1 (soft > 0 == bit 0,
/// the sbas.rs convention) or -1 (inverted). Zero-valued symbols never match.
pub fn find_sync(soft: &[f32], start: usize) -> Option<(usize, i8)> {
    if soft.len() < SYNC.len() {
        return None;
    }
    let want: Vec<f32> = SYNC.iter().map(|&b| 1.0 - 2.0 * b as f32).collect();
    for i in start..=soft.len() - SYNC.len() {
        if (0..SYNC.len()).all(|k| soft[i + k] * want[k] > 0.0) {
            return Some((i, 1));
        }
        if (0..SYNC.len()).all(|k| soft[i + k] * want[k] < 0.0) {
            return Some((i, -1));
        }
    }
    None
}

/// 240 soft symbols (sync stripped) -> 120 decoded bits, via the shared
/// sbas.rs Viterbi with the Galileo G2-inverted convention.
pub fn decode_part(soft240: &[f32], polarity: i8) -> Vec<u8> {
    assert_eq!(soft240.len(), PART_SYMS);
    let s: Vec<f32> = soft240.iter().map(|&v| polarity as f32 * v).collect();
    viterbi(&deinterleave(&s), /*invert_g2=*/ true)
}

/// CRC/structure verdict on two decoded 120-bit parts.
#[derive(Debug, Clone)]
pub struct PageCheck {
    /// zero CRC syndrome over the 220-bit form AND non-degenerate weight.
    pub crc_ok: bool,
    pub weight_ok: bool,
    /// even part E/O == 0 and odd part E/O == 1.
    pub eo_ok: bool,
    /// Page Type bit (even part; 0 nominal, 1 alert).
    pub page_type: u8,
    /// CRC-clean Page Type 1: content MUST be discarded (fail-closed).
    pub alert: bool,
    /// The assembled 128-bit word — Some only for a CRC-clean nominal page.
    pub word: Option<Vec<u8>>,
}

/// CRC + structure check (mirrors inav_reference.check_page bit for bit).
pub fn check_page(even: &[u8], odd: &[u8]) -> PageCheck {
    assert_eq!(even.len(), PART_BITS);
    assert_eq!(odd.len(), PART_BITS);
    let mut span: Vec<u8> = Vec::with_capacity(CRC_SPAN_BITS + 24);
    span.extend_from_slice(&even[..114]);
    span.extend_from_slice(&odd[..82]);
    let w: usize = span.iter().map(|&b| (b & 1) as usize).sum();
    span.extend_from_slice(&odd[82..106]); // received CRC -> 220-bit form
    let weight_ok = (MIN_SPAN_WEIGHT..=CRC_SPAN_BITS - MIN_SPAN_WEIGHT).contains(&w);
    let crc_ok = crc24q(&span) == 0 && weight_ok;
    let eo_ok = even[0] == 0 && odd[0] == 1;
    let page_type = even[1];
    let alert = crc_ok && eo_ok && page_type == 1;
    let word = if crc_ok && eo_ok && page_type == 0 && odd[1] == 0 {
        let mut wbits = even[2..114].to_vec();
        wbits.extend_from_slice(&odd[2..18]);
        Some(wbits)
    } else {
        None
    };
    PageCheck { crc_ok, weight_ok, eo_ok, page_type, alert, word }
}

/// One decoded page position in a soft-symbol stream.
#[derive(Debug, Clone)]
pub struct Page {
    /// Index of the even part's first sync symbol — the instant the decoded
    /// TOW names (ICD 4.1.5: leading edge of the first page symbol).
    pub sym_index: usize,
    pub polarity: i8,
    pub check: PageCheck,
}

/// Decode the 500-symbol page whose even-part sync starts at `soft[i]`.
pub fn decode_page_at(soft: &[f32], i: usize, polarity: i8) -> Option<Page> {
    if i + PAGE_SYMS > soft.len() {
        return None;
    }
    let even = decode_part(&soft[i + 10..i + 250], polarity);
    let odd = decode_part(&soft[i + 260..i + 500], polarity);
    Some(Page { sym_index: i, polarity, check: check_page(&even, &odd) })
}

/// Scan a soft-symbol stream; return CRC-validated pages newest-last
/// (the find_subframes analogue). Sync candidates at EVERY offset, both
/// polarities; a page is accepted only on CRC pass (alert pages surface with
/// `word: None`). Advances by a full page on success, one symbol otherwise.
pub fn find_pages(soft: &[f32]) -> Vec<Page> {
    let mut pages = Vec::new();
    let mut i = 0usize;
    while i + PAGE_SYMS <= soft.len() {
        let (j, pol) = match find_sync(soft, i) {
            Some(hit) => hit,
            None => break,
        };
        match decode_page_at(soft, j, pol) {
            Some(p) if p.check.crc_ok && p.check.eo_ok => {
                pages.push(p);
                i = j + PAGE_SYMS;
            }
            _ => i = j + 1,
        }
    }
    pages
}

// ---------------------------------------------------------------------------
// word types (ICD 4.3.5; offsets per spec Section 3, 0-based from word MSB)
// ---------------------------------------------------------------------------

/// Word 1: ephemeris 1/4 (Table 40). Angles in radians, times in s.
#[derive(Debug, Clone, PartialEq)]
pub struct GalWord1 {
    pub iodnav: u16,
    pub toe: f64,
    pub m0: f64,
    pub e: f64,
    pub sqrt_a: f64,
}

/// Word 2: ephemeris 2/4 (Table 41).
#[derive(Debug, Clone, PartialEq)]
pub struct GalWord2 {
    pub iodnav: u16,
    pub omega0: f64,
    pub i0: f64,
    pub omega: f64,
    pub idot: f64,
}

/// Word 3: ephemeris 3/4 + SISA (Table 42).
#[derive(Debug, Clone, PartialEq)]
pub struct GalWord3 {
    pub iodnav: u16,
    pub omega_dot: f64,
    pub delta_n: f64,
    pub cuc: f64,
    pub cus: f64,
    pub crc: f64,
    pub crs: f64,
    pub sisa: u8,
}

/// Word 4: ephemeris 4/4 + clock (Table 43).
#[derive(Debug, Clone, PartialEq)]
pub struct GalWord4 {
    pub iodnav: u16,
    pub svid: u8,
    pub cic: f64,
    pub cis: f64,
    pub toc: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
}

/// Word 5: iono + BGD + health + GST (Table 44). TOW here is the primary
/// ranging anchor source (sits entirely in the even part).
#[derive(Debug, Clone, PartialEq)]
pub struct GalWord5 {
    pub ai0: f64,
    pub ai1: f64,
    pub ai2: f64,
    pub bgd_e1e5a: f64,
    pub bgd_e1e5b: f64,
    pub e5b_hs: u8,
    pub e1b_hs: u8,
    pub e5b_dvs: u8,
    pub e1b_dvs: u8,
    /// GST week mod 4096.
    pub wn: u16,
    pub tow: u32,
}

/// Word 6: GST-UTC conversion (Table 45); second TOW source.
#[derive(Debug, Clone, PartialEq)]
pub struct GalWord6 {
    pub a0: f64,
    pub a1: f64,
    pub dt_ls: i32,
    pub t0t: f64,
    pub wn0t: u16,
    pub wn_lsf: u16,
    pub dn: u8,
    pub dt_lsf: i32,
    pub tow: u32,
}

/// Word 0: spare word (Table 52). WN/TOW valid ONLY when Time == '10' (2) —
/// fail-closed: `time` is None otherwise. Third TOW source.
#[derive(Debug, Clone, PartialEq)]
pub struct GalWord0 {
    pub time: Option<(u16, u32)>,
}

/// Broadcast GST-GPST offset parameters, word 10 (ICD 5.1.8 / Table 74).
/// Present only when the all-ones invalid sentinel is absent.
#[derive(Debug, Clone, PartialEq)]
pub struct Ggto {
    /// seconds (broadcast 2^-35 scaling applied).
    pub a0g: f64,
    /// s/s (2^-51).
    pub a1g: f64,
    /// seconds (broadcast x3600).
    pub t0g: f64,
    /// GST week mod 64.
    pub wn0g: u16,
}

/// Word 10: almanac tail + GGTO (Table 49). The almanac fields are parsed
/// past (not retained); `ggto` is None on the all-ones sentinel (fail-closed).
#[derive(Debug, Clone, PartialEq)]
pub struct GalWord10 {
    pub ioda: u8,
    pub ggto: Option<Ggto>,
}

/// A parsed I/NAV word. Types outside {0,1,2,3,4,5,6,10} parse to None
/// (ignored, not an error) — LAW: dispatch on the decoded word type, never on
/// the nominal sub-frame slot (ICD 4.3.3 marks the sequence indicative).
#[derive(Debug, Clone, PartialEq)]
pub enum InavWord {
    W0(GalWord0),
    W1(GalWord1),
    W2(GalWord2),
    W3(GalWord3),
    W4(GalWord4),
    W5(GalWord5),
    W6(GalWord6),
    W10(GalWord10),
}

impl InavWord {
    pub fn word_type(&self) -> u8 {
        match self {
            InavWord::W0(_) => 0,
            InavWord::W1(_) => 1,
            InavWord::W2(_) => 2,
            InavWord::W3(_) => 3,
            InavWord::W4(_) => 4,
            InavWord::W5(_) => 5,
            InavWord::W6(_) => 6,
            InavWord::W10(_) => 10,
        }
    }
}

/// Parse a 128-bit assembled word. Scaling mirrors inav_reference.parse_word
/// exactly (semicircle fields: raw * 2^-n * PI, in that association order,
/// so f64 results are bit-identical to the pinned python values).
pub fn parse_word(word: &[u8]) -> Option<InavWord> {
    assert_eq!(word.len(), WORD_BITS);
    let wt = ubits(word, 0, 6);
    match wt {
        1 => Some(InavWord::W1(GalWord1 {
            iodnav: ubits(word, 6, 10) as u16,
            toe: ubits(word, 16, 14) as f64 * 60.0,
            m0: sbits(word, 30, 32) as f64 * 2f64.powi(-31) * PI,
            e: ubits(word, 62, 32) as f64 * 2f64.powi(-33),
            sqrt_a: ubits(word, 94, 32) as f64 * 2f64.powi(-19),
        })),
        2 => Some(InavWord::W2(GalWord2 {
            iodnav: ubits(word, 6, 10) as u16,
            omega0: sbits(word, 16, 32) as f64 * 2f64.powi(-31) * PI,
            i0: sbits(word, 48, 32) as f64 * 2f64.powi(-31) * PI,
            omega: sbits(word, 80, 32) as f64 * 2f64.powi(-31) * PI,
            idot: sbits(word, 112, 14) as f64 * 2f64.powi(-43) * PI,
        })),
        3 => Some(InavWord::W3(GalWord3 {
            iodnav: ubits(word, 6, 10) as u16,
            omega_dot: sbits(word, 16, 24) as f64 * 2f64.powi(-43) * PI,
            delta_n: sbits(word, 40, 16) as f64 * 2f64.powi(-43) * PI,
            cuc: sbits(word, 56, 16) as f64 * 2f64.powi(-29),
            cus: sbits(word, 72, 16) as f64 * 2f64.powi(-29),
            crc: sbits(word, 88, 16) as f64 * 2f64.powi(-5),
            crs: sbits(word, 104, 16) as f64 * 2f64.powi(-5),
            sisa: ubits(word, 120, 8) as u8,
        })),
        4 => Some(InavWord::W4(GalWord4 {
            iodnav: ubits(word, 6, 10) as u16,
            svid: ubits(word, 16, 6) as u8,
            cic: sbits(word, 22, 16) as f64 * 2f64.powi(-29),
            cis: sbits(word, 38, 16) as f64 * 2f64.powi(-29),
            toc: ubits(word, 54, 14) as f64 * 60.0,
            af0: sbits(word, 68, 31) as f64 * 2f64.powi(-34),
            af1: sbits(word, 99, 21) as f64 * 2f64.powi(-46),
            af2: sbits(word, 120, 6) as f64 * 2f64.powi(-59),
        })),
        5 => Some(InavWord::W5(GalWord5 {
            ai0: ubits(word, 6, 11) as f64 * 2f64.powi(-2),
            ai1: sbits(word, 17, 11) as f64 * 2f64.powi(-8),
            ai2: sbits(word, 28, 14) as f64 * 2f64.powi(-15),
            bgd_e1e5a: sbits(word, 47, 10) as f64 * 2f64.powi(-32),
            bgd_e1e5b: sbits(word, 57, 10) as f64 * 2f64.powi(-32),
            e5b_hs: ubits(word, 67, 2) as u8,
            e1b_hs: ubits(word, 69, 2) as u8,
            e5b_dvs: ubits(word, 71, 1) as u8,
            e1b_dvs: ubits(word, 72, 1) as u8,
            wn: ubits(word, 73, 12) as u16,
            tow: ubits(word, 85, 20) as u32,
        })),
        6 => Some(InavWord::W6(GalWord6 {
            a0: sbits(word, 6, 32) as f64 * 2f64.powi(-30),
            a1: sbits(word, 38, 24) as f64 * 2f64.powi(-50),
            dt_ls: sbits(word, 62, 8) as i32,
            t0t: ubits(word, 70, 8) as f64 * 3600.0,
            wn0t: ubits(word, 78, 8) as u16,
            wn_lsf: ubits(word, 86, 8) as u16,
            dn: ubits(word, 94, 3) as u8,
            dt_lsf: sbits(word, 97, 8) as i32,
            tow: ubits(word, 105, 20) as u32,
        })),
        0 => {
            let time_flag = ubits(word, 6, 2);
            let time = if time_flag == 2 {
                Some((ubits(word, 96, 12) as u16, ubits(word, 108, 20) as u32))
            } else {
                None // WN/TOW valid ONLY when Time == '10' — fail-closed
            };
            Some(InavWord::W0(GalWord0 { time }))
        }
        10 => {
            // GGTO invalid sentinel: all four fields all-ones (ICD 5.1.8).
            let sentinel = ubits(word, 86, 16) == 0xFFFF
                && ubits(word, 102, 12) == 0xFFF
                && ubits(word, 114, 8) == 0xFF
                && ubits(word, 122, 6) == 0x3F;
            let ggto = if sentinel {
                None
            } else {
                Some(Ggto {
                    a0g: sbits(word, 86, 16) as f64 * 2f64.powi(-35),
                    a1g: sbits(word, 102, 12) as f64 * 2f64.powi(-51),
                    t0g: ubits(word, 114, 8) as f64 * 3600.0,
                    wn0g: ubits(word, 122, 6) as u16,
                })
            };
            Some(InavWord::W10(GalWord10 { ioda: ubits(word, 6, 4) as u8, ggto }))
        }
        _ => None,
    }
}

// --------------------------- word builders (raw ints -> 128 bits) ----------
// Test/synthetic-stream builders mirroring inav_reference.make_word*.

fn new_word(wt: u32) -> Vec<u8> {
    let mut w = vec![0u8; WORD_BITS];
    put_bits(&mut w, 0, 6, wt as i64);
    w
}

pub fn make_word1(iodnav: u32, t0e: u32, m0: i64, e: i64, sqrt_a: i64) -> Vec<u8> {
    let mut w = new_word(1);
    put_bits(&mut w, 6, 10, iodnav as i64);
    put_bits(&mut w, 16, 14, t0e as i64);
    put_bits(&mut w, 30, 32, m0);
    put_bits(&mut w, 62, 32, e);
    put_bits(&mut w, 94, 32, sqrt_a);
    w
}

pub fn make_word2(iodnav: u32, omega0: i64, i0: i64, omega: i64, idot: i64) -> Vec<u8> {
    let mut w = new_word(2);
    put_bits(&mut w, 6, 10, iodnav as i64);
    put_bits(&mut w, 16, 32, omega0);
    put_bits(&mut w, 48, 32, i0);
    put_bits(&mut w, 80, 32, omega);
    put_bits(&mut w, 112, 14, idot);
    w
}

#[allow(clippy::too_many_arguments)]
pub fn make_word3(
    iodnav: u32,
    omega_dot: i64,
    delta_n: i64,
    cuc: i64,
    cus: i64,
    crc_h: i64,
    crs: i64,
    sisa: u32,
) -> Vec<u8> {
    let mut w = new_word(3);
    put_bits(&mut w, 6, 10, iodnav as i64);
    put_bits(&mut w, 16, 24, omega_dot);
    put_bits(&mut w, 40, 16, delta_n);
    put_bits(&mut w, 56, 16, cuc);
    put_bits(&mut w, 72, 16, cus);
    put_bits(&mut w, 88, 16, crc_h);
    put_bits(&mut w, 104, 16, crs);
    put_bits(&mut w, 120, 8, sisa as i64);
    w
}

#[allow(clippy::too_many_arguments)]
pub fn make_word4(
    iodnav: u32,
    svid: u32,
    cic: i64,
    cis: i64,
    t0c: u32,
    af0: i64,
    af1: i64,
    af2: i64,
) -> Vec<u8> {
    let mut w = new_word(4);
    put_bits(&mut w, 6, 10, iodnav as i64);
    put_bits(&mut w, 16, 6, svid as i64);
    put_bits(&mut w, 22, 16, cic);
    put_bits(&mut w, 38, 16, cis);
    put_bits(&mut w, 54, 14, t0c as i64);
    put_bits(&mut w, 68, 31, af0);
    put_bits(&mut w, 99, 21, af1);
    put_bits(&mut w, 120, 6, af2);
    w
}

#[allow(clippy::too_many_arguments)]
pub fn make_word5(
    ai0: u32,
    ai1: i64,
    ai2: i64,
    bgd_e1e5a: i64,
    bgd_e1e5b: i64,
    e5b_hs: u32,
    e1b_hs: u32,
    e5b_dvs: u32,
    e1b_dvs: u32,
    wn: u32,
    tow: u32,
    region: [u8; 5],
) -> Vec<u8> {
    let mut w = new_word(5);
    put_bits(&mut w, 6, 11, ai0 as i64);
    put_bits(&mut w, 17, 11, ai1);
    put_bits(&mut w, 28, 14, ai2);
    for (i, &r) in region.iter().enumerate() {
        put_bits(&mut w, 42 + i, 1, r as i64);
    }
    put_bits(&mut w, 47, 10, bgd_e1e5a);
    put_bits(&mut w, 57, 10, bgd_e1e5b);
    put_bits(&mut w, 67, 2, e5b_hs as i64);
    put_bits(&mut w, 69, 2, e1b_hs as i64);
    put_bits(&mut w, 71, 1, e5b_dvs as i64);
    put_bits(&mut w, 72, 1, e1b_dvs as i64);
    put_bits(&mut w, 73, 12, wn as i64);
    put_bits(&mut w, 85, 20, tow as i64);
    w
}

#[allow(clippy::too_many_arguments)]
pub fn make_word6(
    a0: i64,
    a1: i64,
    dt_ls: i64,
    t0t: u32,
    wn0t: u32,
    wn_lsf: u32,
    dn: u32,
    dt_lsf: i64,
    tow: u32,
) -> Vec<u8> {
    let mut w = new_word(6);
    put_bits(&mut w, 6, 32, a0);
    put_bits(&mut w, 38, 24, a1);
    put_bits(&mut w, 62, 8, dt_ls);
    put_bits(&mut w, 70, 8, t0t as i64);
    put_bits(&mut w, 78, 8, wn0t as i64);
    put_bits(&mut w, 86, 8, wn_lsf as i64);
    put_bits(&mut w, 94, 3, dn as i64);
    put_bits(&mut w, 97, 8, dt_lsf);
    put_bits(&mut w, 105, 20, tow as i64);
    w
}

pub fn make_word0(time_flag: u32, wn: u32, tow: u32) -> Vec<u8> {
    let mut w = new_word(0);
    put_bits(&mut w, 6, 2, time_flag as i64);
    put_bits(&mut w, 96, 12, wn as i64);
    put_bits(&mut w, 108, 20, tow as i64);
    w
}

/// Word-10 almanac tail (SVID3 2/2) raw ints — carried through the builder so
/// synthetic pages can have realistic nonzero content.
#[derive(Debug, Clone, Default)]
pub struct AlmTail10 {
    pub omega0: i64,
    pub omega_dot: i64,
    pub m0: i64,
    pub af0: i64,
    pub af1: i64,
    pub e5b_hs: u32,
    pub e1b_hs: u32,
}

pub fn make_word10(ioda: u32, a0g: i64, a1g: i64, t0g: u32, wn0g: u32, alm: &AlmTail10) -> Vec<u8> {
    let mut w = new_word(10);
    put_bits(&mut w, 6, 4, ioda as i64);
    put_bits(&mut w, 10, 16, alm.omega0);
    put_bits(&mut w, 26, 11, alm.omega_dot);
    put_bits(&mut w, 37, 16, alm.m0);
    put_bits(&mut w, 53, 16, alm.af0);
    put_bits(&mut w, 69, 13, alm.af1);
    put_bits(&mut w, 82, 2, alm.e5b_hs as i64);
    put_bits(&mut w, 84, 2, alm.e1b_hs as i64);
    put_bits(&mut w, 86, 16, a0g);
    put_bits(&mut w, 102, 12, a1g);
    put_bits(&mut w, 114, 8, t0g as i64);
    put_bits(&mut w, 122, 6, wn0g as i64);
    w
}

// ---------------------------------------------------------------------------
// ephemeris batch assembly (words 1-4 same IODnav; word 4 carries clock)
// ---------------------------------------------------------------------------

/// Assembled I/NAV ephemeris: a BrdcEph (so the existing selection/health
/// belts fire unmodified) plus the Galileo-only issue/quality fields.
/// Conventions inside `eph`: sys = 2 (Galileo), `tgd` holds BGD(E1,E5b) so
/// `sat_clock_gal`'s subtraction is the exact analogue of GPS `- e.tgd`,
/// `week` = GST WN + 1024 (GPS-continuous; 0.0 when no WN source yet),
/// `health` = Some(0) healthy or Some(HS<<1 | DVS) (two-sided law).
#[derive(Debug, Clone)]
pub struct GalEph {
    pub eph: BrdcEph,
    pub iodnav: u16,
    pub sisa: u8,
}

/// Batch gate (ICD 5.1.9.2): words 1-4 must carry the SAME IODnav — reject
/// mixed sets. BGD/health ride word 5, which carries NO IODnav: pair the
/// newest valid word 5 by recency, never by IOD. `gst_wn` (word 5/6/0 TOW
/// source) sets the continuous week; word 5's WN is used when absent.
pub fn assemble_ephemeris(
    w1: &GalWord1,
    w2: &GalWord2,
    w3: &GalWord3,
    w4: &GalWord4,
    w5: Option<&GalWord5>,
    gst_wn: Option<u16>,
) -> Option<GalEph> {
    if !(w1.iodnav == w2.iodnav && w2.iodnav == w3.iodnav && w3.iodnav == w4.iodnav) {
        return None;
    }
    let mut eph = BrdcEph {
        sys: 2,
        prn: w4.svid,
        toe: w1.toe,
        toc: w4.toc,
        week: 0.0,
        sqrt_a: w1.sqrt_a,
        e: w1.e,
        m0: w1.m0,
        delta_n: w3.delta_n,
        omega0: w2.omega0,
        omega: w2.omega,
        i0: w2.i0,
        idot: w2.idot,
        omega_dot: w3.omega_dot,
        cuc: w3.cuc,
        cus: w3.cus,
        crc: w3.crc,
        crs: w3.crs,
        cic: w4.cic,
        cis: w4.cis,
        af0: w4.af0,
        af1: w4.af1,
        af2: w4.af2,
        tgd: 0.0,
        ..Default::default()
    };
    let mut wn = gst_wn;
    if let Some(f5) = w5 {
        eph.tgd = f5.bgd_e1e5b;
        let composite = if f5.e1b_hs == 0 && f5.e1b_dvs == 0 {
            0
        } else {
            (f5.e1b_hs << 1) | f5.e1b_dvs
        };
        eph.health = Some(composite);
        if wn.is_none() {
            wn = Some(f5.wn);
        }
    } else {
        eph.health = Some(0);
    }
    if let Some(w) = wn {
        eph.week = (w as u32 + GST_GPS_WEEK_OFFSET) as f64;
    }
    Some(GalEph { eph, iodnav: w1.iodnav, sisa: w3.sisa })
}

// ---------------------------------------------------------------------------
// Keplerian evaluation (GPS pipeline, Galileo constants — ICD Table 66)
// Local kepler_e/wrap_tk mirror src/gps/broadcast.rs (the beidou_d1
// precedent: constants differ, the algorithm is identical — don't fork it).
// ---------------------------------------------------------------------------

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

/// Satellite ECEF (m) at GST time-of-week `t` (ICD Table 66 == IS-GPS-200
/// form with mu/omega_e swapped for the Galileo values).
pub fn sat_pos_ecef_gal(e: &BrdcEph, t: f64) -> [f64; 3] {
    let a = e.sqrt_a * e.sqrt_a;
    let n0 = (MU_GAL / (a * a * a)).sqrt();
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
    let om = e.omega0 + (e.omega_dot - OMEGA_E_GAL) * tk - OMEGA_E_GAL * e.toe;
    let (co, so, ci, si) = (om.cos(), om.sin(), ik.cos(), ik.sin());
    [xp * co - yp * ci * so, xp * so + yp * ci * co, yp * si]
}

/// dt_sv for the E1-only user: clock poly + relativity - BGD(E1,E5b).
/// ICD Eq. 13-14 give the (E1,E5b) pair clock; Eq. 17 (f1 = E1) subtracts
/// the broadcast BGD(E1,E5b), stored in `e.tgd` — the exact analogue of the
/// GPS `- e.tgd` term. Set `tgd = 0.0` to get the raw (E1,E5b) clock.
pub fn sat_clock_gal(e: &BrdcEph, t: f64) -> f64 {
    let dt = wrap_tk(t - e.toc);
    let poly = e.af0 + e.af1 * dt + e.af2 * dt * dt;
    let a = e.sqrt_a * e.sqrt_a;
    let n0 = (MU_GAL / (a * a * a)).sqrt();
    let tk = wrap_tk(t - e.toe);
    let mk = e.m0 + (n0 + e.delta_n) * tk;
    let ek = kepler_e(mk, e.e);
    poly + F_REL_GAL * e.e * e.sqrt_a * ek.sin() - e.tgd
}

/// Satellite ECEF (m) at transmit time, Sagnac-rotated into the reception
/// frame, plus clock (s) and geometric range (m) — the Galileo twin of
/// `beidou_d1::sat_at_txtime_bds` (tau = 0.075 start, two iterations,
/// rotation by OMEGA_E_GAL * tau, clock at t_tx - tau).
pub fn sat_at_txtime_gal(e: &BrdcEph, t_tx: f64, rx_m: [f64; 3]) -> ([f64; 3], f64, f64) {
    let mut tau = 0.075;
    let mut s = [0.0f64; 3];
    for _ in 0..2 {
        let s0 = sat_pos_ecef_gal(e, t_tx - tau);
        let th = OMEGA_E_GAL * tau;
        let (ct, st) = (th.cos(), th.sin());
        s = [s0[0] * ct + s0[1] * st, -s0[0] * st + s0[1] * ct, s0[2]];
        let d = [s[0] - rx_m[0], s[1] - rx_m[1], s[2] - rx_m[2]];
        tau = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() / C_LIGHT;
    }
    let dt = sat_clock_gal(e, t_tx - tau);
    let d = [s[0] - rx_m[0], s[1] - rx_m[1], s[2] - rx_m[2]];
    let rng = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    (s, dt, rng)
}

// ---------------------------------------------------------------------------
// GGTO (ICD 5.1.8 Eq. 23) — spec 5.4 decision Option B
// ---------------------------------------------------------------------------

/// dt_systems = t_Galileo - t_GPS at GST (wn, tow). WN0G is GST week mod 64;
/// rollover law |dW| <= 31 (mod-64 fold).
pub fn ggto_offset(g: &Ggto, tow: f64, wn: u32) -> f64 {
    let mut dw = (wn as i64 - g.wn0g as i64).rem_euclid(64);
    if dw > 31 {
        dw -= 64;
    }
    g.a0g + g.a1g * (tow - g.t0g + WEEK_S * dw as f64)
}

/// t_tx GST -> GPST using the cached word-10 GGTO; identity when absent
/// (fail-closed: apply zero, the residual folds into the unmodeled ISB the
/// analyzer quantifies via n_gal — spec 5.4 Option B).
pub fn apply_ggto(t_tx_gst: f64, ggto: Option<&Ggto>, tow: f64, wn: u32) -> f64 {
    match ggto {
        Some(g) => t_tx_gst - ggto_offset(g, tow, wn),
        None => t_tx_gst,
    }
}

// ===========================================================================
// tests — run at the maintenance window (cargo forbidden while the live
// tracker runs). Every vector is python-pinned: scripts/inav_reference.py
// generated tests/fixtures/inav/*.json AFTER scripts/test_inav_reference.py
// passed against the running python implementation.
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const FX_CRC: &str = include_str!("../tests/fixtures/inav/crc24q_vectors.json");
    const FX_CONV: &str = include_str!("../tests/fixtures/inav/conv_viterbi_vectors.json");
    const FX_INTER: &str = include_str!("../tests/fixtures/inav/interleaver_vectors.json");
    const FX_PAGES: &str = include_str!("../tests/fixtures/inav/pages_synthetic.json");
    const FX_RINEX: &str = include_str!("../tests/fixtures/inav/rinex_gal_eval.json");
    const FX_GGTO: &str = include_str!("../tests/fixtures/inav/ggto_bgd_vectors.json");

    fn fx(text: &str) -> Value {
        serde_json::from_str(text).expect("fixture JSON")
    }

    fn jf(v: &Value) -> f64 {
        v.as_f64().expect("f64 field")
    }

    fn ju(v: &Value) -> u64 {
        v.as_u64().expect("u64 field")
    }

    fn ji(v: &Value) -> i64 {
        v.as_i64().expect("i64 field")
    }

    fn jstr<'a>(v: &'a Value) -> &'a str {
        v.as_str().expect("str field")
    }

    fn jbits(case: &Value, hex_key: &str, nbits: usize) -> Vec<u8> {
        hex_to_bits(jstr(&case[hex_key]), nbits)
    }

    /// Soft symbols for a page case: the pinned noise vector when present,
    /// otherwise +-1 from the hard symbols, with the case polarity applied
    /// (the fixture's symbols_hex is always the upright encoder output).
    fn case_soft(case: &Value) -> Vec<f32> {
        if let Some(soft) = case.get("soft").and_then(|s| s.as_array()) {
            return soft.iter().map(|v| jf(v) as f32).collect();
        }
        let nsym = ju(&case["nsym"]) as usize;
        let sym = jbits(case, "symbols_hex", nsym);
        let pol = ji(&case["polarity"]) as f32;
        sym.iter().map(|&s| pol * (1.0 - 2.0 * s as f32)).collect()
    }

    /// Bit-exact float agreement is expected (identical IEEE op order to the
    /// python oracle); tolerance exists only for transcendental-library
    /// last-ulp differences, far inside the task's 1e-9 bar.
    fn close(a: f64, b: f64, tol: f64) {
        assert!(
            (a - b).abs() <= tol,
            "float mismatch: {a} vs {b} (tol {tol})"
        );
    }

    // ---------------- CRC-24Q (reused sbas::crc24q) ------------------------

    #[test]
    fn crc24q_fixture_vectors() {
        let d = fx(FX_CRC);
        assert_eq!(ju(&d["poly"]) as u32, crate::sbas::CRC24Q_POLY);
        for v in d["vectors"].as_array().unwrap() {
            let bits = jbits(v, "bits_hex", ju(&v["nbits"]) as usize);
            let crc = crc24q(&bits);
            assert_eq!(crc, ju(&v["crc"]) as u32, "{}", jstr(&v["name"]));
            if let Some(exp) = v.get("crc_expect") {
                assert_eq!(crc, ju(exp) as u32);
            }
            if let Some(z) = v.get("zero_syndrome_over_220") {
                // appended-CRC linear-code property: syndrome over span+CRC
                let mut full = bits.clone();
                for i in 0..24 {
                    full.push(((crc >> (23 - i)) & 1) as u8);
                }
                assert_eq!(crc24q(&full), ju(z) as u32);
                assert_eq!(crc24q(&full), 0);
            }
        }
        // one-bit damage must break the zero syndrome (196-bit span vector)
        let v = &d["vectors"].as_array().unwrap()[3];
        let bits = jbits(v, "bits_hex", CRC_SPAN_BITS);
        let crc = crc24q(&bits);
        let mut full = bits.clone();
        for i in 0..24 {
            full.push(((crc >> (23 - i)) & 1) as u8);
        }
        full[57] ^= 1;
        assert_ne!(crc24q(&full), 0);
    }

    // ---------------- conv encode / Viterbi (reused sbas machinery) --------

    #[test]
    fn conv_noiseless_both_conventions() {
        let d = fx(FX_CONV);
        for case in d["cases"].as_array().unwrap() {
            let name = jstr(&case["name"]);
            if !name.starts_with("noiseless_") {
                continue;
            }
            let inv = case["invert_g2"].as_bool().unwrap();
            let bits = jbits(case, "bits_hex", ju(&case["nbits"]) as usize);
            let sym = conv_encode(&bits, inv, 0);
            assert_eq!(bits_to_hex(&sym), jstr(&case["sym_hex"]), "{name}");
            assert_eq!(sym.len(), ju(&case["nsym"]) as usize);
            let soft: Vec<f32> = sym.iter().map(|&s| 1.0 - 2.0 * s as f32).collect();
            assert_eq!(viterbi(&soft, inv), bits, "{name}");
            assert!(case["decoded_matches"].as_bool().unwrap());
        }
    }

    #[test]
    fn conv_gauss_soft_vector() {
        let d = fx(FX_CONV);
        let case = d["cases"]
            .as_array()
            .unwrap()
            .iter()
            .find(|c| jstr(&c["name"]).starts_with("gauss_"))
            .unwrap();
        let bits = jbits(case, "bits_hex", ju(&case["nbits"]) as usize);
        let soft: Vec<f32> = case["soft"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| jf(v) as f32)
            .collect();
        assert!(case["decoded_matches"].as_bool().unwrap());
        assert_eq!(viterbi(&soft, true), bits);
    }

    #[test]
    fn conv_hard_flip_correction() {
        let d = fx(FX_CONV);
        let case = d["cases"]
            .as_array()
            .unwrap()
            .iter()
            .find(|c| jstr(&c["name"]).starts_with("hard_12flips"))
            .unwrap();
        let bits = jbits(case, "bits_hex", ju(&case["nbits"]) as usize);
        let flipped = jbits(case, "sym_flipped_hex", ju(&case["nsym"]) as usize);
        let soft: Vec<f32> = flipped.iter().map(|&s| 1.0 - 2.0 * s as f32).collect();
        assert_eq!(viterbi(&soft, true), bits);
    }

    #[test]
    fn conv_g2_convention_differs() {
        // guards the classic Galileo gotcha: the two G2 conventions must
        // produce different streams and our encoder must match both pins.
        let d = fx(FX_CONV);
        let case = d["cases"]
            .as_array()
            .unwrap()
            .iter()
            .find(|c| jstr(&c["name"]) == "g2_convention_differs")
            .unwrap();
        assert!(!case["identical"].as_bool().unwrap());
        let bits = jbits(case, "bits_hex", ju(&case["nbits"]) as usize);
        let s_inv = conv_encode(&bits, true, 0);
        let s_non = conv_encode(&bits, false, 0);
        assert_eq!(bits_to_hex(&s_inv), jstr(&case["sym_invert_hex"]));
        assert_eq!(bits_to_hex(&s_non), jstr(&case["sym_noninvert_hex"]));
        assert_ne!(s_inv, s_non);
    }

    // ---------------- interleaver ------------------------------------------

    #[test]
    fn interleaver_fixture() {
        let d = fx(FX_INTER);
        assert_eq!(ju(&d["rows"]) as usize, INTER_ROWS);
        assert_eq!(ju(&d["cols"]) as usize, INTER_COLS);
        // permutation law: transmitted[i] = encoded[perm[i]]
        let perm: Vec<usize> = d["permutation"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| ju(v) as usize)
            .collect();
        let ident: Vec<u16> = (0..PART_SYMS as u16).collect();
        let inter = interleave(&ident);
        for (i, &p) in perm.iter().enumerate() {
            assert_eq!(inter[i] as usize, p);
        }
        let fixture_ident: Vec<u16> = d["identity_interleaved"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| ju(v) as u16)
            .collect();
        assert_eq!(inter, fixture_ident);
        // bit pattern in/out pair + roundtrip
        let pat = jbits(&d, "pattern_in_hex", PART_SYMS);
        let out = interleave(&pat);
        assert_eq!(bits_to_hex(&out), jstr(&d["pattern_out_hex"]));
        assert_eq!(deinterleave(&out), pat);
        assert!(d["roundtrip_ok"].as_bool().unwrap());
    }

    // ---------------- full pages: decode every pinned case ------------------

    #[test]
    fn pages_decode_all_pinned_cases() {
        let d = fx(FX_PAGES);
        let sync: Vec<u8> = d["sync"].as_array().unwrap().iter().map(|v| ju(v) as u8).collect();
        assert_eq!(sync, SYNC.to_vec());
        for case in d["cases"].as_array().unwrap() {
            let name = jstr(&case["name"]);
            let soft = case_soft(case);
            let exp_pol = ji(&case["polarity"]) as i8;
            let (j, pol) = find_sync(&soft, 0).expect(name);
            assert_eq!(j, 0, "{name}: sync index");
            assert_eq!(pol, exp_pol, "{name}: polarity");
            let page = decode_page_at(&soft, 0, pol).expect(name);
            assert_eq!(
                page.check.crc_ok,
                case["decode_crc_ok"].as_bool().unwrap(),
                "{name}: crc_ok"
            );
            assert_eq!(
                page.check.alert,
                case["decode_alert"].as_bool().unwrap(),
                "{name}: alert"
            );
            let want_word = case["decode_word_matches"].as_bool().unwrap();
            match (&page.check.word, want_word) {
                (Some(w), true) => {
                    let exp = jbits(case, "word_hex", WORD_BITS);
                    assert_eq!(*w, exp, "{name}: word bits");
                }
                (Some(_), false) => panic!("{name}: unexpected word decode"),
                (None, true) => panic!("{name}: word expected but None"),
                (None, false) => {}
            }
        }
    }

    #[test]
    fn pages_reencode_byte_identical() {
        // Encoder pin against python symbols: decoded 120-bit parts contain
        // the full transmitted content (word + OSNMA/SAR/spare/CRC/SSP +
        // tails), so re-encoding them must reproduce symbols_hex exactly.
        let d = fx(FX_PAGES);
        let mut checked = 0;
        for case in d["cases"].as_array().unwrap() {
            if case.get("soft").is_some()
                || !case["decode_crc_ok"].as_bool().unwrap()
                || !case["invert_g2"].as_bool().unwrap()
            {
                continue; // noisy soft or negative cases: no bit-exact symbols
            }
            let name = jstr(&case["name"]);
            let soft = case_soft(case);
            let (_, pol) = find_sync(&soft, 0).unwrap();
            let even = decode_part(&soft[10..250], pol);
            let odd = decode_part(&soft[260..500], pol);
            let mut sym = encode_part(&even, true);
            sym.extend(encode_part(&odd, true));
            assert_eq!(bits_to_hex(&sym), jstr(&case["symbols_hex"]), "{name}");
            checked += 1;
        }
        assert!(checked >= 6, "expected several clean re-encode cases");
    }

    #[test]
    fn page_roundtrip_own_encoder_both_polarities() {
        // self-contained forward+inverse roundtrip (zero extras), both
        // polarities, plus the degenerate-weight guard.
        let word = make_word1(27, 4080, -123456789, 987654, 2851669515);
        let sym = encode_page(&word, &PageExtras::default(), 0, true);
        assert_eq!(sym.len(), PAGE_SYMS);
        for pol in [1i8, -1i8] {
            let soft: Vec<f32> = sym
                .iter()
                .map(|&s| pol as f32 * (1.0 - 2.0 * s as f32))
                .collect();
            let (j, p) = find_sync(&soft, 0).unwrap();
            assert_eq!((j, p), (0, pol));
            let page = decode_page_at(&soft, 0, p).unwrap();
            assert!(page.check.crc_ok && page.check.eo_ok);
            assert_eq!(page.check.word.as_ref().unwrap(), &word);
        }
        // all-zero 196-bit span passes CRC arithmetically -> weight guard
        let zeros = vec![0u8; PART_BITS];
        let mut odd = vec![0u8; PART_BITS];
        odd[0] = 1; // E/O=1 keeps span weight 1 (< MIN_SPAN_WEIGHT)
        let chk = check_page(&zeros, &odd);
        assert!(!chk.weight_ok && !chk.crc_ok);
    }

    #[test]
    fn find_pages_stream_with_garbage() {
        let d = fx(FX_PAGES);
        let cases = d["cases"].as_array().unwrap();
        let by_name = |n: &str| cases.iter().find(|c| jstr(&c["name"]) == n).unwrap();
        let p1 = case_soft(by_name("word1_clean"));
        let p5 = case_soft(by_name("word5_clean"));
        // alternating garbage never matches either sync polarity (no equal
        // adjacent signs), so the scanner walks straight to the pages.
        let mut soft: Vec<f32> = (0..37).map(|i| if i % 2 == 0 { 0.8 } else { -0.8 }).collect();
        soft.extend_from_slice(&p1);
        soft.extend_from_slice(&p5);
        soft.extend((0..23).map(|i| if i % 2 == 0 { -0.6f32 } else { 0.6 }));
        let pages = find_pages(&soft);
        assert_eq!(pages.len(), 2);
        assert_eq!(pages[0].sym_index, 37);
        assert_eq!(pages[1].sym_index, 37 + PAGE_SYMS);
        let w1 = jbits(by_name("word1_clean"), "word_hex", WORD_BITS);
        let w5 = jbits(by_name("word5_clean"), "word_hex", WORD_BITS);
        assert_eq!(pages[0].check.word.as_ref().unwrap(), &w1);
        assert_eq!(pages[1].check.word.as_ref().unwrap(), &w5);
        // TOW lattice: pinned word 5 parses and carries the anchor TOW
        match parse_word(&w5).unwrap() {
            InavWord::W5(f5) => assert!(f5.tow > 0),
            _ => panic!("word 5 expected"),
        }
    }

    // ---------------- word builders + parsers vs pinned raw/fields ----------

    fn raw_i(case: &Value, key: &str) -> i64 {
        ji(&case["parsed"]["raw"][key])
    }

    fn raw_u(case: &Value, key: &str) -> u64 {
        ju(&case["parsed"]["raw"][key])
    }

    fn field_f(case: &Value, key: &str) -> f64 {
        jf(&case["parsed"]["fields"][key])
    }

    #[test]
    fn word_builders_match_pinned_word_hex() {
        let d = fx(FX_PAGES);
        for case in d["cases"].as_array().unwrap() {
            let parsed = &case["parsed"];
            if parsed.is_null() {
                continue;
            }
            let name = jstr(&case["name"]);
            if name.starts_with("negative_") || name.starts_with("alert_") {
                continue; // same words as word1_clean; builders pinned there
            }
            let wt = ju(&parsed["word_type"]);
            let built = match wt {
                1 => make_word1(
                    raw_u(case, "iodnav") as u32,
                    raw_u(case, "t0e") as u32,
                    raw_i(case, "m0"),
                    raw_i(case, "e"),
                    raw_i(case, "sqrt_a"),
                ),
                2 => make_word2(
                    raw_u(case, "iodnav") as u32,
                    raw_i(case, "omega0"),
                    raw_i(case, "i0"),
                    raw_i(case, "omega"),
                    raw_i(case, "idot"),
                ),
                3 => make_word3(
                    raw_u(case, "iodnav") as u32,
                    raw_i(case, "omega_dot"),
                    raw_i(case, "delta_n"),
                    raw_i(case, "cuc"),
                    raw_i(case, "cus"),
                    raw_i(case, "crc"),
                    raw_i(case, "crs"),
                    raw_u(case, "sisa") as u32,
                ),
                4 => make_word4(
                    raw_u(case, "iodnav") as u32,
                    raw_u(case, "svid") as u32,
                    raw_i(case, "cic"),
                    raw_i(case, "cis"),
                    raw_u(case, "t0c") as u32,
                    raw_i(case, "af0"),
                    raw_i(case, "af1"),
                    raw_i(case, "af2"),
                ),
                5 => {
                    let region: Vec<u8> = case["parsed"]["raw"]["region"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .map(|v| ju(v) as u8)
                        .collect();
                    make_word5(
                        raw_u(case, "ai0") as u32,
                        raw_i(case, "ai1"),
                        raw_i(case, "ai2"),
                        raw_i(case, "bgd_e1e5a"),
                        raw_i(case, "bgd_e1e5b"),
                        raw_u(case, "e5b_hs") as u32,
                        raw_u(case, "e1b_hs") as u32,
                        raw_u(case, "e5b_dvs") as u32,
                        raw_u(case, "e1b_dvs") as u32,
                        raw_u(case, "wn") as u32,
                        raw_u(case, "tow") as u32,
                        [region[0], region[1], region[2], region[3], region[4]],
                    )
                }
                6 => make_word6(
                    raw_i(case, "a0"),
                    raw_i(case, "a1"),
                    raw_i(case, "dt_ls"),
                    raw_u(case, "t0t") as u32,
                    raw_u(case, "wn0t") as u32,
                    raw_u(case, "wn_lsf") as u32,
                    raw_u(case, "dn") as u32,
                    raw_i(case, "dt_lsf"),
                    raw_u(case, "tow") as u32,
                ),
                0 => make_word0(
                    raw_u(case, "time_flag") as u32,
                    raw_u(case, "wn") as u32,
                    raw_u(case, "tow") as u32,
                ),
                10 => make_word10(
                    raw_u(case, "ioda") as u32,
                    raw_i(case, "a0g"),
                    raw_i(case, "a1g"),
                    raw_u(case, "t0g") as u32,
                    raw_u(case, "wn0g") as u32,
                    &AlmTail10 {
                        omega0: raw_i(case, "alm_omega0"),
                        omega_dot: raw_i(case, "alm_omega_dot"),
                        m0: raw_i(case, "alm_m0"),
                        af0: raw_i(case, "alm_af0"),
                        af1: raw_i(case, "alm_af1"),
                        e5b_hs: raw_u(case, "alm_e5b_hs") as u32,
                        e1b_hs: raw_u(case, "alm_e1b_hs") as u32,
                    },
                ),
                _ => panic!("unexpected word type {wt} in fixtures"),
            };
            let exp = jbits(case, "word_hex", WORD_BITS);
            assert_eq!(built, exp, "{name}: builder bits");
        }
    }

    #[test]
    fn word_parse_fields_match_pinned() {
        // scaled engineering fields: identical IEEE op order to python ->
        // exact f64 agreement expected; tolerance 0.0 (strict ==) via close.
        let d = fx(FX_PAGES);
        for case in d["cases"].as_array().unwrap() {
            let parsed = &case["parsed"];
            if parsed.is_null() {
                continue;
            }
            let name = jstr(&case["name"]);
            if name.starts_with("negative_") || name.starts_with("alert_") {
                continue;
            }
            let word = jbits(case, "word_hex", WORD_BITS);
            let w = parse_word(&word).expect(name);
            assert_eq!(w.word_type() as u64, ju(&parsed["word_type"]), "{name}");
            match w {
                InavWord::W1(f) => {
                    assert_eq!(f.iodnav as u64, ju(&parsed["fields"]["iodnav"]));
                    close(f.toe, field_f(case, "toe"), 0.0);
                    close(f.m0, field_f(case, "m0"), 0.0);
                    close(f.e, field_f(case, "e"), 0.0);
                    close(f.sqrt_a, field_f(case, "sqrt_a"), 0.0);
                }
                InavWord::W2(f) => {
                    close(f.omega0, field_f(case, "omega0"), 0.0);
                    close(f.i0, field_f(case, "i0"), 0.0);
                    close(f.omega, field_f(case, "omega"), 0.0);
                    close(f.idot, field_f(case, "idot"), 0.0);
                }
                InavWord::W3(f) => {
                    close(f.omega_dot, field_f(case, "omega_dot"), 0.0);
                    close(f.delta_n, field_f(case, "delta_n"), 0.0);
                    close(f.cuc, field_f(case, "cuc"), 0.0);
                    close(f.cus, field_f(case, "cus"), 0.0);
                    close(f.crc, field_f(case, "crc"), 0.0);
                    close(f.crs, field_f(case, "crs"), 0.0);
                    assert_eq!(f.sisa as f64, field_f(case, "sisa"));
                }
                InavWord::W4(f) => {
                    assert_eq!(f.svid as u64, ju(&parsed["fields"]["svid"]));
                    close(f.cic, field_f(case, "cic"), 0.0);
                    close(f.cis, field_f(case, "cis"), 0.0);
                    close(f.toc, field_f(case, "toc"), 0.0);
                    close(f.af0, field_f(case, "af0"), 0.0);
                    close(f.af1, field_f(case, "af1"), 0.0);
                    close(f.af2, field_f(case, "af2"), 0.0);
                }
                InavWord::W5(f) => {
                    close(f.ai0, field_f(case, "ai0"), 0.0);
                    close(f.ai1, field_f(case, "ai1"), 0.0);
                    close(f.ai2, field_f(case, "ai2"), 0.0);
                    close(f.bgd_e1e5a, field_f(case, "bgd_e1e5a"), 0.0);
                    close(f.bgd_e1e5b, field_f(case, "bgd_e1e5b"), 0.0);
                    assert_eq!(f.e1b_hs as u64, ju(&parsed["fields"]["e1b_hs"]));
                    assert_eq!(f.e1b_dvs as u64, ju(&parsed["fields"]["e1b_dvs"]));
                    assert_eq!(f.wn as u64, ju(&parsed["fields"]["wn"]));
                    assert_eq!(f.tow as u64, ju(&parsed["fields"]["tow"]));
                }
                InavWord::W6(f) => {
                    close(f.a0, field_f(case, "a0"), 0.0);
                    close(f.a1, field_f(case, "a1"), 0.0);
                    assert_eq!(f.dt_ls as i64, ji(&parsed["fields"]["dt_ls"]));
                    close(f.t0t, field_f(case, "t0t"), 0.0);
                    assert_eq!(f.dn as u64, ju(&parsed["fields"]["dn"]));
                    assert_eq!(f.dt_lsf as i64, ji(&parsed["fields"]["dt_lsf"]));
                    assert_eq!(f.tow as u64, ju(&parsed["fields"]["tow"]));
                }
                InavWord::W0(f) => {
                    let valid = parsed["fields"]["time_valid"].as_bool().unwrap();
                    assert_eq!(f.time.is_some(), valid, "{name}: Time gate");
                    if let Some((wn, tow)) = f.time {
                        assert_eq!(wn as u64, ju(&parsed["fields"]["wn"]));
                        assert_eq!(tow as u64, ju(&parsed["fields"]["tow"]));
                    }
                }
                InavWord::W10(f) => {
                    let valid = parsed["fields"]["ggto_valid"].as_bool().unwrap();
                    assert_eq!(f.ggto.is_some(), valid, "{name}: GGTO sentinel");
                    if let Some(g) = f.ggto {
                        close(g.a0g, field_f(case, "a0g"), 0.0);
                        close(g.a1g, field_f(case, "a1g"), 0.0);
                        close(g.t0g, field_f(case, "t0g"), 0.0);
                        assert_eq!(g.wn0g as u64, ju(&parsed["fields"]["wn0g"]));
                    }
                }
            }
        }
    }

    #[test]
    fn sign_extension_extremes() {
        // most-negative / most-positive raw values roundtrip every signed
        // width used by the parsers (native guard, no fixture needed).
        let w = make_word4(1, 2, -(1 << 15), (1 << 15) - 1, 0, -(1 << 30), -(1 << 20), -32);
        match parse_word(&w).unwrap() {
            InavWord::W4(f) => {
                assert_eq!(f.cic, -(1i64 << 15) as f64 * 2f64.powi(-29));
                assert_eq!(f.cis, ((1i64 << 15) - 1) as f64 * 2f64.powi(-29));
                assert_eq!(f.af0, -(1i64 << 30) as f64 * 2f64.powi(-34));
                assert_eq!(f.af1, -(1i64 << 20) as f64 * 2f64.powi(-46));
                assert_eq!(f.af2, -32.0 * 2f64.powi(-59));
            }
            _ => panic!(),
        }
        let w = make_word2(3, i64::from(i32::MIN), i64::from(i32::MAX), -1, -(1 << 13));
        match parse_word(&w).unwrap() {
            InavWord::W2(f) => {
                assert_eq!(f.omega0, i32::MIN as f64 * 2f64.powi(-31) * PI);
                assert_eq!(f.i0, i32::MAX as f64 * 2f64.powi(-31) * PI);
                assert_eq!(f.omega, -1.0 * 2f64.powi(-31) * PI);
                assert_eq!(f.idot, -(1i64 << 13) as f64 * 2f64.powi(-43) * PI);
            }
            _ => panic!(),
        }
        // unknown word type ignored, not an error
        let mut w = vec![0u8; WORD_BITS];
        put_bits(&mut w, 0, 6, 21);
        assert!(parse_word(&w).is_none());
    }

    // ---------------- ephemeris assembly -----------------------------------

    fn fixture_words_1to5(d: &Value) -> (GalWord1, GalWord2, GalWord3, GalWord4, GalWord5) {
        let cases = d["cases"].as_array().unwrap();
        let get = |n: &str| {
            let c = cases.iter().find(|c| jstr(&c["name"]) == n).unwrap();
            parse_word(&jbits(c, "word_hex", WORD_BITS)).unwrap()
        };
        let w1 = match get("word1_clean") {
            InavWord::W1(f) => f,
            _ => panic!(),
        };
        let w2 = match get("word2_noisy") {
            InavWord::W2(f) => f,
            _ => panic!(),
        };
        let w3 = match get("word3_inverted_polarity") {
            InavWord::W3(f) => f,
            _ => panic!(),
        };
        let w4 = match get("word4_noisy") {
            InavWord::W4(f) => f,
            _ => panic!(),
        };
        let w5 = match get("word5_clean") {
            InavWord::W5(f) => f,
            _ => panic!(),
        };
        (w1, w2, w3, w4, w5)
    }

    #[test]
    fn assemble_ephemeris_from_pinned_words() {
        let d = fx(FX_PAGES);
        let (w1, w2, w3, w4, w5) = fixture_words_1to5(&d);
        let g = assemble_ephemeris(&w1, &w2, &w3, &w4, Some(&w5), None).unwrap();
        assert_eq!(g.iodnav, w1.iodnav);
        assert_eq!(g.eph.sys, 2);
        assert_eq!(g.eph.prn, w4.svid);
        assert_eq!(g.eph.toe, w1.toe);
        assert_eq!(g.eph.toc, w4.toc);
        assert_eq!(g.eph.tgd, w5.bgd_e1e5b);
        assert_eq!(g.eph.health, Some(0));
        // week: GST WN (word 5) + 1024 = the GPS-continuous week the pinned
        // RINEX record carries (2434 for the E02 source record)
        let r = fx(FX_RINEX);
        let rec_week = jf(&r["records"][0]["parsed"]["week"]);
        assert_eq!(g.eph.week, rec_week);
        // the words were built by inverse-scaling the same RINEX record:
        // assembled fields must match it to quantization (coarsest LSBs:
        // crc/crs 2^-5 m, af0 2^-34 s)
        let p = &r["records"][0]["parsed"];
        close(g.eph.sqrt_a, jf(&p["sqrt_a"]), 2f64.powi(-19));
        close(g.eph.m0, jf(&p["m0"]), 2f64.powi(-31) * PI);
        close(g.eph.crs, jf(&p["crs"]), 2f64.powi(-5));
        close(g.eph.af0, jf(&p["af0"]), 2f64.powi(-34));
        close(g.eph.tgd, jf(&p["bgd_e1e5b"]), 2f64.powi(-32));
    }

    #[test]
    fn assemble_rejects_mixed_iodnav_and_flags_health() {
        let d = fx(FX_PAGES);
        let (w1, mut w2, w3, w4, mut w5) = fixture_words_1to5(&d);
        w2.iodnav = w1.iodnav.wrapping_add(1);
        assert!(assemble_ephemeris(&w1, &w2, &w3, &w4, Some(&w5), None).is_none());
        w2.iodnav = w1.iodnav;
        // two-sided health law: HS<<1 | DVS composite, nonzero rejected by
        // the existing BrdcEph belts
        w5.e1b_hs = 2; // Extended Operations Mode
        w5.e1b_dvs = 1; // working without guarantee
        let g = assemble_ephemeris(&w1, &w2, &w3, &w4, Some(&w5), None).unwrap();
        assert_eq!(g.eph.health, Some((2 << 1) | 1));
        // no word 5 yet: healthy-by-default composite but no BGD/week
        let g = assemble_ephemeris(&w1, &w2, &w3, &w4, None, None).unwrap();
        assert_eq!(g.eph.health, Some(0));
        assert_eq!(g.eph.tgd, 0.0);
        assert_eq!(g.eph.week, 0.0);
    }

    // ---------------- Kepler / clock / BGD vs pinned RINEX evals ------------

    fn eph_from_json(p: &Value) -> BrdcEph {
        BrdcEph {
            sys: 2,
            prn: ju(&p["prn"]) as u8,
            toe: jf(&p["toe"]),
            toc: jf(&p["toc"]),
            week: jf(&p["week"]),
            sqrt_a: jf(&p["sqrt_a"]),
            e: jf(&p["e"]),
            m0: jf(&p["m0"]),
            delta_n: jf(&p["delta_n"]),
            omega0: jf(&p["omega0"]),
            omega: jf(&p["omega"]),
            i0: jf(&p["i0"]),
            idot: jf(&p["idot"]),
            omega_dot: jf(&p["omega_dot"]),
            cuc: jf(&p["cuc"]),
            cus: jf(&p["cus"]),
            crc: jf(&p["crc"]),
            crs: jf(&p["crs"]),
            cic: jf(&p["cic"]),
            cis: jf(&p["cis"]),
            af0: jf(&p["af0"]),
            af1: jf(&p["af1"]),
            af2: jf(&p["af2"]),
            tgd: jf(&p["bgd_e1e5b"]),
            ..Default::default()
        }
    }

    #[test]
    fn rinex_eval_positions_and_clocks() {
        // tolerances: identical algorithm+op order to the python oracle;
        // the slack covers libm last-ulp sin/cos differences only.
        // 1e-6 m on ~2.9e7 m positions ~ 4e-14 relative (far under 1e-9);
        // 1e-15 s clock ~ 0.3 mm equivalent.
        let d = fx(FX_RINEX);
        let c = &d["constants"];
        assert_eq!(jf(&c["mu"]), MU_GAL);
        assert_eq!(jf(&c["omega_e"]), OMEGA_E_GAL);
        assert_eq!(jf(&c["f_rel"]), F_REL_GAL);
        assert_eq!(jf(&c["c"]), C_LIGHT);
        for rec in d["records"].as_array().unwrap() {
            let eph = eph_from_json(&rec["parsed"]);
            let mut eph_pair = eph.clone();
            eph_pair.tgd = 0.0; // raw (E1,E5b) clock
            let rx = rec["rx_ecef_m"].as_array().unwrap();
            let rx = [jf(&rx[0]), jf(&rx[1]), jf(&rx[2])];
            // data-sources gate holds on every pinned record
            assert!(data_sources_is_inav(ju(&rec["parsed"]["data_sources"]) as u32));
            assert_eq!(ju(&rec["parsed"]["health"]), 0);
            for ev in rec["evaluations"].as_array().unwrap() {
                let t = jf(&ev["t_sow"]);
                let pos = sat_pos_ecef_gal(&eph, t);
                let exp = ev["pos_ecef_m"].as_array().unwrap();
                for k in 0..3 {
                    close(pos[k], jf(&exp[k]), 1e-6);
                }
                close(sat_clock_gal(&eph_pair, t), jf(&ev["clock_e1e5b_s"]), 1e-15);
                close(sat_clock_gal(&eph, t), jf(&ev["clock_e1_s"]), 1e-15);
                let (s, dts, rng) = sat_at_txtime_gal(&eph, t, rx);
                let sexp = ev["txtime_sat_ecef_m"].as_array().unwrap();
                for k in 0..3 {
                    close(s[k], jf(&sexp[k]), 1e-6);
                }
                close(dts, jf(&ev["txtime_clock_s"]), 1e-15);
                close(rng, jf(&ev["txtime_range_m"]), 1e-6);
            }
        }
    }

    #[test]
    fn bgd_identity_pinned() {
        // clock_e1 = clock_e1e5b - BGD(E1,E5b) (ICD Eq. 17, f1=E1) — exact
        // f64 identity by construction in both implementations.
        let d = fx(FX_GGTO);
        let b = &d["bgd_case"];
        assert_eq!(
            jf(&b["clock_e1_s"]),
            jf(&b["clock_e1e5b_s"]) - jf(&b["bgd_e1e5b_s"])
        );
    }

    // ---------------- GGTO ---------------------------------------------------

    #[test]
    fn ggto_pinned_cases() {
        let d = fx(FX_GGTO);
        for case in d["ggto_cases"].as_array().unwrap() {
            let name = jstr(&case["name"]);
            let raw = &case["raw"];
            let tow = jf(&case["tow"]);
            let wn = ju(&case["wn"]) as u32;
            if case["dt_systems_s"].is_null() {
                // all-ones sentinel: parse_word must refuse; apply_ggto is
                // the identity (fail-closed, spec 5.4 Option B)
                let w = make_word10(
                    7,
                    ji(&raw["a0g"]),
                    ji(&raw["a1g"]),
                    ju(&raw["t0g"]) as u32,
                    ju(&raw["wn0g"]) as u32,
                    &AlmTail10::default(),
                );
                match parse_word(&w).unwrap() {
                    InavWord::W10(f) => assert!(f.ggto.is_none(), "{name}"),
                    _ => panic!(),
                }
                assert_eq!(apply_ggto(tow, None, tow, wn), jf(&case["t_tx_gpst"]));
                continue;
            }
            let g = Ggto {
                a0g: ji(&raw["a0g"]) as f64 * 2f64.powi(-35),
                a1g: ji(&raw["a1g"]) as f64 * 2f64.powi(-51),
                t0g: ju(&raw["t0g"]) as f64 * 3600.0,
                wn0g: ju(&raw["wn0g"]) as u16,
            };
            let dt = ggto_offset(&g, tow, wn);
            // pure IEEE arithmetic in the same order: exact agreement
            assert_eq!(dt, jf(&case["dt_systems_s"]), "{name}");
            assert_eq!(apply_ggto(tow, Some(&g), tow, wn), jf(&case["t_tx_gpst"]), "{name}");
        }
    }

    #[test]
    fn ggto_rollover_law() {
        // |dW| <= 31 mod-64 fold, native pins on top of the fixture cases
        let g = Ggto { a0g: 0.0, a1g: 1.0, t0g: 0.0, wn0g: 63 };
        // wn=0: (0-63) mod 64 = 1 -> +1 week
        assert_eq!(ggto_offset(&g, 0.0, 0), WEEK_S);
        let g = Ggto { a0g: 0.0, a1g: 1.0, t0g: 0.0, wn0g: 1 };
        // wn=63: (63-1) mod 64 = 62 -> -2 weeks
        assert_eq!(ggto_offset(&g, 0.0, 63), -2.0 * WEEK_S);
        let g = Ggto { a0g: 0.0, a1g: 1.0, t0g: 0.0, wn0g: 10 };
        assert_eq!(ggto_offset(&g, 100.0, 10), 100.0);
    }

    // ---------------- misc gates --------------------------------------------

    #[test]
    fn data_sources_gate() {
        // verified on live brdc_latest.rnx: F/NAV 258 rejected; I/NAV
        // 513/516/517 accepted
        assert!(!data_sources_is_inav(258));
        assert!(data_sources_is_inav(513));
        assert!(data_sources_is_inav(516));
        assert!(data_sources_is_inav(517));
        assert!(!data_sources_is_inav(DS_INAV_E1B)); // clock-pair bit required
        assert!(!data_sources_is_inav(DS_CLOCK_E5B_E1)); // signal bit required
    }

    #[test]
    fn constants_differ_from_gps() {
        // pin the classic mix-up hazards (spec Section 1 reuse table)
        assert_ne!(MU_GAL, 3.986005e14); // GPS MU_E
        assert_eq!(MU_GAL, crate::beidou_d1::MU_BDS);
        assert_ne!(OMEGA_E_GAL, crate::beidou_d1::OMEGA_BDS);
        assert_eq!(OMEGA_E_GAL, 7.2921151467e-5);
        assert_ne!(F_REL_GAL, -4.442807633e-10); // GPS F
        assert_eq!(GAL_RESCAN_BACK, 502);
        assert_eq!(PAGE_SYMS, 500);
        assert_eq!(CRC_SPAN_BITS, 196);
    }
}
