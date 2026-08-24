//! GLONASS L1OF navigation-message decoder: Manchester (bi-binary) demod,
//! time-mark string sync, Hamming(85,77) verification/correction, string
//! parsing and ephemeris assembly from strings 1-4 (PZ-90 position/velocity).
//!
//! Primary source: GLONASS Interface Control Document, Edition 5.1 (2008),
//! Russian Institute of Space Device Engineering, Section 3.3.2.2 (message
//! generation), 4.3.3 / Fig. 4.3 (string structure), Tables 4.5/4.6 (word
//! layout), Table 4.11 (string 5 / almanac), Table 4.13 (Hamming checksums).
//! PDF mirror of the russianspacesystems.ru ICD retrieved 2026-08-23 from
//! http://gauss.gge.unb.ca/GLONASS.ICD.pdf
//!
//! Facts taken from the ICD (all verified against the text above):
//! - A string is 2 s = 200 chips of 10 ms. The first 170 chips are 85 data
//!   bits: the 50 bps data (in relative code) modulo-2 added to a 100 Hz
//!   meander (bi-binary / Manchester). The last 30 chips are the time mark,
//!   the shortened PR sequence 111110001101110101000010010110 (g(x)=1+x^3+x^5),
//!   transmitted WITHOUT the meander.
//! - String bit numbers increase from right to left: bit 85 (idle chip, always
//!   "0") is transmitted FIRST, bit 1 (check bit beta1) LAST, then the time
//!   mark. Words are registered MSB ahead, i.e. a word occupying bits lo..=hi
//!   has its MSB at bit hi. Signed words are SIGN-MAGNITUDE: MSB=0 is "+",
//!   MSB=1 is "-" (ICD 4.4, remark 2 to Table 4.5).
//! - Check bits beta1..beta8 are string bit positions 1..8; data bits are
//!   positions 9..85 (Table 4.13).
//! - The idle chip (bit 85) supplements the shortened 30-chip time mark to the
//!   complete 31-chip sequence, keeping the relative code continuous across
//!   the string boundary, so the first relative data chip of a string is
//!   decoded against the last chip of the previous time mark.
//!
//! Conservative choices (marked, not guessed):
//! - ICD Table 4.13 rule (b) gives a closed-form error position
//!   icor = C7..C1 + 8 - K. We instead implement single-error correction by
//!   exhaustive search (flip each of the 85 positions, accept iff ALL
//!   checksums then vanish), which is exactly equivalent for a (85,77)
//!   SEC-DED code and immune to any OCR/numbering ambiguity in that formula.
//! - One deliberate deviation: the ICD erases a string when C1..C7=0 but
//!   Csum=1 (a lone error in the overall parity bit beta8); our exhaustive
//!   search corrects that case too. It only ever triggers when a single-bit
//!   flip makes every checksum zero, so it cannot invent valid strings.
//! - Strings 6..15 carry almanac words (Table 4.11); they are detected and
//!   reported by string number but their fields are not decoded here.

use num_complex::Complex;
use std::f64::consts::PI;

use crate::glonass::{glonass_code, CHIP_RATE, CODE_LEN};

/// The 30-chip time mark, MSB first as transmitted (ICD 3.3.2.2).
pub const TIME_MARK: [u8; 30] = [
    1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0,
];
/// Chips per string (2 s at 100 chips/s).
pub const STRING_CHIPS: usize = 200;
/// Data bits per string (first 1.7 s).
pub const STRING_BITS: usize = 85;

// ---- Hamming (85,77) verification, ICD Table 4.13 --------------------------

fn expand(ranges: &[(usize, usize)], extra: &[usize]) -> Vec<usize> {
    let mut v: Vec<usize> = ranges.iter().flat_map(|&(a, b)| a..=b).collect();
    v.extend_from_slice(extra);
    v
}

/// Checksum bit sets, in STRING bit positions (1-based, ICD numbering).
/// C1/C2 are irregular and listed verbatim from Table 4.13; C3..C7 are ranges.
fn checksum_sets() -> [Vec<usize>; 7] {
    [
        vec![
            9, 10, 12, 13, 15, 17, 19, 20, 22, 24, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45,
            47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84,
        ],
        vec![
            9, 11, 12, 14, 15, 18, 19, 21, 22, 25, 26, 29, 30, 33, 34, 36, 37, 40, 41, 44, 45,
            48, 49, 52, 53, 56, 57, 60, 61, 64, 65, 67, 68, 71, 72, 75, 76, 79, 80, 83, 84,
        ],
        expand(
            &[
                (10, 12),
                (16, 19),
                (23, 26),
                (31, 34),
                (38, 41),
                (46, 49),
                (54, 57),
                (62, 65),
                (69, 72),
                (77, 80),
            ],
            &[85],
        ),
        expand(&[(13, 19), (27, 34), (42, 49), (58, 65), (73, 80)], &[]),
        expand(&[(20, 34), (50, 65), (81, 85)], &[]),
        expand(&[(35, 65)], &[]),
        expand(&[(66, 85)], &[]),
    ]
}

/// Outcome of the ICD 4.7 data-verification algorithm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Hamming {
    /// All checksums zero.
    Ok,
    /// Single error in check bits beta1..beta7 (Csum=1, one Ci=1): data valid
    /// as-is per ICD rule (a).
    OkCheckBit,
    /// Single error corrected at string bit position (1..=85).
    Corrected(usize),
    /// Multiple errors: string erased per ICD rule (c).
    Fail,
}

/// Compute (C7..C1 packed into bits 6..0, Csum) for a string held by bit
/// position: `bits[pos-1]` = string bit `pos` (positions 1..=8 are beta1..8).
pub fn hamming_checksums(bits: &[u8; STRING_BITS]) -> (u8, u8) {
    let sets = checksum_sets();
    let mut c = 0u8;
    for (i, set) in sets.iter().enumerate() {
        let mut v = bits[i]; // beta_{i+1} is at position i+1
        for &p in set {
            v ^= bits[p - 1];
        }
        c |= (v & 1) << i;
    }
    let csum = bits.iter().fold(0u8, |a, &b| a ^ b);
    (c, csum)
}

/// Verify a string and, if possible, correct a single-bit error in place
/// (ICD 4.7 rules a/b/c; correction by exhaustive search, see module docs).
pub fn hamming_verify(bits: &mut [u8; STRING_BITS]) -> Hamming {
    let (c, csum) = hamming_checksums(bits);
    if c == 0 && csum == 0 {
        return Hamming::Ok;
    }
    if c.count_ones() == 1 && csum == 1 {
        return Hamming::OkCheckBit;
    }
    for pos in 1..=STRING_BITS {
        bits[pos - 1] ^= 1;
        let (c2, s2) = hamming_checksums(bits);
        if c2 == 0 && s2 == 0 {
            return Hamming::Corrected(pos);
        }
        bits[pos - 1] ^= 1;
    }
    Hamming::Fail
}

// ---- field extraction ------------------------------------------------------

/// Unsigned word from string bit positions lo..=hi (MSB at hi, ICD numbering).
fn bu(b: &[u8; STRING_BITS], hi: usize, lo: usize) -> u64 {
    let mut v = 0u64;
    for p in lo..=hi {
        v |= (b[p - 1] as u64) << (p - lo);
    }
    v
}

/// Sign-magnitude word from positions lo..=hi (sign = MSB at hi), scaled.
fn sm(b: &[u8; STRING_BITS], hi: usize, lo: usize, lsb: f64) -> f64 {
    let n = hi - lo + 1;
    let raw = bu(b, hi, lo);
    let mag = (raw & ((1u64 << (n - 1)) - 1)) as f64;
    let s = if (raw >> (n - 1)) & 1 == 1 { -1.0 } else { 1.0 };
    s * mag * lsb
}

/// Decoded fields of one string (word layout: ICD Table 4.6 and 4.11).
#[derive(Debug, Clone, Copy)]
pub enum StringData {
    /// String 1: P1, tk, x(tb), vx(tb), ax(tb).
    S1 { p1: u8, tk_s: f64, x_m: f64, vx_ms: f64, ax_ms2: f64 },
    /// String 2: Bn, P2, tb, y(tb), vy(tb), ay(tb).
    S2 { bn: u8, p2: u8, tb_min: u16, y_m: f64, vy_ms: f64, ay_ms2: f64 },
    /// String 3: P3, gamma_n, P, ln, z(tb), vz(tb), az(tb).
    S3 { p3: u8, gamma_n: f64, p: u8, ln: u8, z_m: f64, vz_ms: f64, az_ms2: f64 },
    /// String 4: tau_n, delta_tau_n, En, P4, FT, NT, slot n, sat type M.
    S4 {
        tau_n_s: f64,
        dtau_n_s: f64,
        en: u8,
        p4: u8,
        ft: u8,
        nt: u16,
        slot: u8,
        m_type: u8,
    },
    /// String 5: NA, tau_c, N4, tau_GPS, ln (rest is almanac/reserved).
    S5 { na: u16, tau_c_s: f64, n4: u8, tau_gps_day: f64, ln: u8 },
    /// Strings 6..15: almanac, not decoded here (Table 4.11).
    Almanac,
}

/// A Hamming-valid decoded string. `bits[pos-1]` = string bit position `pos`
/// (positions 1..=8 are the check bits beta1..beta8).
#[derive(Debug, Clone)]
pub struct GloString {
    /// Word m: string number within the frame (1..=15).
    pub str_num: u8,
    pub bits: [u8; STRING_BITS],
    pub hamming: Hamming,
    pub data: StringData,
}

/// Parse the fields of a Hamming-valid string. Bits are by position, exactly
/// as produced by the demodulator (bit 85 first on the air).
pub fn parse_string(mut bits: [u8; STRING_BITS]) -> Option<GloString> {
    let hamming = hamming_verify(&mut bits);
    if hamming == Hamming::Fail {
        return None;
    }
    let b = &bits;
    let str_num = bu(b, 84, 81) as u8;
    let data = match str_num {
        1 => StringData::S1 {
            p1: bu(b, 78, 77) as u8,
            tk_s: bu(b, 76, 72) as f64 * 3600.0
                + bu(b, 71, 66) as f64 * 60.0
                + bu(b, 65, 65) as f64 * 30.0,
            x_m: sm(b, 35, 9, 2f64.powi(-11)) * 1e3,
            vx_ms: sm(b, 64, 41, 2f64.powi(-20)) * 1e3,
            ax_ms2: sm(b, 40, 36, 2f64.powi(-30)) * 1e3,
        },
        2 => StringData::S2 {
            bn: bu(b, 80, 78) as u8,
            p2: bu(b, 77, 77) as u8,
            tb_min: (bu(b, 76, 70) * 15) as u16,
            y_m: sm(b, 35, 9, 2f64.powi(-11)) * 1e3,
            vy_ms: sm(b, 64, 41, 2f64.powi(-20)) * 1e3,
            ay_ms2: sm(b, 40, 36, 2f64.powi(-30)) * 1e3,
        },
        3 => StringData::S3 {
            p3: bu(b, 80, 80) as u8,
            gamma_n: sm(b, 79, 69, 2f64.powi(-40)),
            p: bu(b, 67, 66) as u8,
            ln: bu(b, 65, 65) as u8,
            z_m: sm(b, 35, 9, 2f64.powi(-11)) * 1e3,
            vz_ms: sm(b, 64, 41, 2f64.powi(-20)) * 1e3,
            az_ms2: sm(b, 40, 36, 2f64.powi(-30)) * 1e3,
        },
        4 => StringData::S4 {
            tau_n_s: sm(b, 80, 59, 2f64.powi(-30)),
            dtau_n_s: sm(b, 58, 54, 2f64.powi(-30)),
            en: bu(b, 53, 49) as u8,
            p4: bu(b, 34, 34) as u8,
            ft: bu(b, 33, 30) as u8,
            nt: bu(b, 26, 16) as u16,
            slot: bu(b, 15, 11) as u8,
            m_type: bu(b, 10, 9) as u8,
        },
        5 => StringData::S5 {
            na: bu(b, 80, 70) as u16,
            tau_c_s: sm(b, 69, 38, 2f64.powi(-31)),
            n4: bu(b, 36, 32) as u8,
            tau_gps_day: sm(b, 31, 10, 2f64.powi(-30)),
            ln: bu(b, 9, 9) as u8,
        },
        6..=15 => StringData::Almanac,
        _ => return None, // m = 0 or >15 cannot occur in a valid frame
    };
    Some(GloString { str_num, bits, hamming, data })
}

// ---- ephemeris assembly (strings 1-4) --------------------------------------

/// Broadcast ephemeris of the tracked satellite, SI units, PZ-90.02 frame.
#[derive(Debug, Clone, Copy)]
pub struct GloEphemeris {
    pub slot: u8,
    /// Calendar day within the four-year interval (NT).
    pub nt: u16,
    /// Index of the ephemeris time interval, minutes into the day,
    /// UTC(SU)+03:00 scale (tb * 15).
    pub tb_min: u16,
    /// Frame time within the day, seconds (satellite time scale).
    pub tk_s: f64,
    pub pos_m: [f64; 3],
    pub vel_ms: [f64; 3],
    /// Accelerations due to the Sun and Moon (ICD 4.4).
    pub acc_ms2: [f64; 3],
    pub gamma_n: f64,
    pub tau_n_s: f64,
    pub en: u8,
    pub ft: u8,
}

impl GloEphemeris {
    pub fn radius_m(&self) -> f64 {
        self.pos_m.iter().map(|v| v * v).sum::<f64>().sqrt()
    }
    pub fn speed_ms(&self) -> f64 {
        self.vel_ms.iter().map(|v| v * v).sum::<f64>().sqrt()
    }
}

/// Collects strings 1-4 of one frame and assembles the ephemeris.
#[derive(Default)]
pub struct EphCollector {
    s1: Option<GloString>,
    s2: Option<GloString>,
    s3: Option<GloString>,
    s4: Option<GloString>,
}

impl EphCollector {
    pub fn push(&mut self, s: &GloString) {
        match s.str_num {
            1 => self.s1 = Some(s.clone()),
            2 => self.s2 = Some(s.clone()),
            3 => self.s3 = Some(s.clone()),
            4 => self.s4 = Some(s.clone()),
            _ => {}
        }
    }

    /// Assemble strings 1-4 into an ephemeris. Returns None unless all four
    /// are present AND the orbit is a sane GLONASS orbit (radius ~25.5 Mm,
    /// speed ~3.9 km/s) — the same conservative role the IODE/eccentricity
    /// checks play for GPS LNAV.
    pub fn ephemeris(&self) -> Option<GloEphemeris> {
        let (s1, s2, s3, s4) = (
            self.s1.as_ref()?,
            self.s2.as_ref()?,
            self.s3.as_ref()?,
            self.s4.as_ref()?,
        );
        let (StringData::S1 { tk_s, x_m, vx_ms, ax_ms2, .. },
             StringData::S2 { tb_min, y_m, vy_ms, ay_ms2, .. },
             StringData::S3 { gamma_n, ln, z_m, vz_ms, az_ms2, .. },
             StringData::S4 { tau_n_s, en, ft, nt, slot, .. }) =
            (s1.data, s2.data, s3.data, s4.data)
        else {
            return None;
        };
        let e = GloEphemeris {
            slot,
            nt,
            tb_min,
            tk_s,
            pos_m: [x_m, y_m, z_m],
            vel_ms: [vx_ms, vy_ms, vz_ms],
            acc_ms2: [ax_ms2, ay_ms2, az_ms2],
            gamma_n,
            tau_n_s,
            en,
            ft,
        };
        let r = e.radius_m();
        let v = e.speed_ms();
        // GLONASS nominal: h=19100 km -> r ~= 25,508 km, v ~= 3.95 km/s.
        if !(23.0e6..28.0e6).contains(&r) || !(3.0e3..5.0e3).contains(&v) || ln != 0 {
            return None;
        }
        Some(e)
    }
}

// ---- Manchester demod + time-mark sync -------------------------------------

/// Result of the time-mark search over a 10 ms soft-chip stream.
#[derive(Debug, Clone, Copy)]
pub struct ChipSync {
    /// Index of the first chip of the first full string in `chips`
    /// (0..STRING_CHIPS).
    pub offset: usize,
    /// Mean |correlation| against the time mark, 0..1 (1 = all 30 chips match).
    pub score: f64,
    /// +1 if the mark matches as transmitted, -1 if inverted (carrier phase).
    pub polarity: i8,
}

/// Search `chips` (soft 10 ms chips) for the time mark, allowing a few chip
/// errors (soft correlation). Needs at least 2 string periods. Scoring is
/// coherent within 10 s segments and incoherent across them, so a carrier
/// cycle slip that flips the sign of the mark mid-capture does not cancel the
/// accumulation.
pub fn find_time_mark(chips: &[f64]) -> Option<ChipSync> {
    const SEG_BLOCKS: usize = 5; // 5 strings = 10 s
    let tm: Vec<f64> = TIME_MARK.iter().map(|&b| 2.0 * b as f64 - 1.0).collect();
    let mut best: Option<ChipSync> = None;
    for off in 0..STRING_CHIPS {
        let nblocks = (chips.len().saturating_sub(off + STRING_CHIPS)) / STRING_CHIPS + 1;
        if nblocks < 2 {
            continue;
        }
        let mut acc = 0.0;
        for seg in 0..nblocks.div_ceil(SEG_BLOCKS) {
            let lo = seg * SEG_BLOCKS;
            let hi = (lo + SEG_BLOCKS).min(nblocks);
            let mut s = 0.0;
            for k in lo..hi {
                let base = off + STRING_CHIPS * k + 170;
                s += (0..30).map(|j| chips[base + j] * tm[j]).sum::<f64>();
            }
            acc += s.abs();
        }
        let score = acc / (30.0 * nblocks as f64);
        // polarity from the first segment (global sign; data decode does not
        // depend on it, the relative code cancels it — reported for diagnosis)
        let base = off + 170;
        let c0: f64 = (0..30).map(|j| chips[base + j] * tm[j]).sum();
        let better = best.map_or(true, |b| score > b.score);
        if better {
            best = Some(ChipSync {
                offset: off,
                score,
                polarity: if c0 >= 0.0 { 1 } else { -1 },
            });
        }
    }
    // demand a solid match: at most ~4 of 30 chips wrong on average
    best.filter(|b| b.score > 0.85)
}

/// Locate the 10 ms chip phase and string alignment in a 1 ms prompt-
/// correlator stream. Tries all 10 chip phases, returns the best time-mark
/// sync. Returns the phase (0..10 ms), the sync, and the soft chips.
pub fn sync_from_prompt(prompt_i: &[f64]) -> Option<(usize, ChipSync, Vec<f64>)> {
    let mut best: Option<(usize, ChipSync, Vec<f64>)> = None;
    for phase in 0..10 {
        let n = (prompt_i.len().saturating_sub(phase)) / 10;
        if n < 2 * STRING_CHIPS {
            continue;
        }
        let chips: Vec<f64> = (0..n)
            .map(|i| prompt_i[phase + 10 * i..phase + 10 * i + 10].iter().sum())
            .collect();
        if let Some(s) = find_time_mark(&chips) {
            let better = best.as_ref().map_or(true, |(_, bs, _)| s.score > bs.score);
            if better {
                best = Some((phase, s, chips));
            }
        }
    }
    best
}

/// Manchester-demod one string starting at `chips[base..base+200]` and decode
/// the relative code. `prev_last_chip` is the soft value of the last time-mark
/// chip of the previous string (the relative-code reference for the idle bit
/// b85); if None, b85 is set to its ICD-guaranteed value 0.
pub fn demod_string(chips: &[f64], base: usize, prev_last_chip: Option<f64>) -> Option<[u8; STRING_BITS]> {
    if base + STRING_CHIPS > chips.len() {
        return None;
    }
    // relative-coded bits, ±1: Manchester pairs (r, -r), combined softly.
    let mut rel = [0f64; STRING_BITS];
    for i in 0..STRING_BITS {
        rel[i] = chips[base + 2 * i] - chips[base + 2 * i + 1];
    }
    let mut bits = [0u8; STRING_BITS];
    // transmitted order: bit 85 first ... bit 1 last (ICD Fig. 4.3)
    let mut prev = prev_last_chip;
    for i in 0..STRING_BITS {
        let pos = STRING_BITS - i; // string bit position of this chip pair
        bits[pos - 1] = match prev {
            Some(p) => {
                if (rel[i] >= 0.0) == (p >= 0.0) { 0 } else { 1 }
            }
            None => 0, // b85 idle chip, defined "0" by the ICD
        };
        prev = Some(rel[i]);
    }
    Some(bits)
}

/// Demodulate and parse every full string in a chip stream, given a sync.
/// Returns (block index, parsed string) pairs; strings failing Hamming are
/// skipped.
pub fn decode_strings(chips: &[f64], sync: &ChipSync) -> Vec<(usize, GloString)> {
    let mut out = Vec::new();
    let mut k = 0usize;
    let mut prev_last: Option<f64> = None;
    while sync.offset + STRING_CHIPS * (k + 1) <= chips.len() {
        let base = sync.offset + STRING_CHIPS * k;
        if let Some(bits) = demod_string(chips, base, prev_last) {
            if let Some(s) = parse_string(bits) {
                out.push((k, s));
            }
        }
        // soft value of this string's last time-mark chip (pattern chip 29 = 0
        // -> -1 as transmitted), derotated by the measured global polarity so
        // it is in the same sign frame as the demodulated chips.
        let tm_end = base + 170 + 29;
        prev_last = Some(chips[tm_end]);
        k += 1;
    }
    out
}

// ---- block-wise demod path (alternative to the closed-loop tracker) --------
//
// Motivation: indoors the L1OF signals sit near C/N0 ~ 30 dB-Hz and a 10 Hz
// Costas loop started from a 250 Hz acquisition grid may never converge (the
// discriminator is too noisy at 1 ms). This path needs no pull-in: wipe the
// code at the acquired code phase, integrate 10 ms chunks, then estimate the
// residual carrier from the SQUARED chunk sequence — squaring wipes the
// bi-binary data (chip^2 has the same phase for both chip signs) and leaves a
// spectral line at twice the residual frequency, which sliding-window FFTs
// track as it drifts. Validated live on 2026-08-23 where the loop-based path
// above failed to lock.

/// Wipe the 511-chip code and coarse carrier, integrate into `chunk_ms`
/// chunks. `dopp_hz` must be within ~20 Hz of the true Doppler (a fine
/// acquisition grid), otherwise the squared spectrum aliases; the code phase
/// advances with the code Doppler implied by `dopp_hz` (plus `fres`, the
/// residual-Hz-per-chunk trajectory from a previous [`squared_phase_track`]
/// pass, when iterating). Code chips are linearly interpolated at the
/// fractional sample position.
pub fn codewipe_chunks(
    sig: &[Complex<f32>],
    fs: f64,
    f_carrier: f64,
    dopp_hz: f64,
    code_phase0_chips: f64,
    chunk_ms: usize,
    fres: Option<&[f64]>,
) -> Vec<Complex<f64>> {
    let code = glonass_code();
    let clen = CODE_LEN as f64;
    let ns_c = (fs * chunk_ms as f64 / 1000.0).round() as usize;
    let nchunks = sig.len() / ns_c;
    let mut out = Vec::with_capacity(nchunks);
    let mut cp = code_phase0_chips;
    let mut ph = 0.0f64; // carrier phase, accumulated continuously across chunks
    for c in 0..nchunks {
        let base = c * ns_c;
        let f_hz = dopp_hz + fres.map_or(0.0, |f| f[c.min(f.len() - 1)]);
        let code_rate = CHIP_RATE * (1.0 + f_hz / f_carrier);
        let dphi = -2.0 * PI * f_hz / fs;
        let mut acc = Complex::new(0.0f64, 0.0);
        for j in 0..ns_c {
            let s = sig[base + j];
            let (sp, cpp) = ph.sin_cos();
            let d_re = s.re as f64 * cpp - s.im as f64 * sp;
            let d_im = s.re as f64 * sp + s.im as f64 * cpp;
            let cf = (cp + code_rate * (j as f64 / fs)).rem_euclid(clen);
            let i0 = cf as usize % CODE_LEN;
            let fr = cf.fract();
            let g = code[i0] as f64 * (1.0 - fr) + code[(i0 + 1) % CODE_LEN] as f64 * fr;
            acc.re += d_re * g;
            acc.im += d_im * g;
            ph += dphi;
        }
        out.push(acc);
        cp = (cp + code_rate * chunk_ms as f64 / 1000.0).rem_euclid(clen);
    }
    out
}

/// Per-chunk residual carrier estimate from [`squared_phase_track`].
pub struct PhaseTrack {
    /// per-chunk residual phase (rad), integrated from the windowed estimates
    pub phase: Vec<f64>,
    /// per-chunk residual frequency (Hz), interpolated between window centers
    pub freq: Vec<f64>,
}

/// Residual-carrier phase of code-wiped chunks via the squared spectrum.
/// Windows of 2 s (step 0.5 s) of squared chunks are FFT'd; the line at
/// 2*f_res is parabola-interpolated, halved, and integrated into a per-chunk
/// phase. `chunk_rate_hz` = 1000/chunk_ms. Returns None if the capture is too
/// short for the windowed estimate.
pub fn squared_phase_track(chunks: &[Complex<f64>], chunk_rate_hz: f64) -> Option<PhaseTrack> {
    use rustfft::FftPlanner;
    let win = (2.0 * chunk_rate_hz) as usize; // 2 s windows
    let step = win / 4;
    if chunks.len() < win {
        return None;
    }
    let nfft = 8192usize;
    let mut planner = FftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(nfft);
    let half_bw = 45.0f64; // search |2*f_res| < 45 Hz (|f_res| < 22.5 Hz)
    let mut centers: Vec<(f64, f64)> = Vec::new(); // (t_seconds, f_res)
    let mut w0 = 0usize;
    while w0 + win <= chunks.len() {
        // note: no mean removal and bin 0 is searched — after a first
        // derotation pass the true residual can be ~0 Hz, and the squared
        // line then legitimately sits at DC.
        let mut buf = vec![Complex::new(0.0f64, 0.0); nfft];
        for (i, z) in chunks[w0..w0 + win].iter().enumerate() {
            buf[i] = z * z;
        }
        fft.process(&mut buf);
        let bin_hz = chunk_rate_hz / nfft as f64;
        let bmax = (half_bw / bin_hz) as usize;
        let mut best = (0usize, 0.0f64);
        for bi in (0..=bmax).chain((nfft - bmax)..nfft) {
            let m = buf[bi].norm_sqr();
            if m > best.1 {
                best = (bi, m);
            }
        }
        // parabolic interpolation around the peak bin
        let b = best.0;
        let (ym, y0, yp) = (
            buf[(b + nfft - 1) % nfft].norm_sqr().ln(),
            buf[b].norm_sqr().ln(),
            buf[(b + 1) % nfft].norm_sqr().ln(),
        );
        let denom = ym - 2.0 * y0 + yp; // negative at a peak
        let delta = if !denom.is_finite() || denom.abs() < 1e-30 {
            0.0
        } else {
            (0.5 * (ym - yp) / denom).clamp(-0.5, 0.5)
        };
        let bf = b as f64 + delta;
        let f2 = if bf <= nfft as f64 / 2.0 {
            bf * bin_hz
        } else {
            (bf - nfft as f64) * bin_hz
        };
        let t = (w0 + win / 2) as f64 / chunk_rate_hz;
        centers.push((t, f2 / 2.0));
        w0 += step;
    }
    if centers.len() < 2 {
        return None;
    }
    // integrate the f_res(t) estimates into a per-chunk phase, linearly
    // interpolating between window centers and holding the edges.
    let f_at = |t: f64| -> f64 {
        if t <= centers[0].0 {
            return centers[0].1;
        }
        for w in centers.windows(2) {
            if t >= w[0].0 && t <= w[1].0 {
                let a = (t - w[0].0) / (w[1].0 - w[0].0);
                return w[0].1 + a * (w[1].1 - w[0].1);
            }
        }
        centers[centers.len() - 1].1
    };
    let mut phase = Vec::with_capacity(chunks.len());
    let mut freq = Vec::with_capacity(chunks.len());
    let mut phi = 0.0f64;
    for (j, _) in chunks.iter().enumerate() {
        phase.push(phi);
        let t = j as f64 / chunk_rate_hz;
        let f = f_at(t);
        freq.push(f);
        phi += 2.0 * PI * f / chunk_rate_hz;
    }
    Some(PhaseTrack { phase, freq })
}

/// Stage-2 carrier refinement: once the time mark gives string alignment, the
/// 30 known time-mark chips of every string act as pilot symbols. Their
/// phasors (correlation against the known pattern) sample the residual
/// carrier phase once per 2 s; unwrapping and linearly interpolating them
/// yields a per-chunk phase correction with no squaring loss and no data
/// ambiguity. `chunks` must already be derotated by the stage-1 (squared
/// spectrum) phase; `offset` is the `ChipSync::offset` found on the stage-1
/// chips. Returns the per-chunk correction phase (radians).
pub fn pilot_phase_refine(chunks: &[Complex<f64>], offset: usize) -> Option<Vec<f64>> {
    let tm: Vec<f64> = TIME_MARK.iter().map(|&b| 2.0 * b as f64 - 1.0).collect();
    let mut phasors: Vec<(usize, f64)> = Vec::new(); // (center chip, phase)
    let mut k = 0usize;
    let mut prev_phi: Option<f64> = None;
    let mut unwrap = 0.0f64;
    while offset + STRING_CHIPS * k + STRING_CHIPS <= chunks.len() {
        let base = offset + STRING_CHIPS * k + 170;
        let p: Complex<f64> = (0..30).map(|j| chunks[base + j] * tm[j]).sum();
        if p.norm_sqr() < 1e-12 {
            return None;
        }
        let mut phi = p.arg();
        if let Some(pp) = prev_phi {
            // unwrap: fold into pp +/- pi
            while phi + unwrap - pp > PI {
                unwrap -= 2.0 * PI;
            }
            while phi + unwrap - pp < -PI {
                unwrap += 2.0 * PI;
            }
            phi += unwrap;
        }
        prev_phi = Some(phi);
        phasors.push((base + 15, phi));
        k += 1;
    }
    if phasors.len() < 2 {
        return None;
    }
    let corr_at = |idx: usize| -> f64 {
        let x = idx as f64;
        if x <= phasors[0].0 as f64 {
            return phasors[0].1;
        }
        for w in phasors.windows(2) {
            let (x0, y0) = (w[0].0 as f64, w[0].1);
            let (x1, y1) = (w[1].0 as f64, w[1].1);
            if x >= x0 && x <= x1 {
                return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
            }
        }
        phasors[phasors.len() - 1].1
    };
    Some((0..chunks.len()).map(corr_at).collect())
}

/// Soft-vote repeated strings. String content repeats every `period` blocks
/// (15 for the 30 s frame). For each relative-coded bit the product
/// d_i = r_i * r_{i-1} is invariant to both the relative-code state and the
/// global carrier polarity, so the soft products of repeated strings can be
/// summed directly; the sum is hard-limited and Hamming-verified. Strings
/// whose content legitimately changes between frames (string 1's tk field)
/// will simply fail the Hamming check here and must be handled per frame.
/// Returns (residue class = block index of first occurrence, string) pairs.
pub fn vote_strings(chips: &[f64], sync: &ChipSync, period: usize) -> Vec<(usize, GloString)> {
    let nblocks = (chips.len().saturating_sub(sync.offset)) / STRING_CHIPS;
    let mut out = Vec::new();
    for res in 0..period.min(nblocks) {
        let mut acc = [0.0f64; STRING_BITS];
        let mut cnt = 0usize;
        let mut k = res;
        while k < nblocks {
            let base = sync.offset + STRING_CHIPS * k;
            // relative-coded soft bits r_i
            let mut r = [0.0f64; STRING_BITS];
            for i in 0..STRING_BITS {
                r[i] = chips[base + 2 * i] - chips[base + 2 * i + 1];
            }
            for i in 1..STRING_BITS {
                acc[i] += r[i] * r[i - 1];
            }
            cnt += 1;
            k += period;
        }
        if cnt == 0 {
            continue;
        }
        // bit 85 (idle) is defined 0; the rest: d_i = 0 iff acc[i] >= 0
        let mut bits = [0u8; STRING_BITS];
        for i in 1..STRING_BITS {
            let pos = STRING_BITS - i;
            bits[pos - 1] = if acc[i] >= 0.0 { 0 } else { 1 };
        }
        if let Some(s) = parse_string(bits) {
            out.push((res, s));
        }
    }
    out
}

/// Convenience: full block-wise path from baseband to soft 10 ms chips plus
/// the time-mark sync. Pass 1: code-wipe at the acquired Doppler, squared-
/// spectrum carrier track. Pass 2: re-wipe with the tracked frequency (so the
/// code rate — and hence the code phase over a long capture — follows the
/// true Doppler), squared-spectrum track again on the now-small residual.
/// Stage 2: time-mark pilot phase refinement.
pub fn blockwise_chips(
    sig: &[Complex<f32>],
    fs: f64,
    f_carrier: f64,
    dopp_hz: f64,
    code_phase0_chips: f64,
) -> Option<(Vec<f64>, ChipSync)> {
    let c1 = codewipe_chunks(sig, fs, f_carrier, dopp_hz, code_phase0_chips, 10, None);
    let tr1 = squared_phase_track(&c1, 100.0)?;
    let c2 = codewipe_chunks(sig, fs, f_carrier, dopp_hz, code_phase0_chips, 10, Some(&tr1.freq));
    let tr2 = squared_phase_track(&c2, 100.0)?;
    let z1: Vec<Complex<f64>> = c2
        .iter()
        .zip(tr2.phase.iter())
        .map(|(z, &p)| z * Complex::new((-p).cos(), (-p).sin()))
        .collect();
    let chips1: Vec<f64> = z1.iter().map(|z| z.re).collect();
    let sync = find_time_mark(&chips1)?;
    if let Some(corr) = pilot_phase_refine(&z1, sync.offset) {
        let chips2: Vec<f64> = z1
            .iter()
            .zip(corr.iter())
            .map(|(z, &p)| (z * Complex::new((-p).cos(), (-p).sin())).re)
            .collect();
        // re-sync on the cleaned chips (offset should be unchanged)
        let sync2 = find_time_mark(&chips2).unwrap_or(sync);
        Some((chips2, sync2))
    } else {
        Some((chips1, sync))
    }
}

// ---- tracking: Costas PLL + DLL for the 511-chip L1OF code ------------------

/// Result of tracking one GLONASS channel.
pub struct GloTrackResult {
    pub epochs: usize,
    /// prompt I per 1 ms epoch (data-bearing once the PLL is locked)
    pub prompt_i: Vec<f64>,
    pub prompt_q: Vec<f64>,
    pub final_doppler: f64,
    /// rough C/N0 proxy in dB-Hz (1 ms prompt power / quadrature power + 30)
    pub cn0_dbhz: f32,
}

/// Track the L1OF signal at baseband: 2nd-order Costas PLL (Borre, 10 Hz)
/// plus a carrier-aided early/late DLL, 1 ms epochs (= one 511-chip period).
/// Mirrors the GPS tracker in `gps/track.rs` with the GLONASS code; `dopp0`
/// and `code_phase0_chips` come straight from `acquire_codes`.
pub fn track_l1of(
    sig: &[Complex<f32>],
    fs: f64,
    f_carrier: f64,
    dopp0: f64,
    code_phase0_chips: f64,
    epochs: usize,
) -> GloTrackResult {
    track_l1of_bw(sig, fs, f_carrier, dopp0, code_phase0_chips, epochs, 10.0)
}

/// Same as [`track_l1of`] with an explicit Costas PLL noise bandwidth (Hz).
#[allow(clippy::too_many_arguments)]
pub fn track_l1of_bw(
    sig: &[Complex<f32>],
    fs: f64,
    f_carrier: f64,
    dopp0: f64,
    code_phase0_chips: f64,
    epochs: usize,
    pll_bw_hz: f64,
) -> GloTrackResult {
    let code = glonass_code();
    let ns = (fs / 1000.0).round() as usize;
    let clen = CODE_LEN as f64;

    let mut carrier_phase = 0.0f64;
    let mut carrier_freq = dopp0;
    let mut code_phase = code_phase0_chips;
    const PDI: f64 = 0.001;
    let (pll_t1, pll_t2) = borre(pll_bw_hz, 0.7, 0.25);
    let carr_basis = dopp0;
    let mut carr_nco = 0.0f64;
    let mut old_carr_err = 0.0f64;
    let dll_k = 1.0;
    let spacing = 0.5f64;

    let n_epoch = epochs.min(sig.len() / ns);
    let mut prompt_i = Vec::with_capacity(n_epoch);
    let mut prompt_q = Vec::with_capacity(n_epoch);
    let mut prompt_pwr = 0.0f64;
    let mut noise_pwr = 0.0f64;

    for e in 0..n_epoch {
        let (mut ie, mut qe) = (0.0f64, 0.0f64);
        let (mut ip, mut qp) = (0.0f64, 0.0f64);
        let (mut il, mut ql) = (0.0f64, 0.0f64);
        let code_rate = CHIP_RATE * (1.0 + carrier_freq / f_carrier);
        let code_step = code_rate / fs;
        let dphi = 2.0 * PI * carrier_freq / fs;
        let base = e * ns;
        for k in 0..ns {
            let s = sig[base + k];
            let ph = carrier_phase + dphi * k as f64;
            let (sinp, cosp) = ph.sin_cos();
            let bi = s.re as f64 * cosp + s.im as f64 * sinp;
            let bq = -(s.re as f64) * sinp + s.im as f64 * cosp;
            let cp = code_phase + code_step * k as f64;
            let ce = chip(&code, cp - spacing, clen);
            let cpr = chip(&code, cp, clen);
            let cl = chip(&code, cp + spacing, clen);
            ie += bi * ce;
            qe += bq * ce;
            ip += bi * cpr;
            qp += bq * cpr;
            il += bi * cl;
            ql += bq * cl;
        }
        code_phase = (code_phase + code_step * ns as f64).rem_euclid(clen);
        carrier_phase = (carrier_phase + dphi * ns as f64).rem_euclid(2.0 * PI);

        let norm = (ip * ip + qp * qp).sqrt().max(1e-12);
        let carr_err = (qp * ip.signum()) / norm / (2.0 * PI);
        carr_nco += (pll_t2 / pll_t1) * (carr_err - old_carr_err) + carr_err * (PDI / pll_t1);
        old_carr_err = carr_err;
        carrier_freq = carr_basis + carr_nco;

        let ep = (ie * ie + qe * qe).sqrt();
        let lp = (il * il + ql * ql).sqrt();
        let dll_err = if ep + lp > 1e-12 { 0.5 * (ep - lp) / (ep + lp) } else { 0.0 };
        code_phase = (code_phase - dll_k * dll_err).rem_euclid(clen);

        prompt_i.push(ip);
        prompt_q.push(qp);
        prompt_pwr += ip * ip + qp * qp;
        noise_pwr += qp * qp;
    }

    let mean_p = prompt_pwr / n_epoch.max(1) as f64;
    let mean_n = (noise_pwr / n_epoch.max(1) as f64).max(1e-12);
    let cn0 = 10.0 * (mean_p / mean_n).log10() + 30.0;

    GloTrackResult {
        epochs: n_epoch,
        prompt_i,
        prompt_q,
        final_doppler: carrier_freq,
        cn0_dbhz: cn0 as f32,
    }
}

#[inline]
fn chip(code: &[f32], c: f64, clen: f64) -> f64 {
    let idx = c.rem_euclid(clen) as usize % code.len();
    code[idx] as f64
}

fn borre(bn: f64, zeta: f64, k: f64) -> (f64, f64) {
    let wn = bn * 8.0 * zeta / (4.0 * zeta * zeta + 1.0);
    let tau1 = k / (wn * wn);
    let tau2 = 2.0 * zeta / wn;
    (tau1, tau2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gps::ca_code::resample_code;

    /// xorshift for reproducible test vectors.
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
    }

    /// Compute check bits beta1..beta8 for data bits at positions 9..=85
    /// (inverse of the Table 4.13 checksums: beta_i makes C_i vanish, beta8
    /// makes Csum vanish).
    fn set_check_bits(bits: &mut [u8; STRING_BITS]) {
        let sets = checksum_sets();
        for (i, set) in sets.iter().enumerate() {
            let mut v = 0u8;
            for &p in set {
                v ^= bits[p - 1];
            }
            bits[i] = v; // beta_{i+1}
        }
        // beta8: overall parity over beta1..7 and data bits 9..85
        let mut v = 0u8;
        for p in 1..=7 {
            v ^= bits[p - 1];
        }
        for p in 9..=STRING_BITS {
            v ^= bits[p - 1];
        }
        bits[7] = v;
    }

    /// Random valid string with string number `m` and zeroed reserved-ish
    /// fields left as noise (Hamming does not care about field semantics).
    fn random_string(rng: &mut Rng, m: u8) -> [u8; STRING_BITS] {
        let mut bits = [0u8; STRING_BITS];
        for p in 9..=STRING_BITS {
            bits[p - 1] = (rng.next() & 1) as u8;
        }
        bits[84] = 0; // idle chip, ICD 4.3.3
        for (i, p) in [84usize, 83, 82, 81].iter().enumerate() {
            bits[p - 1] = (m >> (3 - i)) & 1; // word m at bits 81-84, MSB ahead
        }
        set_check_bits(&mut bits);
        bits
    }

    /// Modulate one string into 200 soft chips (±1): relative code + meander
    /// for the data part, bare time mark at the end. `prev` is the last chip
    /// of the previous time mark (relative-code reference for the idle bit).
    fn modulate_string(bits: &[u8; STRING_BITS], prev: f64) -> (Vec<f64>, f64) {
        let mut chips = vec![0.0f64; STRING_CHIPS];
        let mut state = prev;
        for i in 0..STRING_BITS {
            let pos = STRING_BITS - i; // bit 85 first
            let d = bits[pos - 1];
            let r = if d == 1 { -state } else { state }; // relative code
            chips[2 * i] = r;
            chips[2 * i + 1] = -r; // meander (bi-binary)
            state = r;
        }
        for j in 0..30 {
            chips[170 + j] = 2.0 * TIME_MARK[j] as f64 - 1.0;
        }
        let last = chips[199];
        (chips, last)
    }

    /// Set a sign-magnitude word (positions lo..=hi, sign at hi) from a scaled
    /// integer magnitude.
    fn put_sm(b: &mut [u8; STRING_BITS], hi: usize, lo: usize, lsb: f64, val: f64) {
        let n = hi - lo + 1;
        let mag = (val.abs() / lsb).round() as u64;
        assert!(mag < (1u64 << (n - 1)), "field overflow");
        let raw = (if val < 0.0 { 1u64 << (n - 1) } else { 0 }) | mag;
        for p in lo..=hi {
            b[p - 1] = ((raw >> (p - lo)) & 1) as u8;
        }
    }

    fn put_u(b: &mut [u8; STRING_BITS], hi: usize, lo: usize, val: u64) {
        for p in lo..=hi {
            b[p - 1] = ((val >> (p - lo)) & 1) as u8;
        }
    }

    #[test]
    fn hamming_accepts_valid_strings() {
        let mut rng = Rng(0x1234_5678_9abc_def0);
        for m in 1..=15u8 {
            let mut b = random_string(&mut rng, m);
            assert_eq!(hamming_verify(&mut b), Hamming::Ok, "string m={m}");
        }
    }

    #[test]
    fn hamming_corrects_every_single_error() {
        let mut rng = Rng(0xdead_beef_cafe_f00d);
        let orig = random_string(&mut rng, 1);
        for pos in 1..=STRING_BITS {
            let mut b = orig;
            b[pos - 1] ^= 1;
            let st = hamming_verify(&mut b);
            match st {
                Hamming::OkCheckBit => assert!(pos <= 7, "check-bit case at data pos {pos}"),
                Hamming::Corrected(p) => {
                    assert_eq!(p, pos);
                    assert_eq!(b, orig, "correction at pos {pos} must restore the string");
                }
                other => panic!("single error at pos {pos} not handled: {other:?}"),
            }
        }
    }

    #[test]
    fn hamming_detects_double_errors() {
        let mut rng = Rng(0x0bad_c0de_1357_9bdf);
        let orig = random_string(&mut rng, 3);
        for p1 in 1..=STRING_BITS {
            for p2 in (p1 + 1)..=STRING_BITS {
                let mut b = orig;
                b[p1 - 1] ^= 1;
                b[p2 - 1] ^= 1;
                assert_eq!(
                    hamming_verify(&mut b),
                    Hamming::Fail,
                    "double error at {p1},{p2} must be detected"
                );
            }
        }
    }

    #[test]
    fn manchester_relative_roundtrip_and_sync() {
        // three consecutive valid strings at a random chip offset, with the
        // whole stream polarity-inverted (carrier phase ambiguity): sync must
        // find the offset and polarity, and the strings must decode exactly.
        let mut rng = Rng(0xabcd_1234_5678_9abc);
        let strs: Vec<[u8; STRING_BITS]> = (1..=3).map(|m| random_string(&mut rng, m)).collect();
        let lead = 37usize; // unknown offset
        let mut chips = vec![0.11f64; lead]; // noise-ish leader
        let mut prev = -1.0; // arbitrary last chip of a previous mark
        for b in &strs {
            let (c, last) = modulate_string(b, prev);
            chips.extend_from_slice(&c);
            prev = last;
        }
        let chips: Vec<f64> = chips.iter().map(|&c| -c).collect(); // invert polarity
        let sync = find_time_mark(&chips).expect("time mark not found");
        assert_eq!(sync.offset, lead);
        assert_eq!(sync.polarity, -1);
        let decoded = decode_strings(&chips, &sync);
        assert_eq!(decoded.len(), 3);
        for ((_, s), orig) in decoded.iter().zip(strs.iter()) {
            assert_eq!(&s.bits, orig);
        }
    }

    #[test]
    fn time_mark_sync_tolerates_chip_errors() {
        let mut rng = Rng(0x1111_2222_3333_4444);
        let mut chips = Vec::new();
        let mut prev = 1.0;
        for m in 1..=3u8 {
            let (c, last) = modulate_string(&random_string(&mut rng, m), prev);
            chips.extend_from_slice(&c);
            prev = last;
        }
        // corrupt 3 chips of the second string's time mark (10% of the mark)
        for &j in &[170 + 200, 175 + 200, 190 + 200] {
            chips[j] = -chips[j];
        }
        let sync = find_time_mark(&chips).expect("sync with 3 bad mark chips");
        assert_eq!(sync.offset, 0);
        assert_eq!(decode_strings(&chips, &sync).len(), 3);
    }

    /// Build valid strings 1-4 carrying a known ephemeris.
    fn eph_strings(pos_km: [f64; 3], vel_kms: [f64; 3], acc_kms2: [f64; 3]) -> Vec<[u8; STRING_BITS]> {
        let mut out = Vec::new();
        // string 1: x, vx, ax + tk
        let mut b = [0u8; STRING_BITS];
        put_u(&mut b, 84, 81, 1);
        put_sm(&mut b, 35, 9, 2f64.powi(-11), pos_km[0]);
        put_sm(&mut b, 64, 41, 2f64.powi(-20), vel_kms[0]);
        put_sm(&mut b, 40, 36, 2f64.powi(-30), acc_kms2[0]);
        put_u(&mut b, 76, 72, 14); // tk hours
        put_u(&mut b, 71, 66, 25); // tk minutes
        put_u(&mut b, 65, 65, 0);
        set_check_bits(&mut b);
        out.push(b);
        // string 2: y, vy, ay + tb
        let mut b = [0u8; STRING_BITS];
        put_u(&mut b, 84, 81, 2);
        put_sm(&mut b, 35, 9, 2f64.powi(-11), pos_km[1]);
        put_sm(&mut b, 64, 41, 2f64.powi(-20), vel_kms[1]);
        put_sm(&mut b, 40, 36, 2f64.powi(-30), acc_kms2[1]);
        put_u(&mut b, 76, 70, 57); // tb index -> 855 min = 14:15
        set_check_bits(&mut b);
        out.push(b);
        // string 3: z, vz, az + gamma_n, ln=0
        let mut b = [0u8; STRING_BITS];
        put_u(&mut b, 84, 81, 3);
        put_sm(&mut b, 35, 9, 2f64.powi(-11), pos_km[2]);
        put_sm(&mut b, 64, 41, 2f64.powi(-20), vel_kms[2]);
        put_sm(&mut b, 40, 36, 2f64.powi(-30), acc_kms2[2]);
        put_sm(&mut b, 79, 69, 2f64.powi(-40), -3.0e-10);
        set_check_bits(&mut b);
        out.push(b);
        // string 4: tau_n, En, FT, NT, slot
        let mut b = [0u8; STRING_BITS];
        put_u(&mut b, 84, 81, 4);
        put_sm(&mut b, 80, 59, 2f64.powi(-30), 1.5e-6);
        put_u(&mut b, 53, 49, 2); // En
        put_u(&mut b, 33, 30, 0); // FT
        put_u(&mut b, 26, 16, 731); // NT
        put_u(&mut b, 15, 11, 9); // slot n
        set_check_bits(&mut b);
        out.push(b);
        out
    }

    #[test]
    fn ephemeris_assembles_from_strings_1_to_4() {
        let pos = [11_000.0, -12_000.0, 20_000.0]; // km, r ~= 25.3k km
        let vel = [1.5, 2.8, -2.1]; // km/s
        let acc = [1.0e-9, -2.0e-9, 0.5e-9]; // km/s^2
        let mut col = EphCollector::default();
        for b in eph_strings(pos, vel, acc) {
            let s = parse_string(b).expect("synthetic string must pass Hamming");
            col.push(&s);
        }
        let e = col.ephemeris().expect("ephemeris must assemble");
        for i in 0..3 {
            assert!((e.pos_m[i] - pos[i] * 1e3).abs() < 1e3 * 2f64.powi(-11), "pos[{i}]");
            assert!((e.vel_ms[i] - vel[i] * 1e3).abs() < 1e3 * 2f64.powi(-20), "vel[{i}]");
        }
        assert_eq!(e.slot, 9);
        assert_eq!(e.nt, 731);
        assert_eq!(e.tb_min, 855);
        assert_eq!(e.tk_s, 14.0 * 3600.0 + 25.0 * 60.0);
        assert!((e.tau_n_s - 1.5e-6).abs() < 2f64.powi(-30));
        assert!((e.gamma_n - -3.0e-10).abs() < 2f64.powi(-40));
        // sqrt(11000^2 + 12000^2 + 20000^2) km = 25,787.55 km
        assert!((e.radius_m() - 25_787_554.7).abs() < 5e3);
    }

    #[test]
    fn end_to_end_track_demod_decode() {
        // 6 s of synthetic L1OF at 2 Msps: 511-chip code x nav chips x carrier
        // with Doppler, tracked from the true parameters. The tracker output
        // must chip-sync, and all three strings must decode Hamming-valid.
        let fs = 2.0e6;
        let ns = (fs / 1000.0) as usize;
        let code = glonass_code();
        let lc = resample_code(&code, ns, CHIP_RATE, fs);
        let dopp = 700.0f64;
        let secs = 6usize;
        let epochs = secs * 1000;

        let mut rng = Rng(0x5555_aaaa_3333_cccc);
        let strs: Vec<[u8; STRING_BITS]> = (1..=3).map(|m| random_string(&mut rng, m)).collect();
        let mut chips = Vec::new();
        let mut prev = 1.0;
        for b in &strs {
            let (c, last) = modulate_string(b, prev);
            chips.extend_from_slice(&c);
            prev = last;
        }
        let mut sig = vec![Complex::<f32>::new(0.0, 0.0); ns * epochs];
        for e in 0..epochs {
            let data = chips[e / 10] as f32; // one 10 ms chip = 10 code periods
            for k in 0..ns {
                let g = e * ns + k;
                let c = lc[g % ns] * data;
                let ph = 2.0 * PI * dopp * g as f64 / fs;
                sig[g] = Complex::new(c, 0.0) * Complex::new(ph.cos() as f32, ph.sin() as f32);
            }
        }
        let tr = track_l1of(&sig, fs, crate::glonass::l1_freq(0), dopp, 0.0, epochs);
        assert_eq!(tr.epochs, epochs);
        let (_phase, sync, soft) = sync_from_prompt(&tr.prompt_i).expect("chip sync");
        assert!(sync.score > 0.95, "clean signal: tm score {}", sync.score);
        let decoded = decode_strings(&soft, &sync);
        assert_eq!(decoded.len(), 3);
        for ((_, s), orig) in decoded.iter().zip(strs.iter()) {
            assert_eq!(&s.bits, orig, "tracked string mismatch");
        }
    }

    #[test]
    fn end_to_end_blockwise_demod_decode() {
        // Same synthetic signal as the closed-loop test, but the demod is the
        // block-wise path and it is only told a Doppler 8 Hz off the truth
        // (the squared-spectrum carrier track must absorb that residual).
        let fs = 2.0e6;
        let ns = (fs / 1000.0) as usize;
        let code = glonass_code();
        let dopp_true = 708.0f64;
        let dopp_told = 700.0f64;
        let secs = 6usize;
        let epochs = secs * 1000;

        let mut rng = Rng(0x7777_1111_9999_3333);
        let strs: Vec<[u8; STRING_BITS]> = (1..=3).map(|m| random_string(&mut rng, m)).collect();
        let mut chips = Vec::new();
        let mut prev = 1.0;
        for b in &strs {
            let (c, last) = modulate_string(b, prev);
            chips.extend_from_slice(&c);
            prev = last;
        }
        let mut sig = vec![Complex::<f32>::new(0.0, 0.0); ns * epochs];
        // code phase advances with the code Doppler, as a real signal does
        let code_rate = CHIP_RATE * (1.0 + dopp_true / crate::glonass::l1_freq(0));
        let mut cph = 0.0f64;
        for e in 0..epochs {
            let data = chips[e / 10] as f32;
            for k in 0..ns {
                let g = e * ns + k;
                let c = code[cph as usize % CODE_LEN] * data;
                cph = (cph + code_rate / fs).rem_euclid(CODE_LEN as f64);
                let ph = 2.0 * PI * dopp_true * g as f64 / fs;
                sig[g] = Complex::new(c, 0.0) * Complex::new(ph.cos() as f32, ph.sin() as f32);
            }
        }
        let (soft, sync) = blockwise_chips(&sig, fs, crate::glonass::l1_freq(0), dopp_told, 0.0)
            .expect("squared-spectrum carrier track");
        assert!(sync.score > 0.95, "clean signal: tm score {}", sync.score);
        let decoded = decode_strings(&soft, &sync);
        assert_eq!(decoded.len(), 3);
        for ((_, s), orig) in decoded.iter().zip(strs.iter()) {
            assert_eq!(&s.bits, orig, "block-wise string mismatch");
        }
    }
}
