//! SBAS / WAAS L1 message decode (RTCA DO-229): 500 sym/s soft symbols ->
//! rate-1/2 K=7 Viterbi -> 250-bit frame sync -> CRC-24Q -> message parsing.
//!
//! Port of `validation/sbas_decode.py` (same algorithms, same measured
//! caveats). The input is the per-1 ms prompt-I stream of an SBAS channel
//! (exactly the `ip` values live.rs computes per epoch).
//!
//! LIVE HOOK (wired in live.rs): the prompt-I collection guard in
//! `Channel::process_epoch` includes `Sys::Sbas`, and `Channel::sbas_tick`
//! runs once per second from `Band::end_second`: `symbols_from_prompt` ->
//! `Decoder::push_symbols` -> `decode` (while locked), with the summary
//! published as the SatReport `sbas_msgs` field. No radio access is
//! involved; the decode is pure post-processing of the prompt stream the
//! channel already produces.
//!
//! Format facts (DO-229, verified against the Python chain on the synthetic
//! capture `../observations/sbas_work/multi.iq`):
//!   * 250-bit blocks at 1 Hz: 8-bit preamble (0x53 / 0x9A / 0xC6, rotating
//!     across three consecutive blocks), 6-bit message type, 212-bit data,
//!     24-bit CRC-24Q over the whole block (receiver check: CRC == 0).
//!   * Rate-1/2 K=7 convolutional code, G1 = 171 oct, G2 = 133 oct, G1 symbol
//!     first, CONTINUOUS encoder (not reset per block) -> 500 symbols/s.
//!   * BPSK with a 180 deg Costas ambiguity. Both generator polynomials have
//!     ODD weight, so an inverted symbol stream is still a valid codeword and
//!     the Viterbi output is simply complemented from the slip onward — the
//!     ambiguity is resolved per block by testing the CRC in both polarities
//!     (CRC-24Q of an all-ones block is 0xEF339D != 0).

// ---------------------------------------------------------------------------
// constants
// ---------------------------------------------------------------------------

/// Bits per SBAS block (1 Hz block rate).
pub const BLOCK_BITS: usize = 250;
/// The rotating 8-bit preambles; the 24-bit sync pattern spans 3 blocks.
pub const PREAMBLES: [u8; 3] = [0x53, 0x9A, 0xC6];
/// CRC-24Q polynomial (same as GPS CNAV): x^24 + x^23 + x^18 + x^17 + x^14 +
/// x^11 + x^10 + x^7 + x^6 + x^5 + x^4 + x^3 + x + 1.
pub const CRC24Q_POLY: u32 = 0x1864CFB;
/// A real 250-bit SBAS message sets roughly 100 bits. An all-zero (or
/// near-zero) block satisfies CRC-24Q trivially (CRC(0) = 0 and the appended
/// CRC field is also 0), so the zero syndrome alone is not a detection —
/// measured on a noise capture: the Viterbi output occasionally collapses
/// toward zeros and those blocks "pass" (they all decode as MT0). Reject them.
pub const MIN_BLOCK_WEIGHT: usize = 12;

/// Constraint length of the convolutional code.
pub const K: usize = 7;
/// Generator polynomials, octal 171 / 133.
pub const G_POLY: (u32, u32) = (0o171, 0o133);
/// Number of encoder states (2^(K-1)).
pub const NSTATES: usize = 1 << (K - 1);

/// UDRE variance values (m^2) for UDREI 0..=12 (DO-229 table); 13/14/15 are
/// "not monitored" / "do not use" / "no accuracy" and carry no number.
pub const UDRE_M2: [f64; 13] = [
    0.0520, 0.0924, 0.1657, 0.2831, 0.4967, 0.8858, 1.5793, 2.7861, 5.1592,
    10.4326, 20.7863, 230.9661, 2078.695,
];

// ---------------------------------------------------------------------------
// CRC-24Q
// ---------------------------------------------------------------------------

/// CRC-24Q over a sequence of 0/1 bits, MSB-first (bitwise reference).
pub fn crc24q(bits: &[u8]) -> u32 {
    let mask = CRC24Q_POLY & 0xFF_FFFF;
    let mut reg = 0u32;
    for &b in bits {
        let top = ((reg >> 23) & 1) ^ (b as u32 & 1);
        reg = (reg << 1) & 0xFF_FFFF;
        if top != 0 {
            reg ^= mask;
        }
    }
    reg
}

/// True if the block weight is outside the degenerate all-zero / all-ones
/// bands that pass CRC-24Q arithmetically while carrying no information.
pub fn block_weight_ok(weight: usize) -> bool {
    (MIN_BLOCK_WEIGHT..=BLOCK_BITS - MIN_BLOCK_WEIGHT).contains(&weight)
}

/// A 250-bit SBAS block is valid iff the CRC over all 250 bits is zero
/// (appended-CRC linear-code property — no separate "first 226" pass) AND the
/// block is not degenerate.
pub fn block_crc_ok(block: &[u8]) -> bool {
    debug_assert_eq!(block.len(), BLOCK_BITS);
    let w = block.iter().map(|&b| (b & 1) as usize).sum::<usize>();
    block_weight_ok(w) && crc24q(block) == 0
}

// ---------------------------------------------------------------------------
// rate-1/2 K=7 convolutional code, G1 = 171 oct, G2 = 133 oct
// ---------------------------------------------------------------------------

fn parity(mut x: u32) -> u8 {
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    (x & 1) as u8
}

/// `out[state][bit]` = (c0, c1) symbol pair; `nxt[state][bit]` = next state.
/// `invert_g2` models the opposite G2-output convention — the on-air truth is
/// settled by CRC yield, so the decoder tries both.
fn conv_tables(invert_g2: bool) -> ([[[u8; 2]; 2]; NSTATES], [[u8; 2]; NSTATES]) {
    let mut out = [[[0u8; 2]; 2]; NSTATES];
    let mut nxt = [[0u8; 2]; NSTATES];
    for s in 0..NSTATES {
        for b in 0..2usize {
            let full = ((b as u32) << (K - 1)) | s as u32;
            let c0 = parity(full & G_POLY.0);
            let mut c1 = parity(full & G_POLY.1);
            if invert_g2 {
                c1 ^= 1;
            }
            out[s][b] = [c0, c1];
            nxt[s][b] = (full >> 1) as u8;
        }
    }
    (out, nxt)
}

/// Continuous (non-terminated) encoder, G1 symbol first. Exposed for tests
/// and for building synthetic streams; returns 2 symbols per input bit.
pub fn conv_encode(bits: &[u8], invert_g2: bool, state: u8) -> Vec<u8> {
    let (out, nxt) = conv_tables(invert_g2);
    let mut sym = Vec::with_capacity(2 * bits.len());
    let mut s = state as usize;
    for &b in bits {
        let b = (b & 1) as usize;
        sym.push(out[s][b][0]);
        sym.push(out[s][b][1]);
        s = nxt[s][b] as usize;
    }
    sym
}

/// Soft-decision Viterbi decode. `soft[i] > 0` means symbol bit 0 (the
/// `1 - 2*sym` convention of the Python chain). Initial state unknown, as on
/// air; the decode is exact after a few constraint lengths. Output length is
/// `soft.len() / 2` bits.
pub fn viterbi(soft: &[f32], invert_g2: bool) -> Vec<u8> {
    let (out, _nxt) = conv_tables(invert_g2);
    let m = soft.len() / 2;
    // expected symbol as +-1 for (state, input bit, branch): 1.0 - 2*bit
    let mut s0 = [[0f32; 2]; NSTATES];
    let mut s1 = [[0f32; 2]; NSTATES];
    for ns in 0..NSTATES {
        let b = (ns >> 5) & 1;
        for j in 0..2 {
            let ps = 2 * (ns & 31) + j;
            s0[ns][j] = 1.0 - 2.0 * out[ps][b][0] as f32;
            s1[ns][j] = 1.0 - 2.0 * out[ps][b][1] as f32;
        }
    }
    let mut metric = [0f32; NSTATES];
    // decision bit per (time, next-state): which predecessor (j) won
    let mut dec = vec![0u8; m * NSTATES];
    for k in 0..m {
        let r0 = soft[2 * k];
        let r1 = soft[2 * k + 1];
        let mut next = [0f32; NSTATES];
        for ns in 0..NSTATES {
            let mut best = f32::NEG_INFINITY;
            let mut bestj = 0u8;
            for j in 0..2 {
                let ps = 2 * (ns & 31) + j;
                let c = metric[ps] + r0 * s0[ns][j] + r1 * s1[ns][j];
                if c > best {
                    best = c;
                    bestj = j as u8;
                }
            }
            next[ns] = best;
            dec[k * NSTATES + ns] = bestj;
        }
        // normalize against overflow / drift
        let mx = next.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        for v in next.iter_mut() {
            *v -= mx;
        }
        metric = next;
    }
    // traceback from the best final state
    let mut s = metric
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .map(|(i, _)| i)
        .unwrap();
    let mut bits = vec![0u8; m];
    for k in (0..m).rev() {
        bits[k] = (s >> 5) as u8;
        s = 2 * (s & 31) + dec[k * NSTATES + s] as usize;
    }
    bits
}

// ---------------------------------------------------------------------------
// symbol recovery: 1 ms prompts -> 2 ms soft symbols
// ---------------------------------------------------------------------------

/// Combine per-1 ms prompt correlations into 2 ms SBAS symbols. A symbol is
/// exactly two code periods, so there are only two possible alignments; pick
/// the one with more energy. Returns (soft symbols, parity offset consumed).
/// Soft > 0 means symbol bit 0.
pub fn symbols_from_prompt(prompts: &[f64]) -> (Vec<f32>, usize) {
    symbols_from_prompt_par(prompts, None)
}

/// As `symbols_from_prompt`, but with an optional forced parity. A locked
/// decoder latches its parity: re-picking by energy every second lets a
/// noisy parity flip insert or delete one coded symbol and silently break
/// the Viterbi stream (review round 5).
pub fn symbols_from_prompt_par(prompts: &[f64], forced: Option<usize>) -> (Vec<f32>, usize) {
    let par = match forced {
        Some(p) => p.min(1),
        None => {
            let m = prompts.len() / 2 * 2;
            let e0: f64 = prompts[..m].chunks_exact(2).map(|c| (c[0] + c[1]).abs()).sum();
            let m1 = (prompts.len().saturating_sub(1)) / 2 * 2;
            let e1: f64 = if m1 > 0 {
                prompts[1..1 + m1].chunks_exact(2).map(|c| (c[0] + c[1]).abs()).sum()
            } else {
                -1.0
            };
            if e0 >= e1 { 0 } else { 1 }
        }
    };
    if prompts.len() <= par {
        return (Vec::new(), par);
    }
    let n = (prompts.len() - par) / 2 * 2;
    let soft = prompts[par..par + n]
        .chunks_exact(2)
        .map(|c| (c[0] + c[1]) as f32)
        .collect();
    (soft, par)
}

/// As `symbols_from_prompt_par`, but the forced parity is ABSOLUTE: pairs
/// hold absolute 1 ms indices (k, k+1) with k ≡ forced (mod 2), where
/// `head_abs` is the absolute index of `prompts[0]`. A queue-relative latch
/// cannot hold that grid: a straddling leftover ms shifts the queue head by
/// one, and re-forcing the same queue offset pairs the OTHER grid for a
/// whole call (the par=1 phase slip). Returns (soft symbols, queue-relative
/// start consumed, absolute pairing parity) — the last is what the caller
/// latches and compares across calls. With forced=None the energy probe
/// picks the queue-relative start (as `symbols_from_prompt_par`) and the
/// absolute parity follows from `head_abs`.
pub fn symbols_from_prompt_abs(
    prompts: &[f64],
    forced: Option<usize>,
    head_abs: u64,
) -> (Vec<f32>, usize, usize) {
    let hq = (head_abs % 2) as usize;
    let forced_q = forced.map(|pa| (pa.min(1) + 2 - hq) % 2);
    let (soft, s) = symbols_from_prompt_par(prompts, forced_q);
    (soft, s, (hq + s) % 2)
}

/// MT2-5 rows by ORDINAL through the MT1 mask, UDREI-UNFILTERED (unlike
/// `fast_corrections`): every ordinal-mapped GPS row is returned with its
/// UDREI so the caller can act on UDREI >= 14 (not monitored / don't use)
/// — a fresh don't-use row must EVICT any cached usable correction for
/// that PRN, which a filtered view cannot express. Same IODP gate, same
/// scales; see `fast_corrections` for the addressing rules.
pub fn fast_rows(
    mask_slots: &[u8],
    mask_iodp: u8,
    msg_iodp: u8,
    first_slot: u8,
    prc: &[i16; 13],
    udrei: &[u8; 13],
) -> Vec<(u8, f64, u8)> {
    if mask_iodp != msg_iodp {
        return Vec::new();
    }
    (0..13usize)
        .filter_map(|k| {
            let ordinal = first_slot as usize + k;
            let slot = *mask_slots.get(ordinal - 1)?;
            if (1..=37).contains(&slot) {
                Some((slot, prc[k] as f64 * 0.125, udrei[k]))
            } else {
                None
            }
        })
        .collect()
}

/// Harvest MT2-5 fast corrections as (GPS PRN, PRC metres, UDREI) rows.
/// Entries map to satellites by ORDINAL through the set bits of the MT1
/// PRN mask, not by absolute slot (DO-229D A.4.4.3: "Message Type 2
/// contains the data sets for the first 13 satellites designated in the
/// PRN mask. Message Type 3 ... satellites 14 - 26", etc.): `mask_slots`
/// is the 1-based absolute slot numbers of the set bits in mask order
/// (Message::PrnMask), `first_slot` the 1-based ordinal of this message's
/// first entry ((mt-2)*13+1). A WAAS mask always has gaps, so ordinal and
/// slot differ. Corrections are valid only while the message IODP equals
/// the mask's IODP (DO-229D A.4.4.2) — a mismatch or an empty/short mask
/// yields no rows. Absolute slots 1..=37 are GPS (slot number = GPS PRN);
/// UDREI >= 14 (not monitored / don't use) rows are excluded. PRC scale
/// is 0.125 m. DO-229 convention: the PRC is ADDED to the measured
/// pseudorange.
pub fn fast_corrections(
    mask_slots: &[u8],
    mask_iodp: u8,
    msg_iodp: u8,
    first_slot: u8,
    prc: &[i16; 13],
    udrei: &[u8; 13],
) -> Vec<(u8, f64, u8)> {
    fast_rows(mask_slots, mask_iodp, msg_iodp, first_slot, prc, udrei)
        .into_iter()
        .filter(|&(_, _, u)| u < 14)
        .collect()
}

/// One harvested long-term correction (MT25 half / MT24 long-term slot),
/// physical units. Velocity-code-0 rows carry no rates and no time of
/// applicability (their t of applicability is the message transmission
/// time, DO-229D A.4.4.7) — the Option fields are None and the row
/// propagates as a constant.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LtCorr {
    /// GPS PRN (absolute mask slot 1..=37).
    pub prn: u8,
    /// Position correction, metres (WGS-84 ECEF), at t_lt.
    pub dx: f64,
    pub dy: f64,
    pub dz: f64,
    /// Clock correction δaf0, seconds, at t_lt.
    pub daf0: f64,
    /// Velocity-code-1 only: position rate corrections, m/s (8-bit, 2^-11
    /// m/s LSB — DO-229D Table A-11).
    pub ddx: Option<f64>,
    pub ddy: Option<f64>,
    pub ddz: Option<f64>,
    /// Velocity-code-1 only: clock drift δaf1, s/s (8-bit, 2^-39 s/s LSB —
    /// DO-229D Table A-11).
    pub daf1: Option<f64>,
    /// Velocity-code-1 only: time-of-day applicability t0, seconds of the
    /// GPS day (13-bit, 16 s LSB, range 0..=86384 — DO-229D Table A-11
    /// "Time-of-Day Applicability t0"; it is a time of day, NOT a GPS
    /// time-of-week).
    pub t_lt_s: Option<u32>,
    /// GPS IODE of the ephemeris the correction was generated against
    /// (DO-229D Table A-10 Note 3 / A.4.4.7: it must match the broadcast
    /// IODC's low 8 bits AND the IODE) — the application must match it
    /// against the ephemeris in use.
    pub iod: u8,
}

impl LtCorr {
    /// Propagate to the current epoch per DO-229D A.4.4.7: position
    /// eq. (A-19) [δxk δyk δzk] = [δx δy δz] + [δẋ δẏ δż]·(t − t0), clock
    /// eq. (A-18) δΔtSV(t) = δaf0 + δaf1·(t − t0) (+ δafG0, which is 0 for
    /// GPS satellites). Returns (dx, dy, dz metres, daf0 seconds) at t.
    ///
    /// `tow_s` is GPS time (seconds-of-week is fine — it is folded to
    /// time-of-day). t0 is a time-of-DAY (Table A-11), so the difference is
    /// taken on seconds-of-day, "correcting for rollover if needed"
    /// (A.4.4.7): wrapped into ±43200 s, the end-of-day cross-over. t0 is
    /// "usually approximately 2 minutes in the future of the transmission
    /// time ... [but] may be in the past if the prior long-term message is
    /// missed" (A.4.4.7), so dt is small and signed. Velocity-code-0 rows
    /// (t_lt None) propagate as constants: the rates are 0 and the time of
    /// applicability is the message transmission time (A.4.4.7).
    pub fn propagate(&self, tow_s: f64) -> (f64, f64, f64, f64) {
        let dt = match self.t_lt_s {
            Some(t_lt) => {
                let mut d = tow_s.rem_euclid(86_400.0) - t_lt as f64;
                if d > 43_200.0 {
                    d -= 86_400.0;
                } else if d < -43_200.0 {
                    d += 86_400.0;
                }
                d
            }
            None => 0.0,
        };
        (
            self.dx + self.ddx.unwrap_or(0.0) * dt,
            self.dy + self.ddy.unwrap_or(0.0) * dt,
            self.dz + self.ddz.unwrap_or(0.0) * dt,
            self.daf0 + self.daf1.unwrap_or(0.0) * dt,
        )
    }
}

/// Harvest a long-term half message (MT25 halves, MT24 long-term slot) as
/// one `LtCorr` row per satellite (physical units). The 6-bit PRN mask
/// number is an ORDINAL into the mask, exactly like the MT2-5 entries:
/// DO-229D A.4.4.7 — "The PRN Mask No. is the sequence number of the bits
/// set in the 210 bit mask (that is, between 1 and 51)" (Table A-10 Note 2:
/// the count of 1's in the mask up to the subject satellite's bit; 0 = no
/// satellite, ignore the entry). The half's IODP must match the mask's
/// IODP (A.4.4.2/A.4.4.7). Velocity-code-1 halves (one satellite with
/// rates and a t_lt — what WAAS broadcasts in practice) are carried WITH
/// their rates; the application propagates them at solve time
/// (LtCorr::propagate, A.4.4.7 eq. A-18/A-19). Velocity-code-0 rows carry
/// rates/t_lt as None and apply as constants. Absolute slots 1..=37 are
/// GPS. Scales (Tables A-10/A-11): dx/dy/dz 0.125 m; daf0 2^-31 s; rates
/// 2^-11 m/s; daf1 2^-39 s/s. DO-229 sign convention (A.4.4.7): the
/// position correction vector is ADDED to the broadcast satellite
/// coordinates and δΔtSV is ADDED to the broadcast clock offset.
pub fn lt_corrections(
    mask_slots: &[u8],
    mask_iodp: u8,
    half: &LongTermHalf,
) -> Vec<LtCorr> {
    if half.iodp != mask_iodp {
        return Vec::new();
    }
    half.sats
        .iter()
        .filter_map(|s| {
            let slot = *mask_slots.get((s.mask as usize).checked_sub(1)?)?;
            if (1..=37).contains(&slot) {
                Some(LtCorr {
                    prn: slot,
                    dx: s.dx as f64 * 0.125,
                    dy: s.dy as f64 * 0.125,
                    dz: s.dz as f64 * 0.125,
                    daf0: s.daf0 as f64 * 2.0f64.powi(-31),
                    ddx: s.ddx.map(|v| v as f64 * 2.0f64.powi(-11)),
                    ddy: s.ddy.map(|v| v as f64 * 2.0f64.powi(-11)),
                    ddz: s.ddz.map(|v| v as f64 * 2.0f64.powi(-11)),
                    daf1: s.daf1.map(|v| v as f64 * 2.0f64.powi(-39)),
                    t_lt_s: half.t_lt_s,
                    iod: s.iod,
                })
            } else {
                None
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// frame sync

/// Result of a frame-sync search over a decoded bit stream.
#[derive(Debug, Clone, Default)]
pub struct SyncResult {
    /// Bit offset of the first block boundary (None if no blocks fit).
    pub offset: Option<usize>,
    /// Number of 250-bit blocks examined at the winning offset.
    pub nblocks: usize,
    /// Blocks passing CRC-24Q in either polarity (and the weight guard).
    pub npass: usize,
    /// Per-block CRC validity at the winning offset.
    pub valid: Vec<bool>,
    /// Per-block flag: CRC passed only with inverted bits (180 deg slip).
    pub inverted: Vec<bool>,
    /// True when `npass >= min_blocks`.
    pub locked: bool,
}

/// Length of the TRAILING run of consecutive blocks that are CRC-valid,
/// nondegenerate (content differs from the previous block), and carry the
/// correctly rotating preamble (0x53 -> 0x9A -> 0xC6 across consecutive
/// blocks). This is the frame-lock criterion (review round 5): scattered
/// CRC passes anywhere in the window do NOT count — a single bad or
/// mis-rotating block resets the run.
pub fn lock_streak(blocks: &[u8], valid: &[bool]) -> usize {
    let mut streak = 0usize;
    let mut expect: Option<u8> = None;
    let mut prev: Option<&[u8]> = None;
    for (i, blk) in blocks.chunks_exact(BLOCK_BITS).enumerate() {
        let ok = valid.get(i).copied().unwrap_or(false) && {
            let mut pre = 0u8;
            for &b in &blk[..8] {
                pre = (pre << 1) | (b & 1);
            }
            let rot_ok = match expect {
                None => PREAMBLES.contains(&pre),
                Some(e) => pre == e,
            };
            let degen = prev.map(|p| p == blk).unwrap_or(false);
            if rot_ok && !degen {
                let idx = PREAMBLES.iter().position(|&p| p == pre).unwrap();
                expect = Some(PREAMBLES[(idx + 1) % 3]);
                true
            } else {
                false
            }
        };
        if ok {
            streak += 1;
        } else {
            streak = 0;
            expect = None;
        }
        prev = Some(blk);
    }
    streak
}

/// Search all 250 bit offsets; return the one with the most CRC-valid blocks.
/// Every candidate block is tested in both polarities (the 180 deg Costas
/// ambiguity only complements the bits — see module docs).
pub fn frame_sync(bits: &[u8], min_blocks: usize) -> SyncResult {
    let n = bits.len();
    let mut best = SyncResult::default();
    for off in 0..BLOCK_BITS.min(n) {
        let nb = (n - off) / BLOCK_BITS;
        if nb == 0 {
            continue;
        }
        let mut valid = vec![false; nb];
        let mut inverted = vec![false; nb];
        let mut npass = 0;
        for (i, blk) in bits[off..off + nb * BLOCK_BITS]
            .chunks_exact(BLOCK_BITS)
            .enumerate()
        {
            if block_crc_ok(blk) {
                valid[i] = true;
                npass += 1;
            } else {
                let inv: Vec<u8> = blk.iter().map(|&b| b ^ 1).collect();
                if block_crc_ok(&inv) {
                    valid[i] = true;
                    inverted[i] = true;
                    npass += 1;
                }
            }
        }
        if npass > best.npass {
            best = SyncResult {
                offset: Some(off),
                nblocks: nb,
                npass,
                valid,
                inverted,
                locked: false,
            };
        }
    }
    best.locked = best.npass >= min_blocks;
    best
}

/// Check the 0x53 / 0x9A / 0xC6 preamble rotation across valid blocks.
/// `blocks` is the concatenated block stream (nblocks * 250 bits) at the sync
/// offset. Returns (phase, n_matching, n_valid).
pub fn preamble_phase(blocks: &[u8], valid: &[bool]) -> (usize, usize, usize) {
    let nvalid = valid.iter().filter(|&&v| v).count();
    let mut best_ph = 0;
    let mut best_n = 0usize;
    for ph in 0..3 {
        let mut n = 0;
        for (i, blk) in blocks.chunks_exact(BLOCK_BITS).enumerate() {
            if !valid.get(i).copied().unwrap_or(false) {
                continue;
            }
            let mut pre = 0u8;
            for &b in &blk[..8] {
                pre = (pre << 1) | (b & 1);
            }
            if pre == PREAMBLES[(i + ph) % 3] {
                n += 1;
            }
        }
        if n > best_n {
            best_n = n;
            best_ph = ph;
        }
    }
    (best_ph, best_n, nvalid)
}

// ---------------------------------------------------------------------------
// bit reader
// ---------------------------------------------------------------------------

struct BitReader<'a> {
    b: &'a [u8],
    p: usize,
}

impl<'a> BitReader<'a> {
    fn new(b: &'a [u8]) -> Self {
        BitReader { b, p: 0 }
    }
    fn u(&mut self, width: usize) -> u64 {
        let mut v = 0u64;
        for _ in 0..width {
            v = (v << 1) | (self.b[self.p] & 1) as u64;
            self.p += 1;
        }
        v
    }
    fn s(&mut self, width: usize) -> i64 {
        let v = self.u(width) as i64;
        if v >= (1i64 << (width - 1)) {
            v - (1i64 << width)
        } else {
            v
        }
    }
    fn raw(&mut self, width: usize) -> &'a [u8] {
        let v = &self.b[self.p..self.p + width];
        self.p += width;
        v
    }
    fn skip(&mut self, width: usize) {
        self.p += width;
    }
}

// ---------------------------------------------------------------------------
// message parsing (212-bit data field)
// ---------------------------------------------------------------------------

/// One satellite entry of a long-term correction half message (MT24/25).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LongTermSat {
    /// PRN mask number: ORDINAL of the satellite among the set bits of
    /// the MT1 mask (1..=51; DO-229D A.4.4.7), not an absolute slot. 0 =
    /// no satellite (Table A-10 Note 2).
    pub mask: u8,
    /// Issue of data (ephemeris).
    pub iod: u8,
    /// Position correction, raw counts (scale 0.125 m).
    pub dx: i16,
    pub dy: i16,
    pub dz: i16,
    /// Clock correction, raw counts (scale 2^-31 s).
    pub daf0: i16,
    /// Velocity-code-1 only: rate corrections, raw counts (scale 2^-11 m/s).
    pub ddx: Option<i16>,
    pub ddy: Option<i16>,
    pub ddz: Option<i16>,
    /// Velocity-code-1 only: clock rate, raw counts (scale 2^-39 s/s).
    pub daf1: Option<i16>,
}

/// One 106-bit long-term half message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LongTermHalf {
    /// 0: two satellites, no rates. 1: one satellite with rates.
    pub velocity_code: u8,
    pub sats: Vec<LongTermSat>,
    /// Time-of-applicability, velocity-code-1 only (scale 16 s).
    pub t_lt_s: Option<u32>,
    pub iodp: u8,
}

/// One GEO almanac entry (MT17).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GeoAlm {
    pub prn: u8,
    pub health: u8,
    /// Position, raw counts (x/y scale 2600 m, z scale 26000 m).
    pub x: i16,
    pub y: i16,
    pub z: i16,
    /// Velocity, raw counts (x/y scale 10 m/s, z scale 60 m/s).
    pub vx: i8,
    pub vy: i8,
    pub vz: i8,
}

/// A parsed SBAS message. Raw integer fields are kept raw; scale factors are
/// documented per field (DO-229).
#[derive(Debug, Clone, PartialEq)]
pub enum Message {
    /// MT0: "Don't Use for Safety Applications" (test mode).
    DontUse,
    /// MT1: PRN mask — slot numbers 1..=210 (1..37 are GPS).
    PrnMask { slots: Vec<u8>, iodp: u8 },
    /// MT2-5: fast corrections for 13 satellites designated in the MT1
    /// PRN mask (addressed by ordinal, see fast_corrections). PRC scale
    /// 0.125 m.
    Fast {
        iodf: u8,
        iodp: u8,
        /// Ordinal of the first entry among the mask's set bits (1-based,
        /// (mt-2)*13+1).
        first_slot: u8,
        prc: [i16; 13],
        /// UDRE indicator per slot (index into UDRE_M2; 13/14/15 special).
        udrei: [u8; 13],
    },
    /// MT6: integrity — UDREI for 51 slots.
    Integrity { iodf: [u8; 4], udrei: Vec<u8> },
    /// MT7: fast-correction degradation factors.
    FastDegradation {
        /// System latency (s).
        latency: u8,
        iodp: u8,
        ai: Vec<u8>,
    },
    /// MT9: GEO navigation message.
    GeoNav {
        iodn: u8,
        /// Time of day, s (scale 16 s from the start of the GPS day... kept
        /// in raw 13-bit units * 16, as broadcast).
        t0_s: u32,
        ura: u8,
        /// ECEF position, raw counts (x/y scale 0.08 m, z scale 0.4 m).
        xyz: [i32; 3],
        /// Velocity, raw counts (x/y scale 0.000625 m/s, z 0.004 m/s).
        vxyz: [i32; 3],
        /// Acceleration, raw counts (x/y scale 0.0000125 m/s^2, z 0.0000625).
        axyz: [i16; 3],
        /// Clock offset, raw counts (scale 2^-31 s).
        agf0: i16,
        /// Clock drift, raw counts (scale 2^-40 s/s).
        agf1: i16,
    },
    /// MT17: GEO almanacs.
    GeoAlmanac { alms: Vec<GeoAlm>, t0_s: u32 },
    /// MT18: ionospheric grid point mask.
    IonoMask {
        nbands: u8,
        band: u8,
        iodi: u8,
        /// IGP numbers (1-based within the band's 201-bit mask).
        igps: Vec<u16>,
    },
    /// MT24: mixed fast (6 slots) + one long-term half message.
    MixedFastLongTerm {
        prc: [i16; 6],
        udrei: [u8; 6],
        iodp: u8,
        block_id: u8,
        iodf: u8,
        lt: LongTermHalf,
    },
    /// MT25: long-term satellite error corrections (two half messages).
    LongTerm { a: LongTermHalf, b: LongTermHalf },
    /// MT26: ionospheric delay corrections for 15 IGPs.
    IonoDelay {
        band: u8,
        block_id: u8,
        iodi: u8,
        /// (vertical delay, GIVEI) per IGP; delay scale 0.125 m, 511 = not
        /// monitored.
        igps: [(u16, u8); 15],
    },
    /// Any other / reserved message type.
    Other(u8),
}

impl Message {
    /// The 6-bit message type number.
    pub fn mt(&self) -> u8 {
        match self {
            Message::DontUse => 0,
            Message::PrnMask { .. } => 1,
            Message::Fast { first_slot, .. } => 2 + (first_slot - 1) / 13,
            Message::Integrity { .. } => 6,
            Message::FastDegradation { .. } => 7,
            Message::GeoNav { .. } => 9,
            Message::GeoAlmanac { .. } => 17,
            Message::IonoMask { .. } => 18,
            Message::MixedFastLongTerm { .. } => 24,
            Message::LongTerm { .. } => 25,
            Message::IonoDelay { .. } => 26,
            Message::Other(mt) => *mt,
        }
    }
}

fn parse_lt_half(d: &mut BitReader) -> LongTermHalf {
    let vc = d.u(1) as u8;
    if vc == 0 {
        let sats = (0..2)
            .map(|_| LongTermSat {
                mask: d.u(6) as u8,
                iod: d.u(8) as u8,
                dx: d.s(9) as i16,
                dy: d.s(9) as i16,
                dz: d.s(9) as i16,
                daf0: d.s(10) as i16,
                ddx: None,
                ddy: None,
                ddz: None,
                daf1: None,
            })
            .collect();
        let iodp = d.u(2) as u8;
        LongTermHalf {
            velocity_code: 0,
            sats,
            t_lt_s: None,
            iodp,
        }
    } else {
        let s = LongTermSat {
            mask: d.u(6) as u8,
            iod: d.u(8) as u8,
            dx: d.s(11) as i16,
            dy: d.s(11) as i16,
            dz: d.s(11) as i16,
            daf0: d.s(11) as i16,
            ddx: Some(d.s(8) as i16),
            ddy: Some(d.s(8) as i16),
            ddz: Some(d.s(8) as i16),
            daf1: Some(d.s(8) as i16),
        };
        let t_lt_s = Some(d.u(13) as u32 * 16);
        let iodp = d.u(2) as u8;
        LongTermHalf {
            velocity_code: 1,
            sats: vec![s],
            t_lt_s,
            iodp,
        }
    }
}

/// Parse one 250-bit block. Returns None if the CRC fails or the block is
/// degenerate (all zeros / all ones pass CRC trivially — see
/// MIN_BLOCK_WEIGHT). Reachable without frame_sync, so it carries the guard
/// itself.
pub fn parse_block(bits: &[u8]) -> Option<Message> {
    if bits.len() != BLOCK_BITS || !block_crc_ok(bits) {
        return None;
    }
    let mut r = BitReader::new(bits);
    let _pre = r.u(8);
    let mt = r.u(6) as u8;
    let mut d = BitReader::new(r.raw(212));
    let msg = match mt {
        0 => Message::DontUse,
        1 => {
            let m = d.raw(210);
            let slots = m
                .iter()
                .enumerate()
                .filter(|&(_, &b)| b & 1 == 1)
                .map(|(i, _)| (i + 1) as u8)
                .collect();
            let iodp = d.u(2) as u8;
            Message::PrnMask { slots, iodp }
        }
        2..=5 => {
            let iodf = d.u(2) as u8;
            let iodp = d.u(2) as u8;
            let mut prc = [0i16; 13];
            for v in prc.iter_mut() {
                *v = d.s(12) as i16;
            }
            let mut udrei = [0u8; 13];
            for v in udrei.iter_mut() {
                *v = d.u(4) as u8;
            }
            Message::Fast {
                iodf,
                iodp,
                first_slot: (mt - 2) * 13 + 1,
                prc,
                udrei,
            }
        }
        6 => {
            let mut iodf = [0u8; 4];
            for v in iodf.iter_mut() {
                *v = d.u(2) as u8;
            }
            let udrei = (0..51).map(|_| d.u(4) as u8).collect();
            Message::Integrity { iodf, udrei }
        }
        7 => {
            let latency = d.u(4) as u8;
            let iodp = d.u(2) as u8;
            d.skip(2);
            let ai = (0..51).map(|_| d.u(4) as u8).collect();
            Message::FastDegradation { latency, iodp, ai }
        }
        9 => {
            let iodn = d.u(8) as u8;
            let t0_s = d.u(13) as u32 * 16;
            let ura = d.u(4) as u8;
            let xyz = [d.s(30) as i32, d.s(30) as i32, d.s(25) as i32];
            let vxyz = [d.s(17) as i32, d.s(17) as i32, d.s(18) as i32];
            let axyz = [d.s(10) as i16, d.s(10) as i16, d.s(10) as i16];
            let agf0 = d.s(12) as i16;
            let agf1 = d.s(8) as i16;
            Message::GeoNav {
                iodn,
                t0_s,
                ura,
                xyz,
                vxyz,
                axyz,
                agf0,
                agf1,
            }
        }
        17 => {
            let alms = (0..3)
                .map(|_| {
                    d.skip(2);
                    GeoAlm {
                        prn: d.u(8) as u8,
                        health: d.u(8) as u8,
                        x: d.s(15) as i16,
                        y: d.s(15) as i16,
                        z: d.s(9) as i16,
                        vx: d.s(3) as i8,
                        vy: d.s(3) as i8,
                        vz: d.s(4) as i8,
                    }
                })
                .collect();
            let t0_s = d.u(11) as u32 * 64;
            Message::GeoAlmanac { alms, t0_s }
        }
        18 => {
            let nbands = d.u(4) as u8;
            let band = d.u(4) as u8;
            let iodi = d.u(2) as u8;
            let m = d.raw(201);
            let igps = m
                .iter()
                .enumerate()
                .filter(|&(_, &b)| b & 1 == 1)
                .map(|(i, _)| (i + 1) as u16)
                .collect();
            Message::IonoMask {
                nbands,
                band,
                iodi,
                igps,
            }
        }
        24 => {
            let mut prc = [0i16; 6];
            for v in prc.iter_mut() {
                *v = d.s(12) as i16;
            }
            let mut udrei = [0u8; 6];
            for v in udrei.iter_mut() {
                *v = d.u(4) as u8;
            }
            let iodp = d.u(2) as u8;
            let block_id = d.u(2) as u8;
            let iodf = d.u(2) as u8;
            d.skip(4);
            let mut half = BitReader::new(d.raw(106));
            let lt = parse_lt_half(&mut half);
            Message::MixedFastLongTerm {
                prc,
                udrei,
                iodp,
                block_id,
                iodf,
                lt,
            }
        }
        25 => {
            let mut ha = BitReader::new(d.raw(106));
            let a = parse_lt_half(&mut ha);
            let mut hb = BitReader::new(d.raw(106));
            let b = parse_lt_half(&mut hb);
            Message::LongTerm { a, b }
        }
        26 => {
            let band = d.u(4) as u8;
            let block_id = d.u(4) as u8;
            let mut igps = [(0u16, 0u8); 15];
            for v in igps.iter_mut() {
                *v = (d.u(9) as u16, d.u(4) as u8);
            }
            let iodi = d.u(2) as u8;
            Message::IonoDelay {
                band,
                block_id,
                iodi,
                igps,
            }
        }
        other => Message::Other(other),
    };
    Some(msg)
}

// ---------------------------------------------------------------------------
// full back end: soft symbols -> messages
// ---------------------------------------------------------------------------

/// One decoded message and its position.
#[derive(Debug, Clone)]
pub struct DecodedMessage {
    /// Block index within the framed region of the decode window.
    pub block_index: usize,
    /// Symbol position of this block's first symbol. `decode_symbols`
    /// reports it relative to the input buffer; `Decoder::decode` rebases
    /// it to the ABSOLUTE stream position (symbols pushed since decoder
    /// creation) — an immutable block identity for apply-once bookkeeping:
    /// the same physical block keeps the same sym_pos across re-decodes
    /// of the sliding retained window, and a newer block always sorts
    /// higher (review round 6).
    pub sym_pos: usize,
    pub message: Message,
}

/// Report of the full back-end chain over a soft-symbol buffer.
#[derive(Debug, Clone)]
pub struct DecodeReport {
    /// Winning symbol-pair phase (0/1).
    pub sym_offset: usize,
    /// Winning G2-output convention.
    pub invert_g2: bool,
    pub sync: SyncResult,
    /// Preamble rotation check: (phase, n_matching, n_valid).
    pub preamble: (usize, usize, usize),
    /// Trailing run of consecutive CRC-valid, correctly-rotating,
    /// nondegenerate blocks — the actual lock criterion.
    pub streak: usize,
    /// Messages parsed from CRC-valid, polarity-resolved blocks.
    pub messages: Vec<DecodedMessage>,
}

/// One-shot decode of a soft-symbol buffer: tries both symbol-pair phases and
/// both G2 conventions (the 180 deg carrier ambiguity needs no separate
/// decode — it only complements bits and is resolved per block by CRC).
/// Hypotheses are scored by their CURRENT rotating-block streak first
/// (review round 6): a hypothesis with many scattered CRC passes but no
/// consecutive run is noise; a clean streak is signal. Total CRC count is
/// only the tiebreaker.
pub fn decode_symbols(soft: &[f32], min_blocks: usize) -> DecodeReport {
    // Polarity-resolve a hypothesis' framed blocks and return (streak, corr).
    fn resolve(bits: &[u8], sync: &SyncResult) -> (usize, Vec<u8>) {
        let Some(o) = sync.offset else { return (0, Vec::new()) };
        let mut corr = bits[o..o + sync.nblocks * BLOCK_BITS].to_vec();
        let mut cur_inv = false;
        for i in 0..sync.nblocks {
            if sync.valid[i] {
                cur_inv = sync.inverted[i];
            }
            if cur_inv {
                for b in corr[i * BLOCK_BITS..(i + 1) * BLOCK_BITS].iter_mut() {
                    *b ^= 1;
                }
            }
        }
        (lock_streak(&corr, &sync.valid), corr)
    }
    let mut best: Option<(usize, bool, SyncResult, usize, Vec<u8>)> = None;
    for off in 0..2 {
        for inv in [false, true] {
            let bits = viterbi(&soft[off.min(soft.len())..], inv);
            let sync = frame_sync(&bits, min_blocks);
            let (streak, corr) = resolve(&bits, &sync);
            let score = (streak, sync.npass);
            if best
                .as_ref()
                .map(|b| score > (b.3, b.2.npass))
                .unwrap_or(true)
            {
                best = Some((off, inv, sync, streak, corr));
            }
        }
    }
    let (off, inv, sync, streak, corr) = best.unwrap();
    let mut sync = sync;
    let mut preamble = (0, 0, 0);
    let mut messages = Vec::new();
    if let Some(o) = sync.offset {
        preamble = preamble_phase(&corr, &sync.valid);
        // Lock means a CURRENT streak of consecutive, correctly-rotating,
        // nondegenerate CRC-valid blocks — not scattered passes anywhere in
        // the window (review round 5).
        sync.locked = streak >= min_blocks;
        for i in 0..sync.nblocks {
            if !sync.valid[i] {
                continue;
            }
            let blk = &corr[i * BLOCK_BITS..(i + 1) * BLOCK_BITS];
            if let Some(m) = parse_block(blk) {
                messages.push(DecodedMessage {
                    block_index: i,
                    // bit o + i*250 of the viterbi stream = symbol
                    // off + 2*(o + i*250) of the input buffer
                    sym_pos: off + 2 * (o + i * BLOCK_BITS),
                    message: m,
                });
            }
        }
    } else {
        sync.locked = false;
    }
    DecodeReport {
        sym_offset: off,
        invert_g2: inv,
        sync,
        preamble,
        streak,
        messages,
    }
}

/// Streaming decoder: accumulate 500 sym/s soft symbols and decode the buffer
/// on demand. Cheap to call once per second on a locked SBAS channel; keeps
/// only the most recent `keep` symbols so the frame-sync search stays
/// bounded. The retained window is re-decoded whole on every call, so
/// `decode` reports each message's ABSOLUTE stream position
/// (DecodedMessage::sym_pos) — the caller uses it as an immutable block
/// identity to apply each block exactly once (review round 6).
pub struct Decoder {
    soft: Vec<f32>,
    keep: usize,
    /// Absolute stream position of `soft[0]`: the total symbols retired by
    /// the retention window since creation. Zero with the decoder — a new
    /// decode generation starts a new identity space.
    base: usize,
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder {
    pub fn new() -> Self {
        // 32 s of symbols: comfortably above the ~4 s needed for lock, small
        // enough that frame_sync's 250-offset search stays sub-ms per call.
        Decoder {
            soft: Vec::new(),
            keep: 500 * 32,
            base: 0,
        }
    }

    /// Push soft symbols (`> 0` means symbol bit 0), e.g. from
    /// `symbols_from_prompt` on the channel's 1 ms prompt stream.
    pub fn push_symbols(&mut self, soft: &[f32]) {
        self.soft.extend_from_slice(soft);
        if self.soft.len() > self.keep {
            let drop = self.soft.len() - self.keep;
            self.soft.drain(..drop);
            self.base += drop;
        }
    }

    /// Decode everything buffered so far; message positions
    /// (DecodedMessage::sym_pos) are rebased to absolute stream positions,
    /// so they stay valid as the retention window slides.
    pub fn decode(&self, min_blocks: usize) -> DecodeReport {
        let mut rep = decode_symbols(&self.soft, min_blocks);
        for dm in rep.messages.iter_mut() {
            dm.sym_pos += self.base;
        }
        rep
    }
}

// ---------------------------------------------------------------------------
// tests
// ---------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;

    /// xorshift for reproducible test vectors (crate test idiom).
    struct Rng(u64);
    impl Rng {
        fn new(seed: u64) -> Self {
            Rng(seed)
        }
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
        fn bit(&mut self) -> u8 {
            (self.next() & 1) as u8
        }
        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
        /// standard normal via Box-Muller
        fn gauss(&mut self) -> f64 {
            let u1 = ((self.next() >> 11) as f64 + 0.5) / (1u64 << 53) as f64;
            let u2 = ((self.next() >> 11) as f64 + 0.5) / (1u64 << 53) as f64;
            (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
        }
    }

    /// MSB-first bit packer mirroring sbas_sim.py's BitWriter.
    struct BitWriter {
        bits: Vec<u8>,
    }
    impl BitWriter {
        fn new() -> Self {
            BitWriter { bits: Vec::new() }
        }
        fn u(&mut self, value: u64, width: usize) -> &mut Self {
            for i in (0..width).rev() {
                self.bits.push(((value >> i) & 1) as u8);
            }
            self
        }
        fn s(&mut self, value: i64, width: usize) -> &mut Self {
            self.u((value as u64) & ((1u64 << width) - 1), width)
        }
        fn raw(&mut self, seq: &[u8]) -> &mut Self {
            self.bits.extend(seq.iter().map(|&b| b & 1));
            self
        }
        fn pad(&mut self, n: usize) -> &mut Self {
            self.bits.extend(std::iter::repeat(0).take(n));
            self
        }
    }

    /// Assemble a 250-bit block: preamble, message type, 212-bit payload,
    /// appended CRC-24Q over the first 226 bits.
    fn make_block(preamble: u8, mt: u8, payload: &[u8]) -> Vec<u8> {
        assert_eq!(payload.len(), 212);
        let mut w = BitWriter::new();
        w.u(preamble as u64, 8).u(mt as u64, 6).raw(payload);
        let mut block = w.bits.clone();
        let crc = crc24q(&block);
        for i in (0..24).rev() {
            block.push(((crc >> i) & 1) as u8);
        }
        assert_eq!(block.len(), BLOCK_BITS);
        block
    }

    // -- payload builders mirroring sbas_sim.py --

    fn mt1_payload(rng: &mut Rng, nslots: usize) -> (Vec<u8>, Vec<u8>, u8) {
        let iodp = 2u8;
        let mut mask = vec![0u8; 210];
        let mut slots = Vec::new();
        while slots.len() < nslots {
            let s = (rng.below(37) + 1) as u8; // GPS slots
            if !slots.contains(&s) {
                slots.push(s);
                mask[(s - 1) as usize] = 1;
            }
        }
        slots.sort();
        let mut w = BitWriter::new();
        w.raw(&mask).u(iodp as u64, 2);
        (w.bits, slots, iodp)
    }

    fn mt2to5_payload(rng: &mut Rng) -> (Vec<u8>, u8, u8, [i16; 13], [u8; 13]) {
        let iodf = (rng.below(4)) as u8;
        let iodp = 2u8;
        let mut prc = [0i16; 13];
        let mut udrei = [0u8; 13];
        for v in prc.iter_mut() {
            *v = rng.below(4096) as i16 - 2048;
        }
        for v in udrei.iter_mut() {
            *v = rng.below(14) as u8;
        }
        let mut w = BitWriter::new();
        w.u(iodf as u64, 2).u(iodp as u64, 2);
        for &v in prc.iter() {
            w.s(v as i64, 12);
        }
        for &v in udrei.iter() {
            w.u(v as u64, 4);
        }
        (w.bits, iodf, iodp, prc, udrei)
    }

    fn mt6_payload(rng: &mut Rng) -> (Vec<u8>, [u8; 4], Vec<u8>) {
        let mut iodf = [0u8; 4];
        for v in iodf.iter_mut() {
            *v = rng.below(4) as u8;
        }
        let udrei: Vec<u8> = (0..51).map(|_| rng.below(14) as u8).collect();
        let mut w = BitWriter::new();
        for &v in iodf.iter() {
            w.u(v as u64, 2);
        }
        for &v in udrei.iter() {
            w.u(v as u64, 4);
        }
        (w.bits, iodf, udrei)
    }

    fn mt7_payload(rng: &mut Rng) -> (Vec<u8>, u8, u8, Vec<u8>) {
        let latency = rng.below(16) as u8;
        let iodp = 2u8;
        let ai: Vec<u8> = (0..51).map(|_| rng.below(16) as u8).collect();
        let mut w = BitWriter::new();
        w.u(latency as u64, 4).u(iodp as u64, 2).pad(2);
        for &v in ai.iter() {
            w.u(v as u64, 4);
        }
        (w.bits, latency, iodp, ai)
    }

    fn lt_half_v0(rng: &mut Rng) -> (Vec<u8>, Vec<LongTermSat>, u8) {
        let iodp = 2u8;
        let sats: Vec<LongTermSat> = (0..2)
            .map(|_| LongTermSat {
                mask: (rng.below(50) + 1) as u8,
                iod: rng.below(256) as u8,
                dx: rng.below(512) as i16 - 256,
                dy: rng.below(512) as i16 - 256,
                dz: rng.below(512) as i16 - 256,
                daf0: rng.below(1024) as i16 - 512,
                ddx: None,
                ddy: None,
                ddz: None,
                daf1: None,
            })
            .collect();
        let mut w = BitWriter::new();
        w.u(0, 1);
        for s in sats.iter() {
            w.u(s.mask as u64, 6)
                .u(s.iod as u64, 8)
                .s(s.dx as i64, 9)
                .s(s.dy as i64, 9)
                .s(s.dz as i64, 9)
                .s(s.daf0 as i64, 10);
        }
        w.u(iodp as u64, 2).pad(1);
        assert_eq!(w.bits.len(), 106);
        (w.bits, sats, iodp)
    }

    fn lt_half_v1(rng: &mut Rng) -> (Vec<u8>, LongTermSat, u32, u8) {
        let iodp = 2u8;
        let s = LongTermSat {
            mask: (rng.below(50) + 1) as u8,
            iod: rng.below(256) as u8,
            dx: rng.below(2048) as i16 - 1024,
            dy: rng.below(2048) as i16 - 1024,
            dz: rng.below(2048) as i16 - 1024,
            daf0: rng.below(2048) as i16 - 1024,
            ddx: Some(rng.below(256) as i16 - 128),
            ddy: Some(rng.below(256) as i16 - 128),
            ddz: Some(rng.below(256) as i16 - 128),
            daf1: Some(rng.below(256) as i16 - 128),
        };
        let t_lt = rng.below(8192) as u32;
        let mut w = BitWriter::new();
        w.u(1, 1)
            .u(s.mask as u64, 6)
            .u(s.iod as u64, 8)
            .s(s.dx as i64, 11)
            .s(s.dy as i64, 11)
            .s(s.dz as i64, 11)
            .s(s.daf0 as i64, 11)
            .s(s.ddx.unwrap() as i64, 8)
            .s(s.ddy.unwrap() as i64, 8)
            .s(s.ddz.unwrap() as i64, 8)
            .s(s.daf1.unwrap() as i64, 8)
            .u(t_lt as u64, 13)
            .u(iodp as u64, 2);
        assert_eq!(w.bits.len(), 106);
        (w.bits, s, t_lt * 16, iodp)
    }

    fn mt18_payload(rng: &mut Rng) -> (Vec<u8>, u8, u8, u8, Vec<u16>) {
        let (nbands, band, iodi) = (9u8, 3u8, 1u8);
        let mut mask = vec![0u8; 201];
        let mut igps = Vec::new();
        while igps.len() < 60 {
            let i = rng.below(201) as u16;
            if !igps.contains(&i) {
                igps.push(i);
                mask[i as usize] = 1;
            }
        }
        let mut want: Vec<u16> = igps.iter().map(|&i| i + 1).collect();
        want.sort();
        let mut w = BitWriter::new();
        w.u(nbands as u64, 4).u(band as u64, 4).u(iodi as u64, 2);
        w.raw(&mask).pad(1);
        assert_eq!(w.bits.len(), 212);
        (w.bits, nbands, band, iodi, want)
    }

    fn mt26_payload(rng: &mut Rng) -> (Vec<u8>, u8, u8, u8, [(u16, u8); 15]) {
        let (band, block_id, iodi) = (3u8, 7u8, 1u8);
        let mut igps = [(0u16, 0u8); 15];
        for v in igps.iter_mut() {
            *v = (rng.below(512) as u16, rng.below(16) as u8);
        }
        let mut w = BitWriter::new();
        w.u(band as u64, 4).u(block_id as u64, 4);
        for &(d, g) in igps.iter() {
            w.u(d as u64, 9).u(g as u64, 4);
        }
        w.u(iodi as u64, 2).pad(7);
        assert_eq!(w.bits.len(), 212);
        (w.bits, band, block_id, iodi, igps)
    }

    /// A plausible WAAS-like 1 Hz schedule (same set as sbas_sim.py).
    const SCHEDULE: [u8; 28] = [
        1, 2, 3, 4, 5, 25, 18, 26, 7, 9, 24, 17, 6, 0, 2, 3, 4, 5, 25, 26, 1,
        18, 24, 25, 26, 6, 9, 7,
    ];

    /// Build `n` blocks with correct rotating preambles and CRCs; payloads
    /// are random but well-formed where the parser round trip checks them.
    fn build_blocks(rng: &mut Rng, n: usize) -> Vec<Vec<u8>> {
        (0..n)
            .map(|i| {
                let mt = SCHEDULE[i % SCHEDULE.len()];
                let payload = match mt {
                    0 => vec![0u8; 212],
                    1 => mt1_payload(rng, 24).0,
                    2..=5 => mt2to5_payload(rng).0,
                    6 => mt6_payload(rng).0,
                    7 => mt7_payload(rng).0,
                    9 => {
                        let mut w = BitWriter::new();
                        w.u(rng.below(256), 8)
                            .u(rng.below(8192), 13)
                            .u(rng.below(16), 4)
                            .s(rng.below(1 << 29) as i64 - (1 << 28), 30)
                            .s(rng.below(1 << 29) as i64 - (1 << 28), 30)
                            .s(rng.below(1 << 24) as i64 - (1 << 23), 25)
                            .s(rng.below(1000) as i64 - 500, 17)
                            .s(rng.below(1000) as i64 - 500, 17)
                            .s(rng.below(1000) as i64 - 500, 18)
                            .s(rng.below(200) as i64 - 100, 10)
                            .s(rng.below(200) as i64 - 100, 10)
                            .s(rng.below(200) as i64 - 100, 10)
                            .s(rng.below(4096) as i64 - 2048, 12)
                            .s(rng.below(256) as i64 - 128, 8);
                        w.bits
                    }
                    17 => {
                        let mut w = BitWriter::new();
                        for _ in 0..3 {
                            w.pad(2)
                                .u(131 + rng.below(3), 8)
                                .u(rng.below(256), 8)
                                .s(rng.below(32000) as i64 - 16000, 15)
                                .s(rng.below(32000) as i64 - 16000, 15)
                                .s(rng.below(512) as i64 - 256, 9)
                                .s(rng.below(8) as i64 - 4, 3)
                                .s(rng.below(8) as i64 - 4, 3)
                                .s(rng.below(16) as i64 - 8, 4);
                        }
                        w.u(rng.below(2048), 11);
                        w.bits
                    }
                    18 => mt18_payload(rng).0,
                    24 => {
                        let (half, _, _) = lt_half_v0(rng);
                        let mut w = BitWriter::new();
                        for _ in 0..6 {
                            w.s(rng.below(4096) as i64 - 2048, 12);
                        }
                        for _ in 0..6 {
                            w.u(rng.below(14), 4);
                        }
                        w.u(2, 2).u(0, 2).u(rng.below(4), 2).pad(4).raw(&half);
                        w.bits
                    }
                    25 => {
                        let (ha, _, _) = lt_half_v0(rng);
                        let (hb, _, _, _) = lt_half_v1(rng);
                        let mut w = BitWriter::new();
                        w.raw(&ha).raw(&hb);
                        w.bits
                    }
                    26 => mt26_payload(rng).0,
                    other => panic!("bad schedule mt {other}"),
                };
                assert_eq!(payload.len(), 212, "mt {mt}");
                make_block(PREAMBLES[i % 3], mt, &payload)
            })
            .collect()
    }

    // -----------------------------------------------------------------
    // CRC-24Q
    // -----------------------------------------------------------------

    #[test]
    fn crc24q_known_vectors() {
        // Qualcomm standard check: CRC-24Q("123456789") = 0xCDE703.
        let bits: Vec<u8> = b"123456789"
            .iter()
            .flat_map(|&byte| (0..8).rev().map(move |i| (byte >> i) & 1))
            .collect();
        assert_eq!(crc24q(&bits), 0xCDE703);
        // Degenerate blocks: CRC of all zeros is 0 (hence the weight guard);
        // CRC of all ones is NOT 0, so the CRC resolves the 180 deg polarity.
        assert_eq!(crc24q(&vec![0u8; 250]), 0);
        assert_eq!(crc24q(&vec![1u8; 250]), 0xEF339D);
    }

    #[test]
    fn block_crc_ok_roundtrip() {
        let mut rng = Rng::new(0x5BA5);
        let payload: Vec<u8> = (0..212).map(|_| rng.bit()).collect();
        let block = make_block(PREAMBLES[0], 9, &payload);
        assert!(block_crc_ok(&block));
        // single-bit error anywhere breaks the CRC
        for pos in [0, 100, 249] {
            let mut bad = block.clone();
            bad[pos] ^= 1;
            assert!(!block_crc_ok(&bad), "bit flip at {pos} must fail CRC");
        }
        // degenerate blocks are rejected despite the zero syndrome
        assert!(!block_crc_ok(&vec![0u8; 250]));
    }

    // -----------------------------------------------------------------
    // convolutional code / Viterbi
    // -----------------------------------------------------------------

    fn soft_of(sym: &[u8]) -> Vec<f32> {
        sym.iter().map(|&s| 1.0 - 2.0 * s as f32).collect()
    }

    #[test]
    fn viterbi_noiseless_loopback() {
        let mut rng = Rng::new(42);
        let bits: Vec<u8> = (0..5000).map(|_| rng.bit()).collect();
        let sym = conv_encode(&bits, false, 0);
        let dec = viterbi(&soft_of(&sym), false);
        assert_eq!(dec.len(), bits.len());
        // unknown start state, as on air: exact after a few constraint lengths
        assert_eq!(dec[40..], bits[40..]);
        // wrong G2 convention is distinguishable
        assert_ne!(viterbi(&soft_of(&sym), true)[40..], bits[40..]);
    }

    #[test]
    fn viterbi_polarity_complement() {
        // Both generators have odd weight, so viterbi(-soft) is the exact
        // complement of viterbi(soft) — the 180 deg slip property.
        let mut rng = Rng::new(7);
        let bits: Vec<u8> = (0..2000).map(|_| rng.bit()).collect();
        let sym = conv_encode(&bits, false, 0);
        let soft = soft_of(&sym);
        let neg: Vec<f32> = soft.iter().map(|&v| -v).collect();
        let a = viterbi(&soft, false);
        let b = viterbi(&neg, false);
        for i in 40..2000 {
            assert_eq!(b[i], a[i] ^ 1, "bit {i}");
        }
    }

    #[test]
    fn viterbi_corrects_noise() {
        let mut rng = Rng::new(99);
        let bits: Vec<u8> = (0..5000).map(|_| rng.bit()).collect();
        let sym = conv_encode(&bits, false, 0);
        let soft = soft_of(&sym);
        // Es/N0 = 4 dB
        let es = 10f64.powf(0.4);
        let sigma = (1.0 / (2.0 * es)).sqrt();
        let noisy: Vec<f32> = soft
            .iter()
            .map(|&s| (s as f64 + sigma * rng.gauss()) as f32)
            .collect();
        let dec = viterbi(&noisy, false);
        let raw_ser = noisy
            .iter()
            .zip(soft.iter())
            .filter(|&(&n, &s)| n.signum() != s.signum())
            .count() as f64
            / soft.len() as f64;
        let ber = dec[40..]
            .iter()
            .zip(bits[40..].iter())
            .filter(|&(&d, &b)| d != b)
            .count() as f64
            / (bits.len() - 40) as f64;
        assert!(ber < 0.01, "coded BER {ber} too high");
        assert!(ber < raw_ser, "code must beat raw SER {raw_ser}");
    }

    // -----------------------------------------------------------------
    // frame sync
    // -----------------------------------------------------------------

    #[test]
    fn frame_sync_finds_true_offset() {
        let mut rng = Rng::new(5);
        let blocks = build_blocks(&mut rng, 30);
        let mut stream: Vec<u8> = (0..137).map(|_| rng.bit()).collect();
        for b in blocks.iter() {
            stream.extend_from_slice(b);
        }
        let fs = frame_sync(&stream, 3);
        assert_eq!(fs.offset, Some(137));
        assert_eq!(fs.npass, 30, "{fs:?}");
        assert!(fs.locked);
        // preamble rotation must check out at exactly one phase
        let flat: Vec<u8> = blocks.concat();
        let valid = vec![true; 30];
        let (ph, n, nvalid) = preamble_phase(&flat, &valid);
        assert_eq!(nvalid, 30);
        assert_eq!((ph, n), (0, 30));
    }

    #[test]
    fn frame_sync_rides_mid_stream_polarity_slip() {
        let mut rng = Rng::new(11);
        let blocks = build_blocks(&mut rng, 30);
        let mut stream: Vec<u8> = (0..137).map(|_| rng.bit()).collect();
        for b in blocks.iter() {
            stream.extend_from_slice(b);
        }
        // Costas slip inside block 12: bits complement from there on
        let at = 137 + 250 * 12 + 60;
        for b in stream[at..].iter_mut() {
            *b ^= 1;
        }
        let fs = frame_sync(&stream, 3);
        assert_eq!(fs.offset, Some(137));
        assert!(fs.npass >= 29, "npass {} of 30", fs.npass);
        assert!(fs.locked);
        // blocks after the slip pass only inverted
        assert!(fs.inverted[13]);
        assert!(!fs.inverted[11]);
    }

    #[test]
    fn frame_sync_rejects_random_bits() {
        let mut rng = Rng::new(1234);
        let noise: Vec<u8> = (0..250 * 400).map(|_| rng.bit()).collect();
        let fs = frame_sync(&noise, 3);
        assert!(!fs.locked, "npass {} on pure noise", fs.npass);
    }

    #[test]
    fn preamble_rotation_is_enforced() {
        let mut rng = Rng::new(77);
        let mut blocks = build_blocks(&mut rng, 12);
        // break the rotation: stamp block 5 with block 3's preamble value
        let wrong = PREAMBLES[0];
        for i in 0..8 {
            blocks[5][i] = (wrong >> (7 - i)) & 1;
        }
        // CRC no longer matches after mangling, so rebuild it properly: use a
        // block whose DATA is fine but whose preamble is off-rotation.
        let mut fixed = blocks[5].clone();
        let crc = crc24q(&fixed[..226]);
        for i in 0..24 {
            fixed[226 + i] = ((crc >> (23 - i)) & 1) as u8;
        }
        blocks[5] = fixed;
        let flat = blocks.concat();
        let valid = vec![true; 12];
        let (_, n, nvalid) = preamble_phase(&flat, &valid);
        assert_eq!(nvalid, 12);
        assert_eq!(n, 11, "the off-rotation preamble must not match");
    }

    // -----------------------------------------------------------------
    // parsers
    // -----------------------------------------------------------------

    #[test]
    fn parse_roundtrip_all_types() {
        let mut rng = Rng::new(2026);

        // MT0 (test mode): preamble + zeros can be light; make sure the block
        // passes the weight guard for this seed, else skip the assertion.
        let b0 = make_block(PREAMBLES[0], 0, &vec![0u8; 212]);
        if block_crc_ok(&b0) {
            assert_eq!(parse_block(&b0), Some(Message::DontUse));
        }

        // MT1
        let (p, slots, iodp) = mt1_payload(&mut rng, 24);
        let b = make_block(PREAMBLES[1], 1, &p);
        match parse_block(&b) {
            Some(Message::PrnMask { slots: s, iodp: i }) => {
                assert_eq!(s, slots);
                assert_eq!(i, iodp);
            }
            other => panic!("MT1 misparse: {other:?}"),
        }

        // MT2..5
        for mt in 2..=5u8 {
            let (p, iodf, iodp, prc, udrei) = mt2to5_payload(&mut rng);
            let b = make_block(PREAMBLES[mt as usize % 3], mt, &p);
            match parse_block(&b) {
                Some(Message::Fast {
                    iodf: f,
                    iodp: i,
                    first_slot,
                    prc: pc,
                    udrei: u,
                }) => {
                    assert_eq!((f, i), (iodf, iodp));
                    assert_eq!(first_slot, (mt - 2) * 13 + 1);
                    assert_eq!(pc, prc);
                    assert_eq!(u, udrei);
                    assert_eq!(Message::Fast {
                        iodf: f,
                        iodp: i,
                        first_slot,
                        prc: pc,
                        udrei: u,
                    }
                    .mt(), mt);
                }
                other => panic!("MT{mt} misparse: {other:?}"),
            }
        }

        // MT6
        let (p, iodf, udrei) = mt6_payload(&mut rng);
        let b = make_block(PREAMBLES[0], 6, &p);
        match parse_block(&b) {
            Some(Message::Integrity { iodf: f, udrei: u }) => {
                assert_eq!(f, iodf);
                assert_eq!(u, udrei);
            }
            other => panic!("MT6 misparse: {other:?}"),
        }

        // MT7
        let (p, latency, iodp, ai) = mt7_payload(&mut rng);
        let b = make_block(PREAMBLES[1], 7, &p);
        match parse_block(&b) {
            Some(Message::FastDegradation {
                latency: l,
                iodp: i,
                ai: a,
            }) => {
                assert_eq!((l, i), (latency, iodp));
                assert_eq!(a, ai);
            }
            other => panic!("MT7 misparse: {other:?}"),
        }

        // MT18
        let (p, nbands, band, iodi, igps) = mt18_payload(&mut rng);
        let b = make_block(PREAMBLES[2], 18, &p);
        match parse_block(&b) {
            Some(Message::IonoMask {
                nbands: n,
                band: bd,
                iodi: io,
                igps: g,
            }) => {
                assert_eq!((n, bd, io), (nbands, band, iodi));
                assert_eq!(g, igps);
            }
            other => panic!("MT18 misparse: {other:?}"),
        }

        // MT25: velocity code 0 half + velocity code 1 half
        let (ha, sats0, iodp0) = lt_half_v0(&mut rng);
        let (hb, sat1, t_lt, iodp1) = lt_half_v1(&mut rng);
        let mut w = BitWriter::new();
        w.raw(&ha).raw(&hb);
        let b = make_block(PREAMBLES[0], 25, &w.bits);
        match parse_block(&b) {
            Some(Message::LongTerm { a, b: bb }) => {
                assert_eq!(a.velocity_code, 0);
                assert_eq!(a.sats, sats0);
                assert_eq!(a.iodp, iodp0);
                assert_eq!(bb.velocity_code, 1);
                assert_eq!(bb.sats, vec![sat1]);
                assert_eq!(bb.t_lt_s, Some(t_lt));
                assert_eq!(bb.iodp, iodp1);
            }
            other => panic!("MT25 misparse: {other:?}"),
        }

        // MT26
        let (p, band, block_id, iodi, igps) = mt26_payload(&mut rng);
        let b = make_block(PREAMBLES[1], 26, &p);
        match parse_block(&b) {
            Some(Message::IonoDelay {
                band: bd,
                block_id: bi,
                iodi: io,
                igps: g,
            }) => {
                assert_eq!((bd, bi, io), (band, block_id, iodi));
                assert_eq!(g, igps);
            }
            other => panic!("MT26 misparse: {other:?}"),
        }
    }

    // -----------------------------------------------------------------
    // full back end
    // -----------------------------------------------------------------

    /// Encode a block stream into a continuous soft-symbol stream (the
    /// encoder is NOT reset per block), with optional noise and a global
    /// polarity flip.
    fn stream_of(blocks: &[Vec<u8>], rng: &mut Rng, es_n0_db: f64, flip: bool) -> Vec<f32> {
        let bits: Vec<u8> = blocks.concat();
        let sym = conv_encode(&bits, false, 0);
        let es = 10f64.powf(es_n0_db / 10.0);
        let sigma = (1.0 / (2.0 * es)).sqrt();
        let sgn = if flip { -1.0 } else { 1.0 };
        sym.iter()
            .map(|&s| {
                (sgn * (1.0 - 2.0 * s as f64) + sigma * rng.gauss()) as f32
            })
            .collect()
    }

    #[test]
    fn decode_symbols_end_to_end() {
        let mut rng = Rng::new(31415);
        let blocks = build_blocks(&mut rng, 30);
        let soft = stream_of(&blocks, &mut rng, 8.0, false);
        let rep = decode_symbols(&soft, 3);
        assert!(rep.sync.locked, "{:?}", rep.sync);
        assert_eq!(rep.sync.npass, 30, "{:?}", rep.sync);
        assert_eq!(rep.preamble.1, 30, "preamble rotation");
        assert_eq!(rep.messages.len(), 30);
        // the schedule's message types must come back in order
        for (i, dm) in rep.messages.iter().enumerate() {
            assert_eq!(dm.block_index, i);
            assert_eq!(dm.message.mt(), SCHEDULE[i % SCHEDULE.len()]);
        }
    }

    #[test]
    fn decode_symbols_resolves_global_polarity_flip() {
        let mut rng = Rng::new(2718);
        let blocks = build_blocks(&mut rng, 20);
        let soft = stream_of(&blocks, &mut rng, 8.0, true);
        let rep = decode_symbols(&soft, 3);
        assert!(rep.sync.locked);
        assert_eq!(rep.sync.npass, 20);
        // every block needed the inverted CRC
        assert!(rep.sync.inverted.iter().all(|&v| v));
        assert_eq!(rep.messages.len(), 20);
        assert_eq!(rep.messages[0].message.mt(), 1);
    }

    #[test]
    fn decoder_streaming_api() {
        let mut rng = Rng::new(555);
        let blocks = build_blocks(&mut rng, 12);
        let soft = stream_of(&blocks, &mut rng, 10.0, false);
        let mut dec = Decoder::new();
        // feed in 1 s (500 symbol) chunks, as a live integration would
        for chunk in soft.chunks(500) {
            dec.push_symbols(chunk);
        }
        let rep = dec.decode(3);
        assert!(rep.sync.locked);
        assert_eq!(rep.messages.len(), 12);
        assert_eq!(rep.preamble.1, 12);
    }

    /// Review round 6: block identities are ABSOLUTE stream positions —
    /// stable across re-decodes of the retained window and across the
    /// retention drain, so the consumer can apply each block exactly once.
    #[test]
    fn decoder_block_identity_survives_window_churn() {
        let mut rng = Rng::new(86);
        let blocks = build_blocks(&mut rng, 40);
        // 20 dB: the identity mechanics don't need noise realism, and an
        // essentially clean stream keeps the block accounting exact
        let soft = stream_of(&blocks, &mut rng, 20.0, false);
        let mut dec = Decoder::new();
        // feed the first 12 blocks in 1 s chunks, as live does
        for chunk in soft[..12 * 500].chunks(500) {
            dec.push_symbols(chunk);
        }
        let rep = dec.decode(3);
        assert!(rep.sync.locked);
        let ids: Vec<usize> = rep.messages.iter().map(|m| m.sym_pos).collect();
        assert_eq!(ids.len(), 12);
        // the stream starts on a block boundary: block i starts at symbol
        // 500*i, and the identity is that absolute position
        for (i, &id) in ids.iter().enumerate() {
            assert_eq!(id, 500 * i, "block {i}");
        }
        // re-decoding the same window must not shift the identities
        let again: Vec<usize> = dec.decode(3).messages.iter().map(|m| m.sym_pos).collect();
        assert_eq!(again, ids, "re-decode must not shift identities");
        // push the remaining 28 blocks: 20000 symbols total > 16000 keep,
        // so the window drains 4000 — identities must survive the slide
        for chunk in soft[12 * 500..].chunks(500) {
            dec.push_symbols(chunk);
        }
        let rep = dec.decode(3);
        let ids: Vec<usize> = rep.messages.iter().map(|m| m.sym_pos).collect();
        assert_eq!(ids.first(), Some(&4000), "oldest retained block is #8");
        assert_eq!(ids.last(), Some(&19500), "newest block is #39");
        // a pre-drain block still in the window keeps its identity
        assert!(ids.contains(&5500), "block 11 survived the drain");
    }

    #[test]
    fn symbols_from_prompt_picks_right_parity() {
        // 2 ms symbols cut into 1 ms prompts at both possible parities; the
        // combiner must pick the one that doesn't straddle transitions.
        let mut rng = Rng::new(31);
        let bits: Vec<u8> = (0..400).map(|_| rng.bit()).collect();
        let sym = conv_encode(&bits, false, 0);
        let sign: Vec<f64> = sym.iter().map(|&s| 1.0 - 2.0 * s as f64).collect();
        for par in 0..2 {
            // prompts: symbol value at its own two 1 ms slots; the straddling
            // slot of the WRONG parity gets the average of two symbols.
            let mut prompts = vec![0.0f64; 2 * sign.len() + par];
            for (i, &s) in sign.iter().enumerate() {
                prompts[par + 2 * i] = s;
                prompts[par + 2 * i + 1] = s;
            }
            let (soft, got) = symbols_from_prompt(&prompts);
            assert_eq!(got, par);
            let dec = viterbi(&soft, false);
            assert_eq!(dec[40..], bits[40..]);
        }
    }

    /// Review round 5: scattered CRC-valid blocks (no consecutiveness, no
    /// rotation) must NOT lock; three consecutive correctly-rotating blocks
    /// lock; a rotation break resets the streak.
    #[test]
    fn lock_streak_requires_consecutive_rotation() {
        let payload = vec![0xA5u8; 212];
        let good = |i: usize| make_block(PREAMBLES[i % 3], 63, &payload);
        // scattered: three valid blocks separated by garbage — never locks
        let mut bits = Vec::new();
        let mut valid = Vec::new();
        for (i, &g) in [true, false, true, false, true].iter().enumerate() {
            if g {
                bits.extend_from_slice(&good(i));
            } else {
                bits.extend_from_slice(&vec![0u8; BLOCK_BITS]);
            }
            valid.push(g);
        }
        assert!(lock_streak(&bits, &valid) < 3, "scattered blocks must not streak");
        // consecutive, rotating: streaks to 3
        let mut bits = Vec::new();
        let mut valid = Vec::new();
        for i in 0..3 {
            bits.extend_from_slice(&good(i));
            valid.push(true);
        }
        assert_eq!(lock_streak(&bits, &valid), 3);
        // rotation break in the middle kills the run
        let mut bits = Vec::new();
        let mut valid = Vec::new();
        bits.extend_from_slice(&good(0));
        valid.push(true);
        bits.extend_from_slice(&good(2)); // wrong rotation member
        valid.push(true);
        bits.extend_from_slice(&good(2)); // correct successor of 0xC6 is 0x53? no: good(2) is 0xC6 again
        valid.push(true);
        assert!(lock_streak(&bits, &valid) < 3, "rotation break must reset");
    }

    /// Fast-correction harvest: 0.125 m scale, UDREI >= 14 excluded. DO-229
    /// sign convention: PRC is ADDED to the measured pseudorange (verified
    /// by the scale/sign assertions here — a sign flip would show as a
    /// doubled error downstream, the classic SBAS application bug).
    #[test]
    fn fast_corrections_scale_sign_and_gates() {
        // full GPS mask: the ordinal mapping degenerates to slot = ordinal
        let mask: Vec<u8> = (1..=37).collect();
        let mut prc = [0i16; 13];
        let mut udrei = [0u8; 13];
        prc[0] = 16; // +2.0 m
        prc[1] = -8; // -1.0 m
        udrei[1] = 3;
        udrei[2] = 15; // don't use
        let rows = fast_corrections(&mask, 2, 2, 1, &prc, &udrei);
        assert_eq!(rows[0], (1, 2.0, 0)); // ordinal 1 -> PRN 1, +2.0 m (not -2.0)
        assert_eq!(rows[1], (2, -1.0, 3));
        assert_eq!(rows.len(), 12, "only the UDREI-15 row is excluded");
        // ordinals beyond the mask length yield nothing
        let rows = fast_corrections(&mask, 2, 2, 40, &prc, &udrei);
        assert!(rows.is_empty());
    }

    /// MT2-5 entries address satellites by ORDINAL through the set bits of
    /// the MT1 mask (DO-229D A.4.4.3: "Message Type 2 contains the data
    /// sets for the first 13 satellites designated in the PRN mask"), not
    /// by absolute slot — with gaps in the mask the same entry lands on a
    /// different PRN.
    #[test]
    fn fast_corrections_ordinal_through_gapped_mask() {
        // slots 4 and 6 are not monitored: the mask has gaps
        let mask = vec![1u8, 2, 3, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15];
        let mut prc = [0i16; 13];
        let udrei = [0u8; 13];
        for (k, v) in prc.iter_mut().enumerate() {
            *v = (k as i16) + 1; // distinct PRC per entry
        }
        let rows = fast_corrections(&mask, 2, 2, 1, &prc, &udrei);
        assert_eq!(rows.len(), 13);
        for (k, &(prn, prc_m, _)) in rows.iter().enumerate() {
            assert_eq!(prn, mask[k], "ordinal {} must map through the mask", k + 1);
            assert_eq!(prc_m, (k as f64 + 1.0) * 0.125);
        }
        // entry 4 lands on PRN 5, not 4: the gap at slot 4 shifts it
        assert_eq!(rows[3].0, 5);
        // MT3 (ordinals 14-26) on a 13-satellite mask is empty
        assert!(fast_corrections(&mask, 2, 2, 14, &prc, &udrei).is_empty());
    }

    /// Fast corrections are valid only while the message IODP matches the
    /// IODP of the MT1 mask in use (DO-229D A.4.4.2/A.4.4.3).
    #[test]
    fn fast_corrections_iodp_mismatch_yields_nothing() {
        let mask = vec![1u8, 2, 3, 5, 7];
        let prc = [8i16; 13];
        let udrei = [0u8; 13];
        assert!(fast_corrections(&mask, 2, 3, 1, &prc, &udrei).is_empty());
        assert!(
            fast_corrections(&[], 2, 2, 1, &prc, &udrei).is_empty(),
            "no mask, no corrections"
        );
        assert_eq!(fast_corrections(&mask, 2, 2, 1, &prc, &udrei).len(), 5);
    }

    /// Mask slots beyond 37 (GLONASS 38-61, GEO 120-138) are never GPS
    /// PRNs, even when their ordinal falls inside the 13-entry range.
    #[test]
    fn fast_corrections_non_gps_slots_excluded() {
        let mask = vec![1u8, 2, 38, 39, 120, 3, 4];
        let prc = [8i16; 13];
        let udrei = [0u8; 13];
        let rows = fast_corrections(&mask, 0, 0, 1, &prc, &udrei);
        let prns: Vec<u8> = rows.iter().map(|r| r.0).collect();
        assert_eq!(prns, vec![1, 2, 3, 4]);
    }

    /// fast_rows keeps the UDREI >= 14 rows that fast_corrections drops:
    /// the caller needs them to EVICT a cached usable correction when a
    /// fresh don't-use arrives (DO-229D A.4.4.3 UDRE table).
    #[test]
    fn fast_rows_includes_dont_use_rows() {
        let mask = vec![1u8, 2, 3, 5, 7];
        let prc = [8i16; 13];
        let mut udrei = [0u8; 13];
        udrei[1] = 14; // don't use
        udrei[3] = 15; // not monitored
        let rows = fast_rows(&mask, 2, 2, 1, &prc, &udrei);
        assert_eq!(rows.len(), 5, "only ordinals beyond the mask drop out");
        assert_eq!(rows[1], (2, 1.0, 14));
        assert_eq!(rows[3], (5, 1.0, 15));
        // the IODP gate still applies to the unfiltered view
        assert!(fast_rows(&mask, 2, 3, 1, &prc, &udrei).is_empty());
        // and fast_corrections remains the filtered view of the same rows
        let usable = fast_corrections(&mask, 2, 2, 1, &prc, &udrei);
        assert_eq!(usable.iter().map(|r| r.0).collect::<Vec<_>>(), vec![1, 3, 7]);
    }

    /// lt_corrections: raw->physical scaling and slot filtering. A sign or
    /// scale slip here silently corrupts satellite position/clock in the
    /// solver (same failure class as the PRC sign bug the fast-corr test
    /// guards).
    #[test]
    fn lt_corrections_scale_and_slots() {
        // full GPS mask: ordinal = slot
        let mask: Vec<u8> = (1..=37).collect();
        let half = LongTermHalf {
            velocity_code: 0,
            sats: vec![
                LongTermSat {
                    mask: 5,
                    iod: 42,
                    dx: 8,   // +1.0 m
                    dy: -16, // -2.0 m
                    dz: 0,
                    daf0: 1024, // 1024 * 2^-31 s
                    ddx: None,
                    ddy: None,
                    ddz: None,
                    daf1: None,
                },
                LongTermSat {
                    mask: 40, // ordinal beyond the mask — skipped
                    iod: 0,
                    dx: 100,
                    dy: 100,
                    dz: 100,
                    daf0: 100,
                    ddx: None,
                    ddy: None,
                    ddz: None,
                    daf1: None,
                },
            ],
            t_lt_s: None,
            iodp: 0,
        };
        let rows = lt_corrections(&mask, 0, &half);
        assert_eq!(rows.len(), 1);
        let r = &rows[0];
        assert_eq!(r.prn, 5);
        assert_eq!(r.dx, 1.0);
        assert_eq!(r.dy, -2.0);
        assert_eq!(r.dz, 0.0);
        assert!((r.daf0 - 1024.0 * 2.0f64.powi(-31)).abs() < 1e-15);
        assert_eq!(r.iod, 42, "the IOD must ride along for the ephemeris gate");
        // velocity-code-0 rows carry no rates and no t_lt (they apply as
        // constants — A.4.4.7: t of applicability = transmission time)
        assert_eq!((r.ddx, r.ddy, r.ddz, r.daf1), (None, None, None, None));
        assert_eq!(r.t_lt_s, None);
    }

    /// LT half messages address satellites by ordinal through the mask
    /// (DO-229D A.4.4.7: "The PRN Mask No. is the sequence number of the
    /// bits set in the 210 bit mask (that is, between 1 and 51)") and are
    /// gated on the mask IODP — for both velocity codes.
    #[test]
    fn lt_corrections_ordinal_iodp_and_vc_gate() {
        let sat = |mask_no: u8, iod: u8| LongTermSat {
            mask: mask_no,
            iod,
            dx: 8,
            dy: 0,
            dz: 0,
            daf0: 0,
            ddx: None,
            ddy: None,
            ddz: None,
            daf1: None,
        };
        let mask = vec![1u8, 2, 5, 7, 9];
        let half = LongTermHalf {
            velocity_code: 0,
            // ordinal 3 -> slot 5; 0 = "no satellite" (Table A-10 Note 2);
            // 6 is beyond the mask
            sats: vec![sat(3, 200), sat(0, 1), sat(6, 2)],
            t_lt_s: None,
            iodp: 1,
        };
        let rows = lt_corrections(&mask, 1, &half);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].prn, 5);
        assert_eq!(rows[0].iod, 200);
        // IODP mismatch: the corrections belong to another mask generation
        assert!(lt_corrections(&mask, 2, &half).is_empty());
        // non-GPS slots (GLONASS 38-61) are excluded
        let glo = LongTermHalf {
            velocity_code: 0,
            sats: vec![sat(1, 3)],
            t_lt_s: None,
            iodp: 0,
        };
        assert!(lt_corrections(&[38u8], 0, &glo).is_empty());
        // velocity code 1 (what WAAS broadcasts in practice): the row is
        // carried WITH its rates and t_lt for solve-time propagation
        // (A.4.4.7 eq. A-18/A-19), not skipped
        let vc1 = LongTermHalf {
            velocity_code: 1,
            sats: vec![LongTermSat {
                ddx: Some(16),
                ddy: Some(-8),
                ddz: Some(0),
                daf1: Some(32),
                ..sat(1, 7)
            }],
            t_lt_s: Some(16),
            iodp: 1,
        };
        let rows = lt_corrections(&mask, 1, &vc1);
        assert_eq!(rows.len(), 1, "vc=1 halves must propagate, not vanish");
        assert_eq!(rows[0].prn, 1);
        assert_eq!(rows[0].iod, 7);
        assert_eq!(rows[0].t_lt_s, Some(16));
        assert_eq!(rows[0].ddx, Some(16.0 * 2.0f64.powi(-11)));
        assert_eq!(rows[0].ddy, Some(-8.0 * 2.0f64.powi(-11)));
        // the IODP gate applies to vc=1 halves too
        assert!(lt_corrections(&mask, 2, &vc1).is_empty());
    }

    /// REGRESSION (dead-LT finding): WAAS broadcasts velocity-code-1 MT25
    /// halves in practice; skipping them under a "rates not propagated"
    /// premise left n_lt_corr = 0 in every live solve. A vc=1 half must
    /// yield a row — DO-229D A.4.4.7 eq. (A-18)/(A-19) define the
    /// propagation the application performs at solve time.
    #[test]
    fn lt_corrections_vc1_half_yields_a_row() {
        let mask: Vec<u8> = (1..=37).collect();
        let half = LongTermHalf {
            velocity_code: 1,
            sats: vec![LongTermSat {
                mask: 5,
                iod: 42,
                dx: 8,
                dy: -16,
                dz: 0,
                daf0: 1024,
                ddx: Some(16),
                ddy: Some(-8),
                ddz: Some(0),
                daf1: Some(32),
            }],
            t_lt_s: Some(3600),
            iodp: 0,
        };
        let rows = lt_corrections(&mask, 0, &half);
        assert_eq!(
            rows.len(),
            1,
            "vc=1 halves must no longer be skipped (the live WAAS case)"
        );
        assert_eq!(rows[0].prn, 5);
        assert!(
            rows[0].ddx.is_some() && rows[0].daf1.is_some() && rows[0].t_lt_s.is_some(),
            "the rates and t_lt must ride along for solve-time propagation"
        );
    }

    /// vc=1 scale factors, pinned to DO-229D Table A-11: dx/dy/dz 11-bit
    /// at 0.125 m, daf0 11-bit at 2^-31 s, rates 8-bit at 2^-11 m/s, daf1
    /// 8-bit at 2^-39 s/s, t_lt 13-bit at 16 s (0..=86384 s of the day).
    #[test]
    fn lt_corrections_vc1_scales_and_t_lt() {
        let mask: Vec<u8> = (1..=37).collect();
        let half = LongTermHalf {
            velocity_code: 1,
            sats: vec![LongTermSat {
                mask: 5,
                iod: 42,
                dx: 1024, // 128.0 m (Table A-11 range is ±128 m)
                dy: -8,   // -1.0 m
                dz: 0,
                daf0: 1024, // 1024 * 2^-31 s
                ddx: Some(16),
                ddy: Some(-1),
                ddz: Some(-128), // full-scale negative: -0.0625 m/s
                daf1: Some(64),
            }],
            t_lt_s: Some(86384), // end of the Table A-11 range
            iodp: 0,
        };
        let rows = lt_corrections(&mask, 0, &half);
        assert_eq!(rows.len(), 1);
        let r = &rows[0];
        assert_eq!(r.prn, 5);
        assert_eq!(r.dx, 128.0);
        assert_eq!(r.dy, -1.0);
        assert_eq!(r.dz, 0.0);
        assert!((r.daf0 - 1024.0 * 2.0f64.powi(-31)).abs() < 1e-15);
        assert_eq!(r.ddx, Some(16.0 * 2.0f64.powi(-11)));
        assert_eq!(r.ddy, Some(-2.0f64.powi(-11)));
        assert_eq!(r.ddz, Some(-0.0625));
        assert_eq!(r.daf1, Some(64.0 * 2.0f64.powi(-39)));
        assert_eq!(r.t_lt_s, Some(86384));
        assert_eq!(r.iod, 42);
    }

    /// DO-229D A.4.4.7 eq. (A-18)/(A-19): correction(t) = base +
    /// rate·(t − t_lt), with t and t_lt time-of-day. t_lt "may be in the
    /// past if the prior long-term message is missed" (signed dt).
    #[test]
    fn ltcorr_propagate_linear_law() {
        let c = LtCorr {
            prn: 5,
            dx: 10.0,
            dy: -5.0,
            dz: 1.0,
            daf0: 2.0e-6,
            ddx: Some(0.01),
            ddy: Some(-0.02),
            ddz: Some(0.0),
            daf1: Some(1.0e-9),
            t_lt_s: Some(3600),
            iod: 7,
        };
        // dt = +120 s
        let (x, y, z, a) = c.propagate(3720.0);
        assert!((x - 11.2).abs() < 1e-12, "{x}");
        assert!((y - (-7.4)).abs() < 1e-12, "{y}");
        assert_eq!(z, 1.0);
        assert!((a - (2.0e-6 + 1.2e-7)).abs() < 1e-18, "{a}");
        // dt = −60 s (t_lt in the future is the usual case, A.4.4.7)
        let (x, ..) = c.propagate(3540.0);
        assert!((x - 9.4).abs() < 1e-12, "{x}");
        // a time-of-WEEK input is folded to time-of-day — t_lt is not TOW
        let (x2, ..) = c.propagate(3.0 * 86_400.0 + 3720.0);
        assert!((x2 - 11.2).abs() < 1e-12, "{x2}");
    }

    /// "correcting for rollover if needed" (DO-229D A.4.4.7): t and t_lt
    /// are time-of-day, so across midnight the difference wraps to the
    /// nearest day (±43200 s), not a −86400 s cliff.
    #[test]
    fn ltcorr_propagate_day_rollover() {
        let base = LtCorr {
            prn: 5,
            dx: 0.0,
            dy: 0.0,
            dz: 0.0,
            daf0: 0.0,
            ddx: Some(1.0), // dx reads out dt directly
            ddy: None,
            ddz: None,
            daf1: None,
            t_lt_s: Some(86384),
            iod: 0,
        };
        // 48 s past midnight, t_lt 16 s before midnight: dt = +64, not
        // −86336
        let (x, ..) = base.propagate(86_400.0 + 48.0);
        assert_eq!(x, 64.0);
        // the other direction: t just before midnight, t_lt just after
        let c2 = LtCorr {
            t_lt_s: Some(16),
            ..base.clone()
        };
        let (x, ..) = c2.propagate(86_400.0 - 16.0);
        assert_eq!(x, -32.0);
    }

    /// Velocity-code-0 rows propagate as constants: rates are 0 and the
    /// time of applicability is the message transmission time
    /// (DO-229D A.4.4.7), so the correction never drifts.
    #[test]
    fn ltcorr_propagate_vc0_is_constant() {
        let c = LtCorr {
            prn: 5,
            dx: 1.0,
            dy: -2.0,
            dz: 0.5,
            daf0: 1.0e-6,
            ddx: None,
            ddy: None,
            ddz: None,
            daf1: None,
            t_lt_s: None,
            iod: 3,
        };
        assert_eq!(c.propagate(12_345.0), (1.0, -2.0, 0.5, 1.0e-6));
        assert_eq!(c.propagate(600_000.0), (1.0, -2.0, 0.5, 1.0e-6));
    }

    /// The forced-parity variant: a locked decoder keeps its pairing even
    /// when the noisy energy pick would flip it.
    #[test]
    fn symbols_from_prompt_forced_parity_holds() {
        let mut rng = Rng::new(32);
        let bits: Vec<u8> = (0..200).map(|_| rng.bit()).collect();
        let sym = conv_encode(&bits, false, 0);
        let sign: Vec<f64> = sym.iter().map(|&s| 1.0 - 2.0 * s as f64).collect();
        // build prompts at parity 1, then CORRUPT parity 0's energy pick so
        // the free chooser would flip
        let mut prompts = vec![0.0f64; 2 * sign.len() + 1];
        for (i, &s) in sign.iter().enumerate() {
            prompts[1 + 2 * i] = s;
            prompts[1 + 2 * i + 1] = s;
        }
        let (soft, got) = symbols_from_prompt_par(&prompts, Some(1));
        assert_eq!(got, 1);
        let dec = viterbi(&soft, false);
        assert_eq!(dec[40..], bits[40..]);
    }

    /// REGRESSION (par=1 symbol-grid phase slip): with the parity latched at
    /// 1, three consecutive seconds of exactly 1000 fresh 1 ms prompts per
    /// call must pair on the constant ABSOLUTE grid — every emitted symbol
    /// holds absolute indices (k, k+1) with k ≡ 1 (mod 2) in EVERY call, no
    /// symbol is emitted twice, and at most the boundary straddle is
    /// deferred across a call (never dropped silently). The old
    /// queue-relative latch flipped the grid for a whole second whenever a
    /// straddling leftover ms sat at the queue head, and drained that
    /// leftover unpaired.
    #[test]
    fn symbols_from_prompt_abs_holds_grid_across_straddles() {
        // one second of the live sbas_tick drain protocol: pair `queue`
        // (whose [0] sits at absolute 1 ms index `head`) at forced absolute
        // parity 1; returns (soft, queue-relative start, absolute parity)
        let tick = |q: &[f64], head: u64| -> (Vec<f32>, usize, usize) {
            symbols_from_prompt_abs(q, Some(1), head)
        };
        // distinct marker per absolute 1 ms index: prompt k has value k, so
        // a correctly paired symbol sums to 2k+1 and k is recovered exactly
        let mut queue: Vec<f64> = Vec::new();
        let mut head = 0u64; // absolute index of queue[0]
        let mut next = 0u64; // next absolute index to generate
        let mut firsts = std::collections::BTreeSet::new(); // first index of every emitted pair
        let mut paired = std::collections::BTreeSet::new(); // every paired absolute index
        let mut origin_skipped = 0u64; // heads consumed unpaired at a call start
        for _sec in 0..3 {
            for _ in 0..1000 {
                queue.push(next as f64);
                next += 1;
            }
            let (soft, s, pa) = tick(&queue, head);
            assert_eq!(pa, 1, "the absolute pairing parity must stay latched");
            origin_skipped += s as u64;
            let used = s + 2 * soft.len();
            for &v in &soft {
                assert_eq!(v.fract(), 0.0, "marker sum must stay integral");
                let k = (v as u64 - 1) / 2; // pair (k, k+1) sums to 2k+1
                assert_eq!(v as u64, 2 * k + 1, "symbol is not an adjacent pair");
                assert_eq!(k % 2, 1, "pair ({k},{}) off the absolute par=1 grid", k + 1);
                assert!(firsts.insert(k), "symbol at abs {k} emitted twice");
                assert!(paired.insert(k) && paired.insert(k + 1));
            }
            queue.drain(..used);
            head += used as u64;
        }
        // conservation: every generated prompt is paired, skipped once at
        // the grid origin, or still queued as the one boundary straddle
        assert_eq!(
            paired.len() as u64 + origin_skipped + queue.len() as u64,
            next,
            "prompts dropped silently across calls"
        );
        assert!(queue.len() <= 1, "at most the boundary straddle is deferred");
    }
}
