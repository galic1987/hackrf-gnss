//! Live multi-constellation 1 Hz tracker: GPS L1 C/A + SBAS (WAAS) + Galileo
//! E1B + BeiDou B1I, one DLL+PLL channel per satellite, one JSON report per
//! tracked PRN per second.
//!
//! Two layers:
//!   - [`Band`]: one decimated baseband stream (e.g. L1 at 4 Msps) with its
//!     set of tracking channels. Seeding = acquisition on the first seconds of
//!     buffered stream; then every 4 ms slice is fed to all channels (in
//!     parallel — channels are independent). This is the layer the
//!     integration tests drive directly with f32 baseband fixtures.
//!   - [`Engine`]: the live front end. Consumes interleaved int8 at the
//!     capture rate (16 Msps @ 1568.25 MHz spans BeiDou B1I .. GPS L1), mixes
//!     each constellation's band to baseband with a phase-continuous NCO,
//!     FIR-decimates to 4 Msps, and feeds two `Band`s.
//!
//! The tracking loop is the `gps::track` structure generalized to arbitrary
//! code length / chip rate, with a BOC(1,1)-shaped replica option for Galileo
//! E1B: 2nd-order Costas PLL (Borre coefficients) around the acquired Doppler,
//! carrier-aided code NCO, normalized early-minus-late DLL. No bit sync / nav
//! decode — the decisive outputs are per-second Doppler, a C/N0 proxy and the
//! lock duration.
//!
//! Lock state machine (per channel, evaluated once per tracked second):
//! declare lock after 3 consecutive seconds with C/N0 proxy >= 33 dB-Hz,
//! drop after 5 consecutive seconds below 30. The engine re-acquires a
//! channel whose lock has been lost for > 60 s and drops it after 5 min.

use num_complex::Complex;
use rayon::prelude::*;
use std::collections::VecDeque;
use std::f64::consts::PI;

use crate::gps::broadcast::BrdcEph;
use crate::dsp_calib::generate_beidou_b1_code;
use crate::galileo::{acquire_e1b, e1b_code};
use crate::gps::ca_code::gps_ca;
use crate::gps::{acquire, acquire_sbas, acquire_codes, sbas_code, F_L1};

/// BeiDou B1I carrier (Hz).
pub const F_B1I: f64 = 1561.098e6;
/// B1I chip rate (Hz).
const B1I_CHIP_RATE: f64 = 2.046e6;
/// Decimated rate every band runs at (Hz).
pub const BAND_FS: f64 = 4.0e6;
/// Seconds of stream buffered before the seeding acquisition runs.
const SEED_S: f64 = 2.0;
/// Seconds of processed stream kept for re-acquisition / discovery
/// snapshots. Raised to 2.5 s so background discovery seeds (2 s) can run
/// on history without interrupting tracking.
const HIST_S: f64 = 2.5;
/// C/N0-proxy thresholds for the lock state machine (dB-Hz).
const LOCK_DB: f64 = 33.0;
const UNLOCK_DB: f64 = 30.0;
/// Consecutive seconds above/below threshold to declare / drop lock.
const LOCK_SECS: u32 = 3;
const UNLOCK_SECS: u32 = 5;
/// Re-acquire a channel unlocked this long; drop it after DROP_S.
const REACQ_S: f64 = 60.0;
const DROP_S: f64 = 300.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sys {
    Gps,
    Sbas,
    Galileo,
    Beidou,
}

impl Sys {
    pub fn name(self) -> &'static str {
        match self {
            Sys::Gps => "gps",
            Sys::Sbas => "sbas",
            Sys::Galileo => "galileo",
            Sys::Beidou => "beidou",
        }
    }

    pub fn from_name(s: &str) -> Option<Sys> {
        match s {
            "gps" => Some(Sys::Gps),
            "sbas" => Some(Sys::Sbas),
            "galileo" => Some(Sys::Galileo),
            "beidou" => Some(Sys::Beidou),
            _ => None,
        }
    }
}

/// Stream time of the start of nav bit `abs_bit` (absolute index into the
/// channel's emitted bit stream; 20 ms per bit for both GPS LNAV and BDS D1).
/// The un-emitted ms remainder in nav_ms sits between the last emitted bit
/// and t_proc — forgetting it puts each channel's anchor off by a random
/// 0-19 ms (thousands of km of pseudorange; the reason the first anchored
/// PVT landed in Hudson Bay). The estimate is refined to the code-phase
/// wrap: bit edges align with code period starts, so the true boundary is an
/// integer number of CODE PERIODS before the most recent code wrap.
///
/// The refinement must snap to the code-period (~1 ms) lattice, NOT to
/// 20 ms buckets hanging off t_wrap: the number of code periods between the
/// boundary and t_wrap is a multiple of 20 only by luck — Doppler drift
/// walks it, and it steps by one every time the code phase wraps at the
/// 1-s report boundary. A 20 ms snap then answered r ms early (r = periods
/// mod 20), flickering by +/-1 ms per refresh and by ~20 ms near the
/// rounding boundary (both observed live, 2026-08-24). The nav bookkeeping
/// already pins the boundary to +/-0.5 ms (bit sync resolves the edge to
/// the enclosing 1 ms epoch), so the nearest code-period boundary IS the
/// true bit edge — PROVIDED the epoch quantization picked the right 1 ms
/// cell; when it hasn't (noisy sync in churn), or while the comb wander has
/// pushed the edge across the tie zone, the snap parks/flips one code
/// period off. The propagation and flip-dip logic in the body exist to make
/// the tooth choice immune to both.
fn anchor_stream_time(ch: &mut Channel, t_proc: f64, abs_bit: usize, t_tx: f64) -> f64 {
    // abs_bit is always BEHIND the emitted-bit count (the newest complete
    // subframe ends >= 300 bits before the stream edge): this difference is
    // negative, so it MUST be computed in f64 — the usize subtraction
    // underflowed and wrapped to ~3.7e17 s, which is exactly the frozen
    // rho_m = 1.1e26 m garbage seen live.
    let t_bit_approx = t_proc - ch.nav_ms.len() as f64 / 1000.0
        + 0.02 * (abs_bit as f64 - ch.nav_bits.len() as f64);
    // the comb period is MEASURED by tooth counting (see track_comb), not
    // derived from carrier_freq: a marginal channel can hold a sustained
    // carrier bias of 60-280 Hz while staying code-locked, and the carrier
    // -derived rate then leaks into every anchor as a secular rho drift of
    // delta_f/f_carrier (36-150 ns/s per channel, observed live)
    let t_code = ch.t_code_meas;
    let t_wrap = ch.wrap_time(t_proc);
    // Tooth-exact propagation: boundaries t_tx seconds apart are exactly
    // 1000 code periods per second apart on the received comb (20 teeth per
    // 20 ms bit, transmit-synchronous), so referencing the previous anchor
    // recovers the boundary with no approximation error at all.
    //
    // The nav bookkeeping (t_bit_approx) is only a SANITY reference here:
    // it is epoch-gridded and wanders away from the comb at ~1.3-2.5 us/s
    // from bit-sync time, so a tight cross-check is fatal — with a 2 ms
    // gate, every refresh started failing after ~15-25 min of channel age
    // and the anchor fell back to the bare snap, which RAMPS at the
    // comb-vs-epoch creep rate (up to +/-2.4 us/s) and jumps a whole tooth
    // every time the creep crosses a tie boundary (both observed live: the
    // smooth 40-150 ns/s per-channel marches and the +/-2-3 ms jumps).
    // 15 ms still catches the real failures by orders of magnitude: a bit
    // slip is a 20 ms multiple, a garbage t_tx decode is seconds.
    if let Some((prev_t_bit, prev_t_tx)) = ch.anchor {
        let d_tx = t_tx - prev_t_tx;
        // Wrong-tooth self-heal. A first pick can land one code period off
        // (e.g. picked before the flip-dip audit converged); left alone the
        // propagation below propagates the wrong tooth forever (observed
        // live: a channel parked +1 tooth for 15+ min). The rescue is the
        // bookkeeping snap — but it is only trustworthy while YOUNG: the
        // approximation wanders from the comb at up to ~3 us/s from bit
        // sync, so within ~90 s it is provably inside a quarter-tooth of
        // the true boundary and a confident snap IS the true tooth. Older
        // channels can sit confidently on the WRONG tooth, so the gate
        // closes with age and an established anchor is never disturbed by
        // a wandered approximation.
        if d_tx >= 0.0 && d_tx < 120.0 && ch.nav_bits.len() < 4500 {
            let q = (t_wrap - t_bit_approx) / t_code;
            let cand = t_wrap - t_code * q.round();
            let conf = (q - q.round()).abs(); // teeth from approx to its snap
            let expected = prev_t_bit + (d_tx * 1000.0).round() * t_code;
            let d_teeth = ((cand - expected) / t_code).abs();
            if conf <= 0.25 && (0.5..=1.5).contains(&d_teeth) {
                eprintln!(
                    "live: PRN {} anchor {:+.1} us off the confident bookkeeping snap — re-picked (wrong-tooth first pick)",
                    ch.prn,
                    (expected - cand) * 1e6
                );
                ch.edge_off_s = cand - t_bit_approx;
                ch.edge_off_valid = true;
                return cand;
            }
        }
        if d_tx == 0.0 {
            // Same subframe re-validated (the per-second re-anchor from the
            // widened re-scan window): the boundary instant is FIXED, and
            // the TOW match makes the pair self-consistent — return the
            // established instant unconditionally. (Recomputing it from the
            // current comb measures ~10^4-10^5 teeth at the CURRENT
            // carrier-derived rate, injecting loop wobble amplified by the
            // whole span; and cross-checking against the wandering
            // approximation throws the channel into the ramping fallback —
            // both seen live.)
            ch.edge_off_s = prev_t_bit - t_bit_approx;
            ch.edge_off_valid = true;
            return prev_t_bit;
        }
        if d_tx > 0.0 && d_tx < 120.0 {
            let periods = (d_tx * 1000.0).round();
            let m = ((t_wrap - prev_t_bit) / t_code - periods).round();
            let t_prop = t_wrap - t_code * m;
            if (t_prop - t_bit_approx).abs() < 15.0e-3 {
                // keep the fallback snap reference comb-referenced: it
                // wanders against the epoch grid and goes stale in minutes
                ch.edge_off_s = t_prop - t_bit_approx;
                ch.edge_off_valid = true;
                return t_prop;
            }
        }
    }
    // first anchor (or a broken propagation chain): the approximation is
    // good to one 1 ms epoch; the flip-dip side (when known) says which
    // interval holds the edge — without it the nearest-tooth snap can park
    // one code period off (the static +/-1 ms anchors seen live)
    let off = if ch.edge_off_valid { ch.edge_off_s } else { 0.0 };
    t_wrap - t_code * ((t_wrap - t_bit_approx - off) / t_code).round()
}

/// Max age of the newest validated subframe before a channel's rho_m/t_tx
/// are withdrawn. With a healthy bit stream the anchor re-anchors every
/// second (see the re-scan windows below), so 30 s stale means the nav
/// pipeline is dead even if the RF loops still read "locked" — publishing
/// the frozen anchor then reports the satellite's range RATE as a growing
/// pseudorange error (observed live 2026-08-24: a GPS channel locked the
/// whole time diverged at 147-885 ns/s = its range rate, because its
/// subframes stopped validating during a cn0 churn while its neighbour
/// kept refreshing).
const ANCHOR_MAX_AGE_S: f64 = 30.0;

/// Subframe re-scan overlaps. find_subframes reports the NEWEST valid
/// subframe in the window; the window must reach back far enough that the
/// last validated subframe stays findable, so the anchor re-anchors to it
/// EVERY SECOND between validations instead of freezing at its last value
/// (the freeze was the per-channel divergence above). GPS LNAV skips
/// indices 0-1 (it needs the two trailing bits of the previous subframe),
/// hence 300 + 2. BDS D1 keeps only cross-consistent PAIRS (b = a + 300
/// bits, SOW + 6 s), so the window must cover both members of the last
/// pair: 2 x 300.
const GPS_RESCAN_BACK: usize = 302;
const D1_RESCAN_BACK: usize = 600;

/// CRC-valid 250-bit blocks required for SBAS frame-sync lock (same value
/// as the sbas.rs end-to-end tests).
const SBAS_MIN_BLOCKS: usize = 3;

/// One tracked satellite: a Costas PLL + early/late DLL over one code period
/// per epoch (1 ms for GPS/SBAS/B1I, 4 ms for Galileo E1B).
pub struct Channel {
    pub sys: Sys,
    pub prn: usize,
    fs: f64,
    ns_epoch: usize,
    code: Vec<f32>,
    code_len: f64,
    chip_rate: f64,
    f_carrier: f64,
    boc: bool,
    spacing: f64,
    // loop state
    carrier_freq: f64,
    carrier_phase: f64,
    code_phase: f64,
    carr_nco: f64,
    old_carr_err: f64,
    old_ip: f64,
    old_qp: f64,
    pll_t1: f64,
    pll_t2: f64,
    carr_basis: f64,
    // per-second accumulators
    sec_prompt: f64,
    sec_noise: f64,
    sec_epochs: u32,
    // lock state
    pub lock_s: f64,
    pub locked: bool,
    pub lost_s: f64,
    cn0_ema: f64,
    above: u32,
    below: u32,
    last_cn0: f64,
    // carrier phase observable
    /// Integrated replica carrier phase in cycles, zero at channel (re)seed.
    /// Advances by the exact per-epoch NCO increment (carrier_phase is the
    /// same quantity wrapped to 2*pi), so it is continuous across reports
    /// while the loop holds. The absolute value carries the Costas 180
    /// degree ambiguity — only differences (rates) are physical.
    carr_cycles: f64,
    /// Phase break in the in-progress report second: set by the lock
    /// watchdog on lock loss and by reseed (the phase chain is broken and
    /// carr_cycles restarts at zero). Cleared when the report is emitted.
    slip: bool,
    /// total phase breaks since channel creation (diagnostic)
    pub slip_count: u32,
    // nav demod state (GPS LNAV 50 bps): prompt-I per 1 ms epoch, the
    // discovered 20 ms bit boundary, and the emitted bit stream
    nav_ms: Vec<f64>,
    /// Absolute 1 ms-prompt index of nav_ms[0], advanced by EVERY drain of
    /// nav_ms below (bit sync, bit slicing, the 200k cap, sbas_tick). The
    /// SBAS 2 ms pairing holds a constant ABSOLUTE grid through it: a
    /// straddling leftover ms shifts the queue head, and a queue-relative
    /// parity latch would then flip the pairing every other second (the
    /// par=1 phase slip). Reset with the SBAS generation (sbas_reset).
    nav_abs_ms: u64,
    bit_off: Option<usize>,
    /// decoded nav bits (0/1), polarity unresolved (Costas) — lnav handles it
    pub nav_bits: Vec<u8>,
    /// how far into nav_bits find_subframes has already scanned
    nav_scanned: usize,
    /// (stream time, GPS transmit time) of the newest decoded subframe
    /// boundary — the anchor for true pseudoranges.
    pub anchor: Option<(f64, f64)>,
    /// t_proc of the last anchor refresh; rho_m is published only while
    /// this is younger than ANCHOR_MAX_AGE_S (frozen anchors lie — see the
    /// constant's comment)
    anchor_t: f64,
    /// Snap-reference offset for anchor picks that can't use propagation:
    /// which side of the emitted group boundary holds the true bit edge
    /// (+/-0.5 ms from the flip-dip audit), refined to the exact comb
    /// offset by every propagated anchor. Without it the nearest-tooth
    /// snap can park one code period off (the static +/-1 ms anchors).
    edge_off_s: f64,
    edge_off_valid: bool,
    // flip-dip audit accumulators (GPS only — BDS nav_ms carries NH20 sign
    // flips that drown the data-edge dip): mean |prompt| of the first vs
    // last epoch of groups at bit-value transitions. The smaller side
    // holds the edge (the mixed epoch integrates to |1 - 2*delta| of full
    // amplitude).
    dip_first: f64,
    dip_last: f64,
    dip_n: u32,
    prev_group_tail: Option<f64>,
    // Measured comb period (stream-time seconds per code period), from
    // direct tooth counting between report seconds — NOT the
    // carrier-derived rate. A marginal channel can hold a sustained
    // carrier_freq bias of 60-280 Hz while staying code-locked (a 200 Hz
    // offset costs <1 dB of 1 ms prompt power); the carrier-derived code
    // rate then leaks into every anchor step as a secular rho drift of
    // delta_f/f_c (observed live: 36-150 ns/s per channel, both signs,
    // matching the per-channel Doppler-vs-ephemeris residuals). Teeth
    // elapsed per second are INTEGER — counting them and dividing elapsed
    // stream time is bias-free.
    t_code_meas: f64,
    comb_acc_t: f64,
    comb_acc_teeth: f64,
    last_wrap: f64,
    /// self-decoded broadcast ephemeris (subframes 1-3 assembled live)
    pub eph: Option<BrdcEph>,
    /// SBAS/WAAS streaming decoder (Sys::Sbas only): fed once per second
    /// from the same 1 ms prompt buffer the GPS/BDS nav demod collects in
    /// nav_ms (sbas_tick drains it, so the 200k cap never binds for SBAS).
    sbas_dec: crate::sbas::Decoder,
    /// Latched 1 ms->2 ms symbol pairing while the decoder is locked
    /// (review round 5: re-picking by energy every second lets a noisy
    /// flip insert/delete a coded symbol mid-stream). The ABSOLUTE grid
    /// parity: pairs hold absolute 1 ms indices (k, k+1) with k ≡ par —
    /// the queue-relative start is derived per tick from nav_abs_ms. None
    /// = probing.
    sbas_par: Option<usize>,
    /// The pairing the RETAINED decoder window was built with, pinned when
    /// the latch engages and kept across lock losses. A re-probed pairing
    /// that differs from it means a coded symbol was inserted/deleted at
    /// the seam — the symbol stream is broken and the decode generation
    /// terminates (sbas_reset). None = no established window pairing.
    sbas_par_prev: Option<usize>,
    /// Apply-once watermark: the absolute stream position
    /// (sbas::DecodedMessage::sym_pos) of the newest block applied to the
    /// correction caches. The decoder re-runs over its RETAINED window
    /// every tick and returns every message again; only blocks beyond this
    /// watermark may touch the caches — a replayed block must never refresh
    /// an insert timestamp (review round 6: the replay re-inserted every
    /// cached message each second with age 0.0, freshness fabricated by
    /// replay). Reset with the SBAS generation (sbas_reset).
    sbas_applied: Option<usize>,
    /// Latest per-PRN fast corrections (PRC m, UDREI, insert stream-time
    /// s), from MT2-5 messages decoded by this channel. Cleared on reseed
    /// and on an MT1 mask-generation change (IODP).
    sbas_prc: std::collections::BTreeMap<u8, (f64, u8, f64)>,
    /// Latest MT1 PRN mask: (absolute slot numbers of the set bits in
    /// mask order, IODP). MT2-5 and MT24/25 corrections only decode
    /// against this mask — DO-229 addresses their entries by ORDINAL of
    /// the set bits and gates them on IODP match. Cleared on reseed.
    sbas_mask: Option<(Vec<u8>, u8)>,
    /// Latest per-PRN long-term corrections (LtCorr in physical units —
    /// vc=1 rows carry rates and t_lt, insert stream-time s), from MT24/25
    /// halves decoded by this channel. Cleared on reseed and on an MT1
    /// mask-generation change (IODP).
    sbas_lt: std::collections::BTreeMap<u8, (crate::sbas::LtCorr, f64)>,
    /// Latest iono grid masks by band (iodi, IGP list, insert stream-time
    /// s), from MT18. Cleared on reseed.
    sbas_igpmask: std::collections::BTreeMap<u8, (u8, Vec<u16>, f64)>,
    /// Latest iono delay blocks by (band, block_id): (iodi, 15 x
    /// (vertical-delay counts, GIVEI), insert stream-time s), from MT26.
    /// Cleared on reseed.
    sbas_iono: std::collections::BTreeMap<(u8, u8), (u8, [(u16, u8); 15], f64)>,
}

/// Borre 2nd-order loop-filter time constants (see gps::track).
fn borre(bn: f64, zeta: f64, k: f64) -> (f64, f64) {
    let wn = bn * 8.0 * zeta / (4.0 * zeta * zeta + 1.0);
    (k / (wn * wn), 2.0 * zeta / wn)
}

impl Channel {
    /// New channel seeded from an acquisition result: `dopp0` Hz at the
    /// band's baseband, `code_phase0` chips, both referring to the current
    /// stream position.
    ///
    /// CONVENTION: every acquisition here (gps::acquire*, galileo::acquire_e1b)
    /// reports the correlation LAG — the signal is `code[k*step - cp0]`, so the
    /// tracker's forward-running replica must start at `-cp0` (mod code_len).
    /// Verified empirically on real L1 baseband: prompt power peaks at the
    /// negated phase (32x over noise) and is flat at the raw reported one.
    pub fn new(sys: Sys, prn: usize, fs: f64, dopp0: f64, code_phase0: f64) -> Channel {
        let (code, chip_rate, f_carrier, boc, spacing, period_ms) = match sys {
            Sys::Gps => (gps_ca(prn), 1.023e6, F_L1, false, 0.5, 1usize),
            Sys::Sbas => (sbas_code(prn), 1.023e6, F_L1, false, 0.5, 1),
            Sys::Galileo => (
                e1b_code(prn).iter().map(|&c| c as f32).collect(),
                1.023e6,
                F_L1,
                true,
                0.125,
                4,
            ),
            Sys::Beidou => (generate_beidou_b1_code(prn), B1I_CHIP_RATE, F_B1I, false, 0.5, 1),
        };
        let code_phase0 = (-code_phase0).rem_euclid(code.len() as f64);
        let ns_epoch = (fs * period_ms as f64 / 1000.0).round() as usize;
        let (pll_t1, pll_t2) = borre(10.0, 0.7, 0.25);
        let t_code0 = code.len() as f64 / (chip_rate * (1.0 + dopp0 / f_carrier));
        Channel {
            sys,
            prn,
            fs,
            ns_epoch,
            code_len: code.len() as f64,
            code,
            chip_rate,
            f_carrier,
            boc,
            spacing,
            carrier_freq: dopp0,
            carrier_phase: 0.0,
            code_phase: code_phase0,
            carr_nco: 0.0,
            old_carr_err: 0.0,
            old_ip: 0.0,
            old_qp: 0.0,
            pll_t1,
            pll_t2,
            carr_basis: dopp0,
            sec_prompt: 0.0,
            sec_noise: 0.0,
            sec_epochs: 0,
            lock_s: 0.0,
            locked: false,
            lost_s: 0.0,
            cn0_ema: 0.0,
            above: 0,
            below: 0,
            last_cn0: 0.0,
            carr_cycles: 0.0,
            slip: false,
            slip_count: 0,
            nav_ms: Vec::new(),
            nav_abs_ms: 0,
            bit_off: None,
            nav_bits: Vec::new(),
            nav_scanned: 0,
            anchor: None,
            anchor_t: f64::NEG_INFINITY,
            edge_off_s: 0.0,
            edge_off_valid: false,
            dip_first: 0.0,
            dip_last: 0.0,
            dip_n: 0,
            prev_group_tail: None,
            // seeded from the carrier estimate; the tooth-counting
            // accumulator converges to the true comb period within seconds
            t_code_meas: t_code0,
            comb_acc_t: 0.0,
            comb_acc_teeth: 0.0,
            last_wrap: f64::NAN,
            eph: None,
            sbas_dec: crate::sbas::Decoder::new(),
            sbas_par: None,
            sbas_par_prev: None,
            sbas_applied: None,
            sbas_prc: std::collections::BTreeMap::new(),
            sbas_mask: None,
            sbas_lt: std::collections::BTreeMap::new(),
            sbas_igpmask: std::collections::BTreeMap::new(),
            sbas_iono: std::collections::BTreeMap::new(),
        }
    }

    /// Measured tooth instant of the most recent code wrap, using the
    /// MEASURED comb period for the fractional part (carrier-bias-immune).
    fn wrap_time(&self, t_proc: f64) -> f64 {
        t_proc - (self.code_phase / self.code_len) * self.t_code_meas
    }

    /// Per-second comb tracking: count the integer teeth elapsed since the
    /// last report and accumulate the measured tooth period. The round is
    /// robust to any plausible rate error (0.5 tooth in ~1000 needs 500
    /// ppm). Only accumulates while locked — an unlocked DLL free-runs.
    fn track_comb(&mut self, t_proc: f64) {
        let t_wrap = self.wrap_time(t_proc);
        if !self.locked {
            self.last_wrap = f64::NAN;
            return;
        }
        if self.last_wrap.is_finite() {
            let teeth = ((t_wrap - self.last_wrap) / self.t_code_meas).round();
            if teeth >= 1.0 && teeth < 100_000.0 {
                self.comb_acc_t += t_wrap - self.last_wrap;
                self.comb_acc_teeth += teeth;
                // sliding ~64 s window (halve when full): tracks the true
                // Doppler drift with negligible lag
                if self.comb_acc_teeth > 64_000.0 {
                    self.comb_acc_t *= 0.5;
                    self.comb_acc_teeth *= 0.5;
                }
                if self.comb_acc_teeth >= 1000.0 {
                    self.t_code_meas = self.comb_acc_t / self.comb_acc_teeth;
                }
            }
        }
        self.last_wrap = t_wrap;
    }

    /// Code replica value at fractional chip phase (with BOC(1,1) subcarrier
    /// shaping for Galileo E1B: +1 over the first half-chip, -1 over the
    /// second, like `galileo::boc11_replica`).
    #[inline]
    fn chip_at(&self, cp: f64) -> f64 {
        // cp is always within [-1, 1.6 * code_len) here (E/P/L ± spacing and
        // the +code_len/2 noise replica), so ONE conditional wrap replaces
        // rem_euclid: fmod was ~25 ns x 5 calls/sample/channel — multiple
        // whole cores of nothing, and the reason the consumer couldn't hold
        // line rate with channels tracking (found via `sample`: the main
        // thread lived in libm fmod).
        let c = if cp < 0.0 {
            cp + self.code_len
        } else if cp >= self.code_len {
            cp - self.code_len
        } else {
            cp
        };
        let v = self.code[c as usize % self.code.len()] as f64;
        if self.boc && c.fract() >= 0.5 {
            -v
        } else {
            v
        }
    }

    /// Prompt-correlator power over one epoch at a FIXED doppler/code phase,
    /// no loop-state updates — used by the seed-time Doppler refinement scan.
    fn prompt_only(&self, sig: &[Complex<f32>], dopp: f64, cp0: f64) -> f64 {
        let ns = self.ns_epoch;
        let code_rate = self.chip_rate * (1.0 + dopp / self.f_carrier);
        let code_step = code_rate / self.fs;
        let dphi = 2.0 * PI * dopp / self.fs;
        let (mut ip, mut qp) = (0.0f64, 0.0f64);
        let mut cp = cp0;
        for (k, s) in sig[..ns].iter().enumerate() {
            let ph = dphi * k as f64;
            let (si, co) = ph.sin_cos();
            let (re, im) = (s.re as f64, s.im as f64);
            let c = self.chip_at(cp);
            ip += (re * co + im * si) * c;
            qp += (-re * si + im * co) * c;
            cp += code_step;
            if cp >= self.code_len {
                cp -= self.code_len;
            }
        }
        ip * ip + qp * qp
    }

    /// Scan doppler in ±`half` Hz around the acquisition value in `step`
    /// steps, keep the argmax of the non-coherent prompt power over `nep`
    /// epochs — a 500 Hz acquisition grid leaves up to ±250 Hz of residual,
    /// far outside the pull-in of a 10 Hz Costas loop.
    fn refine_dopp(&mut self, sig: &[Complex<f32>], half: f64, step: f64, nep: usize) -> f64 {
        let need = nep * self.ns_epoch;
        if sig.len() < need {
            return self.carrier_freq;
        }
        let mut best_d = self.carrier_freq;
        let mut best_p = -1.0f64;
        let n = (half / step).round() as i64;
        for k in -n..=n {
            let d = self.carr_basis + k as f64 * step;
            // code phase advances epoch to epoch at the carrier-aided rate
            let mut p = 0.0;
            for e in 0..nep {
                let cp0 = self.code_phase
                    + self.chip_rate * (1.0 + d / self.f_carrier) * e as f64
                        * self.ns_epoch as f64
                        / self.fs;
                p += self.prompt_only(&sig[e * self.ns_epoch..], d, cp0);
            }
            if p > best_p {
                best_p = p;
                best_d = d;
            }
        }
        self.carrier_freq = best_d;
        self.carr_basis = best_d;
        best_d
    }

    /// Run one epoch (one code period) over `sig` (must be >= ns_epoch).
    /// Returns (prompt, quadrature, off-code-noise) powers for diagnostics.
    fn process_epoch(&mut self, sig: &[Complex<f32>]) -> (f64, f64, f64) {
        let ns = self.ns_epoch;
        let pdi = ns as f64 / self.fs;
        // carrier-aided code rate (code Doppler = carrier Doppler scaled)
        let code_rate = self.chip_rate * (1.0 + self.carrier_freq / self.f_carrier);
        let code_step = code_rate / self.fs;
        let dphi = 2.0 * PI * self.carrier_freq / self.fs;
        let (sd, cd) = dphi.sin_cos();
        // carrier wipe: s * exp(-j*theta), advancing theta by dphi per
        // sample — the gps::track convention. The previous recurrence
        // rotated the wrong way, which made the Costas loop's stable points
        // sit at ±90° (signal in Q, noise in I): channels "locked" on code
        // power but the prompt sign carried no data — no nav bits.
        let (mut sn, mut cs) = self.carrier_phase.sin_cos();

        let (mut ie, mut qe) = (0.0f64, 0.0f64);
        let (mut ip, mut qp) = (0.0f64, 0.0f64);
        let (mut il, mut ql) = (0.0f64, 0.0f64);
        let (mut inz, mut qnz) = (0.0f64, 0.0f64);
        let noise_shift = self.code_len / 2.0; // off-code replica = pure noise
        let mut cp = self.code_phase;
        for s in &sig[..ns] {
            // wipe carrier (recurrence rotation; re-anchored each epoch)
            let (re, im) = (s.re as f64, s.im as f64);
            let bi = re * cs + im * sn;
            let bq = -re * sn + im * cs;
            let ncs = cs * cd - sn * sd;
            sn = cs * sd + sn * cd;
            cs = ncs;
            // code replicas E/P/L + off-code noise reference
            let ce = self.chip_at(cp - self.spacing);
            let cpr = self.chip_at(cp);
            let cl = self.chip_at(cp + self.spacing);
            let cn = self.chip_at(cp + noise_shift);
            ie += bi * ce;
            qe += bq * ce;
            ip += bi * cpr;
            qp += bq * cpr;
            il += bi * cl;
            ql += bq * cl;
            inz += bi * cn;
            qnz += bq * cn;
            cp += code_step;
            if cp >= self.code_len {
                cp -= self.code_len;
            }
        }
        self.code_phase = cp;
        self.carrier_phase = (self.carrier_phase + dphi * ns as f64).rem_euclid(2.0 * PI);
        // integrated carrier phase (the SatReport observable): the exact
        // NCO advance in cycles, unwrapped — carrier_phase is this mod 1.
        self.carr_cycles += dphi * ns as f64 / (2.0 * PI);

        // FLL aid: cross-product frequency discriminator. The seeds/aligns
        // refine Doppler to ±25 Hz but the Costas PLL's pull-in is ~10 Hz,
        // so without this the carrier phase kept rotating — code lock held
        // (cn0 is code-domain) yet nav bits were unsliceable noise. The
        // gain is deliberately tiny (τ ≈ 0.5 s): the discriminator is
        // noise-dominated per epoch, and the first attempt (0.1) injected
        // ±16 Hz of jitter per epoch and destroyed phase coherence.
        // BeiDou D1 carries the NH20 secondary code: (ip,qp) flips sign at
        // ~11 of every 20 1-ms epochs, and a raw cross product reads those
        // flips as pi phase steps — the aid random-walked the carrier off
        // and every B1I channel lost lock within ~10 s on the wideband
        // replay (GPS held: LNAV flips are 20x rarer). The dot product
        // detects the flip; un-flipping makes the discriminator NH-immune
        // (the Costas discriminator below already is).
        let pwr = (ip * ip + qp * qp).max(1e-12);
        let mut cross = (self.old_ip * qp - ip * self.old_qp) / pwr; // ~sin(dphi)
        if self.sys == Sys::Beidou && self.old_ip * ip + self.old_qp * qp < 0.0 {
            cross = -cross;
        }
        self.old_ip = ip;
        self.old_qp = qp;
        let f_err = cross / (2.0 * PI * pdi);
        self.carr_nco += 0.002 * f_err;

        // Costas discriminator (data-bit insensitive), Borre 2nd-order filter
        let norm = pwr.sqrt();
        let carr_err = (qp * ip.signum()) / norm / (2.0 * PI);
        self.carr_nco +=
            (self.pll_t2 / self.pll_t1) * (carr_err - self.old_carr_err) + carr_err * (pdi / self.pll_t1);
        self.old_carr_err = carr_err;
        self.carrier_freq = self.carr_basis + self.carr_nco;

        // DLL: normalized early-minus-late power, direct code-phase nudge
        let e = (ie * ie + qe * qe).sqrt();
        let l = (il * il + ql * ql).sqrt();
        if e + l > 1e-12 {
            let dll_err = 0.5 * (e - l) / (e + l);
            self.code_phase = (self.code_phase - dll_err).rem_euclid(self.code_len);
        }

        self.sec_prompt += ip * ip + qp * qp;
        self.sec_noise += inz * inz + qnz * qnz; // off-code replica: pure noise
        self.sec_epochs += 1;
        // nav demod: collect prompt-I per 1 ms epoch (GPS LNAV, BeiDou D1,
        // and SBAS/WAAS — the Costas loop carries the data on ip; SBAS
        // epochs are 1 ms like GPS, one push per epoch, drained per second
        // by sbas_tick). Cap at ~3 min.
        if matches!(self.sys, Sys::Gps | Sys::Beidou | Sys::Sbas) {
            self.nav_ms.push(ip);
            if self.nav_ms.len() > 200_000 {
                let drop = self.nav_ms.len() - 200_000;
                self.nav_ms.drain(..drop);
                self.nav_abs_ms += drop as u64;
            }
        }
        (ip * ip + qp * qp, qp * qp, inz * inz + qnz * qnz)
    }

    /// Debug: transition-phase histogram of the 1 ms prompt series.
    pub fn debug_nav_hist(&self) -> (usize, [usize; 20]) {
        let mut trans = [0usize; 20];
        let mut ntrans = 0usize;
        for i in 1..self.nav_ms.len() {
            if self.nav_ms[i].signum() != self.nav_ms[i - 1].signum() {
                trans[i % 20] += 1;
                ntrans += 1;
            }
        }
        (ntrans, trans)
    }

    /// Debug/test hook: force the carrier frequency estimate (simulates a
    /// marginal-lock loop bias for the anchor-path immunity tests).
    pub fn debug_set_dopp(&mut self, f: f64) {
        self.carrier_freq = f;
        self.carr_basis = f;
    }

    /// Nav-bit slicing, called once per second. GPS: bit sync uses the
    /// transition-phase histogram (gps::track's proven method): nearly all
    /// 1 ms sign changes land on ONE phase mod 20 for real data. Then one
    /// bit per 20 ms group. Polarity stays unresolved — the LNAV subframe
    /// finder tries both.
    /// BeiDou D1: bit sync IS the NH20 secondary-code sync (non-coherent
    /// correlation over 20 ms groups); bits are the NH-wiped 20 ms sums.
    pub fn nav_tick(&mut self) {
        match self.sys {
            Sys::Gps => self.nav_tick_gps(),
            Sys::Beidou => self.nav_tick_bds(),
            _ => {}
        }
    }

    fn nav_tick_gps(&mut self) {
        if self.bit_off.is_none() {
            if self.nav_ms.len() < 400 || !self.locked {
                return;
            }
            let mut trans = [0usize; 20];
            let mut ntrans = 0usize;
            let n = self.nav_ms.len().min(6000);
            for i in 1..n {
                if self.nav_ms[i].signum() != self.nav_ms[i - 1].signum() {
                    trans[i % 20] += 1;
                    ntrans += 1;
                }
            }
            let (best, cnt) = trans
                .iter()
                .copied()
                .enumerate()
                .max_by_key(|&(_, c)| c)
                .unwrap_or((0, 0));
            // real GPS: nearly all transitions on one phase; noise: ~1/20
            if ntrans >= 20 && cnt as f64 / ntrans as f64 > 0.5 {
                self.bit_off = Some(best);
                self.nav_ms.drain(..best);
                self.nav_abs_ms += best as u64;
                // new grid: the flip-dip audit starts over
                self.dip_first = 0.0;
                self.dip_last = 0.0;
                self.dip_n = 0;
                self.prev_group_tail = None;
            } else {
                return;
            }
        }
        // emit bits for every complete 20 ms group
        while self.nav_ms.len() >= 20 {
            // flip-dip audit: when the bit value changes at a group
            // boundary, exactly one straddling epoch contains the edge and
            // integrates to |1 - 2*delta| of full amplitude. The dip side
            // says which 1 ms interval holds the true bit edge — the
            // nearest-tooth anchor snap is ambiguous exactly there.
            if let Some(tail) = self.prev_group_tail {
                let head = self.nav_ms[0];
                if head.signum() != tail.signum() {
                    self.dip_first = 0.95 * self.dip_first + 0.05 * head.abs();
                    self.dip_last = 0.95 * self.dip_last + 0.05 * tail.abs();
                    self.dip_n += 1;
                    if self.dip_n >= 8 {
                        let lo = self.dip_first.min(self.dip_last);
                        let hi = self.dip_first.max(self.dip_last);
                        if hi > 0.0 && (hi - lo) / hi > 0.15 {
                            self.edge_off_s = if self.dip_first < self.dip_last {
                                0.5e-3 // edge in the group's first epoch
                            } else {
                                -0.5e-3 // edge in the previous group's last epoch
                            };
                            self.edge_off_valid = true;
                        }
                    }
                }
            }
            self.prev_group_tail = Some(self.nav_ms[19]);
            let s: f64 = self.nav_ms[..20].iter().sum();
            self.nav_bits.push(if s > 0.0 { 1 } else { 0 });
            self.nav_ms.drain(..20);
            self.nav_abs_ms += 20;
        }
    }

    /// BeiDou D1: the NH20 secondary code period IS the 20 ms nav bit, so
    /// the NH phase estimate doubles as bit sync. Wiping NH20 then gives one
    /// clean bit integration per 20 ms group (no transition histogram needed
    /// — NH wipe removes the dominant within-bit sign flips).
    fn nav_tick_bds(&mut self) {
        if self.bit_off.is_none() {
            if self.nav_ms.len() < 2000 || !self.locked {
                return;
            }
            let n = self.nav_ms.len().min(6000);
            match crate::beidou_d1::nh_sync(&self.nav_ms[..n]) {
                Some(off) => {
                    self.bit_off = Some(off);
                    self.nav_ms.drain(..off);
                    self.nav_abs_ms += off as u64;
                }
                None => return,
            }
        }
        while self.nav_ms.len() >= 20 {
            let b = crate::beidou_d1::nh_bit(&self.nav_ms[..20]);
            self.nav_bits.push(b);
            self.nav_ms.drain(..20);
            self.nav_abs_ms += 20;
        }
    }

    /// SBAS/WAAS message decode, called once per second (like nav_tick):
    /// drain the 1 ms prompt buffer into 2 ms soft symbols, feed the
    /// streaming decoder, and summarize the current decode window. Symbols
    /// accumulate regardless of lock (the decoder wants ~4 s of history at
    /// lock time); the decode itself runs only while LOCKED — an unlocked
    /// channel's prompt stream is noise and the frame-sync search over it
    /// is wasted CPU. Returns None for non-SBAS channels.
    ///
    /// APPLY-ONCE (review round 6): the decode re-runs over the RETAINED
    /// symbol window every tick, so every decoded block returns every
    /// second. Each block carries an immutable identity (its absolute
    /// stream position, DecodedMessage::sym_pos); only blocks newer than
    /// the `sbas_applied` watermark are applied to the caches — a replayed
    /// message never refreshes an insert timestamp. The summary's
    /// n_msgs/types deliberately still count the whole window (they are a
    /// decode-health diagnostic, not an insert).
    ///
    /// `t_proc` is the stream time of the most recently processed sample
    /// (the same clock Band::end_second uses for anchor freshness) and
    /// stamps every cache insert / freshness check: it advances
    /// monotonically for the life of the channel, unlike lock_s, which
    /// RESETS on RF unlock and let stale rows re-pass the freshness
    /// windows with a NEGATIVE age (review round 4). Replay-driven in
    /// tests — no system time.
    fn sbas_tick(&mut self, t_proc: f64) -> Option<SbasSummary> {
        if self.sys != Sys::Sbas {
            return None;
        }
        // Pair on the constant ABSOLUTE 2 ms grid: `par` is the absolute
        // parity (pairs hold absolute 1 ms indices (k, k+1), k ≡ par), the
        // queue-relative start `s` follows from where the queue head sits
        // (nav_abs_ms). A straddling leftover ms then pairs with the first
        // fresh prompt instead of flipping the grid for a whole second (the
        // par=1 phase slip) — and the latch/change detector below compare
        // absolute parities, so the queue-head shift no longer reads as a
        // pairing change.
        let (soft, s, par) =
            crate::sbas::symbols_from_prompt_abs(&self.nav_ms, self.sbas_par, self.nav_abs_ms);
        // drain only the consumed prompts: when the 2 ms symbol grid sits
        // at the odd parity, one straddling ms must survive into the next
        // second or one symbol per second would be lost
        let used = s + 2 * soft.len();
        self.nav_ms.drain(..used);
        self.nav_abs_ms += used as u64;
        // Generation termination on a symbol-pairing CHANGE (review round
        // 6): while the decode is locked the pairing is latched (round 5);
        // a lock loss releases the latch and the energy probe re-picks. If
        // the re-pick DIFFERS from the pairing the retained window was
        // built with, one coded symbol was inserted/deleted at the seam —
        // the stream is broken, so the whole generation (window + caches +
        // watermark) dies BEFORE the new-pairing symbols land in a fresh
        // decoder. A re-pick of the SAME pairing is seamless: no reset.
        if self.sbas_par_prev.is_some() && self.sbas_par_prev != Some(par) {
            self.sbas_reset();
        }
        self.sbas_dec.push_symbols(&soft);
        if !self.locked {
            return Some(SbasSummary {
                locked: false,
                n_msgs: 0,
                types: std::collections::BTreeMap::new(),
                fast_corr: Vec::new(),
                lt_corr: Vec::new(),
                igp_mask: Vec::new(),
                iono_delay: Vec::new(),
            });
        }
        let rep = self.sbas_dec.decode(SBAS_MIN_BLOCKS);
        // Latch the pairing that produced a locked decode (and remember it
        // as the window's pairing for the change detector above); release
        // the latch when lock drops so the next lock re-probes (round 5) —
        // the release alone does NOT break the stream, only a changed
        // re-pick does.
        if rep.sync.locked {
            self.sbas_par = Some(par);
            self.sbas_par_prev = Some(par);
        } else {
            self.sbas_par = None;
        }
        let mut types = std::collections::BTreeMap::new();
        for dm in &rep.messages {
            *types.entry(dm.message.mt()).or_insert(0usize) += 1;
            // apply-once (review round 6): only blocks newer than the
            // watermark may touch the caches. Messages ascend in sym_pos
            // within a report, so the watermark lands on the newest.
            if self.sbas_applied.map(|a| dm.sym_pos > a).unwrap_or(true) {
                self.sbas_apply(&dm.message, t_proc);
                self.sbas_applied = Some(dm.sym_pos);
            }
        }
        // publish only fresh entries (<= 60 s of stream time)
        let now = t_proc;
        let fast_corr: Vec<(u8, f64, u8, f64)> = self
            .sbas_prc
            .iter()
            .filter(|(_, (_, _, t))| now - t < 60.0)
            .map(|(&prn, &(prc_m, udrei, t))| (prn, prc_m, udrei, now - t))
            .collect();
        // long-term corrections have their own, longer validity: DO-229D
        // Table 2-1 gives a 360 s timeout for MT24/25 (en-route/terminal;
        // 240 s on approach) — the 60 s fast-corr window would drop
        // still-valid LT data
        let lt_corr: Vec<LtCorrReport> = self
            .sbas_lt
            .iter()
            .filter(|(_, (_, t))| now - t < 360.0)
            .map(|(_, (corr, t))| LtCorrReport {
                corr: corr.clone(),
                age_s: now - t,
            })
            .collect();
        // iono data has a longer life than fast corrections (DO-229
        // timeouts: 5 min for MT26, 10 min for the MT18 mask — 300 s is
        // the conservative common window)
        let igp_mask: Vec<(u8, u8, Vec<u16>)> = self
            .sbas_igpmask
            .iter()
            .filter(|(_, (_, _, t))| now - t < 300.0)
            .map(|(&band, &(iodi, ref igps, _))| (band, iodi, igps.clone()))
            .collect();
        let iono_delay: Vec<(u8, u8, u8, Vec<(u16, u8)>)> = self
            .sbas_iono
            .iter()
            .filter(|(_, (_, _, t))| now - t < 300.0)
            .map(|(&(band, block), &(iodi, rows, _))| {
                (band, block, iodi, rows.to_vec())
            })
            .collect();
        Some(SbasSummary {
            locked: rep.sync.locked,
            n_msgs: rep.messages.len(),
            types,
            fast_corr,
            lt_corr,
            igp_mask,
            iono_delay,
        })
    }

    /// Apply one decoded SBAS message to the held correction state:
    /// insert/refresh usable corrections, EVICT on don't-use, and drop the
    /// correction caches when a new mask generation arrives. `t_s` is the
    /// stream-time insert stamp (see sbas_tick).
    fn sbas_apply(&mut self, m: &crate::sbas::Message, t_s: f64) {
        match m {
            crate::sbas::Message::PrnMask { slots, iodp } => {
                // A DIFFERENT IODP is a new mask GENERATION: every
                // correction harvested under the old mask is invalid
                // (DO-229D A.4.4.2 — MT2-5/MT24/25 data is valid only
                // against the mask whose IODP it carries). Clear the
                // caches before adopting the new mask (review round 4).
                if self.sbas_mask.as_ref().map(|(_, p)| *p) != Some(*iodp) {
                    self.sbas_prc.clear();
                    self.sbas_lt.clear();
                }
                self.sbas_mask = Some((slots.clone(), *iodp));
            }
            // harvest fast corrections (MT2-5): entries address satellites
            // by ORDINAL through the MT1 mask's set bits, and are valid
            // only while the message IODP matches the mask's (DO-229D
            // A.4.4.2/A.4.4.3). No mask held -> nothing decodes. A fresh
            // UDREI >= 14 row (not monitored / don't use) EVICTS the
            // cached usable correction for that PRN — otherwise a dead
            // satellite's stale PRC stays applicable for the rest of the
            // freshness window (review round 4).
            crate::sbas::Message::Fast { first_slot, iodp, prc, udrei, .. } => {
                if let Some((mask_slots, mask_iodp)) = &self.sbas_mask {
                    for (prn, prc_m, u) in
                        crate::sbas::fast_rows(mask_slots, *mask_iodp, *iodp, *first_slot, prc, udrei)
                    {
                        if u < 14 {
                            self.sbas_prc.insert(prn, (prc_m, u, t_s));
                        } else {
                            self.sbas_prc.remove(&prn);
                        }
                    }
                }
            }
            // MT6 integrity (DO-229D A.4.4.4): UDREI per mask ORDINAL
            // 1..=51 — the same through-mask addressing as the MT2-5
            // entries. MT6 carries no IODP; it is applied against the
            // currently held mask. APPROXIMATIONS, honestly scoped: the
            // four IODF fields sequence these UDREIs against the IODFs of
            // the MT2-5 messages — that sequencing is NOT tracked, so
            // every MT6 is treated as refreshing all 51 slots wholesale;
            // and only the don't-use half is enforced (UDREI >= 14 evicts
            // the satellite's cached fast AND long-term corrections — a
            // not-monitored/don't-use satellite has no usable corrections
            // at all). Degraded-but-usable UDREI updates do not rewrite
            // cached rows.
            crate::sbas::Message::Integrity { udrei, .. } => {
                let evict: Vec<u8> = match &self.sbas_mask {
                    Some((slots, _)) => udrei
                        .iter()
                        .enumerate()
                        .filter(|&(_, &u)| u >= 14)
                        .filter_map(|(i, _)| slots.get(i).copied())
                        .filter(|s| (1..=37).contains(s))
                        .collect(),
                    None => Vec::new(),
                };
                for slot in evict {
                    self.sbas_prc.remove(&slot);
                    self.sbas_lt.remove(&slot);
                }
            }
            // harvest long-term corrections (MT25 halves; MT24 long-term
            // slot): corrected sat position/clock = broadcast + delta
            // propagated to the current epoch (the consumer runs
            // LtCorr::propagate; DO-229D A.4.4.7 eq. A-18/A-19); same
            // ordinal-through-mask addressing and IODP gate as the fast
            // corrections (DO-229D A.4.4.7)
            crate::sbas::Message::LongTerm { a, b } => {
                if let Some((mask_slots, mask_iodp)) = &self.sbas_mask {
                    for h in [a, b] {
                        for corr in crate::sbas::lt_corrections(mask_slots, *mask_iodp, h) {
                            self.sbas_lt.insert(corr.prn, (corr, t_s));
                        }
                    }
                }
            }
            crate::sbas::Message::MixedFastLongTerm { lt, .. } => {
                if let Some((mask_slots, mask_iodp)) = &self.sbas_mask {
                    for corr in crate::sbas::lt_corrections(mask_slots, *mask_iodp, lt) {
                        self.sbas_lt.insert(corr.prn, (corr, t_s));
                    }
                }
            }
            crate::sbas::Message::IonoMask { band, iodi, igps, .. } => {
                self.sbas_igpmask
                    .insert(*band, (*iodi, igps.clone(), t_s));
            }
            crate::sbas::Message::IonoDelay { band, block_id, iodi, igps } => {
                self.sbas_iono
                    .insert((*band, *block_id), (*iodi, *igps, t_s));
            }
            _ => {}
        }
    }

    /// diagnostics for examples/dbg_live.rs
    pub fn debug_refine(&mut self, sig: &[Complex<f32>]) -> f64 {
        self.refine_dopp(sig, 250.0, 25.0, 100)
    }
    pub fn debug_epoch(&mut self, sig: &[Complex<f32>]) -> (f64, f64, f64) {
        self.process_epoch(sig)
    }
    pub fn debug_dopp(&self) -> f64 {
        self.carrier_freq
    }
    pub fn debug_prompt(&self, sig: &[Complex<f32>], dopp: f64, cp: f64) -> f64 {
        self.prompt_only(sig, dopp, cp)
    }

    /// rho_m/t_tx may be published only while the anchor is fresh — see
    /// ANCHOR_MAX_AGE_S. A locked channel whose nav pipeline stopped still
    /// has loops and a stored anchor; only the refresh timestamp tells the
    /// difference between a measurement and a frozen number.
    fn anchor_fresh(&self, t_proc: f64) -> bool {
        self.anchor.is_some() && t_proc - self.anchor_t <= ANCHOR_MAX_AGE_S
    }

    /// Close out one tracked second: C/N0 proxy + lock state machine.
    /// Returns (cn0_proxy, doppler_hz, code_phase_chips).
    fn end_second(&mut self) -> (f64, f64, f64) {
        let p = self.sec_prompt / self.sec_epochs.max(1) as f64;
        let n = (self.sec_noise / self.sec_epochs.max(1) as f64).max(1e-12);
        let pdi = self.ns_epoch as f64 / self.fs;
        // narrowband-power estimator: (prompt - noise)/noise per unit time.
        // Pure noise averages to ~0 (not a +3 dB floor like the quadrature
        // estimator), so the lock threshold has real margin.
        let cn0 = 10.0 * ((p - n).max(1e-12) / n).log10() + 10.0 * (1.0 / pdi).log10();
        self.sec_prompt = 0.0;
        self.sec_noise = 0.0;
        self.sec_epochs = 0;
        self.cn0_ema = if self.last_cn0 == 0.0 && self.cn0_ema == 0.0 {
            cn0
        } else {
            0.5 * self.cn0_ema + 0.5 * cn0
        };
        self.last_cn0 = cn0;
        let v = self.cn0_ema;
        if self.locked {
            if v < UNLOCK_DB {
                self.below += 1;
                if self.below >= UNLOCK_SECS {
                    self.locked = false;
                    self.lock_s = 0.0;
                    self.below = 0;
                    // lock watchdog fired: the phase chain is broken — flag
                    // a slip for this report second
                    self.slip = true;
                    self.slip_count += 1;
                    // RF unlock breaks the SBAS symbol stream: the decode
                    // generation (window + correction caches + apply
                    // watermark) dies with it (review round 6)
                    self.sbas_reset();
                }
            } else {
                self.below = 0;
            }
            if self.locked {
                self.lock_s += 1.0;
            }
        } else if v >= LOCK_DB {
            self.above += 1;
            if self.above >= LOCK_SECS {
                self.locked = true;
                self.lock_s = 1.0;
                self.above = 0;
            }
        } else {
            self.above = 0;
        }
        if self.locked {
            self.lost_s = 0.0;
        } else {
            self.lost_s += 1.0;
        }
        (self.cn0_ema, self.carrier_freq, self.code_phase)
    }

    /// Take the slip flag for the report being emitted (clears it for the
    /// next second).
    fn take_slip(&mut self) -> bool {
        std::mem::take(&mut self.slip)
    }

    /// Re-seed the loops from a re-acquisition (keeps identity + lock stats
    /// reset; called after a long unlock).
    fn reseed(&mut self, dopp0: f64, code_phase0: f64) {
        self.carrier_freq = dopp0;
        self.carr_basis = dopp0;
        self.carr_nco = 0.0;
        self.old_carr_err = 0.0;
        self.code_phase = code_phase0.rem_euclid(self.code_len);
        // the phase chain is broken: the accumulator re-zeros (zero is
        // defined at channel start) and the break is flagged — but not on
        // the initial install right after Channel::new (no phase existed)
        if self.carr_cycles != 0.0 {
            self.slip = true;
            self.slip_count += 1;
        }
        self.carr_cycles = 0.0;
        self.above = 0;
        self.below = 0;
        // the reseed also breaks the SBAS symbol stream: a fresh decoder
        // and a released parity latch (review round 5 — stale state would
        // otherwise decode across the break); review round 6: the whole
        // decode generation dies together, correction caches included
        self.sbas_reset();
    }

    /// Terminate the SBAS decode generation (review round 6): the decoder
    /// window, the parity latch, the apply-once watermark, and every
    /// harvested correction cache (fast, long-term, mask, iono) die
    /// together. Any break in the 500 sym/s symbol stream — RF unlock, a
    /// symbol-pairing change, reseed, input gap — invalidates both the
    /// retained window AND the corrections harvested from it; the next
    /// lock starts clean instead of replaying or trusting the old one.
    fn sbas_reset(&mut self) {
        self.sbas_dec = crate::sbas::Decoder::new();
        self.sbas_par = None;
        self.sbas_par_prev = None;
        self.sbas_applied = None;
        // the absolute pairing grid dies with the generation: the next lock
        // re-probes from a fresh origin (nav_ms content across the break is
        // discontinuous or redefined, so the old grid origin is meaningless)
        self.nav_abs_ms = 0;
        self.sbas_prc.clear();
        self.sbas_mask = None;
        self.sbas_lt.clear();
        self.sbas_igpmask.clear();
        self.sbas_iono.clear();
    }

    /// Shift the carrier loops by `df` Hz WITHOUT touching lock state.
    /// Used when the reference clock correction steps: the LO moves, so the
    /// measured Doppler of every channel moves by the same amount — telling
    /// the channels converts an instant lock-killer (a 157 Hz step into a
    /// 10 Hz FLL) into seamless tracking through the discipline step.
    fn shift_dopp(&mut self, df: f64) {
        self.carrier_freq += df;
        self.carr_basis += df;
    }
}

/// One published long-term correction row: the harvested LtCorr (physical
/// units — serde-flattened into the same JSON object) plus the insert age
/// in seconds of stream time. vc=1 rows carry the rates and t_lt the
/// consumer needs for solve-time propagation (DO-229D A.4.4.7 eq.
/// A-18/A-19); for vc=0 rows those fields serialize as null.
#[derive(Debug, Clone, serde::Serialize)]
pub struct LtCorrReport {
    #[serde(flatten)]
    pub corr: crate::sbas::LtCorr,
    pub age_s: f64,
}

/// Compact SBAS/WAAS decode summary, published on Sys::Sbas SatReports.
/// tracker_producer.py passes per-PRN report dicts straight through into
/// state.tracker.json, so this rides along with no producer change.
#[derive(Debug, Clone, serde::Serialize)]
pub struct SbasSummary {
    /// frame sync locked on the current decode window (>= SBAS_MIN_BLOCKS
    /// consecutive, correctly-rotating CRC-valid blocks)
    pub locked: bool,
    /// WAAS messages decoded from the current window
    pub n_msgs: usize,
    /// per-message-type counts in the current window (DO-229 MT -> n)
    pub types: std::collections::BTreeMap<u8, usize>,
    /// Latest fast corrections (MT2-5) held by this channel:
    /// (GPS PRN, PRC metres, UDREI, insert age in seconds of stream time),
    /// decoded by ordinal through the current MT1 mask and only while
    /// IODPs match. UDREI >= 14 rows are never included
    /// (not-monitored/don't-use) — a fresh don't-use row EVICTS the cached
    /// entry instead. Entries older than 60 s of stream time are
    /// dropped (conservative vs the MT7 degradation model, not yet
    /// implemented). Empty when none decoded.
    pub fast_corr: Vec<(u8, f64, u8, f64)>,
    /// Latest long-term corrections (MT24/25) held by this channel, one
    /// JSON object per row: the flattened sbas::LtCorr (GPS PRN; dx, dy,
    /// dz metres and daf0 seconds at t_lt; vc=1 rates ddx/ddy/ddz in m/s
    /// and daf1 in s/s plus the t_lt_s time-of-day applicability — null
    /// for vc=0; IOD) plus age_s (insert age in seconds of stream time).
    /// The consumer propagates at solve time: corrected satellite
    /// position = broadcast + (δx + δẋ·(t−t_lt), ...) and corrected
    /// satellite clock offset = broadcast + δaf0 + δaf1·(t−t_lt)
    /// (DO-229D A.4.4.7 eq. A-18/A-19; vc=0 rows are constants). IOD is
    /// the GPS IODE of the ephemeris the correction was generated against
    /// (DO-229D Table A-10 Note 3) — apply only when it matches the
    /// ephemeris in use. 360 s freshness window (DO-229D Table 2-1
    /// MT24/25 timeout).
    pub lt_corr: Vec<LtCorrReport>,
    /// Latest iono grid masks (MT18): (band, iodi, IGP numbers). 300 s
    /// freshness (DO-229 mask timeout is 10 min).
    pub igp_mask: Vec<(u8, u8, Vec<u16>)>,
    /// Latest iono delay blocks (MT26): (band, block_id, iodi, 15 x
    /// (vertical-delay counts, GIVEI)). Delay scale 0.125 m, 511 counts
    /// = not monitored. 300 s freshness (DO-229 MT26 timeout is 5 min).
    pub iono_delay: Vec<(u8, u8, u8, Vec<(u16, u8)>)>,
}

/// Per-PRN 1 Hz report — serialized to JSON by the front end.
#[derive(Debug, Clone, serde::Serialize)]
pub struct SatReport {
    pub prn: usize,
    pub sys: &'static str,
    pub doppler_hz: f64,
    pub cn0_proxy: f64,
    pub code_phase: f64,
    pub lock_s: f64,
    pub epoch: f64,
    /// decoded nav bits (0/1), polarity unresolved (Costas) — lnav handles it
    pub nav_bits: usize,
    /// parity-valid LNAV subframes decoded from those bits
    pub nav_subs: usize,
    /// true pseudorange in metres (Some once a TOW anchor exists)
    pub rho_m: Option<f64>,
    /// GPS transmit time-of-week of the anchor (s)
    pub t_tx: Option<f64>,
    /// Integrated replica carrier phase in cycles, zero at channel (re)seed.
    /// Continuous across reports while the loop holds; its rate IS the
    /// Doppler. The absolute value carries the Costas 180 deg ambiguity —
    /// only differences between reports are physical. Meaningless while
    /// unlocked (the loop free-runs); check lock_s / slip.
    pub carrier_cycles: f64,
    /// Fractional carrier phase at the report instant in cycles, published
    /// modulo the data-bit half-cycle (the Costas 180 deg ambiguity makes
    /// the full cycle unobservable): always in [0, 0.5).
    pub phase_frac: f64,
    /// A phase break happened this second (lock watchdog fired or the
    /// channel was re-seeded — carrier_cycles re-zeroed).
    pub slip: bool,
    /// WAAS message decode summary (Sys::Sbas rows only; absent otherwise).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sbas_msgs: Option<SbasSummary>,
}

/// Acquisition runs on a WORKER THREAD, never on the consumer: any
/// acquisition burst (full seed ~1 min, narrow align ~seconds) is longer
/// than the input queue can absorb, so a consumer-side acquisition forces
/// dropped input, and every drop kills the channels' phases. The consumer
/// snapshots a buffer, hands it to the worker, and keeps riding the live
/// edge; results arrive with code phases extrapolated to install time
/// (carrier-aided code rate — good to ~0.1 chip over worker runtimes).
#[derive(Debug, Clone, Copy)]
enum SeedState {
    /// Accumulate a 2 s snapshot, then spawn the full-seed worker.
    NeedSeed,
    /// Seed worker running; discard input meanwhile.
    SeedWait,
    /// Accumulate a 1 s snapshot, then spawn the narrow-align worker.
    NeedAlign,
    /// Align worker running; discard input meanwhile.
    AlignWait,
    /// Channels live; normal drain.
    Tracking,
}

enum WorkerMsg {
    /// Full-seed result: (sys, prn, doppler) candidates (Doppler is valid
    /// for minutes; code phase is NOT — that is what alignment is for).
    Seeded(Vec<(Sys, usize, f64)>),
    /// Align result: (sys, prn, doppler, code phase at snapshot end), and
    /// the band input-time of the snapshot end for install extrapolation.
    Aligned(Vec<(Sys, usize, f64, f64)>, f64),
}

/// Acquisition workloads get their OWN rayon pool, capped below the core
/// count. They used to run on the global pool: its minute-long tasks
/// starved the consumer's small join/par tasks (rayon does not preempt),
/// the consumer fell behind the 32 MB/s stream, the input queue overflowed,
/// and the resulting gaps killed the channels — a self-sustaining loop.
fn acq_pool() -> &'static rayon::ThreadPool {
    static POOL: std::sync::OnceLock<rayon::ThreadPool> = std::sync::OnceLock::new();
    POOL.get_or_init(|| {
        let cores = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(4);
        // Well below the core count: the consumer thread (front-end +
        // tracking) must keep real-time even while a worker runs, or the
        // input queue overflows and the resulting gap kills every channel.
        rayon::ThreadPoolBuilder::new()
            .num_threads(cores.saturating_sub(6).max(2))
            .thread_name(|i| format!("acq-{i}"))
            .build()
            .expect("acquisition thread pool")
    })
}

/// One acquisition worker at a time GLOBALLY (both bands): two concurrent
/// seed bursts saturate the pool and starve the consumer. Workers block on
/// this inside their own thread, so the consumer never waits.
static WORKER_SLOT: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// One decimated baseband stream and its channels. `fs` must be a rate where
/// every system's code period lands on a whole number of samples (4 Msps:
/// 4000 samples per 1 ms, 16000 per 4 ms).
pub struct Band {
    pub name: &'static str,
    pub fs: f64,
    /// Pending samples as Vec + read cursor: a VecDeque needed
    /// make_contiguous() (a full-buffer memmove) on every pump once it
    /// wrapped — 100+ MB memcpys per push under backlog, the consumer's
    /// top hotspot. The cursor makes consumption pointer arithmetic;
    /// compaction is amortized.
    buf: Vec<Complex<f32>>,
    pos: usize,
    hist: VecDeque<Complex<f32>>,
    pub channels: Vec<Channel>,
    state: SeedState,
    /// Seeding acquisition runs so far (>= 1 means decimated data reached
    /// the seeding stage; retries stay un-seeded until something acquires).
    pub seed_attempts: u32,
    /// Seed candidates awaiting alignment on fresh data: (sys, prn, dopp0).
    candidates: Vec<(Sys, usize, f64)>,
    worker: Option<std::sync::mpsc::Receiver<WorkerMsg>>,
    /// Seconds of band-domain input received since creation (counts ALL
    /// pushes, including discarded ones) — the install-time extrapolation
    /// reference.
    in_t: f64,
    /// Consecutive empty align passes; after a few, fall back to a full
    /// re-seed (the candidate Dopplers have presumably gone stale/bogus).
    align_fails: u32,
    /// in_t of the next background discovery seed while Tracking (the seed
    /// cache primes a subset of the sky; without rediscovery the tracker
    /// would sit on that subset forever).
    discover_at: f64,
    seed_need: usize,
    t_s: f64,
    next_report_s: u64,
    epoch0: f64,
}

impl Band {
    /// L1 band: GPS C/A + SBAS + Galileo E1B (all share 1575.42 MHz).
    pub fn new_l1(fs: f64, epoch0: f64) -> Band {
        Band::new("l1", fs, epoch0)
    }

    /// BeiDou B1I band (1561.098 MHz).
    pub fn new_b1i(fs: f64, epoch0: f64) -> Band {
        Band::new("b1i", fs, epoch0)
    }

    fn new(name: &'static str, fs: f64, epoch0: f64) -> Band {
        Band {
            name,
            fs,
            buf: Vec::new(),
            pos: 0,
            hist: VecDeque::new(),
            channels: Vec::new(),
            state: SeedState::NeedSeed,
            seed_attempts: 0,
            candidates: Vec::new(),
            worker: None,
            in_t: 0.0,
            align_fails: 0,
            discover_at: f64::INFINITY, // armed when Tracking begins
            seed_need: (SEED_S * fs).round() as usize,
            t_s: 0.0,
            next_report_s: 1,
            epoch0,
        }
    }

    pub fn seeded(&self) -> bool {
        matches!(self.state, SeedState::Tracking)
    }

    /// Compact state readout for the live_track status line.
    pub fn status(&self) -> String {
        format!(
            "{:?}/cand{}/seed{}",
            self.state,
            self.candidates.len(),
            self.seed_attempts
        )
    }

    /// True while an acquisition worker runs off-thread (tests poll this).
    pub fn worker_active(&self) -> bool {
        self.worker.is_some()
    }

    /// Unprocessed samples.
    fn remaining(&self) -> &[Complex<f32>] {
        &self.buf[self.pos..]
    }

    /// Drop all pending samples (state transitions, gaps).
    fn clear_buf(&mut self) {
        self.buf.clear();
        self.pos = 0;
    }

    /// Advance the cursor; compact when the dead prefix is worth moving.
    fn consume_buf(&mut self, n: usize) {
        self.pos += n;
        if self.pos == self.buf.len() {
            self.buf.clear();
            self.pos = 0;
        } else if self.pos > 4_000_000 {
            self.buf.drain(..self.pos);
            self.pos = 0;
        }
    }

    /// Push decimated baseband samples; returns the 1 Hz reports for every
    /// 1-s boundary crossed while draining the buffer.
    pub fn push(&mut self, sig: &[Complex<f32>]) -> Vec<SatReport> {
        self.in_t += sig.len() as f64 / self.fs;
        self.buf.extend_from_slice(sig);
        self.pump()
    }

    /// Advance the stream-time clock past dropped input (in band-domain
    /// samples). See Engine::note_gap_bytes.
    pub fn note_gap(&mut self, band_samples: u64) {
        self.in_t += band_samples as f64 / self.fs;
        // The gap drops samples: no channel's SBAS symbol stream crosses
        // it, so every decode generation (retained window + correction
        // caches + apply watermark) dies here (review round 6). Channel
        // phases are invalidated separately (force_reseed on big gaps).
        for ch in self.channels.iter_mut() {
            ch.sbas_reset();
        }
        // A partial seed/align accumulation that spans the gap is corrupt
        // (acquisition needs a coherent snapshot) — restart it on fresh
        // data. Observed live: gappy align snapshots failed 3x and forced
        // needless full re-seeds.
        if matches!(self.state, SeedState::NeedSeed | SeedState::NeedAlign) {
            self.clear_buf();
        }
    }

    /// Poll the acquisition worker without new data (tests waiting on a
    /// worker with a finite fixture call this in a sleep loop).
    pub fn poll(&mut self) {
        self.worker_msg();
    }

    /// Pick up a finished worker's result and advance the state machine.
    fn worker_msg(&mut self) {
        let Some(rx) = &self.worker else { return };
        let msg = match rx.try_recv() {
            Ok(m) => m,
            Err(std::sync::mpsc::TryRecvError::Empty) => return,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                // worker panicked: don't wedge in *Wait forever
                self.worker = None;
                self.state = match self.state {
                    SeedState::SeedWait => SeedState::NeedSeed,
                    SeedState::AlignWait => SeedState::NeedAlign,
                    s => s,
                };
                return;
            }
        };
        self.worker = None;
        match msg {
            WorkerMsg::Seeded(cands) => {
                eprintln!("live[{}]: seed done — {} candidates", self.name, cands.len());
                if matches!(self.state, SeedState::Tracking) {
                    // background discovery: keep the locks, align only the
                    // NEWLY found PRNs from fresh history
                    let new: Vec<_> = cands
                        .into_iter()
                        .filter(|c| {
                            !self.channels.iter().any(|ch| ch.sys == c.0 && ch.prn == c.1)
                        })
                        .collect();
                    eprintln!("live[{}]: discovery — {} new candidates", self.name, new.len());
                    if !new.is_empty() {
                        self.spawn_align(new);
                    }
                    return;
                }
                if cands.is_empty() {
                    self.state = SeedState::NeedSeed; // dead air: retry
                } else {
                    self.candidates = cands;
                    self.state = SeedState::NeedAlign;
                }
            }
            WorkerMsg::Aligned(found, t_end) => {
                // Extrapolate to the stream time of the NEXT samples to be
                // processed (buf start), not the latest arrival (buf end).
                let t_next = self.in_t - self.remaining().len() as f64 / self.fs;
                let dt = (t_next - t_end).max(0.0);
                eprintln!(
                    "live[{}]: align done — {}/{} found, dt {:.2} s",
                    self.name,
                    found.len(),
                    self.candidates.len(),
                    dt
                );
                let was_tracking = matches!(self.state, SeedState::Tracking);
                for (sys, prn, dopp, cp_end) in found {
                    let (chip_rate, f_carrier, code_len) = sys_params(sys);
                    // extrapolate the code phase from the snapshot end to
                    // NOW (carrier-aided code rate; ~0.1 chip over worker
                    // runtimes)
                    let cp = (cp_end + chip_rate * (1.0 + dopp / f_carrier) * dt)
                        .rem_euclid(code_len);
                    let mut ch = Channel::new(sys, prn, self.fs, dopp, 0.0);
                    ch.reseed(dopp, cp);
                    // a re-acquire of an already-tracked PRN REPLACES the
                    // lost channel, never duplicates it
                    self.channels.retain(|c| !(c.sys == sys && c.prn == prn));
                    self.channels.push(ch);
                }
                if was_tracking {
                    return; // discovery merge: state untouched, locks intact
                }
                if self.channels.is_empty() {
                    self.align_fails += 1;
                    self.state = if self.align_fails >= 3 {
                        SeedState::NeedSeed // candidates hopeless: full re-seed
                    } else {
                        SeedState::NeedAlign
                    };
                } else {
                    self.align_fails = 0;
                    self.state = SeedState::Tracking;
                    self.discover_at = self.in_t + 900.0; // next sky rediscovery
                }
            }
        }
    }

    /// Spawn an align worker over a fresh 1 s snapshot (from history when
    /// Tracking, else the accumulation buffer) for the given candidates.
    fn spawn_align(&mut self, cands: Vec<(Sys, usize, f64)>) {
        let one_s = self.fs.round() as usize;
        let tracking = matches!(self.state, SeedState::Tracking);
        let (snap, t_end): (Vec<Complex<f32>>, f64) = if tracking {
            if self.hist.len() < one_s {
                return;
            }
            // hist end == last processed sample == in_t - pending buffer
            let t = self.in_t - self.remaining().len() as f64 / self.fs;
            let hlen = self.hist.len();
            let h = self.hist.make_contiguous()[hlen - one_s..].to_vec();
            (h, t)
        } else {
            if self.remaining().len() < one_s {
                return;
            }
            let snap: Vec<Complex<f32>> = self.remaining()[..one_s].to_vec();
            let overshoot = self.remaining().len() - one_s;
            let t = self.in_t - overshoot as f64 / self.fs;
            self.clear_buf();
            (snap, t)
        };
        let snap_duration = snap.len() as f64 / self.fs;
        let (tx, rx) = std::sync::mpsc::channel();
        let fs = self.fs;
        self.candidates = cands.clone();
        std::thread::spawn(move || {
            // aligns are SHORT (seconds) and latency-critical (their
            // code phases age in real time) — they bypass the global
            // worker slot, which exists to serialise minutes-long
            // full seeds
            let found = acq_pool().install(|| {
                cands
                    .par_iter()
                    .filter_map(|&(sys, prn, dopp0)| {
                        let grid: Vec<f64> =
                            (-3..=3).map(|i| dopp0 + i as f64 * 50.0).collect();
                        reacquire_on(fs, &snap, sys, prn, &grid, snap_duration)
                            .map(|(dopp, cp)| (sys, prn, dopp, cp))
                    })
                    .collect::<Vec<_>>()
            });
            let _ = tx.send(WorkerMsg::Aligned(found, t_end));
        });
        self.worker = Some(rx);
        if !tracking {
            self.state = SeedState::AlignWait;
        }
    }

    fn pump(&mut self) -> Vec<SatReport> {
        self.worker_msg();
        match self.state {
            SeedState::NeedSeed => {
                if self.remaining().len() < self.seed_need {
                    return Vec::new();
                }
                let snap: Vec<Complex<f32>> = self.remaining()[..self.seed_need].to_vec();
                self.clear_buf();
                self.seed_attempts += 1;
                let (tx, rx) = std::sync::mpsc::channel();
                let (name, fs) = (self.name, self.fs);
                std::thread::spawn(move || {
                    let _slot = WORKER_SLOT.lock().unwrap();
                    let cands = acq_pool().install(|| seed_snapshot(name, fs, &snap));
                    let _ = tx.send(WorkerMsg::Seeded(cands));
                });
                self.worker = Some(rx);
                self.state = SeedState::SeedWait;
                Vec::new()
            }
            SeedState::SeedWait => {
                self.clear_buf();
                Vec::new()
            }
            SeedState::NeedAlign => {
                let one_s = self.fs.round() as usize;
                if self.remaining().len() < one_s || self.worker.is_some() {
                    // worker.is_some(): a discovery align is in flight — its
                    // result IS the recovery; spawning another would orphan it
                    return Vec::new();
                }
                let cands = self.candidates.clone();
                self.spawn_align(cands);
                Vec::new()
            }
            SeedState::AlignWait => {
                self.clear_buf();
                Vec::new()
            }
            SeedState::Tracking => self.pump_tracking(),
        }
    }

    fn pump_tracking(&mut self) -> Vec<SatReport> {
        // background sky rediscovery: the seed cache primes a SUBSET of the
        // sky; without this the tracker would sit on that subset forever
        if self.in_t >= self.discover_at && self.worker.is_none() {
            self.discover_at = self.in_t + 900.0;
            let two_s = (2.0 * self.fs).round() as usize;
            if self.hist.len() >= two_s {
                let hlen = self.hist.len();
                let snap: Vec<Complex<f32>> =
                    self.hist.make_contiguous()[hlen - two_s..].to_vec();
                self.seed_attempts += 1;
                let (tx, rx) = std::sync::mpsc::channel();
                let (name, fs) = (self.name, self.fs);
                std::thread::spawn(move || {
                    let _slot = WORKER_SLOT.lock().unwrap();
                    let cands = acq_pool().install(|| seed_snapshot(name, fs, &snap));
                    let _ = tx.send(WorkerMsg::Seeded(cands));
                });
                self.worker = Some(rx);
            }
        }
        let slice = (4.0 * self.fs / 1000.0).round() as usize; // 4 ms super-epoch
        let hist_cap = (HIST_S * self.fs).round() as usize;
        let mut reports = Vec::new();
        // Process slices DIRECTLY from the contiguous buffer: the previous
        // drain(..slice).collect() per 4 ms slice allocated + copied 128 KB
        // 250x/s/band and was the consumer's main-thread hotspot (found via
        // `sample` — not the DSP, the buffer shuffling). Slices run only up
        // to the next 1-s boundary at a time so end_second can take &mut
        // self after the buffer borrow is released.
        while self.remaining().len() >= slice {
            let avail = self.remaining().len() / slice;
            let to_boundary = ((self.next_report_s as f64 - self.t_s) * self.fs
                / slice as f64)
                .ceil()
                .max(1.0) as usize;
            let n_slices = avail.min(to_boundary);
            let total = n_slices * slice;
            {
                let contig = &self.buf[self.pos..];
                for s in 0..n_slices {
                    let chunk = &contig[s * slice..(s + 1) * slice];
                    // channels are independent: parallel over the slice
                    self.channels.par_iter_mut().for_each(|ch| {
                        let mut off = 0;
                        while off + ch.ns_epoch <= slice {
                            ch.process_epoch(&chunk[off..off + ch.ns_epoch]);
                            off += ch.ns_epoch;
                        }
                    });
                    self.t_s += slice as f64 / self.fs;
                    self.hist.extend(chunk.iter().copied());
                    if self.hist.len() > hist_cap {
                        let over = self.hist.len() - hist_cap;
                        self.hist.drain(..over);
                    }
                }
            }
            self.consume_buf(total);
            while self.t_s >= self.next_report_s as f64 - 1e-9 {
                let epoch = self.epoch0 + self.next_report_s as f64;
                self.next_report_s += 1;
                reports.extend(self.end_second(epoch));
            }
        }
        reports
    }

    /// Per-second wrap-up for every channel + re-acquire / drop policy.
    fn end_second(&mut self, epoch: f64) -> Vec<SatReport> {
        let mut out = Vec::with_capacity(self.channels.len());
        // stream time of the most recently processed sample
        let t_proc = self.in_t - self.remaining().len() as f64 / self.fs;
        for ch in &mut self.channels {
            let (cn0, dopp, cp) = ch.end_second();
            // carrier phase observable: carr_cycles is the unwrapped
            // replica phase in cycles (carrier_phase is it mod 1); the
            // Costas 180 deg ambiguity caps the publishable fractional
            // phase at the data-bit half-cycle
            let carrier_cycles = (ch.carr_cycles * 1e6).round() / 1e6;
            let phase_frac = (ch.carr_cycles.rem_euclid(0.5) * 1e9).round() / 1e9;
            let slip = ch.take_slip();
            ch.track_comb(t_proc);
            ch.nav_tick();
            let sbas_msgs = ch.sbas_tick(t_proc);
            // scan for new subframes since the last scan (the overlap keeps
            // the LAST validated subframe findable, so the anchor re-anchors
            // to it every second between validations rather than freezing —
            // see GPS_RESCAN_BACK/D1_RESCAN_BACK)
            let nav_subs = {
                let from = ch.nav_scanned.saturating_sub(match ch.sys {
                    Sys::Gps => GPS_RESCAN_BACK,
                    Sys::Beidou => D1_RESCAN_BACK,
                    _ => 300,
                });
                match ch.sys {
                    Sys::Gps => {
                        let subs = crate::gps::lnav::find_subframes(&ch.nav_bits[from..]);
                        let n = subs.len();
                        if let Some(last) = subs.last() {
                            // newest subframe -> refresh the pseudorange anchor
                            let abs_bit = from + last.bit_index;
                            let t_tx = (last.tow_next as f64 - 1.0) * 6.0;
                            let t_bit = anchor_stream_time(ch, t_proc, abs_bit, t_tx);
                            ch.anchor = Some((t_bit, t_tx));
                            ch.anchor_t = t_proc;
                            ch.nav_scanned = abs_bit + 300;
                            // self-decoded ephemeris once subframes 1-3 exist
                            // (scan the FULL buffer — 1/2/3 may predate this window)
                            if ch.eph.is_none() {
                                let all = crate::gps::lnav::find_subframes(&ch.nav_bits);
                                if let Some(mut e) = crate::gps::lnav::parse_ephemeris(&all) {
                                    e.prn = ch.prn as u8;
                                    eprintln!("live[{}]: self-decoded ephemeris for PRN {}", self.name, ch.prn);
                                    ch.eph = Some(e);
                                }
                            }
                        }
                        n
                    }
                    Sys::Beidou => {
                        let subs = crate::beidou_d1::find_subframes(&ch.nav_bits[from..]);
                        let n = subs.len();
                        if let Some(last) = subs.last() {
                            let abs_bit = from + last.bit_index;
                            // D1 SOW is the BDT second-of-week at THIS
                            // subframe's preamble leading edge (unlike the
                            // GPS HOW, which names the next one). t_tx is
                            // carried in GPST for all constellations:
                            // BDT + 14 s.
                            let t_tx = crate::beidou_d1::sow_bdt_to_gpst(last.sow_bdt as f64);
                            let t_bit = anchor_stream_time(ch, t_proc, abs_bit, t_tx);
                            ch.anchor = Some((t_bit, t_tx));
                            ch.anchor_t = t_proc;
                            ch.nav_scanned = abs_bit + 300;
                            if ch.eph.is_none() {
                                let all = crate::beidou_d1::find_subframes(&ch.nav_bits);
                                if let Some(mut e) = crate::beidou_d1::parse_ephemeris(&all) {
                                    e.prn = ch.prn as u8;
                                    eprintln!("live[{}]: self-decoded D1 ephemeris for BDS PRN {}", self.name, ch.prn);
                                    ch.eph = Some(e);
                                }
                            }
                        }
                        n
                    }
                    _ => 0,
                }
            };
            // publish the anchor only while the channel is locked AND
            // the anchor is fresh: the anchor otherwise freezes at its
            // last value and a dead or nav-starved channel keeps
            // reporting a stale pseudorange (observed live: unlocked
            // sats with lock_s 0 carrying a constant rho_m for hours;
            // and a LOCKED channel whose subframes stopped validating
            // diverging at its range rate, 147-885 ns/s).
            let anchor_fresh = ch.anchor_fresh(t_proc);
            out.push(SatReport {
                prn: ch.prn,
                sys: ch.sys.name(),
                doppler_hz: (dopp * 10.0).round() / 10.0,
                cn0_proxy: (cn0 * 10.0).round() / 10.0,
                code_phase: (cp * 100.0).round() / 100.0,
                lock_s: ch.lock_s,
                nav_bits: ch.nav_bits.len(),
                nav_subs,
                rho_m: if ch.locked && anchor_fresh {
                    ch.anchor.map(|(t_bit, t_tx)| (t_bit - t_tx) * 299_792_458.0)
                } else {
                    None
                },
                t_tx: if ch.locked && anchor_fresh {
                    ch.anchor.map(|(_, t_tx)| t_tx)
                } else {
                    None
                },
                carrier_cycles,
                phase_frac,
                slip,
                epoch,
                sbas_msgs,
            });
        }
        // re-acquire channels whose lock has been lost for REACQ_S, drop
        // them after DROP_S. The re-acquisition runs OFF the consumer thread
        // (a full-grid single-PRN acquire is seconds of CPU — inline it
        // stalled the consumer for seconds at a time, overflowed the input
        // queue, and the resulting gaps killed every other channel: the
        // reacq storm WAS the gap storm). The align worker reseeds the
        // channels in place via the Tracking merge.
        let mut reacq: Vec<(Sys, usize, f64)> = Vec::new();
        self.channels.retain(|ch| {
            if ch.lost_s > DROP_S {
                return false;
            }
            if ch.lost_s >= REACQ_S && (ch.lost_s % REACQ_S) < 1.0 {
                reacq.push((ch.sys, ch.prn, ch.debug_dopp()));
            }
            true
        });
        if !reacq.is_empty() && self.worker.is_none() {
            self.spawn_align(reacq);
        }
        out
    }

    /// Per-channel true pseudoranges from TOW/SOW anchors: (prn, rho_m,
    /// t_tx_s). rho = (t_bit - t_tx) * c; the arbitrary stream-time origin is
    /// common to all channels and is absorbed by the PVT clock term. GPS and
    /// BeiDou (BDS t_tx is converted to GPST at the anchor: BDT + 14 s).
    pub fn nav_obs(&self) -> Vec<(u8, f64, f64)> {
        const C: f64 = 299_792_458.0;
        // same freshness gate as the SatReport path: a stale anchor is a
        // frozen pseudorange, not a measurement
        let t_proc = self.in_t - self.remaining().len() as f64 / self.fs;
        self.channels
            .iter()
            .filter_map(|ch| {
                let (t_bit, t_tx) = ch.anchor?;
                if !ch.anchor_fresh(t_proc) {
                    return None;
                }
                Some((ch.prn as u8, (t_bit - t_tx) * C, t_tx))
            })
            .collect()
    }

    /// Skip the blind all-sky seed: start in NeedAlign with candidates from
    /// a recent seed cache (restart recovery in ~15 s instead of minutes).
    /// Only meaningful right after construction.
    pub fn prime_candidates(&mut self, cands: Vec<(Sys, usize, f64)>) {
        if cands.is_empty() || !self.channels.is_empty() {
            return;
        }
        self.candidates = cands;
        self.state = SeedState::NeedAlign;
    }

    /// Current (sys, prn, doppler) knowledge — for persisting a seed cache.
    pub fn candidates_snapshot(&self) -> Vec<(Sys, usize, f64)> {
        if !self.channels.is_empty() {
            self.channels
                .iter()
                .map(|ch| (ch.sys, ch.prn, ch.debug_dopp()))
                .collect()
        } else {
            self.candidates.clone()
        }
    }

    /// Invalidate all channels after an input discontinuity (dropped
    /// chunks): carrier/code phases cannot cross a gap. Channels become
    /// alignment candidates keeping their last Doppler — recovery is a
    /// narrow re-find on fresh data, not a full re-seed.
    pub fn force_reseed(&mut self) {
        // Only Tracking bands need reseeding. If a seed/align worker is
        // already in flight, its results are still valid recovery — forcing
        // a state change here would ORPHAN the running worker (keeps
        // burning CPU and holding the global slot) while a duplicate
        // queues behind it: observed live as seeds 1..7 never completing
        // and align results arriving 40-54 s stale.
        if !matches!(self.state, SeedState::Tracking) {
            return;
        }
        self.candidates = self
            .channels
            .iter()
            .map(|ch| (ch.sys, ch.prn, ch.debug_dopp()))
            .collect();
        self.channels.clear();
        self.clear_buf();
        self.hist.clear();
        self.t_s = 0.0;
        self.next_report_s = 1;
        self.align_fails = 0;
        self.state = if self.candidates.is_empty() {
            SeedState::NeedSeed
        } else {
            SeedState::NeedAlign
        };
    }
}

/// (chip_rate, carrier Hz, code length chips) per system — for code-phase
/// extrapolation (carrier-aided code clock).
fn sys_params(sys: Sys) -> (f64, f64, f64) {
    match sys {
        Sys::Beidou => (B1I_CHIP_RATE, F_B1I, 2046.0),
        Sys::Galileo => (1.023e6, F_L1, 4092.0),
        _ => (1.023e6, F_L1, 1023.0),
    }
}

/// Single-PRN acquisition over `sig` with an explicit Doppler grid,
/// extrapolating the code phase `t_fwd` seconds past the sig START to the
/// point where tracking will begin. Returns (doppler, code phase in the
/// tracker's forward convention) if detected. Free function: also runs on
/// the acquisition worker thread.
fn reacquire_on(
    fs: f64,
    sig: &[Complex<f32>],
    sys: Sys,
    prn: usize,
    grid: &[f64],
    t_fwd: f64,
) -> Option<(f64, f64)> {
    let nms = sig.len() / 1000;
    let res = match sys {
        Sys::Gps => acquire(sig, fs, &[prn], grid, nms, 2.5),
        Sys::Sbas => acquire_sbas(sig, fs, &[prn], grid, nms, 2.5),
        Sys::Galileo => acquire_e1b(sig, fs, &[prn], grid, sig.len() / 4000, 2.5),
        Sys::Beidou => {
            let codes = vec![(prn, generate_beidou_b1_code(prn))];
            acquire_codes(sig, fs, &codes, B1I_CHIP_RATE, grid, nms, 2.5, F_B1I, true)
        }
    };
    let r = res.into_iter().next()?;
    if !r.acquired {
        return None;
    }
    // refine the Doppler to ~±1 Hz (the 50 Hz align grid leaves residuals
    // beyond the Costas pull-in — see seed_snapshot's staged refine)
    let mut tmp = Channel::new(sys, prn, fs, r.doppler, r.code_phase);
    tmp.refine_dopp(sig, 30.0, 5.0, 200);
    let dopp = tmp.refine_dopp(sig, 6.0, 1.0, 400);
    // extrapolate the code phase from the sig start to the track start:
    // the code clock is carrier-aided. The negation converts the
    // acquisition lag convention (see Channel::new).
    let (chip_rate, f_carrier, code_len) = sys_params(sys);
    let cp = (-r.code_phase + chip_rate * (1.0 + dopp / f_carrier) * t_fwd)
        .rem_euclid(code_len);
    Some((dopp, cp))
}

/// Full seeding acquisition on a snapshot, on the worker thread: every PRN
/// of every constellation this band carries. Returns (sys, prn, doppler)
/// candidates — code phases are NOT returned because they are stale by the
/// time the worker finishes; alignment re-finds them on fresh data.
fn seed_snapshot(name: &'static str, fs: f64, sig: &[Complex<f32>]) -> Vec<(Sys, usize, f64)> {
    let nms = (sig.len() as f64 / (fs / 1000.0)) as usize;
    let mut found: Vec<Channel> = Vec::new();
    match name {
        "l1" => {
            let dopp = dopp_grid(5000.0, 500.0);
            let prns: Vec<usize> = (1..=32).collect();
            for r in acquire(sig, fs, &prns, &dopp, nms, 2.5) {
                if r.acquired {
                    found.push(Channel::new(Sys::Gps, r.prn, fs, r.doppler, r.code_phase));
                }
            }
            let sbas_prns = [121, 122, 123, 131, 133, 135, 136, 138, 139];
            let dopp_geo = dopp_grid(1500.0, 250.0);
            for r in acquire_sbas(sig, fs, &sbas_prns, &dopp_geo, nms, 2.5) {
                if r.acquired {
                    found.push(Channel::new(Sys::Sbas, r.prn, fs, r.doppler, r.code_phase));
                }
            }
            let gal_prns: Vec<usize> = (1..=36).collect();
            for r in acquire_e1b(sig, fs, &gal_prns, &dopp, nms / 4, 2.5) {
                if r.acquired {
                    found.push(Channel::new(Sys::Galileo, r.prn, fs, r.doppler, r.code_phase));
                }
            }
        }
        "b1i" => {
            let codes: Vec<(usize, Vec<f32>)> =
                (1..=37).map(|p| (p, generate_beidou_b1_code(p))).collect();
            let dopp = dopp_grid(5000.0, 500.0);
            for r in acquire_codes(sig, fs, &codes, B1I_CHIP_RATE, &dopp, nms, 2.5, F_B1I, true) {
                if r.acquired {
                    found.push(Channel::new(Sys::Beidou, r.prn, fs, r.doppler, r.code_phase));
                }
            }
        }
        _ => {}
    }
    // refine each seed's Doppler in stages to ~±1 Hz: the Costas pull-in is
    // ~10 Hz, so ±25 Hz residuals left the carrier rotating and nav bits
    // were noise (observed: flat transition histograms, zero subframes).
    // Longer integrations sharpen each pass.
    found.par_iter_mut().for_each(|ch| {
        ch.refine_dopp(sig, 250.0, 25.0, 100);
        ch.refine_dopp(sig, 30.0, 5.0, 200);
        ch.refine_dopp(sig, 6.0, 1.0, 400);
    });
    found
        .iter()
        .map(|ch| (ch.sys, ch.prn, ch.debug_dopp()))
        .collect()
}

fn dopp_grid(half: f64, step: f64) -> Vec<f64> {
    let n = (half / step).round() as i64;
    (-n..=n).map(|k| k as f64 * step).collect()
}

// ---------------------------------------------------------------- live engine

/// Hamming-windowed sinc lowpass, unity DC gain, `ntaps` odd.
fn firwin(ntaps: usize, fc: f64, fs: f64) -> Vec<f32> {
    let m = (ntaps - 1) as f64 / 2.0;
    let mut b: Vec<f64> = (0..ntaps)
        .map(|k| {
            let x = k as f64 - m;
            let sinc = if x.abs() < 1e-9 {
                2.0 * fc / fs
            } else {
                (2.0 * PI * fc * x / fs).sin() / (PI * x)
            };
            sinc * (0.54 - 0.46 * (2.0 * PI * k as f64 / (ntaps - 1) as f64).cos())
        })
        .collect();
    let g: f64 = b.iter().sum();
    for v in b.iter_mut() {
        *v /= g;
    }
    b.into_iter().map(|v| v as f32).collect()
}

/// One full-rate -> baseband chain: the mix is FOLDED into complex FIR taps
/// (y[o] = e^{jφ_o} Σ x[base+k]·h[k]·e^{-jkd}), so there is no separate
/// 16M-samples/s mixing pass and no per-chunk `mixed` allocation, and the
/// polyphase decimator runs as an unrolled multi-accumulator MAC over
/// deinterleaved re/im slices.
///
/// The output phase is derived from the ABSOLUTE sample index, not a
/// cross-chunk recurrence: the previous version's stored phase advanced by
/// +dphi/sample while its per-sample recurrence rotated by -dphi/sample, so
/// every chunk boundary carried a phase jump of 2·n·dphi (masked in practice
/// by the tracking loops re-anchoring every epoch). Absolute indexing is
/// continuous by construction and cannot drift.
struct Downconvert {
    if_hz: f64,
    /// Absolute index of the next full-rate sample to arrive (phase origin).
    abs_next: u64,
    /// Logical tap count (63); the tap arrays are zero-padded to 64 so the
    /// NEON kernel runs 16 clean quad-iterations.
    nt: usize,
    taps_re: Vec<f32>,
    taps_im: Vec<f32>,
    q: usize,
    tail_re: Vec<f32>,
    tail_im: Vec<f32>,
}

impl Downconvert {
    fn new(if_hz: f64, fs_in: f64, q: usize, cutoff: f64) -> Downconvert {
        let h = firwin(63, cutoff, fs_in);
        let d = 2.0 * PI * if_hz / fs_in;
        let nt = h.len();
        let mut taps_re: Vec<f32> = h
            .iter()
            .enumerate()
            .map(|(k, &t)| t * (k as f64 * d).cos() as f32)
            .collect();
        let mut taps_im: Vec<f32> = h
            .iter()
            .enumerate()
            .map(|(k, &t)| -t * (k as f64 * d).sin() as f32)
            .collect();
        taps_re.resize(64, 0.0);
        taps_im.resize(64, 0.0);
        Downconvert {
            if_hz,
            abs_next: 0,
            nt,
            taps_re,
            taps_im,
            q,
            tail_re: vec![0.0; nt - 1],
            tail_im: vec![0.0; nt - 1],
        }
    }

    /// Mix + decimate a chunk of `fs_in`-rate samples; appends to `out`.
    fn process(&mut self, sig: &[Complex<f32>], fs_in: f64, out: &mut Vec<Complex<f32>>) {
        let d = 2.0 * PI * self.if_hz / fs_in;
        let nt = self.nt;
        // deinterleave onto the tail carryover
        let mut s_re = Vec::with_capacity(self.tail_re.len() + sig.len());
        let mut s_im = Vec::with_capacity(self.tail_im.len() + sig.len());
        s_re.extend_from_slice(&self.tail_re);
        s_im.extend_from_slice(&self.tail_im);
        s_re.extend(sig.iter().map(|s| s.re));
        s_im.extend(sig.iter().map(|s| s.im));
        // absolute index of s_*[0] (negative while the initial zero tail
        // is still in the window — those taps multiply zeros anyway)
        let abs0 = self.abs_next as i64 - self.tail_re.len() as i64;
        self.abs_next += sig.len() as u64;
        let n_out = (s_re.len().saturating_sub(nt - 1)) / self.q;
        let base_out = out.len();
        out.resize_with(base_out + n_out, || Complex::new(0.0, 0.0));
        let dst = &mut out[base_out..];
        let (t_re, t_im) = (&self.taps_re[..], &self.taps_im[..]);
        let (sr, si) = (&s_re[..], &s_im[..]);
        let q = self.q;
        // Per-output phase rotator as an f64 complex recurrence, anchored
        // from the ABSOLUTE sample index at chunk start: continuous across
        // chunks, no drift (reanchored every chunk).
        let phi0 = (-(abs0 as f64 * d) - std::f64::consts::FRAC_PI_2).rem_euclid(2.0 * PI);
        let (mut sp, mut cp) = phi0.sin_cos();
        let (sd, cd) = (-(q as f64) * d).rem_euclid(2.0 * PI).sin_cos();
        for (o, slot) in dst.iter_mut().enumerate() {
            let base = o * q;
            let (ar, ai) = mac63(&sr[base..], &si[base..], t_re, t_im);
            *slot = Complex::new(
                (ar as f64 * cp - ai as f64 * sp) as f32,
                (ar as f64 * sp + ai as f64 * cp) as f32,
            );
            let ncp = cp * cd - sp * sd;
            sp = cp * sd + sp * cd;
            cp = ncp;
        }
        let consumed = n_out * q;
        self.tail_re = s_re[consumed..].to_vec();
        self.tail_im = s_im[consumed..].to_vec();
    }
}

/// 63-tap complex-coefficient MAC over deinterleaved slices. aarch64: NEON
/// with zero-padded taps (64 taps = 16 clean quad iterations, dual
/// accumulator chains); the window has >= 63 valid samples at `base` and the
/// pad tap multiplies a readable-or-zero 64th — handled by the caller's
/// slice extents. Scalar fallback elsewhere.
#[cfg(target_arch = "aarch64")]
#[inline]
fn mac63(w_re: &[f32], w_im: &[f32], t_re: &[f32], t_im: &[f32]) -> (f32, f32) {
    debug_assert!(t_re.len() == 64 && t_im.len() == 64);
    debug_assert!(w_re.len() >= 63 && w_im.len() >= 63);
    // The padded 64th tap is zero; if the window has no 64th sample the
    // product would be garbage — the caller guarantees >= 64 readable by
    // stopping the NEON path one output early.
    if w_re.len() < 64 || w_im.len() < 64 {
        return mac63_scalar(w_re, w_im, t_re, t_im, 63);
    }
    unsafe {
        use std::arch::aarch64::*;
        let (mut ar0, mut ai0) = (vdupq_n_f32(0.0), vdupq_n_f32(0.0));
        let (mut ar1, mut ai1) = (vdupq_n_f32(0.0), vdupq_n_f32(0.0));
        for k in (0..64).step_by(8) {
            let wr0 = vld1q_f32(w_re.as_ptr().add(k));
            let wi0 = vld1q_f32(w_im.as_ptr().add(k));
            let tr0 = vld1q_f32(t_re.as_ptr().add(k));
            let ti0 = vld1q_f32(t_im.as_ptr().add(k));
            ar0 = vfmaq_f32(ar0, wr0, tr0);
            ar0 = vfmsq_f32(ar0, wi0, ti0);
            ai0 = vfmaq_f32(ai0, wr0, ti0);
            ai0 = vfmaq_f32(ai0, wi0, tr0);
            let wr1 = vld1q_f32(w_re.as_ptr().add(k + 4));
            let wi1 = vld1q_f32(w_im.as_ptr().add(k + 4));
            let tr1 = vld1q_f32(t_re.as_ptr().add(k + 4));
            let ti1 = vld1q_f32(t_im.as_ptr().add(k + 4));
            ar1 = vfmaq_f32(ar1, wr1, tr1);
            ar1 = vfmsq_f32(ar1, wi1, ti1);
            ai1 = vfmaq_f32(ai1, wr1, ti1);
            ai1 = vfmaq_f32(ai1, wi1, tr1);
        }
        (
            vaddvq_f32(vaddq_f32(ar0, ar1)),
            vaddvq_f32(vaddq_f32(ai0, ai1)),
        )
    }
}

#[cfg(not(target_arch = "aarch64"))]
#[inline]
fn mac63(w_re: &[f32], w_im: &[f32], t_re: &[f32], t_im: &[f32]) -> (f32, f32) {
    mac63_scalar(w_re, w_im, t_re, t_im, 63)
}

/// Scalar 4-way-unrolled MAC over `nt` taps (the NEON pads to 64; this
/// handles the true 63 and any tail).
#[inline]
fn mac63_scalar(w_re: &[f32], w_im: &[f32], t_re: &[f32], t_im: &[f32], nt: usize) -> (f32, f32) {
    let (mut ar0, mut ar1, mut ar2, mut ar3) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    let (mut ai0, mut ai1, mut ai2, mut ai3) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    let mut k = 0;
    while k + 4 <= nt {
        ar0 += w_re[k] * t_re[k] - w_im[k] * t_im[k];
        ai0 += w_re[k] * t_im[k] + w_im[k] * t_re[k];
        ar1 += w_re[k + 1] * t_re[k + 1] - w_im[k + 1] * t_im[k + 1];
        ai1 += w_re[k + 1] * t_im[k + 1] + w_im[k + 1] * t_re[k + 1];
        ar2 += w_re[k + 2] * t_re[k + 2] - w_im[k + 2] * t_im[k + 2];
        ai2 += w_re[k + 2] * t_im[k + 2] + w_im[k + 2] * t_re[k + 2];
        ar3 += w_re[k + 3] * t_re[k + 3] - w_im[k + 3] * t_im[k + 3];
        ai3 += w_re[k + 3] * t_im[k + 3] + w_im[k + 3] * t_re[k + 3];
        k += 4;
    }
    let (mut ar, mut ai) = ((ar0 + ar1) + (ar2 + ar3), (ai0 + ai1) + (ai2 + ai3));
    while k < nt {
        ar += w_re[k] * t_re[k] - w_im[k] * t_im[k];
        ai += w_re[k] * t_im[k] + w_im[k] * t_re[k];
        k += 1;
    }
    (ar, ai)
}

/// Live front end: interleaved int8 at `fs_in` centred on `fc_hz` (16 Msps @
/// 1568.25 MHz spans BeiDou B1I .. GPS L1 in one capture). Two downconvert
/// chains feed two 4 Msps bands.
pub struct Engine {
    fs_in: f64,
    l1: Downconvert,
    b1i: Downconvert,
    pub l1_band: Band,
    pub b1i_band: Band,
    dc_i: f32,
    dc_q: f32,
    dc_init: bool,
}

impl Engine {
    pub fn new(fs_in: f64, fc_hz: f64, epoch0: f64) -> Engine {
        let q = (fs_in / BAND_FS).round() as usize;
        Engine {
            fs_in,
            l1: Downconvert::new(F_L1 - fc_hz, fs_in, q, 1.7e6),
            b1i: Downconvert::new(F_B1I - fc_hz, fs_in, q, 1.9e6),
            l1_band: Band::new_l1(BAND_FS, epoch0),
            b1i_band: Band::new_b1i(BAND_FS, epoch0),
            dc_i: 0.0,
            dc_q: 0.0,
            dc_init: false,
        }
    }

    /// Notify the engine that the reference clock correction stepped by
    /// `ppm`: every channel's measured Doppler moves by -ppm x f_carrier
    /// (the LO moved), so shift them all — the step then causes no lock
    /// loss. Code-rate aiding follows carrier_freq automatically.
    pub fn note_clock_step(&mut self, ppm: f64) {
        for band in [&mut self.l1_band, &mut self.b1i_band] {
            for ch in &mut band.channels {
                let df = -ppm * ch.f_carrier / 1e6;
                ch.shift_dopp(df);
            }
        }
    }

    /// Per-channel true pseudoranges from TOW anchors, both bands.
    pub fn nav_obs(&self) -> Vec<(u8, f64, f64)> {
        let mut v = self.l1_band.nav_obs();
        v.extend(self.b1i_band.nav_obs());
        v
    }

    /// Self-decoded broadcast ephemerides from live nav subframes.
    pub fn ephemerides(&self) -> Vec<BrdcEph> {
        self.l1_band
            .channels
            .iter()
            .chain(self.b1i_band.channels.iter())
            .filter_map(|ch| ch.eph.clone())
            .collect()
    }

    /// Force both bands back to alignment after an input discontinuity
    /// (dropped chunks): channel phases cannot cross a gap, so each channel
    /// is re-found narrowly at the live edge on fresh data.
    pub fn request_reseed(&mut self) {
        self.l1_band.force_reseed();
        self.b1i_band.force_reseed();
    }

    /// Account dropped/skipped full-rate int8 BYTES as elapsed stream time
    /// in both bands. The bands' install-time code-phase extrapolation runs
    /// on this stream-time clock; without gap accounting it under-counts by
    /// exactly the dropped time and every channel installed afterwards is
    /// misaligned (observed live: garbage cn0, no locks, endless reseed).
    pub fn note_gap_bytes(&mut self, gap_bytes: u64) {
        let band_samples = gap_bytes / 2 / (self.fs_in / BAND_FS) as u64;
        self.l1_band.note_gap(band_samples);
        self.b1i_band.note_gap(band_samples);
    }

    /// Consume interleaved int8 I/Q; returns all 1 Hz reports produced.
    pub fn push_i8(&mut self, raw: &[u8]) -> Vec<SatReport> {
        let mut sig: Vec<Complex<f32>> = raw
            .chunks_exact(2)
            .map(|c| Complex::new(c[0] as i8 as f32, c[1] as i8 as f32))
            .collect();
        // DC / centre-spike removal (slow EMA of the stream mean)
        if !sig.is_empty() {
            if !self.dc_init {
                let (mut si, mut sq) = (0.0f64, 0.0f64);
                for s in &sig {
                    si += s.re as f64;
                    sq += s.im as f64;
                }
                self.dc_i = (si / sig.len() as f64) as f32;
                self.dc_q = (sq / sig.len() as f64) as f32;
                self.dc_init = true;
            } else {
                let (mut si, mut sq) = (0.0f64, 0.0f64);
                for s in &sig {
                    si += s.re as f64;
                    sq += s.im as f64;
                }
                let a = 0.001f32;
                self.dc_i = (1.0 - a) * self.dc_i + a * (si / sig.len() as f64) as f32;
                self.dc_q = (1.0 - a) * self.dc_q + a * (sq / sig.len() as f64) as f32;
            }
            let (di, dq) = (self.dc_i, self.dc_q);
            for s in sig.iter_mut() {
                s.re -= di;
                s.im -= dq;
            }
        }
        // The two downconvert chains are the serial bottleneck (mix at fs_in
        // + 63-tap FIR on each, all on ONE core while the tracking channels
        // already parallelise via rayon). A consumer even 1% slower than the
        // 32 MB/s stream forces continuous input drops, and every gap breaks
        // the tracking loops' carrier/code phase (observed live: seeds found,
        // then loops wandered with junk cn0). Run the chains on two cores.
        let fs_in = self.fs_in;
        let Engine { l1, b1i, l1_band, b1i_band, .. } = self;
        let (l1_out, b1_out) = rayon::join(
            || {
                let mut v = Vec::new();
                l1.process(&sig, fs_in, &mut v);
                v
            },
            || {
                let mut v = Vec::new();
                b1i.process(&sig, fs_in, &mut v);
                v
            },
        );
        let mut reports = l1_band.push(&l1_out);
        reports.extend(b1i_band.push(&b1_out));
        reports
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verbatim copy of the pre-rewrite Downconvert algorithm (mix pass with
    /// per-sample recurrence + scalar polyphase FIR), kept as the oracle for
    /// the fused-taps rewrite. NOTE: the oracle's stored phase advances by
    /// +dphi/sample while its recurrence rotates by -dphi/sample, so it has a
    /// phase jump at every chunk boundary; the rewrite fixes that. The A/B
    /// comparison below therefore uses ONE chunk, where both are continuous.
    struct RefChain {
        phase: f64,
        taps: Vec<f32>,
        q: usize,
        tail: Vec<Complex<f32>>,
    }

    impl RefChain {
        fn new(if_hz: f64, fs_in: f64, q: usize, cutoff: f64) -> RefChain {
            let _ = if_hz;
            RefChain {
                phase: 0.0,
                taps: firwin(63, cutoff, fs_in),
                q,
                tail: vec![Complex::new(0.0, 0.0); 62],
            }
        }

        fn process_ref(
            &mut self,
            sig: &[Complex<f32>],
            if_hz: f64,
            fs_in: f64,
            out: &mut Vec<Complex<f32>>,
        ) {
            let dphi = 2.0 * PI * if_hz / fs_in;
            let (sd, cd) = dphi.sin_cos();
            let (mut cr, mut ci) = self.phase.sin_cos();
            let mut mixed: Vec<Complex<f32>> = Vec::with_capacity(sig.len());
            for s in sig {
                mixed.push(Complex::new(
                    s.re * cr as f32 + s.im * ci as f32,
                    -s.re * ci as f32 + s.im * cr as f32,
                ));
                let ncr = cr * cd - ci * sd;
                ci = cr * sd + ci * cd;
                cr = ncr;
            }
            self.phase = (self.phase + dphi * sig.len() as f64).rem_euclid(2.0 * PI);
            let nt = self.taps.len();
            let mut stream = std::mem::take(&mut self.tail);
            stream.extend_from_slice(&mixed);
            let n_out = (stream.len().saturating_sub(nt - 1)) / self.q;
            for o in 0..n_out {
                let base = o * self.q;
                let mut acc = Complex::new(0.0f32, 0.0);
                for (k, &t) in self.taps.iter().enumerate() {
                    acc += stream[base + k] * t;
                }
                out.push(acc);
            }
            let consumed = n_out * self.q;
            self.tail = stream[consumed..].to_vec();
        }
    }

    #[test]
    fn downconvert_fused_matches_reference_single_chunk() {
        let (fs, if_hz, q, cutoff) = (16.0e6, 7.17e6, 4, 1.7e6);
        // deterministic noise
        let mut state = 0x1234_5678_9abc_def0u64;
        let sig: Vec<Complex<f32>> = (0..1_000_000)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                Complex::new(
                    ((state >> 32) as i8) as f32,
                    ((state >> 40) as i8) as f32,
                )
            })
            .collect();

        let mut new = Downconvert::new(if_hz, fs, q, cutoff);
        let mut out_new = Vec::new();
        new.process(&sig, fs, &mut out_new);

        let mut old = RefChain::new(if_hz, fs, q, cutoff);
        let mut out_old = Vec::new();
        old.process_ref(&sig, if_hz, fs, &mut out_old);

        assert_eq!(out_new.len(), out_old.len(), "output length mismatch");
        let mut max_diff = 0.0f32;
        let mut max_mag = 0.0f32;
        for (a, b) in out_new.iter().zip(out_old.iter()) {
            max_diff = max_diff.max((a.re - b.re).abs()).max((a.im - b.im).abs());
            max_mag = max_mag.max(b.re.abs()).max(b.im.abs());
        }
        assert!(
            max_diff < max_mag * 1e-3 + 1e-6,
            "fused downconvert diverges from reference: max diff {max_diff} vs magnitude {max_mag}"
        );
    }

    #[test]
    fn downconvert_chunking_invariant() {
        // Feeding the same stream as one big chunk vs many small chunks
        // must produce IDENTICAL output (tail + absolute-phase accounting).
        // The first live run used ~16 KiB FIFO reads; file replay uses
        // 512 KiB — a tail bug shows up only in the small-chunk regime.
        let (fs, if_hz, q, cutoff) = (16.0e6, 7.17e6, 4, 1.7e6);
        let mut state = 0xdead_beef_cafe_1234u64;
        let sig: Vec<Complex<f32>> = (0..300_000)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                Complex::new(((state >> 32) as i8) as f32, ((state >> 40) as i8) as f32)
            })
            .collect();

        let mut one = Downconvert::new(if_hz, fs, q, cutoff);
        let mut out_one = Vec::new();
        one.process(&sig, fs, &mut out_one);

        let mut many = Downconvert::new(if_hz, fs, q, cutoff);
        let mut out_many = Vec::new();
        let mut off = 0;
        while off < sig.len() {
            let n = 8_192.min(sig.len() - off); // ~16 KiB int8 reads
            many.process(&sig[off..off + n], fs, &mut out_many);
            off += n;
        }

        assert_eq!(out_one.len(), out_many.len(), "chunked output length drift");
        let mut max_diff = 0.0f32;
        for (a, b) in out_one.iter().zip(out_many.iter()) {
            max_diff = max_diff.max((a.re - b.re).abs()).max((a.im - b.im).abs());
        }
        assert!(max_diff < 1e-3, "chunk-size-dependent output: {max_diff}");
    }

    /// Anchor lattice-phase regression (the 2026-08-24 flicker bug): the
    /// 20 ms bit-edge snap MUST be referenced to the code-period lattice,
    /// not to 0.02 s buckets hanging off t_wrap. When the number of code
    /// periods between the boundary and the most recent code wrap is not a
    /// multiple of 20 (Doppler drift walks it; it steps every time the
    /// code phase wraps at the 1-s report boundary), the old snap landed
    /// the anchor r ms off (r = m mod 20) and flickered by +/-1 ms per
    /// refresh, or by ~20 ms near the rounding boundary. The nav
    /// bookkeeping pins the boundary to +/-0.5 ms, so snapping to the
    /// NEAREST code-period boundary is exact.
    fn anchor_case(sys: Sys, code_len: f64, chip_rate: f64, f_carrier: f64, m_true: i64) {
        let dopp = 1000.0;
        let mut ch = Channel::new(sys, 1, 4.0e6, dopp, 0.0);
        ch.carrier_freq = dopp;
        ch.code_phase = 300.0; // chips
        let code_rate = chip_rate * (1.0 + dopp / f_carrier);
        let t_c = code_len / code_rate;
        let t_proc = 100_000.0;
        let t_wrap = t_proc - ch.code_phase / code_rate;
        let t_true = t_wrap - m_true as f64 * t_c;
        // nav bookkeeping places the 20 ms approximation at 1 ms (epoch)
        // resolution; find the (nav_ms remainder, abs_bit) pair closest to
        // the true boundary — the bit sync guarantees within +/-0.5 ms
        let nbits = 10_000usize;
        ch.nav_bits = vec![0u8; nbits];
        let mut best = (usize::MAX, 0usize, f64::MAX); // (nav_ms len, abs_bit, err)
        for ms in 0..20usize {
            let ab = (nbits as f64 + (t_true - t_proc + ms as f64 / 1000.0) / 0.02).round()
                as usize;
            let approx = t_proc - ms as f64 / 1000.0 + 0.02 * (ab as f64 - nbits as f64);
            let err = (approx - t_true).abs();
            if err < best.2 {
                best = (ms, ab, err);
            }
        }
        let (ms, abs_bit, err) = best;
        assert!(err < 0.5e-3, "test setup: approx off by {err}");
        ch.nav_ms = vec![0.0; ms];
        let got = anchor_stream_time(&mut ch, t_proc, abs_bit, 0.0);
        assert!(
            (got - t_true).abs() < 1e-5,
            "anchor off by {:.3} ms (m_true={m_true}, m mod 20 = {})",
            (got - t_true) * 1e3,
            m_true % 20
        );
    }

    #[test]
    fn anchor_stream_time_snaps_to_code_period_lattice_gps() {
        // healthy class: boundary a whole number of 20 code periods back
        anchor_case(Sys::Gps, 1023.0, 1.023e6, F_L1, 6160);
        // the flicker class: 6 code periods past a 20 ms multiple — the old
        // 0.02 s snap answered ~6 ms early
        anchor_case(Sys::Gps, 1023.0, 1.023e6, F_L1, 6166);
        // one period past a multiple: the +/-1 ms live flicker mode
        anchor_case(Sys::Gps, 1023.0, 1.023e6, F_L1, 6161);
    }

    #[test]
    fn anchor_stream_time_snaps_to_code_period_lattice_bds() {
        anchor_case(Sys::Beidou, 2046.0, B1I_CHIP_RATE, F_B1I, 6160);
        anchor_case(Sys::Beidou, 2046.0, B1I_CHIP_RATE, F_B1I, 6161);
    }

    /// The re-scan window must keep the LAST validated subframe findable so
    /// the anchor re-anchors to it every second. The 2026-08-24 divergence
    /// bug: the window reached back exactly one subframe, putting the last
    /// subframe's preamble at slice index 0 — but the LNAV finder starts at
    /// index 2 (it needs the two trailing bits of the previous subframe),
    /// so between validations the anchor was never refreshed and froze.
    #[test]
    fn rescan_window_refinds_last_subframe_gps() {
        const NAVBITS: &str = include_str!("../tests/fixtures/sim_navbits.txt");
        let line = NAVBITS.lines().next().unwrap();
        let mut it = line.split_whitespace();
        let _prn = it.next().unwrap();
        let bits: Vec<u8> = it.next().unwrap().bytes().map(|c| c - b'0').collect();
        let subs = crate::gps::lnav::find_subframes(&bits);
        let last = subs.last().expect("fixture has subframes");
        let b = last.bit_index;
        assert!(b >= 2, "fixture subframe must not start at bit 0");
        // emulate the tracker state right after this subframe validated
        let nav_scanned = b + 300;
        // new behaviour: the widened window re-finds it (as slice index 2)
        let from = nav_scanned - GPS_RESCAN_BACK;
        let again = crate::gps::lnav::find_subframes(&bits[from..]);
        assert!(
            again.iter().any(|s| from + s.bit_index == b),
            "widened window must re-find the last validated subframe"
        );
        // old behaviour, for the record: the same subframe at slice index 0
        // is invisible to the finder (this is why anchors froze)
        let from_old = nav_scanned - 300;
        let missed = crate::gps::lnav::find_subframes(&bits[from_old..b + 300]);
        assert!(
            missed.is_empty(),
            "this test documents the old freeze: preamble at index 0 is skipped"
        );
    }

    /// Freshness gate: publish only within ANCHOR_MAX_AGE_S of the last
    /// anchor refresh; never publish without an anchor.
    #[test]
    fn anchor_freshness_gate() {
        let mut ch = Channel::new(Sys::Gps, 1, 4.0e6, 1000.0, 0.0);
        assert!(!ch.anchor_fresh(100.0), "no anchor -> never fresh");
        ch.anchor = Some((50.0, 40.0));
        ch.anchor_t = 100.0;
        assert!(ch.anchor_fresh(100.0));
        assert!(ch.anchor_fresh(100.0 + ANCHOR_MAX_AGE_S));
        assert!(
            !ch.anchor_fresh(100.0 + ANCHOR_MAX_AGE_S + 1.0),
            "anchor older than ANCHOR_MAX_AGE_S must be withdrawn"
        );
    }

    /// Build a GPS channel whose nav bookkeeping approximation for the
    /// subframe boundary sits a controlled distance OFF the true boundary,
    /// in the (0.5, 1.5) ms wrong-tooth zone where a bare nearest-tooth
    /// snap picks the wrong code period (the static +/-1 ms anchors seen
    /// live). `sign` chooses the side. Returns (channel, t_proc, abs_bit,
    /// t_true, t_c, approx).
    fn wrong_tooth_setup(sign: f64) -> (Channel, f64, usize, f64, f64, f64) {
        let dopp = 1000.0;
        let mut ch = Channel::new(Sys::Gps, 1, 4.0e6, dopp, 0.0);
        ch.carrier_freq = dopp;
        ch.code_phase = 300.0;
        let code_rate = 1.023e6 * (1.0 + dopp / F_L1);
        let t_c = 1023.0 / code_rate;
        let t_proc = 100_000.0;
        let nbits = 10_000usize;
        ch.nav_bits = vec![0u8; nbits];
        // the 20 ms bit grid is commensurate with the 1 ms epoch grid, so
        // the bookkeeping residue is fixed per code phase; sweep the code
        // phase to walk the achieved offset through the wrong-tooth zone
        let mut picked = None;
        for j in 0..1023 {
            let cp = 300.0 + j as f64;
            let t_wrap_j = t_proc - cp / code_rate;
            let t_true_j = t_wrap_j - 6161.0 * t_c;
            let abs_bit = (nbits as f64 + (t_true_j + sign * 0.75e-3 - t_proc) / 0.02).round() as usize;
            for ms in 0..20usize {
                let approx =
                    t_proc - ms as f64 / 1000.0 + 0.02 * (abs_bit as f64 - nbits as f64);
                let actual = (approx - t_true_j) * 1e3;
                if (actual.abs() - 0.75).abs() <= 0.2 && actual.signum() == sign {
                    picked = Some((cp, ms, abs_bit, t_true_j, approx));
                    break;
                }
            }
            if picked.is_some() {
                break;
            }
        }
        let (cp, ms, abs_bit, t_true, approx) = picked.expect("no achievable wrong-tooth offset");
        ch.code_phase = cp;
        ch.nav_ms = vec![0.0; ms];
        (ch, t_proc, abs_bit, t_true, t_c, approx)
    }

    /// Tooth-exact propagation: with a previous anchor in place, the new
    /// boundary is exactly 6000 code periods (per 6 s of t_tx) along the
    /// comb — the wandering nav approximation must NOT be consulted. The
    /// bare snap in this setup lands one full code period off.
    #[test]
    fn anchor_propagates_tooth_exact() {
        let (mut ch, t_proc, abs_bit, t_true, t_c, approx) = wrong_tooth_setup(1.0);
        // previous subframe's boundary: exactly 6000 teeth (6 s of t_tx) back
        let t_prev = t_true - 6000.0 * t_c;
        ch.anchor = Some((t_prev, 1000.0));
        let got = anchor_stream_time(&mut ch, t_proc, abs_bit, 1006.0);
        assert!(
            (got - t_true).abs() < 1e-6,
            "propagated anchor off by {:.3} ms",
            (got - t_true) * 1e3
        );
        // the bare snap on the same bookkeeping picks the wrong tooth —
        // this is the bug class the propagation fixes
        let code_rate = 1.023e6 * (1.0 + 1000.0 / F_L1);
        let t_wrap = t_proc - ch.code_phase / code_rate;
        let bare = t_wrap - t_c * ((t_wrap - approx) / t_c).round();
        assert!(
            (bare - t_true).abs() > 0.5e-3,
            "setup should put the bare snap on the wrong tooth"
        );
        // and the propagation refreshed the fallback offset to the exact one
        assert!(ch.edge_off_valid);
        assert!((ch.edge_off_s - (t_true - approx)).abs() < 1e-6);
    }

    /// Re-anchoring the SAME subframe (d_tx = 0, the per-second re-anchor
    /// from the widened re-scan window) must reproduce the stored boundary
    /// exactly, not re-derive it from the wandering approximation.
    #[test]
    fn anchor_reanchor_same_subframe_is_stable() {
        let (mut ch, t_proc, abs_bit, t_true, t_c, approx) = wrong_tooth_setup(-1.0);
        ch.anchor = Some((t_true, 1000.0));
        // 30 s of stream later, on the same physical comb: 30000 teeth
        // elapsed, bookkeeping advanced — AND the carrier loop's frequency
        // estimate wobbled +15 Hz (the live wobble is +/-10-20 Hz between
        // seconds). The boundary instant is fixed; the 7c84c93 code
        // re-measured the 30 s tooth count with the WOBBLED rate and moved
        // the anchor by span*df/f_c ~= 286 ns.
        let t_proc2 = t_proc + 30.0;
        let t_wrap2 = (t_proc - ch.code_phase / (1.023e6 * (1.0 + 1000.0 / F_L1)))
            + 30000.0 * t_c; // a physical comb tooth 30 s later
        ch.carrier_freq = 1000.0 + 15.0; // loop wobble
        let wobbled_rate = 1.023e6 * (1.0 + ch.carrier_freq / F_L1);
        ch.code_phase = (t_proc2 - t_wrap2) * wobbled_rate; // keeps t_wrap2 on the comb
        assert!((0.0..1023.0).contains(&ch.code_phase), "test setup phase");
        // bookkeeping advanced with the stream: 1500 more bits emitted
        ch.nav_bits.extend(std::iter::repeat(0u8).take(1500));
        let approx2 = t_proc2 - ch.nav_ms.len() as f64 / 1000.0
            + 0.02 * (abs_bit as f64 - ch.nav_bits.len() as f64);
        assert!(
            (approx2 - approx).abs() < 1e-9,
            "test setup: bookkeeping should be time-consistent"
        );
        let got = anchor_stream_time(&mut ch, t_proc2, abs_bit, 1000.0);
        assert!(
            (got - t_true).abs() < 1e-9,
            "same-subframe re-anchor moved by {:.3} ns under carrier wobble",
            (got - t_true) * 1e9
        );
        // document the old behaviour: the rate-sensitive recomputation
        let t_code2 = 1023.0 / wobbled_rate;
        let old = t_wrap2 - t_code2 * ((t_wrap2 - t_true) / t_code2).round();
        assert!(
            (old - t_true).abs() > 100e-9,
            "setup should make the old rate-sensitive path visibly wrong"
        );
    }

    /// Put the nav bookkeeping a chosen ~5 ms off the true boundary — the
    /// post-wander regime (the approximation drifts ~1.3-2.5 us/s from bit
    /// sync, so it passes 2 ms after ~15-25 min of channel age).
    fn wandered_setup() -> (Channel, f64, usize, f64, f64) {
        let (mut ch, t_proc, _ab0, t_true, t_c, _a0) = wrong_tooth_setup(1.0);
        let nbits = ch.nav_bits.len();
        // coarsely place the approximation near t_true + 5 ms via the 20 ms
        // bit grid, then fine-tune with the 1 ms remainder grid
        let target = t_true + 5.0e-3;
        let mut best: Option<(usize, usize, f64)> = None; // (ms, abs_bit, approx)
        for db in -1isize..=1 {
            let abs_bit =
                ((nbits as f64 + (target - t_proc) / 0.02).round() as isize + db) as usize;
            for ms in 0..20usize {
                let a =
                    t_proc - ms as f64 / 1000.0 + 0.02 * (abs_bit as f64 - nbits as f64);
                let good = (a - target).abs() < 0.5e-3
                    && (a - t_true).abs() > 2.5e-3
                    && (a - t_true).abs() < 15e-3;
                if good && best.map_or(true, |b| (a - target).abs() < (b.2 - target).abs()) {
                    best = Some((ms, abs_bit, a));
                }
            }
        }
        let (ms, abs_bit, approx) = best.expect("no achievable wandered offset");
        ch.nav_ms = vec![0.0; ms];
        let _ = approx;
        (ch, t_proc, abs_bit, t_true, t_c)
    }

    /// With the approximation wandered past the old 2 ms cross-check, the
    /// same-subframe re-anchor must STILL return the established instant —
    /// the 7c84c93/2359cc2 gate threw the channel into the bare-snap
    /// fallback, which then ramped at the comb creep rate and flipped whole
    /// teeth (the live 40-150 ns/s marches and +/-1-3 ms jumps).
    #[test]
    fn anchor_reanchor_survives_approx_wander() {
        let (mut ch, t_proc, abs_bit, t_true, _t_c) = wandered_setup();
        ch.anchor = Some((t_true, 1000.0));
        let got = anchor_stream_time(&mut ch, t_proc, abs_bit, 1000.0);
        assert!(
            (got - t_true).abs() < 1e-9,
            "same-subframe re-anchor under wander moved by {:.3} ms",
            (got - t_true) * 1e3
        );
    }

    /// New-subframe propagation must tolerate the same wander: the tooth
    /// count from the previous anchor is exact regardless of what the
    /// bookkeeping has drifted to (a real bookkeeping break is a 20 ms
    /// multiple or a garbage t_tx — both far outside the 15 ms gate).
    #[test]
    fn anchor_propagation_tolerates_approx_wander() {
        let (mut ch, t_proc, abs_bit, t_true, t_c) = wandered_setup();
        // previous subframe's boundary: exactly 6000 teeth (6 s of t_tx) back
        let t_prev = t_true - 6000.0 * t_c;
        ch.anchor = Some((t_prev, 1000.0));
        let got = anchor_stream_time(&mut ch, t_proc, abs_bit, 1006.0);
        assert!(
            (got - t_true).abs() < 1e-6,
            "propagated anchor under wander off by {:.3} ms",
            (got - t_true) * 1e3
        );
    }

    /// The comb period is MEASURED by tooth counting, so a sustained
    /// carrier_freq bias (marginal channels hold 60-280 Hz while staying
    /// code-locked — the live per-channel drift class) must not leak into
    /// the anchor. Setup: carrier_freq biased +150 Hz, t_code_meas already
    /// converged to the true period; a 6 s propagation step must be exact.
    /// The pre-fix code (carrier-derived t_code) errs by
    /// lookback * 150 Hz / f_carrier ~ 0.6-1.2 us here.
    #[test]
    fn anchor_immune_to_carrier_bias_via_measured_comb() {
        let dopp_true = 1000.0;
        let rate_true = 1.023e6 * (1.0 + dopp_true / F_L1);
        let t_c_true = 1023.0 / rate_true;
        let mut ch = Channel::new(Sys::Gps, 1, 4.0e6, dopp_true, 0.0);
        ch.carrier_freq = dopp_true + 150.0; // sustained loop bias
        ch.t_code_meas = t_c_true; // as tooth counting converges to
        let t_proc = 100_000.0;
        ch.code_phase = 300.0;
        let t_wrap = t_proc - (ch.code_phase / 1023.0) * t_c_true;
        let m_true = 6161i64;
        let t_true = t_wrap - m_true as f64 * t_c_true;
        let t_prev = t_true - 6000.0 * t_c_true;
        ch.anchor = Some((t_prev, 1000.0));
        // nav bookkeeping within 0.5 ms of the truth (any cell in the zone)
        let nbits = 10_000usize;
        ch.nav_bits = vec![0u8; nbits];
        let mut chosen = (0usize, 0usize, f64::MAX);
        for ms in 0..20usize {
            let ab =
                (nbits as f64 + (t_true + 0.0002 - t_proc + ms as f64 / 1000.0) / 0.02).round()
                    as usize;
            let approx = t_proc - ms as f64 / 1000.0 + 0.02 * (ab as f64 - nbits as f64);
            let err = (approx - t_true).abs();
            if err < chosen.2 {
                chosen = (ms, ab, err);
            }
        }
        assert!(chosen.2 < 0.5e-3, "test setup: approx too far");
        ch.nav_ms = vec![0.0; chosen.0];
        let got = anchor_stream_time(&mut ch, t_proc, chosen.1, 1006.0);
        assert!(
            (got - t_true).abs() < 1e-9,
            "anchor under +150 Hz carrier bias off by {:.3} ns",
            (got - t_true) * 1e9
        );
        // document the pre-fix behaviour: carrier-derived t_code
        let t_code_biased = 1023.0 / (1.023e6 * (1.0 + ch.carrier_freq / F_L1));
        let old = t_wrap
            - t_code_biased
                * (((t_wrap - t_prev) / t_code_biased - 6000.0).round() + 6000.0);
        assert!(
            (old - t_true).abs() > 100e-9,
            "setup should make the carrier-derived path visibly wrong: {:.1} ns",
            (old - t_true) * 1e9
        );
    }

    /// The tooth-counting measurement itself converges to the true comb
    /// period from a biased (carrier-seeded) start, in ~1 min.
    #[test]
    fn comb_period_measurement_converges() {
        let dopp_true = 1000.0;
        let rate_true = 1.023e6 * (1.0 + dopp_true / F_L1);
        let t_c_true = 1023.0 / rate_true;
        let mut ch = Channel::new(Sys::Gps, 1, 4.0e6, dopp_true, 0.0);
        ch.locked = true;
        // biased seed: as if the carrier estimate were +150 Hz off
        ch.t_code_meas = 1023.0 / (1.023e6 * (1.0 + (dopp_true + 150.0) / F_L1));
        for n in 1..=90u32 {
            let t_proc = n as f64;
            // true most-recent tooth, and the TRUE code phase there
            let t_wrap_true = (t_proc / t_c_true).floor() * t_c_true;
            ch.code_phase = (t_proc - t_wrap_true) / t_c_true * 1023.0;
            ch.track_comb(t_proc);
        }
        let err = (ch.t_code_meas - t_c_true) / t_c_true;
        assert!(
            err.abs() < 1e-9,
            "comb period relative error {err:.2e} after 90 s"
        );
    }

    /// Channel state with the bookkeeping approximation a controlled
    /// distance from the true boundary, with `nbits` bits of stream age
    /// (the self-heal gate opens only for young streams: the approximation
    /// wanders ~3 us/s from bit sync, so only a young approximation is
    /// provably within a quarter-tooth of the true boundary).
    fn approx_setup(
        target_off_ms: f64,
        nbits: usize,
    ) -> (Channel, f64, usize, f64, f64) {
        let dopp = 1000.0;
        let mut ch = Channel::new(Sys::Gps, 1, 4.0e6, dopp, 0.0);
        ch.carrier_freq = dopp;
        let code_rate = 1.023e6 * (1.0 + dopp / F_L1);
        let t_c = 1023.0 / code_rate;
        let t_proc = 100_000.0;
        ch.nav_bits = vec![0u8; nbits];
        let mut picked = None;
        for j in 0..1023 {
            let cp = 300.0 + j as f64;
            let t_true = (t_proc - cp / code_rate) - 6161.0 * t_c;
            let abs_bit =
                (nbits as f64 + (t_true + target_off_ms / 1e3 - t_proc) / 0.02).round() as usize;
            for ms in 0..20usize {
                let approx =
                    t_proc - ms as f64 / 1000.0 + 0.02 * (abs_bit as f64 - nbits as f64);
                let off = (approx - t_true) * 1e3;
                if (off - target_off_ms).abs() < 0.05 {
                    picked = Some((cp, ms, abs_bit, t_true));
                    break;
                }
            }
            if picked.is_some() {
                break;
            }
        }
        let (cp, ms, abs_bit, t_true) = picked.expect("no achievable offset");
        ch.code_phase = cp;
        ch.nav_ms = vec![0.0; ms];
        (ch, t_proc, abs_bit, t_true, t_c)
    }

    /// Wrong-tooth self-heal: a young channel whose stored anchor sits one
    /// tooth off the confidently-snapped bookkeeping boundary must re-pick.
    /// Pre-fix (no heal) the stored wrong tooth propagates forever.
    #[test]
    fn anchor_self_heal_recovers_wrong_tooth_pick() {
        let (mut ch, t_proc, abs_bit, t_true, t_c) = approx_setup(0.12, 4_000);
        ch.anchor = Some((t_true - t_c, 1000.0)); // the bad first pick
        let got = anchor_stream_time(&mut ch, t_proc, abs_bit, 1000.0);
        assert!(
            (got - t_true).abs() < 1e-9,
            "heal left the anchor {:.3} ms from truth",
            (got - t_true) * 1e3
        );
        // and the fallback offset was re-referenced to the corrected anchor
        assert!(ch.edge_off_valid);
    }

    /// The same gate must never disturb a CORRECT anchor: (a) the gate is
    /// closed once the bit stream is old enough for the approximation to be
    /// confidently wrong; (b) a non-confident approximation (mid-cell)
    /// never triggers a re-pick.
    #[test]
    fn anchor_self_heal_never_disturbs_correct_anchor() {
        // (a) old stream (20000 bits ~ 400 s): approximation confidently one
        // tooth off — a young gate would mis-heal; the age gate must hold
        let (mut ch, t_proc, abs_bit, t_true, _t_c) = approx_setup(1.0, 20_000);
        ch.anchor = Some((t_true, 1000.0));
        let got = anchor_stream_time(&mut ch, t_proc, abs_bit, 1000.0);
        assert!(
            (got - t_true).abs() < 1e-9,
            "old channel's correct anchor was disturbed by {:.3} ms",
            (got - t_true) * 1e3
        );
        // (b) young but mid-cell approximation (confidence 0.5 > 0.25 gate)
        let (mut ch, t_proc, abs_bit, t_true, _t_c) = approx_setup(0.5, 4_000);
        ch.anchor = Some((t_true, 1000.0));
        let got = anchor_stream_time(&mut ch, t_proc, abs_bit, 1000.0);
        assert!(
            (got - t_true).abs() < 1e-9,
            "mid-cell approximation disturbed a correct anchor by {:.3} ms",
            (got - t_true) * 1e3
        );
    }

    /// First pick with no previous anchor and no dip knowledge: the bare
    /// snap parks one tooth off (documenting the live +/-1 ms class). With
    /// the flip-dip side known, the pick is exact.
    #[test]
    fn anchor_first_pick_uses_dip_side() {
        let (mut ch, t_proc, abs_bit, t_true, _t_c, _approx) = wrong_tooth_setup(1.0);
        // no anchor, no dip knowledge: wrong tooth (the bug class)
        let bare = anchor_stream_time(&mut ch, t_proc, abs_bit, 1000.0);
        assert!(
            (bare - t_true).abs() > 0.5e-3 && (bare - t_true).abs() < 1.5e-3,
            "bare snap should park ~1 tooth ({:.3} ms) off",
            (bare - t_true) * 1e3
        );
        // dip says the edge sits BELOW the approximation (last epoch of the
        // previous group): shifting the snap reference down resolves it
        ch.edge_off_s = -0.5e-3;
        ch.edge_off_valid = true;
        let fixed = anchor_stream_time(&mut ch, t_proc, abs_bit, 1000.0);
        assert!(
            (fixed - t_true).abs() < 1e-6,
            "dip-guided pick off by {:.3} ms",
            (fixed - t_true) * 1e3
        );
    }

    /// The flip-dip audit: synthetic 1 ms epochs with a known edge position
    /// must set edge_off_s on the correct side of the group boundary.
    fn dip_case(edge_after_boundary_ms: f64) -> Channel {
        // 40 groups of 20 epochs, bit value alternating every group (a flip
        // at every boundary), amplitude 1000. The edge sits
        // edge_after_boundary_ms into group epoch 0 (positive) or that far
        // before the boundary (negative: mixed epoch is the previous
        // group's last).
        let mut ch = Channel::new(Sys::Gps, 1, 4.0e6, 1000.0, 0.0);
        ch.bit_off = Some(0);
        let d = edge_after_boundary_ms;
        let mut ms = Vec::with_capacity(800);
        for g in 0..40 {
            let v = if g % 2 == 0 { 1000.0 } else { -1000.0 };
            let pv = -v;
            for e in 0..20 {
                let val = if d >= 0.0 {
                    if e == 0 {
                        // first epoch: old value for d ms, new for the rest
                        pv * d + v * (1.0 - d)
                    } else {
                        v
                    }
                } else if e == 19 {
                    // last epoch: old value for 1+d ms, new for -d ms
                    v * (1.0 + d) + pv * (-d)
                } else {
                    v
                };
                ms.push(val);
            }
        }
        ch.nav_ms = ms;
        ch.nav_tick_gps();
        ch
    }

    #[test]
    fn flip_dip_audit_finds_edge_side() {
        // edge 0.3 ms into the group's first epoch -> snap reference +0.5 ms
        let ch = dip_case(0.3);
        assert_eq!(ch.nav_bits.len(), 40, "all groups emitted");
        assert!(ch.edge_off_valid, "dip audit should converge");
        assert!(
            ch.edge_off_s > 0.0,
            "edge in first epoch must bias the snap up, got {}",
            ch.edge_off_s
        );
        // edge 0.3 ms before the boundary (previous group's last epoch)
        let ch = dip_case(-0.3);
        assert!(ch.edge_off_valid, "dip audit should converge");
        assert!(
            ch.edge_off_s < 0.0,
            "edge in last epoch must bias the snap down, got {}",
            ch.edge_off_s
        );
    }

    /// Single-PRN synthetic GPS signal: C/A code + alternating 20 ms data
    /// bits on a carrier at `dopp` Hz, light noise, code phase 0 at sample
    /// 0 (the same generator shape as tests/live_track.rs's fallback).
    fn synth_gps(prn: usize, dopp: f64, secs: usize, fs: f64) -> Vec<Complex<f32>> {
        let n = (fs as usize) * secs;
        let code = gps_ca(prn);
        let code_len = code.len();
        let ns_ms = (fs / 1000.0) as usize;
        let mut sig = vec![Complex::new(0.0f32, 0.0); n];
        let mut st = 0x243f_6a88_85a3_08d3u64;
        let mut nxt = || {
            st ^= st << 13;
            st ^= st >> 7;
            st ^= st << 17;
            ((st >> 40) as f32 / 8_388_608.0) - 1.0
        };
        for (k, s) in sig.iter_mut().enumerate() {
            let t = k as f64 / fs;
            let ci = (k as f64 * 1.023e6 * (1.0 + dopp / F_L1) / fs) as usize;
            let c = code[ci % code_len];
            let bit = if (k / ns_ms) / 20 % 2 == 0 { 1.0f32 } else { -1.0 };
            let ph = 2.0 * PI * dopp * t;
            let v = c * bit;
            s.re += v * ph.cos() as f32 + 0.05 * nxt();
            s.im += v * ph.sin() as f32 + 0.05 * nxt();
        }
        sig
    }

    /// The integrated carrier phase must accumulate at the true Doppler
    /// rate: a channel driven with a known-frequency synthetic, settled,
    /// must match truth to < 0.1 cycle/s.
    #[test]
    fn carrier_phase_integrates_known_doppler() {
        let fs = 4.0e6;
        let dopp = 800.0;
        let secs = 6;
        let sig = synth_gps(11, dopp, secs, fs);
        let mut ch = Channel::new(Sys::Gps, 11, fs, dopp, 0.0);
        let ns = (fs / 1000.0) as usize;
        let mut cyc = Vec::new();
        for (i, chunk) in sig.chunks(ns).enumerate() {
            ch.process_epoch(chunk);
            if (i + 1) % 1000 == 0 {
                ch.end_second();
                cyc.push(ch.carr_cycles);
            }
        }
        assert_eq!(cyc.len(), secs);
        // the loop locks with an arbitrary constant phase offset (and the
        // Costas 180 deg ambiguity), so compare RATES after a 3 s settle
        let rate = (cyc[5] - cyc[2]) / 3.0;
        assert!(
            (rate - dopp).abs() < 0.1,
            "phase rate {rate} cycles/s vs truth {dopp}"
        );
        assert!(ch.locked, "strong synthetic must lock");
        assert!(!ch.take_slip(), "no slip on a clean track");
        // fractional phase is published modulo the data-bit half-cycle
        let frac = ch.carr_cycles.rem_euclid(0.5);
        assert!((0.0..0.5).contains(&frac), "phase_frac out of [0, 0.5)");
        // and carrier_phase really is carr_cycles mod 1 (same accumulator)
        let frac1 = ch.carr_cycles.rem_euclid(1.0);
        assert!(
            (frac1 - ch.carrier_phase / (2.0 * PI)).abs() < 1e-6,
            "carrier_phase disagrees with carr_cycles mod 1"
        );
    }

    /// The slip flag must fire on exactly the report second the lock
    /// watchdog drops the channel, and on a reseed with a phase history.
    #[test]
    fn slip_flag_fires_on_lock_loss_and_reseed() {
        let fs = 4.0e6;
        let dopp = 800.0;
        let ns = (fs / 1000.0) as usize;
        let mut ch = Channel::new(Sys::Gps, 11, fs, dopp, 0.0);
        let sig = synth_gps(11, dopp, 5, fs);
        for sec in sig.chunks(1000 * ns) {
            for chunk in sec.chunks(ns) {
                ch.process_epoch(chunk);
            }
            ch.end_second();
            assert!(!ch.take_slip(), "no slip while the signal holds");
        }
        assert!(ch.locked, "strong synthetic must lock");
        // cut the signal: pure noise. The watchdog (UNLOCK_SECS consecutive
        // seconds below threshold) must drop lock and flag the slip on
        // exactly that second.
        let mut st = 0x1b87_3593_u64;
        let mut slip_at = None;
        for s in 0..12 {
            for _ in 0..1000 {
                let noise: Vec<Complex<f32>> = (0..ns)
                    .map(|_| {
                        st ^= st << 13;
                        st ^= st >> 7;
                        st ^= st << 17;
                        let v = ((st >> 40) as f32 / 8_388_608.0) - 1.0;
                        Complex::new(0.05 * v, 0.05 * v)
                    })
                    .collect();
                ch.process_epoch(&noise);
            }
            ch.end_second();
            if ch.take_slip() {
                slip_at = Some(s);
                assert!(!ch.locked, "slip must coincide with lock loss");
                break;
            }
        }
        let s = slip_at.expect("watchdog must fire within 12 s of noise");
        assert!(
            s + 1 >= UNLOCK_SECS as usize,
            "slip fired before the watchdog could: second {s}"
        );
        // reseed semantics: no phase history -> silent install; with a
        // phase history -> flagged break, accumulator re-zeros
        let mut ch2 = Channel::new(Sys::Gps, 11, fs, dopp, 0.0);
        ch2.reseed(900.0, 100.0);
        assert!(!ch2.take_slip(), "initial install is not a slip");
        ch2.carr_cycles = 123.0;
        ch2.reseed(900.0, 100.0);
        assert!(ch2.take_slip(), "reseed with a phase history must flag");
        assert_eq!(ch2.carr_cycles, 0.0, "zero is defined at (re)seed");
    }

    /// Round-7 regression: the DLL lock point — and with it the anchor's
    /// sub-ms fraction, which IS the DLL phase via wrap_time — must carry
    /// NO fractional-chip bias, for any sub-chip signal phase, Doppler, and
    /// at both the live band rate (4 Msps) and the synth-bench rate
    /// (8 Msps). A stable bias here lands 1:1 in every anchor pick (the
    /// 0.1-0.6 km live-residual class investigated 2026-08-25; the engine
    /// was exonerated — the class traced to truth-mapping artifacts in the
    /// benches — and this test keeps it that way).
    #[test]
    fn dll_lock_point_has_no_subchip_bias() {
        for &fs in &[4.0e6, 8.0e6] {
            for &dopp in &[0.0, 2500.0, -4200.0] {
                let ns = (fs as f64 / 1000.0).round() as usize;
                let code = gps_ca(1);
                let code_rate = 1.023e6 * (1.0 + dopp / F_L1);
                let step = code_rate / fs;
                let epochs = 400;
                let total = ns * epochs;
                for i in 0..10 {
                    let phi = i as f64 * 0.1; // chips, sweeps sub-sample too
                    let sig: Vec<Complex<f32>> = (0..total)
                        .map(|k| {
                            let c = code
                                [((k as f64 * step + phi) as usize) % code.len()]
                                as f32;
                            let (sn, cs) =
                                (2.0 * PI * dopp * k as f64 / fs as f64).sin_cos();
                            Complex::new(c * cs as f32, c * sn as f32)
                        })
                        .collect();
                    // acquisition-convention seed: signal is code[k*step -
                    // (-phi)]; the DLL must pull the replica onto the true
                    // phase from anywhere in the pull-in range
                    let mut ch = Channel::new(Sys::Gps, 1, fs, dopp, -phi - 0.31);
                    for e in 0..epochs {
                        ch.process_epoch(&sig[e * ns..(e + 1) * ns]);
                    }
                    let true_phase = (total as f64 * step + phi).rem_euclid(1023.0);
                    let bias =
                        (ch.code_phase - true_phase + 511.5).rem_euclid(1023.0) - 511.5;
                    assert!(
                        bias.abs() < 0.05,
                        "fs {fs:e} dopp {dopp:+} phi {phi:.1}: DLL lock bias {bias:+.4} chips ({:+.1} m)",
                        bias * 293.05
                    );
                    // and wrap_time (the anchor's fractional reference) must
                    // place the last code wrap at the TRUE crossing
                    let t_proc = total as f64 / fs;
                    let t_wrap_true = t_proc - true_phase / code_rate;
                    let t_wrap = ch.wrap_time(t_proc);
                    assert!(
                        (t_wrap - t_wrap_true).abs() * 1e6 < 0.05,
                        "fs {fs:e} dopp {dopp:+} phi {phi:.1}: wrap_time off by {:+.3} us",
                        (t_wrap - t_wrap_true) * 1e6
                    );
                }
            }
        }
    }

    /// WAAS 250-bit block: rotating preamble, message type, 212-bit payload,
    /// appended CRC-24Q over the first 226 bits (the same assembly as
    /// sbas.rs's test make_block).
    fn sbas_block(idx: usize, mt: u8, payload: &[u8]) -> Vec<u8> {
        assert_eq!(payload.len(), 212);
        let mut block = Vec::with_capacity(crate::sbas::BLOCK_BITS);
        for i in (0..8).rev() {
            block.push((crate::sbas::PREAMBLES[idx % 3] >> i) & 1);
        }
        for i in (0..6).rev() {
            block.push((mt >> i) & 1);
        }
        block.extend_from_slice(payload);
        let crc = crate::sbas::crc24q(&block);
        for i in (0..24).rev() {
            block.push(((crc >> i) & 1) as u8);
        }
        block
    }

    /// MT1 payload: 210-bit PRN mask + 2-bit IODP.
    fn mt1_payload(slots: &[u8], iodp: u8) -> Vec<u8> {
        let mut p = vec![0u8; 212];
        for &s in slots {
            p[(s - 1) as usize] = 1;
        }
        p[210] = (iodp >> 1) & 1;
        p[211] = iodp & 1;
        p
    }

    /// MT9 payload with a healthy block weight (alternating bits).
    fn mt9_payload() -> Vec<u8> {
        (0..212).map(|i| (i % 2) as u8).collect()
    }

    /// Single-PRN synthetic SBAS/WAAS L1 signal: C/A code + 500 sym/s
    /// convolutionally encoded data on a carrier at `dopp` Hz (the
    /// synth_gps shape with the SBAS 2 ms symbol structure). One 250-bit
    /// block per second, continuous encoder across blocks, as on air.
    fn synth_sbas(prn: usize, dopp: f64, blocks: &[Vec<u8>], fs: f64) -> Vec<Complex<f32>> {
        let bits: Vec<u8> = blocks.concat();
        let sym = crate::sbas::conv_encode(&bits, false, 0);
        let code = sbas_code(prn);
        let code_len = code.len();
        let ns_ms = (fs / 1000.0) as usize;
        let n = 2 * sym.len() * ns_ms;
        let mut sig = vec![Complex::new(0.0f32, 0.0); n];
        let mut st = 0x9e37_79b9_7f4a_7c15u64;
        let mut nxt = || {
            st ^= st << 13;
            st ^= st >> 7;
            st ^= st << 17;
            ((st >> 40) as f32 / 8_388_608.0) - 1.0
        };
        for (k, s) in sig.iter_mut().enumerate() {
            let t = k as f64 / fs;
            let ci = (k as f64 * 1.023e6 * (1.0 + dopp / F_L1) / fs) as usize;
            let c = code[ci % code_len];
            // soft > 0 means symbol bit 0, so bit 0 modulates as +1
            let bit = 1.0 - 2.0 * sym[k / (2 * ns_ms)] as f32;
            let ph = 2.0 * PI * dopp * t;
            let v = c * bit;
            s.re += v * ph.cos() as f32 + 0.05 * nxt();
            s.im += v * ph.sin() as f32 + 0.05 * nxt();
        }
        sig
    }

    /// The SBAS live hook end to end: a synthetic WAAS stream fed through
    /// the Channel (prompt-I collection -> sbas_tick -> symbols_from_prompt
    /// -> Viterbi -> frame sync -> CRC-24Q) must decode the encoded
    /// messages, and the summary must ride the report.
    #[test]
    fn sbas_channel_decodes_waas_messages() {
        let fs = 4.0e6;
        let dopp = 1200.0;
        let slots = vec![3u8, 7, 12, 18, 24, 29, 31, 36];
        let blocks: Vec<Vec<u8>> = (0..10)
            .map(|i| {
                if i % 2 == 0 {
                    sbas_block(i, 1, &mt1_payload(&slots, 2))
                } else {
                    sbas_block(i, 9, &mt9_payload())
                }
            })
            .collect();
        let sig = synth_sbas(131, dopp, &blocks, fs);
        let ns = (fs / 1000.0) as usize;
        let mut ch = Channel::new(Sys::Sbas, 131, fs, dopp, 0.0);
        let mut summ = None;
        let mut t = 0.0;
        for sec in sig.chunks(1000 * ns) {
            for chunk in sec.chunks(ns) {
                ch.process_epoch(chunk);
            }
            ch.end_second();
            t += 1.0;
            summ = ch.sbas_tick(t);
        }
        assert!(ch.locked, "strong synthetic must lock");
        let s = summ.expect("an SBAS channel must produce a summary");
        assert!(s.locked, "frame sync must lock on the clean stream");
        assert!(s.n_msgs >= 6, "decoded {} of 10 blocks", s.n_msgs);
        // every decoded block is one of the two encoded types (no Other)
        assert_eq!(
            s.types.get(&1).copied().unwrap_or(0) + s.types.get(&9).copied().unwrap_or(0),
            s.n_msgs,
            "unexpected message types: {:?}",
            s.types
        );
        // deep check: every decoded MT1 mask round-trips exactly
        let rep = ch.sbas_dec.decode(SBAS_MIN_BLOCKS);
        let masks: Vec<&Vec<u8>> = rep
            .messages
            .iter()
            .filter_map(|dm| match &dm.message {
                crate::sbas::Message::PrnMask { slots: sl, .. } => Some(sl),
                _ => None,
            })
            .collect();
        assert!(!masks.is_empty(), "no MT1 decoded");
        assert!(masks.iter().all(|m| **m == slots), "PRN mask mismatch");
    }

    /// An unlocked SBAS channel accumulates symbols but must not run the
    /// decode: the summary reports not-locked with zero messages.
    #[test]
    fn sbas_tick_unlocked_decodes_nothing() {
        let fs = 4.0e6;
        let ns = (fs / 1000.0) as usize;
        let mut ch = Channel::new(Sys::Sbas, 131, fs, 800.0, 0.0);
        let mut st = 0x1b87_3593u64;
        for _ in 0..2000 {
            let noise: Vec<Complex<f32>> = (0..ns)
                .map(|_| {
                    st ^= st << 13;
                    st ^= st >> 7;
                    st ^= st << 17;
                    let v = ((st >> 40) as f32 / 8_388_608.0) - 1.0;
                    Complex::new(0.05 * v, 0.05 * v)
                })
                .collect();
            ch.process_epoch(&noise);
        }
        ch.end_second();
        let s = ch.sbas_tick(1.0).expect("SBAS summary even when unlocked");
        assert!(!ch.locked, "pure noise must not lock");
        assert!(!s.locked);
        assert_eq!(s.n_msgs, 0);
    }

    // -----------------------------------------------------------------
    // SBAS correction cache semantics (review round 4)
    // -----------------------------------------------------------------

    fn push_bits(p: &mut Vec<u8>, v: u64, w: usize) {
        for i in (0..w).rev() {
            p.push(((v >> i) & 1) as u8);
        }
    }
    fn push_sbits(p: &mut Vec<u8>, v: i64, w: usize) {
        push_bits(p, (v as u64) & ((1u64 << w) - 1), w);
    }

    /// MT2 payload: 2-bit IODF, 2-bit IODP, 13 x 12-bit signed PRC,
    /// 13 x 4-bit UDREI (212 bits; MT3-5 differ only in the type field).
    fn mt2_payload(iodf: u8, iodp: u8, prc: &[i16; 13], udrei: &[u8; 13]) -> Vec<u8> {
        let mut p = Vec::with_capacity(212);
        push_bits(&mut p, iodf as u64, 2);
        push_bits(&mut p, iodp as u64, 2);
        for &v in prc {
            push_sbits(&mut p, v as i64, 12);
        }
        for &u in udrei {
            push_bits(&mut p, u as u64, 4);
        }
        assert_eq!(p.len(), 212);
        p
    }

    /// MT6 payload: 4 x 2-bit IODF + 51 x 4-bit UDREI (212 bits).
    fn mt6_payload(udrei: &[u8; 51]) -> Vec<u8> {
        let mut p = Vec::with_capacity(212);
        for _ in 0..4 {
            push_bits(&mut p, 0, 2);
        }
        for &u in udrei {
            push_bits(&mut p, u as u64, 4);
        }
        assert_eq!(p.len(), 212);
        p
    }

    /// MT25 payload: two velocity-code-0 long-term halves (106 bits each);
    /// each half carries two satellites addressed by mask ordinal, with
    /// dx = +1 m, dy = dz = 0, daf0 = 16 counts, IOD 42.
    fn mt25_payload(mask_nos: [u8; 2], iodp: u8) -> Vec<u8> {
        let mut p = Vec::with_capacity(212);
        for _ in 0..2 {
            push_bits(&mut p, 0, 1); // velocity code 0
            for &mn in mask_nos.iter() {
                push_bits(&mut p, mn as u64, 6);
                push_bits(&mut p, 42, 8); // IOD
                push_sbits(&mut p, 8, 9); // dx = +1.0 m
                push_sbits(&mut p, 0, 9);
                push_sbits(&mut p, 0, 9);
                push_sbits(&mut p, 16, 10); // daf0
            }
            push_bits(&mut p, iodp as u64, 2);
            push_bits(&mut p, 0, 1); // pad
        }
        assert_eq!(p.len(), 212);
        p
    }

    /// MT25 payload: first half velocity-code-1 (one satellite with rates
    /// and t_lt — the live WAAS case), second half velocity-code-0 (two
    /// satellites, dx = +1 m like mt25_payload). IOD 42, t_lt = 3600 s.
    fn mt25_vc1_payload(mask_no1: u8, mask_nos0: [u8; 2], iodp: u8) -> Vec<u8> {
        let mut p = Vec::with_capacity(212);
        push_bits(&mut p, 1, 1); // velocity code 1
        push_bits(&mut p, mask_no1 as u64, 6);
        push_bits(&mut p, 42, 8); // IOD
        push_sbits(&mut p, 8, 11); // dx = +1.0 m
        push_sbits(&mut p, 0, 11);
        push_sbits(&mut p, 0, 11);
        push_sbits(&mut p, 16, 11); // daf0
        push_sbits(&mut p, 16, 8); // ddx = 16 * 2^-11 m/s
        push_sbits(&mut p, 0, 8);
        push_sbits(&mut p, 0, 8);
        push_sbits(&mut p, 32, 8); // daf1 = 32 * 2^-39 s/s
        push_bits(&mut p, 225, 13); // t_lt = 225 * 16 = 3600 s
        push_bits(&mut p, iodp as u64, 2);
        push_bits(&mut p, 0, 1); // second half: velocity code 0
        for &mn in mask_nos0.iter() {
            push_bits(&mut p, mn as u64, 6);
            push_bits(&mut p, 42, 8); // IOD
            push_sbits(&mut p, 8, 9); // dx = +1.0 m
            push_sbits(&mut p, 0, 9);
            push_sbits(&mut p, 0, 9);
            push_sbits(&mut p, 16, 10); // daf0
        }
        push_bits(&mut p, iodp as u64, 2);
        push_bits(&mut p, 0, 1); // pad
        assert_eq!(p.len(), 212);
        p
    }

    /// Feed SBAS blocks into the channel as noiseless prompt-I pairs (two
    /// identical 1 ms prompts per 2 ms symbol — exactly what process_epoch
    /// produces on a clean capture), one block per simulated second, with
    /// sbas_tick after each. `t0` is the stream time feeding sbas_tick's
    /// freshness clock for the first block's second (the caller continues
    /// it across calls). Forces the lock flags: these tests exercise
    /// the decode/apply/publish path, not the tracking loops (those are
    /// covered by sbas_channel_decodes_waas_messages).
    fn feed_sbas_blocks(ch: &mut Channel, t0: f64, blocks: &[Vec<u8>]) -> Vec<SbasSummary> {
        let bits: Vec<u8> = blocks.concat();
        let sym = crate::sbas::conv_encode(&bits, false, 0);
        let mut out = Vec::new();
        for (i, chunk) in sym.chunks(500).enumerate() {
            for &s in chunk {
                let v = 1.0 - 2.0 * s as f64;
                ch.nav_ms.push(v);
                ch.nav_ms.push(v);
            }
            ch.locked = true;
            ch.lock_s += 1.0;
            out.push(ch.sbas_tick(t0 + (i + 1) as f64).expect("SBAS summary"));
        }
        out
    }

    /// A decoded MT1 carrying a DIFFERENT IODP than the held mask starts a
    /// new mask generation: every correction harvested under the old mask
    /// is invalid (DO-229D A.4.4.2 — fast/LT data is valid only with the
    /// mask whose IODP they carry) and must be cleared when the new mask
    /// is adopted.
    #[test]
    fn sbas_new_mask_generation_clears_corrections() {
        let mut ch = Channel::new(Sys::Sbas, 131, 4.0e6, 800.0, 0.0);
        let slots = vec![3u8, 7, 12, 18, 24, 29, 31, 36];
        let gen0 = vec![
            sbas_block(0, 1, &mt1_payload(&slots, 0)),
            sbas_block(1, 1, &mt1_payload(&slots, 0)),
            sbas_block(2, 1, &mt1_payload(&slots, 0)),
            sbas_block(3, 2, &mt2_payload(0, 0, &[8i16; 13], &[0u8; 13])),
            sbas_block(4, 25, &mt25_payload([1, 2], 0)),
        ];
        let sums = feed_sbas_blocks(&mut ch, 0.0, &gen0);
        assert!(!ch.sbas_prc.is_empty(), "MT2 rows must cache");
        assert!(!ch.sbas_lt.is_empty(), "MT25 rows must cache");
        assert!(!sums.last().unwrap().fast_corr.is_empty());
        // new generation: same mask content, IODP bumped
        let gen1 = vec![sbas_block(5, 1, &mt1_payload(&slots, 1))];
        let sums = feed_sbas_blocks(&mut ch, 5.0, &gen1);
        assert_eq!(ch.sbas_mask.as_ref().map(|(_, p)| *p), Some(1));
        assert!(
            ch.sbas_prc.is_empty(),
            "old-generation fast corrections must be cleared: {:?}",
            ch.sbas_prc
        );
        assert!(
            ch.sbas_lt.is_empty(),
            "old-generation LT corrections must be cleared: {:?}",
            ch.sbas_lt
        );
        assert!(sums.last().unwrap().fast_corr.is_empty());
    }

    /// A fresh fast-correction row with UDREI >= 14 (don't use / not
    /// monitored, DO-229D A.4.4.3 UDRE table) must EVICT the cached usable
    /// row for that PRN — not leave the stale usable one in place.
    #[test]
    fn sbas_udrei_dont_use_evicts_cached_row() {
        let mut ch = Channel::new(Sys::Sbas, 131, 4.0e6, 800.0, 0.0);
        let slots = vec![3u8, 7, 12, 18, 24, 29, 31, 36];
        let mut bad = [0u8; 13];
        bad[1] = 14; // ordinal 2 -> slot 7: don't use
        let blocks = vec![
            sbas_block(0, 1, &mt1_payload(&slots, 0)),
            sbas_block(1, 1, &mt1_payload(&slots, 0)),
            sbas_block(2, 1, &mt1_payload(&slots, 0)),
            sbas_block(3, 2, &mt2_payload(0, 0, &[8i16; 13], &[0u8; 13])),
        ];
        feed_sbas_blocks(&mut ch, 0.0, &blocks);
        assert!(ch.sbas_prc.contains_key(&7), "usable row must cache");
        let blocks2 = vec![sbas_block(4, 2, &mt2_payload(0, 0, &[8i16; 13], &bad))];
        let sums = feed_sbas_blocks(&mut ch, 4.0, &blocks2);
        assert!(
            !ch.sbas_prc.contains_key(&7),
            "a fresh don't-use row must evict the cached usable one"
        );
        assert!(ch.sbas_prc.contains_key(&3), "unaffected rows stay");
        assert!(sums.last().unwrap().fast_corr.iter().all(|r| r.0 != 7));
    }

    /// MT6 integrity UDREIs address mask ORDINALS 1..=51 (same through-mask
    /// mapping as MT2-5); a UDREI >= 14 there evicts the satellite's cached
    /// fast AND long-term corrections.
    #[test]
    fn sbas_mt6_evicts_dont_use_ordinals() {
        let mut ch = Channel::new(Sys::Sbas, 131, 4.0e6, 800.0, 0.0);
        let slots = vec![3u8, 7, 12, 18, 24, 29, 31, 36];
        let blocks = vec![
            sbas_block(0, 1, &mt1_payload(&slots, 0)),
            sbas_block(1, 1, &mt1_payload(&slots, 0)),
            sbas_block(2, 1, &mt1_payload(&slots, 0)),
            sbas_block(3, 2, &mt2_payload(0, 0, &[8i16; 13], &[0u8; 13])),
            sbas_block(4, 25, &mt25_payload([1, 2], 0)),
        ];
        feed_sbas_blocks(&mut ch, 0.0, &blocks);
        assert!(ch.sbas_prc.contains_key(&7));
        assert!(ch.sbas_lt.contains_key(&7), "LT row must cache");
        let mut udrei51 = [0u8; 51];
        udrei51[1] = 14; // ordinal 2 -> slot 7
        let blocks2 = vec![sbas_block(5, 6, &mt6_payload(&udrei51))];
        feed_sbas_blocks(&mut ch, 5.0, &blocks2);
        assert!(
            !ch.sbas_prc.contains_key(&7),
            "MT6 don't-use must evict the fast correction"
        );
        assert!(
            !ch.sbas_lt.contains_key(&7),
            "MT6 don't-use must evict the LT correction"
        );
        assert!(ch.sbas_prc.contains_key(&3), "unaffected rows stay");
        assert!(ch.sbas_lt.contains_key(&3), "unaffected rows stay");
    }

    /// REGRESSION (dead-LT finding): a decoded velocity-code-1 MT25 half
    /// must reach the published lt_corr rows — vc=1 is what WAAS broadcasts
    /// in practice, and skipping it left n_lt_corr = 0 in every live solve.
    /// DO-229D A.4.4.7 eq. (A-18)/(A-19): the application propagates
    /// base + rate·(t − t_lt) at solve time, so the rates and t_lt must
    /// ride along to the consumer.
    #[test]
    fn sbas_vc1_lt_rows_publish() {
        let mut ch = Channel::new(Sys::Sbas, 131, 4.0e6, 800.0, 0.0);
        let slots = vec![3u8, 7, 12, 18, 24, 29, 31, 36];
        let blocks = vec![
            sbas_block(0, 1, &mt1_payload(&slots, 0)),
            sbas_block(1, 1, &mt1_payload(&slots, 0)),
            sbas_block(2, 1, &mt1_payload(&slots, 0)),
            // vc=1 half: ordinal 2 -> slot 7; vc=0 half: ordinals 1, 3
            sbas_block(3, 25, &mt25_vc1_payload(2, [1, 3], 0)),
        ];
        let sums = feed_sbas_blocks(&mut ch, 0.0, &blocks);
        let s = sums.last().unwrap();
        let row = s
            .lt_corr
            .iter()
            .find(|r| r.corr.prn == 7)
            .expect("vc=1 LT row for PRN 7 must publish (the live WAAS case)");
        assert_eq!(row.corr.iod, 42);
        assert_eq!(row.corr.dx, 1.0);
        assert_eq!(row.corr.t_lt_s, Some(3600));
        assert_eq!(row.corr.ddx, Some(16.0 * 2.0f64.powi(-11)));
        assert_eq!(row.corr.daf1, Some(32.0 * 2.0f64.powi(-39)));
        assert!(row.age_s >= 0.0);
        // the vc=0 half of the same message still publishes, with no rates
        let row0 = s
            .lt_corr
            .iter()
            .find(|r| r.corr.prn == 3)
            .expect("vc=0 row still publishes");
        assert_eq!(row0.corr.t_lt_s, None);
        assert_eq!(row0.corr.daf1, None);
        // JSON contract (live_fix.rs is the consumer): one object per row,
        // rates/t_lt numbers for vc=1, null for vc=0, age_s present
        let v = serde_json::to_value(&s.lt_corr).unwrap();
        let arr = v.as_array().unwrap();
        let j1 = arr.iter().find(|r| r["prn"].as_u64() == Some(7)).unwrap();
        assert!(j1["ddx"].is_number() && j1["daf1"].is_number());
        assert_eq!(j1["t_lt_s"].as_u64(), Some(3600));
        assert!(j1["age_s"].is_number());
        let j0 = arr.iter().find(|r| r["prn"].as_u64() == Some(3)).unwrap();
        assert!(j0["ddx"].is_null() && j0["t_lt_s"].is_null());
        // the consumer's typed parse of a row round-trips the correction
        let back: crate::sbas::LtCorr = serde_json::from_value(j1.clone()).unwrap();
        assert_eq!(&back, &row.corr);
    }

    /// Correction freshness is measured on stream time (t_proc), which
    /// advances monotonically across RF unlock/relock — never on lock_s,
    /// which RESETS to 0 on unlock (end_second): a row cached at lock_s
    /// 500 and re-checked after a relock at lock_s 1 computed a NEGATIVE
    /// age and passed any freshness window (review round 4).
    #[test]
    fn sbas_freshness_survives_lock_s_reset() {
        let mut ch = Channel::new(Sys::Sbas, 131, 4.0e6, 800.0, 0.0);
        ch.locked = true;
        ch.sbas_mask = Some((vec![3u8, 7, 12], 0));
        // rows cached at stream t = 100 and t = 175 (the stamp the harvest
        // writes)
        ch.sbas_prc.insert(7, (1.5, 3, 100.0));
        ch.sbas_prc.insert(9, (0.5, 2, 175.0));
        // RF unlock -> lock_s = 0; relock -> the lock clock counts from 1
        ch.lock_s = 1.0;
        let s = ch.sbas_tick(181.0).expect("summary");
        assert!(
            s.fast_corr.iter().all(|r| r.0 != 7),
            "a row 81 s old must not re-pass the 60 s window after a lock_s reset: {:?}",
            s.fast_corr
        );
        // a genuinely fresh row (6 s old) still passes: the window runs on
        // true elapsed time, not on lock state
        let row9 = s.fast_corr.iter().find(|r| r.0 == 9).expect("fresh row");
        assert!((row9.3 - 6.0).abs() < 1e-9, "published age: {:?}", row9);
    }

    // -----------------------------------------------------------------
    // SBAS apply-once + generation termination (review round 6)
    // -----------------------------------------------------------------

    /// REGRESSION (the live replay bug): sbas_tick re-decodes the RETAINED
    /// symbol window every call, so the same blocks come back every second.
    /// The harvest must apply each block exactly once — a replayed message
    /// must NOT refresh the insert timestamp (observed live: cached WAAS
    /// rows showed age 0.0 forever, freshness fabricated by replay).
    #[test]
    fn sbas_replayed_window_never_refreshes_insert_age() {
        let mut ch = Channel::new(Sys::Sbas, 131, 4.0e6, 800.0, 0.0);
        let slots = vec![3u8, 7, 12, 18, 24, 29, 31, 36];
        let blocks = vec![
            sbas_block(0, 1, &mt1_payload(&slots, 0)),
            sbas_block(1, 1, &mt1_payload(&slots, 0)),
            sbas_block(2, 1, &mt1_payload(&slots, 0)),
            sbas_block(3, 2, &mt2_payload(0, 0, &[8i16; 13], &[0u8; 13])),
        ];
        feed_sbas_blocks(&mut ch, 0.0, &blocks);
        // the MT2 block was applied on its own tick: insert stamp 4.0
        assert_eq!(ch.sbas_prc.get(&3).map(|r| r.2), Some(4.0));
        // re-tick with NO new symbols: the same window decodes again...
        let s = ch.sbas_tick(5.0).expect("summary");
        assert!(s.locked, "the retained window still decodes locked");
        assert_eq!(s.n_msgs, 4, "the replay still REPORTS all window messages");
        // ...but the corrections must not be re-inserted: the age grows
        let row = s.fast_corr.iter().find(|r| r.0 == 3).expect("row");
        assert_eq!(row.3, 1.0, "a replayed block must not refresh the age");
        assert_eq!(ch.sbas_prc.get(&3).map(|r| r.2), Some(4.0));
        // and again on the next idle tick
        let s = ch.sbas_tick(6.0).expect("summary");
        assert_eq!(s.fast_corr.iter().find(|r| r.0 == 3).unwrap().3, 2.0);
    }

    /// Generation termination on RF unlock: the lock watchdog dropping the
    /// channel must kill the decoder window AND every correction cache.
    /// After relock, nothing from the dead generation comes back without
    /// NEW messages — and genuinely new blocks do re-populate the caches
    /// (the generation is dead, not wedged).
    #[test]
    fn sbas_unlock_terminates_generation() {
        let fs = 4.0e6;
        let ns = (fs / 1000.0) as usize;
        let mut ch = Channel::new(Sys::Sbas, 131, fs, 800.0, 0.0);
        let slots = vec![3u8, 7, 12, 18, 24, 29, 31, 36];
        let blocks = vec![
            sbas_block(0, 1, &mt1_payload(&slots, 0)),
            sbas_block(1, 1, &mt1_payload(&slots, 0)),
            sbas_block(2, 1, &mt1_payload(&slots, 0)),
            sbas_block(3, 2, &mt2_payload(0, 0, &[8i16; 13], &[0u8; 13])),
            sbas_block(4, 25, &mt25_payload([1, 2], 0)),
        ];
        feed_sbas_blocks(&mut ch, 0.0, &blocks);
        assert!(ch.sbas_mask.is_some() && !ch.sbas_prc.is_empty() && !ch.sbas_lt.is_empty());
        // drive the REAL unlock path: sub-threshold C/N0 until the
        // end_second watchdog fires (same noise shape as the slip test),
        // ticking sbas_tick every second exactly like Band::end_second does
        let mut st = 0x1b87_3593u64;
        let mut t = 5.0;
        for _ in 0..12 {
            for _ in 0..1000 {
                let noise: Vec<Complex<f32>> = (0..ns)
                    .map(|_| {
                        st ^= st << 13;
                        st ^= st >> 7;
                        st ^= st << 17;
                        let v = ((st >> 40) as f32 / 8_388_608.0) - 1.0;
                        Complex::new(0.05 * v, 0.05 * v)
                    })
                    .collect();
                ch.process_epoch(&noise);
            }
            ch.end_second();
            t += 1.0;
            ch.sbas_tick(t);
            if !ch.locked {
                break;
            }
        }
        assert!(!ch.locked, "the watchdog must drop the channel");
        assert!(ch.sbas_mask.is_none(), "the mask dies with the generation");
        assert!(ch.sbas_prc.is_empty(), "fast corrections die with it");
        assert!(ch.sbas_lt.is_empty(), "LT corrections die with it");
        assert!(ch.sbas_applied.is_none(), "the apply watermark resets");
        // relock: ticking with no NEW blocks must not resurrect any row
        ch.locked = true;
        for k in 0..3 {
            let s = ch.sbas_tick(30.0 + k as f64).expect("summary");
            assert!(s.fast_corr.is_empty() && s.lt_corr.is_empty());
        }
        assert!(ch.sbas_prc.is_empty() && ch.sbas_lt.is_empty() && ch.sbas_mask.is_none());
        // genuinely NEW blocks start the next generation cleanly
        let gen1 = vec![
            sbas_block(5, 1, &mt1_payload(&slots, 0)),
            sbas_block(6, 1, &mt1_payload(&slots, 0)),
            sbas_block(7, 1, &mt1_payload(&slots, 0)),
            sbas_block(8, 2, &mt2_payload(0, 0, &[8i16; 13], &[0u8; 13])),
        ];
        let sums = feed_sbas_blocks(&mut ch, 33.0, &gen1);
        assert!(!ch.sbas_prc.is_empty(), "new messages must re-populate");
        let row = sums
            .last()
            .unwrap()
            .fast_corr
            .iter()
            .find(|r| r.0 == 3)
            .unwrap();
        assert_eq!(row.3, 0.0, "a NEW insert is honestly age 0");
    }

    /// Generation termination on an input gap: Band::note_gap advances the
    /// stream clock past dropped samples — the SBAS symbol stream cannot
    /// cross the gap, so every channel's decode generation (window +
    /// caches) dies with it.
    #[test]
    fn sbas_gap_terminates_generation() {
        let mut ch = Channel::new(Sys::Sbas, 131, 4.0e6, 800.0, 0.0);
        let slots = vec![3u8, 7, 12, 18, 24, 29, 31, 36];
        let blocks = vec![
            sbas_block(0, 1, &mt1_payload(&slots, 0)),
            sbas_block(1, 1, &mt1_payload(&slots, 0)),
            sbas_block(2, 1, &mt1_payload(&slots, 0)),
            sbas_block(3, 2, &mt2_payload(0, 0, &[8i16; 13], &[0u8; 13])),
        ];
        feed_sbas_blocks(&mut ch, 0.0, &blocks);
        assert!(!ch.sbas_prc.is_empty());
        let mut band = Band::new_l1(4.0e6, 0.0);
        band.channels.push(ch);
        band.note_gap(4_000_000); // 1 s of dropped band samples
        assert!((band.in_t - 1.0).abs() < 1e-9, "the clock still advances");
        let ch = &band.channels[0];
        assert!(ch.sbas_mask.is_none(), "the mask dies on the gap");
        assert!(ch.sbas_prc.is_empty(), "fast corrections die on the gap");
        assert!(ch.sbas_lt.is_empty());
        assert!(ch.sbas_igpmask.is_empty() && ch.sbas_iono.is_empty());
        assert!(ch.sbas_applied.is_none(), "the apply watermark resets");
    }
}
