//! Iridium message classification and decoding.
//!
//! A demodulated burst starts with a 24-bit access word saying which direction
//! and access scheme it belongs to. What follows is interleaved: symbol pairs
//! are reversed and read out with a stride, so the 32-bit BCH blocks are not
//! contiguous in the received bit stream. Only after de-interleaving can a BCH
//! generator be tried, and it is the generator that *divides cleanly* which
//! identifies the class. Classification and error correction are the same step.
//!
//! Ported from iridium-toolkit's bitsparser.py and checked against it frame by
//! frame on real recordings -- see tests/iridium_golden.rs. Nothing here is
//! validated only against its own expectations, because a decoder that grades
//! its own homework agrees with its own mistakes; that is exactly how 39 SBAS
//! ranging codes stayed time-mirrored in this project for weeks.
//!
//! On message CONTENT: ring alert and broadcast carry system-level information
//! (which satellite, which beam, which channel, system timing) and are decoded
//! fully. Messaging bursts carry third-party paging traffic; their structure is
//! reported so the link can be characterised, but their payload is deliberately
//! left packed.

use super::bch;

pub const ACCESS_DOWNLINK: &str = "001100000011000011110011";
pub const ACCESS_UPLINK: &str = "110011000011110011111100";
pub const ACCESS_NEXT_DL: &str = "110011110011111111111100";
pub const ACCESS_NEXT_UL: &str = "001111000000000011111111";

const HEADER_MESSAGING: &str = "00110011111100110011001111110011";

/// The access words again, but as DQPSK SYMBOLS rather than bits.
///
/// A burst whose unique word arrived with a few bit errors is still a real
/// burst, and refusing it throws away a frame whose payload may be perfectly
/// recoverable -- the BCH blocks that follow have their own protection. The
/// comparison has to happen in symbol space: the bits are differentially
/// encoded, so a single symbol error corrupts a PAIR of bits and a plain
/// Hamming distance on the raw bits overstates the damage.
///
/// Measured on this station's 37,164-frame corpus, allowing up to three symbol
/// errors recovers about 1,700 extra ring alerts, and the ones that reach
/// catalogue_build survive its independent TLE cross-check -- so they are real
/// bursts, not noise admitted by a loose gate.
const UW_DOWNLINK: [u8; 12] = [0, 2, 2, 2, 2, 0, 0, 0, 2, 0, 0, 2];
const UW_UPLINK: [u8; 12] = [2, 2, 0, 0, 0, 2, 0, 0, 2, 0, 2, 2];
const NXT_UW_DOWNLINK: [u8; 12] = [2, 2, 0, 2, 2, 0, 2, 0, 2, 0, 2, 2];
const NXT_UW_UPLINK: [u8; 12] = [0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0];

/// Largest symbol distance still accepted as a unique word.
pub const UW_MAX_ERRORS: u32 = 3;

/// How hard to try before calling a burst undecodable.
///
/// This is a real trade, measured rather than assumed:
///
///   Strict -- every BCH block must divide its generator exactly. Zero false
///   alarms in 800 pure-noise bursts, and 2860 ring alerts from this station's
///   37,164-frame corpus.
///
///   Harder -- blocks need only be REPAIRABLE, backed by the 32nd parity bit.
///   That lifts the yield to 5283 ring alerts, but BCH repair accepts about
///   half of all random syndromes and parity only halves that again, so 20 of
///   800 noise bursts get through: a 2.5% false alarm rate.
///
/// Neither is right in general. Harder pays off for a consumer that validates
/// downstream -- catalogue_build cross-checks every position against TLE
/// propagation and simply rejects what does not fit -- and is wrong for anything
/// that trusts the decoder's word. Hence a switch, defaulting to Strict.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Effort {
    Strict,
    Harder,
}

impl Default for Effort {
    fn default() -> Self {
        Effort::Strict
    }
}

/// Undo the DQPSK mapping: bit pairs to symbols, then remove the differential
/// encoding by accumulating.
fn de_dqpsk(bits: &str) -> Vec<u8> {
    const IMAP: [u8; 4] = [0, 1, 3, 2];
    let b = bits.as_bytes();
    let mut out = Vec::with_capacity(bits.len() / 2);
    let mut i = 0;
    while i + 1 < b.len() {
        let v = ((b[i] - b'0') * 2 + (b[i + 1] - b'0')) as usize;
        out.push(IMAP[v & 3]);
        i += 2;
    }
    for c in 1..out.len() {
        out[c] = (out[c - 1] + out[c]) % 4;
    }
    out
}

fn symbol_distance(a: &[u8], b: &[u8]) -> u32 {
    a.iter().zip(b.iter()).filter(|(x, y)| x != y).count() as u32
}

/// Does a 32-bit block pass its BCH and its parity bit?
///
/// The stricter test -- demanding the 31-bit block divide the generator exactly
/// -- only accepts blocks that arrived undamaged. This accepts blocks that can
/// be REPAIRED, and then leans on the 32nd bit, an overall parity check, to
/// confirm the repair was the right one. That second step is what keeps it
/// honest: BCH will happily "correct" an uncorrectable block to some codeword,
/// and the parity bit catches a large share of those.
///
/// Returns the corrected data bits and the number of errors fixed, or None.
fn block_ok(block: &str, poly: u32, effort: Effort) -> Option<(String, u32)> {
    if block.len() < 32 {
        return None;
    }
    if effort == Effort::Strict {
        // Exact divisibility, and nothing else -- this is what the reference
        // does on its default path, and matching it exactly is what lets the
        // golden test compare the two frame for frame. About one random block
        // in 1024 passes, so three together is one in 2^30.
        if bch::gf2_remainder_bits(poly, &block[..31]) != 0 {
            return None;
        }
        let r = bch::repair(poly, &block[..31]);
        return r.errors.map(|e| (r.data, e as u32));
    }
    // Harder: accept a block that can be REPAIRED, then use the 32nd bit -- an
    // overall parity check -- to test whether the repair was the right one.
    // Without that second step BCH would happily "correct" noise to some
    // codeword; with it, roughly half of those are caught.
    let r = bch::repair(poly, &block[..31]);
    let errs = r.errors?;
    let ones = r.data.bytes().filter(|c| *c == b'1').count()
        + r.check.bytes().filter(|c| *c == b'1').count()
        + if block.as_bytes()[31] == b'1' { 1 } else { 0 };
    if ones % 2 != 0 {
        return None;
    }
    Some((r.data, errs as u32))
}

/// Channel-plan boundaries the reference uses to gate classification.
/// Simplex traffic (ring alert, messaging) sits above the first; duplex traffic
/// (broadcast, LCW) below the second.
pub const F_SIMPLEX_HZ: f64 = 1_626_104_000.0;
pub const F_DUPLEX_HZ: f64 = 1_625_979_000.0;

/// What a burst turned out to be.
#[derive(Debug, Clone, PartialEq)]
pub enum Class {
    /// ring alert: satellite id, beam id and the satellite's own position
    RingAlert(Ira),
    /// broadcast controller: satellite, beam and acquisition channel plan
    Broadcast(Ibc),
    /// messaging: structure only, payload left packed
    Messaging(Ms),
    /// time / location broadcast
    TimeLocation,
    /// link control word
    Lcw,
    /// an Iridium NEXT access word
    Next,
    /// uplink rather than downlink
    Uplink,
    /// carried a known access word but matched no class
    Unknown,
    /// no recognised access word at all
    NoAccess,
}

impl Class {
    /// Short label, matching the names iridium-toolkit prints.
    pub fn label(&self) -> &'static str {
        match self {
            Class::RingAlert(_) => "IRA",
            Class::Broadcast(_) => "IBC",
            Class::Messaging(_) => "MS",
            Class::TimeLocation => "TL",
            Class::Lcw => "LW",
            Class::Next => "NX",
            Class::Uplink => "UL",
            Class::Unknown => "unknown",
            Class::NoAccess => "no-access",
        }
    }
}

/// The contents of a ring-alert frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Ira {
    pub sat: u32,
    pub beam: u32,
    pub pos_x: i32,
    pub pos_y: i32,
    pub pos_z: i32,
    /// 90 ms slot index within this satellite/beam
    pub interval: u32,
    pub slot: u32,
    pub eip: u32,
    /// downlink sub-band the broadcast channel is on
    pub bc_sb: u32,
    /// geocentric latitude, degrees (NOT geodetic: up to 0.2 deg low)
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// radius from the geocentre, km
    pub alt_km: f64,
    /// bits corrected by BCH across the frame
    pub corrected: u32,
}

/// The contents of a broadcast-controller frame.
///
/// This is the type that carries the constellation's own timing and channel
/// plan, and the one a clock-discipline measurement needs.
#[derive(Debug, Clone, PartialEq)]
pub struct Ibc {
    /// header type field, after BCH repair of the 6-bit header
    pub bc_type: u32,
    pub sv_id: u32,
    pub beam_id: u32,
    pub slot: u32,
    pub sv_blocking: u32,
    pub acqu_subband: u32,
    pub acqu_channels: u32,
    pub corrected: u32,
}

/// Structure of a messaging burst. The payload is intentionally not unpacked.
#[derive(Debug, Clone, PartialEq)]
pub struct Ms {
    /// number of 32-bit BCH blocks recovered
    pub blocks: usize,
    /// payload bits recovered after error correction
    pub payload_bits: usize,
    pub corrected: u32,
}

/// Reverse each symbol pair, then read out with stride 2 -- the 64-bit case.
fn de_interleave(group: &str) -> (String, String) {
    let b: Vec<char> = group.chars().collect();
    let symbols: Vec<String> = (0..b.len())
        .step_by(2)
        .filter(|&z| z + 1 < b.len())
        .map(|z| format!("{}{}", b[z + 1], b[z]))
        .collect();
    let n = symbols.len() as i32;
    let take = |start: i32| -> String {
        let mut s = String::new();
        let mut i = start;
        while i >= 0 {
            s.push_str(&symbols[i as usize]);
            i -= 2;
        }
        s
    };
    (take(n - 1), take(n - 2))
}

/// The stride-3 variant, used for the first 96 bits of a ring alert.
fn de_interleave3(group: &str) -> (String, String, String) {
    let b: Vec<char> = group.chars().collect();
    let symbols: Vec<String> = (0..b.len())
        .step_by(2)
        .filter(|&z| z + 1 < b.len())
        .map(|z| format!("{}{}", b[z + 1], b[z]))
        .collect();
    let n = symbols.len() as i32;
    let take = |start: i32| -> String {
        let mut s = String::new();
        let mut i = start;
        while i >= 0 {
            s.push_str(&symbols[i as usize]);
            i -= 3;
        }
        s
    };
    (take(n - 1), take(n - 2), take(n - 3))
}

/// Split into fixed-size chunks, returning the remainder separately.
fn slice_extra(data: &str, size: usize) -> (Vec<String>, String) {
    let mut out = Vec::new();
    let mut i = 0;
    while i + size <= data.len() {
        out.push(data[i..i + size].to_string());
        i += size;
    }
    (out, data[i..].to_string())
}

fn bits_to_u32(bits: &str) -> u32 {
    bits.chars().fold(0u32, |a, c| (a << 1) | if c == '1' { 1 } else { 0 })
}

/// Sign-extend an 11-bit magnitude preceded by its sign bit.
fn signed12(bits: &str, sign_at: usize, mag: std::ops::Range<usize>) -> i32 {
    let m = bits_to_u32(&bits[mag]) as i32;
    let s = if bits.as_bytes()[sign_at] == b'1' { 1 } else { 0 };
    m - s * (1 << 11)
}

/// Run a list of 32-bit blocks through BCH, concatenating the data halves.
///
/// Stops at the first uncorrectable block and keeps what came before, rather
/// than discarding the burst. That matters for ring alerts, whose fields all
/// live in the first 63 payload bits: a corrupt block near the END of a burst
/// should not throw away a satellite position that was received cleanly. It is
/// why this decoder recovers 129 ring alerts that the reference declines. Those
/// extras show the same position distribution as the frames both decoders agree
/// on -- the documented alternation between spacecraft and spot-beam positions
/// -- which is consistent with their being real. It is weaker evidence than a
/// single clean orbital radius would be, because half of ALL ring alerts report
/// a surface point by design.
fn decode_blocks(blocks: &[String], poly: u32) -> (String, u32, usize) {
    let mut payload = String::new();
    let mut corrected = 0u32;
    let mut used = 0usize;
    for blk in blocks {
        if blk.len() < 31 {
            break;
        }
        let r = bch::repair(poly, &blk[..31]);
        match r.errors {
            None => break,
            Some(n) => corrected += n as u32,
        }
        payload.push_str(&r.data);
        used += 1;
    }
    (payload, corrected, used)
}

/// Classify a burst without consulting its frequency.
pub fn classify(bits: &str) -> Class {
    classify_at(bits, None)
}

/// Classify with a chosen effort level. See `Effort`.
pub fn classify_effort(bits: &str, freq_hz: Option<f64>, effort: Effort) -> Class {
    classify_inner(bits, freq_hz, effort)
}

/// Classify a burst, optionally applying the channel-plan gates.
///
/// iridium-toolkit only considers a burst for the simplex classes when it is
/// above `F_SIMPLEX_HZ`, and for the duplex classes when below `F_DUPLEX_HZ`.
/// Passing the frequency reproduces that; passing None tries every class, which
/// recovers real frames the gate would exclude but relies on the BCH checks
/// alone to keep noise out.
pub fn classify_at(bits: &str, freq_hz: Option<f64>) -> Class {
    classify_inner(bits, freq_hz, Effort::Strict)
}

fn classify_inner(bits: &str, freq_hz: Option<f64>, effort: Effort) -> Class {
    let uw_len = ACCESS_DOWNLINK.len();
    let mut uw_errors = 0u32;
    let data = if let Some(d) = bits.strip_prefix(ACCESS_DOWNLINK) {
        d
    } else if bits.starts_with(ACCESS_UPLINK) {
        return Class::Uplink;
    } else if bits.starts_with(ACCESS_NEXT_DL) || bits.starts_with(ACCESS_NEXT_UL) {
        return Class::Next;
    } else if effort == Effort::Harder && bits.len() >= uw_len {
        // No exact match. Try again in symbol space, allowing a few errors --
        // see UW_DOWNLINK above for why this is worth doing and why it is not
        // simply a looser gate.
        let sym = de_dqpsk(&bits[..uw_len]);
        let d_dl = symbol_distance(&sym, &UW_DOWNLINK);
        let d_ul = symbol_distance(&sym, &UW_UPLINK);
        let d_ndl = symbol_distance(&sym, &NXT_UW_DOWNLINK);
        let d_nul = symbol_distance(&sym, &NXT_UW_UPLINK);
        let best = d_dl.min(d_ul).min(d_ndl).min(d_nul);
        if best > UW_MAX_ERRORS {
            return Class::NoAccess;
        }
        if best == d_ul {
            return Class::Uplink;
        }
        if best == d_ndl || best == d_nul {
            return Class::Next;
        }
        uw_errors = d_dl;
        &bits[uw_len..]
    } else {
        return Class::NoAccess;
    };

    let simplex_ok = freq_hz.map_or(true, |f| f > F_SIMPLEX_HZ);
    let duplex_ok = freq_hz.map_or(true, |f| f < F_DUPLEX_HZ);

    // order follows the reference: messaging, time/location, broadcast, LCW,
    // then ring alert
    if simplex_ok && data.len() >= 32 && &data[..32] == HEADER_MESSAGING {
        if let Some(ms) = decode_ms(data) {
            return Class::Messaging(ms);
        }
        return Class::Messaging(Ms { blocks: 0, payload_bits: 0, corrected: 0 });
    }

    if simplex_ok
        && data.len() >= 96
        && &data[..2] == "11"
        && data[2..96].chars().all(|c| c == '0')
    {
        return Class::TimeLocation;
    }

    if duplex_ok {
        if let Some(mut ibc) = decode_ibc(data, effort) {
            ibc.corrected += uw_errors;
            return Class::Broadcast(ibc);
        }
    }

    const FIRST: usize = 3 * 32;
    if simplex_ok && data.len() >= FIRST {
        let (a, b, c) = de_interleave3(&data[..FIRST]);
        // All three blocks must survive BCH repair AND their parity bit. Any
        // one block alone matches by chance roughly once in 2^11 with the
        // parity included, so three together is about once in 2^33.
        if block_ok(&a, bch::RINGALERT, effort).is_some()
            && block_ok(&b, bch::RINGALERT, effort).is_some()
            && block_ok(&c, bch::RINGALERT, effort).is_some()
        {
            if let Some(mut ira) = decode_ira(data) {
                ira.corrected += uw_errors;
                return Class::RingAlert(ira);
            }
        }
    }
    Class::Unknown
}

/// Rebuild the payload of a ring alert and unpack its fields.
fn decode_ira(data: &str) -> Option<Ira> {
    const FIRST: usize = 3 * 32;
    if data.len() < FIRST {
        return None;
    }
    let mut blocks: Vec<String> = Vec::new();
    let (a, b, c) = de_interleave3(&data[..FIRST]);
    blocks.push(a);
    blocks.push(b);
    blocks.push(c);
    let (groups, _extra) = slice_extra(&data[FIRST..], 64);
    for g in groups {
        let (x, y) = de_interleave(&g);
        blocks.push(x);
        blocks.push(y);
    }

    let (payload, corrected, _) = decode_blocks(&blocks, bch::RINGALERT);
    if payload.len() < 63 {
        return None;
    }
    let pos_x = signed12(&payload, 13, 14..25);
    let pos_y = signed12(&payload, 25, 26..37);
    let pos_z = signed12(&payload, 37, 38..49);
    let (fx, fy, fz) = (pos_x as f64, pos_y as f64, pos_z as f64);
    Some(Ira {
        sat: bits_to_u32(&payload[0..7]),
        beam: bits_to_u32(&payload[7..13]),
        pos_x,
        pos_y,
        pos_z,
        interval: bits_to_u32(&payload[49..56]),
        slot: bits_to_u32(&payload[56..57]),
        eip: bits_to_u32(&payload[57..58]),
        bc_sb: bits_to_u32(&payload[58..63]),
        lat_deg: fz.atan2((fx * fx + fy * fy).sqrt()).to_degrees(),
        lon_deg: fy.atan2(fx).to_degrees(),
        alt_km: (fx * fx + fy * fy + fz * fz).sqrt() * 4.0,
        corrected,
    })
}

/// Broadcast controller: a 6-bit BCH header, then 64-bit interleaved groups.
///
/// A broadcast burst has a 64-symbol preamble rather than 16, which caps it at
/// 131 symbols where other duplex traffic reaches 179.
fn decode_ibc(data: &str, effort: Effort) -> Option<Ibc> {
    const HDRLEN: usize = 6;
    const BLOCKLEN: usize = 64;
    const IBCLEN: usize = 131 * 2;
    if data.len() <= HDRLEN + BLOCKLEN {
        return None;
    }
    if bch::gf2_remainder_bits(bch::HDR, &data[..HDRLEN]) != 0 {
        return None;
    }
    let (b1, b2) = de_interleave(&data[HDRLEN..HDRLEN + BLOCKLEN]);
    // repairable plus parity, as for the ring alert above
    if block_ok(&b1, bch::RINGALERT, effort).is_none()
        || block_ok(&b2, bch::RINGALERT, effort).is_none() {
        return None;
    }

    let hdr = bch::repair(bch::HDR, &format!("{:0>31}", &data[..HDRLEN]));
    let bc_type = bits_to_u32(hdr.data.trim_start_matches('0'));

    let end = IBCLEN.min(data.len());
    let (groups, _extra) = slice_extra(&data[HDRLEN..end], BLOCKLEN);
    let mut blocks: Vec<String> = Vec::new();
    for g in groups {
        let (x, y) = de_interleave(&g);
        blocks.push(x);
        blocks.push(y);
    }
    let (payload, corrected, used) = decode_blocks(&blocks, bch::RINGALERT);
    if used == 0 || payload.len() < 40 {
        return None;
    }
    Some(Ibc {
        bc_type,
        sv_id: bits_to_u32(&payload[0..7]),
        beam_id: bits_to_u32(&payload[7..13]),
        slot: bits_to_u32(&payload[14..15]),
        sv_blocking: bits_to_u32(&payload[15..16]),
        acqu_subband: bits_to_u32(&payload[32..37]),
        acqu_channels: bits_to_u32(&payload[37..40]),
        corrected,
    })
}

/// Messaging: a 32-bit header then 64-bit interleaved groups. Structure only.
fn decode_ms(data: &str) -> Option<Ms> {
    const HDRLEN: usize = 32;
    if data.len() <= HDRLEN {
        return None;
    }
    let (groups, _extra) = slice_extra(&data[HDRLEN..], 64);
    let mut blocks: Vec<String> = Vec::new();
    for g in groups {
        let (x, y) = de_interleave(&g);
        blocks.push(x);
        blocks.push(y);
    }
    let (payload, corrected, used) = decode_blocks(&blocks, bch::MESSAGING);
    Some(Ms {
        blocks: used,
        payload_bits: payload.len(),
        corrected,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a burst whose first blocks are valid BCH codewords, so the
    /// classifier has something real to find.
    fn ra_burst(payload_bits: &str) -> String {
        // three stride-3 blocks are needed; construct them then re-interleave
        let mut blocks: Vec<u32> = Vec::new();
        for i in 0..3 {
            let chunk: String = payload_bits
                .chars()
                .skip(i * 21)
                .take(21)
                .chain(std::iter::repeat('0'))
                .take(21)
                .collect();
            let d = u32::from_str_radix(&chunk, 2).unwrap() << 10;
            blocks.push(d | bch::gf2_remainder(bch::RINGALERT, d));
        }
        // 31 bits + 1 parity = 32 per block; re-interleave stride 3
        let bits: Vec<String> = blocks
            .iter()
            .map(|w| {
                let mut s: String = (0..31)
                    .map(|i| if (w >> (30 - i)) & 1 == 1 { '1' } else { '0' })
                    .collect();
                // the 32nd bit is an overall parity check over the block; a real
                // transmitter sets it, so a synthetic frame must too
                let ones = s.bytes().filter(|c| *c == b'1').count();
                s.push(if ones % 2 == 1 { '1' } else { '0' });
                s
            })
            .collect();
        let mut symbols: Vec<String> = vec![String::new(); 48];
        for (bi, b) in bits.iter().enumerate() {
            let sym: Vec<String> = (0..16).map(|k| b[k * 2..k * 2 + 2].to_string()).collect();
            for (k, s) in sym.iter().enumerate() {
                // block a came from index 47-3k, b from 46-3k, c from 45-3k
                symbols[47 - (k * 3 + bi)] = s.clone();
            }
        }
        let inter: String = symbols
            .iter()
            .map(|s| {
                let c: Vec<char> = s.chars().collect();
                format!("{}{}", c[1], c[0])
            })
            .collect();
        format!("{}{}", ACCESS_DOWNLINK, inter)
    }

    #[test]
    fn a_burst_without_an_access_word_is_rejected() {
        assert_eq!(classify(&"1".repeat(200)), Class::NoAccess);
    }

    #[test]
    fn uplink_and_next_access_words_are_recognised() {
        assert_eq!(classify(&format!("{}{}", ACCESS_UPLINK, "0".repeat(200))), Class::Uplink);
        assert_eq!(classify(&format!("{}{}", ACCESS_NEXT_DL, "0".repeat(200))), Class::Next);
        assert_eq!(classify(&format!("{}{}", ACCESS_NEXT_UL, "0".repeat(200))), Class::Next);
    }

    #[test]
    fn a_messaging_header_is_recognised() {
        let b = format!("{}{}{}", ACCESS_DOWNLINK, HEADER_MESSAGING, "0".repeat(256));
        match classify(&b) {
            Class::Messaging(ms) => assert!(ms.blocks > 0, "no blocks recovered"),
            other => panic!("expected messaging, got {:?}", other),
        }
    }

    #[test]
    fn a_time_location_header_is_recognised() {
        let b = format!("{}11{}{}", ACCESS_DOWNLINK, "0".repeat(94), "1".repeat(64));
        assert_eq!(classify(&b), Class::TimeLocation);
    }

    #[test]
    fn a_ring_alert_round_trips_through_classification() {
        // 63 payload bits: sat 51, beam 27, then zeros
        let mut p = String::new();
        p.push_str(&format!("{:07b}", 51));
        p.push_str(&format!("{:06b}", 27));
        p.push_str(&"0".repeat(50));
        let burst = ra_burst(&p);
        match classify(&burst) {
            Class::RingAlert(ira) => {
                assert_eq!(ira.sat, 51);
                assert_eq!(ira.beam, 27);
            }
            other => panic!("expected a ring alert, got {:?}", other),
        }
    }

    #[test]
    fn noise_after_a_valid_access_word_does_not_become_a_message() {
        // The access word alone must never be enough.
        let mut alarms = 0;
        for seed in 0u64..800 {
            let mut s = String::from(ACCESS_DOWNLINK);
            let mut x = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            for _ in 0..500 {
                x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                s.push(if (x >> 33) & 1 == 1 { '1' } else { '0' });
            }
            match classify(&s) {
                Class::RingAlert(_) | Class::Broadcast(_) => alarms += 1,
                _ => {}
            }
        }
        assert_eq!(alarms, 0, "{} noise bursts decoded as real messages", alarms);
    }

    #[test]
    fn the_access_words_and_their_symbol_patterns_agree() {
        // The strongest check available on de_dqpsk and on the symbol constants
        // at once: de-differentiating the exact access word must reproduce the
        // symbol pattern the error-correcting path compares against. If either
        // were wrong, unique-word correction would be measuring the distance to
        // the wrong target and would quietly accept the wrong bursts.
        assert_eq!(de_dqpsk(ACCESS_DOWNLINK), UW_DOWNLINK.to_vec());
        assert_eq!(de_dqpsk(ACCESS_UPLINK), UW_UPLINK.to_vec());
        assert_eq!(de_dqpsk(ACCESS_NEXT_DL), NXT_UW_DOWNLINK.to_vec());
        assert_eq!(de_dqpsk(ACCESS_NEXT_UL), NXT_UW_UPLINK.to_vec());
    }

    #[test]
    fn de_dqpsk_undoes_the_differential_encoding() {
        // an unchanging symbol stream differentiates to zeros
        assert_eq!(de_dqpsk("00000000"), vec![0, 0, 0, 0]);
        // and the mapping is 00->0, 01->1, 11->3, 10->2 before accumulation
        assert_eq!(de_dqpsk("0001"), vec![0, 1]);
        assert_eq!(de_dqpsk(""), Vec::<u8>::new());
        assert_eq!(de_dqpsk("0"), Vec::<u8>::new());   // half a symbol is none
    }

    #[test]
    fn the_default_effort_is_strict() {
        // A caller that does not think about it must get the mode that cannot
        // invent messages.
        assert_eq!(Effort::default(), Effort::Strict);
    }

    #[test]
    fn a_corrupted_unique_word_is_recovered_only_in_harder_mode() {
        let mut p = String::new();
        p.push_str(&format!("{:07b}", 23));
        p.push_str(&format!("{:06b}", 9));
        p.push_str(&"0".repeat(50));
        let good = ra_burst(&p);
        // Damage the LAST symbol of the access word. Position matters a great
        // deal here and it is worth being explicit about why: de_dqpsk undoes
        // differential encoding by accumulating, so an error early in the word
        // shifts every symbol after it and blows the distance well past the
        // limit, while an error in the final symbol costs exactly one. Error
        // correction on a differential unique word is therefore far more
        // forgiving at the end than at the start.
        let mut b: Vec<char> = good.chars().collect();
        b[22] = if b[22] == '1' { '0' } else { '1' };
        let damaged: String = b.into_iter().collect();

        assert_eq!(classify_effort(&damaged, None, Effort::Strict), Class::NoAccess,
                   "strict mode must not accept a damaged unique word");
        match classify_effort(&damaged, None, Effort::Harder) {
            Class::RingAlert(ira) => {
                assert_eq!((ira.sat, ira.beam), (23, 9));
                assert!(ira.corrected >= 1, "the unique-word errors were not counted");
            }
            other => panic!("harder mode should have recovered it, got {:?}", other),
        }
    }

    #[test]
    fn a_random_access_word_is_still_rejected() {
        // Error correction is not licence to accept anything. Rather than pick
        // one "obviously broken" example -- which is easy to get wrong, since a
        // differential word can fold back onto itself -- measure the rate at
        // which random 24-bit prefixes are mistaken for a unique word.
        let mut accepted = 0;
        let n = 600;
        for seed in 0u64..n {
            let mut x = seed.wrapping_mul(0x9E3779B97F4A7C15).wrapping_add(17);
            let mut s = String::new();
            for _ in 0..24 {
                x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                s.push(if (x >> 33) & 1 == 1 { '1' } else { '0' });
            }
            s.push_str(&"0".repeat(300));
            if classify_effort(&s, None, Effort::Harder) != Class::NoAccess {
                accepted += 1;
            }
        }
        // 12 symbols, four patterns, distance up to 3: a small but nonzero
        // share of random words land close enough. It must stay small.
        assert!(accepted * 20 < n,
                "{} of {} random access words were accepted; the distance limit \
                 is too loose", accepted, n);
    }

    #[test]
    fn harder_mode_still_recognises_uplink_and_next() {
        // the corrected path has to route direction correctly, not just accept
        let mut b: Vec<char> = format!("{}{}", ACCESS_UPLINK, "0".repeat(300)).chars().collect();
        b[22] = if b[22] == '1' { '0' } else { '1' };   // last symbol, see above
        let s: String = b.into_iter().collect();
        assert_eq!(classify_effort(&s, None, Effort::Harder), Class::Uplink);
    }

    #[test]
    fn the_two_effort_levels_trade_yield_against_false_alarms() {
        // The whole point of the switch, asserted rather than described.
        // Strict must never manufacture a message from noise; Harder is allowed
        // to, and does, which is why it is not the default.
        let mut strict = 0;
        let mut harder = 0;
        for seed in 0u64..800 {
            let mut s = String::from(ACCESS_DOWNLINK);
            let mut x = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            for _ in 0..500 {
                x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                s.push(if (x >> 33) & 1 == 1 { '1' } else { '0' });
            }
            if matches!(classify_effort(&s, None, Effort::Strict),
                        Class::RingAlert(_) | Class::Broadcast(_)) { strict += 1; }
            if matches!(classify_effort(&s, None, Effort::Harder),
                        Class::RingAlert(_) | Class::Broadcast(_)) { harder += 1; }
        }
        assert_eq!(strict, 0, "strict mode invented {} messages from noise", strict);
        assert!(harder > 0,
                "harder mode showed no false alarms in 800 noise bursts; either \
                 the measurement changed or the mode is not doing anything");
        assert!(harder < 80, "harder mode false alarm rate exceeded 10%: {}", harder);
    }

    #[test]
    fn the_frequency_gate_reproduces_the_reference_behaviour() {
        let mut p = String::new();
        p.push_str(&format!("{:07b}", 12));
        p.push_str(&format!("{:06b}", 3));
        p.push_str(&"0".repeat(50));
        let burst = ra_burst(&p);
        // above the simplex boundary a ring alert is allowed
        assert!(matches!(classify_at(&burst, Some(1_626_300_000.0)), Class::RingAlert(_)));
        // below it the reference declines to consider the class at all
        assert!(!matches!(classify_at(&burst, Some(1_625_900_000.0)), Class::RingAlert(_)));
        // with no frequency supplied, every class is tried
        assert!(matches!(classify_at(&burst, None), Class::RingAlert(_)));
    }

    #[test]
    fn labels_match_the_reference_names() {
        assert_eq!(Class::TimeLocation.label(), "TL");
        assert_eq!(Class::Next.label(), "NX");
        assert_eq!(Class::Uplink.label(), "UL");
        assert_eq!(Class::Unknown.label(), "unknown");
        assert_eq!(Class::NoAccess.label(), "no-access");
        assert_eq!(Class::Lcw.label(), "LW");
        assert_eq!(Class::Messaging(Ms { blocks: 0, payload_bits: 0, corrected: 0 }).label(), "MS");
    }

    #[test]
    fn de_interleave3_keeps_every_bit() {
        let g: String = (0..96).map(|i| if i % 3 == 0 { '1' } else { '0' }).collect();
        let (a, b, c) = de_interleave3(&g);
        assert_eq!(a.len() + b.len() + c.len(), 96);
        let ones = a.matches('1').count() + b.matches('1').count() + c.matches('1').count();
        assert_eq!(ones, g.matches('1').count(), "bits were lost or invented");
    }

    #[test]
    fn de_interleave_splits_a_64_bit_group_evenly() {
        let g: String = (0..64).map(|i| if i % 2 == 0 { '1' } else { '0' }).collect();
        let (a, b) = de_interleave(&g);
        assert_eq!((a.len(), b.len()), (32, 32));
    }

    #[test]
    fn slice_extra_keeps_the_remainder() {
        let (blocks, extra) = slice_extra(&"0".repeat(150), 64);
        assert_eq!(blocks.len(), 2);
        assert_eq!(extra.len(), 22);
        let (none, all) = slice_extra("0110", 64);
        assert!(none.is_empty());
        assert_eq!(all, "0110");
    }

    #[test]
    fn signed_position_fields_sign_extend() {
        let mut bits = vec!['0'; 63];
        bits[13] = '1';
        bits[24] = '1';
        let s: String = bits.into_iter().collect();
        assert_eq!(signed12(&s, 13, 14..25), 1 - 2048);
        let z: String = vec!['0'; 63].into_iter().collect();
        assert_eq!(signed12(&z, 13, 14..25), 0);
    }

    #[test]
    fn short_bursts_are_rejected_rather_than_read_past_the_end() {
        for n in [0usize, 1, 24, 30, 63, 95] {
            let b = format!("{}{}", ACCESS_DOWNLINK, "0".repeat(n));
            let _ = classify(&b); // must not panic
        }
        assert!(decode_ira(&"0".repeat(10)).is_none());
        assert!(decode_ibc(&"0".repeat(10), Effort::Strict).is_none());
        assert!(decode_ms(&"0".repeat(10)).is_none());
    }

    #[test]
    fn labels_cover_the_decoded_variants_too() {
        let ira = Ira { sat: 1, beam: 2, pos_x: 0, pos_y: 0, pos_z: 0, interval: 0,
                        slot: 0, eip: 0, bc_sb: 0, lat_deg: 0.0, lon_deg: 0.0,
                        alt_km: 0.0, corrected: 0 };
        assert_eq!(Class::RingAlert(ira).label(), "IRA");
        let ibc = Ibc { bc_type: 0, sv_id: 1, beam_id: 2, slot: 0, sv_blocking: 0,
                        acqu_subband: 0, acqu_channels: 0, corrected: 0 };
        assert_eq!(Class::Broadcast(ibc).label(), "IBC");
    }

    #[test]
    fn a_messaging_burst_with_no_usable_blocks_still_classifies() {
        // The header identifies the class; if nothing after it survives error
        // correction the burst is still messaging, reported with zero blocks
        // rather than misfiled as unknown.
        let b = format!("{}{}", ACCESS_DOWNLINK, HEADER_MESSAGING);
        match classify(&b) {
            Class::Messaging(ms) => assert_eq!(ms.blocks, 0),
            other => panic!("expected messaging, got {:?}", other),
        }
    }

    #[test]
    fn decode_blocks_stops_at_a_short_block() {
        // a truncated final block must end decoding rather than be padded
        let d = u32::from_str_radix("101100111000110101101", 2).unwrap() << 10;
        let word = d | bch::gf2_remainder(bch::RINGALERT, d);
        let good: String = (0..31)
            .map(|i| if (word >> (30 - i)) & 1 == 1 { '1' } else { '0' })
            .collect();
        let (payload, _c, used) = decode_blocks(&[good, "0101".to_string()], bch::RINGALERT);
        assert_eq!(used, 1);
        assert_eq!(payload.len(), 21);
    }

    #[test]
    fn the_broadcast_path_rejects_bursts_it_cannot_verify() {
        // too short to hold a header plus one interleaved block
        assert!(decode_ibc(&format!("000000{}", "0".repeat(60)), Effort::Strict).is_none());
        // header that does not divide by the header generator
        assert!(decode_ibc(&format!("000001{}", "0".repeat(300)), Effort::Strict).is_none());
        // header divides, but the interleaved blocks do not: three independent
        // checks and any one failing must reject the burst
        let mut rejected = 0;
        for seed in 0u64..40 {
            let mut body = String::new();
            let mut x = seed.wrapping_mul(6364136223846793005).wrapping_add(7);
            for _ in 0..300 {
                x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                body.push(if (x >> 33) & 1 == 1 { '1' } else { '0' });
            }
            if decode_ibc(&format!("000000{}", body), Effort::Strict).is_none() {
                rejected += 1;
            }
        }
        assert_eq!(rejected, 40, "random bodies passed the broadcast checks");
    }

    #[test]
    fn decode_blocks_stops_at_an_uncorrectable_block() {
        // a good block then a hopeless one: the good payload must survive and
        // the bad block must not contribute
        let d = u32::from_str_radix("101100111000110101101", 2).unwrap() << 10;
        let word = d | bch::gf2_remainder(bch::RINGALERT, d);
        let good: String = (0..31)
            .map(|i| if (word >> (30 - i)) & 1 == 1 { '1' } else { '0' })
            .collect();
        let mut ruined = good.clone();
        // flip enough bits to exceed the code's correcting power
        unsafe {
            let b = ruined.as_bytes_mut();
            for i in 0..9 {
                b[i * 3] = if b[i * 3] == b'1' { b'0' } else { b'1' };
            }
        }
        let (payload, _c, used) = decode_blocks(&[good.clone(), ruined], bch::RINGALERT);
        assert_eq!(used, 1, "the uncorrectable block should have stopped decoding");
        assert_eq!(payload.len(), 21);
    }
}
