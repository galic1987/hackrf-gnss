//! Timestamp nibble extractor for the 2_extprec_rx (extended-precision) stream.
//!
//! Gateware side: `TimestampNibbler` in firmware/fpga/dsp/timestamp.py.
//!
//! Wire format: the 12-bit I/Q samples are packed right-justified in
//! little-endian int16 lanes; the top nibble (bits 15:12) of EACH lane
//! carries a rotating 4-bit window over the FPGA timestamp counter instead
//! of sign extension. One nibble per sample pair, identical in I and Q:
//!
//!   - 0xF, 0xE     resync marker pair; also the head of every capture
//!   - 11 data nibbles, counter bits [0:44] LSB-nibble first
//!
//! The rotation period is 13 samples. Markers are NOT unique — a data nibble
//! can be 0xF or 0xE — so the F,E pair is only a rotation-index anchor: after
//! it, exactly 11 data nibbles are consumed. The shipped image has a 40-bit
//! counter, so the 11th data nibble always reads 0; 2^40 ticks roll over
//! every ~7.6 h.
//!
//! THE FAST-COUNTER SUBTLETY: the nibbler emits nibbles of the LIVE counter,
//! which ticks `tps` times per sample (tps = AFE clock / sample rate: 4 for
//! 32 MHz / 8 Msps — the boosted ext image and std alike — and 16 for legacy
//! 40 MHz / 2.5 Msps ext captures). Data nibble j, emitted
//! at rotation position p = 2+j, therefore carries bits [4j:4j+4] of
//! C0 + tps*(2+j), where C0 is the counter at the F marker. The naive
//! nibble-wise reconstruction R mixes 11 DIFFERENT counter values, and the
//! additive offsets carry across nibble boundaries — observed on hardware as
//! rotation-to-rotation deltas of 208+-256 instead of exactly 208.
//!
//! The extractor solves for C0 constructively instead of searching: nibble j
//! of the sum C + tps*(2+j) depends only on the low 4j+4 bits of C (addition
//! carries propagate upward only), so C's nibbles are recovered LSB-first —
//! with the low 4j bits known, the observed n_j fixes nibble j exactly:
//! c_j = (n_j - ((lo + tps*(2+j)) >> 4j)) mod 16. (A window search around
//! R - 7*tps was considered and rejected: the carry spread of R-C reaches
//! +-256 ticks, far past any small window; the constructive solve IS the
//! unique exact-match candidate such a search would find.) The 44 observed
//! bits over-determine the 40-bit counter: the solved top nibble must be 0,
//! which is the consistency check that flags nibble damage — a damaged
//! rotation still solves to some C, so the chain check below is the arbiter.
//!
//! EMISSION GUARANTEE for consumers (the discipline loop relies on this):
//! every emitted (sample_idx, counter) pair is sequence-validated — sample
//! indices are strictly increasing multiples-of-13 apart, and the counter
//! advance per 13-sample step is 13*tps plus a bounded dither of at most +-1
//! tick per rotation. The dither is real hardware behaviour, not noise: the
//! sample enable is dithered +-1 tick (measured on a live capture: per-rotation
//! advances split 207/208/209 with mean EXACTLY 16.000000 ticks/sample), so
//! the mean rate is exact and the accumulated phase error stays within a tick.
//! Anchors that violate the chain (alias markers, uncorrectable nibble
//! damage) are dropped, never fabricated. A genuine counter discontinuity
//! mid-stream (dropped samples) ends the chain rather than being smoothed
//! over.

pub const MARKER: [u8; 2] = [0xF, 0xE];
pub const DATA_NIBBLES: usize = 11; // counter bits [0:44]
pub const PERIOD: usize = 2 + DATA_NIBBLES; // 13 samples per rotation

/// Timestamp nibble carried by one int16 lane.
pub fn nibble(v: i16) -> u8 {
    ((v >> 12) & 0xF) as u8
}

/// One completed rotation: sample index of the F marker + the 11 nibbles.
struct Rotation {
    anchor: usize,
    nib: [u8; DATA_NIBBLES],
}

/// Phase 1: split the stream into rotations. Resyncs on the next F,E pair
/// when a rotation is interrupted (capture restart, dropped samples); the I
/// lane is the source of truth (Q carries the same nibble).
fn scan_rotations(raw: &[i16]) -> Vec<Rotation> {
    let mut out = Vec::new();
    let mut nib = [0u8; DATA_NIBBLES];
    let mut have = 0usize;
    let mut anchor = 0usize;
    let mut prev_f = false;
    let mut synced = false;
    for (s, pair) in raw.chunks_exact(2).enumerate() {
        let n = nibble(pair[0]);
        if prev_f && n == MARKER[1] {
            synced = true;
            have = 0;
            anchor = s - 1;
        } else if synced && have < DATA_NIBBLES {
            nib[have] = n;
            have += 1;
            if have == DATA_NIBBLES {
                out.push(Rotation { anchor, nib });
                synced = false;
            }
        }
        prev_f = n == MARKER[0];
    }
    out
}

/// Phase 2: solve one rotation for C0, the counter at its F marker.
/// Constructive LSB-first solve — see the module docstring. Returns
/// (counter, exact): exact=false means the solved counter overran the 40-bit
/// field (the solved top nibble is nonzero), i.e. the rotation's nibbles are
/// damaged; the value may still be repaired by the phase-3 chain.
fn solve_counter(nib: &[u8; DATA_NIBBLES], tps: u64) -> (u64, bool) {
    let mut c = 0u64;
    for (j, &n) in nib.iter().enumerate() {
        let off = tps * (2 + j as u64);
        let cj = (n as u64).wrapping_sub((c + off) >> (4 * j)) & 0xF;
        c |= cj << (4 * j);
    }
    let exact = c >> 40 == 0;
    (c, exact)
}

/// Phase 3: emit only sequence-validated anchors (see the module docstring
/// for the guarantee). The chain seeds from an exact solve and accepts the
/// grid value plus a +-1-tick-per-rotation sample-enable dither; a damaged
/// rotation is repaired by the chain when its solved value lands on the grid.
fn validate(solved: Vec<(usize, u64, bool)>, tps: u64) -> Vec<(usize, u64)> {
    let mut out: Vec<(usize, u64)> = Vec::new();
    for (anchor, c, exact) in solved {
        match out.last() {
            None => {
                if exact {
                    out.push((anchor, c));
                }
            }
            Some(&(li, lc)) => {
                let di = anchor - li;
                if di == 0 || di % PERIOD != 0 {
                    continue; // alias anchor
                }
                let k = (di / PERIOD) as u64;
                let expected = lc + (PERIOD as u64 * tps) * k;
                if c.abs_diff(expected) <= k {
                    out.push((anchor, c));
                }
                // otherwise dropped: never fabricate a counter value
            }
        }
    }
    out
}

/// Reconstruct counter values from an interleaved i16 I/Q stream with a known
/// ticks-per-sample rate. See the module docstring for the emission guarantee.
pub fn extract_with_tps(raw: &[i16], tps: u64) -> Vec<(usize, u64)> {
    assert!(tps > 0, "ticks-per-sample must be positive");
    let solved = scan_rotations(raw)
        .iter()
        .map(|r| {
            let (c, exact) = solve_counter(&r.nib, tps);
            (r.anchor, c, exact)
        })
        .collect();
    validate(solved, tps)
}

/// Estimate ticks-per-sample from the stream itself: median of the naive
/// reconstruction deltas between consecutive rotations, divided by the 13
/// sample period. Carry corruption moves individual deltas by +-256 ticks;
/// the median stands as long as most inter-rotation gaps are carry-free.
pub fn estimate_ticks_per_sample(raw: &[i16]) -> Option<u64> {
    let rots = scan_rotations(raw);
    let mut deltas: Vec<u64> = Vec::new();
    for w in rots.windows(2) {
        if w[1].anchor - w[0].anchor != PERIOD {
            continue;
        }
        let (mut a, mut b) = (0u64, 0u64);
        for (j, &n) in w[0].nib.iter().enumerate() {
            a |= (n as u64) << (4 * j);
        }
        for (j, &n) in w[1].nib.iter().enumerate() {
            b |= (n as u64) << (4 * j);
        }
        if b > a {
            deltas.push(b - a);
        }
    }
    if deltas.len() < 2 {
        return None;
    }
    deltas.sort_unstable();
    let med = deltas[deltas.len() / 2];
    let tps = (med + PERIOD as u64 / 2) / PERIOD as u64;
    (tps > 0).then_some(tps)
}

/// Reconstruct counter values, estimating ticks-per-sample from the stream
/// (falls back to 1 when estimation is impossible). Prefer
/// `extract_with_tps` when the radio config is known: tps = afe/fs.
pub fn extract(raw: &[i16]) -> Vec<(usize, u64)> {
    let tps = estimate_ticks_per_sample(raw).unwrap_or(1);
    extract_with_tps(raw, tps)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Embed a live counter into an interleaved i16 stream, mirroring
    /// TimestampNibbler: the counter ticks `tps` times per sample pair, and
    /// the data nibble at rotation position k reads the counter value AT THAT
    /// SAMPLE. `payload` is the 12-bit sample value (sign-extended to i16
    /// range -2048..2047).
    fn embed<F: Fn(usize) -> u64, P: Fn(usize) -> i16>(n: usize, counter: F, payload: P) -> Vec<i16> {
        let mut raw = Vec::with_capacity(2 * n);
        for s in 0..n {
            let idx = s % PERIOD;
            let nib = match idx {
                0 => MARKER[0],
                1 => MARKER[1],
                k => ((counter(s) >> (4 * (k - 2))) & 0xF) as u8,
            };
            let lane = (((nib as u16) << 12) | (payload(s) as u16 & 0xFFF)) as i16;
            raw.push(lane); // I
            raw.push(lane); // Q carries the same nibble
        }
        raw
    }

    #[test]
    fn roundtrip_synthetic_stream() {
        // one tick per sample: the solver must still land exactly
        let c0 = 0x0123_4567_89u64;
        let counter = |s: usize| c0 + s as u64;
        let payload = |s: usize| ((s * 37) % 4000) as i16 - 2000;
        let raw = embed(4 * PERIOD, counter, payload);
        let ts = extract_with_tps(&raw, 1);
        assert_eq!(ts.len(), 4);
        for (r, &(idx, val)) in ts.iter().enumerate() {
            assert_eq!(idx, r * PERIOD);
            assert_eq!(val, counter(idx), "counter at the marker sample");
        }
    }

    #[test]
    fn fast_counter_roundtrip_t16() {
        // THE hardware regression test: counter ticks 16x per sample, so the
        // 11 data nibbles of one rotation read 11 different counter values.
        let c0 = 0x0000_0001_0000u64;
        let tps = 16u64;
        let counter = |s: usize| c0 + tps * s as u64;
        let payload = |s: usize| ((s * 53) % 4000) as i16 - 2000;
        let raw = embed(6 * PERIOD, counter, payload);
        assert_eq!(estimate_ticks_per_sample(&raw), Some(tps));
        let ts = extract_with_tps(&raw, tps);
        assert_eq!(ts.len(), 6);
        for (r, &(idx, val)) in ts.iter().enumerate() {
            assert_eq!(idx, r * PERIOD);
            assert_eq!(val, counter(idx));
        }
        for w in ts.windows(2) {
            assert_eq!(w[1].0 - w[0].0, PERIOD);
            assert_eq!(w[1].1 - w[0].1, PERIOD as u64 * tps, "208-tick grid");
        }
    }

    #[test]
    fn fast_counter_roundtrip_t4() {
        // The boosted ext image: 32 MHz AFE at 8 Msps = 4 ticks/sample,
        // the same relation std mode has — the solver's easiest case.
        let c0 = 0x0000_0001_0000u64;
        let tps = 4u64;
        let counter = |s: usize| c0 + tps * s as u64;
        let payload = |s: usize| ((s * 53) % 4000) as i16 - 2000;
        let raw = embed(6 * PERIOD, counter, payload);
        assert_eq!(estimate_ticks_per_sample(&raw), Some(tps));
        let ts = extract_with_tps(&raw, tps);
        assert_eq!(ts.len(), 6);
        for (r, &(idx, val)) in ts.iter().enumerate() {
            assert_eq!(idx, r * PERIOD);
            assert_eq!(val, counter(idx));
        }
        for w in ts.windows(2) {
            assert_eq!(w[1].0 - w[0].0, PERIOD);
            assert_eq!(w[1].1 - w[0].1, PERIOD as u64 * tps, "52-tick grid");
        }
    }

    #[test]
    fn fast_counter_straddling_carries() {
        // Start just below a 4096 boundary so every rotation's data nibbles
        // straddle carries into bits 8+ and the +/-256 corruption region.
        let tps = 16u64;
        let c0 = 4096u64 - PERIOD as u64 * tps / 2; // first rotation crosses 0x1000
        let counter = |s: usize| c0 + tps * s as u64;
        let payload = |_: usize| -3i16;
        let raw = embed(8 * PERIOD, counter, payload);
        let ts = extract_with_tps(&raw, tps);
        assert_eq!(ts.len(), 8);
        for (r, &(idx, val)) in ts.iter().enumerate() {
            assert_eq!(val, counter(idx), "rotation {r} across the carry");
        }
    }

    #[test]
    fn marker_loss_resyncs_and_drops_the_gap() {
        let c0 = 0x0000_00AB_CDu64;
        let counter = |s: usize| c0 + s as u64;
        let payload = |_: usize| 7i16;
        let mut raw = embed(5 * PERIOD, counter, payload);
        // Corrupt the F,E marker of rotation 2 (data can alias F/E — a
        // dropped/corrupted marker must never wedge the parser).
        raw[2 * PERIOD * 2] &= 0x0FFF;
        raw[2 * PERIOD * 2 + 2] &= 0x0FFF;
        let ts = extract_with_tps(&raw, 1);
        let idxs: Vec<usize> = ts.iter().map(|t| t.0).collect();
        assert_eq!(idxs, vec![0, PERIOD, 3 * PERIOD, 4 * PERIOD]);
        assert_eq!(ts[2].1, counter(3 * PERIOD));
        // and the emitted chain is gap-consistent across the lost rotation
        assert_eq!(ts[3].1 - ts[2].1, PERIOD as u64);
        assert_eq!(ts[2].1 - ts[1].1, 2 * PERIOD as u64);
    }

    #[test]
    fn an_alias_anchor_is_rejected() {
        // a data nibble pair aliasing F,E mid-rotation costs the interrupted
        // rotation but must not produce a bogus anchor on the emitted stream
        let tps = 16u64;
        let c0 = 1_000_000u64;
        let counter = |s: usize| c0 + tps * s as u64;
        let payload = |_: usize| 0i16;
        let mut raw = embed(4 * PERIOD, counter, payload);
        // force an F,E pair at data positions 5,6 of rotation 1
        let s = PERIOD + 5;
        raw[2 * s] = (raw[2 * s] & 0x0FFF) | ((MARKER[0] as i16) << 12);
        raw[2 * (s + 1)] = (raw[2 * (s + 1)] & 0x0FFF) | ((MARKER[1] as i16) << 12);
        let ts = extract_with_tps(&raw, tps);
        // rotation 1 is lost to the alias; nothing anchored at sample 18
        let idxs: Vec<usize> = ts.iter().map(|t| t.0).collect();
        assert_eq!(idxs, vec![0, 2 * PERIOD, 3 * PERIOD], "alias anchor leaked");
        for &(idx, val) in &ts {
            assert_eq!(val, counter(idx));
        }
        assert_eq!(ts[1].1 - ts[0].1, 2 * PERIOD as u64 * tps, "gap-consistent");
    }

    #[test]
    fn capture_restart_reanchors() {
        let counter = |s: usize| 100u64 + s as u64;
        let payload = |_: usize| 0i16;
        // Two captures back to back: 10 samples, then a fresh rotation.
        let first = embed(10, counter, payload);
        let second = embed(2 * PERIOD, |s| counter(10 + s), payload);
        let raw = [first, second].concat();
        let ts = extract_with_tps(&raw, 1);
        // First rotation never completes (interrupted at sample 10); the
        // restart's F,E re-anchors and both its rotations reconstruct.
        assert_eq!(ts.len(), 2);
        assert_eq!(ts[0].0, 10);
        assert_eq!(ts[1].0, 10 + PERIOD);
        assert_eq!(ts[0].1, counter(10));
    }

    #[test]
    fn estimator_reads_tps_from_the_stream() {
        for tps in [1u64, 4, 16] {
            let counter = |s: usize| 777u64 + tps * s as u64;
            let payload = |_: usize| 0i16;
            let raw = embed(20 * PERIOD, counter, payload);
            assert_eq!(estimate_ticks_per_sample(&raw), Some(tps), "tps {tps}");
        }
    }

    #[test]
    fn sample_enable_dither_is_absorbed() {
        // hardware reality: the sample enable dithers +-1 tick, so rotations
        // advance 207/208/209 ticks with an exact mean of 208
        let tps = 16u64;
        let c0 = 500_000u64;
        // phase toggles 0/1 per rotation -> advances alternate 208/209/207
        let counter = |s: usize| c0 + tps * s as u64 + (s / PERIOD) as u64 % 2;
        let payload = |_: usize| 0i16;
        let raw = embed(9 * PERIOD, counter, payload);
        let ts = extract_with_tps(&raw, tps);
        assert_eq!(ts.len(), 9);
        for (r, &(idx, val)) in ts.iter().enumerate() {
            assert_eq!(idx, r * PERIOD);
            assert_eq!(val, counter(idx));
        }
    }

    #[test]
    fn real_capture_validates_on_the_208_tick_grid() {
        // First ~200k samples of a live 2.5 Msps ext-image capture. Derived
        // expectations live (the 50 MB file is not a fixture); skipped when
        // absent. Hardware truth on this capture: per-rotation advances split
        // 207/208/209 (sample-enable dither) with mean exactly 16 ticks/sample.
        let path = std::path::Path::new("/tmp/ts_gate3.rawiq");
        if !path.exists() {
            eprintln!("skipped: /tmp/ts_gate3.rawiq not present");
            return;
        }
        let bytes = std::fs::read(path).unwrap();
        let raw: Vec<i16> = bytes[..800_000]
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect();
        assert_eq!(estimate_ticks_per_sample(&raw), Some(16));
        let ts = extract_with_tps(&raw, 16);
        let rotations = 200_000 / PERIOD;
        assert!(
            ts.len() as f64 >= 0.95 * rotations as f64,
            "{} of ~{rotations} rotations validated",
            ts.len()
        );
        for w in ts.windows(2) {
            let di = w[1].0 - w[0].0;
            assert_eq!(di % PERIOD, 0);
            let k = (di / PERIOD) as u64;
            assert!(w[1].1.abs_diff(w[0].1 + 208 * k) <= k, "dither bound exceeded");
        }
        // and the mean rate is exact over the capture
        let (i0, c0) = ts[0];
        let (i1, c1) = *ts.last().unwrap();
        let mean = (c1 - c0) as f64 / (i1 - i0) as f64;
        assert!((mean - 16.0).abs() < 0.01, "mean ticks/sample {mean}");
    }

    #[test]
    fn sign_reconstruction_of_negative_samples() {
        use crate::iridium::demod3::IqSample;
        // Every 12-bit payload incl. extremes survives a timestamp nibble
        // in the top 4 bits: mask + re-extend from bit 11.
        for &payload in &[-2048i16, -2047, -1, 0, 1, 2047] {
            for nib in 0..16u16 {
                let lane = ((nib << 12) | (payload as u16 & 0xFFF)) as i16;
                assert_eq!(lane.to_f32(), payload as f32, "payload {payload} nib {nib}");
            }
        }
    }
}
