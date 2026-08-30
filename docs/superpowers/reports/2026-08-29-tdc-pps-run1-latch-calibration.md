# 2026-08-29 — TDC PPS Run 1 + Ticks-per-GPS-Second Latch Calibration

Window: 21:59–23:30 EDT, Pro #2 (645061de) only. Tracker deliberately down
21:59–23:22 for the register port; restored and re-acquired after the runs
(12 ch / 8 locked at +86 s, clock_bias rows flowing, RMS p50 32 m).
Topology: Bodnar LBE-1421 GPSDO — OUT2 10 MHz split to both radios' CLKIN
(matched cables), OUT1 1PPS split to both radios' P28 pin 16 TRIGGER.IN
(matched cables). P28 pinout correction of this date stands: pin 16 =
TRIGGER.IN on Pro and One alike.

Evidence: `gnss/observations/tdc_pps_run1.jsonl` (3,658 events),
`gnss/observations/ts_latch_run1.jsonl` (900 s), analyzers
`scripts/tdc_pps_analyze.py` (extended, fc08d0d) and
`scripts/tdc_ts_latch_run.sh`.

## 1. Run 1 — first external single-edge TDC dataset

3,658 external 1PPS edges captured into the slot-0 48-tap carry-chain TDC
over 3,657 s: rate exactly 1.0000 s, one missed poll, **100%
thermometer-valid** (zero races/bubbles — the external-arm path is clean;
contrast with the RO self-test runs, which show bubbles from the known
un-synchronised window gate, Round-14 `meas_gate_ro` finding).

This is the station's first TDC dataset of *external* single edges. All
prior numbers were ring-oscillator self-tests.

## 2. The Bodnar 1PPS is steered on a ~16-tap comb (sawtooth flag resolved)

- The PPS-vs-adclk phase cycles through the ~1.3 ns measurement window
  every **10.0 s median** (367 in-window bursts, median 2 s, dwells 8 s) —
  the phase is *not* fixed; the "coherence trap" does not bind.
- But second-to-second popcount steps cluster at multiples of 16 taps
  (−16 ×29, +16 ×23, +32 ×16 = 68/307 nonzero steps), and in-window counts
  pile on teeth k = 1, 17, 33 (79/165/191 of 741 in-window samples).
- Control: the incommensurate RO self-test histogram (run2_clean, 5,000
  samples) has **no** teeth at 1/17/33 → the comb is a property of the
  PPS source, not of the chain (not DNL).

Conclusion: the Bodnar's 1PPS edges are placed/steered at a quantum of
~16 taps. Absolute size awaits tap calibration (≈400 ps at the provisional
25 ps/tap). This resolves the reviewer's sawtooth flag: present, but a
*fine* quantum — not a coarse u-blox-style ~8 ns sawtooth. Practical
consequence: **code-density DNL from this PPS source is impossible** —
edges land on three teeth, so intermediate bins can never accumulate
counts. DNL/INL needs a swept-delay source or the RO self-test with the
CDC window fix landed.

## 3. Ticks per GPS second — the sample-epoch measurement

900 s of slot-1 48-bit trigger-latch deltas
(`ts_latch_run1.jsonl`, 900/900 seconds captured):

| metric | value |
|---|---|
| mean | 39,999,973.31 ticks/GPS-second |
| offset vs 40.000 MHz claimed | **−0.667 ppm** |
| trend over 900 s | +0.0003 ticks/s² — static, not convergence slew |
| jitter | sd 0.69 ticks (≈17 ns); second-to-second sd 1.18 ticks (≈30 ns) |

So the nominal 25.00 ns tick counts 40 MHz but the domain runs 0.667 ppm
slow against GPS, and it is *statically* slow — a synthesis-ratio or
local-oscillator offset, not GPSDO discipline transient.

## 4. Cross-domain finding: the latch clock is not the sample clock

Post-restart tracker clock-drift (326 s, RMS<200 m rows): **+1.08 ns/s**
— the ADC/sample clock sits in the GPSDO band (pre-window era: +0.885
ns/s; the TCXO band is +530 ns/s, three orders away).

The latch domain runs at −667 ns/s against the same GPS. Disagreement
≈ 668 ns/s (27 ticks/s; 2.4 ms/hour): **the slot-1 trigger timestamp
counter and the ADC sample clock are different clock domains.** Using
latch ticks directly as sample indices would inject 668 ns/s of epoch
error. Before epoch anchoring: either re-clock the timestamp counter from
adclk, or carry the measured rate correction (39,999,973.31 ticks/s) and
accept that the two domains can drift independently. Mechanism (Si5351
rounding vs fabric on local osc) is distinguishable by long-baseline
temperature correlation — queued.

## 5. Coherence verdict

- Pro #2 sample clock: GPS-coherent, +1.08 ns/s (GPSDO band). Star feed
  through the splitter is healthy post-replug.
- Latch/TDC timestamp domain: −0.667 ppm vs GPS — **not** coherent with
  the sample clock (finding 4).
- HackRF One: still dark. ATSC producer publishes fresh `lock:false`
  tombstones; pilot ~37 dB starved, 0.03 z vs 0.75 floor. Physical
  ClearStream feed check remains owner-owed. No cross-radio coherence
  pairs possible until it locks.

## 6. Standing gaps (unchanged)

- Atomic coarse+fine: still no image with both. Run 1 (fine, slot 0) and
  the latch run (coarse, slot 1) measured *different* edges; statistical
  join only.
- 499 ps stays provisional; "calibrated" stays retracted. Run 1 is the
  first external evidence, not a calibration — no swept delay, and the
  comb (finding 2) blocks code-density from this source.
- clock_bias v3 keeps the known session-provenance gap (gen straddles the
  23:22 restart; analyzer segments on it).

## Next

1. Tap-size calibration path that bypasses the comb: swept-delay source,
   or RO self-test after the `meas_gate_ro` CDC fix lands.
2. Long-baseline latch series to separate Si5351-rounding (constant) from
   local-oscillator (temperature-wandering) for the −0.667 ppm.
3. Re-clock or rate-correct the timestamp domain before any sample-zero
   anchoring (finding 4).
4. One RF path physical check → splitter test → cross-radio coherence.
