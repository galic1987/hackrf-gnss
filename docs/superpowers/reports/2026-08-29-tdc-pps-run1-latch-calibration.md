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

*Amended same night (23:55 EDT): sections 3–5 rewritten. The latch number
is a ratio; the tracker clock-drift breaks the tie. First reading
("timestamp domain slow") was wrong — the bias is on the PPS side. The
Pro's clock tree is coherent. Correction propagated before any downstream
use.*

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

Conclusion: the Bodnar's 1PPS edges are placed on a grid with a quantum of
~16 taps (≈400 ps at the provisional 25 ps/tap) — consistent with edges
synthesised/dithered on an internal multi-GHz VCO grid, not a coarse
u-blox-style ~8 ns sawtooth. Practical consequence: **code-density DNL
from this PPS source is impossible** — edges land on three teeth, so
intermediate bins can never accumulate counts. DNL/INL needs a swept-delay
source or the RO self-test with the CDC window fix landed.

## 3. Ticks per PPS interval — and what it is a ratio *of*

900 s of slot-1 48-bit trigger-latch deltas
(`ts_latch_run1.jsonl`, 900/900 seconds captured):

| metric | value |
|---|---|
| mean | 39,999,973.31 ticks/PPS-interval |
| trend over 900 s | +0.0003 ticks/s² — static |
| jitter | sd 0.69 ticks (≈17 ns); second-to-second sd 1.18 ticks (≈30 ns) |

The latch counts adclk ticks between consecutive PPS edges. The number is
a *ratio* of two clocks: adclk (CLKIN→Si5351→AFE 40 MHz) against the
Bodnar's PPS interval. Standing alone it cannot say which side is off.

## 4. Tie-break: the bias is on the PPS, not the Pro

Post-restart tracker clock-drift (326 s of RMS<200 m rows): **+1.08 ns/s**
— the ADC sample clock (= the same adclk domain; gateware source puts the
timestamp counter in `adclk`, and has since the counter was introduced in
9db94343) sits in the GPSDO band, GPS-true to ~1 ppb. The Mac host clock
only scales the drift by ~ppm of itself and cannot move a −667 ns/s
signal to +1.08 ns/s.

Therefore adclk is GPS-true and the ratio in §3 belongs to the PPS:

> **Bodnar output-1 "1PPS" runs +0.667 ppm fast** (interval 0.999999333 s),
> statically, with edges confined to a ~400 ps comb (§2).

Interpretation (ranked): (a) output 1 generates its "1PPS" as a
*synthesised 1 Hz* from the disciplined VCO — finite fractional-divider
resolution gives the static rate rounding, fractional-N dither gives the
VCO-grid comb; (b) a GPS-timing-engine PPS with a pathological steering
loop — disfavoured, a phase-steered loop would hold zero mean rate error.
Discriminator queued: query the device config (lbe-1420 CLI) in a window —
the USB serial port is busy with gpsdo_probe outside windows — and compare
against the DCD-pin PPS, which the vendor documents as a second 1PPS
source and which is likely the GPS engine's own.

Station consequences:

- The Pro's clock tree is **coherent**: CLKIN→Si5351→adclk→sample clock,
  one domain, GPS-true. No re-clocking needed. (Supersedes the
  cross-domain reading in the first draft of this section.)
- The wired PPS is a valid *coarse trigger*: 1 Hz nominal, one edge per
  GPS second, identity of seconds preserved. Rate bias is irrelevant for
  epoch anchoring (we count seconds, not phase) and for the TDC runs.
- The wired PPS is **not** a precision time/phase reference: +0.667 ppm
  rate bias, ~400 ps comb, ~10 s / ~7 ns phase wobble. Any jitter or
  phase claim against it must say so.

## 5. Coherence verdict

- Pro #2 sample clock: GPS-coherent, +1.08 ns/s (GPSDO band). Star feed
  through the splitter is healthy post-replug.
- Latch/TDC path: works, counts the GPS-true adclk; measured ratio is
  dominated by the PPS-side bias (§4).
- HackRF One: still dark. ATSC producer publishes fresh `lock:false`
  tombstones; pilot ~37 dB starved, 0.03 z vs 0.75 floor. Physical
  ClearStream feed check remains owner-owed. No cross-radio coherence
  pairs possible until it locks — and with it dark, the tie-break above
  rests on the Pro's tracker alone (noted for honesty: a second-radio
  CLKIN-side measurement would independently confirm the 10 MHz path).

## 6. Standing gaps (unchanged)

- Atomic coarse+fine: still no image with both. Run 1 (fine, slot 0) and
  the latch run (coarse, slot 1) measured *different* edges; statistical
  join only.
- 499 ps stays provisional; "calibrated" stays retracted. Run 1 is the
  first external evidence, not a calibration — no swept delay, and the
  comb (§2) blocks code-density from this source.
- clock_bias v3 keeps the known session-provenance gap (gen straddles the
  23:22 restart; analyzer segments on it).

## Next

1. Tap-size calibration path that bypasses the comb: swept-delay source,
   or RO self-test after the `meas_gate_ro` CDC fix lands.
2. Query the Bodnar's output-1 configuration in a window; test the DCD
   PPS as a candidate GPS-engine-grade reference (§4).
3. One RF path physical check → splitter test → cross-radio coherence
   (also gives the second CLKIN-side confirmation of the 10 MHz path).
