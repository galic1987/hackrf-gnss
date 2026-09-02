# 2026-08-29 — Run 1 / Run 3 preliminary clock-ratio and TDC observations

Window: 21:59–23:30 EDT (Run 1), 08:25–08:45 EDT next day (Run 3). Pro #2 (645061de).
Topology: Bodnar LBE-1421 GPSDO — OUT2 10 MHz split to both radios' CLKIN (matched cables), OUT1 1PPS split to both radios' P28 pin 16 TRIGGER.IN (matched cables). P28 pinout correction of this date stands: pin 16 = TRIGGER.IN on Pro and One alike.

Retained evidence: `gnss/observations/tdc_pps_run1.jsonl` (3,658 fine-code
rows), `gnss/observations/ts_latch_run1.jsonl` (the coarse ratio used for the
historical −0.667 ppm estimate), and `gnss/observations/ts_latch_run3.jsonl`
(1,200 s coarse ratio after a stream selected CLKIN). The modern analyzers are
`scripts/tdc_pps_analyze.py` and `scripts/tdc_ts_analyze.py`.

**Status amendment:** these artifacts are useful preliminary relative
observations, not a completed calibration. Run 1 did not retain image/build,
actual AFE clock, selected-reference readback, reset/stream history, GPSDO
output telemetry, or temperature. “Idle” is not a clock-source state: stream
shutdown does not reselect the internal reference. Consequently the historic
source labels and absolute scale cannot be recovered from the JSONL alone.

## 1. What the coarse latch rows establish

The slot-1 coarse trigger latch counts synthesized AFE ticks between observed
PPS edges. It therefore measures a **ratio of two clocks**.

- **Run 1:** the retained ratio is consistent with an AFE/reference relation
  about −0.667 ppm from nominal under the then-assumed clock configuration.
  The artifact does not prove that the selected source was the TCXO or that
  the nominal rate used by the old analyzer was correct.
- **Run 3:** after a 16 Msps stream selected external CLKIN, the retained mean
  was 32,000,000.000834 ticks per observed PPS. This is strong relative
  common-source coherence evidence for that 20-minute window. “+1 count in
  1,200 s” is counter quantization, not a ±0.026 ppb absolute uncertainty or a
  proof of UTC/PPS truth.

WAAS code-Doppler and the Bodnar receiver's NMEA fix are useful cross-checks,
but neither supplies a retained traceable absolute frequency/PPS uncertainty
for these files. No conclusion here assigns either clock perfect truth.

## 2. Fine-code occupancy

`tdc_pps_run1.jsonl` contains a strong occupancy pattern, including structure at
16-code boundaries. That is **consistent with** iCE40 carry/fabric-hop DNL;
the capture cannot uniquely exclude nonuniform source-phase visitation.

The former 105.5 ps estimate divided an inferred in-window fraction by 48 and
then used the result to claim that the same saturation fraction confirmed a
5.07 ns window. That is circular. In external-trigger mode:

- genuine strict-prefix interior codes are 1..47;
- code 0 is not a qualified capture;
- code 48 combines full-chain overflow with possible qualifier-deferred
  capture after a bubbled first sample.

Therefore 79.74% code-48 occupancy is not an ordinary right-censored tap bin,
and no absolute width, DNL/INL, 499 ps RMS, or fine-time LUT follows from this
run. An independently swept/randomized or phase-tagged stimulus is required.

This describes the retained pre-A2 run, not the source-tree A2 capture
contract, which is not deployed on the retained 0x469/API 0x0116 station.
A2 preserves every nonzero tap-0-anchored raw word: strict prefixes 1..47 are
`interior`, all ones is `composite-full-scale`, and anchored words with holes
are `bubbled`. A bubbled word is valid mailbox/capture evidence, but its raw
popcount is not a timing-bin code. The current density analyzer retains and
counts such words while excluding them from the occupancy/calibration input;
their presence is an explicit absolute-calibration gate failure.

## 3. What was legitimately corrected

The small coarse-latch spread is compelling functional evidence that the
trigger snapshot is hardware-latched rather than a free-running counter read.
It does not identify the selected clock source, calibrate fine bins, or prove
that every six-byte fine word was status-bracketed against a mid-read PPS.

## 4. Conditional coherence verdict

- Pro #2: Run 3 supports close relative rate coherence between the synthesized
  32 MHz clock and the Bodnar-derived PPS during that window. Absolute phase,
  UTC offset, and PPS quality were not measured.
- HackRF One: no paired RF/common-start dataset is part of this report. Shared
  10 MHz and 1PPS wiring alone does not establish receiver-delay coherence.

## 5. Next Steps

1. Keep the legacy capture procedures quarantined.
2. Build a long-lived, fresh-toggle, status-before/after reader with complete
   build/source/rate/temperature provenance.
3. Supply independent phase-uniformity evidence bound to the capture hash
   before emitting absolute widths, DNL/INL, or a LUT.
4. Then perform safe-gain shared-RF ABBA receiver-delay work on separate USB
   roots; report uncertainty rather than parabolic-fit picoseconds alone.

## 6. Midday addendum (2026-08-30) — Leg 1 hour-gate retry verdict

(Numbering restarts at 6: the da5c02d restructure dropped the f706d62 overnight addendum, which survives in git history. Verdict below supersedes it with fresh numbers.)

**Verdict: INSUFFICIENT DATA. No qualifying continuous hour has ever existed on this station.**

Collection timeline on the window-package build (nh_sync newest-window + emit gate 5→4, deployed 06:56):

- 06:56–08:20: gen `v3-1788087599` (3,498 rows). ~08:20 full-cohort wedge — tracker log and clock_bias both froze. The new supervisor (launchd `com.hormuz.tracker`, da5c02d) restarted the cohort at 09:35:09. 75-min hole.
- 09:38–12:25: gen `v3-1788096912-tb1788086830`, 4,683 rows / 2.77 h. Inside it, a **65-min clock_bias emission hole 10:56:39–12:01:32** while the tracker held 7–12 locked throughout and `clock_bias_shadow` — reading the same `state.tracker.json` — produced 1,369 rows continuously. Same pid, same gen, self-recovered: a producer-side stall, root cause open (candidate: per-sat ephemeris/freshness gate collapse below the emit floor; the `parse_rinex_gps … rejected (unit Radians)` spam in clock_bias.log is untimestamped, unproven). Second distinct stall signature of the day.

Analyzer on the current gen (12:25):

- n_sat distribution: 4 → 2,539 rows (54%), 5 → 1,901, 6 → 243. n_bds=1 on 2,309 rows.
- Quality gate (n_sat≥5, slips=0) keeps 2,144 rows → **12.9 quality rows/min vs the 56.7/min** the 3,400-row/3,600-s gate requires. Total emission duty 28.2/min — half of required even counting n_sat=4 rows.
- Segmentation (split at >1.5× median dt 1.03 s): **longest clean segment 142 s / 138 rows** vs the 3,600 s / 3,400-row gate.
- Structurally impossible today regardless: the gen began 09:38, so a qualifying hour could not exist before ~10:38 even under perfect continuity; the 65-min hole removed it.

Comparison vs overnight (f706d62: 12,838 rows/6.9 h, best segment 312 s):

- The window package did **not** move the hour gate (142 s vs 312 s; both >20× below the gate — different sky, not evidence of regression).
- **New structural finding — emit/analyzer mismatch:** the emit gate was lowered to 4 but the analyzer quality floor stayed at 5, so 54% of emitted rows are discarded before TDEV. The change raised emission volume, not analyzable density. Either the analyzer floor moves to 4 with a documented ISB caveat, or the emit gate returns to 5; as-is they work against each other.
- Binding constraints, ranked: (1) producer emission duty (~half of required); (2) single-epoch 2-s holes shattering segments under the 1.5× median-dt split; (3) producer-side stalls (two signatures in one day — full-cohort freeze at 08:20, silent consumer hole 10:57–12:01).
