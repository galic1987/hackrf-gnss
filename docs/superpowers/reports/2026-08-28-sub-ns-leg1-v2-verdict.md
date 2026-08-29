# Leg 1 v2 — first full-day verdict (2026-08-28)

**Pipeline verdict: `INSUFFICIENT DATA` — correct, honest, and informative.**
The v2 pipeline worked end-to-end on live data for a full day; the sky and the
observable — not the code — set the outcome. The sub-ns claim gates were NOT
met and cannot be met with the current `carrier_cycles` observable; the
evidence and the path forward are below.

## Analyzer output (19:35 EDT, 7,810 rows, gen v2-1787924784)

```
quality gates: 6,332 rows (n_sat>=5, slips==0);
excluded 4,772 poison-class rows (residual_rms_m >= 100 m)
segments: 108 (split at inter-row gaps > 5x median dt 1.04 s)
longest segment: seg 26 — span 161 s, 140 rows, max gap 5.2 s
continuity gates: span>=3600 s AND rows>=3400 AND max-gap<=5 s
gate failure: span 161 s < 3600 s; rows 140 < 3400; max gap 5.2 s > 5 s
INSUFFICIENT DATA (exit 1)
```

The day never produced a continuous clean hour: a GPS-poor day-side sky
(mostly 1–4 GPS above 10° vs the n≥5 gate) plus channel churn fragmented the
series. Segmentation did exactly what it was built to do — refuse to average
holes into a claim.

## What the v2 pipeline proved today

- **Carrier-sign fix (the morning's root cause)**: shadow validator
  (`scripts/clock_bias_shadow.py`, 302 fresh-update samples, 5 PRNs) —
  median |prediction error| 22–48 m at ~6 s staircase age (PRN 16: 23.2,
  PRN 26: 22.4, PRN 31: 48.3). The wrong-sign v1 rule lands km-class at the
  same ages (measured 25–520 km in the poisoned runs).
- **Healthy row mechanics**: m-class residuals (100–180 m typical, zero
  km-class), staircase pattern visible (n_pred > n_fresh), slips only on
  churn, fail-closed silence when the sat-set is sick (60% of rows excluded
  poison-class on churned sets — the gate working, not failing).
- **The observable's ceiling**: code–carrier rate mismatch ~4–8 m/s and
  WANDERING (replica-NCO frequency steering, decorrelates in seconds).
  Constant-rate de-bias replay does not recover it (debiased ≥ raw on every
  PRN). Only a true prompt carrier-phase residual (spec component 1 —
  exact sample epoch, generation counters) beats it.

## The day's real measurements

- **Receiver-clock drift vs GPS**: least-squares full-span slope
  −0.038 ns/s over 5.20 h (OLS σ_slope ≈ 0.0014 ns/s).
  **Status: RETRACTED (round 18, re-measured 2026-08-29).** Independent
  hourly slopes scatter 0.075 ns/s (1.4826×MAD over hours with >1800
  rows) — 54× the claimed uncertainty; alternative windowings put the
  scatter up to ~0.6 ns/s (~400×). The OLS fit sigma on this fragmented,
  sat-set-offset series was never meaningful, and the number is not
  reproducible in any sub-window. The hourly slopes do share a sign
  (−0.027…−0.242 ns/s) — suggestive of a real negative drift, but this
  data cannot say how large. No drift value from Leg 1 v2 enters any
  downstream claim.
- **GEO cross-check** (phase_drift_producer): sbas131 −0.32 ns/s,
  sbas135 −0.98 ns/s — same sign, ~3× apart from each other (the known
  GEO-motion residual); the clock-bias path's −0.04 ns/s disagrees with both
  beyond their fit sigmas — discrepancy **UNRESOLVED**: consistent with the
  GEO path's unmodeled motion, not proven (see the P0b gate split below).
- **Within-segment noise**: best 4.5 ns, median 25 ns (vs 273 ns global —
  the global figure is segment-to-segment sat-set bias jumps, the accepted
  cost of the raw-solve/no-median design). Within-seg noise ≈ code noise/√n
  (4–7 ns theory at n=5) — code-limited.
- **Claim-gate read**: RMS < 1 ns unreachable (25 ns typical within-seg);
  TDEV(1000) ≈ 0.8 ns plausible on white within-seg noise — untested today
  (no segment long enough). Tonight's richer sky (6–8 GPS expected
  18:00→05:00) may deliver the first evaluable continuous hour.

## Status of the claim

**Sub-ns claim: NOT SUPPORTED by today's data** — the pipeline is
mechanically complete (sign fix, solver, gates, honest segmentation) but
NOT claim-grade end-to-end: generation provenance spans tracker restarts,
and until 2026-08-29 the GEO uncertainty path was miswired (rows emitted
the raw OLS sigma instead of the calibrated max(OLS, cross-window
scatter); fixed that morning with tests, provisional 5×-OLS inflation
before 5 disjoint windows). The observable remains the limit. The path to
the gates, in order:
1. Spec component 1 (prompt carrier-phase residual with sample-exact epochs)
   — the 100×-class observable improvement.
2. Tonight's continuous-hour attempt (n=6–8 also buys √1.6 code averaging).
3. Segment-level sat-set bias characterization (the 273 ns global term).

## Method appendix (what ran)

Producer `examples/clock_bias.rs` v2 (1c3e1e3): negated-carrier Hatch with
skip-and-predict staircase semantics, anchor-fixed clock-only solve with
studentized rejection (aeae8ec pattern), gen-stamped rows. Analyzer
`scripts/clock_bias_analyzer.py` v2 (b32ab97 + 58e143c): NIST SP 1065
MDEV-TDEV, gap segmentation, continuity gates, gen grouping, missing-τ
vacuous-pass guard. Deployed 09:45 after cargo test 322/322 and
manifest_check PASS ×4 on Pro#2 (0x469 all slots). Shadow validator log:
/tmp/clock_bias_shadow.jsonl (302 rows).

---

## Overnight attempt (2026-08-29, three runs, final 03:40 EDT)

**The pipeline continues to fail closed on fragmented, code-limited data. No
qualifying hour exists, so the RMS and TDEV claim gates have not been
evaluated.**

Three analyzer runs over the historically GPS-rich 23:00–05:00 window
(analyzer v2.1, e7f0e22: NIST MDEV-TDEV, quality/poison gates, gen grouping,
segmentation now splitting on EVERY missed epoch — inter-row gap > 1.5× the
1.04 s median — so within-segment cadence is uniform to sub-epoch jitter):

```
attempt #1 00:40 — 12,955 rows; kept 2,318; longest segment 52 s / 50 rows
attempt #2 02:12 — 13,828 rows; kept 2,589; longest segment 51 s / 50 rows
attempt #3 03:40 — 16,726 rows; 14,609 quality-pass (n_sat>=5, slips==0);
  11,517 poison-class excluded (79%); kept 3,092;
  461 holes; longest segment 51 s / 50 rows (unchanged)
continuity gates: span>=3600 s AND rows>=3400 AND max-gap<=5 s
INSUFFICIENT DATA (exit 1) on all three attempts
```

**Hole anatomy.** Two compounding layers: (a) multi-thousand-second absences
(8,550 s during the pre-restart churn era, plus 6,278 s, 5,447 s, 3,263 s,
2,251 s holes) where no clean solve exists at all; (b) a constant fabric of
2–25 s solve misses — the producer emits only on cleanly-converged seconds,
and on the night's sat-sets that is roughly one second in five at best.

**Density argument — CORRECTED 2026-08-29 (round 18): the original text
blamed "observable density on the current sat-set/anchor geometry" and
prescribed a richer/tilted antenna. Re-derived from all 20.8k v2 rows, that
diagnosis is REFUTED by the data itself.** The best raw hour carried 3,214
quality-passing rows (5–7 sats, zero slips) — 53.6 rows/min against the
56.7/min the gate needs; the sky delivered a near-complete clean hour. What
removed it was the flat 100 m poison gate: healthy hours have a residual
MEDIAN of 137 m (p25 = 107 m) — the threshold sat below the 25th percentile
of good data and cut that hour to 585 rows (18.2%). The binding constraint
was **solve residual quality**, not sky: the clock_bias solve applies no
ionosphere, no troposphere and no SBAS corrections, and its Hatch filter
runs a 100 s window against a 4–8 m/s code-carrier rate mismatch. Tilting
or replacing the antenna would not have moved it. Gate re-derived from the
data's own distribution (quality residuals p50 148 / p95 300 / p99 410 m;
0.02% beyond 500 m; none beyond 1000 m): the poison threshold is now 500 m.
With it, the longest clean segment improves 52 s → 443 s and the excluded
count collapses ~9,000 → 4 rows. The NEW binding constraint is the
continuity gate against the producer's own emission duty cycle: the longest
zero-missed-epoch run anywhere in the data is ~760 s — 21% of the 3,600 s
gate — so the path forward is (a) diagnosing why ~2% of 1 Hz epochs never
emit, and/or (b) explicit uniform-grid resampling with a documented
interpolation rule, per the standing analyzer recommendation. The
antenna/tilt prescription for Leg 1 is **withdrawn**.

**GEO cross-check (03:40, state.phase_drift.json, scatter-calibrated
sigmas):** sbas131 +0.00061 ppm (scatter σ 1.9e-4), sbas135 −0.00151 ppm
(scatter σ 3.5e-4) — the two GEO phase slopes disagree with each other and
with the clock-bias series' −0.00004 ppm/day-class slope by orders of
magnitude beyond their sigmas; the discrepancy stays **UNRESOLVED** —
consistent with the unremoved GEO-motion residual but not proven. P0b is
three gates, and only the first exists: gate 1, MT9 publication
(`sbas_geonav` in tracker reports), deploys **publisher-only** in the
window bundle — no consumer reads it yet; gate 2, a signed LOS-rate +
GEO-clock-drift correction consumer (handling MT9 age, IODN/URA,
propagation epoch, fail-closed), is not yet written; gate 3, live PRN
131/135 convergence, is the acceptance test. No assignment of the
discrepancy to orbital motion until gate 3 passes. The new disjoint-window
scatter sigma confirmed live
that per-window OLS sigmas were 6–20× too tight (sbas131: 8.9e-6 vs 1.8e-4
ppm), and the consensus sigma now reports the honest 2.1e-4 class.

**Standing:** the v2 pipeline is mechanically complete (sign fix, solver,
gates, honest segmentation) but NOT claim-grade end-to-end (generation
provenance spans tracker restarts; GEO uncertainty was miswired until the
2026-08-29 fix); the sub-ns claim remains NOT SUPPORTED — no
qualifying hour has ever existed in this data. The corrected path
(round 18): (1) the measurement model — ionosphere/troposphere/SBAS
corrections in the clock_bias solve and the Hatch window/mismatch review —
is what the residuals actually need; (2) diagnose the ~2% epoch-miss rate
that caps clean segments at ~760 s, or move the analyzer to explicit
uniform-grid resampling; (3) spec component 1 (prompt carrier-phase
residual with sample-exact epochs) for the 100×-class observable. Antenna
work is OFF the Leg 1 critical path.
