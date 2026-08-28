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

- **Receiver-clock drift vs GPS (the instrument's first number)**:
  least-squares slope **−0.038 ns/s over 5.20 h** (σ_slope ≈ 0.0014 ns/s)
  = −0.00004 ppm — the Bodnar's GPS-steering measured through the receiver
  chain. The free-running TCXO era would read ~+400 ns/s.
- **GEO cross-check** (phase_drift_producer): sbas131 −0.32 ns/s,
  sbas135 −0.98 ns/s — same sign, ~3× apart from each other (the known
  GEO-motion residual); the clock-bias path's −0.04 ns/s disagrees with both
  beyond their fit sigmas — discrepancy assigned to the GEO path's unmodeled
  motion, not to the new series.
- **Within-segment noise**: best 4.5 ns, median 25 ns (vs 273 ns global —
  the global figure is segment-to-segment sat-set bias jumps, the accepted
  cost of the raw-solve/no-median design). Within-seg noise ≈ code noise/√n
  (4–7 ns theory at n=5) — code-limited.
- **Claim-gate read**: RMS < 1 ns unreachable (25 ns typical within-seg);
  TDEV(1000) ≈ 0.8 ns plausible on white within-seg noise — untested today
  (no segment long enough). Tonight's richer sky (6–8 GPS expected
  18:00→05:00) may deliver the first evaluable continuous hour.

## Status of the claim

**Sub-ns claim: NOT SUPPORTED by today's data** — the pipeline is validated,
the observable is the limit. The path to the gates, in order:
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
