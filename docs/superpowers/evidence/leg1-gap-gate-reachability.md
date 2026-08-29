# Leg-1 gap-gate reachability — measured arithmetic (2026-08-29, R15)

Reviewer question: after GAP_FACTOR moved 5× → 2× → 1.5× (`690ca52`,
`e7f0e22`), is the analyzer's 3600 s continuous-hour gate *unreachable by
construction* at the observed epoch cadence — i.e. is "INSUFFICIENT DATA" an
arithmetic consequence rather than a measurement?

## Measured (clock_bias.jsonl, gen v2, 20,462 rows over 21.5 h)

| quantity | value |
|---|---|
| median epoch interval | 1.040 s |
| gap threshold (1.5× median) | 1.560 s |
| intervals exceeding threshold | 403 / 20,461 |
| epoch-miss probability p per interval | 0.0197 |
| intervals inside a 3600 s window | ~3,461 |
| P(one clean 3600 s segment) = (1−p)^3461 | **1.3 × 10⁻³⁰** |
| longest clean segment observed (21.5 h) | 764 s |
| segments / mean length | 404 / 52.7 s |
| top segments (s) | 764, 553, 500, 459, 445, 440, 410, 381, 380, 363 |

Reproduce: `scripts/` ad-hoc analysis over `gnss/observations/clock_bias.jsonl`
(gen `v2*`), split rule identical to `clock_bias_analyzer.py` (GAP_FACTOR 1.5).

## Verdict on the question

Yes — unreachable by construction. At a ~2 % per-epoch miss rate, a 1.5×-median
gap rule makes a clean hour a 10⁻³⁰ event. No physically achievable clock
improvement changes that number; only the miss rate or the gate rule does.

This does **not** rescue the sub-ns claim: the poison-gate kept-flow shortfall
(~5.6 kept rows/min vs the 56.7/min the hour needs — overnight verdict,
`76cb0ff`) independently blocks the hour even with a permissive gap rule. The
station has two independent, separately-quantified blockers:

1. **Density** (poison gate): kept-flow an order of magnitude short — a real
   measurement of the current observable.
2. **Continuity** (gap gate): arithmetically unreachable at the observed 2 %
   epoch-miss rate — a property of the gate × cadence, not of the clock.

## 2026-08-29 addendum (round 18): blocker 1 was the GATE, not the observable

Re-derived from all 20.8k v2 rows after the round-18 review: the best raw
hour carried 3,214 quality rows (53.6/min — near-complete), and the flat
100 m poison threshold cut it to 585 (18.2%) because healthy hours have a
residual median of 137 m — the threshold sat below the p25 of good data.
"Density shortfall" was a miscalibrated gate measuring itself. The poison
threshold is now 500 m, set from the data's own distribution (quality
residuals p50 148 / p95 300 / p99 410 m; 0.02% > 500 m; none > 1000 m);
with it, longest clean segment 52 s → 443 s and poison-excluded rows
~9,000 → 4. The underlying cause of the 137 m-class residuals is the
measurement model (no iono/tropo/SBAS corrections; Hatch window vs
code-carrier mismatch) — NOT the sky. Blocker 2 (continuity at the ~2 %
epoch-miss rate, longest zero-miss run ~760 s = 21 % of the gate) stands
exactly as measured and is now THE binding constraint.

## Paths (decision deferred — user away, no unilateral gate loosening)

- Fix the miss rate: identify why ~2 % of 1 Hz clock_bias epochs never publish
  (producer stall, solver drop, channel realignment). If p drops to ~10⁻⁴,
  P(clean hour) rises to ~0.7 — the gate becomes a real test again.
- Or change the gate to explicit uniform-grid resampling with a documented
  interpolation rule (already the standing recommendation in
  `clock_bias_analyzer.py` review notes) instead of index-based TDEV across
  split segments.

Until one lands, any "final" verdict wording must keep both blockers named;
"insufficient data" alone misattributes blocker 2 to the observable.
