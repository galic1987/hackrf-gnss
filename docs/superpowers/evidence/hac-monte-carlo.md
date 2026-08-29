# HAC Monte-Carlo — archived artifact (2026-08-28, commit `d70ba03`)

Round-13 prescribed Newey-West HAC (L=4, Bartlett) for `fit_drift` slope sigma
on AR(1)-correlated phase residuals. It was implemented, measured, and
**reverted** — this file pins the calibration evidence as a durable artifact
(R15 reviewer request: evidence as artifact, not prose).

## Monte-Carlo setup

- 4,000 trials; AR(1) phase noise ρ = 0.95; window n = 40 samples.
- Truth: empirical slope scatter across trials = **1.15 × 10⁻³ Hz**.

## Result

| estimator | median reported sigma | 1-σ coverage | verdict |
|---|---|---|---|
| white-noise OLS (s²/Sₓₓ) | 2.4 × 10⁻⁴ Hz | **17 %** | ~5× too tight (confirms the R13 review) |
| Newey-West HAC (L=4) | 6.1 × 10⁻⁵ Hz | **4 %** | ~19× too tight — *worse than OLS* |

## Mechanism (why residual-based HAC fails here)

Detrending absorbs the low-frequency wander into the fit itself; the residuals
every residual-based variance estimator reads are already high-passed, so the
long-run variance Ω is not recoverable from a single short window. The
prescription was falsified by measurement, not by argument.

## Round-18 re-run (2026-08-29): the reviewer's ~90% is NOT reproduced — revert stands

The round-18 review reproduced the opposite (~90% coverage with a "correct
Bartlett/Newey-West") and suspected the revert measurement was wrong. Re-ran
the MC at the documented parameters plus a regime sweep
(`/tmp/hac_rerun.py`, 2,000–4,000 trials per cell, AR(1) ρ=0.95):

| setting | true scatter | median sigma | 1-σ coverage |
|---|---|---|---|
| n=40, OLS | 3.26 × 10⁻² | 9.4 × 10⁻³ | 23.4 % |
| n=40, HAC on residuals | 3.26 × 10⁻² | 2.5 × 10⁻⁴ | 0.5 % |
| n=40, HAC on scores (proper sandwich) | 3.26 × 10⁻² | 2.4 × 10⁻³ | 5.8 % |
| n=40, HAC L=10 / L=20 | 3.29 × 10⁻² | 2.9–3.0 × 10⁻³ | 6.7–6.9 % |
| n=40, AR(1)-prewhitened HAC | 3.29 × 10⁻² | 5.2 × 10⁻³ | 19.4 % |
| n=200, HAC L=4 / L=20 | 6.03 × 10⁻³ | 1.5–2.4 × 10⁻⁴ | 2.0–3.2 % |
| n=400, HAC L=20 | 2.45 × 10⁻³ | 6.9 × 10⁻⁵ | 2.4 % |

No cell approaches 90%; the sandwich (scores, not residuals — the suspected
implementation bug) helps and still fails. The revert's mechanism
(detrending absorbs the low-frequency wander; one window cannot recover it)
holds at every tested setting. The calibrated answer remains the
disjoint-window scatter, which round 16 wired into the emitted rows.

The reviewer's separate point — "the single-GEO consensus sigma understated
scatter by 64×" — was measured against the pre-round-16 emitter; post-fix
the fused GEO row carries the calibrated sigma (live-verified 2026-08-29:
7.46 × 10⁻⁵ provisional-5× before 5 windows; 0.00179 class fused).

- `d70ba03`: `fit_drift` restored to OLS, limitation documented in the
  docstring; test suite pins the calibration fact (AR(1) MC: sigma understates,
  slope unbiased).
- `9fc6fb3`: disjoint-window MAD scatter (`scatter_sigma_ppm = 1.4826 × MAD` of
  per-window OLS slopes). Live-verified 2026-08-29: sbas131 per-window OLS
  8.9 × 10⁻⁶ ppm vs cross-window scatter 1.78 × 10⁻⁴ ppm — i.e. OLS understates
  6–20× and the scatter estimator covers the empirical spread (1.32 × 10⁻³ ppm
  vs 1.15 × 10⁻³ ppm benchmark on synthetic AR(1)).
