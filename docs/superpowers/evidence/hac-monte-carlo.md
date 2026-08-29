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

## What replaced it

- `d70ba03`: `fit_drift` restored to OLS, limitation documented in the
  docstring; test suite pins the calibration fact (AR(1) MC: sigma understates,
  slope unbiased).
- `9fc6fb3`: disjoint-window MAD scatter (`scatter_sigma_ppm = 1.4826 × MAD` of
  per-window OLS slopes). Live-verified 2026-08-29: sbas131 per-window OLS
  8.9 × 10⁻⁶ ppm vs cross-window scatter 1.78 × 10⁻⁴ ppm — i.e. OLS understates
  6–20× and the scatter estimator covers the empirical spread (1.32 × 10⁻³ ppm
  vs 1.15 × 10⁻³ ppm benchmark on synthetic AR(1)).
