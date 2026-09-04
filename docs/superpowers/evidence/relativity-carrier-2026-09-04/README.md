# GPS relativistic periodic clock term, measured from the station's carrier phase (2026-09-04)

**Result: a = 0.91 ± 0.05 (formal, 5 windows, χ² 3.3/4); a = 0.89 ± 0.23 with
leave-one-satellite-out errors.** a = 0 excluded at 17σ (formal) / 3.8σ
(systematic-inclusive); a = 1 consistent (1.7σ / 0.5σ). The IS-GPS-200 term
dt_r = F·e·√A·sin(E_k) is present with the correct sign; amplitude good to ~20%.

Method (relativity_carrier_ms.py): between-satellite carrier differences
L = −(cyc_H − cyc_R)·c/f_L1 (sign verified: carrier_cycles rate == doppler_hz,
ratio 0.9996; dL/dt vs dM/dt slope 1.000 ± 0.005); model from broadcast
ephemeris (BKG daily BRDC DOY 237–246) with the POLYNOMIAL clock only, Klobuchar
iono, 2.418 m/sin(el) tropo; per-epoch projection of span{1, ρ̇_s} over all GPS
carriers absorbs the receiver clock, the stream-time tag lag (δ0 = 4.4 / 45.2 /
2692 s per session) and the per-channel steps left by USB-drop holes; first
differences at the native 10 s cadence with |residual| ≤ 0.10 m exclusion.
Regressor R = −c·Δdt_r; a = +1 ⇔ IS-GPS-200 sign. Null regressor (cos E):
−0.20 ± 0.12.

Windows: G15+G24 08-31 (1.21±0.27), G02+G07+G16 08-31 (0.86±0.09),
G15+G24 09-01 (0.94±0.31), G02+G07+G16 09-01 (0.90±0.08), G15+G24 09-02
(1.53±0.46); G02+G07+G16 09-03 excluded (2692 s tag lag, 45% steps — host-load
session). Swings per pass: G07 21–22 m, G02 17–19 m, G16 15–16 m, G24 12–13 m.

Caveats: single-frequency ionosphere is the dominant systematic (~15–20% of
amplitude); broadcast ephemeris only; per-satellite a values scatter ±1
(G16 anomalous). Pre-2026-08-30 archive unusable (wall-clock epoch stamping
before commit 3107967). The task-specified pairwise design
(relativity_carrier.py / relativity_carrier_pairwise.json) is degenerate with
a quadratic over the ≤2 h arcs this sky allows and is kept as a diagnostic.
The code-phase null test (relativity_null_test.*) was inconclusive
(a = −2.0 ± 2.2) for the same geometric reason.
