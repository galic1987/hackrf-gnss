# Precision instrument comparison — live station state

Date: 2026-08-26 (evening, post-GPSDO window). All numbers measured on the
live station today; nothing here is aspirational unless marked TARGET.

> **HISTORICAL SNAPSHOT — NOT CURRENT STATION STATE.** Pro #1 subsequently
> died. The retained station is Pro #2 + AA.250 for GNSS and HackRF One + the
> south-facing ClearStream for ATSC, with split 10 MHz and 1PPS wiring. The
> One has no demonstrated L-band/GNSS path, and no shared-RF or cross-radio
> delay calibration has been performed. Current status lives in `AGENTS.md`.

## The instruments, ranked by what they actually achieve right now

| Instrument / method | Measures | Live reading (today) | Demonstrated floor | Limit / target |
|---|---|---|---|---|
| Bodnar LBE-1421 (referee) | GPS-disciplined 10 MHz + 1PPS | reference | vendor-class expectation (~1e-11 ADEV class — NOT locally measured) | the standard others are judged against |
| WAAS GEO Doppler drift (Pro#1, tracker) | station clock error vs GEO carriers | +0.0006 ppm deadband (Bodnar-referenced); TCXO era was −0.34 ppm | ~±0.01 ppm (GEO orbital motion floor) | sub-ppb reality when referenced |
| ATSC pilot carrier phase (One, phase_producer) | clock error + displacement stability | offset −0.052 ppm (= ch35 transmitter constant); displacement σ 13 mm/10 s best, 90 mm/10 s median today | σ ~13 mm (10 s) | sub-mm needs the P0b carrier contract |
| FPGA tick counter (32 MHz, in-band nibble) | time quantization | 31.25 ns/tick, PC-poll jitter dominates host reads (median −0.02 ppm class) | 31.25 ns quantization | absolute epoch via 1PPS (bench item) |
| Carry-chain TDC (slot 0) | sub-tick subdivision | historical 499 ps figure retracted for external-trigger calibration | no demonstrated absolute floor | independent swept-edge calibration pending |
| Code PVT (live_fix) | position vs surveyed anchor | 6.3 m = solve RESIDUAL RMS (not error vs the anchor; the anchor itself is ~2 m class), 5 sats, 3D, fresh at publication (a preserved older trusted fix carries its own aging epoch) | ~2–5 m in good geometry | dm-class ceiling at L1-only (ionosphere); cm–mm needs short-baseline ∇Δ |
| Carrier replica phase (tracker channels) | per-channel phase | ~84–101 mm/10 s class — diagnostic, not a true observable | same | 0.5 mm TARGET requires P0b (prompt residual + epoch + ambiguity) |
| Band snapshots (band_producer) | per-band presence/drift | GATED OFF since round-11 (never touches the tracker-owned Pro) — off-tune bands show last measurement | — | can return on Pro#2 once it has a real antenna |
| Consensus (series_producer) | cross-instrument clock truth | −0.0285 ppm when drift-locked; null (fail-closed) when not | soft-verify wander RMS 0.009 ppm | now keyed on drift-lock, not a radio-opening probe |

## Clock eras measured today (the step is the story)

| Era | WAAS residual | ATSC offset | Note |
|---|---|---|---|
| TCXO, both radios internal | −0.34 ppm | +1.72 ppm | the pre-GPSDO baseline |
| Bodnar chain (Bodnar→Pro#1→Pro#2→One) | −0.0006–0.003 ppm | −0.052 ppm | current |

The clock-correction register semantics, measured in the window: fresh boot
= UNSET; boot init = unity; write lands requested==applied exactly; reset =
UNSET. The six-round 0.34 ppm puzzle was phantom cached intent; never a
hardware write.

## The antenna comparison (Pro#2 bench, this evening)

Three antennas, 3 cm apart on a south-facing line, same sky, same clock:

| Antenna | Instrument | Result |
|---|---|---|
| AA.250 (active patch) | Pro#1 tracker, continuous | 8–14 locks steady; top C/N0 43.6–45.3 dB-Hz |
| Bodnar puck | LBE-1421 NMEA GSV | 10 GPS sats, SNR max 47 (G26, el 81°) — a REFERENCE antenna readout, not a third coherent IQ array element |
| Antenna #3 ("GPS antenna", 3–5 V active) | Pro#2, per-band captures | **works at L1 GPS** — 5 PRNs (G27 63.7, G04 18.0, G03 10.6, G31 6.7, G16 4.2); the "no BeiDou/Galileo/SBAS" claim is RETRACTED (round-18: the B1I leg fed beidou_acq the wrong input format — invalid leg, not invalid band) |

CORRECTION (same evening, round-15 review chain): the first "zero
acquisitions" verdict was a TOOL bug, not the antenna —
antenna_compare.py's first revision captured 8 Msps at 1568.25 MHz
(±4 MHz spans neither L1 nor B1I) and the second left L1 at +7.17 MHz IF,
outside the acquisition engines' zero-IF Doppler search. Per-band zero-IF
captures (the band_producer pattern) fixed it; #3 hears fine.

What the fair captures show: #3 favors the LOW southern sky (G03 at 3° el
and G31 at 28° acquired solidly; G16 at 62-87° is its weakest sat in both
independent captures) — a horizon/south-facing pattern, consistent with
its physical mounting. The AA.250 is flatter and stronger (35-44 dB-Hz
across 7-81°, and it alone locks BeiDou + SBAS). #3's silence at B1I
(capture noise std 20.5 vs 41.4 at L1) plus no Galileo says its filter is
narrow around L1 — it is effectively an L1-only antenna. If #3 is to serve
MEO positioning, re-aim it toward zenith; as mounted it is a fine
GEO-belt/horizon antenna. The whip, when tested, should land several dB
under the AA.250 by physics (linear vs RHCP, no ground plane, no LNA).

## What the 3-cm baseline array is actually good for (and the honest caveats)

The pasted array analysis is mathematically right: total 6 cm < λ/2 (9.5 cm)
means zero integer ambiguity, the single-difference ΔΦ = (1/λ)(eᵏ·b) + cable
term is directly invertible, and the midpoint symmetry lets you split
geometry from cable/LNA group delay. The caveats from this station's review
discipline:

- "shared 10 MHz ⇒ 0 ppb relative drift" — precisely: the measured
  30-min wander RMS of the ATSC−WAAS difference is 0.009 ppm when
  drift-locked (soft-verified, with generation resets at chain breaks).
  Frequency lock is proven; an absolute common phase origin is NOT
  established by it — USB frame phase and the nibble/IQ skew put an unknown
  constant sub-µs offset between the radios' sample streams. The 1PPS line
  exists to calibrate exactly that.
- The sub-mm formal claims need a real carrier observable — today's
  carrier number is an integrated-replica diagnostic at ~90 mm/10 s. The
  ladder rungs are DIFFERENT observables, not refinements of one error:
  31.25 ns tick ≈ 9.4 m of light; the historical 499 ps TDC rung is not
  calibrated; 150 ps ≈ 45 mm;
  30–50 ps ≈ 9–15 mm; and carrier sub-mm additionally needs prompt
  residuals, sample epoch, ambiguity generation, slip/gap invalidation and
  calibrated hardware phase (the P0b contract — ledgered, not started).
- The two-HackRF differential pair over a 6 cm baseline is the legitimate
  cm→mm path (double-differencing cancels clocks/ionosphere/orbit) — and it
  is now physically possible: two Pros, one GPSDO, co-located antennas.

## Why GLONASS is not on the dashboard (and the stale band rows)

- The tracker tunes 1568.25 MHz at 16 Msps (span 1560.25–1576.25): GLONASS
  G1 (1602 MHz) and G2 (1246 MHz) lie outside it. GLONASS appears as
  predicted-only (hollow yellow on the dome, folded chips in the sky card,
  `cls: "predicted"` in the archive) — a deliberate honest label, and it is
  excluded from the learned mask by construction.
- The band table's GLONASS/L2C/L5/E5/B3I/E6 rows are snapshot products of
  band_producer, which has been gated off the tracker-owned Pro since
  round-11 (every snapshot collapsed the tracker). Those rows therefore show
  their last measurement (~68 h). They will come back live when
  band_producer's snapshots are repointed at Pro#2 — which needs a working
  antenna on Pro#2 first.
- Live today: GPS L1 C/A, Galileo E1, BeiDou B1I (live track), SBAS L1.
