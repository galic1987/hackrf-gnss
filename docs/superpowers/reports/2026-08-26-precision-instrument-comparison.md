# Precision instrument comparison — live station state

Date: 2026-08-26 (evening, post-GPSDO window). All numbers measured on the
live station today; nothing here is aspirational unless marked TARGET.

## The instruments, ranked by what they actually achieve right now

| Instrument / method | Measures | Live reading (today) | Demonstrated floor | Limit / target |
|---|---|---|---|---|
| Bodnar LBE-1421 (referee) | GPS-disciplined 10 MHz + 1PPS | reference | GPSDO class (~1e-11 ADEV) | the standard others are judged against |
| WAAS GEO Doppler drift (Pro#1, tracker) | station clock error vs GEO carriers | +0.0006 ppm deadband (Bodnar-referenced); TCXO era was −0.34 ppm | ~±0.01 ppm (GEO orbital motion floor) | sub-ppb reality when referenced |
| ATSC pilot carrier phase (One, phase_producer) | clock error + displacement stability | offset −0.052 ppm (= ch35 transmitter constant); displacement σ 13 mm/10 s best, 90 mm/10 s median today | σ ~13 mm (10 s) | sub-mm needs the P0b carrier contract |
| FPGA tick counter (32 MHz, in-band nibble) | time quantization | 31.25 ns/tick, PC-poll jitter dominates host reads (median −0.02 ppm class) | 31.25 ns quantization | absolute epoch via 1PPS (bench item) |
| Carry-chain TDC (slot 0) | sub-tick subdivision | 499 ps RMS — PROVISIONAL internal ring-oscillator self-test | 499 ps (internal) | external 1PPS swept-edge calibration pending |
| Code PVT (live_fix) | position vs surveyed anchor | 6.3 m RMS, 5 sats, 3D (current); 9–16 m typical with WAAS corr | ~2–5 m in good geometry | dm-class ceiling at L1-only (ionosphere); cm–mm needs short-baseline ∇Δ |
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
| Bodnar puck | LBE-1421 NMEA GSV | 10 GPS sats, SNR max 47 (G26, el 81°) |
| Antenna #3 ("GPS antenna", 3–5 V active) | Pro#2, 12-s captures + 4 acquisition engines | **zero acquisitions**, even at 10-s integration / 250 Hz Doppler step; LNA alive (noise std 23.6→31.2 with bias) but no signal |

Reading: #3's LNA draws bias and raises the noise floor, yet not one
satellite crosses threshold where the AA.250 locks 8+. Either its patch
element is dead/disconnected behind a live LNA, or it is not an L1-band
antenna at all. Next decisive test (no production cost): put #3 on the
Bodnar's own SMA — the GPSDO's GSV SNR table will say within a minute
whether it hears anything. The whip, when tested, should land several dB
under the AA.250 by physics (linear vs RHCP, no ground plane, no LNA).

## What the 3-cm baseline array is actually good for (and the honest caveats)

The pasted array analysis is mathematically right: total 6 cm < λ/2 (9.5 cm)
means zero integer ambiguity, the single-difference ΔΦ = (1/λ)(eᵏ·b) + cable
term is directly invertible, and the midpoint symmetry lets you split
geometry from cable/LNA group delay. The caveats from this station's review
discipline:

- "shared 10 MHz ⇒ 0 ppb relative drift" is now TRUE and soft-verified
  (wander RMS 0.009 ppm). But shared clock ≠ shared sample zero: USB frame
  phase and the nibble/IQ skew put an unknown constant sub-µs offset between
  the radios' sample streams. The 1PPS line exists to calibrate exactly that.
- The <0.5 mm formal claim needs a real carrier observable — today's
  carrier number is an integrated-replica diagnostic at ~90 mm/10 s. The
  P0b contract (prompt residual + sample epoch + ambiguity generation + gap
  invalidation) is the gate; the design is ledgered, not started.
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
