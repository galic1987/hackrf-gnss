# Historical calibration ledger — superseded, not claim-grade

> **QUARANTINED 2026-08-30.** This ledger predates the deployed Split Star and
> mixes conditional ratios, NMEA navigation status, nominal clock assumptions,
> and topology-dependent observations under the word "calibrated." It is kept
> as a historical index to raw evidence, not as a current calibration
> certificate. In particular, the Bodnar NMEA stream does not report 10 MHz
> lock, PPS phase, UTC offset, or holdover; Pro/One clock and TDC claims below
> must be re-established under the evidence gates in the current timing docs.

Date: 2026-08-27. Declared reference configuration: Leo Bodnar LBE-1421
10 MHz + 1PPS, wired into the station 2026-08-26. The retained evidence does
not independently attest the output lock state or make every number below an
absolute measurement.

## Frequency references

| Source | Calibration datum | Method | Status |
|---|---|---|---|
| Bodnar LBE-1421 | GPSDO-derived outputs; vendor class ~1e-11 ADEV was quoted but NOT locally measured, and current NMEA is not output-lock telemetry | GPS survey-in via NMEA (position verified ~2 m class) | declared reference, uncalibrated locally |
| Pro#1 tick (adclk counter) | −0.018 ppm vs session ref (median, PC-read jitter dominates); WAAS residual +0.000–0.003 ppm deadband | in-process tick poll while streaming + WAAS GEO Doppler | CALIBRATED live |
| Pro#1 correction register | unity (never written; write→exact→clear→unity verified) | hackrf_debug radio-register readback, bank APPLIED | VERIFIED 2026-08-26/27 |
| Pro#2 TCXO | +0.53 ppm | ATSC pilot offset on the One (fed by Pro#2 CLKOUT), continuous | CALIBRATED live |
| The One (chain-fed) | −0.052 ppm when GPSDO-chained (the ch35 transmitter constant); +0.53 ppm now that it rides Pro#2's TCXO | ATSC ch35 carrier-phase track | CALIBRATED per topology |
| Clock chain drift lock | wander RMS 0.004–0.009 ppm (30-min windows), generation resets at chain breaks | series_producer soft verifier (ATSC−WAAS diff) | VERIFIED live |

## Time & phase planes

| Plane | Figure | Status |
|---|---|---|
| Tick quantization | 31.25 ns (verified on hardware: 2.84M sequence-validated anchors) | REAL |
| Absolute epoch (UTC anchor) | NONE — 1PPS TS_SET pending; the trigger latch froze on its first edge (ledger) | UNCALIBRATED |
| TDC (48-tap rev3c) | median 0.72 ns/bin (best 149 ps, worst 3.2 ns), DNL-uneven, 5k samples | **RETRACTED 2026-08-28** — was recorded as CALIBRATED (code-density); audit showed the run was the on-die ring-oscillator self-test, not an external edge: popcount is a wave-occupancy statistic aliased at T_ro ≈ 8.76 ns (cannot resolve single-edge phase), and the absolute scale is a hardcoded nominal constant, never measured. Raw data preserved in `observations/tdc_rescue_20260827/`. External swept-edge calibration still pending (latch gap) |
| Carrier phase (ATSC ruler) | σ 13 mm/10 s best, ~90 mm/10 s median | measured; not a true P0b observable |
| GNSS carrier (tracker) | integrated replica diagnostic, ~84–101 mm/10 s | diagnostic only |

## Position & receive plane

| Source | Datum | Status |
|---|---|---|
| Site anchor | 39.0029556, −77.6051478, 77.1 m ellipsoidal (~2 m class) | SURVEYED (Bodnar survey-in, 901 samples) |
| PVT residual | 6–16 m RMS class (per-solve residual, not anchor error) | measured |
| Per-band noise floors | sweep 2026-08-26 (Pro#2+whip): HF −13 dB max, GNSS L1 −33 dB max, floor −39.2 dB median | measured |
| Antennas | AA.250: 8–14 locks, C/N0 to 45.3; whip: broadband, uniform; #3: L1-only pattern, horizon-facing | measured |
| Iridium | AA.250 deaf (std 5.1, SAW rolloff >1610); whip hears band, 0 bursts in 3×30 s windows tonight | measured |

## Known-suspect / uncalibrated

- **Idle tick-snapshot path**: `--ts-read now` on an IDLE radio returns
  wildly drifting rates (+48.8 → +152.3 ppm across two measurements) — the
  now-latch is not trustworthy without an active RX stream. Use the in-
  process tick poll (streamed) or the ATSC path instead. Firmware question
  for the ledger: is the sync→adclk snapshot latch guaranteed with no RX?
- **Trigger latch re-freeze**: latched its first edge and never moved —
  armed-start edges don't re-freeze it (GPSDO ABI blocker class).
- **Cable/LNA group delays**: unmeasured; the midpoint-symmetry method is
  the plan once the trigger path works.
- **Common phase origin between radios**: shared 10 MHz removes drift, not
  sample zero. Needs the 1PPS epoch transfer.
