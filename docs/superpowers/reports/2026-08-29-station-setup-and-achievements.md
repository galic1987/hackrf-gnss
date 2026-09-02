# Station Setup & Achievements — GNSS Precision Timing Station

2026-08-29 19:15 EDT. Standing reference for what the station is, how it is
wired, what it has verifiably achieved, and what it has not. Every number
here was re-checked against live state at writing time; where a figure is
provisional or retracted, it says so.

> **Evening addendum (2026-08-29 22:10):** three updates. (1) The v3
> bake-off: no clean hour (best 684 s raw / 312 s quality-filtered) but
> the failure mechanism is identified — the constellation hovers at
> exactly 5–6 measurements and the inherited n≥5 emission gate turns
> every dip into a 3–60 s stall; relaxing it to n≥4 for the 1-state
> solver is the queued fix. (2) A clock-cable interruption wedged the
> nav pipeline (zero subframe validations for ~1 h, surviving two
> relocks and an MCU reset); root-caused as FPGA-side state that only a
> USB power cycle clears — the reset law now includes it. (3) **First
> external TDC PPS capture**: 1PPS via P28 pin 16 (the Pro's TRIGGER.IN,
> same header as the One — an "SMA-only" doc claim was corrected), one
> valid-toggle per pulse, thermometer codes across the full 48-tap
> window; a 3600-pulse jitter dataset is being collected. Topology is
> now: single Bodnar output → matched splitter → both CLKINs; second
> output → 1PPS → both TRIGGER.INs, matched cables.

## TL;DR

A HackRF Pro, referenced to a GPSDO in a star topology, autonomously
tracks GPS L1 C/A, Galileo E1, BeiDou B1I and SBAS WAAS, demodulates SBAS
messages, solves a fixed-anchor clock-bias observable at 1 Hz, and
publishes everything to a live education panel (http://localhost:8090/sync.html).
The day's headline: the P0b correction (MT9 GEO orbit+clock removal from
WAAS carrier phase) collapses the ±2e-3 ppm diurnal GEO-motion term to a
~3e-5 ppm cross-GEO closure — a verified ~74× improvement, now the panel's
live observable. The sub-nanosecond claim gates remain unevaluated: no
continuous clean hour of clock-bias data exists yet.

## 1. Physical setup

```
                Leo Bodnar LBE-1421 GPSDO (GPS-disciplined reference)
             OUT2 (10 MHz)                        OUT1 (1PPS)
                   |                                   |
            [ SMA Power Splitter ]              [ SMA Power Splitter ]
            /                    \              /                    \
           / matched cables       \            / matched cables       \
          v                        v          v                        v
 HackRF One P1 (CLKIN)   HackRF Pro P1      HackRF One P28 pin 16   HackRF Pro P28 pin 16
 …922c63dc21748847       …645061de252d6613      (TRIGGER.IN)            (TRIGGER.IN)
 
 RF paths are independent in this retained configuration:
   AA.250 active GNSS patch -> HackRF Pro RF IN
   south-facing ClearStream -> HackRF One RF IN (ATSC operational 2026-09-01)
 A shared-RF splitter/ABBA calibration is proposed, not deployed evidence.

  HackRF Pro #1 (…977c…) — DEAD, out of the station. Never target it.
```

- **Star, not a cascade.** Both radios hang directly off the GPSDO over
  equal-length cables. Shared 10 MHz syntonizes the radios (same frequency);
  it does NOT align sample zero, timestamp phase, or RF/carrier phase.
- **Clock eras, measured on air:** free-running Pro TCXO ≈ +0.53 ppm;
  GPSDO-referenced star era ≈ −0.054 ppm; broken-chain era −1.7 ppm. The
  star restoration is visible in the drift history.
- **Physical labels:** OUT2 is the split 10 MHz source; OUT1 is the split
  1PPS source. Electrical presence and NMEA navigation health do not prove
  PPS phase, output delay, or oscillator lock; retain photos/configuration
  records with any final measurement.
- **USB:** the Pro streams ~32 MB/s; simultaneous Pro+One streaming on one
  USB2 bus is unsafe — separate root controllers required for dual-radio
  work.

### Hardware inventory

| unit | role | status |
|---|---|---|
| HackRF Pro #2 `645061de` | production GNSS radio | live; release **0x469 self-reported BUILD_ID tags smoke-tested on all 4 FPGA slots** (2026-08-28); flash bytes/bitstream hashes were not read back |
| HackRF One `922c63dc` | ATSC carrier-phase witness | clock-detected, **ClearStream ATSC operational, GPS-unproven**; use fresh `state.phase.json` for live lock state |
| HackRF Pro #1 `977c` | — | dead |
| Leo Bodnar LBE-1421 | station frequency reference | live; NMEA health probed at 5 s cadence (`gpsdo_probe`) |
| Taoglas AA.250 | Pro #2 antenna (Ø86.4 mm active patch) | live |

> **2026-09-01 ClearStream power finding:** commit d1bc6c0 records that
> antenna-port power raised amplitude from ~0.05 to >8 and restored a real
> lock. The running child therefore remains at `-p 1`; do not "correct" it
> off. Although the antenna is described as passive, the complete coax/DC
> path is unresolved and may contain an inline powered stage or injector.
> Future starts require the explicit acknowledged bias-on profile and publish
> requested RF config; identify the DC load at a controlled maintenance
> boundary before changing it.

## 2. Software pipeline (all processes verified alive 19:00 EDT)

| process | what it does |
|---|---|
| `live_radio` (Rust, tracker) | GPS L1 C/A + Galileo E1 + BeiDou B1I + SBAS WAAS acquisition/tracking; nav decode; publishes `state.tracker.json` |
| `clock_bias` v3 (Rust) | fixed-anchor 1-state clock solver vs surveyed `site.json`; GPS+BDS measurement vector; 1 Hz rows to `clock_bias.jsonl` |
| `phase_drift_producer.py` | WAAS GEO carrier-phase drift with **P0b correction**; consensus vote to the panel |
| `tracker_producer.py` / `series_producer.py` / `band_producer` / `sky_producer` | panel series, band occupancy snapshots, sky view |
| `gpsdo_probe.py` | Bodnar NMEA health → `state.gpsdo.json` (fix quality, sats, HDOP; **NMEA fix, not oscillator telemetry**) |
| `clock_bias_shadow.py` / `archive_roller.py` | shadow validation copies; daily Parquet archive |
| web panel (`web/sync.html`) | live sync dashboard: per-component educational modals, hardware diagrams wired to real board state |

Live tracker snapshot at writing: **15 satellites — GPS 6, Galileo 5,
BeiDou 2, SBAS 2.**

## 3. What has been achieved (verified)

1. **Four-constellation live tracking on a GPSDO-referenced HackRF Pro.**
   GPS L1 C/A, Galileo E1, BeiDou B1I, SBAS WAAS concurrently, with
   acquisition seeded from broadcast ephemeris (RINEX hourly).
2. **SBAS demodulation depth.** 250-sym Viterbi + CRC24Q framing; MT2–5
   fast corrections decoded *and applied*; MT25/26 (long-term + iono)
   parsed with tests; MT9 GEO navigation vectors published to consumers.
3. **P0b — the day's headline (commits fcd66eb, 0000c8f, hardened
   ff3671a + ccf84e2).** The MT9 GEO state vector is propagated
   (2nd-order, Sagnac-frame site vector) and removed from the WAAS
   carrier phase: the ±2e-3 ppm diurnal GEO-motion term collapses to a
   **~3e-5 ppm cross-GEO closure** (paired same-epoch diff, PRN 131 vs
   135) — **~74× improvement**, verified on a preserved, hashed slice
   (`2026-08-29-p0b-acceptance-reissued.md`). Fail-closed: no usable MT9
   → no row. Message-freshness gate rejects frozen vectors (300 s);
   ephemeris swaps are validated and stitch-bounded (500 cycles). The
   corrected value is the panel's emitted GEO observable.
   **Honesty note:** an earlier "~9000× / 2.36e-7 ppm" figure was
   retracted the same day — it was an unpaired-median artifact. The 74×
   paired figure is the standing one. Output is **observe-only** —
   inter-GEO disagreement (~3e-5 ppm) is the same order as the receiver
   clock signal itself (−2.4e-5 ppm).
4. **BeiDou in the clock solver (8d420ed).** BDS B1I measurements join
   the clock-bias vector; a raw-BDT-toe bug in the orbit node term
   (BDT = GPST − 14) was found and fixed — cross-check `isx` went
   −2.29 km → −2.1 m. 55% of v3 rows now carry BDS.
5. **Clock-bias engine v3.** Fixed-anchor 1-state solver against the
   surveyed site anchor (15–40× variance reduction vs free 4D solves in
   simulation); 1 Hz rows with generation provenance (producer start +
   tracker build identity); residual RMS live ~120–440 m (code-limited,
   no iono/tropo modeled yet).
6. **Analyzer v2 (NIST SP 1065).** MDEV-derived TDEV (τ/√3·MDEV), gap
   segmentation at 1.5× median dt, continuous-hour gates (span ≥ 3600 s,
   rows ≥ 3400), generation separation, missing-τ rejection. The 500 m
   poison gate is **pre-registered as data-derived** — verdicts using it
   are labeled exploratory.
7. **Firmware identity smoke test.** Release 0x469 BUILD_ID tags were read on all four FPGA slots
   of the production radio; the invalid 0xE91 experiment (reversed
   FPGA-clock/trigger pin swap) quarantined to cold storage with hashes;
   this does not attest flash bytes or deployed bitstream hashes. The
   misleading "calibrated TDC" claim is retracted: no external-trigger
   calibration or 499 ps absolute floor follows from the retained run.
8. **GPSDO health monitoring.** The reference's own NMEA is watched at
   5 s cadence with a 30 s TTL — a dead probe reads degraded, never
   healthy.
9. **Operational lesson.** The station survived a Pro unplug/wedge cycle,
   which established the restart ordering in AGENTS.md. The legacy automatic
   reset watcher is now retired; monitoring is observe-only and hardware
   recovery is a manual maintenance-window procedure, not a supervisor mode.
10. **Education interface.** The sync panel documents every stage of the
    signal/clock path against the real hardware (MAX2831/MAX5864/
    Si5351C/RFFC5072 class components), each tooltip carrying
    what-happens-here, why-it-matters, and what-can-go-wrong.

## 4. What has NOT been achieved (standing negative results)

- **Sub-nanosecond absolute code-phase is physically unachievable** due to ionospheric diurnal variation (10–100+ ns), multipath, and anchor uncertainty (physical floor is 10–50 ns). The `< 1.0 ns` target is officially re-registered as a **carrier-phase TDEV stability claim** ($\sigma_x(\tau) < 1.0\text{ ns}$).
- **TCXO vs Bodnar remains conditional:** the historical Run 1 ratio is
  consistent with about $-0.667\text{ ppm}$, but the retained artifact lacks
  clock-source/rate/build/temperature provenance. Run 3 supports relative
  common-source coherence. NMEA fix validity is not oscillator/PPS truth.
- **TDC scale is not calibrated:** 79.74% full-scale occupancy and the
  historical 105.5 ps estimate are circular unless uniform input phase and
  the composite code-48 transfer function are independently established.
  The 16-code pattern is consistent with fabric-hop DNL; source visitation
  remains unresolved.
- **TDC external calibration is blocked:** the first sweep procedure was
  quarantined after review found a 10 MHz post-reset clock could be mislabeled
  as 40 MHz, non-atomic register reads, and no independent uniform-phase proof.
- **The One is not a GPS receiver yet.** Its south-facing ClearStream now provides a valid ATSC pilot stream, but that does not establish L-band reception. The common-antenna shared-RF calibration remains a separate scheduled experiment.
- **BDS carrier sign verified; inter-system bias unmodeled** — v3 clock-bias uses fixed-anchor clock solve with studentized rejection.
- **The GPSDO discipline loop is design-only/shadow-only** — the correction register is computed, never written.

## 5. What is next (ranked)

1. **Preserve the live station:** observe-only supervision; no build, image
   switch, reset, capture, or source change outside an explicit hardware
   window.
2. **Next maintenance window (tracker restart required):** tracker
   session-UUID provenance propagated into clock_bias/P0b/PVT rows; BDS
   inter-system-bias state + retained-satellite identity emission;
   deaf-band watchdog (the 15:24 poisoned-start incident class);
   fit_hist session keying.
3. **TDC redesign before another run:** atomic fresh-event/status-bracketed
   capture, direct clock/build/source attestation, and an independently swept
   or phase-tagged stimulus.
4. **User-physical:** retain photos/labels for both splitters and matched
   cables; preserve the One's confirmed ClearStream ATSC path; perform a
   safe-gain shared-RF/ABBA test as an explicit, reviewed RF rewire only
   after the capture manifest and USB-root plan are frozen.
5. **Then:** MT9 oracle validation (independent ephemeris cross-check),
   second-GEO promotion review for Tier-1 voting, GEO-in-solver ranging
   (needs SBAS pseudorange production in the tracker), TDC external
   swept-edge calibration (needs the PPS port).

## Provenance

Repo `gnss/hackrf_gnss` (master, local-only). Today's 20 commits
28bd1ca…ccf84e2. Live state files under `gnss/observations/`. Working
ledger `.superpowers/sdd/progress.md` (local, uncommitted by design).
