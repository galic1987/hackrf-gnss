# Station Setup & Achievements — GNSS Precision Timing Station

2026-08-29 19:15 EDT. Standing reference for what the station is, how it is
wired, what it has verifiably achieved, and what it has not. Every number
here was re-checked against live state at writing time; where a figure is
provisional or retracted, it says so.

## TL;DR

A HackRF Pro, disciplined by a GPSDO in a star topology, autonomously
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
                Leo Bodnar LBE-1421 GPSDO (GPS-disciplined 10 MHz ref)
                 OUT1 (10 MHz)                OUT2 (10 MHz)
                   | matched-length             | matched-length
                   | 50 ohm SMA                 | 50 ohm SMA
                   v                            v
        HackRF One P1 (CLKIN)          HackRF Pro #2 P1 (CLKIN)
        …922c63dc21748847              …645061de252d6613
        whip antenna (RF-dark)         AA.250 active GNSS patch
                                       P2 SMA free (CLKOUT off in sw)

  HackRF Pro #1 (…977c…) — DEAD, out of the station. Never target it.
```

- **Star, not a cascade.** Both radios hang directly off the GPSDO over
  equal-length cables. Shared 10 MHz syntonizes the radios (same frequency);
  it does NOT align sample zero, timestamp phase, or RF/carrier phase.
- **Clock eras, measured on air:** free-running Pro TCXO ≈ +0.53 ppm;
  GPSDO-referenced star era ≈ −0.054 ppm; broken-chain era −1.7 ppm. The
  star restoration is visible in the drift history.
- **Physical-labeling hold (USER-PHYSICAL):** repo documents say OUT2→Pro,
  OUT1→One, but external-clock detection cannot prove which Bodnar output
  feeds which radio. Both cable ends must be photographed and labeled
  before either output is re-tasked to 1PPS.
- **USB:** the Pro streams ~32 MB/s; simultaneous Pro+One streaming on one
  USB2 bus is unsafe — separate root controllers required for dual-radio
  work.

### Hardware inventory

| unit | role | status |
|---|---|---|
| HackRF Pro #2 `645061de` | production GNSS radio | live; release **0x469 manifest-verified on all 4 FPGA slots** (2026-08-28) |
| HackRF One `922c63dc` | second clock witness | clock-detected, **RF-dark, GPS-unproven** (user owes physical RF-path inspection) |
| HackRF Pro #1 `977c` | — | dead |
| Leo Bodnar LBE-1421 | station frequency reference | live; NMEA health probed at 5 s cadence (`gpsdo_probe`) |
| Taoglas AA.250 | Pro #2 antenna (Ø86.4 mm active patch) | live |

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

1. **Four-constellation live tracking on a GPSDO-disciplined HackRF Pro.**
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
7. **Firmware integrity.** Release 0x469 attested on all four FPGA slots
   of the production radio; the invalid 0xE91 experiment (reversed
   FPGA-clock/trigger pin swap) quarantined to cold storage with hashes;
   the misleading "calibrated TDC" claim retracted everywhere (499 ps is
   a ring-oscillator self-test LSB, provisional).
8. **GPSDO health monitoring.** The reference's own NMEA is watched at
   5 s cadence with a 30 s TTL — a dead probe reads degraded, never
   healthy.
9. **Operational robustness.** The station survived a Pro unplug/wedge
   cycle: a recovery watcher detected, reset, restarted and re-verified
   the pipeline unaided. The corrected window law (reset immediately
   after kill, before any build) is in AGENTS.md.
10. **Education interface.** The sync panel documents every stage of the
    signal/clock path against the real hardware (MAX2831/MAX5864/
    Si5351C/RFFC5072 class components), each tooltip carrying
    what-happens-here, why-it-matters, and what-can-go-wrong.

## 4. What has NOT been achieved (standing negative results)

- **Sub-nanosecond timing: unevaluated, not failed.** No continuous
  clean hour of clock-bias data has ever existed (v2 best 764 s; v3 best
  at writing ~680 s and improving). The RMS/TDEV claim gates have never
  had qualifying input. Any "−0.038 ns/s" style number in older docs is
  exploratory and unretracted-in-place only where marked historical.
- **TDC external calibration: not done.** All "carry-chain TDC" figures
  are the internal ring-oscillator self-test; no external single-edge
  measurement, no atomic coarse+fine image exists in any shipped slot.
- **The One is not a GPS receiver yet.** It runs an ATSC pilot stream,
  clock-detected but RF-dark (~0.02–0.05 z vs 0.75 z floor). No common
  antenna splitter test has been performed.
- **Absolute PPS epoch: not available.** Both Bodnar outputs are 10 MHz;
  the PPS port-budget decision (distribution amp / alternate CLKIN /
  pause-the-One) is open and cable-labeling-gated.
- **BDS carrier sign unverified; GPS/BDS inter-system bias unmodeled** —
  v3 clock-bias is observe-only for mixed constellation.
- **The GPSDO discipline loop is design-only/shadow-only** — the
  correction register is computed, never written.

## 5. What is next (ranked)

1. **Tonight 20:41 EDT — v3 availability bake-off (armed, automatic):**
   does the GPS+BDS solver deliver a continuous clean hour? If yes →
   first-ever TDEV evaluation on clean data.
2. **Next maintenance window (tracker restart required):** tracker
   session-UUID provenance propagated into clock_bias/P0b/PVT rows; BDS
   inter-system-bias state + retained-satellite identity emission;
   deaf-band watchdog (the 15:24 poisoned-start incident class);
   fit_hist session keying.
3. **User-physical, when the user returns:** photograph/label both
   Bodnar cable ends; inspect the One's RF path; decide the PPS port
   budget; common-antenna splitter test for the One; antenna phase-center
   survey if spatial work resumes.
4. **Then:** MT9 oracle validation (independent ephemeris cross-check),
   second-GEO promotion review for Tier-1 voting, GEO-in-solver ranging
   (needs SBAS pseudorange production in the tracker), TDC external
   swept-edge calibration (needs the PPS port).

## Provenance

Repo `gnss/hackrf_gnss` (master, local-only). Today's 20 commits
28bd1ca…ccf84e2. Live state files under `gnss/observations/`. Working
ledger `.superpowers/sdd/progress.md` (local, uncommitted by design).
