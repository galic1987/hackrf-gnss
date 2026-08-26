# Clock-write continuity — live experiment design (P1)

Date: 2026-08-25. Status: design, awaiting a maintenance window (needs the
user present; deliberately issues hardware writes).

## What is already proven (retro pass, 2026-08-25)

Every observable correction write in the archive (122/122, step sizes down to
±0.01 ppm) is followed within 5–15 s by a tracker-wide lock collapse across
ALL constellations; median relock ~60 s. Base rate far from writes: 1.8%
(mostly known USB churn). Mechanism is firmware-guaranteed: SGPIO stream
disable → Si5351 MS0/MS1 reprogram → PLL-A reset (~2 ms) → stream restart,
plus a forced same-freq retune that re-syncs the LO (`live_radio.rs` write
path; `radio.c`, `clock_gen.c:331`). `note_clock_step` only shifts Doppler
bookkeeping; it cannot save the loops through a physical hole.

Consequence already deployed: the discipline loop runs SHADOW by default
(`HACKRF_GNSS_ACTUATE=1` to actuate) — corrections are computed, logged,
published, never written.

## What remains unknown (this experiment answers)

1. Is the killer the **correction write** (Si5351/PLL-A path) or the
   **forced same-freq retune** (LO re-sync)? They always happen together
   today; nobody has separated them.
2. Does the collapse come with a measurable carrier-phase step, or is it
   pure dead-time?
3. Can a write survive at all if issued at a chosen boundary, with dead-time
   accounting (`note_gap`-style) instead of `note_clock_step`?

## Design

Maintenance window, tracker healthy, ≥2 mature WAAS GEO locks, shadow mode
active (the loop itself never writes). A script issues ONE controlled write
every 5 min while logging every 1 Hz channel field (lock_s, carrier_cycles,
phase_frac, code_phase, cn0_proxy) plus USB byte/drop counters, at 1 s
granularity, ±90 s around each write.

Write matrix (each cell ≥ 3 trials, order randomized):
- A: correction write ±0.01 ppm, NO retune (needs a host path that skips the
  tune — `hackrf_pro --clock-corr` alone vs the loop's write+retune pair)
- B: correction write ±0.01 ppm + same-freq retune (today's path)
- C: same as A/B at ±0.10 ppm
- D: retune alone (no correction change) — the LO-resync-only control
- E: no-op control epochs (no USB control traffic at all)

Per-write measurements: (a) sample-count continuity (bytes/s, drop counter),
(b) per-channel lock survival vs collapse and relock time, (c) carrier_cycles
residual vs the expected −step·f_c·dt signature (tests note_clock_step's
sign/magnitude where loops DO survive), (d) C/N0 watchdog timeline.

## Decision matrix

| outcome | conclusion | next step |
|---|---|---|
| A clean, B/C collapse | the retune's LO re-sync is the killer | split the firmware contract: correction steers the AFE/sample clock ONLY, never the LO (radio.c change), retune dropped from the loop |
| A also collapses, phase step bounded | PLL-A reset hole is the killer | capture-boundary writes + dead-time accounting in the tracker (note_gap_bytes), bounded-rate actuation |
| everything collapses with unbounded phase | no host-side mitigation | phase-continuous steering redesign (FPGA NCO / multisynth-only updates), the big firmware path |
| D alone collapses | even retunes are unsafe | discipline actuation stays shadow until the M4 GPSDO loop owns timing |

## Open question added 2026-08-25 (round-5 review): the 0.34 ppm restart step

Across the 17:13 UTC tracker restart the measured residual stepped by
0.34 ppm — almost exactly the cached correction value. Either the
pre-restart loop was double-counting the applied correction, or the
correction register is not reaching hardware the way the bookkeeping
assumes (consistent with agent-11's finding that nothing compares
requested==applied). The experiment above must add a measurement leg:
read the correction register back AND measure the actual tick rate
(`state.tick.json`) before/after a single write, to establish what a write
physically does. Until this is resolved, re-actuation is off the table
regardless of the continuity outcome.

### Round-9 update (2026-08-26, free measurements from the 0x469 flashes)

Two control observations landed for free during the flash windows:

1. A full board reset + reflash stepped the measured TCXO residual by
   **−0.0345 ppm with a slow thermal-recovery tail**; a plain tracker
   restart does NOT produce such a step. Treat "the board was reset" as a
   ~0.03 ppm-class disturbance event when comparing residuals across one.
2. Across that full reset the residual did NOT jump by the cached ±0.34 ppm
   — arguing the correction register was already at unity (nothing applied
   to lose). This REMOVES the leading "double-counted applied correction"
   explanation for the 0.34 ppm step and makes the read-back leg above the
   decisive remaining test: it costs one minute of any window that is
   already running the experiment.

## Safety

Shadow mode stays on except the scripted single writes; each write is
exactly the class the loop used to issue every ~2 min, so blast radius is
the known ~1 min relock per trial. The tracker is restarted afterwards only
if locks fail to recover within 5 min.
