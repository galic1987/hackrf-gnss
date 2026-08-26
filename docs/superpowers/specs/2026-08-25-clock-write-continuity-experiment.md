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

1. A full board reset + reflash stepped the measured TCXO residual by
   **−0.0345 ppm with a slow thermal-recovery tail**; a plain tracker
   restart does NOT produce such a step. Treat "the board was reset" as a
   ~0.03 ppm-class disturbance event when comparing residuals across one.

### Round-10b CORRECTION (2026-08-26, supersedes an earlier wrong inference)

An earlier revision of this section claimed the 08:17 flash showed NO
±0.34 ppm residual jump and argued the register was already unity,
"removing" the double-counting explanation. That reading was backwards.
The +0.3398 ppm step DID appear — at the 07:58 restart when 587da63
(phantom add-back removal in the phase-drift producer) went live: the
displayed consensus moved −0.7228 → −0.3829 ppm, i.e. up by exactly the
cached correction that had been illegitimately added. That SUPPORTS the
double-counted-add-back explanation for the original 0.34 ppm puzzle: the
restart-era step was a bookkeeping artifact, visible only while the phantom
add-back existed. The read-back leg remains the decisive test for what the
hardware physically holds — the bookkeeping artifact is now believed dead,
not the question of what a write physically does.

### Round-11 erratum (2026-08-26), CORRECTED same day (round-14)

~~The vendor crate has NO radio-register read path~~ — too narrow a lens:
the RUST vendor crate the tracker drives has no read path, but the C host
stack does: `hackrf_radio_read_register(BANK_APPLIED, …)` is a real vendor
request (b4041dd5-era, present on the flashed 0x469-v2 firmware), exposed
as `hackrf_debug -d <serial> --radio -n 23 -r` (bank 0 = APPLIED by
default). The getter is DEVICE-side bookkeeping — the firmware's applied
bank, i.e. what it believes it wrote, not a physical Si5351 readback — and
on a fresh boot the register reads RADIO_UNSET (reported as 0 ppm), so a
read RIGHT NOW settles the six-round-old unity question: 0/UNSET proves
the applied bank never received a correction this boot. The full leg runs
in a tracker-down window with the user present: read, one scripted write
(HACKRF_GNSS_ACTUATE=1), read back, and state.tick.json before/after —
that establishes what a write physically does to the tick rate. The tick
half remains runnable any time. Re-actuation stays off the table until the
leg has run.

## Window results (2026-08-26 ~21:00-21:30 UTC, user present, Bodnar LBE-1421 online)

Register-semantics chain MEASURED end to end (hackrf_debug --radio -n 23 -r,
bank 0 = APPLIED):
- fresh power-on: 0xFFFFFFFFFFFFFFFF (RADIO_UNSET; the getter maps it to 0 ppm)
- after boot init: 0x8000000000000000 (FRAC_ONE = unity)
- after `--clock-corr 0.1`: requested == applied == FRAC_ONE + 0.1 ppm exactly
- after `hackrf_spiflash -R`: back to 0xFFFF... (UNSET)
The six-round 0.34 ppm puzzle is CLOSED on the register side: the applied
bank sat at unity while the software spoke of -0.3378 ppm — a phantom
cached-intent baseline (the 07:58 +0.3398 ppm step was the add-back removal,
as the double-count evidence said). hackrf_pro's own printout says
"(applied: 0.00 ppm)" regardless — cosmetic tool text; the register readback
is the truth. CAVEAT on the write leg: pkill self-matched the wrapper
(`pkill -f tracker_producer.py` matches the invoking shell's own cmdline) so
live_radio kept streaming — the +0.1 ppm write landed on a LIVE tracker.
The register semantics are unaffected (firmware bookkeeping), but the
tracker-quiescent purity the full matrix wants was not achieved; the
lock-collapse/phase-step cells remain for the dedicated bench session.
Future windows: break the pattern up (`'tracker''_producer'`) so pkill
cannot self-match.

Clock chain activated: Bodnar 10 MHz -> Pro#1 CLKIN (P1) -> CLKOUT ->
Pro#2 CLKIN (switched via a forced 1-s RX; dormant radios never switch) ->
CLKOUT -> One CLKIN. Post-restart: WAAS-measured residual collapsed from
the -0.34 ppm TCXO era to -0.0011 ppm; the One's ATSC pilot offset went
+1.72 ppm -> -0.052 ppm with SNR 16 -> 55 dB; 15/15 tracker locks in 4 min.
The site anchor is now Bodnar-surveyed (site.json: 39.0029556, -77.6051478,
77.1 m ellipsoidal, ~2 m class, survey-in).

## Safety

Shadow mode stays on except the scripted single writes; each write is
exactly the class the loop used to issue every ~2 min, so blast radius is
the known ~1 min relock per trial. The tracker is restarted afterwards only
if locks fail to recover within 5 min.
