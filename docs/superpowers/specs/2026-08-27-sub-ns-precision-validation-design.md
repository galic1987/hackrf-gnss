# Sub-ns Timing Precision on HackRF Pro + AA.250, Bodnar-Validated — Design

Date: 2026-08-27. Status: approved by user (2026-08-27 ~15:10 EDT).

## Goal and claim definition

Demonstrate **sub-nanosecond timing precision/stability** (not absolute accuracy)
of the station's GPS-derived receiver clock, using only HackRF Pro #2 + AA.250,
validated against the Leo Bodnar LBE-1421 GPSDO. Absolute accuracy vs GPS time is
explicitly out of scope for now (needs calibrated antenna phase center, precise
ephemeris, and the Bodnar's own ~ns-class PPS accuracy; also circular because the
Bodnar is GPS-disciplined).

Precision claim gates (all over ≥ 1 h of continuous data):
- RMS of detrended clock-bias series b(t) < 1 ns.
- Allan deviation < 1 ns for τ = 10–1000 s.
- WAAS GEO carrier-phase cross-check consistent (structure in b(t) is clock, not
  channel noise).

## Current baseline (verified 2026-08-27)

- Pro#1 (977c…): DEAD, hardware power-input fault (J1/Q4 path; no LED on dumb
  charger, no DFU boot-ROM). Awaiting repair/RMA. All TDC/custom gateware work is
  blocked on it or on a deliberate Pro#2 flash decision.
- Pro#2 (6450…): production tracker (16 Msps @ 1568.25 MHz; GPS L1 + B1I + E1 +
  SBAS), CLKIN = Bodnar out2 10 MHz (verified "clock signal detected"), stock
  2026.01.3 firmware, discipline loop in SHADOW (HACKRF_GNSS_ACTUATE unset;
  hardware never written). Residual ~+0.007 ppm in deadband.
- One (922c…): CLKIN = Bodnar out1 (reconfigured to 10 MHz). L-band unproven:
  5 acquisition tests (2 antennas incl. unpowered active patch, 8/10 Msps, max
  gain, ±40 kHz Doppler) all zero; FM/VHF verified working. Feeding it a clean
  pre-amplified signal via splitter is the decisive L-band test.
- Antennas (DO NOT MOVE — mechanical freeze once measuring): AA.250 — 3 cm —
  Bodnar puck — 3 cm — One's patch, south-facing line.
- Bodnar: out2 = 10 MHz → Pro#2 P1; out1 = 10 MHz → One (PPS currently NOT
  available anywhere; out1 must return to PPS for the TDC leg).

## Leg 1 — solo clock-bias series (starts now, no new hardware)

The defensible observable. Today's `carrier_cycles` is integrated replica-NCO
advance (no sample epoch, no ambiguity state, survives gaps) — not usable.

Components:
1. **Tracker phase integrity** (`examples/live_radio.rs`, code only while tracker
   is live): per-ms prompt correlator phase atan2(Q,I) with exact sample epoch,
   per-channel slip/generation counters, hard phase invalidation on every input
   gap (bundles the already-committed 664514a gap-slip work into one deployed
   generation).
2. **Clock-bias solver** (`examples/live_fix.rs`): Hatch carrier-smoothing of
   pseudoranges (~100 s window, reset on slip/generation change) feeding the
   existing PVT solve. Publishes `observations/clock_bias.jsonl`:
   `{epoch, b_ns, sigma_ns, n_sats, gdop, slips, generation}`.
3. **Analyzer** (`scripts/clock_bias_analyzer.py`): detrend b(t) (Bodnar's slow
   GPS-steering is common-mode), then RMS + ADEV(τ=1–1000 s). Physics: over these
   τ the Bodnar OCXO is ~1e-11, so residual structure in b(t) = our measurement
   noise = the precision number.
4. **GEO cross-check**: reuse phase_drift_producer's WAAS GEO carrier series
   (P0b Doppler/Sagnac-corrected) as an independent second clock measurement.

Deployment: all code written with tracker live; **build happens only in a
tracker-down maintenance window** (build law), shared with the Leg 2 window.

## Leg 1b — common-clock differential (activates when splitter arrives)

Requires: 2-way GPS L1 active splitter, DC-pass on the Pro#2 port (user sourcing).
AA.250 → splitter → Pro#2 (powers patch) + One. Same phase center: geometry and
multipath cancel exactly.

- One runs its own tracker instance (8-bit, L1-centered config path, no rebuild).
- `scripts/single_difference.py`: ΔΦ per common PRN per epoch from both trackers'
  channel states. Single difference cancels receiver clock AND satellite clock by
  construction (both sample clocks driven by the same 10 MHz). Residual =
  constant differential hardware group delay (calibrates the term that normally
  blocks absolute claims) + coherence + noise.
- Common-mode rotation across ALL satellites at once = relative clock wander
  (directly validates the Bodnar distribution); per-satellite structure =
  geometry/multipath. This is the coherence test — no TDC needed.
- Verdict: differential noise floor must agree with Leg 1's b(t) — the
  validation triangle (Pro b(t), One b(t), ΔΦ).
- Also settles the One's L-band health definitively.
- **GLONASS**: surface band_producer's G2 snapshot rows on the sync dashboard
  (live G1 tracking out of scope — outside the 16 MHz window).

## Leg 2 — TDC PPS validation (optional, user-triggered window)

- Prereq: out1 back to PPS (One pauses or moves to Pro#2 CLKOUT for the window).
- Flash Pro#2 with the 0x36 debug build (pin-swap fix + TRIGGER.IN diagnostic
  register; already built, blob sha 332225ae…, factory 2026.01.3 image staged for
  rollback, DFU recovery rehearsed 2026-08-27).
- First read 0x36 bit0: 1 Hz toggling = PPS reaches FPGA pin 47; static =
  bench/board split (loopback test). Then 0x30=0, poll 0x31, connectivity gates
  (1 toggle/PPS, no dup/miss, thermometer-ish; all-ones = phase outside 48-tap
  coverage → cable-delay sweep), then 3,600-pulse jitter run (1 h).
- Do NOT use `--tdc-read` for PPS (forces ring-osc selftest 0x30=0x03).

## Operational rules (hard)

- **USB bus contention**: Pro#2 + One share one USB2 hub. No captures on the
  second radio while the tracker streams — a 10 Msps One capture on 2026-08-27
  13:21 coincided with the live_radio death and a 108-min zero-channel gap.
- **Build law**: no cargo build / full test suite while the tracker is live.
- **Shadow invariant**: HACKRF_GNSS_ACTUATE stays unset; clock correction is
  observe-only on the Bodnar-referenced radio.
- **Antenna freeze**: no re-seating once Leg 1b starts; every move changes the
  constant being calibrated.
- band_producer runs under the ownership guard; Pro-band snapshot rows stay STALE
  while the tracker owns the radio.

## Error handling

- Flash failure → DFU recovery (rehearsed; factory images at
  /tmp/hackrf-2026.01.3/firmware-bin/hackrf_pro_usb.{dfu,bin}).
- Tracker restart pattern (only): pattern-broken pkill, sleep 3,
  `hackrf_spiflash -d <pro2> -R`, sleep 6, relaunch tracker_producer.py.
  Never pkill -9.
- band_producer respawner is unknown; `pgrep -fl band_producer` before any radio
  work; SIGSTOP it for radio work, SIGCONT after.
- TDC still zero freezes after pin swap → 0x36 splits bench-vs-board; production
  restored regardless.

## Testing

- Unit tests: Hatch filter convergence + slip-reset; gap-invalidation counter
  behavior (crate test suite, run in the maintenance window).
- Replay validation: new acquisition/phase path against recorded known-good
  captures (e.g. /tmp/band_waas.f32 lineage) — must reproduce metric ≥ 4.9 PRNs.
- Analyzer: synthetic clock series with known ADEV → recovered within tolerance.
- Leg 2: connectivity gates as above; jitter distribution vs 499 ps tap budget.

## Scheduling

- Now → 15:53: Leg 1 code (tracker live, edits only). 15:53 EDT: GDOP cron
  re-measure (existing history; tracker restarted 15:10 after the 13:21 gap).
- After 15:53: one maintenance window — build + deploy Leg 1 generation, restart
  tracker, begin ≥ 1 h b(t) collection.
- Splitter arrival: Leg 1b activation. TDC window: user-triggered.
