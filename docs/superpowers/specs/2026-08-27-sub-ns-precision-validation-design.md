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
  available anywhere; out1 must return to PPS for the TDC leg). Star topology
  verified 2026-08-28 (ATSC offset −0.0536 ppm matches the GPSDO era).
  **Port budget — 2 outputs, 3 wanted signals (10 MHz ×2 + PPS):** exactly
  one escape per experiment: (a) 10 MHz distribution amp/splitter on out2
  feeding both radios, out1 restored to PPS; (b) the Pro's P22 alternate
  CLKIN path to free a front-panel port; (c) pause the One for the window —
  out1 → PPS → Pro#2 P2 (the One free-runs on its TCXO; its downstream
  attestation for that window is void). Pro#2 P2 is free in the star; one
  trigger master per experiment, never mid-collection.

## Leg 1 — solo clock-bias series (starts now, no new hardware)

The defensible observable. Today's `carrier_cycles` is integrated replica-NCO
advance (no sample epoch, no ambiguity state, survives gaps) — not usable.

Components:
1. **Tracker phase integrity** (`examples/live_radio.rs`, code only while tracker
   is live): per-ms prompt correlator phase atan2(Q,I) with exact sample epoch,
   per-channel slip/generation counters, hard phase invalidation on every input
   gap (bundles the already-committed 664514a gap-slip work into one deployed
   generation). **STATUS: still pending — and the 2026-08-27 first-run poisoning
   proved it load-bearing**: the published `slip` flag never fired across live
   relocks (lock_s 5188→0→relock cycles), and a struggling channel republished
   frozen `rho_m` verbatim for 30–90 s. Until this lands, downstream consumers
   must defend themselves (see component 2's defenses).
2. **Clock-bias solver** (`examples/clock_bias.rs`, built 2026-08-27/28): Hatch
   carrier-smoothing of GPS pseudoranges (100 s window) feeding the existing PVT
   solve (weighted + `solve_unweighted` paired A/B per epoch), n≥5 redundancy
   gate. Defenses after the first-run poisoning: smoother reset on lock_s
   regression OR >500 m innovation (the `slip` flag alone is insufficient),
   frozen-rho skip with 3-freeze eviction, 10 s per-sat freshness gate; every
   reset epoch counts as `slips` and is excluded from the claim. Publishes
   `observations/clock_bias.jsonl`: `{epoch, clock_ns, clock_ns_uw, tdop,
   n_sat, gdop, residual_rms_m, residual_rms_m_uw, n_smoothed, slips, source}`.
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
multipath cancel TO FIRST ORDER — splitter, cable, front-end and post-split
reflections remain as residual differential terms.

- One runs its own tracker instance (8-bit, L1-centered config path, no rebuild).
- `scripts/single_difference.py`: ΔΦ per common PRN per epoch from both trackers'
  channel states. The single difference cancels the SATELLITE clock (and, to
  first order, geometry/multipath); it RETAINS (a) the inter-receiver epoch/
  clock offset — a shared 10 MHz syntonizes frequency, it does not align clock
  phase or sample zero — (b) the differential integer ambiguities, and (c) the
  constant differential cable/analog group delay. The constant terms drop out
  of the time variation, which is what makes the residual useful: differential
  hardware group delay (calibrates the term that normally blocks absolute
  claims) + coherence + noise. Where receiver-clock cancellation itself is
  required, use satellite DOUBLE differences, not single ones.
- Common-mode rotation across ALL satellites at once = relative clock wander
  (directly validates the Bodnar distribution); per-satellite structure =
  geometry/multipath. This is the coherence test — no TDC needed.
- Verdict: differential noise floor must agree with Leg 1's b(t) — the
  validation triangle (Pro b(t), One b(t), ΔΦ).
- Also settles the One's L-band health definitively.
- **GLONASS**: surface band_producer's G2 snapshot rows on the sync dashboard
  (live G1 tracking out of scope — outside the 16 MHz window).

## Leg 2 — TDC PPS validation (optional, user-triggered window)

- Prereq: ONE coherent port-budget choice from AGENTS.md ("Port budget") —
  e.g. (a) 10 MHz distribution amp on out2 feeding both radios, out1
  restored to PPS; or (c) pause the One, out1 → PPS → Pro#2 P2. The old
  phrasing "One pauses OR MOVES TO Pro#2 CLKOUT" was electrically
  inconsistent (round-14): Pro#2 P2 cannot be CLKOUT-to-the-One and the
  TDC's trigger/PPS input at the same time. And per the AGENTS.md
  labeling hold: NO output is reconfigured to PPS until both cable ends
  are photographed and labeled.
- **SUPERSEDED 2026-08-28 — DO NOT FLASH the 0x36 debug build (blob sha
  332225ae…).** Its pin swap is backwards: official gateware maps FPGA clock to
  pin 47 and TRIGGER.IN to pin 48; this build swaps them, so the 0x36 register
  labels both signals backwards and the TDC would see the 40 MHz clock as its
  trigger. The blob and every firmware image embedding it (including the former
  /private/tmp/hackrf-fw tree, moved 2026-08-28) are quarantined in
  mac-archive/hackrf/quarantine-0xE91-20260828/. Any replacement diagnostic
  image MUST retain the official mapping (clock 47, trigger 48) and be built
  from a clean, attested tree.
- Corrected procedure once a valid diagnostic image exists: first read 0x36
  bit0: 1 Hz toggling = PPS reaches the FPGA trigger pin; static = bench/board
  split (loopback test). Then 0x30=0, poll 0x31, connectivity gates (1
  toggle/PPS, no dup/miss, thermometer-ish; all-ones = phase outside 48-tap
  coverage → cable-delay sweep), then 3,600-pulse jitter run (1 h).
- Do NOT use `--tdc-read` for PPS (forces ring-osc selftest 0x30=0x03).

## Phase 3 — triangle interferometry (vision, NOT scheduled)

User's goal: three antennas arranged around the space (~180° triangle closure),
absolute-time measurements on Pro and One simultaneously, testing clock
coherence and closing the array geometry — a spatial analog of the validation
triangle in Leg 1b.

- **Prereqs (hard):** Leg 1b differential completed (group-delay calibration is
  the term that otherwise blocks cross-radio absolute phase); Pro#1 back from
  repair (third coherent RF path) OR sequential antenna moves on Pro#2 with
  bodnar-referenced re-sync between positions; Bodnar out1 available as PPS
  again (Leg 2 window conflict — one at a time).
- **Geometry note:** the 2026-08-26 linear array analysis (3 × 3 cm S–N,
  max baseline 6 cm < λ/2 ≈ 9.5 cm ⇒ zero integer ambiguity) does NOT carry
  over to a triangle of useful aperture: baselines > λ/2 reintroduce cycle
  ambiguity, so Phase 3 needs either LAMBDA-class integer resolution or
  ambiguity-free initialization from a known <λ/2 start configuration that
  is then expanded. Decide at planning time, not during the experiment.
- **Observables:** per-baseline single-difference carrier phase per common PRN
  (both radios Bodnar-referenced ⇒ frequency-syntonized; the residual epoch/
  phase offset is measured, not assumed zero), triangle closure residual ΣΔΦ
  around the loop (must close to noise), and absolute clock-bias b(t) per
  radio from the Leg 1 pipeline.
- **Claim gates (draft):** closure residual consistent with the Leg 1 noise
  floor; inter-radio b(t) agreement within the Leg 1b group-delay bound;
  antenna-position solve recovers surveyed baselines to < 5 mm.

## Operational rules (hard)

- **USB bus contention**: Pro#2 + One share one USB2 hub. No captures on the
  second radio while the tracker streams — a 10 Msps One capture on 2026-08-27
  13:21 coincided with the live_radio death and a 108-min zero-channel gap.
- **Build law**: no cargo build / full test suite while the tracker is live.
- **Shadow invariant**: HACKRF_GNSS_ACTUATE stays unset; clock correction is
  observe-only on the Bodnar-referenced radio.
- **Antenna freeze**: no re-seating once Leg 1b starts; every move changes the
  constant being calibrated.
- **Bias-tee latch**: never hot-plug antennas on live ports. The bias-tee
  overcurrent protection latches OFF on hot-plug transients and survives USB
  `-R` reset — the 2026-08-27 13:21–18:44 outage. After ANY antenna event,
  FULL power-cycle of the radio (unplug/replug), then verify LNA noise.
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
