# hackrf_gnss — live GNSS/SDR timing station

Rust crate + Python producers + panel server, running 24/7 against two
HackRFs. Read this before touching anything that talks to the radios.

## Hardware ownership (the law)

- **HackRF Pro #2** `…645061de252d6613` — owned 24/7 by `examples/live_radio`
  (spawned by `scripts/tracker_producer.py`). Became the production radio
  2026-08-27 when Pro#1 died. `hackrf_open` is EXCLUSIVE:
  no other process can open the Pro while the tracker runs. External
  pollers/captures targeting the Pro will fail or, worse, inject USB
  contention that overflows the tracker's stream queue (the 2026-08-24
  churn: 115 ms FIFO overflows → full channel realigns).
- **HackRF One** `…922c63dc21748847` — owned by `scripts/phase_producer.py`
  (ATSC ch35 carrier-phase track). CLKIN fed by Bodnar OUT2 (10 MHz splitter)
  in the Split Star topology — not by any HackRF; no CLKOUT assertion
  on the Pro is needed for the One's reference. Current classification
  (2026-09-01): **clock-detected, ClearStream ATSC operational,
  GPS-unproven**. The south-facing ClearStream feed restored a real ch35
  pilot lock; current truth is the producer's fresh `state.phase.json`
  heartbeat, never the historical dark-era notes. It has never run a GPS
  tracker, so every L-band claim for it remains unproven until the
  AA.250-splitter test (one DC-pass leg, DC-block the One leg, zero-gain
  baseline, separate USB controller) demonstrates real acquisitions. That
  asymmetric test can establish L-band health only; it cannot calibrate
  receiver delay or support a claim-grade RF-leg ABBA result.
- **HackRF Pro #1** `…977c64de2b557213` — **DEAD 2026-08-27** (no power on
  any cable/charger incl. dumb charger and A-to-C, no DFU boot-ROM
  enumeration — J1/Q4 input-path hardware fault, repair/RMA pending). Do
  NOT target this serial in any command. Legacy entry points that carried it
  as a default are quarantined rather than redirected onto the production
  radio. When it returns from repair it re-enters as an independent bench radio.
  Under the deployed Split Star it is not upstream of the One, so resetting or
  loading it cannot break the One's clock path. Verify each repaired radio's
  own CLKIN selection after its own stream start; do not resurrect the old
  Pro→Pro→One cascade or use a different radio's reset as clock attestation.

## Clock & PPS topology (since 2026-08-29; GPSDO-referenced SPLIT STAR)

**Split Star**:
- **10 MHz**: Bodnar LBE-1421 OUT2 (10 MHz GPSDO output; output-lock state is not exposed by the current probe) → SMA Power Splitter → Pro#2 `…6450…` P1 CLKIN & HackRF One `…922c…` CLKIN (matched cables).
- **1PPS**: Bodnar OUT1 (GPSDO 1PPS; output phase/holdover state is not exposed by the current probe) → SMA Power Splitter → Pro#2 P28 pin 16 (TRIGGER.IN) & HackRF One P28 pin 16 (TRIGGER.IN) (matched cables).

Both radios hang directly off the GPSDO's split 10 MHz and 1PPS outputs.
The shared 10 MHz provides a common frequency reference; the matched 1PPS
cables establish physical distribution only. They do **not** by themselves
prove sample-zero alignment, trigger latency, receiver delay, or RF-phase
synchronization. Hardware evidence currently establishes:
`hackrf_clock -d …922c… -i` reads "clock signal detected" at the One's
CLKIN (checked 2026-08-28 with the One free) — NOT the ATSC row: the
phase_history ppm "eras" (−3 / +0.53 / −1.7 ppm) are noise-lock artifacts
(78.1% of history rows contaminated; 2026-08-28 scan after the SNR-floor
work — a 2026-08-29 re-scan with other classifiers could not reproduce the
78.1%, see phase_history.QUARANTINE-README.txt; the quarantine stands on
the verified unaudited pre-tombstone lock basis). The history file itself
is QUARANTINED since 2026-08-29 10:35 EDT (renamed
phase_history.jsonl.quarantine-noiselock-20260829). Those pre-fix rows remain
invalid. On 2026-09-01 the south-facing ClearStream feed restored a
pilot-class signal. Commit d1bc6c0 records the key causal evidence: enabling
antenna-port power raised amplitude from ~0.05 to >8, restored `lock:true`,
and produced ~1 mm reported sigma. The currently running child therefore uses
`-p 1`; preserve it. The antenna itself may be passive, but the complete coax
path is electrically unresolved—identify any inline powered amp, injector,
splitter DC-pass, and termination before claiming what consumes the DC or
changing power. Future starts fail closed unless the reviewed
`clearstream_bias_on_20260901` profile and matching acknowledgement are both
explicit; the single `HACKRF_ANT_POWER` override is retired. The producer logs
and publishes requested settings, not hardware readback. Do not restart merely
to adopt source changes; use a controlled One maintenance window. Only after
the inline path review, an exact graceful stop, and proof that the old
`hackrf_transfer` is gone, the future launch form is:

```sh
cd "/Volumes/Radiator 8TB/gnss/hackrf_gnss"
HACKRF_ANTENNA_PROFILE=clearstream_bias_on_20260901 \
HACKRF_RF_PROFILE_ACK=clearstream_bias_on_20260901 \
  nohup python3 scripts/phase_producer.py >> /tmp/phase_producer.log 2>&1 &
```

Both profile variables are mandatory; do not copy the legacy
`HACKRF_ANT_POWER=1` launch. The reviewed profile fixes amp off, LNA/VGA
40/44, and bias tee requested on; changing any of those requires a newly named
and reviewed profile. With
LNA/VGA 40/44 the producer
publishes a fresh post-quarantine history and reports
`lock:false` honestly if the pilot later disappears. A live lock establishes
ATSC carrier tracking, not GPS capability or physical CLKIN topology.
Clock switches happen ONLY at RX/TX
begin (per radio) — connecting or reconfiguring a link does nothing until
that radio's next stream start. The soft drift-lock verifier
(series_producer, state.series.json `clkin_soft_verified`) returns True
under BOTH the old cascade and the star (2026-08-28 audit) — it cannot
detect a rewire; treat it as a liveness check only, never as topology
proof. NOTE: the Pro's local clock-correction register acts on PLL-A
(sample clocks) ONLY — CLKOUT rides PLL-B (si5351c.c Praline map): local
correction never propagates off-radio, and in GPSDO-referenced operation
the shadow loop's intent is ~0 by construction.

**Clock & 1PPS Distribution (Deployed 2026-08-29):**
- **10 MHz Syntonization:** Bodnar OUT2 (10 MHz) \u2192 SMA Power Splitter \u2192 Pro P1 (CLKIN) & One P1 (CLKIN) over matched cables.
- **1PPS Distribution:** Bodnar OUT1 (1PPS) \u2192 SMA Power Splitter \u2192 Pro P28.16 (TRIGGER.IN) & One P28.16 (TRIGGER.IN) over matched cables; capture synchronization remains uncalibrated.
- **Pro P2 SMA:** Physically free and disabled (`set_clkout_enable(false)` in `live_radio`).
- **Trigger Inputs:** Both Pro and One trigger via internal header **P28 pin 16 (TRIGGER.IN)**.

**Trigger input is P28 pin 16 (2026-08-29 correction, user-verified against
the official expansion-interface pinout):** on BOTH the Pro and the One,
P28 pin 16 = TRIGGER.IN and pin 15 = TRIGGER.OUT — the Pro *also* offers
trigger on its configurable clock SMAs, but the station's PPS path is the
header pin. An earlier revision of this file claimed the Pro was SMA-only
for trigger; that was wrong. First external TDC PPS capture succeeded via
P28.16 on 2026-08-29 (1 valid-toggle per pulse, thermometer codes).
- Radio work (flashes, captures) uses the atomic protocol in
  `scripts/pro_lease.py`; `pgrep` and a plain file-exists check are not
  ownership. Normal clients hold `pro.radio.lock.d` for the complete device
  lifetime and check `maintenance.lock` before and after atomic acquisition.
  `tracker_producer` passes its unguessable owner token to `live_radio`; the
  child verifies the fixed owner JSON, wrapper parent PID, role, exact Pro #2
  serial, and non-maintenance state before opening USB, then verifies the
  immutable board ID/serial before its first configuration write. Direct
  `live_radio` launches and alternate serials fail closed. The private exec
  launcher unblocks SIGINT/SIGTERM inherited across the wrapper's protected
  spawn window, so graceful termination remains possible. Locks are never
  auto-reclaimed from PID liveness.

  Deployment runbook (pre-stage everything first):

  1. `python3 scripts/pro_lease.py gate --serial 0000000000000000645061de252d6613 --token-file /tmp/pro-maint.token`
     creates the gate atomically. If deploying the lease code for the first
     time, also pause the already-running legacy `band_producer` because an
     old Python process has not loaded the new guard.
  2. Gracefully stop the exact `tracker_producer`/`live_radio` processes.
     The gate is already up, so a lease-aware band snapshot cannot take the
     just-freed Pro.
  3. `python3 scripts/pro_lease.py acquire --token-file /tmp/pro-maint.token --wait-seconds 30`
     must succeed before any radio
     command. Then reset the Pro immediately and perform only the staged work.
  4. Restore the verified production image/config. Run `release` with the
     same token file, start the tracker, use `pro_lease.py status` to verify
     the tracker owns the lease, prove stream health, and resume
     `band_producer` last.

  If work aborts before `acquire`, `cancel` with the same token. If any command
  reports malformed/stale state, stop: never `rm -rf` or infer ownership from
  a dead PID. Inspect the exact JSON owner and radio state in a maintenance
  window; all ambiguous states deliberately remain gated.
- **Tracker restarts are only reliable after a board reset** (2026-08-24,
  three trials): SIGTERM or SIGKILL of `live_radio` can leave the Pro's
  USB streaming state wedged — the next `live_radio` then seeds deaf
  ("seed done — 0 candidates" forever). Do not use the retired raw
  `pkill`/`hackrf_spiflash`/`nohup` restart sequence: it bypasses atomic
  ownership and can race another producer. Follow the token-owned deployment
  runbook above: create the gate, stop gracefully, acquire the maintenance
  radio lease, reset immediately, restore/verify production state, release,
  then start and health-check the tracker. Never `pkill -9` `live_radio`.
  One-time transition caveat: an already-running pre-launcher child may have
  inherited blocked SIGINT/SIGTERM. Try the wrapper shutdown first; if the
  exact child PID remains, use only the signed runbook's bounded exact-PID
  SIGHUP fallback, prove exit, acquire the lease, and reset immediately. Do
  not use a name pattern or infer a clean USB state from process exit.
  `tracker_producer` therefore never auto-reopens after unexpected
  `live_radio` EOF/exit. While it still owns the lease it creates
  `observations/tracker-reset-required.maintenance.token`, raises the
  maintenance gate, releases only after the child is proven stopped, and
  exits 78. Use that exact token with `pro_lease.py acquire`, reset/restore,
  then `release`; do not merely relaunch the tracker.
  **The reset must IMMEDIATELY follow the kill — before ANY build/test**
  (2026-08-29 incident): the 11:07 window deferred the reset until after
  cargo build+test; the wedged Pro deepened from empty-serial to a full
  bus disconnect ([Removed] @ 0x100000, 11:23) and no host-side recovery
  (serial-addressed reset, unaddressed reset, libusb reset_device) could
  reach it — only a physical replug or spontaneous re-enumeration can.
  Correct deployment order: token-owned maintenance gate → pause competing
  producers → graceful exact-process stop → acquire the maintenance radio
  lease → **board reset immediately** → pre-staged flash/probes →
  verified slot restoration → token-owned lease release →
  start tracker → health soak → resume producers.
  Never remove either lock object as a liveness shortcut. The ONLY sanctioned
  stale-lease recovery is the pro_lease.py docstring transaction: raise the
  maintenance gate, PROVE the recorded owner PID is dead, then remove the exact
  stale object inside that window. The legacy recovery watcher is retired;
  recovery is manual and fail-closed.
- **Host build load kills the tracker** (2026-08-25, measured live): cargo/
  nextpnr stalls >115 ms overflow the ~190 ms USB transfer queue → `big
  gap` → full channel realign. NEVER run cargo builds/tests while
  tracker_producer runs. Build first, then restart the tracker; keep the
  host quiet while it tracks.
  - Refinement (2026-08-26, measured both ways): `nice -n 19` ALONE is not
    sufficient — the 09:45:38 realign happened under a nice-19 capped
    child. What actually worked: niced AND thread-capped
    (RAYON_NUM_THREADS=4) short jobs, and no hot loops (a `continue` that
    skipped the producer's sleep once spun acq children back-to-back —
    fixed d1c8490). ERRATA: the realign timestamp cited in d1c8490's
    commit and an earlier revision of this note (09:45:38) was wrong —
    the real event was 09:12:57 (tracker log), under the hot-looping
    niced+capped children. Full `cargo build`/`cargo test` still belong to
    tracker-down windows. `nice -n 19 cargo check`/targeted small test
    runs are tolerated; watch the tracker log for realigns after each.
    The real fix is negative-nice for live_radio (needs user sudo).

## Producers and state files (merge architecture)

Each producer writes ONLY `observations/state.<name>.json` (tmp file +
os.replace; never read-modify-write shared state). The server
(`src/main.rs --mode serve --port 8090`, serves `observations/sync.html`)
merges at read time: legacy `sync_state.json` first, then `state.*.json`
in filename order; `sources` merge by band, `clock` shallow-merges.
**Every file expires by mtime vs its `ttl_s`** (default 20 min) — a dead
producer's values disappear; they never outrank live ones (the tombstone
class). Producers with nothing to report must still heartbeat their file.

| producer | file | notes |
|---|---|---|
| tracker_producer + live_radio | state.tracker.json | 1 Hz channels, discipline estimator (in-process; **enforced SHADOW-only** — every correction write was followed by a tracker-wide collapse, the production write call and both direct/streaming Rust transport APIs are removed, and `HACKRF_GNSS_ACTUATE` is rejected before radio open; unity is an expected reset state, explicitly not register readback), tick counter reads |
| phase_producer | state.phase.json | 60 Hz carrier phase; heartbeats `lock:false` when dark; re-acquires after 60 s dark |
| series_producer | state.series.json | 30 s; rolling 1-h band series, consensus, spoof z-alerts (sigma floor 0.05 ppm); CLKIN soft-verify (ATSC−WAAS drift-lock over a 30-min paired diff, fail-closed null) as the ATSC-voter fallback gate while the hardware probe can't open the One |
| band_producer | state.band.json | snapshot rotation — shares the atomic Pro lease; skips while tracker owns it or maintenance is gated, so rows age while file heartbeats |
| gpsdo_probe | state.gpsdo.json | 5 s; Bodnar LBE-1421 NMEA over USB CDC (`/dev/cu.usbmodem*`): `nmea_fix_valid`, fix quality, n_sat, HDOP, GSV SNR, TTL 30 s. A valid fresh GGA is navigation-receiver health only; it does **not** attest 10 MHz lock, PPS phase, UTC offset, or holdover. A dark/invalid probe is an operational warning, while a valid probe is never sufficient evidence for a clock claim |
| position_producer | state.position.json | runs examples/live_fix every 5 min; refreshes BRDC from BKG HOURLY (the ±4 h ephemeris fit window makes a 6-h refresh guarantee a modeled-sky blind gap). Publication law (round-13, single gate `publish_position`): `position` is TRUSTED-only (redundant AND plausible); exact-but-plausible solves publish as `position_candidate`, plausibility-failing ones as `position_diagnostic`, and both untrusted classes preserve the last trusted `position` (honestly aging). Trust fields: `geometry_redundant`, `plausibility_pass`, `trusted_for_history` |
| sky_producer | state.sky.json | 30 s; az/el from BRDC+live eph vs tracker: GPS/BDS/Galileo Kepler (Galileo SIS-ICD constants, GST≈GPST) + GLONASS PZ-90 state-vector RK4 — GLONASS is `cls:"predicted"` (G1 1602 MHz FDMA outside the L1 tune: sky map + trails only, never the tracked/absent/expected coverage counts or the learned mask); tracked/absent/unexpected; learns 5°×5° sky_mask.json (schema 2: provenance block — site identity, rig string from the tracker's GPS L1 source, created/learn-start epochs, pass counts; learning GATED on tracker health — fresh within ttl, ≥ MASK_MIN_LOCKED=8 locked, ≥80% of lock ages ≥ the 30 s window — gated passes classify but teach nothing, so receiver outages/realigns never paint the mask; schema- or site/rig-mismatched masks on load are moved to sky_mask.json.quarantine-* and learning restarts empty); appends sky_history.jsonl; per-sat alt_km/speed_mps/track_deg + 30-min recent_trails; re-reads **observations/site.json every pass** (the canonical anchor — no hardcoded coordinates anywhere; a missing anchor is an error heartbeat, never a guess) |

## Consensus semantics (hard-won)

`resid = raw − corr` (discipline march, verified live). Rows measured on
the Pro AFTER the hardware clock-correction register must add the
correction back before voting (tracker_producer does). Consensus voters
floor sigma at 0.05 ppm — inter-path systematics exceed any instrument's
short-term precision; tight sigmas make the spoof alarm cry wolf.

## Telemetry archive

Durable historical archive for later modeling (thermal drift, sky-visibility
mask, dropout-cause suggestion). Schema/layout/query doc:
`observations/archive/README.md`.

- `scripts/telemetry_collector.py` — always-on (nohup, log
  `/tmp/telemetry_collector.log`); every 10 s merges the live
  `state.*.json` READ-ONLY (same ttl/tombstone rule as the server) into one
  JSONL line appended to `observations/telemetry_log.jsonl` (its own file).
  Must never die on a missing/corrupt state file.
- `scripts/archive_roller.py` — rolls the JSONL histories
  (band_drift, phase, clock/fused loop logs, telemetry_log, sky_history)
  into `observations/archive/YYYY-MM-DD/<stream>.parquet` (streams: satellite,
  clock_drift, discipline, phase, loop_log, telemetry, presence, tdc
  reserved). Atomic tmp+replace writes; idempotent re-runs (natural-key
  dedupe against existing partitions; `_roller_state.json` offsets are only
  an incremental-read optimization). `--full` re-reads everything,
  `--loop N` runs forever. Parquet via system pyarrow; falls back to
  csv.gz if pyarrow is missing. Reserved placeholder columns (az/el,
  residual_m, temp_c, gain_db) exist from day one — fill columns, don't
  migrate schemas. sky_history rows fill the satellite stream's az_deg/el_deg
  (parse_sky; sky_producer is the source).
- Tests: `python3 scripts/test_archive_roller.py` (sandboxed via
  HACKRF_GNSS_OBS / HACKRF_GNSS_CRATE env overrides; never touches live
  observations), `python3 scripts/test_sky_producer.py` (plain asserts;
  every pass_once runs in a tmp sandbox via sky_producer.bind_obs — the
  live cross-check copies the live INPUTS, so production state/mask/
  history are never touched), `python3 scripts/test_series_producer.py`
  (consensus election + alert history), `python3
  scripts/test_position_watch.py` (plausibility gate), `python3
  scripts/test_position_chart.py` (drives position_watch.cycle over
  fixtures + the position chart render), `python3
  scripts/test_phase_drift_producer.py` and `python3
  scripts/test_phase_tracker.py` (carrier-phase producers), and `node
  web/smoke_sync.js` for the panel (live API or a fixture; exits nonzero
  on failure). The archive tooling touches NO radio and signals NO
  process.

## Testing

`cargo test` (lib + `tests/`); `--bin hackrf_gnss` covers the merge/dash
logic. Gateware sims live in the firmware repo
(`mac-archive/hackrf/firmware/fpga/tests/`, venv `tools/venv-fpga`).
The anchored-PVT truth tool: `cargo run --release --example anchor_residuals`
— per-satellite anchor residuals vs the site anchor (observations/site.json, self-bootstrapped — never surveyed); GPS anchors must be
meter-class before trusting any anchored solve (4-sat solves have zero
residual by construction — rms is not a quality gate at n=4). Residuals are
only measurable with >= 3 anchored GPS channels (fewer and the median pins
one channel to 0.000 by construction; the tool warns). It also prints
per-channel drift of (rho − geom) vs the previous run — the stale-anchor
detector; any sustained rate beyond a few ns/s is an anchor gone stale.

## Firmware

Custom HackRF Pro gateware/MCU: `/Volumes/Radiator 8TB/mac-archive/hackrf`
(branch `upstream-pr-submit`). Multi-image blob: slot 0 = timing variant
(std + TDC − notch), 1 = half_precision, 2 = ext_precision_rx (12-bit,
nibble timestamps, CIC 4× = 8 Msps), 3 = ext_precision_tx. Build-ID regs
0x3E/0x3F per image; the tracked `firmware/fpga/manifest_check.py` performs
the tag-level slot smoke test after every flash. The ignored `build/` copy is
obsolete. Never run the checker during an active tracker or without explicit
serial and restore arguments.

## Version control

Both repos push to PRIVATE remotes: `galic1987/hackrf-gnss` (this crate),
`galic1987/hackrf-timing-firmware` (firmware). The `galic1987/hackrf`
fork is PUBLIC (forks of public repos can't be private) — never push
there. Large captures are gitignored (`*.iq *.f32 *.rawiq`); /tmp is the
boot SSD — no unbounded captures there.

## Firmware provenance boundary

The previous warning that this source tree could not produce the deployed
behavior was wrong. `radio.c` selects resample `n=1` for a 16 Msps standard
stream and passes 32 MHz to `sample_rate_set()`. Slot 0 ties off only the
area-constrained `TimestampCounter` trigger snapshot; its independent TDC is
wired to `trigger_raw`, and slot 1's 48-bit timestamp trigger latch is wired.

Do not confuse source compatibility with deployed-binary identity. Release
0x469 was built from clean source
`a5e8b28c2cf2f9b109b44d5f13e4032dd4cc143e`; its retained manifest records
blob SHA-256
`12ebb3d817a97496a79f785810c56bd15f8b41dc230b3e6146bbb4f60fac4978`.
Slot-0 build registers `0x3E=0x04, 0x3F=0x69` are a useful tag/slot smoke
test, not hash integrity. The full manifest check establishes the four-slot
release but switches images, so never run it during an active tracker or a
measurement window. Also note that a reset with no stream initializes the
AFE/TDC clock at nominal 10 MHz (100 ns), not 40 MHz; record direct clock
source/rate telemetry for every timing claim.
