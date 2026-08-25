# hackrf_gnss — live GNSS/SDR timing station

Rust crate + Python producers + panel server, running 24/7 against two
HackRFs. Read this before touching anything that talks to the radios.

## Hardware ownership (the law)

- **HackRF Pro** `…977c64de2b557213` — owned 24/7 by `examples/live_radio`
  (spawned by `scripts/tracker_producer.py`). `hackrf_open` is EXCLUSIVE:
  no other process can open the Pro while the tracker runs. External
  pollers/captures targeting the Pro will fail or, worse, inject USB
  contention that overflows the tracker's stream queue (the 2026-08-24
  churn: 115 ms FIFO overflows → full channel realigns).
- **HackRF One** `…922c63dc21748847` — owned by `scripts/phase_producer.py`
  (ATSC ch35 carrier-phase track). CLKIN-slaved to the Pro's CLKOUT.
  CLKOUT ownership follows radio ownership: `live_radio` asserts it at
  startup; any flash/reset must re-assert it (`hackrf_clock -o 1`).
- Radio work (flashes, captures) requires stopping `tracker_producer` +
  `live_radio` first and SIGSTOPping `band_producer`; restart after, from
  current binaries (they carry queued fixes).
- **Tracker restarts are only reliable after a board reset** (2026-08-24,
  three trials): SIGTERM or SIGKILL of `live_radio` can leave the Pro's
  USB streaming state wedged — the next `live_radio` then seeds deaf
  ("seed done — 0 candidates" forever). Procedure: `pkill -TERM
  tracker_producer.py; pkill -TERM -f examples/live_radio; sleep 3;
  hackrf_spiflash -d 0000000000000000977c64de2b557213 -R; sleep 6;
  nohup python3 scripts/tracker_producer.py >> /tmp/tracker_producer.log &`.
  Never `pkill -9` live_radio.

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
| tracker_producer + live_radio | state.tracker.json | 1 Hz channels, discipline loop (in-process, steers clock-corr via control handle), tick counter reads |
| phase_producer | state.phase.json | 60 Hz carrier phase; heartbeats `lock:false` when dark; re-acquires after 60 s dark |
| series_producer | state.series.json | 30 s; rolling 1-h band series, consensus, spoof z-alerts (sigma floor 0.05 ppm) |
| band_producer | state.band.json | snapshot rotation — CANNOT snapshot while tracker owns the Pro; rows age, file heartbeats |
| position_producer | state.position.json | runs examples/live_fix every 5 min |
| sky_producer | state.sky.json | 30 s; az/el from BRDC+live eph vs tracker: tracked/absent/unexpected; learns 5°×5° sky_mask.json; appends sky_history.jsonl |

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
  observations). The archive tooling touches NO radio and signals NO
  process.

## Testing

`cargo test` (lib + `tests/`); `--bin hackrf_gnss` covers the merge/dash
logic. Gateware sims live in the firmware repo
(`mac-archive/hackrf/firmware/fpga/tests/`, venv `tools/venv-fpga`).
The anchored-PVT truth tool: `cargo run --release --example anchor_residuals`
— per-satellite anchor residuals vs the surveyed site; GPS anchors must be
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
0x3E/0x3F per image; `firmware/fpga/build/manifest_check.py` verifies all
slots after every flash — never skip it.

## Version control

Both repos push to PRIVATE remotes: `galic1987/hackrf-gnss` (this crate),
`galic1987/hackrf-timing-firmware` (firmware). The `galic1987/hackrf`
fork is PUBLIC (forks of public repos can't be private) — never push
there. Large captures are gitignored (`*.iq *.f32 *.rawiq`); /tmp is the
boot SSD — no unbounded captures there.
