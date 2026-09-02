# 2026-08-30: TDC Code-Density Sweep Window (TCXO DNL Calibration)

> **STATUS: QUARANTINED / SUPERSEDED — DO NOT EXECUTE.** The operator entry
> point now exits without touching hardware. Review established that a reset
> with no stream initializes nominal adclk at **10 MHz / 100 ns**, not the
> 40 MHz / 25 ns assumed below; the retained Run 1 artifact cannot attest its
> source or rate. A deterministic 1 Hz sample of a fixed TCXO/PPS ratio is a
> phase rotation, not independent proof of uniform finite-sample excitation.
> The old seven-process register poll can also accept a PPS transition during
> its six-byte read, and code 48 is a composite overflow/deferred-capture code,
> not a simple saturation bin. Finally, `gpsdo.lock` was only a fresh NMEA
> navigation fix, not PPS/oscillator telemetry. The remainder of this file is
> retained as historical design rationale, not an approved procedure.
>
> A replacement requires direct build/source/adclk attestation, one long-lived
> status-before/data/status-after reader, atomic maintenance ownership with
> verified restoration, and independently swept/randomized or phase-tagged
> stimulus evidence bound to the capture SHA-256. See
> `scripts/tdc_sweep_window.sh` and `scripts/tdc_density_cal.py` for the current
> fail-closed boundary.
>
> Repository boundary: run the offline seal/analyzer commands from
> `/Volumes/Radiator 8TB/gnss/hackrf_gnss`. Release-manifest generation and
> FPGA timing review belong to the companion firmware repository at
> `/Volumes/Radiator 8TB/mac-archive/hackrf`; never assume the same cwd or a
> similarly named ignored `build/` artifact.

Executes step 6 of `docs/superpowers/plans/2026-08-30-shared-rf-calibration-plan.md`
("Free TDC Code-Density DNL Calibration") as a single checkpointed operator
procedure: `scripts/tdc_sweep_window.sh`. Companion: `scripts/supervisor_v2.sh`
(manual-start supervisor that honors `maintenance.lock`; v1 does not and is
quarantined).

## Objective

Map the per-tap widths (DNL/INL) of the 48-tap iCE40 carry-chain TDC on the
slot-0 timing image, using the station's free incoherent sweep source:

- In TCXO mode the Pro's adclk runs at a nominal 40 MHz (25 ns period) with a
  measured **−0.667 ± 0.004 ppm** offset against the Bodnar PPS (run1; honest
  error bar per the 2026-08-30 adversarial audit — the earlier ±0.00003 ppm was
  quantization, not stability).
- −0.667 ppm × 1 s = the Bodnar PPS phase slews **~667 ns per second** against
  the adclk, wrapping the 25 ns clock period every **~37.5 ms**. Sampling at
  1 Hz (one PPS edge per second) therefore lands **quasi-uniformly** across the
  clock period: a free code-density (histogram/DNL) calibration source with
  zero new hardware.
- Bodnar/CLKIN mode is **coherent** (run3 null: +0.026 ppb — every edge lands
  on the same phase) and blocks calibration entirely. TCXO mode is not a
  degraded fallback here; it is the *requirement*.

The 48 × 105.5 ps ≈ 5.07 ns chain covers 5.07/25 = **20.3%** of the period, so
**79.7% saturation (popcount = 48) is structural geometry, not a defect** —
run1 measured 79.74%. Known real features the sweep must reproduce: the k=1
first bin is a wide ~540 ps bin (NOT left-censoring — a left-censored edge
would surface as saturation one period later), and the segment-16 carry-hop
bins at codes 17/33 (~1128/1306 ps). Si5351 jitter (3.5 ps RMS) is not a floor
on any of this.

## THE COHERENCE TRAP (why no transfer may run — P6)

`activate_best_clock_source` (firmware `usb_api_transceiver.c:411`) runs on
**ANY streaming transfer, including OFF-mode transfers**, and latches CLKIN
whenever a 10 MHz reference is present at the connector. The Bodnar 10 MHz is
*always* cabled at this station. Consequences:

- One `hackrf_transfer`, one `live_radio` start, one `band_producer` snapshot
  during the sweep, and the board **silently goes Bodnar-coherent**. There is
  no error, no log line; the PPS phase freezes, the histogram collapses onto a
  few codes, and hours of data are garbage.
- Therefore `tdc_sweep_window.sh` invokes **only** register-level control
  operations (`hackrf_pro --read-reg`, `--tdc-selftest`) and
  `hackrf_spiflash -R`. Register polling is proven transfer-safe: run1
  (`observations/tdc_pps_run1.jsonl`) was captured exactly this way and held
  −0.667 ppm (TCXO) for its full duration.
- Existing scripts `run_latch_capture.sh` and `observations/tdc_cal_run1_hil.sh`
  deliberately FORCE a stream to latch CLKIN / set adclk. **Do not reuse that
  idiom here** — it is precisely the trap. (`run_latch_capture.sh` also uses
  `hackrf_debug -P`, which is banned station-wide for silently reverting the
  FPGA slot.)
- The tracker is killed and `band_producer` is SIGSTOPped for the whole window;
  the script verifies no orphan `live_radio` holds the radio before resetting.

## Preconditions

1. **Supervisor v1 must be OFF.** It ignores `maintenance.lock` and would
   relaunch the tracker (a streaming transfer → coherence) within 30 s. It is
   quarantined and not running now; verify with `pgrep -f supervisor` — expect
   no `scripts/supervisor.sh` match. Note: the bare pattern can match unrelated
   processes (this bench shows a `service-supervisor.ts` node process from
   another project); the precise check is `pgrep -f 'scripts/supervisor\.sh'`.
   `supervisor_v2.sh` may be running: it stands down on `maintenance.lock`.
2. The NMEA probe may remain running because it holds a serial port, not a
   radio, but `nmea_fix_valid:true` is only receiver-navigation health. The
   current bench has no telemetry that attests 10 MHz lock, PPS phase, UTC
   offset, or holdover; this missing output-state evidence is an additional
   reason the quarantined procedure cannot make an absolute claim.
3. Bodnar 1PPS on P28.16 (external trigger), slot-0 timing image flashed
   (the script verifies TDC regs respond post-reset; it will NOT touch slots).
4. Bench temperature known (see Temperature, below).

## Procedure (what the script does)

```
scripts/tdc_sweep_window.sh [DURATION_S=14400] [OUT=observations/tdc_sweep_run1.jsonl]
START_TEMP_C=23.4 scripts/tdc_sweep_window.sh        # temp goes in the header
```

1. This procedure remains quarantined, but its replacement must use
   `scripts/pro_lease.py gate` with the exact production serial and an
   operator token—never create or remove `maintenance.lock` directly. The
   gate is established before any stop and
   normal clients double-check it around atomic `pro.radio.lock.d` acquisition.
2. During the first deployment of lease-aware producers, SIGSTOP the already
   loaded legacy `band_producer`; then gracefully stop the exact tracker
   (spec `2026-08-25-clock-write-continuity-experiment.md:127-133` — an intact
   pattern once self-matched the wrapper and live_radio kept streaming);
   sleep 3; verify `live_radio` exited (TERM the orphan; never −9; abort if
   the radio stays held).
3. `pro_lease.py acquire --token-file … --wait-seconds 30` must succeed;
   only then may the replacement issue `hackrf_spiflash -R` with checked
   exit. Abort on failure. sleep 6. A cold
   boot with no stream started leaves the Si5351 on the internal TCXO: the
   calibration source is armed by *doing nothing*.
4. Verify slot-0 TDC: reg 0x31 readable, reg 0x30 == 0x00 (RO self-test OFF —
   an active ring oscillator corrupts external-edge capture).
5. Write a full JSON config header line (clock source TCXO/XTAL nominal
   40 MHz, slot 0, board serial, firmware provenance note, `start_temp_c`) —
   **measurements without recorded configuration are this station's recurring
   sin**; then capture at 1 Hz: read 0x31, read the frozen thermometer map
   0x20–0x25, append run1-schema lines (`{"t":…, "reg31":…, "bytes":…}`).
   Checkpoint lines every 300 s; abort after 30 consecutive read failures.
6. Restore the verified production state while the maintenance lease remains
   held. Release with the same token, start the tracker, verify its JSON lease
   owner and stream health, then resume band producer. If acquisition never
   occurred, use token-owned `cancel`; ambiguous/stale state remains gated for
   manual inspection. Print the offline analysis command.

## Sample-size math

| Quantity | Value (hits/bin figures are AVERAGES over the 48 bins) |
|---|---|
| Event rate | 1 PPS/s = 3600/h |
| In-window fraction | ~20.3% (79.7% structural saturation) |
| In-window events/h | ~731 |
| Target ≥100 hits/bin × 48 bins | 4800 in-window → 4800/0.203 ≈ 23,600 s ≈ **6.6 h** |
| Minimum publishable 50/bin | 2400 in-window → ≈ 11,800 s ≈ **3.3 h** |
| **Default 4 h (14,400 s)** | ≈ 2,920 in-window → ≈ **60/bin** (per-bin counting error √60/60 ≈ 13%) |

The 4 h default is a bounded first run; extend toward 6.6 h (max 8 h bound in
the script) for the ≤10%/bin table.

Note: the hits/bin figures above are *averages* over the 48 bins. Narrow bins
fill proportionally slower — that differential fill rate IS the DNL signal —
so a run averaging 100/bin can leave its narrowest bins well short of that.
Publishability should therefore gate on the **minimum** per-bin count, not the
average.

## Success criteria

- Saturation (popcount = 48) fraction **79.7 ± 2%** over the full run and
  stable across 30-min blocks (drift outside that band ⇒ clock mode changed
  or PPS lost).
- Phase-uniformity check: two-sample chi-square between first-half and
  second-half in-window histograms consistent (p > 0.01) — the *bin* profile
  is the DNL signal and is not expected flat, but it must be stationary.
- Known features reproduced: wide k=1 first bin (~540 ps), segment-16 hop
  bins at codes 17/33.
- DNL/INL table published (`tdc_density_cal.py --out-json`) with per-bin
  counting errors σ_k ≈ W_k/√N_k noted alongside.

## Abort criteria

- 30 consecutive register-read failures (script auto-aborts; board likely
  wedged — see AGENTS.md 2026-08-29 replug incident).
- Any streaming transfer observed/started during the window (tracker, band
  snapshot, hackrf_transfer) → data void from that instant; abort and restart
  the sweep from a fresh reset.
- Collapsed code diversity (long runs of identical popcounts / saturation
  → ~100% or → well below 60%): the CLKIN-latch signature. Void the run.
- `state.gpsdo.json` goes `nmea_fix_valid:false` or stale: record the GNSS
  receiver-health warning and abort this already-quarantined procedure. A
  valid NMEA fix would still not establish PPS truth.
- Bench temperature excursion > ~2 °C without logging (see below).

## RO self-test cross-check (regs 0x30–0x35)

After the capture (never during — 0x30 must be 0x00 while sampling external
edges), the ring-oscillator self-test in the gateware provides an independent
tap-rate anchor: `hackrf_pro --tdc-ro-meas N` writes windows from regs
0x32–0x35 (idiom: `observations/tdc_cal_run1_hil.sh` steps 3/5 — but note that
script's stream-start step is FORBIDDEN here, and it carries a `-d $PROG`
serial-vs-path bug). Hard-gate the RO off afterwards and verify 0x30 reads
0x00. Compare RO-implied mean tap delay against the code-density mean
(105.5 ps nominal); disagreement > ~10% flags a decode or clock-mode error.

## Temperature (P5)

The TCXO tempco is 0.014–0.1 ppm/°C and run1's bench temperature went
unlogged — the −0.667 ± 0.004 ppm error bar is partly a confession of that.
The sweep *rate* (667 ns/s) entering the uniformity argument inherits it.
Record `START_TEMP_C` in the header and the end temperature in the stop line
(manual entry). The shared-RF plan makes a $5 USB temperature logger a
**requirement** before this step; this kit permits manual start/end bracketing
as a bootstrap for the 4 h run, but the ≥100/bin publishable run should wait
for the logger. This is a deliberate, declared relaxation — not a
contradiction of the shared-RF plan.

## Analysis

```
python3 scripts/tdc_density_cal.py observations/tdc_sweep_run1.jsonl \
    --clock-ns 25.0 --taps 48 \
    --out-json observations/tdc_sweep_run1_dnl.json
```

The analyzer parses the `"bytes"` schema written by the sweep (same as
`tdc_pps_run1.jsonl`), reports saturation rate, per-bin widths, DNL/INL, and a
calibrated prefix-sum LUT matching `src/tdc.rs`.

## Cross-references

- `docs/superpowers/plans/2026-08-30-shared-rf-calibration-plan.md` step 6
  (this plan implements it; steps 1–5/7–8 unaffected) and its note that the
  autonomous supervisor is quarantined.
- `docs/superpowers/reports/2026-08-29-tdc-pps-run1-latch-calibration.md`
  (amended 5a033dc): run1/run3 differential, DNL-comb retraction, 79.74%
  saturation geometry.
- `docs/superpowers/plans/2026-08-27-sub-ns-leg1-clock-bias.md:519-532`
  (maintenance-window choreography this script automates).
- `AGENTS.md` restart law (~:100-131): kill → reset IMMEDIATELY → relaunch;
  never `pkill -9 live_radio`; no cargo during tracker streams.
- `docs/superpowers/specs/2026-08-25-clock-write-continuity-experiment.md:127-133`
  (pattern-broken pkill idiom).
