# A2 Deployment and Final TDC Calibration Runbook

**Date:** 2026-09-01  
**Status:** **NO-GO — staging document, not authorization to touch hardware**

This is the controlling plan for the next HackRF Pro timing deployment and
the bench work that follows it. It replaces the idea that the present wiring
alone constitutes a final calibration.

The current production Pro is still running FPGA tag `0x469` with MCU USB API
`0x0116`. The A2 held-mailbox TDC, API `0x0117`, and the corresponding host
commands are source candidates only. No A2 artifact has been approved,
flashed, or read back from hardware.

## Bench truth

| Function | Current path | What it establishes |
|---|---|---|
| Frequency reference | Bodnar OUT2 10 MHz -> splitter -> equal-length cables -> Pro #2 P1 and HackRF One P1 | A common nominal frequency source and repeatable distribution |
| PPS stimulus | Bodnar OUT1 1PPS -> splitter -> equal-length cables -> both boards' P28 pin 16 | Physical PPS distribution; not calibrated arrival time |
| Pro RF | AA.250 -> Pro #2 `0000000000000000645061de252d6613` | Live GNSS reception |
| One RF | south-facing ClearStream path -> One `0000000000000000922c63dc21748847` | Live ATSC ch35 carrier tracking; GPS remains unproven |
| Retired hardware | Pro #1 `0000000000000000977c64de2b557213` | Nothing; never target this serial |

The One currently runs with antenna-port power requested (`-p 1`). That state
is empirically associated with the 2026-09-01 recovery from pilot amplitude
about 0.05 to greater than 8 and roughly millimetre-class phase-equivalent
scatter in the tracker. That is a repeatability observation, not calibrated
delay, distance, or absolute accuracy. Although the
ClearStream antenna was described as passive, those observations suggest an
unrecorded powered inline element or another unresolved DC-path dependency.
Do not silently change this setting. Identify and record every inline RF
component and its DC requirements before the One's next controlled restart.
The currently running legacy process is not to be restarted merely to adopt
source changes. If a controlled One restart is later approved, first complete
that DC-path review, gracefully stop the exact Python wrapper, and prove its
`hackrf_transfer` child is gone. Then use both profile acknowledgements from
the GNSS repository root:

```text
HACKRF_ANTENNA_PROFILE=clearstream_bias_on_20260901 \
HACKRF_RF_PROFILE_ACK=clearstream_bias_on_20260901 \
  nohup python3 scripts/phase_producer.py >> /tmp/phase_producer.log 2>&1 &
```

The old `HACKRF_ANT_POWER=1` launch form is rejected by the new wrapper. The
named profile explicitly requests bias on, RF amp off, and LNA/VGA 40/44; any
change requires a separately named and reviewed profile.

Equal cable length is useful symmetry, not a calibration certificate. Cable
velocity-factor error, splitter-port skew, connectors, input thresholds,
sample-clock phase, synchronizers, FPGA placement, and analog group delay
remain in the measurement.

## Claim boundary

The present coherent 10 MHz plus PPS wiring can support:

- PPS connectivity and one-event-per-pulse checks;
- A2 mailbox atomicity, sequence, overflow, and long-high behavior;
- fixed-phase code repeatability and short-term stability;
- coarse start-alignment experiments, once both capture paths have a proven
  trigger contract.

It cannot by itself support:

- a TDC DNL/INL table or absolute bin widths;
- an absolute splitter/cable delay;
- cross-radio RF delay or interferometric phase;
- a claim that the HackRF One receives GPS;
- a 30--50 ps or 0.1 mm system result.

Those claims require the separate experiments below.

## Gate 1 — freeze an internally consistent release

All boxes in this section are mandatory before a maintenance window is
scheduled.

- [ ] Commit the intended FPGA, MCU, libhackrf, CLI, test, and documentation
  source. The build must begin from a clean worktree using the repository's
  pinned FPGA environment.
- [ ] Allocate a fresh, non-colliding 12-bit build ID. Tag `0x469` is burned:
  historical records bind it to more than one blob.
- [ ] Generate a candidate first. Append and commit the exact explicit
  authorization row emitted by `firmware/fpga/build.py`, then rebuild with the
  same build ID. Publication must reproduce the authorized packed-blob hash.
- [ ] Bind all four slot bitstream hashes, packed-blob hash, source snapshot,
  toolchain, and timing evidence into a v2 manifest.
- [ ] For slot 0, automatically audit and bind nextpnr's detailed report: the
  exact 48 `tdc.s1` endpoints must be `<async>` to positive-edge `adclk`
  sampler paths; every other async endpoint must be explicitly classified;
  the synthesized transitive sampler cone must exclude synchronized
  `selftest_en`; ordinary `adclk` logic must close at at least 40 MHz.
- [ ] Reconcile runtime rates with placed timing. Slot 0's current DAC gate is
  34 MHz and slot 3's is 36 MHz, so firmware/host must reject unqualified TX
  configurations unless fresh placement closes those paths at the advertised
  rate. A warning in documentation is not a runtime interlock.
- [ ] Run the full FPGA simulation suite, host C tests, provenance/release
  tests, and `sphinx-build -W`. Preserve the logs and exact commands in the
  release bundle.
- [ ] Build the matching UNIVERSAL MCU image with API `0x0117` and prove that
  it embeds the exact authorized FPGA blob by unique byte search, length, and
  SHA-256. Never use a hard-coded offset and never flash the bare FPGA blob.
- [ ] Record a rollback bundle before deployment.
- [ ] Stage and hash a known-good ROM-DFU RAM recovery loader as well as the
  normal rollback image. The archived rollback MCU alone is not a complete
  recovery bundle if ordinary USB firmware no longer enumerates.

Current rollback identity:

- MCU image:
  `mac-archive/hackrf/docs/superpowers/releases/2026-08-26-0x469-v2/hackrf_usb.bin`
- MCU SHA-256:
  `1c5a2969ab08be0280f057f879736572a2e2139fa2ac7c1103e910df1a6a67b5`
- FPGA blob SHA-256 recorded by its manifest:
  `12ebb3d817a97496a79f785810c56bd15f8b41dc230b3e6146bbb4f60fac4978`
- Expected restored image: slot 0, tag `0x469`, MCU API `0x0116`.

Any missing checkbox is a NO-GO, not an operator judgment call.

## Gate 2 — pre-stage the maintenance transaction

Nothing should be compiled after the tracker is stopped. Before acquiring
the maintenance lease, prepare:

- the candidate MCU image and manifest, each addressed by SHA-256;
- the rollback MCU image and old manifest, each independently verified;
- the rebuilt `hackrf_pro`, `hackrf_spiflash`, and canonical
  `firmware/fpga/manifest_check.py` paths;
- an immutable command transcript destination with sufficient free space;
- explicit expected serial, build ID, API, ABI, and restore slot values;
- abort conditions and the person responsible for physical USB recovery.

Before this document can change from NO-GO, the signed command transcript
must contain the literal, absolute-path, SHA-verified commands for reset,
candidate flash, re-enumeration, manifest verification, rollback flash, and
ROM-DFU recovery. Placeholders and `PATH`-resolved tool names are not an
executable deployment plan.

All radio-owning clients must honor one atomic maintenance lease. `pgrep`, a
state-file timestamp, and a plain check-then-create file are observations,
not mutual exclusion. The band producer must be unable to claim the Pro
between tracker shutdown and the first maintenance operation.

### Mandatory electrical preflight

No fixed-phase, DNL/INL, absolute-delay, or cross-radio result may be called
final until a retained oscilloscope/VNA preflight records the actual signals at
the board endpoints, with probe/instrument loading and uncertainty stated:

The official [LBE-1421 V1.0 datasheet](https://leobodnar.com/files/datasheets/LBE-1421-Datasheet-V1.0_Initial_Release-15-07-2025.pdf)
is the source-side design target: each output is a 50-ohm source specified as
1.65 V into 50 ohms or 3.3 V into high impedance, and the PPS pulse length is
100 ms. Those values are not endpoint measurements. A passive splitter feeding
high-impedance HackRF inputs changes loading and can create reflections, so the
loaded waveform at each board remains mandatory evidence.

- At **each P28 pin 16 endpoint**, record PPS high/low amplitude, polarity,
  rise time, fall time, pulse width, ringing/overshoot/undershoot, repetition
  rate, and the margin to the documented input thresholds and absolute limits.
- Record the common-ground path and measured ground offset. Document the input
  termination at both boards and the splitter's DC/low-frequency transfer,
  impedance, droop, and port isolation; an RF splitter label is not evidence
  that a 1 Hz pulse is distributed faithfully.
- At **each P1 CLKIN endpoint**, record loaded 10 MHz amplitude, frequency,
  waveform quality, termination, and margin to the documented clock-input
  limits while both splitter legs are attached.
- Give the splitter, every port, adaptor, terminator, PPS cable, and 10 MHz
  cable a stable ID. Record make/model, nominal impedance, measured length or
  delay, connection direction, and the complete as-run port map.
- Retain A-B-B-A swaps of the two PPS cable/port legs and, in a separately
  controlled cold-start sequence, the two 10 MHz cable/port legs. A claimed
  fixed offset must reverse or remain invariant as its physical model predicts;
  otherwise it stays an unresolved systematic.

Pass/fail limits must be copied from the reviewed board/splitter/component
specifications into the evidence bundle before probing. Any threshold-margin,
termination, grounding, overshoot, or splitter-transfer failure is a NO-GO.
Nominally equal cable length is routing symmetry only, never zero-delay or
calibration evidence.

## Gate 3 — controlled Pro deployment

This phase changes hardware and therefore occurs only after Gates 1 and 2
are signed off. The order is part of the safety contract:

Run the ownership commands from the GNSS repository root. The CLI is fixed to
the production observations directory and intentionally has no alternate
lock-root option:

```text
python3 scripts/pro_lease.py gate \
  --serial 0000000000000000645061de252d6613 \
  --token-file /tmp/pro-a2-maint.token
# Pause the named competing producers and gracefully stop the exact tracker.
python3 scripts/pro_lease.py acquire \
  --token-file /tmp/pro-a2-maint.token --wait-seconds 30
# The serial-addressed board reset is the next hardware operation.
```

1. Atomically create the token-owned maintenance **gate**. This blocks new
   client leases but does not try to evict or acquire the tracker's existing
   radio lease.
2. Pause `band_producer` and every other potential Pro client.
3. Gracefully stop the exact `tracker_producer` and `live_radio` processes;
   require the tracker to release its normal client lease.
4. Acquire the exclusive maintenance radio lease with the same gate token.
   Refuse to continue if the old client lease is still present.
5. Reset Pro #2 immediately. Do not build, test, or investigate in the gap;
   a delayed reset previously turned a recoverable stream wedge into a USB
   disconnect.
6. Verify the target serial is exactly
   `0000000000000000645061de252d6613` and the retired `...977c...` serial is
   absent from the transaction.
7. Flash only the SHA-verified matching MCU image that embeds the candidate
   blob, then reset and allow re-enumeration.
8. With the tracker still down, run the canonical manifest checker against
   the candidate manifest with explicit serial and `--restore 0`. A pass is a
   four-slot BUILD_ID smoke test plus verified slot restoration; it is not a
   flash-hash readback.
9. Verify API `0x0117`, slot 0, the new build tag, and capability `0xA2` before
   issuing any TDC measurement command.
10. Release the maintenance radio lease and its gate while `band_producer`
   remains paused. The gate is removed last by the token-owned transaction:
   `python3 scripts/pro_lease.py release --token-file
   /tmp/pro-a2-maint.token`.
11. Start the tracker immediately, confirm that it acquired the normal client
    lease and exclusive Pro ownership, and require a health soak with fresh
    state, GNSS reacquisition, no USB realign storm, and
    `actuate=false`/`corr_applied=false`.
12. Resume `band_producer` last.

If flash, re-enumeration, manifest verification, slot restoration, API/ABI
preflight, or tracker health fails, stop forward work and execute the same
transaction with the rollback MCU image and old manifest. Do not improvise a
partial mixed-version state.

The HackRF One does not need to be restarted for the Pro deployment. Keep its
working RF state untouched until its unresolved powered ClearStream path has
been physically inventoried.

The first lease-aware rollout is a transition exception: the currently
running tracker and band producer were launched from pre-lease code, so they
will not own or honor the new objects already loaded into memory. Create the
gate while the legacy tracker still owns the USB device, pause the legacy
band process before stopping the tracker, verify both exact legacy processes
are gone, and then acquire the maintenance lease. After the lease-aware
tracker has been deployed once, absence of its expected client lease is a
fault rather than an acceptable transition state.

The legacy child may also have inherited blocked SIGINT/SIGTERM from the
wrapper's protected `Popen` window. The new source fixes this with
`scripts/exec_unblocked.py` and a real subprocess regression, but that fix is
not present in an already-running process. During the one-time transition,
try the normal wrapper shutdown first. If and only if the exact legacy
`live_radio` PID remains, the signed transcript must specify a bounded
non-SIGKILL termination of that exact PID (for example SIGHUP), prove it is
gone, acquire the lease, and reset the board immediately before any other
work. Never use a broad `pkill` pattern or declare the USB state healthy after
an abrupt child exit.

## Gate 4 — A2 protocol qualification

Run this in a second bounded maintenance window after deployment health is
established. Re-enter the complete token-owned Gate 3 transaction: create the
gate before stopping the tracker, pause `band_producer`, stop gracefully,
acquire the maintenance radio lease, reset immediately, and verify the exact
serial/API/build/ABI. Hold that lease for the entire capture. The canonical
external capture primitive is:

```text
"/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_pro" \
  -d 0000000000000000645061de252d6613 \
  --tdc-trigger-read N > tdc_external_raw.jsonl
```

Before this command, verify that exact executable's SHA-256 against the staged
release bundle. Do not rely on whichever `hackrf_pro` appears first in `PATH`;
the pre-A2 binary cannot enforce the held-mailbox protocol.
The tool requires and verifies the complete immutable serial for every device
operation, and libhackrf commands the selected transceiver OFF when the handle
closes, including after read-only work. It therefore belongs only inside this
exclusive maintenance transaction.

Before accepting the file, require:

- exact device/API/build/ABI preflight;
- self-test forced physically off and read back safe;
- trigger low and externally armed before BEGIN;
- one contiguous FPGA sequence per accepted PPS;
- no overflow or protocol fault;
- one immutable thermometer word per READY state;
- CLOSE completion, unchanged postflight identity, safe-off, and rearm;
- exactly one terminal stop or abort record and a successful output flush.

Use the actual Bodnar PPS width as the long-high case. A high level must yield
one capture, remain disarmed while high, and rearm only after the sampled line
has drained. Deliberately exercise an extra edge while READY in a bounded HIL
test: it must set overflow without replacing the held word. Also exercise
interruption and broken-output cleanup before relying on unattended captures.

At the end of every Gate 4--6 capture, CLOSE the capture, prove the owning
process is stopped, restore and verify the production slot/configuration, then
use the same token file to release the radio lease and gate. Start and
health-check the tracker before resuming `band_producer`. A successful TDC
command never implicitly releases the maintenance transaction.

The production four-bank `SB_RAM40_4K` mailbox still needs phase-swept HIL or
vendor-cell/post-route simulation. Structural lowering alone does not prove
dual-clock read latency or four-bank atomicity.

## Gate 5 — fixed-phase repeatability run

Keep the Bodnar 10 MHz and 1PPS star exactly as wired. Collect a temperature-
logged series long enough to cover startup, steady state, and at least one
deliberate capture restart.

This run answers:

- Does every clean PPS produce exactly one A2 event?
- Is the mailbox stable and loss-detecting?
- How many thermometer codes occur at the coherent operating phase?
- What are within-run and restart-to-restart repeatability?
- Does swapping equal-length PPS legs reverse a measurable fixed offset?

This run does **not** estimate DNL/INL. A coherent source is expected to visit
only a narrow part of the TDC transfer curve.

Seal the immutable raw capture from this GNSS repository root, supplying the
exact release manifest, serial, measured `adclk`, and physical source
descriptions. The command shape is:

```text
cd "/Volumes/Radiator 8TB/gnss/hackrf_gnss"
python3 scripts/tdc_capture_seal.py \
  /absolute/path/tdc_external_raw.jsonl \
  /absolute/path/tdc_external_sealed.jsonl \
  --manifest /absolute/path/hackrf-fpga-manifest-v2.json \
  --expected-serial 0000000000000000645061de252d6613 \
  --clock-source "<measured source and distribution IDs>" \
  --trigger-source "<measured PPS source and distribution IDs>" \
  --adclk-hz <MEASURED_INTEGER_HZ>
```

Keep raw, sealed, manifest, environment, temperature, and tool hashes together.
The seal is manifest plus BUILD_ID smoke-test provenance, not a hardware
bitstream-hash readback. Use new output names: the sealer intentionally refuses
to overwrite an existing sealed artifact.

## Gate 6 — absolute TDC calibration

Use an independently evidenced phase sweep. Preferred order:

1. A programmable delay generator swept across at least one complete `adclk`
   period, with commanded delay and independent phase readback recorded for
   every event.
2. If that instrument is unavailable, a deliberately asynchronous clock/PPS
   arrangement may be evaluated, but its phase trajectory must be measured
   independently and bound to the capture. A TCXO frequency estimate alone
   does not prove uniform finite-sample phase coverage.

For either method:

- record temperature continuously;
- retain bubbles as diagnostic words but exclude them from bin-density
  estimates;
- treat code 48 as composite full-scale, not an ordinary final bin;
- gate on minimum occupancy and phase coverage, not average counts alone;
- split the data into time blocks and require a stationary transfer function;
- repeat after a cold start and after PPS-leg ABBA swaps;
- report DNL, INL, per-bin uncertainty, missing codes, bubble rate, overflow
  rate, and between-run systematic shift;
- run `scripts/tdc_density_cal.py --require-calibration`; an integrity-only
  success is not a calibration pass.

Only this phase can promote the TDC from a structural/fixed-phase instrument
to a calibrated fine-time estimator. The historical 499 ps number remains an
internal self-test estimate until this gate passes.

## Gate 7 — cross-radio delay calibration

The current AA.250/Pro GNSS and ClearStream/One ATSC paths observe different
signals and cannot reveal receiver-to-receiver RF delay. A later experiment
must feed one common RF waveform through a reviewed symmetric splitter and
DC network to both receivers, starting both gain chains at 0/0/0 and proving
separate USB-root capacity.

Use retained ABBA swaps of RF legs and PPS legs to separate:

- source/splitter/cable delay;
- trigger-input and FPGA delay;
- receiver analog/digital group delay;
- reset- or start-dependent phase ambiguity.

Restore the independent antennas only after the delay model and uncertainty
budget are sealed. A cross-radio interferometer claim comes after this gate,
not from the shared reference wiring alone.

## Final evidence ladder

| Evidence | Maximum defensible statement |
|---|---|
| Shared 10 MHz detected, PPS wired | common distribution exists |
| Clean A2 protocol HIL | external events are captured atomically and loss is detectable |
| Coherent fixed-phase run | code and restart repeatability at one phase |
| Independently tagged phase sweep | DNL/INL and calibrated fine-time estimate |
| PPS-leg ABBA plus temperature repeats | bounded distribution/input systematic |
| Common-RF symmetric ABBA | bounded cross-receiver RF delay |
| Independent antennas after all above | calibrated baseline/interferometric experiment |

The final report must state which row was reached. Precision from a later row
must never be projected upward from an earlier one.
