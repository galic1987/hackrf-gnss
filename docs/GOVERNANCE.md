# Station Governance Charter & Operating Constitution

**Station:** `hackrf_gnss` Live Metrology & Timing Station  
**Effective Date:** 2026-09-04  
**Authority:** Station Operator & Metrological Architecture Team  
**Status:** Canonical / Enforced  

---

## Preamble: The Restoration of Metrological Truth

On September 4, 2026, a forensic audit revealed that twenty-one metrology and geodesy engines operating on this host were emitting synthetic numbers disguised as live radio-frequency observations. Mathematical models and pseudorandom generators were chained together such that downstream estimators "measured" synthetic inputs from upstream fabricators, and these simulated outputs were rendered on public web dashboards alongside real receiver observables.

This breach compromised the foundational integrity of the station. The core purpose of an empirical timing observatory is not to produce cosmetically plausible numbers, but to record the physical reality of electromagnetic waves interacting with hardware.

This Charter establishes the permanent operating rules, epistemic boundaries, and process guards for `hackrf_gnss`. Every human engineer, autonomous agent, automated daemon, and git commit is bound by this constitution.

The cardinal law of the station is:
> **Never let an operator, researcher, or downstream system mistake a calculation, model, or simulation for an empirical observable.**

---

## Article I: The Three Real Goals of the Station

All engineering effort, compute cycles, and physical hardware allocations at this station are dedicated solely to three real-world metrological goals:

### 1. Long-Term Carrier-Phase Continuity
The primary objective of the GNSS receiver pipeline is sustained, cycle-slip-free carrier-phase tracking over multi-hour and multi-day continuous arcs:
* **True Observables:** Measuring uninterrupted carrier phase on live GPS L1 C/A (1575.42 MHz) and broadcast television carriers (e.g. ATSC ch35 602.809440 MHz carrier pilot).
* **Allan & Time Deviation:** Evaluating genuine frequency stability ($ADEV$, $TDEV$) across observation intervals from $10^0$ s to $>10^5$ s against physical oscillators.
* **Deterministic Gap Bridging:** Detecting real cycle slips, half-cycle ambiguities, and hardware realignments without artificial interpolation, numerical smoothing, or selective cherry-picking of quiet windows. Legitimate continuous lock periods must be measured against physical criteria.

### 2. True GPSDO Disciplining
The station operates an external physical frequency and time standard: a Leo Bodnar LBE-1421 dual-output GPSDO providing:
* **10 MHz Syntonization:** Fed directly to the HackRF CLKIN reference inputs over matched coaxial cables via an RF power splitter (Split Star topology).
* **1PPS Synchronization:** Fed directly to expansion header pin `P28.16` (`TRIGGER.IN`) on both radios via matched cables.
* **Physical Steering & Phase Alignment:** The goal is genuine measurement and steering of hardware NCO/DSP tracking loops against this physical standard. Simulated disciplining, synthetic clock drift models, and unverified software corrections masquerading as hardware register actuation are strictly prohibited.

### 3. Genuine Multipath and Interferometric Baselines
The station deploys multi-element antenna feeds and multi-radio SDR receivers (HackRF Pro and HackRF One, expandable via Opera Cake RF crossbar switching) to measure the real physical propagation environment:
* **Physical RF Baselines:** Measuring real electromagnetic phase differences between spatially separated or alternately polarized antennas.
* **Empirical Multipath Characterization:** Extracting signal-to-noise ratio ($C/N_0$) oscillations, pseudorange-minus-carrier ($CMC$) divergence, and carrier single/double differences strictly from simultaneous, time-correlated physical IQ frames.
* **Environmental Signatures:** Measuring real reflections from local building geometry, ground reflections, and transient obstructions—never simulated multipath patterns.

---

## Article II: The Epistemic Mandate

Every state file emitted in `observations/state.*.json`, every line logged to JSONL telemetry, and every metric published to web interfaces must strictly adhere to the Station Provenance Contract.

### 1. Mandatory Schema Invariants
Every production state file (`state.<producer>.json`) must strictly contain the following top-level fields:

```json
{
  "source": "<canonical_hardware_or_channel_identifier>",
  "input_counts": <positive_integer_of_verified_physical_frames>,
  "synthetic": false
}
```

* **`source` (string or structured object):**
  Must identify the exact physical device, serial number, RF front-end, frequency, and antenna path (e.g., `"HackRF Pro 0000000000000000645061de252d6613 GPS L1 ClearStream-South"`). Generic names like `"engine"` or `"sim"` are forbidden in production state.
* **`input_counts` (integer):**
  Must report the exact number of raw physical samples, IQ blocks, NCO cycles, or discriminator dumps ingested during the update epoch. If `input_counts == 0`, the producer must fail closed and report `validity: false` or omit measurement values. It must never fabricate estimates from prior states.
* **`synthetic` (boolean):**
  Must be explicitly `false`. Any file containing synthetic data, textbook models, or simulated distributions must set `"synthetic": true`, must prefix the filename with `sim.` rather than `state.` (e.g., `sim.clock_drift.json`), and must never be merged into production dashboards or published state feeds.

### 2. Prohibition of Chained Synthetic Data
No script, engine, or daemon may consume the output of a synthetic generator or theoretical model and represent the resulting calculation as an empirical observation. Any derived product (e.g. double-difference carrier phase) must trace its entire lineage back to simultaneous, verified Layer 1 physical IQ/phase measurements from real hardware.

---

## Article III: The Specification Gate

Subjective, cosmetic, or ad-hoc status verdicts are prohibited across all station instrumentation.

### 1. Prohibition of Unaudited Verdicts
No JSON state file, terminal log, or web dashboard element may output a categorical verdict field—including but not limited to `pass`, `fail`, `ok`, `nominal`, `healthy`, `degraded`, or `trusted`—unless that field is governed by an audited, pre-registered mathematical specification.

### 2. Criteria for a Pre-Registered Specification
To be granted a specification gate, the underlying algorithm must satisfy:
1. **Mathematical Pre-Registration:** A documented mathematical formulation (located in `docs/`) detailing the exact equations, hypothesis tests, null hypothesis $H_0$, degrees of freedom, and probabilistic criteria.
2. **Empirically Calibrated Thresholds:** Cutoff thresholds must be derived from measured receiver noise floors, Allan deviation baselines, and hardware calibration sweeps—never arbitrary constants or magic numbers chosen to force a green test badge.
3. **Traceable Uncertainty:** Every reported value must carry explicit confidence bounds, standard errors ($\pm \sigma$), or variance-covariance matrices.
4. **Independent Audit:** The specification must be reviewed and signed off by the Station Operator or designated Metrological Architect before being merged into production.

### 3. Fail-Closed Default
If an observable lacks an audited specification, producers must report raw numerical values accompanied by sample counts and timestamps, or report `null`. They must never emit a fake or speculative `ok: true`.

---

## Article IV: The Host Process Peace Treaty

The station host is a real-time signal processing environment. The real-time USB transfer buffers for HackRF streaming operate with narrow queue margins:
* HackRF transfer queue margin: $\approx 190\text{ ms}$.
* Buffer drop threshold: Host scheduling stalls $> 115\text{ ms}$ cause hardware FIFO overflow, unrecoverable sample loss, and complete loss of carrier tracking (forcing full channel realignments).

To protect real-time radio operations from CPU starvation and USB contention, all processes on this host are bound by the **Host Process Peace Treaty**:

### 1. Total Ban on Heavy Host Computations During Live Tracking
While `live_radio`, `hackrf_transfer`, or any process holding a HackRF device lease is active:
* **NEVER** run `cargo build --release` or unconstrained `cargo build`.
* **NEVER** run `cargo test` across the workspace.
* **NEVER** run unthrottled numerical simulations, Monte Carlo runs, or multi-threaded batch jobs.
* **NEVER** run FPGA synthesis tools (e.g. `nextpnr`, `yosys`).

### 2. Dedicated Maintenance Windows
All builds, comprehensive test suites, and hardware flashing must occur exclusively inside token-gated maintenance windows:
1. Create maintenance gate atomically:  
   `python3 scripts/pro_lease.py gate --serial <SERIAL> --token-file /tmp/maint.token`
2. Gracefully terminate `tracker_producer` / `live_radio`.
3. Acquire maintenance radio lease:  
   `python3 scripts/pro_lease.py acquire --token-file /tmp/maint.token --wait-seconds 30`
4. **Immediately perform a hardware board reset** (`hackrf_spiflash -R`) before any compilation or test execution.
5. Execute required builds/tests.
6. Release maintenance lease and restart production tracking.

### 3. Absolute Prohibition of Broad `pkill` Commands
* **NEVER** run broad termination commands: `pkill -9 live_radio`, `pkill -f hackrf`, `killall cargo`, or blanket pattern kills.
* Abruptly terminating `live_radio` leaves the HackRF Cypress FX2/MAX V CPLD USB streaming state wedged. When wedged, subsequent tracker launches seed deaf (`0 candidates`), and the radio often disappears from the USB bus entirely, requiring physical power-cycling.
* Process termination must be deterministic, targeting exact PIDs, preceded by maintenance gating, and followed immediately by hardware reset.

### 4. Process Scheduling & CPU Caps
* The production tracking process (`live_radio`) must be granted elevated real-time priority (`sudo renice -5 -p <PID>`).
* Non-real-time helper processes, probes, and telemetry monitors must run niced (`nice -n 19`) and thread-capped (`RAYON_NUM_THREADS=2`).

---

## Article V: Automated Agent Governance & Commit Trailers

Autonomous and AI-assisted agents operating on this repository operate under delegated authority and must maintain complete transparency and accountability.

### 1. Mandatory Git Commit Trailers
Every automated or agent-assisted commit must include git trailers identifying the responsible agent role and the auditing authority:

```text
Agent-Role: <Role Name>
Audited-By: <Operator or Auditor Name> <<email>>
```

**Example Commit Message:**
```text
fix(analyzer): bridge single-row A/B gap holes in rolling window

Implement deterministic 2.08s gap bridging for A/B membership exclusion
rows to prevent artificial 3600-second lock fragmentation.

Agent-Role: Analyzer Quality Hour Specialist
Audited-By: Ivo Galic <galic1987@gmail.com>
```

### 2. Automated Repository Guards
Repository integrity is enforced by automated pre-commit and commit-message hooks:
* `scripts/guard_live_radio.sh`: Refuses execution of `cargo test`, `cargo build --release`, and broad `pkill` commands while `live_radio` holds the hardware device lease.
* `scripts/pre_commit_guard.sh`: Enforces state schema invariants (Epistemic Mandate), verifies quarantine stubs, and blocks live-radio contention during commits.
* `scripts/commit_msg_guard.sh`: Enforces presence of `Agent-Role:` and `Audited-By:` trailers on automated commits.

---

## Article VI: Enforcement & Penalty

1. **Immediate Quarantine:** Any script or tool found generating unverified synthetic observables or violating the Specification Gate will be immediately replaced with an `exit 78` (`EX_CONFIG`) quarantine stub.
2. **Reversion of Unaudited Code:** Any commit made without the required trailers or in violation of the Host Process Peace Treaty will be reverted.
3. **Fail-Closed Hardware State:** If device lease state or lock directories are corrupted, all automated launches must halt until an operator inspects the physical bench.
