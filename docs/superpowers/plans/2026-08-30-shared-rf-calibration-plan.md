# 2026-08-30: Shared-RF Coherence and TDC Calibration Plan

> **STATUS: SUPERSEDED / DO NOT EXECUTE AS WRITTEN.** The current retained RF
> topology is AA.250 → Pro #2 GNSS and south-facing ClearStream → HackRF
> One ATSC. Shared split 10 MHz and 1PPS are wired, but no common-L1 feed or
> receiver-delay calibration exists. The single-DC-pass RF scheme below is
> asymmetric and is not a claim-grade RF-leg ABBA calibration; any future
> common-signal experiment needs a reviewed symmetric bias/DC network, safe
> zero-gain preflight, separate USB roots, immutable serials, an atomic
> maintenance lease, and a new fail-closed capture procedure. The live phase
> producer is deployed on the ClearStream path, contrary to the old note at
> the end of this historical plan.

## Current measurement boundary

- **Current RF is not shared:** AA.250 → Pro #2 GNSS and ClearStream →
  HackRF One ATSC are different antennas, bands, transmitters, and receiver
  paths. The present bench can check PPS edge liveness/coarse-start behavior
  and compare independent drift observables. It cannot measure inter-receiver
  RF lag, common-satellite phase, or a dual-GNSS baseline.
- **Matched cables are not zero delay:** equal nominal length does not remove
  splitter-port skew, connector/adaptor delay, trigger synchronizer latency,
  sample-clock phase, analog group delay, or board-to-board receiver delay.
  Those terms need retained ABBA swaps and an uncertainty budget.
- **Coherent 10 MHz + PPS is a repeatability stimulus, not DNL excitation:**
  when both come from the GPSDO, the PPS approaches the TDC at a nearly fixed
  clock phase. That can test connectivity and code repeatability but cannot
  populate a code-density histogram or establish DNL/INL.
- **A DNL run needs deliberately asynchronous phase coverage:** for example,
  run the Pro from its internal TCXO while keeping Bodnar PPS on the trigger.
  Even then, deterministic phase rotation is not by itself proof of uniform
  visitation; an independent phase tag/sweep record must be bound to the raw
  capture before absolute widths or a LUT can be claimed.

The historical objective below targeted sub-nanosecond receiver-delay
calibration before a nominal 6 cm dual-antenna experiment. Its execution
details are retained for review, not approval.

## Mandatory Topology Constraints
- **Clock**: Bodnar LBE-1421 OUT2 (10 MHz) -> Splitter -> Pro P1 & One CLKIN.
- **PPS**: Bodnar OUT1 (1PPS) -> Splitter -> Pro P28.16 & One P28.16.
- **RF Source (historical, asymmetric, not ABBA-approved)**: the old plan used one DC-pass AA.250 leg and one DC-blocked leg. A replacement needs a reviewed symmetric network.
- **USB**: Pro and One must reside on separate USB root controllers.
- **Gain**: HackRF One must start at `amp/LNA/VGA = 0/0/0` (safe mode) to prevent front-end damage.

## Execution Sequence

1. **Verify Clock Tree**: Verify both HackRF serials successfully detect CLKIN (`hackrf_clock -i`) only *after* RX stream start. 
2. **Verify Hardware Arming**: Verify both boards successfully arm off the same PPS edge.
3. **Shared-RF Captures**: Run same-band (L1), same-rate (16 Msps) shared-AA.250 captures.
4. **Fractional Lag Check**: Measure integer/fractional phase lag between the two boards.
5. **ABBA Swaps**: Repeat measurements doing PPS-leg and RF-leg ABBA swaps (and cold starts) to establish deterministic physical biases vs hardware reset races.
6. **TDC asynchronous-sweep candidate**: run the Pro on its internal TCXO while feeding Bodnar PPS to the trigger. The historical -0.667 ppm estimate suggests deterministic phase rotation, but that does not by itself prove uniform visitation or authorize DNL/INL; independent phase tags/sweep evidence are mandatory.
   - *Requirement*: Add a $5 USB temperature logger to the bench before running this.
   - *Cross-check*: Compare the DNL results against the ring-oscillator self-test already sitting in the gateware at registers `0x30–0x35`.
7. **Atomic Coarse+Fine**: Eventually deploy a single atomic coarse (32 MHz tick) + fine (TDC carry chain) FPGA image.
8. **Restore Geometry**: Only after receiver-chain delays have a retained calibration and uncertainty budget, restore separate antennas for a baseline test.

*Current correction: the automatic recovery supervisor remains quarantined,
but the phase producer is deployed on the independent ClearStream ATSC path;
that deployment is not evidence that any shared-RF step occurred.*
