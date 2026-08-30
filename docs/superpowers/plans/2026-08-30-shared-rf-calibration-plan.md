# 2026-08-30: Shared-RF Coherence and TDC Calibration Plan

The next phase of station operation targets true sub-nanosecond receiver delay calibration before resuming the nominal 6 cm dual-antenna carrier-phase experiment. A 6 cm baseline generates ~200 ps of delay (0.315 cycle at L1), which swamps our 30-50 ps timing goal. 

## Mandatory Topology Constraints
- **Clock**: Bodnar LBE-1421 OUT2 (10 MHz) -> Splitter -> Pro P1 & One CLKIN.
- **PPS**: Bodnar OUT1 (1PPS) -> Splitter -> Pro P28.16 & One P28.16.
- **RF Source**: A single shared AA.250 RF signal must be used. One leg must be DC-pass (to the Pro) and the other DC-blocked (to the One) to prevent bias-tee conflicts.
- **USB**: Pro and One must reside on separate USB root controllers.
- **Gain**: HackRF One must start at `amp/LNA/VGA = 0/0/0` (safe mode) to prevent front-end damage.

## Execution Sequence

1. **Verify Clock Tree**: Verify both HackRF serials successfully detect CLKIN (`hackrf_clock -i`) only *after* RX stream start. 
2. **Verify Hardware Arming**: Verify both boards successfully arm off the same PPS edge.
3. **Shared-RF Captures**: Run same-band (L1), same-rate (16 Msps) shared-AA.250 captures.
4. **Fractional Lag Check**: Measure integer/fractional phase lag between the two boards.
5. **ABBA Swaps**: Repeat measurements doing PPS-leg and RF-leg ABBA swaps (and cold starts) to establish deterministic physical biases vs hardware reset races.
6. **Free TDC Code-Density DNL Calibration**: Break the TDC coherence trap using zero new hardware. Run the Pro on the TCXO clock (free-running) while feeding the Bodnar PPS to the trigger. The -0.667 ppm offset sweeps the PPS phase across the full 48-tap window at ~667 ns/s, giving a free, incoherent code-density calibration source.
   - *Requirement*: Add a $5 USB temperature logger to the bench before running this.
   - *Cross-check*: Compare the DNL results against the ring-oscillator self-test already sitting in the gateware at registers `0x30–0x35`.
7. **Atomic Coarse+Fine**: Eventually deploy a single atomic coarse (32 MHz tick) + fine (TDC carry chain) FPGA image.
8. **Restore Geometry**: Only after receiver-chain delays are perfectly mapped, restore the dual separate antennas for the nominal 6 cm baseline test.

*Note: The current autonomous supervisor has been quarantined. The phase producer is undeployed until step 8 is reached.*
