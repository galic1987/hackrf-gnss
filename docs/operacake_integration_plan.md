# HackRF Opera Cake Integration Specification on HackRF Pro

**Target Hardware:** HackRF Pro (`…645061de252d6613`) + HackRF Opera Cake Rev 1  
**Author:** Antigravity Metrology Team  
**Date:** September 2026  
**Status:** Architecture Design & Implementation Plan  

---

## 1. Hardware Architecture & Mounting

The HackRF Opera Cake is an antenna switching add-on board designed by Great Scott Gadgets. When mounted directly on top of the **HackRF Pro**, it interfaces via the digital expansion headers (P20 and P22) to draw power and accept SPI/GPIO switching commands from the LPC43xx microcontroller and FPGA.

### 1.1 RF Matrix Topology
The Opera Cake provides two primary bidirectional RF ports (**PA** and **PB**) and eight secondary ports (**A1–A4** and **B1–B4**):
- **Switch Matrix:** Dual 1x4 RF switches, reconfigurable as a single 1x8 switch via an internal interconnect trace or external SMA jumper.
- **Frequency Range:** 1 MHz to 4000 MHz (fully covering GNSS L1/E1/B1 @ 1575.42 MHz / 1561.098 MHz, L2/B2 @ 1227.60 MHz / 1207.14 MHz, L5/E5a @ 1176.45 MHz, and ATSC ch35 @ 599 MHz).
- **Insertion Loss:** $\sim 1.5\text{ dB}$ at 1.5 GHz (L-band).
- **Isolation:** $> 30\text{ dB}$ port-to-port isolation across L-band.
- **Switching Speed:** $\le 5\ \mu\mathrm{s}$ transition time via digital control lines.

```
                  +-------------------------------------------------------+
                  |               HACKRF OPERA CAKE                       |
                  |                                                       |
  Default Indoor->| A1 [Telescopic Whip - Wideband/FM]                    |
  Passive Whip  ->| A2 [VHF/FM Resonant Metallic Whip]  [A0 / PA] --------> HackRF Pro RF IN
  Outdoor Omni  ->| A3 [High-Band Cellular / PCS / Omni] |                (MAX2839 Front End)
  Mohu+ClearStrm->| A4 [Mohu Leaf Amp -> ClearStream TV] |                
                  |                                      | (Interconnect) 
  ANT500 Indoor ->| B1 [ANT500 Telescopic 75-1000 MHz]   |                
  WLAN 2.4 GHz  ->| B2 [2.4 GHz WiFi Rubber Ducky]      [B0 / PB] --------> (Optional Secondary)
  Active GPS Ant->| B3 [Active GPS Patch (Unpowered LNA)]                 
  Indoor Omni St->| B4 [VHF High-Band Resonant Stand]                     
                  +-------------------------------------------------------+
                                    |                |
                          Control & Power: P20/P22 Expansion
                                    |                |
                  +-------------------------------------------------------+
                  |               HACKRF PRO BENCH                        |
                  +-------------------------------------------------------+
```

---

## 2. Metrological Applications on the Pro Bench

Mounting the Opera Cake on the HackRF Pro transforms a static single-antenna receiver into an agile, multi-port metrological instrument.

### 2.1 Polarimetric & Multipath Inversion (Ports A1 vs A3)
- **Physics:** Direct GNSS line-of-sight signals are Right-Hand Circularly Polarized (RHCP). Upon specular reflection from terrestrial ground planes or water bodies, the electromagnetic wave undergoes a phase reversal, becoming primarily Left-Hand Circularly Polarized (LHCP).
- **Metrology:** Rapidly toggling between Port A1 (RHCP active patch) and Port A3 (LHCP cross-polarized patch) isolates the pure multipath bounce signature from the direct line-of-sight carrier phase. This allows real-time measurement of the multipath phase distortion $\delta \Phi_{\text{mp}}$ without requiring months of sidereal stacking.

### 2.2 In-Situ Front-End Radiometry & Cold-Load Calibration (Port A4)
- **Physics:** In absolute link-budget radiometry ([`scripts/rf_link_budget_radiometer.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/rf_link_budget_radiometer.py)), receiver noise temperature $T_{\text{sys}}$ is typically estimated from carrier-to-noise density $C/N_0$.
- **Metrology:** Switching to Port A4 (a precision $50\ \Omega$ RF load at ambient temperature $T_0 \approx 295\text{ K}$) establishes an absolute thermal noise baseline $P_{\text{ref}} = k_B T_0 B F$. Toggling between A1 and A4 provides automated Y-factor receiver noise figure calibration without disconnecting cables.

### 2.3 Automated ABBA Cable & Splitter Calibration
- **Problem:** When operating HackRF Pro and HackRF One in dual-receiver differencing mode, asymmetric cable lengths and splitter internal delays introduce an uncalibrated receiver differential delay $\Delta \tau_{\text{rx}}$.
- **Metrology:** Using the Opera Cake to route the single antenna feed sequentially:
  $$\text{State A: Antenna} \to \text{Receiver 1 (Pro)}, \quad \text{State B: Antenna} \to \text{Receiver 2 (One)}$$
  and alternating in an $\mathrm{A} \to \mathrm{B} \to \mathrm{B} \to \mathrm{A}$ sequence rigorously cancels linear instrument drift and determines the differential hardware delay down to picoseconds.

### 2.4 Multi-Baseline Interferometric Switched Array
- **Interferometry:** By connecting a 2-element or 3-element spatial antenna array to ports B1, B2, and A1, time-division multiplexed switching across epochs allows single-receiver interferometric baseline determination and attitude vector calculation.

---

## 3. Software Architecture & Control API

### 3.1 Linux/macOS Command Line Interface (`hackrf_operacake`)
The host toolchain controls the Opera Cake via USB vendor requests:
```bash
# Query detected Opera Cake hardware and address
hackrf_operacake -d 00000000000000006450c7dc238f5f67 -l

# Set Port PA to connect to A1 (Zenith Antenna)
hackrf_operacake -d 00000000000000006450c7dc238f5f67 -a 0 -m manual -p A1

# Set Port PA to connect to A4 (50-Ohm Precision Noise Load)
hackrf_operacake -d 00000000000000006450c7dc238f5f67 -a 0 -m manual -p A4

# Configure automated time-dwell switching (e.g. 100 ms on A1, 10 ms on A4)
hackrf_operacake -d 00000000000000006450c7dc238f5f67 -a 0 -m time -w 100000
```

### 3.2 Evidence Envelope Latching
Every time the Opera Cake switches ports:
1. The `continuity_id` is incremented.
2. The `evidence_envelope.receiver_topology.antenna_topology` records the exact active port (e.g., `"OperaCake:PA->A1 (Zenith RHCP)"`).
3. Carrier phase accumulators flag a scheduled cycle slip / phase step during the $5\ \mu\mathrm{s}$ switching transient.

---

## 4. Phased Rollout Schedule

1. **Phase 1 (Hardware Inspection & Seating):** Mount Opera Cake on HackRF Pro #2; verify P20/P22 alignment; check LED power-on indicators.
2. **Phase 2 (Control Verification):** Run `hackrf_operacake -l` to verify I2C address detection and digital latching.
3. **Phase 3 (RF Insertion Loss Characterization):** Measure S-parameters ($S_{21}$) across 1.1 GHz to 1.6 GHz on all 8 ports using tracking generator.
4. **Phase 4 (Daemon Integration):** Create `scripts/operacake_switch_daemon.py` to coordinate scheduled radiometer calibration cycles and multipath polarization sweeps.
