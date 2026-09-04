# The Photon Debugger: HackRF GNSS Metrology & Interferometry Station

[![Metrology Daemons](https://img.shields.io/badge/Daemons-30%2F30%20Active-brightgreen.svg)](#the-30-metrology-sounders)
[![Epistemic Architecture](https://img.shields.io/badge/Epistemics-4--Layer%20Quarantine-blue.svg)](#epistemic-architecture--evidence-contracts)
[![Hardware](https://img.shields.io/badge/Hardware-HackRF%20Pro%20%2B%20One%20%2B%20Opera%20Cake-orange.svg)](#hardware-topology)
[![Reference Clock](https://img.shields.io/badge/Reference-Bodnar%20GPSDO%20Split%20Star-purple.svg)](#clock--pps-distribution)

An autonomous, 24/7 dual-receiver space geodesy, atmospheric sounding, and carrier-phase interferometry platform built on open-source Software Defined Radios (**HackRF Pro** & **HackRF One**), disciplined by an external **Leo Bodnar LBE-1421 GPSDO**, and expandable with the **HackRF Opera Cake** antenna switch matrix.

---

## 🌟 Essential Reading & Learning Paths

If you are new to the repository or want to explore the science and engineering behind this station:

- 📖 [**Engineering Blog Post (`BLOG.md`)**](BLOG.md) — *The Photon Debugger: Turning a $300 SDR into an Atomic-Scale Space Geodesy & Astrophysics Laboratory.* The complete narrative covering the physics, hardware struggles, DSP tracking loops, atmospheric sounders, and the simulation quarantine.
- 🎓 [**Educational Curriculum (`CURRICULUM.md`)**](CURRICULUM.md) — *A Self-Directed 8-Module Masterclass.* From SDR basics, I/Q demodulation, and Allan Deviation, to carrier-phase interferometry, LAMBDA ambiguity resolution, and space weather inversion.
- 🍰 [**HackRF Opera Cake Integration (`docs/operacake_integration_plan.md`)**](docs/operacake_integration_plan.md) — *RF Matrix Switching on HackRF Pro.* Architecture for sub-5 µs switching between Zenith RHCP, multipath LHCP, cold-load calibration, and multi-baseline spatial antenna arrays.
- 🛡️ [**Station Laws & Hardware Ownership (`AGENTS.md`)**](AGENTS.md) — Hardware rules, device serial allocations, RF power policies, and operational constraints.

---

## 🛰️ System Architecture & Hardware Topology

The station operates in a **Split Star** clock distribution topology referenced to a GPS-disciplined oscillator (GPSDO):

```
                       +-----------------------------------+
                       |    Leo Bodnar LBE-1421 GPSDO      |
                       |    (Atomic-Grade Clock Ref)       |
                       +-----------------------------------+
                               |                   |
                     OUT2: 10 MHz Sine       OUT1: 1PPS Pulse
                               |                   |
                        50Ω Power Splitter   50Ω Power Splitter
                          /          \         /          \
                  (Matched Coax) (Matched Coax)
                        /              \     /              \
                       v                v   v                v
                 +------------+       +------------+
                 | HackRF Pro |       | HackRF One |
                 | (P1 CLKIN) |       | (CLKIN)    |
                 | (P28.16 PPS)       | (P28.16 PPS)
                 +------------+       +------------+
                       |
               [P20/P22 Headers]
                       |
                       v
            +--------------------+
            | HackRF Opera Cake  |
            | (Dual 1x4 RF Sw.)  |
            +--------------------+
             /    |        |    \
            A1    A2       A3    A4
          Zenith Low-El   LHCP  50Ω Cal
```

- **HackRF Pro #2** (`…645061de252d6613`): Dedicated production multi-constellation tracker (GPS L1 C/A @ 1575.42 MHz + BeiDou B1I @ 1561.098 MHz at 20 Msps).
- **HackRF One** (`…922c63dc21748847`): Dedicated secondary reference / ATSC ch35 carrier-phase receiver.
- **HackRF Opera Cake**: Antenna matrix add-on mounted directly on the Pro for instant polarization toggling, cold-load radiometry, and switched interferometry.

---

## 🔬 The 30 Metrology & Space Weather Sounders

The station runs an autonomous fleet of 30 concurrent daemons managed by [`scripts/metrology_suite_daemon.sh`](scripts/metrology_suite_daemon.sh):

| # | Daemon Script | Metrological Observable | Physical Principle / Phenomenon |
| :- | :--- | :--- | :--- |
| 1 | `carrier_single_difference_engine.py` | Single-Difference Carrier Phase | Receiver clock offset & satellite geometry |
| 2 | `carrier_double_difference_engine.py` | Double-Difference Phase Residual | Millimeter baseline geometry; clock bias elimination |
| 3 | `carrier_triple_difference_engine.py` | Triple-Difference Step Residual | Autonomous cycle slip detection & phase continuity |
| 4 | `lambda_ambiguity_resolution_engine.py`| Integer Cycle Ambiguities | LAMBDA Z-transformation decorrelation |
| 5 | `hatch_divergence_sounder.py` | Code-Carrier Divergence | Dispersive ionospheric plasma (v_p > c, v_g < c) |
| 6 | `hoi_refraction_engine.py` | Higher-Order Ionosphere (I_2, I_3) | Geomagnetic split & Faraday rotation |
| 7 | `iono_tid_analyzer.py` | Traveling Ionospheric Disturbances | Upper-atmospheric acoustic gravity waves (TIDs) |
| 8 | `agw_tid_wavevector_engine.py` | TID Wavevector | Spatial gradient multi-satellite wavevector inversion |
| 9 | `solar_dawn_detector.py` | Solar Terminator Phase Shift | Sunrise EUV Chapman layer photo-ionization |
| 10 | `solar_flare_sid_monitor.py` | Sudden Ionospheric Disturbance | Solar X-ray / EUV flux flare acceleration |
| 11 | `solar_noon_photochemistry_engine.py` | Midday Maximum Electron Density | Solar elevation & Chapman absorption profile |
| 12 | `multi_frequency_linear_combinations.py`| Ionosphere-Free & Wide-Lane | Multi-frequency dispersive linear combinations |
| 13 | `tropo_saastamoinen_model.py` | Zenith Hydrostatic & Wet Delay | Saastamoinen neutral atmosphere delay model |
| 14 | `tropospheric_refractivity_ducting_sounder.py` | Modified Refractivity & Lapse Rate | 4/3 Earth standard refraction & RF ducting risk |
| 15 | `gnss_meteorology_pwv.py` | Precipitable Water Vapor (PWV) | Bevis water vapor inversion from Zenith Wet Delay |
| 16 | `earth_solid_tide_sounder.py` | Crustal Body Tide Displacement | IERS 2010 Love/Shida lunar-solar gravitational bulge |
| 17 | `solar_radiation_pressure_sounder.py` | Photon Acceleration | Cannon-Milani solar photon momentum transfer |
| 18 | `relativistic_space_time_inspector.py` | GR Redshift & SR Dilation | Gravitational potential difference & orbital speed |
| 19 | `rf_link_budget_radiometer.py` | System Noise Temp & Margin | Friis path loss & radiometer link budget |
| 20 | `frontend_iq_imbalance_sounder.py` | Gain Mismatch & Phase Skew | Image Rejection Ratio (IRR) & quadrature errors |
| 21 | `gnss_reflectometry_sounder.py` | Multipath Power Ratio (dB MPI) | Ground specular reflection & GNSS-R soil sensing |
| 22 | `sidereal_multipath_analyzer.py` | Sidereal Repeat Phase Signature | Orbit repetition filtering (23h 56m) |
| 23 | `isb_adev_analyzer.py` | Inter-System Bias (ISB / ISX) | GPS vs BeiDou receiver hardware delay & Allan Dev |
| 24 | `thermal_phase_analyzer.py` | Thermal Phase Drift | Receiver chassis temperature phase sensitivity |
| 25 | `iono_klobuchar_benchmark.py` | Klobuchar Broadcast vs Real TEC | Broadcast iono model error benchmark |
| 26 | `iono_scintillation.py` | Amplitude & Phase Scintillation | Ionospheric plasma turbulence & scintillation index |
| 27 | `post_sunrise_flux_tracker.py` | Post-Sunrise Ionospheric Ramp | Post-dawn solar flux electron buildup |
| 28 | `ppp_sequential_ekf_engine.py` | Precise Point Positioning (PPP) | Extended Kalman Filter kinematic position state |
| 29 | `gdop_error_ellipsoid_analyzer.py` | 95% Horizontal Error Ellipsoid & GDOP | Multi-constellation geometric dilution & covariance |
| 30 | `satellite_atomic_clock_analyzer.py` | In-Orbit Clock Stability | Quarantined simulation model (PHM vs Rubidium) |

---

## 🛡️ Epistemic Architecture & Evidence Contracts

To guarantee that calculations and simulations are never confused with physical measurements, all outputs implement the **Four-Layer Epistemic Model**:

1. **Layer 1: Immutable Observations (`OBSERVED`)** — Direct ADC samples, raw carrier phase, Doppler shifts, and hardware-latched epochs.
2. **Layer 2: Versioned Calibrations (`CALIBRATED`)** — Tapped delay line DNL tables, front-end I/Q imbalance, and thermal delay slopes.
3. **Layer 3: Derived Scientific Products (`DERIVED` / `MODEL`)** — Inversions and forward models carrying cryptographic lineage hashes, SI uncertainty units, and skew gates.
4. **Layer 4: Presentation Contract (`PRESENTATION`)** — Dashboards refuse unverified data and isolate synthetic models.

### Simulation Quarantine
Any publisher generating synthetic data (such as `satellite_atomic_clock_analyzer.py`) writes exclusively to `sim.*.json`. The Rust server aggregates these files into `out["simulations"]` and segregates them from the live observable namespace. In web dashboards, they are labeled with an amber `⚠️ QUARANTINED SIMULATION` badge.

---

## 🚀 Quickstart & Operations

### 1. Prerequisites
- macOS or Linux with Rust (1.75+) and Python 3.9+.
- `libhackrf` and HackRF host command-line tools.
- External Leo Bodnar GPSDO or 10 MHz reference connected to HackRF CLKIN.

### 2. Build the High-Performance Rust Server & Tracker
```bash
# Build the production release binary
cargo build --release --bin hackrf_gnss

# Run unit tests
cargo test --bin hackrf_gnss
```

### 3. Launch the Server & Metrology Fleet
```bash
# Start the web & API server (port 8090)
./target/release/hackrf_gnss --mode serve --port 8090

# In another terminal, start all 30 metrology sounders
bash scripts/metrology_suite_daemon.sh start

# Check status of the fleet
bash scripts/metrology_suite_daemon.sh status
```

### 4. Open the Web Dashboards
Navigate to your local browser:
- **Comprehensive Metrology Dashboard:** `http://localhost:8090/iono.html` (30 high-contrast, accessible cards)
- **Photon Story & Phase Tracker:** `http://localhost:8090/story.html` (8 chapters, 9-stage pipeline, disconnected slip timeline)
- **Clock & Hardware Lab Monitor:** `http://localhost:8090/sync.html` (Split Star topology, Allan deviation, TDC calibration)

---

## 🤝 Contributing & Community
We welcome contributions! Please review [`AGENTS.md`](AGENTS.md) before submitting pull requests to ensure strict adherence to station laws, hardware ownership, and epistemic evidence contracts.
