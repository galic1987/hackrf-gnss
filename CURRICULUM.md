# GNSS Metrology, Space Geodesy, & SDR Interferometry: A Self-Directed Curriculum

**Level:** Intermediate to Advanced  
**Prerequisites:** Familiarity with Python, basic Rust or C, linear algebra, signal processing fundamentals (Fourier transforms, convolution, complex numbers), and introductory physics.  
**Target Platform:** HackRF One / HackRF Pro, external GPSDO (e.g. Leo Bodnar LBE-1421), GNSS active antenna, HackRF Opera Cake (optional/advanced).  
**Repository:** `hackrf-gnss` (https://github.com/galic1987/hackrf-gnss)

---

## Curriculum Overview

This curriculum is designed as an open-access masterclass for self-directed engineers, physicists, and computer scientists. By following these modules, you will learn how to turn commodity Software Defined Radios into atomic-scale metrology instruments capable of millimeter geodesy, ionospheric imaging, and relativistic astrophysics.

```
+------------------------------------------------------------------------------------+
|                                CURRICULUM ROADMAP                                  |
+------------------------------------------------------------------------------------+
| Module 1: SDR & RF Front-End    ---> Module 2: Clocks, Timing & FPGA TDCs          |
| (Downconversion, IQ, Noise Fig)      (Allan Deviation, DNL, Split Star)            |
+------------------------------------------------------------------------------------+
                                         |
                                         v
+------------------------------------------------------------------------------------+
| Module 3: GNSS Baseband DSP     ---> Module 4: Carrier-Phase Interferometry        |
| (PRN Codes, 60 Hz NCO, Hatch)        (Double/Triple Diff, LAMBDA Ambiguities)      |
+------------------------------------------------------------------------------------+
                                         |
                                         v
+------------------------------------------------------------------------------------+
| Module 5: Atmospheric Sounding  ---> Module 6: Relativistic Geodesy & Orbits       |
| (Ionosphere TEC, AGW, Tropo PWV)     (General Relativity, Solid Tides, SRP)        |
+------------------------------------------------------------------------------------+
                                         |
                                         v
+------------------------------------------------------------------------------------+
| Module 7: Epistemic Architecture---> Module 8: HackRF Opera Cake Matrix Switching  |
| (Evidence Envelopes, Quarantine)     (Polarization Diversity, ABBA Calibration)    |
+------------------------------------------------------------------------------------+
```

---

## Module 1: Software Defined Radio & RF Front-End Architecture

### Learning Objectives
- Master the physics of direct-conversion (zero-IF / low-IF) transceivers (MAX2837/MAX2839).
- Understand I/Q demodulation, quadrature phase errors, and gain imbalance.
- Calculate RF link budgets, free-space path loss (FSPL), and receiver system noise temperature ($T_{\text{sys}}$).

### Key Concepts & Equations
1. **Complex Baseband Representation:**
   $$s(t) = I(t)\cos(\omega_c t) - Q(t)\sin(\omega_c t) = \operatorname{Re}\left\{(I(t) + j Q(t)) e^{j\omega_c t}\right\}$$
2. **I/Q Imbalance & Image Rejection Ratio (IRR):**
   $$\mathrm{IRR} = \frac{1 + 2\alpha\cos\phi + \alpha^2}{1 - 2\alpha\cos\phi + \alpha^2}$$
   where $\alpha$ is amplitude gain mismatch and $\phi$ is quadrature phase error.
3. **Friis Link Budget Equation:**
   $$P_{\text{rx}} = P_{\text{tx}} + G_{\text{tx}} + G_{\text{rx}} - 20\log_{10}\left(\frac{4\pi d}{\lambda}\right) - L_{\text{atm}}$$

### Practical Repo Exploration
- Run [`scripts/frontend_iq_imbalance_sounder.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/frontend_iq_imbalance_sounder.py) to measure real-time gain/phase mismatch.
- Inspect [`scripts/rf_link_budget_radiometer.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/rf_link_budget_radiometer.py) to analyze $T_{\text{sys}}$ and carrier-to-noise density $C/N_0$.

---

## Module 2: Oscillators, Timing, & FPGA Time-to-Digital Conversion

### Learning Objectives
- Characterize clock stability using Allan Deviation ($\sigma_y(\tau)$) and Time Deviation (TDEV).
- Eliminate TCXO drift using a dual-receiver GPSDO Split Star topology.
- Design and calibrate tapped delay line Time-to-Digital Converters (TDCs) inside FPGAs.

### Key Concepts & Equations
1. **Allan Deviation:**
   $$\sigma_y^2(\tau) = \frac{1}{2(N-1)}\sum_{k=1}^{N-1}\left(\bar{y}_{k+1} - \bar{y}_k\right)^2$$
2. **Differential Non-Linearity (DNL) in Tapped Delay Chains:**
   $$\mathrm{DNL}_k = \frac{W_k - W_{\text{nominal}}}{W_{\text{nominal}}} = \frac{N_k}{\bar{N}} - 1$$
3. **Two-Stage Clock Domain Crossing (CDC) Synchronizers** with domain-local clear to eliminate metastability.

### Practical Repo Exploration
- Study [`scripts/isb_adev_analyzer.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/isb_adev_analyzer.py) to observe GPS vs BeiDou Inter-System Bias ADEV.
- Review the Split Star wiring diagrams in [`docs/p0c_clock_continuity.md`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/docs/p0c_clock_continuity.md) and [`AGENTS.md`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/AGENTS.md).

---

## Module 3: GNSS Signal Structure & Baseband Tracking DSP

### Learning Objectives
- Generate and correlate Pseudorandom Noise (PRN) Gold codes for GPS L1 C/A ($1.023\text{ MHz}$) and BeiDou B1I ($2.046\text{ MHz}$).
- Implement Delay Lock Loops (DLL) and Phase Lock Loops (PLL) with Numerically Controlled Oscillators (NCO).
- Master stream-accurate sample counting to avoid host PC scheduling jitter.

### Key Concepts & Equations
1. **Correlation Discriminator (Early - Late Power):**
   $$D_{\text{code}} = \frac{E - L}{E + L}, \quad E = \sqrt{I_E^2 + Q_E^2}, \quad L = \sqrt{I_L^2 + Q_L^2}$$
2. **Costas Phase Discriminator:**
   $$\Delta \phi = \operatorname{atan2}(Q_P, I_P)$$
3. **The Hatch Filter (Code-Carrier Smoothing):**
   $$\hat{\rho}_k = \frac{1}{M}\rho_k + \frac{M-1}{M}\left(\hat{\rho}_{k-1} + \lambda (\Phi_k - \Phi_{k-1})\right)$$

### Practical Repo Exploration
- Inspect the high-performance Rust tracking engine in [`src/live.rs`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/src/live.rs).
- Trace the Hatch filter implementation in [`src/gps/hatch.rs`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/src/gps/hatch.rs).
- Run [`scripts/hatch_divergence_sounder.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/hatch_divergence_sounder.py) to observe code-carrier divergence.

---

## Module 4: Carrier-Phase Interferometry & Integer Ambiguity Resolution

### Learning Objectives
- Formulate Single, Double, and Triple Carrier Differences across satellites and receivers.
- Eliminate receiver clock bias, satellite clock error, and common atmospheric delays.
- Solve integer cycle ambiguities using the LAMBDA method to achieve millimeter positioning.

### Key Concepts & Equations
1. **Carrier Double Difference Observable:**
   $$\Delta \nabla \Phi_{AB}^{jk} = \frac{1}{\lambda}\left(\Delta \nabla \rho_{AB}^{jk}\right) + \Delta \nabla N_{AB}^{jk} + \epsilon_{\Delta \nabla \Phi}$$
2. **Triple Difference (Cycle Slip Detector):**
   $$\delta \Delta \nabla \Phi(t_2, t_1) = \Delta \nabla \Phi(t_2) - \Delta \nabla \Phi(t_1)$$
3. **LAMBDA Z-Transformation:**
   Find integer transformation matrix $\mathbf{Z} \in \mathbb{Z}^{n \times n}$ with $|\det(\mathbf{Z})| = 1$ such that $\mathbf{Q}_{\hat{z}} = \mathbf{Z}^T \mathbf{Q}_{\hat{a}} \mathbf{Z}$ is maximally diagonalized.

### Practical Repo Exploration
- Inspect [`scripts/carrier_double_difference_engine.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/carrier_double_difference_engine.py).
- Inspect [`scripts/carrier_triple_difference_engine.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/carrier_triple_difference_engine.py).
- Study the LAMBDA search algorithm in [`scripts/lambda_ambiguity_resolution_engine.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/lambda_ambiguity_resolution_engine.py).

---

## Module 5: Atmospheric Physics & Remote Sensing

### Learning Objectives
- Derive plasma dispersion in the ionosphere and measure Total Electron Content (TEC).
- Invert Acoustic Gravity Waves (AGWs) and Traveling Ionospheric Disturbances (TIDs).
- Compute tropospheric Zenith Hydrostatic Delay (ZHD) and Zenith Wet Delay (ZWD) to determine Precipitable Water Vapor (PWV).

### Key Concepts & Equations
1. **Ionospheric Phase Advance and Group Delay:**
   $$\Delta \tau_{\text{iono}} = \frac{40.3 \cdot \mathrm{STEC}}{c \cdot f^2}$$
2. **Atmospheric Gravity Wave (AGW) Dispersion:**
   $$\vec{v}_p = \frac{\omega}{\|\vec{k}_h\|} \hat{k}, \quad \lambda_h = \frac{2\pi}{\|\vec{k}_h\|}$$
3. **Saastamoinen Zenith Hydrostatic Delay:**
   $$\mathrm{ZHD} = 0.0022768 \cdot \frac{P_0}{1 - 0.00266\cos(2\phi) - 0.00028 H_0}$$
4. **Bevis Relation for Precipitable Water Vapor (PWV):**
   $$\mathrm{PWV} = \Pi \cdot \mathrm{ZWD}, \quad \Pi = \frac{10^6}{\rho_w R_v (k_2' + k_3/T_m)}$$

### Practical Repo Exploration
- Run [`scripts/agw_tid_wavevector_engine.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/agw_tid_wavevector_engine.py).
- Run [`scripts/gnss_meteorology_pwv.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/gnss_meteorology_pwv.py).
- Observe solar flare detection in [`scripts/solar_flare_sid_monitor.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/solar_flare_sid_monitor.py).

---

## Module 6: Relativistic Gravitation & Astrodynamics

### Learning Objectives
- Model Special Relativistic time dilation and General Relativistic gravitational redshift on satellite clocks.
- Calculate the Sagnac effect caused by Earth's rotation during signal transit.
- Model Earth Solid Body Tides and Solar Radiation Pressure (SRP).

### Key Concepts & Equations
1. **Relativistic Satellite Clock Offset:**
   $$\Delta t_r = -\frac{2\sqrt{\mu a}}{c^2} e \sin E = -\frac{2}{c^2}\vec{r} \cdot \vec{v}$$
2. **Sagnac Effect:**
   $$\Delta \tau_{\text{Sagnac}} = \frac{2\omega_E}{c^2} A_{\text{proj}} = \frac{\omega_E}{c^2}(x_{\text{sat}} y_{\text{rx}} - y_{\text{sat}} x_{\text{rx}})$$
3. **Solid Earth Tide Displacement (Love Numbers):**
   $$\Delta \vec{r}_{\text{tide}} = \sum_{j \in \{\text{Moon, Sun}\}} \frac{G M_j R_E^4}{M_E d_j^3}\left\{ h_2 \hat{r}\left(\frac{3}{2}(\hat{r}\cdot\hat{d}_j)^2 - \frac{1}{2}\right) + 3 l_2 (\hat{r}\cdot\hat{d}_j)\left(\hat{d}_j - (\hat{r}\cdot\hat{d}_j)\hat{r}\right) \right\}$$
4. **Solar Radiation Pressure (Cannon-Milani Acceleration):**
   $$\vec{a}_{\text{SRP}} = -P_\odot \frac{\mathrm{AU}^2}{\|\vec{r}_\odot\|^2} \frac{A}{m} (1 + \eta) \cos\theta \hat{e}_\odot$$

### Practical Repo Exploration
- Run [`scripts/relativistic_space_time_inspector.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/relativistic_space_time_inspector.py).
- Run [`scripts/earth_solid_tide_sounder.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/earth_solid_tide_sounder.py).
- Run [`scripts/solar_radiation_pressure_sounder.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/solar_radiation_pressure_sounder.py).

---

## Module 7: Epistemic Architecture & Evidence Contracts

### Learning Objectives
- Understand why scientific instrumentation requires strict separation between physical observations, calibrations, derived products, and presentations.
- Design machine-verifiable evidence envelopes with provenance hashes and permitted epoch skew.
- Implement simulation quarantine subsystems to prevent synthetic models from contaminating live observational data.

### Key Concepts & Schema
1. **The Four-Layer Epistemic Model:**
   - Layer 1: Immutable Observations (`OBSERVED`)
   - Layer 2: Versioned Calibration Records (`CALIBRATED`)
   - Layer 3: Derived Scientific Products (`DERIVED` / `MODEL`)
   - Layer 4: Presentation Contract (`PRESENTATION`)
2. **Automated Quarantine:**
   Any synthetic simulation model carries `claim_class = "simulation"` and is automatically flagged with `quarantined = true, validity = false`.
3. **Skew Verification:**
   Epoch skew $|t_{\text{gen}} - t_{\text{obs}}| \le \Delta t_{\text{max}}$ prevents stale, invalid state propagation.

### Practical Repo Exploration
- Study [`scripts/evidence_envelope.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/evidence_envelope.py) and [`scripts/test_evidence_envelope.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/test_evidence_envelope.py).
- Review simulation quarantine segregation in [`src/main.rs`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/src/main.rs) and [`scripts/satellite_atomic_clock_analyzer.py`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/scripts/satellite_atomic_clock_analyzer.py).

---

## Module 8: Advanced Antenna Switching Matrix with HackRF Opera Cake

### Learning Objectives
- Mount and interface the HackRF Opera Cake add-on board on top of HackRF Pro.
- Execute microsecond-speed RF matrix switching via digital expansion headers P20/P22.
- Implement polarization diversity for instant multipath isolation (RHCP vs LHCP).
- Build automated ABBA RF delay calibration routines and multi-baseline spatial arrays.

### Key Concepts & Topologies
1. **Switch Matrix Architecture:**
   - 2 primary ports (PA, PB), 8 secondary ports (A1–A4, B1–B4), $1\text{ MHz}\text{--}4000\text{ MHz}$.
2. **Polarimetric Multipath Separation:**
   Direct signals are RHCP; reflected signals reverse to LHCP. Toggling A1 (RHCP) and A3 (LHCP) isolates ground-bounce multipath in $< 5\ \mu\mathrm{s}$.
3. **Automated ABBA Receiver Delay Calibration:**
   $$\text{A (Antenna}\to\text{Pro)} \longrightarrow \text{B (Antenna}\to\text{One)} \longrightarrow \text{B} \longrightarrow \text{A}$$
   Cancels linear temperature and cable drift down to picoseconds.
4. **In-Situ Y-Factor Radiometer Calibration:**
   Switching between the antenna and an internal shielded $50\ \Omega$ termination (Port A4) provides absolute receiver noise temperature $T_{\text{sys}}$ calibration.

### Practical Repo Exploration
- Study [`docs/operacake_integration_plan.md`](file:///Volumes/Radiator%208TB/gnss/hackrf_gnss/docs/operacake_integration_plan.md).
- Control Opera Cake from the terminal: `hackrf_operacake -d <serial> -m manual -p A1`.

---

## Laboratory Assignments & Capstone Challenge

### Lab 1: Calibrate Your SDR's Clock Drift
Using an un-disciplined HackRF TCXO, record 10 minutes of GPS L1 I/Q samples. Track the carrier phase of a single PRN and plot the phase slope. Calculate the exact frequency bias in ppm. Re-run the experiment locked to a GPSDO and verify that the drift vanishes.

### Lab 2: Detect the Solar Terminator Transition
Monitor the code-carrier divergence ($\dot{I}$) or carrier double-difference phase residuals around local sunrise. Identify the exact epoch when solar EUV radiation creates the morning ionospheric ionization ramp.

### Capstone Challenge: Single-Frequency Millimeter Baseline Determination
Using two synchronized HackRFs connected to separate antennas with a known 2-meter physical baseline:
1. Track at least 4 common GPS/BeiDou satellites.
2. Form double-difference carrier phase observations.
3. Solve the integer ambiguities using the LAMBDA engine.
4. Reconstruct the 3D baseline vector $\vec{b} = [X, Y, Z]^T$ and demonstrate sub-centimeter agreement with physical tape-measure ground truth!
