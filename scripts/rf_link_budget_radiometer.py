#!/usr/bin/env python3
"""RF Radiometry & Link Budget Metrology Daemon.

Analyzes live GNSS signal reception, link margins, Free Space Path Loss (FSPL),
atmospheric absorption, and equivalent antenna noise temperature:

1. Free Space Path Loss (FSPL):
   FSPL = 20*log10(d) + 20*log10(f) - 147.55  [dB]

2. ITU-R P.676 Atmospheric Gaseous Absorption:
   Slant path absorption through oxygen & water vapor lines at L-band (~0.04-0.5 dB).

3. Receiver Thermal Noise & System Temperature (T_sys):
   Inverts observed C/N0 into equivalent system noise temperature T_sys and
   noise spectral density N0 = -174 + 10*log10(T_sys/290) [dBm/Hz].

4. Link Margin:
   Evaluates headroom above demodulation/tracking threshold (28 dB-Hz).

5. RFI / Jamming Threat Monitoring:
   Detects elevated noise floor or intentional out-of-band interference.

Publishes state to:
  /Volumes/Radiator 8TB/gnss/observations/state.radiometry.json
"""

import argparse
import json
import math
import os
import sys
import time

# Physical & RF Constants
C_LIGHT = 299792458.0              # Speed of light (m/s)
KB_BOLTZMANN = 1.380649e-23        # Boltzmann's constant (J/K)
R_EARTH = 6378137.0               # Earth radius (m)

F_L1_GPS = 1575.42e6              # GPS L1 (Hz)
F_B1I_BDS = 1561.098e6            # BeiDou B1I (Hz)
F_E1_GAL = 1575.42e6              # Galileo E1 (Hz)

TRACKING_THRESHOLD_CN0 = 28.0     # Demodulation tracking threshold (dB-Hz)
NOMINAL_EIRP_DBM = 58.5           # ~28.5 dBW EIRP at edge of Earth (~700 W ERP)
ACTIVE_ANT_PREAMP_GAIN_DB = 28.0  # Active patch preamplifier gain (dB)
LNA_NOISE_FIGURE_DB = 1.2         # Active antenna LNA noise figure (dB)
CABLE_LOSS_DB = 3.5               # Low-loss RG-58 / SMA run loss (dB)
ANT_ZENITH_PASSIVE_GAIN_DBI = 3.5 # Patch antenna gain at zenith (dBi)

SDR_IMPLEMENTATION_LOSS_DB = 4.0  # HackRF ADC quantization, baseband filtering, tracking loop loss
SURFACE_P_RX_MIN_DBM = -128.5     # ICD-200 guaranteed minimum received power at Earth surface (0 dBi)

ORBIT_ALTITUDE_MAP = {
    "gps": 20200e3,
    "galileo": 23222e3,
    "beidou": 21528e3,
    "glonass": 19100e3,
    "sbas": 35786e3
}

def compute_slant_distance_m(el_deg, altitude_m=20200e3):
    """Compute geometric slant distance d (meters) from elevation angle E and orbital altitude."""
    el = math.radians(max(0.0, min(90.0, el_deg)))
    re = R_EARTH
    h = altitude_m
    # Law of cosines topocentric triangle
    d = -re * math.sin(el) + math.sqrt((re * math.sin(el))**2 + 2.0 * re * h + h**2)
    return d

def compute_fspl_db(d_meters, f_hz):
    """Compute Free Space Path Loss (dB)."""
    d = max(1.0, d_meters)
    f = max(1.0, f_hz)
    return 20.0 * math.log10(d) + 20.0 * math.log10(f) - 147.5522

def compute_atm_absorption_db(el_deg):
    """Compute atmospheric oxygen and water vapor absorption loss (dB) via ITU-R P.676 at L-band."""
    el_rad = math.radians(max(1.0, min(90.0, el_deg)))
    a_zenith = 0.04  # ~0.04 dB at 1.5 GHz zenith
    denom = math.sin(el_rad) + 0.00143 / (math.tan(el_rad) + 0.0445)
    return a_zenith / denom

def compute_noise_spectral_density_dbm_hz(t_sys_k):
    """Compute thermal noise spectral density N0 (dBm/Hz) from system temperature Tsys."""
    t = max(1.0, t_sys_k)
    return -174.0 + 10.0 * math.log10(t / 290.0)

def compute_tsys_from_n0(n0_dbm_hz):
    """Invert thermal noise spectral density N0 (dBm/Hz) into system temperature Tsys (K)."""
    ratio = 10.0 ** ((n0_dbm_hz + 174.0) / 10.0)
    return ratio * 290.0

def compute_link_margin_db(cn0_db_hz, threshold_db_hz=TRACKING_THRESHOLD_CN0):
    """Compute carrier-to-noise link margin (dB) above receiver demodulation threshold."""
    return round(cn0_db_hz - threshold_db_hz, 1)

def classify_rfi_threat(t_sys_k):
    """Classify local RF noise and interference threat level."""
    if t_sys_k < 450.0:
        return "QUIET"
    elif t_sys_k < 750.0:
        return "ELEVATED_NOISE"
    else:
        return "RFI_INTERFERENCE"

def evaluate_radiometry(tracked_sats):
    """Synthesize complete RF link budget and radiometry state from tracked satellite channels."""
    sat_results = {}
    fspl_list = []
    margin_list = []
    cn0_list = []
    tsys_list = []

    for ts in tracked_sats:
        sys_str = ts.get("sys", "gps").lower()
        prn = ts.get("prn")
        if not prn:
            continue
        key = f"{sys_str.upper()}_{prn}"

        el = ts.get("el_deg", 45.0)
        az = ts.get("az_deg", 0.0)
        cn0 = ts.get("cn0_proxy", ts.get("cn0", 35.0))
        if cn0 is None or cn0 <= 0:
            continue

        f_carrier = F_B1I_BDS if "beidou" in sys_str or "bds" in sys_str else F_L1_GPS
        alt_m = ORBIT_ALTITUDE_MAP.get(sys_str, 20200e3)
        dist_m = compute_slant_distance_m(el, alt_m)

        fspl_db = compute_fspl_db(dist_m, f_carrier)
        atm_loss_db = compute_atm_absorption_db(el)

        # Realistic GNSS patch antenna elevation gain pattern:
        # ~ +3.5 dBi at zenith, smoothly transitioning to -2.0 dBi at 10 deg
        sin_el = math.sin(math.radians(max(5.0, el)))
        g_rx_dbi = 3.5 * sin_el - 3.0 * (1.0 - sin_el)

        # Expected incident carrier power at antenna element (dBm)
        p_ant_expected_dbm = SURFACE_P_RX_MIN_DBM + g_rx_dbi - atm_loss_db

        # Observed N0 = Pant - (C/N0) - L_impl
        n0_observed = p_ant_expected_dbm - cn0 - SDR_IMPLEMENTATION_LOSS_DB
        t_sys = compute_tsys_from_n0(n0_observed)
        # Clamp to realistic physical bounds [120 K, 1500 K]
        t_sys_clamped = max(120.0, min(1500.0, t_sys))

        margin_db = compute_link_margin_db(cn0)

        sat_results[key] = {
            "sys": sys_str,
            "prn": prn,
            "az_deg": round(az, 1),
            "el_deg": round(el, 1),
            "distance_km": round(dist_m / 1e3, 1),
            "fspl_db": round(fspl_db, 2),
            "atm_loss_db": round(atm_loss_db, 2),
            "p_rx_dbm": round(p_ant_expected_dbm, 2),
            "cn0_db_hz": round(cn0, 1),
            "link_margin_db": margin_db,
            "t_sys_k": round(t_sys_clamped, 1),
            "lock_s": ts.get("lock_s", 0.0)
        }

        fspl_list.append(fspl_db)
        margin_list.append(margin_db)
        cn0_list.append(cn0)
        tsys_list.append(t_sys_clamped)

    mean_fspl = sum(fspl_list) / len(fspl_list) if fspl_list else 183.2
    mean_margin = sum(margin_list) / len(margin_list) if margin_list else 12.0
    mean_cn0 = sum(cn0_list) / len(cn0_list) if cn0_list else 38.0

    # Station benchmark Tsys is computed from high-elevation satellites (el >= 40 deg)
    # to avoid horizon multipath and obstruction attenuation
    zenith_sats = [s["t_sys_k"] for s in sat_results.values() if s["el_deg"] >= 40.0]
    if zenith_sats:
        mean_tsys = sum(zenith_sats) / len(zenith_sats)
    elif tsys_list:
        mean_tsys = sum(tsys_list) / len(tsys_list)
    else:
        mean_tsys = 295.0

    min_tsys = min(tsys_list) if tsys_list else 195.0
    rfi_threat = classify_rfi_threat(min_tsys)
    mean_n0 = compute_noise_spectral_density_dbm_hz(min_tsys)

    # Antenna sky temp: T_A ~ T_sys - T_LNA ~ T_sys - 92 K
    t_lna = 290.0 * (10.0 ** (LNA_NOISE_FIGURE_DB / 10.0) - 1.0)  # ~92.2 K
    antenna_sky_temp = max(20.0, min_tsys - t_lna)

    return {
        "epoch": time.time(),
        "ttl_s": 30.0,
        "radiometry_summary": {
            "mean_tsys_k": round(mean_tsys, 1),
            "min_tsys_k": round(min_tsys, 1),
            "mean_n0_dbm_hz": round(mean_n0, 2),
            "mean_fspl_db": round(mean_fspl, 2),
            "mean_link_margin_db": round(mean_margin, 1),
            "mean_cn0_db_hz": round(mean_cn0, 1),
            "antenna_sky_temp_k": round(antenna_sky_temp, 1),
            "lna_noise_temp_k": round(t_lna, 1),
            "lna_noise_figure_db": LNA_NOISE_FIGURE_DB,
            "rfi_threat_level": rfi_threat,
            "tracking_threshold_db_hz": TRACKING_THRESHOLD_CN0,
            "n_sats_radiometry": len(sat_results)
        },
        "radiometry_satellites": sat_results
    }

def run_radiometry_cycle():
    """Execute one radiometry cycle and write state.radiometry.json atomically."""
    # Poll tracker state and tropo state for accurate elevations
    tracker_file = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json"
    tropo_file = "/Volumes/Radiator 8TB/gnss/observations/state.tropo.json"

    tracked_sats = []
    el_map = {}

    if os.path.exists(tropo_file):
        try:
            with open(tropo_file) as f:
                tr_data = json.load(f)
                for k, v in tr_data.get("tropo_satellites", {}).items():
                    el_map[k] = (v.get("el_deg"), v.get("az_deg"))
        except Exception:
            pass

    if os.path.exists(tracker_file):
        try:
            with open(tracker_file) as f:
                tdata = json.load(f)
                raw_sats = tdata.get("tracker", {}).get("sats", [])
                for rs in raw_sats:
                    sys_name = rs.get("sys", "gps").lower()
                    prn = rs.get("prn")
                    sat_key = f"{sys_name.upper()}_{prn}"
                    el_az = el_map.get(sat_key, (45.0, 0.0))
                    rs["el_deg"] = el_az[0] if el_az[0] is not None else 45.0
                    rs["az_deg"] = el_az[1] if el_az[1] is not None else 0.0
                    tracked_sats.append(rs)
        except Exception:
            pass

    out = evaluate_radiometry(tracked_sats)

    target_path = "/Volumes/Radiator 8TB/gnss/observations/state.radiometry.json"
    tmp_path = target_path + f".tmp.{os.getpid()}"
    with open(tmp_path, "w") as f:
        json.dump(out, f, indent=2)
    os.replace(tmp_path, target_path)

    return out

def main():
    parser = argparse.ArgumentParser(description="RF Radiometry & Link Budget Metrology Daemon")
    parser.add_argument("--loop", action="store_true", help="Run continuously in a loop")
    parser.add_argument("--interval", type=float, default=2.0, help="Loop poll interval in seconds (default: 2.0)")
    parser.add_argument("--once", action="store_true", help="Run a single evaluation cycle")
    args = parser.parse_args()

    print(f"Starting RF Radiometry & Link Budget Analyzer (interval={args.interval}s, loop={args.loop})...")

    while True:
        try:
            state = run_radiometry_cycle()
            summ = state["radiometry_summary"]
            print(f"[{time.strftime('%H:%M:%S')}] Radiometry: Tsys={summ['mean_tsys_k']:.1f} K (N0={summ['mean_n0_dbm_hz']:.1f} dBm/Hz) | "
                  f"FSPL={summ['mean_fspl_db']:.1f} dB, Margin=+{summ['mean_link_margin_db']:.1f} dB (C/N0={summ['mean_cn0_db_hz']:.1f} dB-Hz) | "
                  f"Threat: {summ['rfi_threat_level']} | Across {summ['n_sats_radiometry']} sats")
        except Exception as e:
            print(f"Error in radiometry cycle: {e}", file=sys.stderr)

        if not args.loop or args.once:
            break
        time.sleep(args.interval)

if __name__ == "__main__":
    main()
