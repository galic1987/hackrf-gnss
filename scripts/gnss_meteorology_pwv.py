#!/usr/bin/env python3
"""GNSS Meteorology & Precipitable Water Vapor (PWV) Inversion Daemon.

Inverts GNSS Tropospheric Zenith Wet Delay (ZWD) into integrated atmospheric
Precipitable Water Vapor (PWV / IPWV) via the Bevis et al. (1992, 1994) formulation:

1. Weighted atmospheric mean temperature:
   Tm = 70.2 + 0.72 * Ts  [K]

2. Dimensionless water vapor conversion factor:
   Pi = 10^6 / (rho_w * Rv * (k3 / Tm + k2'))  ~ 0.159

3. Integrated Precipitable Water Vapor:
   PWV = Pi * ZWD  [mm of liquid water equivalent, or kg/m^2]

4. Line-of-sight Slant Water Vapor per satellite:
   SWV_i = PWV * m_w(E_i)  [mm]

5. Atmospheric thermodynamics:
   Saturation vapor pressure (e_s), actual vapor pressure (e_0), dew point (T_d),
   water vapor mass density (g/cm^2), and convective stability regime classification.

Publishes state to:
  /Volumes/Radiator 8TB/gnss/observations/state.meteorology.json
"""

import argparse
import json
import math
import os
import sys
import time

# Physical Constants
RV = 461.524               # Specific gas constant for water vapor (J / (kg * K))
RHO_W = 1000.0             # Density of liquid water (kg / m^3)
K3 = 3.776e5               # Refractivity constant (K^2 / hPa) (Bevis et al., 1994)
K2_PRIME = 22.1            # Refractivity constant k2' (K / hPa)

def compute_bevis_tm(surface_t_c):
    """Compute atmospheric weighted mean temperature Tm (Kelvin) from surface temperature Ts."""
    ts_k = surface_t_c + 273.15
    return 70.2 + 0.72 * ts_k

def compute_bevis_pi(tm_k):
    """Compute the dimensionless Bevis conversion factor Pi."""
    # 1 hPa = 100 Pa = 100 J/m^3 -> factor of 1e-2 to convert K/hPa to K*m^3/J
    term = (K3 / tm_k + K2_PRIME) * 1e-2
    denom = 1e-6 * RHO_W * RV * term
    return 1.0 / denom

def compute_pwv_mm(zwd_m, tm_k):
    """Compute Integrated Precipitable Water Vapor (PWV in mm = kg/m^2) from ZWD in meters."""
    pi = compute_bevis_pi(tm_k)
    zwd_mm = zwd_m * 1000.0
    return pi * zwd_mm

def compute_vapor_pressures_and_dewpoint(t_c, rh_pct):
    """Compute saturation vapor pressure es (hPa), actual vapor pressure e0 (hPa),

    and dewpoint temperature Td (deg C) using the Magnus-Tetens formula.
    """
    es = 6.112 * math.exp((17.67 * t_c) / (t_c + 243.5))
    e0 = max(0.01, (rh_pct / 100.0) * es)
    ln_e = math.log(e0 / 6.112)
    td = (243.5 * ln_e) / (17.67 - ln_e)
    return es, e0, td

def compute_slant_water_vapor(pwv_mm, map_wet):
    """Compute Slant Water Vapor (SWV in mm) along the satellite line-of-sight."""
    return pwv_mm * map_wet

def classify_convective_regime(pwv_mm):
    """Classify convective moisture regime based on mid-latitude meteorological thresholds."""
    if pwv_mm < 15.0:
        return "DRY_STABLE"
    elif pwv_mm < 30.0:
        return "NORMAL"
    elif pwv_mm < 45.0:
        return "HUMID_CONVECTIVE"
    else:
        return "SEVERE_STORM_POTENTIAL"

def evaluate_meteorology(tropo_state):
    """Synthesize complete meteorological and water vapor state from tropospheric parameters."""
    t_c = tropo_state.get("surface_t0_c", 20.0)
    p0_hpa = tropo_state.get("surface_p0_hpa", 1010.8)
    rh_pct = tropo_state.get("surface_rh_pct", 50.0)
    zwd_m = tropo_state.get("zwd_m", 0.115)
    zhd_m = tropo_state.get("zhd_m", 2.303)
    ztd_m = tropo_state.get("ztd_m", 2.418)

    tm_k = compute_bevis_tm(t_c)
    pi_factor = compute_bevis_pi(tm_k)
    pwv_mm = compute_pwv_mm(zwd_m, tm_k)
    pwv_kg_m2 = pwv_mm  # 1 mm of water depth over 1 m^2 is exactly 1 kg
    pwv_g_cm2 = pwv_mm * 0.1  # 1 mm = 0.1 g/cm^2

    es_hpa, e0_hpa, td_c = compute_vapor_pressures_and_dewpoint(t_c, rh_pct)
    regime = classify_convective_regime(pwv_mm)

    sat_met = {}
    swv_list = []
    tropo_sats = tropo_state.get("tropo_satellites", {})

    for sat_key, sdata in tropo_sats.items():
        map_wet = sdata.get("map_wet", 1.0)
        swv_mm = compute_slant_water_vapor(pwv_mm, map_wet)
        swv_list.append(swv_mm)
        sat_met[sat_key] = {
            "sys": sdata.get("sys"),
            "prn": sdata.get("prn"),
            "el_deg": sdata.get("el_deg"),
            "map_wet": round(map_wet, 3),
            "slant_wet_m": sdata.get("slant_wet_m", round(zwd_m * map_wet, 2)),
            "swv_mm": round(swv_mm, 2),
            "swv_kg_m2": round(swv_mm, 2),
            "cn0": sdata.get("cn0")
        }

    max_swv = max(swv_list) if swv_list else pwv_mm

    return {
        "epoch": tropo_state.get("epoch", time.time()),
        "ttl_s": 30.0,
        "station_lat": tropo_state.get("station_lat", 39.0029),
        "station_lon": tropo_state.get("station_lon", -77.6058),
        "station_height_m": tropo_state.get("station_height_m", 20.0),
        "meteorology_summary": {
            "pwv_mm": round(pwv_mm, 2),
            "pwv_kg_m2": round(pwv_kg_m2, 2),
            "pwv_g_cm2": round(pwv_g_cm2, 3),
            "bevis_tm_k": round(tm_k, 2),
            "bevis_tm_c": round(tm_k - 273.15, 2),
            "bevis_pi_factor": round(pi_factor, 5),
            "surface_temp_c": round(t_c, 1),
            "surface_pressure_hpa": round(p0_hpa, 1),
            "surface_rh_pct": round(rh_pct, 1),
            "sat_vapor_pressure_hpa": round(es_hpa, 2),
            "actual_vapor_pressure_hpa": round(e0_hpa, 2),
            "dew_point_c": round(td_c, 2),
            "convective_regime": regime,
            "max_slant_wv_mm": round(max_swv, 2),
            "zwd_m": round(zwd_m, 3),
            "zhd_m": round(zhd_m, 3),
            "ztd_m": round(ztd_m, 3),
            "n_sats_swv": len(sat_met)
        },
        "meteorology_satellites": sat_met
    }

def run_meteorology_cycle():
    """Execute one meteorology cycle and write state.meteorology.json atomically."""
    tropo_file = "/Volumes/Radiator 8TB/gnss/observations/state.tropo.json"
    tropo_state = {}
    if os.path.exists(tropo_file):
        try:
            with open(tropo_file) as f:
                tropo_state = json.load(f)
        except Exception:
            pass

    out = evaluate_meteorology(tropo_state)

    target_path = "/Volumes/Radiator 8TB/gnss/observations/state.meteorology.json"
    tmp_path = target_path + f".tmp.{os.getpid()}"
    with open(tmp_path, "w") as f:
        json.dump(out, f, indent=2)
    os.replace(tmp_path, target_path)

    return out

def main():
    parser = argparse.ArgumentParser(description="GNSS Meteorology & Precipitable Water Vapor (PWV) Inversion Daemon")
    parser.add_argument("--loop", action="store_true", help="Run continuously in a loop")
    parser.add_argument("--interval", type=float, default=2.0, help="Loop poll interval in seconds (default: 2.0)")
    parser.add_argument("--once", action="store_true", help="Run a single evaluation cycle")
    args = parser.parse_args()

    print(f"Starting GNSS Meteorology & PWV Inversion (interval={args.interval}s, loop={args.loop})...")

    while True:
        try:
            state = run_meteorology_cycle()
            summ = state["meteorology_summary"]
            print(f"[{time.strftime('%H:%M:%S')}] Meteorology: PWV={summ['pwv_mm']:.2f} mm ({summ['pwv_kg_m2']:.2f} kg/m^2) | "
                  f"Tm={summ['bevis_tm_c']:.1f} C, Td={summ['dew_point_c']:.1f} C, RH={summ['surface_rh_pct']:.0f}% | "
                  f"Regime: {summ['convective_regime']} | Max SWV={summ['max_slant_wv_mm']:.1f} mm across {summ['n_sats_swv']} sats")
        except Exception as e:
            print(f"Error in meteorology cycle: {e}", file=sys.stderr)

        if not args.loop or args.once:
            break
        time.sleep(args.interval)

if __name__ == "__main__":
    main()
