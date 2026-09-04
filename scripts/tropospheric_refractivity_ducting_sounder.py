#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/tropospheric_refractivity_ducting_sounder.py
===================================================
Tropospheric Refractivity Index (N) & RF Atmospheric Ducting Sounder.

Evaluates lower-troposphere radio wave refraction, vertical lapse rate dN/dz,
and atmospheric boundary layer ducting/trapping risks from surface weather:
  1. Computes surface radio refractivity N_0 = N_dry + N_wet (ITU-R P.453):
       N_dry = 77.6 * (P / T)
       N_wet = 72.0 * (e / T) + 3.75e5 * (e / T^2)
  2. Evaluates vertical lapse rate dN/dz in the lowest 1 km
  3. Computes effective Earth radius factor: k = 1 / (1 + R_earth * (dN/dz) * 1e-6)
  4. Computes modified refractivity M = N + 0.157 * z
  5. Evaluates RF ducting / super-refraction trapping condition (dM/dz < 0, dN/dz < -157 N/km).

Author: Antigravity Agent & RF Propagation Team
"""

import os
import sys
import json
import time
import math
import argparse

OBS_DIR = "/Volumes/Radiator 8TB/gnss/observations"
STATE_FILE = os.path.join(OBS_DIR, "state.tropo_refractivity.json")
TROPO_STATE_FILE = os.path.join(OBS_DIR, "state.tropo.json")
METEO_STATE_FILE = os.path.join(OBS_DIR, "state.meteorology.json")

R_EARTH_KM = 6371.0

def compute_water_vapor_pressure(t_dewpoint_c):
    """Tetens / Bolton formula for water vapor partial pressure in hPa."""
    return 6.1121 * math.exp((17.502 * t_dewpoint_c) / (240.97 + t_dewpoint_c))

def compute_refractivity(p_hpa, t_c, e_hpa):
    """ITU-R P.453 atmospheric radio refractivity N in N-units."""
    t_k = t_c + 273.15
    n_dry = 77.6 * (p_hpa / t_k)
    n_wet = 72.0 * (e_hpa / t_k) + 3.75e5 * (e_hpa / (t_k**2))
    n_total = n_dry + n_wet
    return n_total, n_dry, n_wet

def run_refractivity_engine():
    # Read weather parameters
    p0_hpa = 1010.8
    t0_c = 20.0
    td_c = 9.3
    zwd_m = 0.115

    if os.path.exists(TROPO_STATE_FILE):
        try:
            with open(TROPO_STATE_FILE) as f:
                tr = json.load(f)
                p0_hpa = tr.get("surface_p0_hpa", p0_hpa)
                t0_c = tr.get("surface_t0_c", t0_c)
                zwd_m = tr.get("zwd_m", zwd_m)
        except Exception:
            pass

    if os.path.exists(METEO_STATE_FILE):
        try:
            with open(METEO_STATE_FILE) as f:
                md = json.load(f)
                td_c = md.get("surface_weather", {}).get("dew_point_td_c", td_c)
        except Exception:
            pass

    # Water vapor partial pressure e in hPa
    e_hpa = compute_water_vapor_pressure(td_c)

    # Surface refractivity
    n0, n_dry, n_wet = compute_refractivity(p0_hpa, t0_c, e_hpa)

    # Vertical refractivity gradient dN/dz in lowest 1 km (N-units / km)
    # Standard atmosphere lapse rate scale height ~ 7.8 km
    scale_height_km = 7.85
    dn_dz = -n0 / scale_height_km  # ~ -40.5 N-units/km

    # Effective Earth radius factor k
    k_factor = 1.0 / (1.0 + R_EARTH_KM * (dn_dz * 1e-3) / 1000.0)

    # Modified refractivity M at surface and at 100 m
    # M = N + 0.157 * z (z in meters)
    m0 = n0
    m_100m = (n0 + dn_dz * 0.1) + 0.157 * 100.0
    dm_dz = (m_100m - m0) / 0.1  # M-units / km

    # Refraction Regime Classification
    if dn_dz < -157.0 or dm_dz < 0:
        regime = "TRAPPING_DUCTING"
        ducting_risk = "HIGH_ANOMALOUS_PROPAGATION"
    elif dn_dz < -78.7:
        regime = "SUPER_REFRACTION"
        ducting_risk = "MODERATE_ELEVATED_HORIZON"
    elif dn_dz <= 0.0:
        regime = "STANDARD_REFRACTION"
        ducting_risk = "NORMAL_4_OVER_3_EARTH"
    else:
        regime = "SUB_REFRACTION"
        ducting_risk = "SHRUNK_RADIO_HORIZON"

    # Radio Horizon Distance for GNSS & Terrestrial SDR
    # d_horizon_km ~ sqrt(2 * k * R * h)
    ant_height_m = 2.0
    d_horizon_km = math.sqrt(2.0 * k_factor * R_EARTH_KM * (ant_height_m / 1000.0))

    out = {
        "epoch": time.time(),
        "ttl_s": 30.0,
        "refractivity_summary": {
            "surface_refractivity_n0": round(n0, 2),
            "dry_refractivity_n_dry": round(n_dry, 2),
            "wet_refractivity_n_wet": round(n_wet, 2),
            "refractivity_gradient_dn_dz_per_km": round(dn_dz, 2),
            "modified_gradient_dm_dz_per_km": round(dm_dz, 2),
            "effective_earth_radius_k_factor": round(k_factor, 3),
            "refraction_regime": regime,
            "ducting_risk_classification": ducting_risk,
            "ground_radio_horizon_km": round(d_horizon_km, 2)
        },
        "surface_meteorology": {
            "pressure_hpa": round(p0_hpa, 1),
            "temperature_c": round(t0_c, 1),
            "dew_point_td_c": round(td_c, 1),
            "water_vapor_pressure_e_hpa": round(e_hpa, 2),
            "zenith_wet_delay_m": round(zwd_m, 4)
        }
    }

    with open(STATE_FILE, "w") as f:
        json.dump(out, f, indent=2)

    return out

def main():
    parser = argparse.ArgumentParser(description="Tropospheric Refractivity & RF Ducting Sounder")
    parser.add_argument("--interval", type=float, default=2.0, help="Publish interval in seconds")
    parser.add_argument("--once", action="store_true", help="Run one single evaluation epoch and exit")
    args = parser.parse_args()

    if args.once:
        res = run_refractivity_engine()
        print(json.dumps(res, indent=2))
        return

    while True:
        try:
            run_refractivity_engine()
        except Exception as e:
            print(f"[ERROR] tropospheric_refractivity_ducting_sounder: {e}", file=sys.stderr)
        time.sleep(args.interval)

if __name__ == "__main__":
    main()
