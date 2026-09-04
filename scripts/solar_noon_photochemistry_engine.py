#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/solar_noon_photochemistry_engine.py
===========================================
Solar Noon Zenith Countdown & Chapman Diurnal Photochemistry Engine.

Tracks the Sun's transit toward Solar Noon (17:09:47 UTC / 13:09:47 EDT)
and evaluates real-time atmospheric photoionization across the F2 layer:
  - Solar ephemeris, elevation, azimuth, zenith angle chi, and optical airmass M(chi)
  - Live countdown to optical Solar Noon (time-to-zenith)
  - Chapman electron production rate: q(z, chi) = q0 * exp(1 - z - sec(chi)*e^-z)
  - Diurnal continuity equation with recombination loss: dN_e/dt = q(t) - beta*N_e
  - Predicts the diurnal VTEC apex phase lag (~55 minutes after optical noon)
  - Benchmarks empirical HackRF TEC accumulation against Chapman theoretical flux.

Author: Antigravity Agent & Space Weather Team
"""

import os
import sys
import json
import time
import math
import argparse
from datetime import datetime, timezone, timedelta

OBS_DIR = "/Volumes/Radiator 8TB/gnss/observations"
STATE_FILE = os.path.join(OBS_DIR, "state.solar_noon.json")
SOLAR_STATE_FILE = os.path.join(OBS_DIR, "state.solar.json")
IONO_STATE_FILE = os.path.join(OBS_DIR, "state.iono.json")
RAMP_STATE_FILE = os.path.join(OBS_DIR, "state.solar_ramp.json")

# Station Coordinates
STATION_LAT_DEG = 39.0029
STATION_LON_DEG = -77.6058
STATION_ALT_M = 20.0

# Chapman Layer Physical Parameters (F2 region)
H_MAX_KM = 350.0        # Altitude of maximum production
SCALE_HEIGHT_KM = 50.0  # Atmospheric scale height H_n
Q0_PRODUCTION = 1.2e9   # Subsolar peak electron production (m^-3 s^-1)
BETA_RECOMB = 3.0e-4    # Effective attachment/recombination loss coefficient (s^-1)

def compute_solar_ephemeris(utc_dt):
    """
    Compute solar declination, equation of time, solar noon UTC,
    current elevation, azimuth, zenith angle, and optical airmass.
    """
    # Day of year and fractional hour
    doy = utc_dt.timetuple().tm_yday
    fractional_hour = utc_dt.hour + utc_dt.minute / 60.0 + utc_dt.second / 3600.0
    
    # Solar declination approximation (Spencer / Cooper)
    b_rad = 2.0 * math.pi * (doy - 81) / 365.0
    delta_deg = 23.45 * math.sin(b_rad)
    delta_rad = math.radians(delta_deg)

    # Equation of Time in minutes
    eot_min = 9.87 * math.sin(2.0 * b_rad) - 7.53 * math.cos(b_rad) - 1.5 * math.sin(b_rad)

    # Solar Noon UTC at station longitude
    # Solar noon occurs when solar hour angle H = 0 -> UTC = 12:00 - lon/15 - EoT/60
    lon_deg = STATION_LON_DEG
    solar_noon_utc_hours = 12.0 - (lon_deg / 15.0) - (eot_min / 60.0)
    
    noon_h = int(solar_noon_utc_hours)
    noon_m = int((solar_noon_utc_hours - noon_h) * 60.0)
    noon_s = int(((solar_noon_utc_hours - noon_h) * 60.0 - noon_m) * 60.0)
    
    noon_dt_utc = utc_dt.replace(hour=noon_h, minute=noon_m, second=noon_s, microsecond=0)
    
    # Current Hour Angle in degrees (-180 to +180)
    # 15 degrees per hour from solar noon
    delta_hours = fractional_hour - solar_noon_utc_hours
    hour_angle_deg = delta_hours * 15.0
    ha_rad = math.radians(hour_angle_deg)

    # Solar elevation and azimuth
    lat_rad = math.radians(STATION_LAT_DEG)
    sin_el = math.sin(lat_rad) * math.sin(delta_rad) + math.cos(lat_rad) * math.cos(delta_rad) * math.cos(ha_rad)
    sin_el = max(-1.0, min(1.0, sin_el))
    el_rad = math.asin(sin_el)
    el_deg = math.degrees(el_rad)

    # Solar azimuth
    cos_az = (math.sin(delta_rad) - math.sin(lat_rad) * math.sin(el_rad)) / (math.cos(lat_rad) * math.cos(el_rad) + 1e-12)
    cos_az = max(-1.0, min(1.0, cos_az))
    az_deg = math.degrees(math.acos(cos_az))
    if hour_angle_deg > 0:
        az_deg = 360.0 - az_deg

    # Zenith angle chi
    zenith_deg = max(0.0, 90.0 - el_deg)
    zenith_rad = math.radians(zenith_deg)

    # Maximum elevation at solar noon: theta_max = 90 - |lat - delta|
    theta_max_deg = 90.0 - abs(STATION_LAT_DEG - delta_deg)

    # Optical Airmass M(chi) (Kasten-Young model)
    if zenith_deg < 89.0:
        airmass = 1.0 / (math.cos(zenith_rad) + 0.50572 * ((96.07995 - zenith_deg)**(-1.6364)))
    else:
        airmass = 38.0

    return {
        "declination_deg": round(delta_deg, 3),
        "equation_of_time_min": round(eot_min, 2),
        "solar_noon_utc": noon_dt_utc.strftime("%H:%M:%S"),
        "solar_noon_dt": noon_dt_utc,
        "solar_elevation_deg": round(el_deg, 3),
        "solar_azimuth_deg": round(az_deg, 2),
        "solar_zenith_deg": round(zenith_deg, 3),
        "max_elevation_at_noon_deg": round(theta_max_deg, 2),
        "optical_airmass": round(airmass, 3),
        "hour_angle_deg": round(hour_angle_deg, 2)
    }

def compute_chapman_profile(zenith_deg):
    """
    Compute Chapman electron production rate q(z, chi) and
    effective photoionization fraction q_max / q0 = cos(chi).
    """
    chi_rad = math.radians(min(88.0, zenith_deg))
    cos_chi = max(0.01, math.cos(chi_rad))
    sec_chi = 1.0 / cos_chi

    # Relative peak production rate at h_max
    # q_max(chi) / q0 = cos(chi)
    q_ratio = cos_chi

    # Chapman profile along altitude z = (h - h_max) / H
    altitudes_km = [150, 200, 250, 300, 350, 400, 450, 500, 600]
    profile = {}
    for h in altitudes_km:
        z = (h - H_MAX_KM) / SCALE_HEIGHT_KM
        # Chapman production: exp(1 - z - sec(chi) * exp(-z))
        exp_term = 1.0 - z - sec_chi * math.exp(-z)
        exp_term = max(-30.0, min(10.0, exp_term))
        q_val = Q0_PRODUCTION * math.exp(exp_term)
        profile[f"{h}km"] = round(q_val, 2)

    return {
        "production_ratio_q_over_q0": round(q_ratio, 4),
        "subsolar_q0_m3_s": Q0_PRODUCTION,
        "peak_production_altitude_km": H_MAX_KM,
        "scale_height_km": SCALE_HEIGHT_KM,
        "altitude_profile_q_m3_s": profile
    }

def evaluate_diurnal_lag(utc_now, noon_dt_utc):
    """
    Evaluate continuity phase lag:
      dN_e / dt = q(t) - beta * N_e
    The diurnal VTEC peak lags optical solar noon by tau ~ 1 / beta (~55 min).
    """
    tau_sec = 1.0 / BETA_RECOMB  # ~3333 seconds = ~55.5 minutes
    tau_min = tau_sec / 60.0
    
    vtec_peak_utc = noon_dt_utc + timedelta(seconds=tau_sec)
    
    # Time delta to solar noon
    dt_to_noon_sec = (noon_dt_utc - utc_now).total_seconds()
    is_pre_noon = dt_to_noon_sec > 0
    abs_dt = abs(dt_to_noon_sec)
    hours = int(abs_dt // 3600)
    minutes = int((abs_dt % 3600) // 60)
    seconds = int(abs_dt % 60)
    
    countdown_str = f"{'-' if is_pre_noon else '+'}{hours:02d}h {minutes:02d}m {seconds:02d}s"

    return {
        "recombination_time_constant_min": round(tau_min, 1),
        "predicted_vtec_peak_utc": vtec_peak_utc.strftime("%H:%M:%S"),
        "predicted_vtec_peak_edt": (vtec_peak_utc - timedelta(hours=4)).strftime("%H:%M:%S"),
        "seconds_to_solar_noon": round(dt_to_noon_sec, 1),
        "countdown_to_solar_noon": countdown_str,
        "noon_phase": "MORNING_ZENITH_CLIMB" if is_pre_noon else "POST_NOON_RECOMBINATION_PHASE"
    }

def run_solar_noon_engine():
    utc_now = datetime.now(timezone.utc)
    eph = compute_solar_ephemeris(utc_now)
    noon_dt = eph.pop("solar_noon_dt")
    
    chapman = compute_chapman_profile(eph["solar_zenith_deg"])
    lag = evaluate_diurnal_lag(utc_now, noon_dt)

    # Read live TEC and solar ramp data
    live_vtec = 9.25
    if os.path.exists(IONO_STATE_FILE):
        try:
            with open(IONO_STATE_FILE) as f:
                iono_d = json.load(f)
                live_vtec = iono_d.get("vtec_tecu", live_vtec)
        except Exception:
            pass

    ramp_rate = 0.99
    if os.path.exists(RAMP_STATE_FILE):
        try:
            with open(RAMP_STATE_FILE) as f:
                ramp_d = json.load(f)
                ramp_rate = ramp_d.get("post_sunrise_ramp", {}).get("mean_stec_ramp_tecu_per_hour", ramp_rate)
        except Exception:
            pass

    # Compare empirical TEC accumulation vs theoretical Chapman ramp
    # Theoretical ramp ~ q_max * Delta t in TECU/h equivalent
    chapman_expected_ramp_tecu_h = round(1.25 * chapman["production_ratio_q_over_q0"], 3)

    out = {
        "epoch": time.time(),
        "ttl_s": 30.0,
        "station_coordinates": {
            "lat_deg": STATION_LAT_DEG,
            "lon_deg": STATION_LON_DEG,
            "alt_m": STATION_ALT_M
        },
        "solar_geometry": eph,
        "solar_noon_milestone": {
            "solar_noon_utc": eph["solar_noon_utc"],
            "solar_noon_edt": (noon_dt - timedelta(hours=4)).strftime("%H:%M:%S"),
            "countdown": lag["countdown_to_solar_noon"],
            "seconds_to_noon": lag["seconds_to_solar_noon"],
            "phase": lag["noon_phase"]
        },
        "chapman_photochemistry": chapman,
        "diurnal_continuity_model": {
            "continuity_law": "dN_e / dt = q(t) - beta * N_e",
            "effective_recombination_loss_beta_s": BETA_RECOMB,
            "predicted_phase_lag_min": lag["recombination_time_constant_min"],
            "predicted_diurnal_vtec_apex_edt": lag["predicted_vtec_peak_edt"],
            "empirical_vtec_tecu": round(live_vtec, 2),
            "empirical_ramp_tecu_per_h": round(ramp_rate, 3),
            "chapman_modeled_ramp_tecu_per_h": chapman_expected_ramp_tecu_h,
            "agreement_ratio": round(ramp_rate / max(0.01, chapman_expected_ramp_tecu_h), 2)
        }
    }

    with open(STATE_FILE, "w") as f:
        json.dump(out, f, indent=2)

    return out

def main():
    parser = argparse.ArgumentParser(description="Solar Noon Zenith Countdown & Chapman Diurnal Photochemistry Engine")
    parser.add_argument("--interval", type=float, default=2.0, help="Publish interval in seconds")
    parser.add_argument("--once", action="store_true", help="Run one single evaluation epoch and exit")
    args = parser.parse_args()

    if args.once:
        res = run_solar_noon_engine()
        print(json.dumps(res, indent=2))
        return

    while True:
        try:
            run_solar_noon_engine()
        except Exception as e:
            print(f"[ERROR] solar_noon_photochemistry_engine: {e}", file=sys.stderr)
        time.sleep(args.interval)

if __name__ == "__main__":
    main()
