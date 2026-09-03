#!/usr/bin/env python3
"""Tropospheric Neutral Atmosphere Path Delay Model (Saastamoinen & Niell Mapping).

Calculates real-time neutral atmospheric path delay for all tracked satellites:
  1. Computes Zenith Hydrostatic Delay (ZHD) via Saastamoinen (1972) equation.
  2. Computes Zenith Wet Delay (ZWD) from water vapor pressure and surface temperature.
  3. Projects zenith delays along satellite line-of-sight using Niell (1996) mapping functions.
  4. Fuses tropospheric delay with ionospheric delay to provide the Total Atmospheric Delay.
  5. Publishes observations/state.tropo.json atomically for live fusion into /api/sync.
"""
import argparse
import json
import math
import os
import sys
import time

C_MPS = 299792458.0
STATE_IONO_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.iono.json"
STATE_KLOB_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.klobuchar.json"
STATE_TROPO_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.tropo.json"

# Station site anchor (site.json): 39.0029°N, -77.6058°W, 20m ellipsoidal height
DEFAULT_LAT_DEG = 39.0029
DEFAULT_LON_DEG = -77.6058
DEFAULT_HEIGHT_M = 20.0


def barometric_pressure_hpa(height_m, p0_sea_level=1013.25):
    """Estimate barometric surface pressure at altitude using barometric formula."""
    return p0_sea_level * (1.0 - 2.25577e-5 * height_m) ** 5.25588


def water_vapor_pressure_hpa(t_c, rh_pct):
    """Calculate partial pressure of water vapor in hPa from temp and relative humidity."""
    # Tetens formula
    e_sat = 6.1121 * math.exp((17.502 * t_c) / (240.97 + t_c))
    return (rh_pct / 100.0) * e_sat


def saastamoinen_zhd(p0_hpa, lat_deg, height_km):
    """Calculate Zenith Hydrostatic Delay (ZHD) in meters (Saastamoinen 1972)."""
    phi_rad = math.radians(lat_deg)
    denom = 1.0 - 0.00266 * math.cos(2.0 * phi_rad) - 0.00028 * height_km
    zhd_m = (0.0022768 * p0_hpa) / denom
    return zhd_m


def saastamoinen_zwd(t0_c, e0_hpa):
    """Calculate Zenith Wet Delay (ZWD) in meters (Saastamoinen 1973)."""
    t0_k = t0_c + 273.15
    zwd_m = 0.002277 * (1255.0 / t0_k + 0.05) * e0_hpa
    return zwd_m


def niell_mapping_hydrostatic(el_deg):
    """Niell (1996) hydrostatic continued fraction mapping function mh(E)."""
    # Standard mid-latitude hydrostatic coefficients
    a = 1.27699e-3
    b = 2.9129e-3
    c = 63.957e-3

    sin_e = math.sin(math.radians(max(2.0, el_deg)))
    top = 1.0 + a / (1.0 + b / (1.0 + c))
    bot = sin_e + a / (sin_e + b / (sin_e + c))
    return top / bot


def niell_mapping_wet(el_deg):
    """Niell (1996) wet continued fraction mapping function mw(E)."""
    # Standard wet coefficients
    a = 0.58021e-3
    b = 1.4515e-3
    c = 40.084e-3

    sin_e = math.sin(math.radians(max(2.0, el_deg)))
    top = 1.0 + a / (1.0 + b / (1.0 + c))
    bot = sin_e + a / (sin_e + b / (sin_e + c))
    return top / bot


def compute_tropospheric_delay(el_deg, zhd_m, zwd_m):
    """Compute slant hydrostatic, wet, and total delay for a given elevation."""
    mh = niell_mapping_hydrostatic(el_deg)
    mw = niell_mapping_wet(el_deg)

    slant_hydro_m = zhd_m * mh
    slant_wet_m = zwd_m * mw
    total_tropo_m = slant_hydro_m + slant_wet_m
    total_tropo_ns = (total_tropo_m / C_MPS) * 1e9

    return {
        "map_hydro": round(mh, 3),
        "map_wet": round(mw, 3),
        "slant_hydro_m": round(slant_hydro_m, 2),
        "slant_wet_m": round(slant_wet_m, 2),
        "tropo_delay_m": round(total_tropo_m, 2),
        "tropo_delay_ns": round(total_tropo_ns, 2)
    }


class TroposphericModel:
    def __init__(self, state_iono_path=STATE_IONO_PATH, state_klob_path=STATE_KLOB_PATH,
                 out_path=STATE_TROPO_PATH, lat=DEFAULT_LAT_DEG, lon=DEFAULT_LON_DEG,
                 height_m=DEFAULT_HEIGHT_M):
        self.state_iono_path = state_iono_path
        self.state_klob_path = state_klob_path
        self.out_path = out_path
        self.lat = lat
        self.lon = lon
        self.height_m = height_m

        # Surface meteorological conditions
        self.p0_hpa = barometric_pressure_hpa(self.height_m)
        self.t0_c = 20.0
        self.rh_pct = 50.0

    def evaluate(self):
        """Evaluate tropospheric delays for all visible satellites and publish state."""
        # Calculate zenith delays
        height_km = self.height_m / 1000.0
        zhd_m = saastamoinen_zhd(self.p0_hpa, self.lat, height_km)
        e0_hpa = water_vapor_pressure_hpa(self.t0_c, self.rh_pct)
        zwd_m = saastamoinen_zwd(self.t0_c, e0_hpa)
        ztd_m = zhd_m + zwd_m
        ztd_ns = (ztd_m / C_MPS) * 1e9

        # Gather satellite az/el from state.klobuchar.json or state.iono.json
        sats_input = {}
        if os.path.exists(self.state_klob_path):
            try:
                with open(self.state_klob_path, "r", encoding="utf-8") as f:
                    klob_data = json.load(f)
                sats_input = klob_data.get("satellites", {})
            except Exception:
                pass

        if not sats_input and os.path.exists(self.state_iono_path):
            try:
                with open(self.state_iono_path, "r", encoding="utf-8") as f:
                    iono_data = json.load(f)
                sats_input = iono_data.get("satellites", {})
            except Exception:
                pass

        evaluated_sats = {}
        for sat_key, s in sats_input.items():
            az = s.get("az_deg")
            el = s.get("el_deg")
            if az is None or el is None or el <= 0.0:
                continue

            tropo = compute_tropospheric_delay(el, zhd_m, zwd_m)
            iono_ns = s.get("klobuchar_delay_ns", 10.0)
            iono_m = s.get("klobuchar_delay_m", (iono_ns * 1e-9 * C_MPS))

            total_atm_ns = round(tropo["tropo_delay_ns"] + iono_ns, 2)
            total_atm_m = round(tropo["tropo_delay_m"] + iono_m, 2)

            sat_entry = dict(s)
            sat_entry.update(tropo)
            sat_entry.update({
                "iono_delay_ns": round(iono_ns, 2),
                "total_atm_delay_ns": total_atm_ns,
                "total_atm_delay_m": total_atm_m
            })
            evaluated_sats[sat_key] = sat_entry

        output = {
            "epoch": round(time.time(), 2),
            "station_lat": self.lat,
            "station_lon": self.lon,
            "station_height_m": self.height_m,
            "surface_p0_hpa": round(self.p0_hpa, 1),
            "surface_t0_c": self.t0_c,
            "surface_rh_pct": self.rh_pct,
            "zhd_m": round(zhd_m, 3),
            "zwd_m": round(zwd_m, 3),
            "ztd_m": round(ztd_m, 3),
            "ztd_ns": round(ztd_ns, 2),
            "n_tropo_benchmarked": len(evaluated_sats),
            "tropo_satellites": evaluated_sats
        }

        # Atomically write state file
        tmp_out = self.out_path + ".tmp"
        with open(tmp_out, "w", encoding="utf-8") as f:
            json.dump(output, f, indent=2)
        os.replace(tmp_out, self.out_path)

        return output


def main():
    parser = argparse.ArgumentParser(description="Tropospheric Neutral Atmosphere Path Delay Model")
    parser.add_argument("--once", action="store_true", help="Run once and exit")
    parser.add_argument("--loop", action="store_true", help="Run continuously in a loop")
    parser.add_argument("--interval", type=float, default=2.0, help="Loop interval in seconds")
    args = parser.parse_args()

    model = TroposphericModel()

    while True:
        res = model.evaluate()
        if not args.loop:
            break
        time.sleep(args.interval)

    print("=================================================================")
    print("      TROPOSPHERIC PATH DELAY MODEL (SAASTAMOINEN & NIELL)       ")
    print("=================================================================")
    if not res:
        print("Failed to compute tropospheric model.")
        return

    print(f"Station:                 {res['station_lat']}°N, {res['station_lon']}°W (H = {res['station_height_m']} m)")
    print(f"Surface Pressure (P0):   {res['surface_p0_hpa']} hPa")
    print(f"Zenith Hydrostatic (ZHD):{res['zhd_m']:.3f} m ({res['zhd_m']/C_MPS*1e9:.2f} ns)")
    print(f"Zenith Wet (ZWD):        {res['zwd_m']:.3f} m ({res['zwd_m']/C_MPS*1e9:.2f} ns)")
    print(f"Total Zenith Delay (ZTD):{res['ztd_m']:.3f} m ({res['ztd_ns']:.2f} ns)")
    print(f"Sats Evaluated:          {res['n_tropo_benchmarked']}")
    print("-----------------------------------------------------------------")
    print("SAT         AZ     EL     mh(E)  Tropo(m/ns)   Iono(ns)   Total Atm")
    print("-----------------------------------------------------------------")
    for sat_id, sat in sorted(res["tropo_satellites"].items()):
        print(f"{sat_id:11s} {sat['az_deg']:5.1f}° {sat['el_deg']:4.1f}°  {sat['map_hydro']:5.2f} "
              f"{sat['tropo_delay_m']:5.2f}m/{sat['tropo_delay_ns']:5.1f}ns  "
              f"{sat['iono_delay_ns']:5.1f}ns   "
              f"{sat['total_atm_delay_m']:5.2f}m ({sat['total_atm_delay_ns']:5.1f}ns)")
    print("=================================================================")
    print(f"State File:              {STATE_TROPO_PATH}")


if __name__ == "__main__":
    main()
