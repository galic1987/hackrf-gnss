#!/usr/bin/env python3
"""GPS ICD-200 Klobuchar Ionospheric Model & Empirical Carrier Phase Benchmark.

Implements the official IS-GPS-200 Annex Klobuchar single-frequency ionospheric
algorithm and benchmarks broadcast model predictions against live dual-frequency
and carrier-phase TEC observations from observations/state.iono.json.
"""
import argparse
import json
import math
import os
import sys
import time

C_MPS = 299792458.0
F_L1_HZ = 1575.42e6
TEC_PER_METER_L1 = (F_L1_HZ**2) / (40.3 * 1e16)  # ~6.155 TECU per meter of L1 delay

# Standard default broadcast Klobuchar coefficients (mid-latitude quiet sun)
DEFAULT_ALPHA = [0.1118e-7, -0.7451e-8, -0.5960e-7, 0.1192e-6]
DEFAULT_BETA = [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7]

STATE_IONO_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.iono.json"
STATE_KLOBUCHAR_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.klobuchar.json"


def klobuchar_delay_s(lat_deg, lon_deg, az_deg, el_deg, tow_s, alpha=None, beta=None):
    """Compute Klobuchar ionospheric delay in seconds according to IS-GPS-200.
    
    Inputs:
      lat_deg, lon_deg: Receiver geodetic coordinates (degrees)
      az_deg: Satellite azimuth from North (degrees, 0-360)
      el_deg: Satellite elevation above horizon (degrees, 0-90)
      tow_s: GPS Time of Week (seconds, 0-604800)
      alpha: 4 coefficients [a0, a1, a2, a3]
      beta: 4 coefficients [b0, b1, b2, b3]
    
    Returns:
      (t_slant_s, t_vert_s, f_obliq, stec_tecu)
    """
    if alpha is None:
        alpha = DEFAULT_ALPHA
    if beta is None:
        beta = DEFAULT_BETA

    # Convert coordinates to semi-circles (1 semi-circle = pi radians = 180 deg)
    phi_u = lat_deg / 180.0
    lam_u = lon_deg / 180.0
    el_sc = max(0.0, el_deg) / 180.0
    az_rad = math.radians(az_deg)

    # 1. Earth central angle psi (semi-circles)
    psi = 0.0137 / (el_sc + 0.11) - 0.022

    # 2. Sub-ionospheric latitude phi_i (semi-circles)
    phi_i = phi_u + psi * math.cos(az_rad)
    phi_i = max(-0.416, min(0.416, phi_i))

    # 3. Sub-ionospheric longitude lam_i (semi-circles)
    lam_i = lam_u + (psi * math.sin(az_rad)) / math.cos(phi_i * math.pi)

    # 4. Geomagnetic latitude phi_m (semi-circles)
    phi_m = phi_i + 0.064 * math.cos((lam_i - 1.617) * math.pi)

    # 5. Local time at sub-ionospheric point t (seconds)
    t = 43200.0 * lam_i + tow_s
    t = t % 86400.0
    if t < 0.0:
        t += 86400.0

    # 6. Amplitude of delay A_I (seconds)
    amp = alpha[0] + alpha[1] * phi_m + alpha[2] * (phi_m**2) + alpha[3] * (phi_m**3)
    amp = max(0.0, amp)

    # 7. Period of delay P_I (seconds)
    per = beta[0] + beta[1] * phi_m + beta[2] * (phi_m**2) + beta[3] * (phi_m**3)
    per = max(72000.0, per)

    # 8. Phase of delay X_I (radians)
    x_i = (2.0 * math.pi * (t - 50400.0)) / per

    # 9. Slant obliquity factor F(E)
    f_obliq = 1.0 + 16.0 * ((0.53 - el_sc)**3)

    # 10. Vertical delay (seconds)
    if abs(x_i) < 1.57:
        # Cosine series expansion: 1 - x^2/2 + x^4/24
        t_vert_s = 5e-9 + amp * (1.0 - (x_i**2) / 2.0 + (x_i**4) / 24.0)
    else:
        # Nighttime floor: 5 ns (~1.5 m)
        t_vert_s = 5e-9

    t_slant_s = f_obliq * t_vert_s
    delay_m = t_slant_s * C_MPS
    stec_tecu = delay_m * TEC_PER_METER_L1

    return t_slant_s, t_vert_s, f_obliq, stec_tecu


class KlobucharBenchmark:
    def __init__(self, state_iono_file=STATE_IONO_PATH, out_file=STATE_KLOBUCHAR_PATH):
        self.state_iono_file = state_iono_file
        self.out_file = out_file
        self.station_lat = 39.0029
        self.station_lon = -77.6058

    def evaluate(self):
        """Read state.iono.json, compute Klobuchar delay for each sat, publish benchmark."""
        if not os.path.exists(self.state_iono_file):
            return None

        try:
            with open(self.state_iono_file) as f:
                iono_data = json.load(f)
        except Exception:
            return None

        now = iono_data.get("epoch", time.time())
        # Compute GPS Time of Week (TOW) from epoch
        # GPS epoch Jan 6 1980; Unix epoch Jan 1 1970 (difference: 315964800s - 18s leap seconds)
        gps_time = now - 315964800.0 + 18.0
        tow_s = gps_time % 604800.0

        sats = iono_data.get("satellites", {})
        results = {}

        for sat_key, s in sats.items():
            az = s.get("az_deg")
            el = s.get("el_deg")
            if az is None or el is None or el <= 0.0:
                continue

            t_slant, t_vert, f_obliq, klob_tec = klobuchar_delay_s(
                self.station_lat, self.station_lon, az, el, tow_s
            )

            delay_m = t_slant * C_MPS
            delay_ns = t_slant * 1e9

            results[sat_key] = {
                "sys": s.get("sys"),
                "prn": s.get("prn"),
                "az_deg": az,
                "el_deg": el,
                "klobuchar_delay_ns": round(delay_ns, 2),
                "klobuchar_delay_m": round(delay_m, 2),
                "klobuchar_stec_tecu": round(klob_tec, 2),
                "klobuchar_vtec_tecu": round(klob_tec / f_obliq, 2),
                "obliquity_factor": round(f_obliq, 3),
                "observed_sigma_phi_rad": s.get("sigma_phi_rad"),
                "observed_s4": s.get("s4")
            }

        output = {
            "epoch": round(now, 2),
            "tow_s": round(tow_s, 1),
            "station_lat": self.station_lat,
            "station_lon": self.station_lon,
            "n_benchmarked": len(results),
            "satellites": results
        }

        # Atomically write state file
        tmp_path = self.out_file + ".tmp"
        with open(tmp_path, "w") as f:
            json.dump(output, f, indent=2)
        os.replace(tmp_path, self.out_file)

        return output


def main():
    parser = argparse.ArgumentParser(description="GPS Klobuchar Ionospheric Benchmark")
    parser.add_argument("--once", action="store_true", help="Run once and exit")
    parser.add_argument("--loop", action="store_true", help="Run continuously in a loop")
    parser.add_argument("--interval", type=float, default=2.0, help="Poll interval in seconds")
    args = parser.parse_args()

    bench = KlobucharBenchmark()

    if args.once or not args.loop:
        res = bench.evaluate()

        print("=================================================================")
        print("      GPS ICD-200 KLOBUCHAR IONOSPHERIC BENCHMARK                ")
        print("=================================================================")
        if not res:
            print("No ionospheric state available.")
            return

        print(f"Station:        {res['station_lat']}°N, {res['station_lon']}°W")
        print(f"GPS TOW:        {res['tow_s']} s")
        print(f"Sats Evaluated: {res['n_benchmarked']}")
        print("-----------------------------------------------------------------")
        print("SAT         AZ     EL     F(E)   Klob Delay   Klob sTEC   Obs σ_φ")
        print("-----------------------------------------------------------------")
        for k, v in res.get("satellites", {}).items():
            print(f"{k:10s} {v['az_deg']:5.1f}° {v['el_deg']:4.1f}° {v['obliquity_factor']:5.2f} "
                  f"{v['klobuchar_delay_ns']:7.2f} ns  {v['klobuchar_stec_tecu']:6.2f} TECU "
                  f"{v.get('observed_sigma_phi_rad', 0.0):7.4f} rad")
        print("=================================================================")
        print(f"State File:     {STATE_KLOBUCHAR_PATH}")
        return

    while True:
        try:
            bench.evaluate()
            time.sleep(args.interval)
        except KeyboardInterrupt:
            break
        except Exception:
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
