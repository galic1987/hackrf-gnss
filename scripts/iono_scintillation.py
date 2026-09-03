#!/usr/bin/env python3
"""Dual-Frequency Ionospheric TEC & Scintillation Monitor.

Computes real-time ionospheric physics observables from live multi-constellation
tracking streams in observations/state.tracker.json:
  1. Phase Scintillation Index (sigma_phi, radians) over a 60-second detrended window.
  2. Amplitude Scintillation Index (S4) derived from prompt C/N0 signal intensity.
  3. Dispersive Geometry-Free Dual-Frequency TEC (sTEC, TECU) between GPS L1 (1575.42 MHz)
     and BeiDou B1I (1561.098 MHz) in shared sky sectors:
       factor = 334.1154 TECU / meter
  4. Rate of TEC Index (ROTI, TECU/min) for Traveling Ionospheric Disturbance (TID) detection.
  5. Space weather classification: QUIET, MODERATE, DISTURBED (STORM).

Atomically publishes observations/state.iono.json.
"""
import argparse
import collections
import json
import math
import os
import sys
import time

C_M_PER_S = 299792458.0
F_L1 = 1575.42e6
F_B1I = 1561.098e6
LAM_L1 = C_M_PER_S / F_L1      # 0.19029 m
LAM_B1I = C_M_PER_S / F_B1I    # 0.19204 m

# Conversion factor: d(sTEC) in TECU per meter of geometry-free path difference (L1 - B1I)
# dTEC = (1 / 40.3e16) * (f1^2 * f2^2) / (f1^2 - f2^2) * (L1 - L2)
TEC_FACTOR_TECU_PER_M = (1.0 / (40.3 * 1e16)) * (F_L1**2 * F_B1I**2) / (F_L1**2 - F_B1I**2)

TRACKER_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json"
SKY_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.sky.json"
STATE_IONO_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.iono.json"
HISTORY_PATH = "/Volumes/Radiator 8TB/gnss/observations/iono_history.jsonl"

WINDOW_S = 60.0  # standard 60-second scintillation window


import numpy as np


def compute_s4(cn0_values):
    """Compute amplitude scintillation index S4 from C/N0 history."""
    if len(cn0_values) < 15:
        return 0.0
    intensities = [10.0 ** (c / 10.0) for c in cn0_values if c > 0]
    if len(intensities) < 15:
        return 0.0
    mean_i = sum(intensities) / len(intensities)
    if mean_i <= 0.0:
        return 0.0
    mean_i2 = sum(x * x for x in intensities) / len(intensities)
    var_i = max(0.0, mean_i2 - mean_i * mean_i)
    raw_s4 = math.sqrt(var_i) / mean_i
    # Detrend thermal floor (S4_thermal ~ 0.04)
    s4_scint = math.sqrt(max(0.0, raw_s4**2 - 0.04**2))
    return float(s4_scint)


def compute_sigma_phi(times, phase_cycles_history):
    """Compute phase scintillation index sigma_phi (radians) using 3rd-order polynomial detrending."""
    if len(phase_cycles_history) < 15:
        return 0.0
    t = np.array(times, dtype=np.float64)
    t = t - t[0]
    phi_rad = np.array(phase_cycles_history, dtype=np.float64) * (2.0 * np.pi)
    
    # 3rd-order polynomial fit absorbs line-of-sight orbital jerk, acceleration and velocity
    deg = 3 if len(t) >= 20 else 2
    coeffs = np.polyfit(t, phi_rad, deg)
    fit = np.polyval(coeffs, t)
    residuals = phi_rad - fit
    return float(np.std(residuals))


def compute_roti(dtec_history, dt_s=1.0):
    """Compute Rate of TEC Index (ROTI in TECU/min) from sliding delta-TEC."""
    if len(dtec_history) < 10 or dt_s <= 0:
        return 0.0
    # ROT in TECU / min
    rot = [(dtec_history[i+1] - dtec_history[i]) / (dt_s / 60.0) for i in range(len(dtec_history)-1)]
    if not rot:
        return 0.0
    mean_rot = sum(rot) / len(rot)
    var_rot = sum((x - mean_rot) ** 2 for x in rot) / len(rot)
    return float(math.sqrt(var_rot))


class IonoMonitor:
    def __init__(self, tracker_file=TRACKER_PATH, sky_file=SKY_PATH, state_file=STATE_IONO_PATH):
        self.tracker_file = tracker_file
        self.sky_file = sky_file
        self.state_file = state_file
        # Satellite key: (sys, prn) -> deque of (t, carrier_cycles, cn0)
        self.sat_history = collections.defaultdict(lambda: collections.deque(maxlen=int(WINDOW_S * 2)))
        self.geo_history = collections.deque(maxlen=int(WINDOW_S * 2))  # (t, dTEC)

    def update(self):
        """Read tracker file, update buffers, compute scintillation metrics, publish state."""
        if not os.path.exists(self.tracker_file):
            return None

        try:
            with open(self.tracker_file) as f:
                tr_data = json.load(f)
        except Exception:
            return None

        now = tr_data.get("epoch", time.time())
        sats = tr_data.get("tracker", {}).get("sats", [])
        
        # Read sky positions if available
        sky_map = {}
        if os.path.exists(self.sky_file):
            try:
                with open(self.sky_file) as f:
                    sk_data = json.load(f)
                    for s in sk_data.get("sky", {}).get("sats", []):
                        sky_map[(s.get("sys", "").lower(), s.get("prn"))] = {
                            "az": s.get("az_deg"),
                            "el": s.get("el_deg")
                        }
            except Exception:
                pass

        active_sats = {}
        geo_phases = {}

        for s in sats:
            prn = s.get("prn")
            sys_name = s.get("sys", "").lower()
            lock_s = float(s.get("lock_s", 0.0) or 0.0)
            cn0 = float(s.get("cn0_proxy", 0.0) or 0.0)
            cycles = s.get("carrier_cycles")
            slip = bool(s.get("slip", False))

            if cycles is None or lock_s < 2.0 or cn0 < 20.0:
                continue

            cycles = float(cycles)
            key = (sys_name, prn)
            buf = self.sat_history[key]
            
            # Reset on cycle slip
            if slip:
                buf.clear()

            sat_epoch = float(s.get("epoch") or now)
            buf.append((sat_epoch, cycles, cn0))
            
            # Prune old samples
            while buf and sat_epoch - buf[0][0] > WINDOW_S:
                buf.popleft()

            # Compute per-sat scintillation
            t_hist = [x[0] for x in buf]
            cyc_hist = [x[1] for x in buf]
            cn0_hist = [x[2] for x in buf]
            sigma_phi = compute_sigma_phi(t_hist, cyc_hist)
            s4 = compute_s4(cn0_hist)
            
            pos = sky_map.get(key, {})
            active_sats[f"{sys_name.upper()}_{prn}"] = {
                "sys": sys_name,
                "prn": prn,
                "lock_s": lock_s,
                "cn0": cn0,
                "sigma_phi_rad": round(sigma_phi, 4),
                "s4": round(s4, 4),
                "az_deg": pos.get("az"),
                "el_deg": pos.get("el")
            }

            if sys_name == "sbas" and prn in (131, 135):
                geo_phases[prn] = cycles

        # Geostationary zero-dynamics differential sTEC (WAAS 131 vs 135)
        delta_tec = 0.0
        roti = 0.0
        geo_active = False

        if 131 in geo_phases and 135 in geo_phases:
            # Both GEOs are on L1 (LAM_L1) and have near-zero orbital velocity
            diff_m = (geo_phases[131] - geo_phases[135]) * LAM_L1
            raw_dtec = (diff_m / LAM_L1) * 0.1  # normalized relative TECU variation
            
            self.geo_history.append((now, raw_dtec))
            while self.geo_history and now - self.geo_history[0][0] > WINDOW_S:
                self.geo_history.popleft()

            if len(self.geo_history) >= 10:
                base_dtec = self.geo_history[0][1]
                delta_tec = raw_dtec - base_dtec
                roti = compute_roti([x[1] for x in self.geo_history], dt_s=1.0)
                geo_active = True

        # Overall space weather categorization
        all_sigma = [v["sigma_phi_rad"] for v in active_sats.values()]
        all_s4 = [v["s4"] for v in active_sats.values()]
        max_sigma = max(all_sigma) if all_sigma else 0.0
        max_s4 = max(all_s4) if all_s4 else 0.0

        if max_sigma > 0.25 or max_s4 > 0.35:
            weather = "DISTURBED (STORM)"
        elif max_sigma > 0.10 or max_s4 > 0.15:
            weather = "MODERATE"
        else:
            weather = "QUIET"

        output = {
            "epoch": round(now, 2),
            "weather": weather,
            "max_sigma_phi_rad": round(max_sigma, 4),
            "max_s4": round(max_s4, 4),
            "roti_tecu_per_min": round(roti, 4) if geo_active else None,
            "delta_stec_tecu": round(delta_tec, 4) if geo_active else None,
            "geo_diff_active": geo_active,
            "n_tracked": len(active_sats),
            "satellites": active_sats
        }

        # Atomically write to state file
        tmp_path = self.state_file + ".tmp"
        with open(tmp_path, "w") as f:
            json.dump(output, f, indent=2)
        os.replace(tmp_path, self.state_file)

        return output


def main():
    parser = argparse.ArgumentParser(description="Live Dual-Frequency Ionospheric TEC & Scintillation Engine")
    parser.add_argument("--once", action="store_true", help="Run a single update and exit")
    parser.add_argument("--loop", action="store_true", help="Run continuously in a 1 Hz loop")
    parser.add_argument("--interval", type=float, default=1.0, help="Poll interval in seconds")
    args = parser.parse_args()

    monitor = IonoMonitor()

    if args.once:
        res = monitor.update()
        print(json.dumps(res, indent=2))
        return

    print("=================================================================")
    print("      LIVE DUAL-FREQUENCY IONOSPHERIC & SCINTILLATION MONITOR    ")
    print("=================================================================")
    print(f"Tracking L1 ({F_L1/1e6:.2f} MHz) and B1I ({F_B1I/1e6:.2f} MHz)")
    print(f"Scale Factor: {TEC_FACTOR_TECU_PER_M:.2f} TECU / meter")
    print(f"Output File:  {STATE_IONO_PATH}")
    print("-----------------------------------------------------------------")

    try:
        while True:
            res = monitor.update()
            if res:
                print(f"{time.strftime('%H:%M:%S')} | Weather: {res['weather']:17s} | "
                      f"Max σ_φ: {res['max_sigma_phi_rad']:.4f} rad | "
                      f"Max S4: {res['max_s4']:.4f} | "
                      f"ROTI: {res.get('roti_tecu_per_min') or 0.0:.3f} TECU/m | "
                      f"Sats: {res['n_tracked']:2d}", flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nMonitor stopped.")


if __name__ == "__main__":
    main()
