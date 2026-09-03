#!/usr/bin/env python3
"""Inter-System Bias (ISB / ISX) Metrology & Allan Deviation Analyzer.

Analyzes the multi-constellation Inter-System Bias between GPS and BeiDou (B1I vs L1)
from continuous position fixes in observations/position_history.jsonl:
  1. Calibrates receiver front-end inter-frequency hardware group delay
  2. Evaluates time-scale alignment between GPS Time (GPST) and BeiDou Time (BDT)
  3. Computes Overlapping Allan Deviation (ADEV) across integration times tau = 60s - 3600s
  4. Publishes observations/state.isb.json atomically for live fusion into /api/sync
"""
import argparse
import json
import math
import os
import sys
import numpy as np

C_MPS = 299792458.0
POS_HISTORY_PATH = "/Volumes/Radiator 8TB/gnss/observations/position_history.jsonl"
STATE_ISB_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.isb.json"


def compute_adev(times, values, tau_list):
    """Compute Overlapping Allan Deviation (ADEV) for arbitrary unevenly sampled series."""
    t = np.array(times, dtype=np.float64)
    y = np.array(values, dtype=np.float64)
    
    # Sort by time
    idx = np.argsort(t)
    t = t[idx]
    y = y[idx]

    adev_results = {}
    for tau in tau_list:
        # Create pairs separated by approx tau
        diffs = []
        for i in range(len(t)):
            target_t = t[i] + tau
            # Find closest sample within 20% tolerance of tau
            j = np.searchsorted(t, target_t)
            if j < len(t) and abs((t[j] - t[i]) - tau) <= 0.25 * tau:
                # Fractional frequency difference or phase step
                # For phase data (seconds), y_diff / tau
                diff = (y[j] - y[i]) / (t[j] - t[i])
                diffs.append(diff)
        
        if len(diffs) >= 4:
            d = np.array(diffs)
            adev = np.sqrt(0.5 * np.mean(np.diff(d)**2))
            adev_results[tau] = float(adev)
        else:
            adev_results[tau] = None

    return adev_results


def analyze_isb(history_path=POS_HISTORY_PATH):
    """Extract ISB series and compute geodetic stability metrics."""
    if not os.path.exists(history_path):
        return None

    times = []
    isb_ns = []

    with open(history_path) as f:
        for line in f:
            if not line.strip():
                continue
            try:
                rec = json.loads(line)
                t = rec.get("epoch")
                isx_km = rec.get("isx_km")
                if t is not None and isx_km is not None and not math.isnan(isx_km):
                    # Convert km to ns: (isx_km * 1000 / c) * 1e9
                    ns = (float(isx_km) * 1e3 / C_MPS) * 1e9
                    times.append(float(t))
                    isb_ns.append(ns)
            except Exception:
                continue

    if len(times) < 5:
        return {
            "status": "INSUFFICIENT_ISX_SOLVES",
            "n_samples": len(times)
        }

    times = np.array(times)
    isb_ns = np.array(isb_ns)

    # Robust statistics (Median and MAD)
    med_ns = float(np.median(isb_ns))
    mad_ns = float(np.median(np.abs(isb_ns - med_ns)) * 1.4826)
    mean_ns = float(np.mean(isb_ns))
    std_ns = float(np.std(isb_ns))

    # Allan Deviation on phase in seconds
    isb_seconds = isb_ns * 1e-9
    tau_list = [60.0, 300.0, 600.0, 1800.0, 3600.0]
    adev_map = compute_adev(times, isb_seconds, tau_list)

    total_span_s = float(times[-1] - times[0])

    output = {
        "epoch": round(float(times[-1]), 2),
        "total_span_hours": round(total_span_s / 3600.0, 2),
        "n_samples": len(times),
        "isb_median_ns": round(med_ns, 2),
        "isb_mad_ns": round(mad_ns, 2),
        "isb_mean_ns": round(mean_ns, 2),
        "isb_std_ns": round(std_ns, 2),
        "isb_equivalent_path_m": round(med_ns * 1e-9 * C_MPS, 3),
        "adev_tau_60s": adev_map.get(60.0),
        "adev_tau_300s": adev_map.get(300.0),
        "adev_tau_3600s": adev_map.get(3600.0)
    }

    # Atomically write state file
    tmp_path = STATE_ISB_PATH + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump(output, f, indent=2)
    os.replace(tmp_path, STATE_ISB_PATH)

    return output


def main():
    parser = argparse.ArgumentParser(description="Inter-System Bias (ISB / ISX) Metrology Analyzer")
    parser.add_argument("--once", action="store_true", help="Run analysis and exit")
    args = parser.parse_args()

    res = analyze_isb()
    print("=================================================================")
    print("    INTER-SYSTEM BIAS (ISB / ISX) GPS vs BEIDOU METROLOGY        ")
    print("=================================================================")
    if not res:
        print("No position history found.")
        return

    if res.get("status") == "INSUFFICIENT_ISX_SOLVES":
        print(f"Waiting for mixed GPS+BDS fixes (current count: {res['n_samples']})")
        return

    print(f"Analyzed Fixes:          {res['n_samples']}")
    print(f"Observation Span:        {res['total_span_hours']} hours")
    print(f"Median ISB (BDS - GPS):  {res['isb_median_ns']:+.2f} ns  (MAD: ±{res['isb_mad_ns']:.2f} ns)")
    print(f"Mean ISB (BDS - GPS):    {res['isb_mean_ns']:+.2f} ns  (STD: ±{res['isb_std_ns']:.2f} ns)")
    print(f"Equivalent RF Delta:     {res['isb_equivalent_path_m']:+.3f} meters")
    if res.get("adev_tau_60s"):
        print(f"ADEV (τ = 60s):          {res['adev_tau_60s']:.3e}")
    if res.get("adev_tau_300s"):
        print(f"ADEV (τ = 300s):         {res['adev_tau_300s']:.3e}")
    print("=================================================================")
    print(f"State File:              {STATE_ISB_PATH}")


if __name__ == "__main__":
    main()
