#!/usr/bin/env python3
"""Carrier Phase Diurnal Thermal Expansion & Secular Drift Metrology Analyzer.

Analyzes continuous carrier phase tracking from the HackRF One + south ClearStream
antenna (observations/phase_history.jsonl) over 100+ hours:
  1. Measures fractional frequency offset between the transmitter atomic source and GPSDO
  2. Isolates the 24-hour diurnal solar heating / nocturnal cooling thermal harmonic
  3. Estimates effective thermal phase delay coefficient of the RF feedline
  4. Publishes observations/state.thermal.json atomically for live fusion into /api/sync
"""
import argparse
import json
import math
import os
import sys
import time
import numpy as np

C_MPS = 299792458.0
PHASE_HISTORY_PATH = "/Volumes/Radiator 8TB/gnss/observations/phase_history.jsonl"
STATE_THERMAL_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.thermal.json"
CARRIER_FREQ_HZ = 602.30944e6


def fit_diurnal_harmonics(times, disps_mm):
    """Fit secular linear drift plus 24h and 12h diurnal thermal harmonics.
    
    y(t) = c0 + v * t + A24 * sin(w24*t + phi24) + A12 * sin(w12*t + phi12)
    """
    t = np.array(times, dtype=np.float64)
    t_rel = t - t[0]
    y = np.array(disps_mm, dtype=np.float64)

    # Base frequencies (rad/s)
    w24 = 2.0 * np.pi / 86400.0
    w12 = 2.0 * np.pi / 43200.0

    # Design matrix: [1, t, cos(w24*t), sin(w24*t), cos(w12*t), sin(w12*t)]
    A = np.column_stack([
        np.ones_like(t_rel),
        t_rel,
        np.cos(w24 * t),
        np.sin(w24 * t),
        np.cos(w12 * t),
        np.sin(w12 * t)
    ])

    # Least-squares solution
    coeffs, residuals, rank, s = np.linalg.lstsq(A, y, rcond=None)
    c0, v_mm_per_s, cos24, sin24, cos12, sin12 = coeffs

    # Secular fractional frequency offset: y = c * dt => v_mps = v_mm_per_s * 1e-3
    v_mps = v_mm_per_s * 1e-3
    frac_freq_offset = v_mps / C_MPS
    carrier_freq_offset_hz = frac_freq_offset * CARRIER_FREQ_HZ

    # Diurnal 24h amplitude and phase
    amp_24_mm = float(math.hypot(cos24, sin24))
    phi_24_rad = float(math.atan2(cos24, sin24))

    # Semidiurnal 12h amplitude
    amp_12_mm = float(math.hypot(cos12, sin12))

    # Residual scatter after removing secular + diurnal
    y_model = A @ coeffs
    res_mm = y - y_model
    rms_res_mm = float(np.std(res_mm))

    # Equivalent time delay (picoseconds)
    amp_24_ps = (amp_24_mm * 1e-3 / C_MPS) * 1e12

    return {
        "secular_drift_mm_per_day": round(float(v_mm_per_s * 86400.0), 2),
        "frac_freq_offset": float(frac_freq_offset),
        "carrier_freq_offset_hz": float(carrier_freq_offset_hz),
        "diurnal_amp_24h_mm": round(amp_24_mm, 2),
        "diurnal_amp_24h_ps": round(amp_24_ps, 1),
        "semidiurnal_amp_12h_mm": round(amp_12_mm, 2),
        "residual_rms_mm": round(rms_res_mm, 2)
    }


def analyze_phase_thermals(history_path=PHASE_HISTORY_PATH, min_samples=10):
    """Load phase history, perform thermal analysis, publish state."""
    if not os.path.exists(history_path):
        return None

    times = []
    disps = []

    with open(history_path) as f:
        for i, line in enumerate(f):
            if not line.strip():
                continue
            try:
                # Subsample 10:1 on large files to maintain fast vectorized execution
                if i % 10 == 0:
                    rec = json.loads(line)
                    if rec.get("lock") and rec.get("disp_mm") is not None:
                        times.append(float(rec["t"]))
                        disps.append(float(rec["disp_mm"]))
            except Exception:
                continue

    if len(times) < min_samples:
        return {
            "status": "INSUFFICIENT_PHASE_HISTORY",
            "n_samples": len(times)
        }

    fit = fit_diurnal_harmonics(times, disps)
    span_hours = (times[-1] - times[0]) / 3600.0

    output = {
        "epoch": round(times[-1], 2),
        "total_span_hours": round(span_hours, 2),
        "n_samples_analyzed": len(times),
        "secular_drift_mm_per_day": fit["secular_drift_mm_per_day"],
        "carrier_offset_mhz": round(fit["carrier_freq_offset_hz"] * 1e3, 3),
        "fractional_frequency_offset": f"{fit['frac_freq_offset']:.3e}",
        "diurnal_wander_ptp_mm": round(2.0 * fit["diurnal_amp_24h_mm"], 2),
        "diurnal_wander_ptp_ps": round(2.0 * fit["diurnal_amp_24h_ps"], 1),
        "semidiurnal_amp_mm": fit["semidiurnal_amp_12h_mm"],
        "unmodeled_rms_mm": fit["residual_rms_mm"]
    }

    # Atomically write state file
    tmp_path = STATE_THERMAL_PATH + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump(output, f, indent=2)
    os.replace(tmp_path, STATE_THERMAL_PATH)

    return output


def main():
    parser = argparse.ArgumentParser(description="Carrier Phase Thermal & Diurnal Metrology Analyzer")
    parser.add_argument("--once", action="store_true", help="Run analysis and exit")
    parser.add_argument("--loop", action="store_true", help="Run continuously in a loop")
    parser.add_argument("--interval", type=float, default=60.0, help="Loop interval in seconds")
    args = parser.parse_args()

    while True:
        res = analyze_phase_thermals()
        if not args.loop:
            break
        time.sleep(args.interval)

    print("=================================================================")
    print("   CARRIER PHASE DIURNAL THERMAL & SECULAR DRIFT METROLOGY       ")
    print("=================================================================")
    if not res:
        print("No phase history found.")
        return

    if res.get("status") == "INSUFFICIENT_PHASE_HISTORY":
        print(f"Waiting for carrier phase records (current count: {res['n_samples']})")
        return

    print(f"Analyzed Samples:        {res['n_samples_analyzed']} (subsampled 10:1)")
    print(f"Observation Span:        {res['total_span_hours']} hours ({res['total_span_hours']/24.0:.1f} days)")
    print(f"Secular Drift Rate:      {res['secular_drift_mm_per_day']:+.2f} mm/day")
    print(f"Transmitter Freq Offset: {res['carrier_offset_mhz']:+.3f} mHz ({res['fractional_frequency_offset']})")
    print(f"Diurnal Wander (PtP):    {res['diurnal_wander_ptp_mm']:.2f} mm ({res['diurnal_wander_ptp_ps']:.1f} ps)")
    print(f"Semidiurnal Harmonic:    {res['semidiurnal_amp_mm']:.2f} mm")
    print(f"Residual RMS Scatter:    {res['unmodeled_rms_mm']:.2f} mm")
    print("=================================================================")
    print(f"State File:              {STATE_THERMAL_PATH}")


if __name__ == "__main__":
    main()
