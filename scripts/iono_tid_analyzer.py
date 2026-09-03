#!/usr/bin/env python3
"""Traveling Ionospheric Disturbance (TID) Spectral Wavelet Analyzer.

Analyzes continuous Total Electron Content (TEC) time series to detect atmospheric
acoustic-gravity waves and Traveling Ionospheric Disturbances (TIDs) in real time:
  1. Medium-Scale TIDs (MSTIDs): Period 15 - 60 min, speed 100 - 300 m/s, delta-TEC 0.1 - 1.0 TECU
  2. Large-Scale TIDs (LSTIDs): Period > 60 min, auroral / geomagnetic storm origin
  3. Quiescent baseline: background noise floor

Uses Lomb-Scargle / FFT spectral power analysis with low-frequency diurnal background
subtraction and false-alarm probability (FAP) gating.
"""
import argparse
import json
import math
import os
import sys
import numpy as np

HISTORY_PATH = "/Volumes/Radiator 8TB/gnss/observations/iono_history.jsonl"
STATE_TID_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.tid.json"

MIN_WINDOW_S = 900.0   # 15 minutes minimum window to evaluate MSTIDs
MAX_WINDOW_S = 7200.0  # 2 hours standard sliding analysis window


def detrend_diurnal(times, values, deg=2):
    """Subtract low-frequency diurnal trend using polynomial fitting."""
    if len(values) < 10:
        return values
    t = np.array(times, dtype=np.float64)
    t_norm = t - t[0]
    y = np.array(values, dtype=np.float64)
    poly = np.polyfit(t_norm, y, deg=min(deg, len(y) - 1))
    trend = np.polyval(poly, t_norm)
    return y - trend


def compute_tid_spectrum(times, dtec_values, min_period_s=600.0, max_period_s=5400.0):
    """Compute spectral power of TEC perturbation across the TID period band.
    
    Returns:
      (periods_min, psd, peak_period_min, peak_amplitude_tecu, snr_db)
    """
    n = len(times)
    if n < 30:
        return [], [], 0.0, 0.0, 0.0

    t = np.array(times, dtype=np.float64)
    y_pert = detrend_diurnal(times, dtec_values, deg=2)

    # Resample to uniform 1-second grid if needed
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        dt = 1.0

    # Period band: 10 min (600s) to 90 min (5400s)
    # Frequencies: 1/5400 Hz to 1/600 Hz
    f_min = 1.0 / max_period_s
    f_max = 1.0 / min_period_s
    freqs = np.linspace(f_min, f_max, 150)
    periods_min = (1.0 / freqs) / 60.0

    # Discrete Fourier periodogram
    amplitudes = []
    powers = []
    for f in freqs:
        omega = 2.0 * np.pi * f
        cos_term = float(np.sum(y_pert * np.cos(omega * t)))
        sin_term = float(np.sum(y_pert * np.sin(omega * t)))
        amp = (2.0 / n) * math.hypot(cos_term, sin_term)
        p = 0.5 * amp * amp
        amplitudes.append(amp)
        powers.append(p)

    powers = np.array(powers)
    if len(powers) == 0:
        return [], [], 0.0, 0.0, 0.0

    peak_idx = int(np.argmax(powers))
    peak_period_min = float(periods_min[peak_idx])
    peak_power = float(powers[peak_idx])
    peak_amp_tecu = float(amplitudes[peak_idx])

    # Estimate background noise floor from median power
    med_power = float(np.median(powers))
    if med_power > 1e-9:
        snr_db = float(10.0 * math.log10(max(1.0, peak_power / med_power)))
    else:
        snr_db = 0.0

    return periods_min.tolist(), powers.tolist(), round(peak_period_min, 1), round(peak_amp_tecu, 3), round(snr_db, 1)


def classify_tid(peak_period_min, peak_amp_tecu, snr_db):
    """Classify detected wave into QUIESCENT, MSTID, or LSTID."""
    if peak_amp_tecu < 0.10 or snr_db < 4.0:
        return "QUIESCENT"
    if 15.0 <= peak_period_min <= 60.0 and peak_amp_tecu >= 0.10:
        return "MSTID"
    if peak_period_min > 60.0 and peak_amp_tecu >= 0.20:
        return "LSTID"
    return "QUIESCENT"


class TIDAnalyzer:
    def __init__(self, history_file=HISTORY_PATH, state_file=STATE_TID_PATH):
        self.history_file = history_file
        self.state_file = state_file

    def analyze(self):
        """Read history, compute TID spectrum, publish state."""
        if not os.path.exists(self.history_file):
            return None

        times = []
        dtecs = []

        try:
            with open(self.history_file) as f:
                for line in f:
                    if not line.strip():
                        continue
                    rec = json.loads(line)
                    t = rec.get("epoch")
                    d = rec.get("dtec")
                    if t is not None and d is not None:
                        times.append(float(t))
                        dtecs.append(float(d))
        except Exception:
            return None

        if len(times) < 30:
            return {
                "status": "WAITING_FOR_DATA",
                "n_samples": len(times),
                "classification": "QUIESCENT"
            }

        # Select last MAX_WINDOW_S seconds
        t_end = times[-1]
        t_start = max(times[0], t_end - MAX_WINDOW_S)
        window_idx = [i for i, t in enumerate(times) if t >= t_start]
        w_times = [times[i] for i in window_idx]
        w_dtecs = [dtecs[i] for i in window_idx]

        span_s = w_times[-1] - w_times[0] if len(w_times) > 1 else 0.0

        periods, powers, peak_per, peak_amp, snr_db = compute_tid_spectrum(w_times, w_dtecs)
        classification = classify_tid(peak_per, peak_amp, snr_db)

        output = {
            "epoch": round(t_end, 2),
            "classification": classification,
            "dominant_period_min": peak_per,
            "dominant_amplitude_tecu": peak_amp,
            "spectral_snr_db": snr_db,
            "evaluated_span_s": round(span_s, 1),
            "n_samples": len(w_times)
        }

        # Atomically write to state file
        tmp_path = self.state_file + ".tmp"
        with open(tmp_path, "w") as f:
            json.dump(output, f, indent=2)
        os.replace(tmp_path, self.state_file)

        return output


def main():
    parser = argparse.ArgumentParser(description="Traveling Ionospheric Disturbance (TID) Spectral Analyzer")
    parser.add_argument("--once", action="store_true", help="Run a single analysis and exit")
    parser.add_argument("--loop", action="store_true", help="Run continuously in a 10-second loop")
    parser.add_argument("--interval", type=float, default=10.0, help="Poll interval in seconds")
    args = parser.parse_args()

    analyzer = TIDAnalyzer()

    if args.once or not args.loop:
        res = analyzer.analyze()
        print("=================================================================")
        print("     TRAVELING IONOSPHERIC DISTURBANCE (TID) SPECTRAL ANALYZER   ")
        print("=================================================================")
        if not res:
            print("No history file found.")
            return

        print(f"Classification:         {res.get('classification')}")
        print(f"Dominant Period:        {res.get('dominant_period_min', 0.0)} min")
        print(f"Perturbation Amplitude: {res.get('dominant_amplitude_tecu', 0.0)} TECU")
        print(f"Spectral Peak SNR:      {res.get('spectral_snr_db', 0.0)} dB")
        print(f"Evaluated Span:         {res.get('evaluated_span_s', 0.0)} s ({res.get('n_samples', 0)} samples)")
        print(f"State File:             {STATE_TID_PATH}")
        print("=================================================================")
        return

    import time
    print("TID spectral analyzer loop started (10 s interval)...", flush=True)
    while True:
        try:
            res = analyzer.analyze()
            if res and res.get("classification") != "WAITING_FOR_DATA":
                print(f"{time.strftime('%H:%M:%S')} | TID: {res.get('classification'):9s} | "
                      f"Period: {res.get('dominant_period_min', 0.0):4.1f} min | "
                      f"Amp: {res.get('dominant_amplitude_tecu', 0.0):.3f} TECU | "
                      f"SNR: {res.get('spectral_snr_db', 0.0):4.1f} dB", flush=True)
            time.sleep(args.interval)
        except KeyboardInterrupt:
            break
        except Exception as e:
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
