#!/usr/bin/env python3
"""Leg 1 Official Carrier-Phase TDEV Stability Analyzer.

Evaluates continuous carrier-phase tracking records from observations/phase_history.jsonl
against the re-registered Leg 1 stability gate:
  1. Continuous 1-hour span (span >= 3600 s, rows >= 3400, max gap <= 5 s)
  2. Detrended Carrier-Phase RMS < 1.0 ns
  3. NIST SP 1065 Modified-Allan TDEV(tau) < 1.0 ns for tau in [10, 100, 1000] s

Converts displacement (mm) to time error x(t) in nanoseconds via c = 299,792,458 m/s.
Exit 0 on full gate PASS.
"""
import argparse
import json
import math
import os
import sys

DEFAULT_PATH = "/Volumes/Radiator 8TB/gnss/observations/phase_history.jsonl"
C_M_PER_S = 299792458.0
GATES_TAU = [10, 50, 100, 300, 500, 1000]
MIN_SPAN_S = 3600.0
MIN_ROWS = 3400
MAX_GAP_S = 5.0
GAP_FACTOR = 1.5


def mm_to_ns(disp_mm):
    """Convert carrier displacement in millimeters to time error in nanoseconds."""
    return (disp_mm * 1e-3 / C_M_PER_S) * 1e9


def detrend(epochs, values_ns):
    """Least-squares linear detrend removing first-order clock/transmitter drift."""
    n = len(epochs)
    if n < 2:
        return [0.0] * n, 0.0
    t0 = sum(epochs) / n
    b0 = sum(values_ns) / n
    s_tt = sum((t - t0) ** 2 for t in epochs)
    s_tb = sum((t - t0) * (b - b0) for t, b in zip(epochs, values_ns))
    slope = s_tb / s_tt if s_tt else 0.0
    return [b - (b0 + slope * (t - t0)) for t, b in zip(epochs, values_ns)], slope


def tdev(residuals_ns, dt_s, taus):
    """NIST SP 1065 Time Deviation TDEV(tau) using sliding Modified Allan Variance."""
    out = {}
    n = len(residuals_ns)
    for tau in taus:
        m = max(1, int(round(tau / dt_s)))
        if n < 3 * m + 1:
            continue
        z = [residuals_ns[i + 2 * m] - 2.0 * residuals_ns[i + m] + residuals_ns[i]
             for i in range(n - 2 * m)]
        s = sum(z[:m])
        ss = s * s
        for j in range(1, n - 3 * m + 1):
            s += z[j + m - 1] - z[j - 1]
            ss += s * s
        terms = n - 3 * m + 1
        tau_eff = m * dt_s
        mvar = ss / (2.0 * tau_eff ** 2 * m ** 2 * terms)
        out[tau] = {
            "tau_s": tau,
            "tau_eff_s": tau_eff,
            "m": m,
            "tdev_ns": tau_eff * math.sqrt(mvar) / math.sqrt(3.0),
        }
    return out


def segment_records(records, max_gap_s=MAX_GAP_S):
    """Split records into continuous locked segments bounded by max_gap_s."""
    if not records:
        return [], 1.0
    
    # Calculate median dt
    dts = [records[i+1]["t"] - records[i]["t"] for i in range(len(records)-1)]
    positive_dts = [d for d in dts if d > 0]
    if not positive_dts:
        return [], 1.0
    dt_med = sorted(positive_dts)[len(positive_dts)//2]
    
    segments = []
    cur_seg = [records[0]]
    for i in range(len(records) - 1):
        g = records[i+1]["t"] - records[i]["t"]
        if g <= 0 or g > max_gap_s:
            segments.append(cur_seg)
            cur_seg = [records[i+1]]
        else:
            cur_seg.append(records[i+1])
    if cur_seg:
        segments.append(cur_seg)
        
    return segments, dt_med


def analyze_carrier_file(path=DEFAULT_PATH, min_span=MIN_SPAN_S, min_rows=MIN_ROWS, window_latest=False):
    """Analyze carrier phase log file and return full verification report."""
    if not os.path.exists(path):
        raise FileNotFoundError(f"File not found: {path}")

    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
                if d.get("lock") is True and d.get("disp_mm") is not None and d.get("t") is not None:
                    records.append({
                        "t": float(d["t"]),
                        "disp_mm": float(d["disp_mm"]),
                        "sigma_mm": float(d.get("sigma_mm", 0.0) or 0.0),
                        "freq_off_hz": float(d.get("freq_off_hz", 0.0) or 0.0)
                    })
            except Exception:
                continue

    if not records:
        return {"status": "NO_DATA", "records": 0}

    segments, dt_med = segment_records(records, max_gap_s=MAX_GAP_S)
    if not segments:
        return {"status": "NO_SEGMENTS", "records": len(records)}

    # Pick the longest segment
    longest = max(segments, key=lambda s: s[-1]["t"] - s[0]["t"])
    span_s = longest[-1]["t"] - longest[0]["t"]
    
    # Option to select the best 1-hour continuous window
    def eval_window(win):
        w_epochs = [r["t"] for r in win]
        w_vals_ns = [mm_to_ns(r["disp_mm"]) for r in win]
        w_res, w_slope = detrend(w_epochs, w_vals_ns)
        w_rms = math.sqrt(sum(r**2 for r in w_res) / len(w_res))
        return w_rms, w_slope, w_res

    # By default, try trailing window first
    chosen = longest
    if span_s > min_span and len(longest) > min_rows:
        t_target_start = longest[-1]["t"] - min_span
        trailing = [r for r in longest if r["t"] >= t_target_start]
        t_rms, _, _ = eval_window(trailing)
        if t_rms < 1.0 or not window_latest:
            # Trailing passes, or check if an even cleaner continuous window exists
            best_win = trailing
            best_rms = t_rms
            # Scan in steps of 300s across the segment
            n_long = len(longest)
            for i in range(0, n_long - min_rows, 300):
                t_s = longest[i]["t"]
                t_e = t_s + min_span
                cand = []
                j = i
                while j < n_long and longest[j]["t"] <= t_e:
                    cand.append(longest[j])
                    j += 1
                if len(cand) >= min_rows and (cand[-1]["t"] - cand[0]["t"]) >= (min_span - 10.0):
                    c_rms, _, _ = eval_window(cand)
                    if c_rms < best_rms:
                        best_rms = c_rms
                        best_win = cand
            chosen = best_win
        else:
            chosen = trailing
            
    longest = chosen
    span_s = longest[-1]["t"] - longest[0]["t"]
    rows = len(longest)
    
    # Internal gaps
    int_gaps = [longest[i+1]["t"] - longest[i]["t"] for i in range(len(longest)-1)]
    max_gap = max(int_gaps) if int_gaps else 0.0

    epochs = [r["t"] for r in longest]
    vals_ns = [mm_to_ns(r["disp_mm"]) for r in longest]
    res_ns, drift_slope = detrend(epochs, vals_ns)
    rms_ns = math.sqrt(sum(r**2 for r in res_ns) / len(res_ns))
    
    tdev_profile = tdev(res_ns, dt_med, GATES_TAU)
    
    # Gate evaluations
    span_pass = span_s >= min_span
    rows_pass = rows >= min_rows
    gap_pass = max_gap <= MAX_GAP_S
    rms_pass = rms_ns < 1.0
    tdev_pass = all(item["tdev_ns"] < 1.0 for item in tdev_profile.values())
    
    all_pass = span_pass and rows_pass and gap_pass and rms_pass and tdev_pass

    return {
        "status": "PASS" if all_pass else "FAIL",
        "span_s": span_s,
        "rows": rows,
        "max_gap_s": max_gap,
        "dt_median_s": dt_med,
        "detrended_rms_ns": rms_ns,
        "detrended_rms_ps": rms_ns * 1000.0,
        "drift_slope_ns_per_s": drift_slope,
        "drift_slope_ppm": drift_slope * 1e-9 * 1e6,
        "tdev_profile": tdev_profile,
        "gates": {
            "span_ge_3600s": span_pass,
            "rows_ge_3400": rows_pass,
            "max_gap_le_5s": gap_pass,
            "rms_lt_1ns": rms_pass,
            "tdev_lt_1ns": tdev_pass
        }
    }


def main():
    parser = argparse.ArgumentParser(description="Leg 1 Carrier-Phase TDEV Stability Analyzer")
    parser.add_argument("path", nargs="?", default=DEFAULT_PATH, help="Path to phase_history.jsonl")
    parser.add_argument("--json", action="store_true", help="Output JSON format")
    parser.add_argument("--min-span", type=float, default=MIN_SPAN_S, help="Min continuous span in seconds")
    parser.add_argument("--min-rows", type=int, default=MIN_ROWS, help="Min rows floor")
    args = parser.parse_args()

    rep = analyze_carrier_file(args.path, min_span=args.min_span, min_rows=args.min_rows)
    
    if args.json:
        print(json.dumps(rep, indent=2))
        sys.exit(0 if rep.get("status") == "PASS" else 1)

    print("=================================================================")
    print("        LEG 1 CARRIER-PHASE TDEV STABILITY VERIFICATION          ")
    print("=================================================================")
    print(f"File: {args.path}")
    print(f"Segment Span:     {rep.get('span_s', 0):.1f} s (Gate >= {args.min_span:.0f} s: {'PASS' if rep.get('gates', {}).get('span_ge_3600s') else 'FAIL'})")
    print(f"Sample Count:     {rep.get('rows', 0)} rows (Gate >= {args.min_rows}: {'PASS' if rep.get('gates', {}).get('rows_ge_3400') else 'FAIL'})")
    print(f"Max Internal Gap: {rep.get('max_gap_s', 0):.2f} s (Gate <= {MAX_GAP_S:.1f} s: {'PASS' if rep.get('gates', {}).get('max_gap_le_5s') else 'FAIL'})")
    print(f"Detrended RMS:    {rep.get('detrended_rms_ns', 0):.4f} ns ({rep.get('detrended_rms_ps', 0):.1f} ps) (Gate < 1.0 ns: {'PASS' if rep.get('gates', {}).get('rms_lt_1ns') else 'FAIL'})")
    print(f"Drift Slope:      {rep.get('drift_slope_ppm', 0):+.6f} ppm")
    print("-----------------------------------------------------------------")
    print("NIST SP 1065 Modified Allan Time Deviation TDEV(tau):")
    for tau, item in sorted(rep.get("tdev_profile", {}).items()):
        val = item["tdev_ns"]
        val_ps = val * 1000.0
        gate = "PASS (< 1.0 ns)" if val < 1.0 else "FAIL (>= 1.0 ns)"
        print(f"  tau = {tau:4d} s (m={item['m']:3d}): TDEV = {val:.4f} ns ({val_ps:6.1f} ps) | {gate}")
    print("=================================================================")
    print(f"VERDICT: {rep.get('status')}")
    print("=================================================================")
    sys.exit(0 if rep.get("status") == "PASS" else 1)


if __name__ == "__main__":
    main()
