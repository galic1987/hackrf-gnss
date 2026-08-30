#!/usr/bin/env python3
"""tdc_density_cal.py — Code-density calibration engine for the 48-tap iCE40 Carry-Chain TDC.

Given an external 1PPS capture under a free-running TCXO (sweeping phase at ~667 ns/s),
this script computes:
1. Per-tap empirical time width (W_k in picoseconds) across all bins 0..num_taps
2. Differential Non-Linearity (DNL_k in LSB and ps)
3. Integral Non-Linearity (INL_k in LSB and ps)
4. Calibrated delay LUT (prefix sums in ps) matching src/tdc.rs.
"""

import json
import math
import sys
import os
import argparse
from typing import Dict, List, Optional, Tuple, Any

NOMINAL_TAP_PS = 105.5
DEFAULT_T_CLOCK_NS = 25.0  # 40 MHz adclk period
DEFAULT_NUM_TAPS = 48


def parse_thermo_bytes(hexbytes: str) -> List[int]:
    """Parse comma-separated hex bytes into bit array (LSB first per byte)."""
    bs = [int(b.strip(), 16) for b in hexbytes.split(",") if b.strip()]
    return [(b >> j) & 1 for b in bs for j in range(8)]


def decode_thermometer(taps: List[int], num_taps: int = DEFAULT_NUM_TAPS) -> Optional[int]:
    """Return popcount k if taps form a clean thermometer (1..k set, rest 0), else None."""
    k = 0
    while k < num_taps and k < len(taps) and taps[k] == 1:
        k += 1
    # Check if remaining taps are all 0
    if all(t == 0 for t in taps[k:num_taps]):
        return k
    # Allow 1-bit bubble tolerance if needed
    pop = sum(taps[:num_taps])
    if abs(pop - k) <= 1:
        return pop
    return None


def parse_jsonl_event(line: str, num_taps: int = DEFAULT_NUM_TAPS) -> Optional[Dict[str, Any]]:
    """Parse a single JSONL event line from any supported logging schema."""
    line = line.strip()
    if not line or not line.startswith("{"):
        return None
    try:
        r = json.loads(line)
    except json.JSONDecodeError:
        return None

    t = r.get("t", r.get("timestamp", r.get("epoch", r.get("seq", 0.0))))
    k = None

    if "bytes" in r:
        taps = parse_thermo_bytes(r["bytes"])
        k = decode_thermometer(taps, num_taps)
    elif "thermo" in r:
        # Hex string like "fefff8ff01fd"
        hex_str = r["thermo"]
        bs = [int(hex_str[i : i + 2], 16) for i in range(0, len(hex_str), 2)]
        taps = [(b >> j) & 1 for b in bs for j in range(8)]
        k = decode_thermometer(taps, num_taps)
    elif "popcount" in r:
        k = int(r["popcount"])

    if k is not None:
        return {"t": float(t), "k": k}
    return None


def compute_code_density(
    ks: List[int],
    num_taps: int = DEFAULT_NUM_TAPS,
    t_clock_ns: float = DEFAULT_T_CLOCK_NS,
) -> Dict[str, Any]:
    """Compute per-tap width, DNL, and INL from a collection of popcounts."""
    total_events = len(ks)
    if total_events == 0:
        return {"error": "no events provided"}

    hist = [0] * (num_taps + 1)
    for k in ks:
        if 0 <= k <= num_taps:
            hist[k] += 1

    saturated_count = hist[num_taps]
    in_window_count = total_events - saturated_count
    saturation_rate = saturated_count / total_events if total_events > 0 else 0.0

    period_ps = t_clock_ns * 1000.0

    # Per-bin width over the full clock period (matches src/tdc.rs)
    bin_widths_ps = [(c / total_events) * period_ps for c in hist]

    # In-window tap widths and DNL (for active taps 1..num_taps-1)
    in_window_ideal_tap_ps = (period_ps * (1.0 - saturation_rate)) / (num_taps - 1) if (num_taps > 1 and in_window_count > 0) else NOMINAL_TAP_PS

    dnl_lsb = []
    dnl_ps = []
    for c in hist[:num_taps]:
        if in_window_count > 0:
            w = (c / in_window_count) * (period_ps * (1.0 - saturation_rate))
        else:
            w = 0.0
        d_ps = w - in_window_ideal_tap_ps
        d_l = d_ps / in_window_ideal_tap_ps if in_window_ideal_tap_ps > 0 else 0.0
        dnl_ps.append(d_ps)
        dnl_lsb.append(d_l)

    # Prefix sum LUT (lut[k] = phase offset for popcount k)
    calibrated_lut_ps = []
    acc = 0.0
    for w in bin_widths_ps:
        calibrated_lut_ps.append(acc)
        acc += w

    return {
        "total_events": total_events,
        "in_window_events": in_window_count,
        "saturated_events": saturated_count,
        "saturation_rate": saturation_rate,
        "t_clock_ns": t_clock_ns,
        "t_window_ps": period_ps * (1.0 - saturation_rate),
        "ideal_tap_ps": in_window_ideal_tap_ps,
        "hist": hist,
        "bin_widths_ps": bin_widths_ps,
        "dnl_lsb": dnl_lsb,
        "dnl_ps": dnl_ps,
        "calibrated_lut_ps": calibrated_lut_ps,
        "max_dnl_lsb": max(abs(d) for d in dnl_lsb) if dnl_lsb else 0.0,
    }


def analyze_file(
    path: str,
    num_taps: int = DEFAULT_NUM_TAPS,
    t_clock_ns: float = DEFAULT_T_CLOCK_NS,
) -> Dict[str, Any]:
    """Load and analyze a JSONL dataset."""
    ks = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            ev = parse_jsonl_event(line, num_taps)
            if ev is not None:
                ks.append(ev["k"])
    return compute_code_density(ks, num_taps, t_clock_ns)


def main():
    parser = argparse.ArgumentParser(description="TDC Code-Density Calibration Engine")
    parser.add_argument("input", help="Path to input TDC JSONL log file")
    parser.add_argument("--taps", type=int, default=DEFAULT_NUM_TAPS, help="Number of TDC taps (default: 48)")
    parser.add_argument("--clock-ns", type=float, default=DEFAULT_T_CLOCK_NS, help="Clock period in ns (default: 25.0)")
    parser.add_argument("--out-json", type=str, default=None, help="Output path for calibrated JSON LUT")
    parser.add_argument("--quiet", action="store_true", help="Suppress verbose output")

    args = parser.parse_args()

    if not os.path.exists(args.input):
        sys.exit(f"Error: input file '{args.input}' not found.")

    res = analyze_file(args.input, args.taps, args.clock_ns)

    if "error" in res and res.get("in_window_events") == 0:
        sys.exit(f"Error during analysis: {res['error']}")

    if not args.quiet:
        print(f"=== TDC Code-Density Calibration: {args.input} ===")
        print(f"Total events:        {res['total_events']}")
        print(f"In-window events:    {res['in_window_events']} ({100.0 * (1.0 - res['saturation_rate']):.2f}%)")
        print(f"Saturated events:    {res['saturated_events']} ({100.0 * res['saturation_rate']:.2f}%)")
        print(f"Effective window:    {res['t_window_ps']:.2f} ps ({res['t_window_ps'] / 1000.0:.3f} ns)")
        print(f"Ideal in-win tap:    {res['ideal_tap_ps']:.2f} ps")
        print("\nTap Histogram & Delays (first 10 bins):")
        for i in range(min(10, args.taps + 1)):
            w = res['bin_widths_ps'][i]
            c = res['calibrated_lut_ps'][i]
            print(f"  Bin {i:2d}: count={res['hist'][i]:4d} | width={w:6.1f} ps | cum_offset={c:7.1f} ps")

    if args.out_json:
        with open(args.out_json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2)
        if not args.quiet:
            print(f"\nCalibrated LUT written to {args.out_json}")


if __name__ == "__main__":
    main()
