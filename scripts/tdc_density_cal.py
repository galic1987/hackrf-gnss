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
TCXO_CLOCK_NS = 25.0  # 40 MHz adclk (TCXO / XTAL mode, run1 lineage)
CLKIN_CLOCK_NS = 31.25  # 32 MHz adclk (CLKIN / Bodnar-clocked mode)
DEFAULT_NUM_TAPS = 48

# Fail-closed: there is NO default clock period. Station captures exist at
# both 25 ns and 31.25 ns; a guessed default silently mis-scales every
# width/DNL/LUT value by 20%.
MISSING_CLOCK_ERROR = (
    "clock period unknown: the input file carries no clock metadata and no "
    "--clock-ns was given. This station has captures at BOTH "
    f"{TCXO_CLOCK_NS} ns (TCXO-mode, 40 MHz adclk) and {CLKIN_CLOCK_NS} ns "
    "(CLKIN-mode, 32 MHz adclk); a guessed default would silently mis-scale "
    "every width/DNL/LUT value by 20%, so guessing is forbidden. Re-run with "
    f"--clock-ns {TCXO_CLOCK_NS} (TCXO-mode) or --clock-ns {CLKIN_CLOCK_NS} "
    "(CLKIN-mode), or use a capture whose config/metadata row records the clock."
)


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


def metadata_clock_ns(r: Dict[str, Any]) -> Optional[float]:
    """Extract the adclk period (ns) from a config/metadata row, if it carries one.

    Supported keys (in-repo conventions): "clock_ns"/"t_clock_ns" (direct),
    "adclk_hz" (tdc_cal_analyze.py), "sample_rate" with adclk = 2x sample rate
    (ts_latch_run3 header per tdc_ts_analyze.py).
    """
    try:
        if "clock_ns" in r:
            v = float(r["clock_ns"])
        elif "t_clock_ns" in r:
            v = float(r["t_clock_ns"])
        elif "adclk_hz" in r:
            v = 1e9 / float(r["adclk_hz"])
        elif "sample_rate" in r:
            v = 1e9 / (2.0 * float(r["sample_rate"]))
        else:
            return None
    except (TypeError, ValueError, ZeroDivisionError):
        return None
    # Plausibility gate: an adclk period is a few tens of ns, not a clock-bias.
    if not (1.0 <= v <= 1000.0):
        return None
    return v


def compute_code_density(
    ks: List[int],
    num_taps: int = DEFAULT_NUM_TAPS,
    t_clock_ns: Optional[float] = None,
) -> Dict[str, Any]:
    """Compute per-tap width, DNL, and INL from a collection of popcounts.

    t_clock_ns is REQUIRED (fail-closed): pass the metadata- or CLI-resolved
    period. There is no default — see MISSING_CLOCK_ERROR.
    """
    if t_clock_ns is None:
        return {"error": MISSING_CLOCK_ERROR}
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

    # In-window tap widths and DNL. The DNL loop below covers hist[:num_taps],
    # i.e. the num_taps (48) in-window bins k = 0..num_taps-1 (bin num_taps is
    # the saturated bin, excluded) — so the ideal width divides the window by
    # num_taps, matching the uniform per-bin convention of src/tdc.rs.
    in_window_ideal_tap_ps = (period_ps * (1.0 - saturation_rate)) / num_taps if (num_taps > 0 and in_window_count > 0) else NOMINAL_TAP_PS

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

    # --- Coherence guard statistics ---------------------------------------
    # Code-density calibration is only valid for INCOHERENT sampling (TCXO
    # mode: the -0.667 ppm offset slews the PPS phase ~667 ns/s, so 1 Hz
    # samples land quasi-uniformly). A coherent (CLKIN/Bodnar-clocked)
    # capture parks every sample on the same phase: either ~zero in-window
    # hits, or in-window hits arriving in long consecutive clusters.
    # (a) Window fraction: the chain structurally covers
    #     num_taps * NOMINAL_TAP_PS of the period (~20.3% at 25 ns), derived
    #     here rather than hardcoded so it scales with --taps / --clock-ns.
    # (b) Consecutive-hit run lengths on the ordered in-window/saturated
    #     indicator. The TCXO slew is a deterministic rotation (fractional
    #     advance ~17 ns/s mod the 25 ns period), so a legitimate incoherent
    #     run shows short hit runs (run1: mean 2.0, max 7) — NOT Bernoulli
    #     independence, so a plain runs-test z is over-sensitive (run1 gives
    #     z=-23). A coherent capture instead dwells: at the Bodnar-null slew
    #     (~0.026 ns/s, run3) the phase sits in the ~5 ns window for >=150
    #     consecutive 1 Hz samples, or parks in one bin for the whole run.
    expected_in_window_frac = min(1.0, (num_taps * NOMINAL_TAP_PS) / period_ps)
    measured_in_window_frac = in_window_count / total_events
    MAX_MEAN_HIT_RUN = 8.0  # 4x the run1 lineage mean, 20x the Bernoulli mean
    MAX_SINGLE_HIT_RUN = 50  # no plausible TCXO slew dwells in-window this long
    coherence_reasons = []
    mean_hit_run = None
    max_hit_run = None
    if in_window_count == 0:
        coherence_reasons.append(
            "zero in-window hits — every event saturated the chain "
            "(PPS phase parked past the window, as in a Bodnar-coherent run)"
        )
    else:
        frac_lo = 0.5 * expected_in_window_frac
        frac_hi = min(1.0, 2.0 * expected_in_window_frac)
        if not (frac_lo <= measured_in_window_frac <= frac_hi):
            coherence_reasons.append(
                f"in-window fraction {measured_in_window_frac:.3f} outside "
                f"[{frac_lo:.3f}, {frac_hi:.3f}] expected from the "
                f"{num_taps}-tap window (structural {expected_in_window_frac:.3f})"
            )
        seq = [1 if k < num_taps else 0 for k in ks if 0 <= k <= num_taps]
        hit_runs = []
        cur = 0
        for s in seq:
            if s:
                cur += 1
            elif cur:
                hit_runs.append(cur)
                cur = 0
        if cur:
            hit_runs.append(cur)
        if hit_runs:
            mean_hit_run = sum(hit_runs) / len(hit_runs)
            max_hit_run = max(hit_runs)
            if mean_hit_run > MAX_MEAN_HIT_RUN or max_hit_run >= MAX_SINGLE_HIT_RUN:
                coherence_reasons.append(
                    f"in-window hits strongly clustered (mean consecutive-hit "
                    f"run {mean_hit_run:.1f}, max {max_hit_run}; incoherent "
                    f"limit mean {MAX_MEAN_HIT_RUN:g} / max {MAX_SINGLE_HIT_RUN}): "
                    "the phase is dwelling, not slewing"
                )
    coherence = {
        "expected_in_window_frac": expected_in_window_frac,
        "measured_in_window_frac": measured_in_window_frac,
        "mean_hit_run": mean_hit_run,
        "max_hit_run": max_hit_run,
        "coherent_suspect": bool(coherence_reasons),
        "reasons": coherence_reasons,
    }

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
        "coherence": coherence,
    }


def analyze_file(
    path: str,
    num_taps: int = DEFAULT_NUM_TAPS,
    t_clock_ns: Optional[float] = None,
) -> Dict[str, Any]:
    """Load and analyze a JSONL dataset.

    Clock resolution is FAIL-CLOSED: a config/metadata row in the file wins;
    otherwise t_clock_ns (from --clock-ns) is used; with neither, an error
    result is returned (MISSING_CLOCK_ERROR). No silent default.
    """
    ks = []
    meta_clock_ns = None
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if meta_clock_ns is None and s.startswith("{") and (
                '"kind"' in s or '"metadata"' in s
            ):
                try:
                    r = json.loads(s)
                except json.JSONDecodeError:
                    r = None
                if isinstance(r, dict) and (r.get("kind") == "config" or "metadata" in r):
                    meta_clock_ns = metadata_clock_ns(r)
            ev = parse_jsonl_event(line, num_taps)
            if ev is not None:
                ks.append(ev["k"])

    if meta_clock_ns is not None:
        if t_clock_ns is not None and abs(meta_clock_ns - t_clock_ns) > 1e-3 * meta_clock_ns:
            return {
                "error": (
                    f"clock mismatch: run metadata records {meta_clock_ns:g} ns "
                    f"but --clock-ns says {t_clock_ns:g} ns — fix one; "
                    "refusing to guess which is right."
                )
            }
        resolved, source = meta_clock_ns, "run metadata"
    elif t_clock_ns is not None:
        resolved, source = t_clock_ns, "--clock-ns"
    else:
        return {"error": MISSING_CLOCK_ERROR}

    res = compute_code_density(ks, num_taps, resolved)
    if "error" not in res:
        res["t_clock_source"] = source
    return res


def main():
    parser = argparse.ArgumentParser(description="TDC Code-Density Calibration Engine")
    parser.add_argument("input", help="Path to input TDC JSONL log file")
    parser.add_argument("--taps", type=int, default=DEFAULT_NUM_TAPS, help="Number of TDC taps (default: 48)")
    parser.add_argument(
        "--clock-ns", type=float, default=None,
        help=(
            "Clock period in ns. REQUIRED unless the input file carries clock "
            f"metadata (which wins). Station values: {TCXO_CLOCK_NS} "
            f"(TCXO-mode) / {CLKIN_CLOCK_NS} (CLKIN-mode). No default — "
            "guessing mis-scales the calibration by 20%."
        ),
    )
    parser.add_argument("--out-json", type=str, default=None, help="Output path for calibrated JSON LUT")
    parser.add_argument("--quiet", action="store_true", help="Suppress verbose output")
    parser.add_argument(
        "--force", action="store_true",
        help="Override the coherent-capture guard (loud warning; results suspect)",
    )

    args = parser.parse_args()

    if not os.path.exists(args.input):
        sys.exit(f"Error: input file '{args.input}' not found.")

    res = analyze_file(args.input, args.taps, args.clock_ns)

    if "error" in res:
        sys.exit(f"Error: {res['error']}")

    coh = res.get("coherence", {})
    if coh.get("coherent_suspect"):
        if args.force:
            banner = "!" * 78
            print(banner, file=sys.stderr)
            print("!! WARNING: COHERENT-CAPTURE guard OVERRIDDEN by --force.", file=sys.stderr)
            print("!! This capture does not look like incoherent (TCXO-mode) sampling:", file=sys.stderr)
            for reason in coh["reasons"]:
                print(f"!!   - {reason}", file=sys.stderr)
            print("!! The DNL/LUT below is only valid if you can independently prove the", file=sys.stderr)
            print("!! capture was incoherent (phase slewing across the window).", file=sys.stderr)
            print(banner, file=sys.stderr)
        else:
            msg = ["COHERENT-CAPTURE verdict: refusing to calibrate from this capture."]
            for reason in coh["reasons"]:
                msg.append(f"  - {reason}")
            msg.append(
                "Code-density DNL from a coherent (CLKIN/Bodnar-clocked) capture is "
                "garbage: every sample lands on the same phase, so the histogram no "
                "longer measures tap widths. Use a TCXO-mode capture "
                "(scripts/tdc_sweep_window.sh), or re-run with --force to override."
            )
            sys.exit("\n".join(msg))

    if not args.quiet:
        print(f"=== TDC Code-Density Calibration: {args.input} ===")
        print(f"Clock period:        {res['t_clock_ns']:g} ns (source: {res.get('t_clock_source', 'unknown')})")
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
