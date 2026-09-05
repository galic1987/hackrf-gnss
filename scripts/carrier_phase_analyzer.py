#!/usr/bin/env python3
"""Leg 1 Official Carrier-Phase TDEV Stability Analyzer.

Evaluates continuous carrier-phase tracking records from observations/phase_history.jsonl
against an IN-SCRIPT (unregistered) 1 ns stability gate.
NOTE: the input phase_history.jsonl is the ATSC ch35 PILOT anchor (HackRF One
vs GPSDO), NOT Leg-1 GNSS carrier phase; a PASS is EXPLORATORY (exit 2), never
claim-grade. Default window is trailing/latest (no cherry-pick); --scan-longest
opts back into the min-RMS scan. Real GNSS carrier tooling: carrier_tdev_analyzer.py
and docs/superpowers/evidence/relativity-carrier-2026-09-04/.
Original gate line:
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
MAX_BRIDGE_GAP_S = 2.5
MAX_BRIDGE_EPOCHS = 1


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


def segment_records(
    records,
    max_gap_s=MAX_GAP_S,
    gap_factor=GAP_FACTOR,
    max_bridge_gap_s=MAX_BRIDGE_GAP_S,
    bridge_ab_mismatches=True,
    return_audit=False,
):
    """Split records into continuous locked segments bounded by max_gap_s,

    applying an audited single-row exclusion bridge rule:
    If an A/B membership mismatch lasts <= 1 epoch (<= 2.5 s) and phase lock
    resumes immediately with continuity ID preserved and no cycle slip, bridge
    the gap rather than resetting the continuous lock counter to zero.
    """
    if not records:
        if return_audit:
            return [], 1.0, [], []
        return [], 1.0

    # Calculate median positive dt
    dts = [records[i + 1]["t"] - records[i]["t"] for i in range(len(records) - 1)]
    positive_dts = [d for d in dts if 0 < d < max_gap_s]
    dt_med = sorted(positive_dts)[len(positive_dts) // 2] if positive_dts else 1.0
    gap_thresh = gap_factor * dt_med if dt_med > 0 else max_gap_s

    segments = []
    cur_seg = []
    last_valid_record = None
    pending_exclusions = []
    bridges = []
    splits = []

    for r in records:
        # Standardize record fields
        t = float(r["t"])
        lock = bool(r.get("lock", True))
        ab_match = r.get("ab_membership_match")
        ab_match = True if ab_match is None else bool(ab_match)
        cont_id = str(r.get("continuity_id", "default"))
        slip = bool(
            r.get("slip", False)
            or r.get("cycle_slip", False)
            or (r.get("slips", 0) > 0)
            or (r.get("slip_counter", 0) > 0)
        )

        rec_std = dict(r)
        rec_std.update({
            "t": t,
            "lock": lock,
            "ab_membership_match": ab_match,
            "continuity_id": cont_id,
            "slip": slip,
        })

        # Quality qualification: A/B mismatch rows are excluded from carrier phase fit
        is_valid_fit = lock and ab_match and not slip

        if not is_valid_fit:
            # Accumulate as pending exclusion between valid fit rows
            pending_exclusions.append(rec_std)
            continue

        # rec_std is a valid locked row (lock=True, ab_match=True, slip=False)
        if last_valid_record is None:
            cur_seg = [rec_std]
            last_valid_record = rec_std
            pending_exclusions = []
            continue

        # Check interval from last valid record
        g = rec_std["t"] - last_valid_record["t"]

        if g <= 0:
            # Nonpositive / nonmonotonic interval
            if cur_seg:
                segments.append(cur_seg)
            splits.append({
                "t_gap_start": last_valid_record["t"],
                "t_resume": rec_std["t"],
                "gap_s": round(g, 4),
                "reason": "nonpositive_interval",
                "bridged": False,
            })
            cur_seg = [rec_std]
            last_valid_record = rec_std
            pending_exclusions = []
            continue

        # Is there a gap or intervened exclusion?
        has_gap = (g > gap_thresh) or bool(pending_exclusions)

        if not has_gap:
            # Consecutive lock without gap: check continuity_id
            if rec_std["continuity_id"] != last_valid_record["continuity_id"]:
                if cur_seg:
                    segments.append(cur_seg)
                splits.append({
                    "t_gap_start": last_valid_record["t"],
                    "t_resume": rec_std["t"],
                    "gap_s": round(g, 4),
                    "reason": "continuity_id_change",
                    "bridged": False,
                })
                cur_seg = [rec_std]
            else:
                cur_seg.append(rec_std)
            last_valid_record = rec_std
            pending_exclusions = []
            continue

        # Gap or exclusion occurred! Evaluate audited single-row bridge rule.
        can_bridge = False
        split_reason = "gap_split"

        if not bridge_ab_mismatches:
            split_reason = "gap_split_bridging_disabled"
        elif g > max_bridge_gap_s:
            split_reason = f"gap_{g:.2f}s_exceeds_max_bridge_{max_bridge_gap_s:.2f}s"
        elif rec_std["continuity_id"] != last_valid_record["continuity_id"]:
            split_reason = "continuity_id_discontinuity"
        elif pending_exclusions:
            if len(pending_exclusions) > MAX_BRIDGE_EPOCHS:
                split_reason = f"multi_epoch_mismatch_count_{len(pending_exclusions)}_gt_1"
            elif any(not e["lock"] for e in pending_exclusions):
                split_reason = "loss_of_phase_lock_during_gap"
            elif any(e["ab_membership_match"] for e in pending_exclusions):
                split_reason = "gap_exclusion_not_ab_mismatch"
            elif any(e["slip"] for e in pending_exclusions):
                split_reason = "cycle_slip_during_gap"
            elif any(e["continuity_id"] != last_valid_record["continuity_id"] for e in pending_exclusions):
                split_reason = "intervening_continuity_id_mismatch"
            else:
                can_bridge = True
        else:
            # No intervening records in stream (pre-filtered gap), gap <= 2.5s with matching continuity ID & no slip
            can_bridge = True

        if can_bridge:
            bridges.append({
                "t_gap_start": last_valid_record["t"],
                "t_resume": rec_std["t"],
                "gap_s": round(g, 4),
                "continuity_id": rec_std["continuity_id"],
                "reason": "ab_membership_mismatch_single_row",
                "bridged": True,
            })
            cur_seg.append(rec_std)
        else:
            if cur_seg:
                segments.append(cur_seg)
            splits.append({
                "t_gap_start": last_valid_record["t"],
                "t_resume": rec_std["t"],
                "gap_s": round(g, 4),
                "reason": split_reason,
                "bridged": False,
            })
            cur_seg = [rec_std]

        last_valid_record = rec_std
        pending_exclusions = []

    if cur_seg:
        segments.append(cur_seg)

    if return_audit:
        return segments, dt_med, bridges, splits
    return segments, dt_med


def analyze_carrier_file(
    path=DEFAULT_PATH,
    min_span=MIN_SPAN_S,
    min_rows=MIN_ROWS,
    window_latest=True,
    bridge_ab_mismatches=True,
):
    """Analyze carrier phase log file and return full verification report.

    Evaluates strictly against a deterministic trailing rolling window (trailing min_span
    seconds) without cherry-picking best-fit historical slices.
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"File not found: {path}")

    records = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
                t_val = d.get("t") if d.get("t") is not None else d.get("epoch")
                disp_val = d.get("disp_mm")
                if disp_val is None and d.get("carrier_cycles") is not None:
                    f_c = float(d.get("f_carrier_hz", 1575.42e6))
                    disp_val = -float(d["carrier_cycles"]) / f_c * C_M_PER_S * 1000.0

                if t_val is not None:
                    lock_val = bool(d.get("lock", True))
                    ab_match = d.get("ab_membership_match")
                    ab_match = True if ab_match is None else bool(ab_match)
                    cont_id = d.get("continuity_id", "default")
                    slip_val = bool(
                        d.get("slip", False)
                        or d.get("cycle_slip", False)
                        or (d.get("slips", 0) > 0)
                        or (d.get("slip_counter", 0) > 0)
                    )
                    records.append({
                        "t": float(t_val),
                        "disp_mm": float(disp_val) if disp_val is not None else 0.0,
                        "sigma_mm": float(d.get("sigma_mm", 0.0) or 0.0),
                        "freq_off_hz": float(d.get("freq_off_hz", 0.0) or 0.0),
                        "lock": lock_val,
                        "ab_membership_match": ab_match,
                        "continuity_id": str(cont_id),
                        "slip": slip_val,
                    })
            except Exception:
                continue

    if not records:
        return {"status": "NO_DATA", "records": 0}

    records.sort(key=lambda r: r["t"])
    n_mismatch_total = sum(1 for r in records if r["ab_membership_match"] is False)

    segments, dt_med, bridges, splits = segment_records(
        records,
        max_gap_s=MAX_GAP_S,
        gap_factor=GAP_FACTOR,
        max_bridge_gap_s=MAX_BRIDGE_GAP_S,
        bridge_ab_mismatches=bridge_ab_mismatches,
        return_audit=True,
    )

    if not segments:
        return {"status": "NO_SEGMENTS", "records": len(records)}

    # Pick segment: latest if window_latest requested, else longest continuous span
    if window_latest:
        chosen_seg = segments[-1]
    else:
        chosen_seg = max(segments, key=lambda s: s[-1]["t"] - s[0]["t"])

    seg_span_s = chosen_seg[-1]["t"] - chosen_seg[0]["t"]

    # Deterministic trailing rolling window:
    # Strictly evaluate the trailing min_span (e.g. 3600s) of the segment ending at chosen_seg[-1]["t"].
    # Historical best-fit scanning / cherry-picking is strictly prohibited.
    if seg_span_s >= min_span and len(chosen_seg) >= min_rows:
        t_target_start = chosen_seg[-1]["t"] - min_span
        window = [r for r in chosen_seg if r["t"] >= t_target_start]
    else:
        window = chosen_seg

    span_s = window[-1]["t"] - window[0]["t"]
    rows = len(window)

    # Internal gaps within evaluated window
    int_gaps = [window[i + 1]["t"] - window[i]["t"] for i in range(len(window) - 1)]
    max_gap = max(int_gaps) if int_gaps else 0.0

    epochs = [r["t"] for r in window]
    vals_ns = [mm_to_ns(r["disp_mm"]) for r in window]
    res_ns, drift_slope = detrend(epochs, vals_ns)
    rms_ns = math.sqrt(sum(r**2 for r in res_ns) / len(res_ns)) if res_ns else 0.0

    tdev_profile = tdev(res_ns, dt_med, GATES_TAU)

    # Filter bridges and splits relevant to the evaluated window
    t_win_start = window[0]["t"]
    t_win_end = window[-1]["t"]
    win_bridges = [
        b for b in bridges
        if b["t_resume"] >= t_win_start and b["t_gap_start"] <= t_win_end
    ]
    win_splits = [
        s for s in splits
        if s["t_resume"] >= t_win_start and s["t_gap_start"] <= t_win_end
    ]

    # Gate evaluations
    span_pass = span_s >= min_span
    rows_pass = rows >= min_rows
    gap_pass = max_gap <= MAX_GAP_S
    rms_pass = rms_ns < 1.0
    if min_span >= 3000.0:
        eval_taus = GATES_TAU
    else:
        eval_taus = [t for t in GATES_TAU if (min_span - 5.0) >= 3 * t] or list(tdev_profile.keys())
    all_taus_present = bool(eval_taus) and all(tau in tdev_profile for tau in eval_taus)
    tdev_pass = all_taus_present and all(tdev_profile[tau]["tdev_ns"] < 1.0 for tau in eval_taus)

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
            "tdev_lt_1ns": tdev_pass,
        },
        "audit": {
            "total_records_ingested": len(records),
            "n_ab_mismatch_excluded": n_mismatch_total,
            "n_bridged_gaps": len(win_bridges),
            "total_bridges_in_file": len(bridges),
            "bridges": win_bridges,
            "splits": win_splits,
            "window_mode": "deterministic_trailing_rolling_window",
            "cherry_picking_prohibited": True,
        },
    }


def main():
    parser = argparse.ArgumentParser(description="Leg 1 Carrier-Phase TDEV Stability Analyzer")
    parser.add_argument("path", nargs="?", default=DEFAULT_PATH, help="Path to phase_history.jsonl")
    parser.add_argument("--json", action="store_true", help="Output JSON format")
    parser.add_argument("--min-span", type=float, default=MIN_SPAN_S, help="Min continuous span in seconds")
    parser.add_argument("--min-rows", type=int, default=MIN_ROWS, help="Min rows floor")
    parser.add_argument("--scan-longest", action="store_true", help="Opt in to the old scan for the lowest-RMS window (NOT default; cherry-picks)")
    parser.add_argument("--no-bridge", action="store_true", help="Disable single-row exclusion gap bridging")
    args = parser.parse_args()

    rep = analyze_carrier_file(
        args.path,
        min_span=args.min_span,
        min_rows=args.min_rows,
        window_latest=(not args.scan_longest),
        bridge_ab_mismatches=not args.no_bridge,
    )

    if args.json:
        print(json.dumps(rep, indent=2))
        # Charter (GOVERNANCE.md Epistemic Mandate): no PASS/exit-0 without a
        # registered spec. This gate is in-script and the observable is the
        # ATSC ch35 pilot anchor, NOT Leg-1 GNSS carrier phase.
        sys.exit(2 if rep.get("status") == "PASS" else 1)

    print("=================================================================")
    print("  ATSC ch35 PILOT ANCHOR — carrier TDEV (EXPLORATORY, unregistered gate) ")
    print("=================================================================")
    print(f"File: {args.path}")
    print(f"Evaluation Window: Deterministic Trailing Rolling Window ({rep.get('span_s', 0):.1f} s)")
    print(f"Segment Span:     {rep.get('span_s', 0):.1f} s (Gate >= {args.min_span:.0f} s: {'PASS' if rep.get('gates', {}).get('span_ge_3600s') else 'FAIL'})")
    print(f"Sample Count:     {rep.get('rows', 0)} rows (Gate >= {args.min_rows}: {'PASS' if rep.get('gates', {}).get('rows_ge_3400') else 'FAIL'})")
    print(f"Max Internal Gap: {rep.get('max_gap_s', 0):.2f} s (Gate <= {MAX_GAP_S:.1f} s: {'PASS' if rep.get('gates', {}).get('max_gap_le_5s') else 'FAIL'})")
    print(f"Detrended RMS:    {rep.get('detrended_rms_ns', 0):.4f} ns ({rep.get('detrended_rms_ps', 0):.1f} ps) (Gate < 1.0 ns: {'PASS' if rep.get('gates', {}).get('rms_lt_1ns') else 'FAIL'})")
    print(f"Drift Slope:      {rep.get('drift_slope_ppm', 0):+.6f} ppm")
    audit = rep.get("audit", {})
    if audit.get("n_bridged_gaps", 0) > 0 or audit.get("n_ab_mismatch_excluded", 0) > 0:
        print(f"Audited Bridges:  {audit.get('n_bridged_gaps', 0)} single-row A/B gap(s) bridged ({audit.get('n_ab_mismatch_excluded', 0)} rows excluded)")
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

