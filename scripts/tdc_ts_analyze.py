#!/usr/bin/env python3
"""Analyze coarse timestamp-latch ratios without assigning either clock truth.

This is an offline ratio analyzer. It recomputes modulo-2^48 counter deltas
from consecutive latch values, rejects irregular host cadence from the
one-PPS interval estimate, and requires a strict config→events→stop v2 capture
whose per-event 48-bit latch is read twice identically. Its strongest verdict
is a capture-consistent relative ratio. It never converts sample_rate into
adclk and never declares the PPS or adclk absolutely correct.
"""

import argparse
import json
import math
import statistics
import sys
from collections import Counter
from typing import Any, Dict, List, Optional


COUNTER_MODULUS = 1 << 48
CAPTURE_SCHEMA = "tdc-ts-latch-capture-v2"
CAPTURE_PROTOCOL = "double-read-equal-v1"
REQUIRED_PROVENANCE = {
    "serial", "slot", "build_id", "clock_source", "clock_source_readback",
    "trigger_source", "adclk_hz", "counter_bits", "capture_protocol",
}
PPS_INTERVAL_TOL_FRAC = 0.02


def _json_int(value: Any) -> int:
    """Return an exact JSON integer; reject bools, strings, and floats."""
    if type(value) is not int:
        raise ValueError("expected JSON integer")
    return value


def _json_number(value: Any) -> float:
    if type(value) not in (int, float):
        raise ValueError("expected JSON number")
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("expected finite JSON number") from exc
    if not math.isfinite(value):
        raise ValueError("expected finite JSON number")
    return value


def _direct_adclk_hz(metadata: Dict[str, Any]) -> Optional[float]:
    try:
        value = _json_number(metadata["adclk_hz"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) and value > 0.0 else None


def analyze(path: str, adclk_hz: Optional[float] = None) -> Dict[str, Any]:
    events: List[Dict[str, Any]] = []
    configs: List[Dict[str, Any]] = []
    stop_rows: List[Dict[str, Any]] = []
    aborts = malformed = outside_segment = unknown_rows = duplicate_rows = 0
    incoherent_reads = missing_coherence = event_kind_errors = 0
    state = "before"
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                malformed += 1
                continue
            if not isinstance(row, dict):
                malformed += 1
                continue
            kind = row.get("kind")
            if kind == "config" or ("metadata" in row and state == "before"):
                configs.append(row)
                if state != "before":
                    outside_segment += 1
                else:
                    state = "capturing"
            elif kind == "stop":
                stop_rows.append(row)
                if state != "capturing":
                    outside_segment += 1
                state = "stopped"
            elif kind == "abort":
                aborts += 1
                state = "aborted"
            elif ("t" in row and
                  ("latch" in row or "latch_read_1" in row
                   or "latch_read_2" in row)):
                if state != "capturing":
                    outside_segment += 1
                    continue
                if row.get("dup") is True:
                    duplicate_rows += 1
                    continue
                if kind != "event":
                    event_kind_errors += 1
                try:
                    t = _json_number(row["t"])
                    if "latch_read_1" in row and "latch_read_2" in row:
                        latch_1 = _json_int(row["latch_read_1"])
                        latch_2 = _json_int(row["latch_read_2"])
                        if latch_1 != latch_2:
                            incoherent_reads += 1
                        latch = latch_1
                        coherent = latch_1 == latch_2
                    else:
                        latch = _json_int(row["latch"])
                        coherent = False
                        missing_coherence += 1
                    seq = _json_int(row["seq"])
                except (KeyError, TypeError, ValueError):
                    malformed += 1
                    continue
                if (not math.isfinite(t) or not (0 <= latch < COUNTER_MODULUS)
                        or seq < 0):
                    malformed += 1
                    continue
                if "latch" in row:
                    try:
                        if _json_int(row["latch"]) != latch:
                            incoherent_reads += 1
                            coherent = False
                    except (TypeError, ValueError):
                        malformed += 1
                        continue
                events.append({"line": lineno, "seq": seq, "t": t,
                               "latch": latch, "coherent": coherent,
                               "stored_delta": row.get("delta")})
            else:
                unknown_rows += 1

    metadata = configs[0] if len(configs) == 1 else {}
    meta_hz = _direct_adclk_hz(metadata)
    metadata_clock_valid = meta_hz is not None
    if adclk_hz is not None:
        try:
            adclk_hz = float(adclk_hz)
        except (TypeError, ValueError, OverflowError):
            return {"error": "--adclk-hz must be finite and > 0"}
        if not math.isfinite(adclk_hz) or adclk_hz <= 0.0:
            return {"error": "--adclk-hz must be finite and > 0"}
        if meta_hz is not None and abs(meta_hz - adclk_hz) > 1e-6 * meta_hz:
            return {"error": (
                f"adclk mismatch: metadata {meta_hz:g} Hz vs CLI {adclk_hz:g} Hz"
            )}
        nominal_hz = adclk_hz
        clock_source = ("--adclk-hz" if metadata_clock_valid
                        else "--adclk-hz (exploratory fallback)")
    elif meta_hz is not None:
        nominal_hz, clock_source = meta_hz, "run metadata"
    else:
        return {"error": (
            "adclk unknown: provide direct adclk_hz metadata or --adclk-hz; "
            "sample_rate*2 inference is forbidden"
        )}

    if len(events) < 2:
        return {"error": f"only {len(events)} latch event(s)"}

    intervals = []
    delta_mismatches = 0
    delta_missing = 0
    nonmonotonic_host = 0
    for previous, current in zip(events, events[1:]):
        host_dt = current["t"] - previous["t"]
        if host_dt <= 0.0:
            nonmonotonic_host += 1
        delta = (current["latch"] - previous["latch"]) % COUNTER_MODULUS
        stored = current.get("stored_delta")
        if stored is None:
            delta_missing += 1
        else:
            try:
                if _json_int(stored) % COUNTER_MODULUS != delta:
                    delta_mismatches += 1
            except (TypeError, ValueError):
                delta_mismatches += 1
        intervals.append({"host_dt": host_dt, "ticks": delta})

    lo = 1.0 - PPS_INTERVAL_TOL_FRAC
    hi = 1.0 + PPS_INTERVAL_TOL_FRAC
    valid = [x for x in intervals
             if lo <= x["host_dt"] <= hi
             and lo * nominal_hz <= x["ticks"] <= hi * nominal_hz]
    irregular = len(intervals) - len(valid)
    gate_fails = []
    if metadata.get("schema") != CAPTURE_SCHEMA:
        gate_fails.append(
            f"capture schema must be {CAPTURE_SCHEMA!r}"
        )
    if metadata.get("capture_protocol") != CAPTURE_PROTOCOL:
        gate_fails.append(
            f"capture_protocol must be {CAPTURE_PROTOCOL!r}"
        )
    if len(configs) != 1:
        gate_fails.append(f"expected exactly one config/metadata row, found {len(configs)}")
    if len(stop_rows) != 1:
        gate_fails.append(f"expected exactly one stop row, found {len(stop_rows)}")
    if aborts:
        gate_fails.append(f"capture contains {aborts} abort row(s)")
    if malformed:
        gate_fails.append(f"capture contains {malformed} malformed row(s)")
    if unknown_rows:
        gate_fails.append(f"capture contains {unknown_rows} unknown row(s)")
    if outside_segment:
        gate_fails.append(
            f"capture contains {outside_segment} control/event row(s) outside one config→stop segment"
        )
    if duplicate_rows:
        gate_fails.append(f"capture contains {duplicate_rows} duplicate-tagged event(s)")
    if event_kind_errors:
        gate_fails.append(f"capture contains {event_kind_errors} event row(s) without kind='event'")
    if incoherent_reads:
        gate_fails.append(f"capture contains {incoherent_reads} unequal/inconsistent double read(s)")
    if missing_coherence:
        gate_fails.append(f"capture contains {missing_coherence} event(s) without two equal latch reads")
    if not metadata_clock_valid:
        gate_fails.append(
            "capture metadata must contain a finite positive direct adclk_hz; "
            "a CLI clock is exploratory only"
        )
    seqs = [event["seq"] for event in events]
    if seqs != list(range(len(events))):
        gate_fails.append("event sequence must be contiguous 0..n-1 in file order")
    if len(stop_rows) == 1:
        stop = stop_rows[0]
        try:
            if _json_int(stop["n"]) != len(events):
                gate_fails.append(
                    f"stop count {stop.get('n')!r} != accepted events {len(events)}"
                )
            if _json_int(stop["last_seq"]) != events[-1]["seq"]:
                gate_fails.append("stop last_seq does not match final event")
        except (KeyError, TypeError, ValueError):
            gate_fails.append("stop row requires integer n and last_seq")
    if nonmonotonic_host:
        gate_fails.append(f"capture contains {nonmonotonic_host} nonmonotonic host interval(s)")
    if delta_mismatches:
        gate_fails.append(f"capture contains {delta_mismatches} stored/recomputed delta mismatch(es)")
    if delta_missing:
        gate_fails.append(f"capture contains {delta_missing} interval event(s) without stored delta")
    if irregular:
        gate_fails.append(f"capture contains {irregular} irregular/missed interval(s)")
    missing = sorted(k for k in REQUIRED_PROVENANCE
                     if k not in metadata or metadata[k] in (None, ""))
    if missing:
        gate_fails.append("metadata missing provenance fields: " + ", ".join(missing))
    try:
        if _json_int(metadata.get("slot")) != 1:
            gate_fails.append("current trigger-timestamp capture requires slot 1")
        if _json_int(metadata.get("counter_bits")) != 48:
            gate_fails.append("counter_bits must be 48")
    except (TypeError, ValueError):
        gate_fails.append("slot and counter_bits must be integers")
    if len(valid) < 10:
        gate_fails.append(f"only {len(valid)} valid one-second interval(s)")

    result: Dict[str, Any] = {
        "path": path,
        "adclk_hz": nominal_hz,
        "adclk_source": clock_source,
        "metadata": metadata,
        "events": len(events),
        "intervals": len(intervals),
        "valid_intervals": len(valid),
        "irregular_intervals": irregular,
        "capture_schema": metadata.get("schema"),
        "capture_protocol": metadata.get("capture_protocol"),
        "outside_segment_rows": outside_segment,
        "incoherent_reads": incoherent_reads,
        "missing_coherent_reads": missing_coherence,
        "sequence": seqs,
        "stored_delta_mismatches": delta_mismatches,
        "stored_delta_missing": delta_missing,
        "gate_fails": gate_fails,
        "relative_ratio_valid": not gate_fails,
    }
    if not valid:
        return result

    ticks = [x["ticks"] for x in valid]
    mean = statistics.mean(ticks)
    median = statistics.median(ticks)
    sd = statistics.stdev(ticks) if len(ticks) > 1 else 0.0
    result.update({
        "mean_ticks_per_observed_pps": mean,
        "median_ticks_per_observed_pps": median,
        "sd_ticks": sd,
        "min_ticks": min(ticks),
        "max_ticks": max(ticks),
        "conditional_pps_interval_s_if_adclk_exact": mean / nominal_hz,
        "conditional_pps_rate_ppm_if_adclk_exact": (nominal_hz / mean - 1.0) * 1e6,
        "conditional_adclk_ppm_if_pps_exact": (mean / nominal_hz - 1.0) * 1e6,
    })

    xs = []
    elapsed = 0.0
    for item in valid:
        elapsed += item["host_dt"]
        xs.append(elapsed)
    if len(ticks) >= 3:
        mx, my = statistics.mean(xs), mean
        sxx = sum((x - mx) ** 2 for x in xs)
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ticks)) / sxx
        residuals = [y - (my + slope * (x - mx)) for x, y in zip(xs, ticks)]
        variance = sum(r * r for r in residuals) / (len(ticks) - 2)
        slope_se = math.sqrt(variance / sxx) if sxx else math.inf
        result["trend_ticks_per_interval_per_s"] = slope
        result["trend_standard_error"] = slope_se
        if slope_se:
            result["trend_z"] = slope / slope_se
        else:
            result["trend_z"] = 0.0 if slope == 0.0 else math.copysign(math.inf, slope)
    result["delta_histogram"] = sorted(Counter(ticks).items())
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path")
    parser.add_argument("--adclk-hz", type=float)
    args = parser.parse_args()
    try:
        result = analyze(args.path, args.adclk_hz)
    except FileNotFoundError:
        sys.exit(f"file not found: {args.path}")
    if "error" in result:
        sys.exit(result["error"])

    print(f"file: {args.path}")
    print(f"latch events: {result['events']} | intervals: {result['intervals']} | "
          f"valid 1 s cadence: {result['valid_intervals']} | "
          f"irregular: {result['irregular_intervals']}")
    if "mean_ticks_per_observed_pps" in result:
        print("delta ticks/accepted interval: "
              f"mean {result['mean_ticks_per_observed_pps']:.3f} "
              f"median {result['median_ticks_per_observed_pps']:.3f} "
              f"sd {result['sd_ticks']:.3f} "
              f"min {result['min_ticks']} max {result['max_ticks']}")
        print("conditional A (adclk exact): PPS interval "
              f"{result['conditional_pps_interval_s_if_adclk_exact']:.12f} s, "
              f"rate {result['conditional_pps_rate_ppm_if_adclk_exact']:+.6f} ppm")
        print("conditional B (PPS exact): adclk error "
              f"{result['conditional_adclk_ppm_if_pps_exact']:+.6f} ppm")
        if "trend_z" in result:
            print("linear trend: "
                  f"{result['trend_ticks_per_interval_per_s']:+.6g} "
                  f"ticks/interval/s ± {result['trend_standard_error']:.3g} "
                  f"(z={result['trend_z']:+.2f})")
    if result["gate_fails"]:
        print("VERDICT: EXPLORATORY ONLY")
        for reason in result["gate_fails"]:
            print(f"  - {reason}")
        sys.exit(1)
    print("VERDICT: VALID CAPTURE-CONSISTENT RELATIVE CLOCK RATIO")
    print("Neither clock is assigned absolute truth by this dataset alone.")


if __name__ == "__main__":
    main()
