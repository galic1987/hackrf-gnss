#!/usr/bin/env python3
"""Descriptive analysis of legacy external-trigger TDC thermometer captures.

This tool reports only what the retained rows establish. A strict-prefix row
does not prove that qualifier-deferred captures are absent, full-scale code 48
is composite, and an occupancy comb cannot uniquely separate FPGA DNL from a
nonuniform stimulus. No absolute tap width or jitter claim is made here.
"""

import json
import math
import statistics
import sys
from collections import Counter


NUM_TAPS = 48


def taps_of(hexbytes):
    if not isinstance(hexbytes, str):
        return None
    try:
        values = [int(b.strip(), 16) for b in hexbytes.split(",") if b.strip()]
    except ValueError:
        return None
    if len(values) != 6 or any(v < 0 or v > 255 for v in values):
        return None
    return [(value >> bit) & 1 for value in values for bit in range(8)]


def thermo_k(taps):
    """Return k for one exact strict prefix, including edge codes 0 and 48."""
    if taps is None or len(taps) != NUM_TAPS:
        return None
    k = 0
    while k < NUM_TAPS and taps[k] == 1:
        k += 1
    return k if all(t == 0 for t in taps[k:]) else None


def load(path):
    events = []
    stats = Counter()
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            if not line.lstrip().startswith("{"):
                stats["nonjson"] += 1
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                stats["malformed"] += 1
                continue
            if not isinstance(row, dict):
                stats["malformed"] += 1
                continue
            if row.get("dup") is True:
                stats["duplicate"] += 1
                continue
            if "bytes" not in row:
                stats[f"control:{row.get('kind', 'other')}"] += 1
                continue
            taps = taps_of(row.get("bytes"))
            try:
                t = float(row["t"])
            except (KeyError, TypeError, ValueError, OverflowError):
                stats["invalid"] += 1
                continue
            k = thermo_k(taps)
            try:
                status_toggle = int(str(row["reg31"]), 0) & 1
            except (KeyError, TypeError, ValueError):
                status_toggle = None
                stats["missing_or_bad_status"] += 1
            if not math.isfinite(t):
                stats["invalid"] += 1
                continue
            if k is None:
                stats["nonthermometer"] += 1
                events.append({"t": t, "k": None,
                               "popcount": sum(taps) if taps else None,
                               "status_toggle": status_toggle})
            else:
                events.append({"t": t, "k": k, "popcount": k,
                               "status_toggle": status_toggle})
    return events, stats


def segment_events(events):
    """Return strict-prefix runs with coherent, alternating status toggles.

    A missing or repeated toggle, invalid thermometer, or irregular cadence
    breaks the run and the boundary event is excluded. This prevents a
    missed/duplicate/torn read from contributing an apparent code step.
    """
    segments = []
    current = []
    previous_t = None
    previous_status = None
    for row in events:
        status = row["status_toggle"]
        cadence_ok = (previous_t is None
                      or 0.0 < row["t"] - previous_t <= 1.5)
        status_ok = (status is not None
                     and (previous_status is None
                          or status != previous_status))
        row_ok = row["k"] is not None and cadence_ok and status_ok
        if not row_ok:
            if current:
                segments.append(current)
            current = []
        elif row["k"] is not None:
            current.append(row)
        previous_t = row["t"]
        previous_status = status
    if current:
        segments.append(current)
    return segments


def main(path):
    events, stats = load(path)
    if len(events) < 10:
        sys.exit(f"only {len(events)} event rows")
    times = [row["t"] for row in events]
    dts = [b - a for a, b in zip(times, times[1:])]
    valid = [row for row in events if row["k"] is not None]
    invalid = [row for row in events if row["k"] is None]
    hist = Counter(row["k"] for row in valid)
    status_pairs = [(a["status_toggle"], b["status_toggle"])
                    for a, b in zip(events, events[1:])
                    if a["status_toggle"] is not None
                    and b["status_toggle"] is not None]
    repeated_status = sum(a == b for a, b in status_pairs)

    print(f"file: {path}")
    print(f"event rows: {len(events)} over {times[-1] - times[0]:.1f} s | "
          f"host cadence median {statistics.median(dts):.4f} s "
          f"(min {min(dts):.3f}, max {max(dts):.3f}) | "
          f"irregular >1.5 s: {sum(dt > 1.5 for dt in dts)}")
    print(f"strict-prefix rows: {len(valid)}/{len(events)} "
          f"({100 * len(valid) / len(events):.1f}%) | "
          f"visible non-thermometer rows: {len(invalid)}")
    if stats:
        print("skipped/control rows: " + ", ".join(
            f"{name}={count}" for name, count in sorted(stats.items())
        ))
    if invalid[:5]:
        print("first visible non-thermometer rows (t, popcount): "
              + str([(round(r["t"], 3), r["popcount"]) for r in invalid[:5]]))

    print("code occupancy (k:count):")
    print("  " + " ".join(f"{k}:{hist[k]}" for k in sorted(hist)))
    print("edge semantics: k=0 is not a qualified external-trigger capture; "
          "k=48 is composite overflow/full-chain plus possible deferred capture")
    print(f"status-toggle check: {repeated_status} repeated state(s) across "
          f"{len(status_pairs)} adjacent readable pairs")
    print("read-coherence limitation: reg31 was not sampled both before and "
          "after each six-byte word, so a strict prefix can still be torn; "
          "toggle alternation is only a missed/duplicate-event screen")

    # Segment at invalid rows, irregular cadence, missing status, or a
    # repeated status toggle. Differences across code 48 are not wrapped into
    # a precise fine-phase step.
    segments = segment_events(events)

    interior_steps = []
    interior_values = []
    for segment in segments:
        for a, b in zip(segment, segment[1:]):
            if 1 <= a["k"] < NUM_TAPS and 1 <= b["k"] < NUM_TAPS:
                interior_steps.append(b["k"] - a["k"])
                interior_values.extend((a["k"], b["k"]))
    if interior_steps:
        abs_steps = sorted(abs(v) for v in interior_steps)
        p95 = abs_steps[min(len(abs_steps) - 1, int(0.95 * len(abs_steps)))]
        print("consecutive interior-code change (descriptive, not jitter): "
              f"median {statistics.median(interior_steps):+.1f}, "
              f"mean {statistics.mean(interior_steps):+.2f}, "
              f"p95 |change| {p95:.0f} codes")

    nonzero = [step for step in interior_steps if step]
    if len(nonzero) > 30:
        step_hist = Counter(nonzero)
        top = step_hist.most_common(8)
        multiples = sum(count for step, count in step_hist.items()
                        if step % 16 == 0)
        print(f"interior step histogram (top): {top}")
        if multiples > 0.15 * len(nonzero):
            dominant = Counter(interior_values).most_common(5)
            print(
                f"SEGMENT-BOUNDARY PATTERN: {multiples}/{len(nonzero)} "
                f"nonzero interior changes are multiples of 16; dominant "
                f"codes {dominant}. This is consistent with fabric-hop DNL, "
                "but source-phase visitation and stimulus contribution are "
                "unresolved by this capture."
            )

    print("VERDICT: DESCRIPTIVE/EXPLORATORY — no absolute scale or source attribution")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         "/Volumes/Radiator 8TB/gnss/observations/tdc_pps_run1.jsonl")
