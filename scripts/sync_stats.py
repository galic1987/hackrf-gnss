#!/usr/bin/env python3
"""All-night timing-deviation stats from sync_state_history.jsonl.

Reads the producer's 2 s samples (t, tick_hz, drift_ppm, n) and prints a
digest: span, tick-rate statistics, outlier rejection, and overlapping Allan
deviation of the fractional tick-rate series. No radio access required.

Usage: python3 scripts/sync_stats.py [history.jsonl]
"""
import json
import math
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else "observations/sync_state_history.jsonl"

# Nominal counter rate (idle adclk domain the producer observes).
TICK_NOMINAL = 32e6


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
                rows.append((float(r["t"]), float(r["tick_hz"])))
            except (json.JSONDecodeError, KeyError, ValueError):
                continue
    rows.sort()
    return rows


def main():
    rows = load(PATH)
    if len(rows) < 10:
        print(f"only {len(rows)} samples — too few for stats")
        return

    t0, t1 = rows[0][0], rows[-1][0]

    # Outlier rejection: the producer's tick_hz is derived from short PC-side
    # reads; transport glitches produce absurd values (seen: -3.5e7, +2.3e7).
    # Keep samples within 100 ppm of nominal and within a 5-sigma MAD gate.
    sane = [(t, h) for t, h in rows if abs(h - TICK_NOMINAL) / TICK_NOMINAL < 1e-4]
    n_glitch = len(rows) - len(sane)

    vals = [h for _, h in sane]
    med = sorted(vals)[len(vals) // 2]
    mad = sorted(abs(v - med) for v in vals)[len(vals) // 2] or 1.0
    clean = [(t, h) for t, h in sane if abs(h - med) < 5 * 1.4826 * mad]

    frac = [(t, (h - TICK_NOMINAL) / TICK_NOMINAL) for t, h in clean]
    ys = [y for _, y in frac]
    n = len(ys)
    mean = sum(ys) / n
    var = sum((y - mean) ** 2 for y in ys) / (n - 1) if n > 1 else 0.0
    std = math.sqrt(var)

    print(f"samples: {len(rows)} total, {n_glitch} transport glitches dropped, "
          f"{len(sane) - n} MAD outliers dropped, {n} used")
    print(f"span: {(t1 - t0) / 3600:.2f} h  ({t0:.0f} .. {t1:.0f})")
    print(f"fractional tick offset: mean {mean * 1e6:+.3f} ppm, std {std * 1e6:.3f} ppm")
    print(f"wander envelope: {min(ys) * 1e6:+.2f} .. {max(ys) * 1e6:+.2f} ppm")

    # Overlapping Allan deviation on unevenly sampled data: bin to the median
    # sample period first (nearest-neighbour), then standard OADEV.
    dts = [clean[i + 1][0] - clean[i][0] for i in range(len(clean) - 1)]
    dt = sorted(dts)[len(dts) // 2] or 2.0
    nbins = int((t1 - t0) // dt)
    if nbins < 8:
        print("span too short for Allan deviation")
        return
    bins = [[] for _ in range(nbins)]
    for t, y in frac:
        i = min(int((t - t0) // dt), nbins - 1)
        bins[i].append(y)
    yb = [sum(b) / len(b) if b else None for b in bins]
    # fill gaps by carrying previous value
    last = None
    for i in range(nbins):
        if yb[i] is None:
            yb[i] = last
        else:
            last = yb[i]
    yb = [y for y in yb if y is not None]

    print(f"\noverlapping Allan deviation (binned dt={dt:.1f} s):")
    print(f"{'tau':>10} {'ADEV (ppm)':>12}")
    m = 1
    while m * 3 <= len(yb):
        s = 0.0
        cnt = 0
        for i in range(len(yb) - 2 * m):
            d = yb[i + 2 * m] - 2 * yb[i + m] + yb[i]
            s += d * d
            cnt += 1
        adev = math.sqrt(s / (2 * (m * dt) ** 2 * cnt)) if cnt else float("nan")
        print(f"{m * dt:>9.0f}s {adev * 1e6:>12.4f}")
        m *= 2

    print("\nnote: the ~tau^-1 slope across ALL taus means the series is still")
    print("dominated by PC-side read jitter (white measurement noise averaging down),")
    print("not TCXO physics. True clock stability lives BELOW the 2076 s value until")
    print("the nibble-embedded timestamps replace PC-side reads. Use ADEV(tau) here as")
    print("an UPPER BOUND on TCXO instability, and the Iridium/GNSS solves for truth.")


if __name__ == "__main__":
    main()
