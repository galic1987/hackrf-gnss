#!/usr/bin/env python3
"""tdc_pps_analyze.py — analyze an external-1PPS TDC capture (tdc_pps_run*.jsonl).

Per event: 6 bytes (0x20-0x25) = 48 taps, byte i bit j = tap 8i+j (LSB
first, matching examples/tdc_cal.rs). A valid single-edge capture is a
THERMOMETER: taps 0..k-1 set, taps k..47 clear; popcount = k = how far the
edge propagated into the chain in one adclk period. Non-thermometer rows
are read races (the next PPS landed mid-read, ~0.9 s read budget) or
bubbles — counted and excluded from the histogram, never silently dropped.

Outputs: event rate/regularity, thermometer-validity rate, 48-bin tap
histogram (code density -> per-tap relative widths), phase-drift between
successive popcounts (the PPS-vs-adclk phase observable), and jitter of
popcount within slow-phase stretches. Read-only.
"""
import json, math, sys, statistics

def taps_of(hexbytes):
    bs = [int(b, 16) for b in hexbytes.split(",")]
    return [(b >> j) & 1 for b in bs for j in range(8)]

def thermo_k(taps):
    """k if taps are a clean thermometer (1..k set, rest clear), else None."""
    k = 0
    while k < 48 and taps[k] == 1:
        k += 1
    if all(t == 0 for t in taps[k:]):
        return k
    return None

def main(path):
    rows = []
    for line in open(path):
        line = line.strip()
        if not line.startswith("{"):
            continue
        r = json.loads(line)
        rows.append((r["t"], taps_of(r["bytes"])))
    n = len(rows)
    if n < 10:
        sys.exit(f"only {n} events")
    ts = [t for t, _ in rows]
    dts = [ts[i + 1] - ts[i] for i in range(n - 1)]
    valid = []
    races = []
    for t, taps in rows:
        k = thermo_k(taps)
        (valid if k is not None else races).append((t, k if k is not None else sum(taps)))
    ks = [k for _, k in valid]
    hist = [0] * 49
    for k in ks:
        hist[k] += 1
    print(f"file: {path}")
    print(f"events: {n} over {ts[-1]-ts[0]:.0f} s | rate {statistics.mean(dts):.4f} s "
          f"(min {min(dts):.3f} max {max(dts):.3f}) | missed: {sum(1 for d in dts if d > 1.5)}")
    print(f"thermometer-valid: {len(valid)}/{n} ({100*len(valid)/n:.1f}%) | "
          f"races/bubbles: {len(races)}")
    if races[:5]:
        print("  first race rows (t, popcount):", [(round(t, 2), k) for t, k in races[:5]])
    print("tap histogram (k: count):")
    nz = [(k, c) for k, c in enumerate(hist) if c]
    print("  " + " ".join(f"{k}:{c}" for k, c in nz))
    if len(ks) > 20:
        # phase drift: successive k deltas wrapped into [-24, 24) taps
        dk = []
        for i in range(1, len(ks)):
            d = ks[i] - ks[i - 1]
            d = ((d + 24) % 48) - 24
            dk.append(d)
        print(f"phase step per pulse (taps): median {statistics.median(dk):+.1f} "
              f"mean {statistics.mean(dk):+.2f} p95 |step| {sorted(abs(d) for d in dk)[int(len(dk)*.95)]:.0f}")
        # jitter: k scatter within stretches where the phase step is small
        slow = [ks[i] for i in range(1, len(ks)) if abs(ks[i] - ks[i - 1]) <= 1]
        if len(slow) > 10:
            print(f"slow-phase stretches: {len(slow)} samples, popcount scatter "
                  f"sd {statistics.pstdev(slow):.2f} taps (chain quantisation dominates)")
        # dwell/burst structure: k=48 = saturated (edge outside the ~1.3 ns
        # measurement window), k<48 = in-window. A GPSDO-locked PPS whose
        # phase slowly slews against adclk alternates between the two.
        inwin = [k < 48 for k in ks]
        runs = []
        cur, start = inwin[0], 0
        for i in range(1, len(ks)):
            if inwin[i] != cur:
                runs.append((cur, start, i - 1))
                cur, start = inwin[i], i
        runs.append((cur, start, len(ks) - 1))
        bursts = [r for r in runs if r[0]]
        dwells = [r for r in runs if not r[0]]

        if bursts and dwells:
            bl = [r[2] - r[1] + 1 for r in bursts]
            dl = [r[2] - r[1] + 1 for r in dwells]
            bstarts = [ts[r[1]] for r in bursts]
            period = ([bstarts[i + 1] - bstarts[i] for i in range(len(bstarts) - 1)]
                      if len(bstarts) > 2 else [])
            line = (f"phase cycles: {len(bursts)} in-window bursts "
                    f"(median {statistics.median(bl):.0f} s) / {len(dwells)} dwells "
                    f"(median {statistics.median(dl):.0f} s)")
            if period:
                line += f" | burst period median {statistics.median(period):.1f} s"
            print(line)
        # comb test: check for 16-tap FPGA logic routing DNL artifacts
        steps = []
        for r in bursts:
            seg = ks[r[1]:r[2] + 1]
            steps += [seg[i + 1] - seg[i] for i in range(len(seg) - 1)]
        nz = [s for s in steps if s]
        if len(nz) > 30:
            from collections import Counter
            sc = Counter(nz)
            top = sorted(sc.items(), key=lambda x: -x[1])[:8]
            print(f"intra-burst step histogram (top): {top}")
            mult = sum(c for s, c in sc.items() if s % 16 == 0)
            if mult > 0.15 * len(nz):
                iw_cnt = Counter(k for k in ks if k < 48)
                dom = sorted(iw_cnt.items(), key=lambda x: -x[1])[:5]
                print(f"DNL ARTIFACT: {mult}/{len(nz)} intra-burst steps are multiples of "
                      f"16 taps; dominant in-window bins {dom}. This is a TDC Differential "
                      f"Non-Linearity (DNL) artifact from FPGA fabric hop boundaries, NOT a "
                      f"Bodnar property.")

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1
         else "/Volumes/Radiator 8TB/gnss/observations/tdc_pps_run1.jsonl")
