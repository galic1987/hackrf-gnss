#!/usr/bin/env python3
"""tdc_ts_analyze.py — analyze a ticks-per-PPS-interval latch capture
(ts_latch_run*.jsonl, produced by tdc_ts_latch_run.sh on slot 1).

Each row is one PPS edge: the 48-bit trigger latch value and its delta
against the previous edge. The delta counts adclk (AFE 40 MHz domain)
ticks per PPS interval, so the series is a RATIO of two clocks. With the
tracker clock-drift establishing adclk GPS-true (2026-08-29 window), the
ratio measures the PPS interval itself; both readings are printed so the
tie-break stays explicit. Read-only.
"""
import json, sys, statistics

NOMINAL = 40_000_000  # adclk (AFE clock) nominal ticks per second

def main(path):
    ts, ds = [], []
    metadata = {}
    for line in open(path):
        line = line.strip()
        if not line.startswith("{"):
            continue
        r = json.loads(line)
        if "metadata" in r:
            metadata = r
            continue
        ts.append(r["t"])
        ds.append(r["delta"])
    n = len(ds)
    if n < 10:
        sys.exit(f"only {n} rows")
    
    if "sample_rate" in metadata:
        NOMINAL = metadata["sample_rate"] * 2
    else:
        NOMINAL = 40_000_000
    
    span = ts[-1] - ts[0]
    dts = [ts[i + 1] - ts[i] for i in range(n - 1)]
    missed = sum(1 for d in dts if d > 1.5)
    mean = statistics.mean(ds)
    print(f"file: {path}")
    print(f"seconds: {n} over {span:.0f} s | missed: {missed}")
    print(f"delta ticks/interval: mean {mean:.2f} median {statistics.median(ds)} "
          f"sd {statistics.stdev(ds):.2f} min {min(ds)} max {max(ds)}")
    print(f"reading A (adclk exact): PPS interval {mean/NOMINAL:.9f} s "
          f"-> PPS rate {(NOMINAL/mean - 1) * 1e6:+.3f} ppm vs 1 Hz")
    print(f"reading B (PPS exact):   adclk rate {(mean/NOMINAL - 1) * 1e6:+.3f} ppm "
          f"vs {NOMINAL/1e6:.3f} MHz  (tie-break: tracker clock-drift, see report)")
    # trend of delta over the run
    t0 = ts[0]
    xs = [t - t0 for t in ts]
    mx, my = statistics.mean(xs), mean
    sxx = sum((x - mx) ** 2 for x in xs)
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ds)) / sxx
    print(f"trend: {slope:+.4f} ticks/s per s (static if |trend| << sd/sqrt(span))")
    if n >= 200:
        f100, l100 = statistics.mean(ds[:100]), statistics.mean(ds[-100:])
        print(f"first-100s mean {f100:.1f} | last-100s mean {l100:.1f} | "
              f"change {l100 - f100:+.1f} ticks/s")
    dd = [ds[i + 1] - ds[i] for i in range(n - 1)]
    tick_ns = 1e9 / NOMINAL
    print(f"second-to-second change: sd {statistics.stdev(dd):.2f} ticks "
          f"({statistics.stdev(dd) * tick_ns:.0f} ns @{tick_ns:.1f} ns/tick)")
    from collections import Counter
    c = Counter(ds)
    top = sorted(c.items())
    if len(top) <= 12:
        print(f"delta histogram: {top}")
        
    ppm_error = (mean/NOMINAL - 1) * 1e6
    if abs(ppm_error) > 100.0:
        print(f"VERDICT: FAIL (clock error {ppm_error:+.1f} ppm > 100 ppm limit)")
        sys.exit(1)
    print("VERDICT: PASS")

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1
         else "/Volumes/Radiator 8TB/gnss/observations/ts_latch_run1.jsonl")
