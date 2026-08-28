#!/usr/bin/env python3
"""Sub-ns claim analyzer (Leg 1): detrend clock_bias.jsonl, RMS + OADEV, gates.

Usage: python3 scripts/clock_bias_analyzer.py [path] [--min-rows 3600]
Gates (spec 2026-08-27): RMS < 1 ns and OADEV < 1 ns for tau in 10..1000 s.
Exit 0 = claim supported, 1 = not (or insufficient data)."""
import json, math, sys

GATES_TAU = [10, 100, 1000]

def detrend(epochs, clock_ns):
    """Least-squares linear detrend (Bodnar GPS-steering is common-mode)."""
    n = len(epochs)
    t0 = sum(epochs) / n
    b0 = sum(clock_ns) / n
    s_tt = sum((t - t0) ** 2 for t in epochs)
    s_tb = sum((t - t0) * (b - b0) for t, b in zip(epochs, clock_ns))
    slope = s_tb / s_tt if s_tt else 0.0
    return [b - (b0 + slope * (t - t0)) for t, b in zip(epochs, clock_ns)]

def oadev(residuals, dt_s, taus):
    """Time deviation TDEV(tau) = tau*ADEV/sqrt(3), in the input's units (ns).

    x_k is the time-error series; ADEV = sqrt(mean((x[k+2m]-2x[k+m]+x[k])^2)/2)/tau
    (dimensionless); TDEV puts it back in ns. White phase noise sigma -> TDEV = sigma
    at all tau (flat), which is what the synthetic test pins."""
    out = {}
    n = len(residuals)
    for tau in taus:
        m = max(1, int(round(tau / dt_s)))
        vals = []
        for k in range(n - 2 * m):
            v = (residuals[k + 2 * m] - 2 * residuals[k + m] + residuals[k])
            vals.append(v * v)
        if vals:
            adev = math.sqrt(sum(vals) / (2 * len(vals))) / tau
            out[tau] = adev * tau / math.sqrt(3.0)
    return out

def verdict(rms_ns, adev_tbl):
    return rms_ns < 1.0 and all(v < 1.0 for v in adev_tbl.values())

def main():
    argv = sys.argv[1:]
    min_rows = 3600
    if "--min-rows" in argv:
        i = argv.index("--min-rows")
        min_rows = int(argv[i + 1])
        del argv[i:i + 2]
    path = argv[0] if argv and not argv[0].startswith("-") else \
        "/Volumes/Radiator 8TB/gnss/observations/clock_bias.jsonl"
    try:
        rows = [json.loads(l) for l in open(path) if l.strip()]
    except FileNotFoundError:
        rows = []
    rows = [r for r in rows if r.get("n_sat", 0) >= 5 and r.get("slips", 1) == 0]
    ep = [r["epoch"] for r in rows]
    ck = [r["clock_ns"] for r in rows]
    print(f"rows {len(rows)} (slip-free, n_sat>=5)")
    if len(rows) < min_rows:
        print(f"INSUFFICIENT DATA (<{min_rows} rows)"); sys.exit(1)
    res = detrend(ep, ck)
    rms = math.sqrt(sum(r * r for r in res) / len(res))
    dt = (ep[-1] - ep[0]) / max(1, len(ep) - 1)
    tbl = oadev(res, dt, GATES_TAU)
    print(f"detrended RMS: {rms:.3f} ns   (dt {dt:.2f} s)")
    for t in GATES_TAU:
        print(f"  OADEV(tau={t:>4}s): {tbl.get(t, float('nan')):.3f} ns")
    ok = verdict(rms, tbl)
    print("VERDICT:", "SUB-NS CLAIM SUPPORTED" if ok else "claim not supported")
    # paired elevation A/B summary (same epochs, both solves)
    d = [r["residual_rms_m"] - r["residual_rms_m_uw"] for r in rows
         if "residual_rms_m_uw" in r]
    if d:
        print(f"paired A/B: median (weighted-raw RMS) = {sorted(d)[len(d)//2]:+.2f} m "
              f"over {len(d)} epochs (negative = weighting helps)")
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
