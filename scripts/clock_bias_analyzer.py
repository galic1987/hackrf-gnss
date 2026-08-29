#!/usr/bin/env python3
"""Sub-ns claim analyzer (Leg 1 v2): detrend clock_bias.jsonl, RMS + TDEV, gates.

Usage: python3 scripts/clock_bias_analyzer.py [path] [--min-rows 3400] [--all-gens]
Gates (spec 2026-08-27): RMS < 1 ns and TDEV < 1 ns for tau in 10..1000 s.
Exit 0 = claim supported, 1 = not (or insufficient data).

v2 (2026-08-28 amendment) fixes v1 defects:
- TDEV is now the textbook MODIFIED-Allan-based statistic (NIST SP 1065):
    ModAllanVar(m) = (1 / (2*tau^2*m^2*(n-3m+1)))
                     * sum_{j=0}^{n-3m} [ sum_{i=j}^{j+m-1}
                       (x_{i+2m} - 2*x_{i+m} + x_i) ]^2,   tau = m*dt
    TDEV(tau) = tau * sqrt(ModAllanVar) / sqrt(3)
  v1 printed tau*ADEV/sqrt(3) from ordinary second differences — a different
  statistic (flat for white PM; proper TDEV goes as tau^-1/2).
- Gap segmentation: the row series is split wherever an inter-row gap exceeds
  1.5x the median dt (tightened 5x -> 2x -> 1.5x, 2026-08-29 e7f0e22: every
  missed epoch now splits, so within-segment cadence is uniform to sub-epoch
  jitter; see docs/superpowers/evidence/leg1-gap-gate-reachability.md for
  the measured reachability arithmetic of this rule); only the LONGEST
  segment is analyzed and every hole is
  reported (v1 substituted the average spacing and printed gaps but never
  rejected or segmented them).
- Continuity gates replace the bare row count: segment span >= 3600 s AND
  rows >= 3400 AND max internal gap <= 5 s. --min-rows overrides the rows
  floor only; the span and gap gates still apply.
- gen session grouping: rows carry a per-start `gen` id. Default analyzes the
  LATEST gen only (restarts can't silently mix); --all-gens pools for
  forensics (pooled TDEV across sessions is not claim-grade).
"""
import json, math, sys

GATES_TAU = [10, 100, 1000]
POISON_RMS_M = 500.0     # residual_rms_m >= this is a poison-class row.
                         # Round-18, re-derived from the data's own
                         # distribution (20.8k v2 rows, 2026-08-29): quality
                         # rows (n_sat>=5, slips==0) have residual p50 148 /
                         # p95 300 / p99 410 m; only 0.02% exceed 500 m and
                         # NONE exceed 1000 m. The original flat 100 m sat
                         # BELOW THE 25TH PERCENTILE of healthy data (107 m)
                         # and cut the best clean hour 3214 -> 585 rows
                         # (18.2%) — it was the binding constraint on the
                         # overnight verdict, misdiagnosed as sky density.
                         # 500 m keeps healthy hours whole while still
                         # killing the km-class poison the gate exists for.
                         # The real fix is the measurement model (no iono /
                         # tropo / SBAS corrections in the clock_bias solve
                         # yet) — see the verdict doc's corrected diagnosis.
MIN_SPAN_S = 3600.0      # "continuous hour" span gate
MIN_ROWS = 3400          # rows floor (--min-rows overrides this one only)
MAX_GAP_S = 5.0          # max internal gap gate
GAP_FACTOR = 1.5         # segmentation split: gap > GAP_FACTOR * median dt.
                         # tdev() below assumes UNIFORM dt = median; at the
                         # 1.04 s live cadence this splits on EVERY missed
                         # epoch (dt >= ~1.6 s), so within a segment the
                         # cadence is uniform to sub-epoch jitter and the
                         # index-based TDEV spacing is physically right
                         # (round-14 review: the 2x rule still let one
                         # missed epoch through with the wrong spacing).
DEFAULT_PATH = "/Volumes/Radiator 8TB/gnss/observations/clock_bias.jsonl"


def detrend(epochs, clock_ns):
    """Least-squares linear detrend (Bodnar GPS-steering is common-mode).

    Linear detrend removes first-order clock steering AND conceals
    instability below ~1/span — LF structure is the GEO cross-check's job
    (spec component 4)."""
    n = len(epochs)
    t0 = sum(epochs) / n
    b0 = sum(clock_ns) / n
    s_tt = sum((t - t0) ** 2 for t in epochs)
    s_tb = sum((t - t0) * (b - b0) for t, b in zip(epochs, clock_ns))
    slope = s_tb / s_tt if s_tt else 0.0
    return [b - (b0 + slope * (t - t0)) for t, b in zip(epochs, clock_ns)]


def tdev(residuals_ns, dt_s, taus):
    """Time deviation TDEV(tau) of a uniform time-error series, NIST SP 1065.

    x_k = residuals_ns (ns), uniform spacing dt_s (the median interval;
    segmentation has already split at every gap > 1.5x median — i.e. on
    every missed epoch — so within a segment the cadence is uniform to
    sub-epoch jitter), tau = m*dt_s:
        ModAllanVar(m) = (1 / (2*tau^2*m^2*(n-3m+1)))
                         * sum_j [ sum_{i=j}^{j+m-1}
                           (x_{i+2m} - 2*x_{i+m} + x_i) ]^2
        TDEV(tau) = tau * sqrt(ModAllanVar) / sqrt(3)        (ns)
    White PM with time-error sigma gives E[TDEV] = sigma/sqrt(m)
    (= sigma*sqrt(dt/tau), slope -1/2 on log-log) — the synthetic test pins
    both the slope and the exact sigma recovery. taus whose span is
    insufficient (n < 3m+1) are omitted from the result.

    The inner sums are a sliding window over the second-difference series,
    so the evaluation is O(n) per tau — same value as the nested-loop
    textbook form."""
    out = {}
    n = len(residuals_ns)
    for tau in taus:
        m = max(1, int(round(tau / dt_s)))
        if n < 3 * m + 1:
            continue  # insufficient span for this tau
        z = [residuals_ns[i + 2 * m] - 2.0 * residuals_ns[i + m] + residuals_ns[i]
             for i in range(n - 2 * m)]
        s = sum(z[:m])          # S_0 = sum of z[0..m-1]
        ss = s * s
        for j in range(1, n - 3 * m + 1):
            s += z[j + m - 1] - z[j - 1]   # slide the length-m window
            ss += s * s
        terms = n - 3 * m + 1
        tau_eff = m * dt_s
        mvar = ss / (2.0 * tau_eff ** 2 * m ** 2 * terms)
        out[tau] = tau_eff * math.sqrt(mvar) / math.sqrt(3.0)
    return out


def verdict(rms_ns, tdev_tbl):
    return rms_ns < 1.0 and all(v < 1.0 for v in tdev_tbl.values())


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return 0.0
    mid = n // 2
    return s[mid] if n % 2 else 0.5 * (s[mid - 1] + s[mid])


def split_segments(epochs, gap_factor=GAP_FACTOR):
    """Split (sorted) epochs at inter-row gaps > gap_factor x median dt.

    Returns (segments, holes, dt_median): segments as inclusive index pairs
    (i0, i1); holes as {"at_epoch", "gap_s"} records, one per split."""
    n = len(epochs)
    if n == 0:
        return [], [], 0.0
    if n == 1:
        return [(0, 0)], [], 0.0
    gaps = [epochs[i + 1] - epochs[i] for i in range(n - 1)]
    dt_med = _median([g for g in gaps if g > 0])
    thresh = gap_factor * dt_med if dt_med > 0 else float("inf")
    segs, holes = [], []
    start = 0
    for i, g in enumerate(gaps):
        if g > thresh:
            segs.append((start, i))
            holes.append({"at_epoch": epochs[i], "gap_s": g})
            start = i + 1
    segs.append((start, n - 1))
    return segs, holes, dt_med


def _seg_stats(epochs, i0, i1):
    n = i1 - i0 + 1
    span = epochs[i1] - epochs[i0] if n > 1 else 0.0
    mg = max((epochs[i + 1] - epochs[i] for i in range(i0, i1)), default=0.0)
    return {"rows": n, "span_s": span, "max_gap_s": mg,
            "t0": epochs[i0], "t1": epochs[i1]}


def continuity_gates(span_s, n_rows, max_gap_s, min_rows=MIN_ROWS):
    """The 'continuous hour' gates: span AND rows AND max-gap (not rows alone)."""
    fails = []
    if span_s < MIN_SPAN_S:
        fails.append(f"span {span_s:.0f} s < {MIN_SPAN_S:.0f} s")
    if n_rows < min_rows:
        fails.append(f"rows {n_rows} < {min_rows}")
    if max_gap_s > MAX_GAP_S:
        fails.append(f"max internal gap {max_gap_s:.1f} s > {MAX_GAP_S:.0f} s")
    return fails


def analyze(rows, min_rows=MIN_ROWS, all_gens=False):
    """Full v2 pipeline on parsed jsonl rows -> report dict (main() prints it).

    Quality gate (n_sat>=5, slips==0) -> poison-class exclusion -> gen
    selection -> gap segmentation -> longest segment -> continuity gates ->
    detrend/RMS/TDEV/verdict + paired A/B. On any data shortfall the report
    carries gate_fails and verdict None (main prints INSUFFICIENT DATA)."""
    rep = {"all_gens": all_gens, "min_rows": min_rows,
           "n_in": len(rows), "gens": {}, "gen": None, "n_gen": 0,
           "segments": [], "holes": [], "dt_median": 0.0,
           "chosen_seg": None, "gate_fails": [], "verdict": None}
    rows = [r for r in rows if r.get("n_sat", 0) >= 5 and r.get("slips", 1) == 0]
    rep["n_quality"] = len(rows)
    sane = [r for r in rows if r.get("residual_rms_m", 1e9) < POISON_RMS_M]
    rep["n_poison"] = len(rows) - len(sane)
    rows = sane
    for r in rows:
        g = r.get("gen")
        rep["gens"][g] = rep["gens"].get(g, 0) + 1
    if all_gens:
        sel = rows
    else:  # latest gen by its most recent epoch
        last = {}
        for r in rows:
            g, e = r.get("gen"), r.get("epoch", 0.0)
            if g not in last or e > last[g]:
                last[g] = e
        rep["gen"] = max(last, key=lambda g: last[g]) if last else None
        sel = [r for r in rows if r.get("gen") == rep["gen"]]
    rep["n_gen"] = len(sel)
    sel = sorted(sel, key=lambda r: r["epoch"])
    ep = [r["epoch"] for r in sel]
    segs, holes, dt_med = split_segments(ep)
    rep["holes"] = holes
    rep["dt_median"] = dt_med
    rep["segments"] = [_seg_stats(ep, i0, i1) for i0, i1 in segs]
    if not segs:
        rep["gate_fails"] = ["no rows after filtering"]
        return rep
    chosen = max(range(len(segs)),
                 key=lambda i: (rep["segments"][i]["span_s"],
                                rep["segments"][i]["rows"]))
    rep["chosen_seg"] = chosen
    i0, i1 = segs[chosen]
    st = rep["segments"][chosen]
    rep["gate_fails"] = continuity_gates(st["span_s"], st["rows"],
                                         st["max_gap_s"], min_rows)
    if rep["gate_fails"]:
        return rep
    seg_rows = sel[i0:i1 + 1]
    seg_ep = ep[i0:i1 + 1]
    res = detrend(seg_ep, [r["clock_ns"] for r in seg_rows])
    rep["rms_ns"] = math.sqrt(sum(r * r for r in res) / len(res))
    rep["dt_s"] = _median([b - a for a, b in zip(seg_ep, seg_ep[1:]) if b > a])
    rep["tdev"] = tdev(res, rep["dt_s"], GATES_TAU)
    if not rep["tdev"]:
        rep["gate_fails"] = ["no gate tau evaluable (span < 3*tau)"]
        return rep
    missing = [t for t in GATES_TAU if t not in rep["tdev"]]
    if missing:
        # never let verdict()'s all() pass vacuously on a partial tau table
        rep["gate_fails"] = [f"gate tau(s) not evaluable: {missing}"]
        return rep
    rep["verdict"] = verdict(rep["rms_ns"], rep["tdev"])
    d = [r["residual_rms_m"] - r["residual_rms_m_uw"] for r in seg_rows
         if "residual_rms_m_uw" in r]
    rep["ab_median_m"] = _median(d) if d else None
    rep["ab_n"] = len(d)
    return rep


def load_rows(path):
    """Parse a jsonl observations file; unparseable (torn-tail) lines skipped."""
    rows, bad = [], 0
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            bad += 1
    return rows, bad


def _gen_label(g):
    return "<none>" if g is None else str(g)


def main():
    argv = sys.argv[1:]
    min_rows = MIN_ROWS
    all_gens = "--all-gens" in argv
    if all_gens:
        argv.remove("--all-gens")
    if "--min-rows" in argv:
        i = argv.index("--min-rows")
        min_rows = int(argv[i + 1])
        del argv[i:i + 2]
    path = argv[0] if argv and not argv[0].startswith("-") else DEFAULT_PATH
    try:
        rows, bad = load_rows(path)
    except FileNotFoundError:
        rows, bad = [], 0
    rep = analyze(rows, min_rows=min_rows, all_gens=all_gens)

    print(f"file: {path} ({rep['n_in']} rows read"
          + (f", {bad} unparseable lines skipped" if bad else "") + ")")
    print(f"quality gates: {rep['n_quality']} rows (n_sat>=5, slips==0); "
          f"excluded {rep['n_poison']} poison-class rows "
          f"(residual_rms_m >= {POISON_RMS_M:.0f} m)")
    print(f"NOTE: poison gate {POISON_RMS_M:.0f} m is DATA-DERIVED from the "
          f"v2 rows it filters (see spec pre-registration 2026-08-29) — "
          f"verdicts using it are exploratory until re-derived from a "
          f"measurement-error budget or a held-out day")
    if rep["gens"]:
        print("gens found: " + ", ".join(
            f"{_gen_label(g)} ({c} rows)" for g, c in sorted(
                rep["gens"].items(), key=lambda kv: str(kv[0]))))
    if all_gens:
        print("gen analyzed: ALL (--all-gens)")
        print("WARNING: pooled TDEV across sessions is not claim-grade")
    else:
        print(f"gen analyzed: {_gen_label(rep['gen'])} "
              f"(latest; --all-gens to pool)")
    print(f"segments (split at inter-row gap > {GAP_FACTOR:.1f}x median dt "
          f"{rep['dt_median']:.2f} s):")
    for i, st in enumerate(rep["segments"]):
        print(f"  seg {i}: span {st['span_s']:.0f} s, {st['rows']} rows, "
              f"max gap {st['max_gap_s']:.1f} s")
    for h in rep["holes"]:
        print(f"  hole: {h['gap_s']:.0f} s gap after epoch {h['at_epoch']:.0f}")
    if rep["chosen_seg"] is not None:
        st = rep["segments"][rep["chosen_seg"]]
        print(f"analyzing longest segment: seg {rep['chosen_seg']} — "
              f"span {st['span_s']:.0f} s, {st['rows']} rows, "
              f"max gap {st['max_gap_s']:.1f} s")
    print(f"continuity gates: span>={MIN_SPAN_S:.0f} s AND rows>={min_rows} "
          f"AND max-gap<={MAX_GAP_S:.0f} s "
          f"(--min-rows overrides the rows floor only)")
    if rep["gate_fails"]:
        print("gate failure: " + "; ".join(rep["gate_fails"]))
        print("INSUFFICIENT DATA")
        sys.exit(1)
    print(f"detrended RMS: {rep['rms_ns']:.3f} ns   "
          f"(median dt {rep['dt_s']:.2f} s)")
    for t in GATES_TAU:
        v = rep["tdev"].get(t)
        print(f"  TDEV(tau={t:>4}s): "
              + (f"{v:.3f} ns" if v is not None else "n/a (span < 3*tau)"))
    ok = rep["verdict"]
    print("VERDICT:", "SUB-NS CLAIM SUPPORTED" if ok else "claim not supported")
    if rep.get("ab_median_m") is not None:
        print(f"paired A/B: median (weighted-raw RMS) = "
              f"{rep['ab_median_m']:+.2f} m over {rep['ab_n']} epochs "
              f"(negative = weighting helps)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
