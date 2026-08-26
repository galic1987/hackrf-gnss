#!/usr/bin/env python3
"""Unit tests for series_producer's consensus election (quarantine /
suspect-midpoint) and durable alert history. Plain asserts, no pytest;
writes only to a temp dir.

  python3 scripts/test_series_producer.py
"""
import json
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import series_producer as sp

FAILURES = []


def check(name, cond, detail=""):
    print(f"{'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILURES.append(name)


def main():
    # --- 3 voters, one 8-sigma outlier -> quarantined, consensus of the rest
    cons, alerts, suspect = sp.elect(
        [("GPS", -0.50, 0.05), ("SBAS", -0.55, 0.05), ("ch35", -1.30, 0.05)])
    check("outlier quarantined", any("QUARANTINED" in a and "ch35" in a
                                     for a in alerts), f"{alerts}")
    check("consensus recomputed without outlier", abs(cons - (-0.525)) < 0.03,
          f"cons={cons}")
    check("3-voter case not suspect", not suspect)

    # --- exactly 2 voters disagreeing: midpoint meaningless, trust neither --
    cons, alerts, suspect = sp.elect([("GPS", -0.50, 0.05), ("ch35", -0.90, 0.05)])
    check("2-voter disagreement is suspect", suspect)
    check("2-voter alert says trust neither", any("trust neither" in a
                                                  for a in alerts), f"{alerts}")
    check("2-voter midpoint still published", abs(cons - (-0.70)) < 1e-9,
          f"cons={cons}")

    # --- agreement -> quiet ---------------------------------------------------
    cons, alerts, suspect = sp.elect([("GPS", -0.50, 0.05), ("SBAS", -0.52, 0.05)])
    check("agreeing voters: no alerts", alerts == [] and not suspect)

    # --- sigma floor: tight per-instrument sigma cannot weaponize the z-test --
    cons, alerts, suspect = sp.elect([("GPS", -0.50, 0.001), ("SBAS", -0.54, 0.001)])
    check("sigma floor 0.05 prevents cry-wolf", alerts == [] and not suspect,
          f"{alerts}")

    # --- one voter -> published but flagged: no redundancy (round-5) ----------
    cons, alerts, suspect = sp.elect([("GPS", -0.5, 0.05)])
    check("single voter is suspect, not silently consensus",
          cons == -0.5 and suspect and alerts, f"{cons} {alerts}")
    cons, alerts, _ = sp.elect([])
    check("no voters -> None", cons is None and alerts == [])

    # --- alert history: durable, deduplicated ----------------------------------
    tmp = tempfile.mkdtemp(prefix="series_test_")
    path = os.path.join(tmp, "alert_history.jsonl")
    sp._last_alert_key = None
    sp.persist_alerts(["a"], 1000.0, path=path)
    sp.persist_alerts(["a"], 1030.0, path=path)   # same set -> not re-appended
    sp.persist_alerts([], 1060.0, path=path)      # cleared -> nothing
    sp.persist_alerts(["a", "b"], 1090.0, path=path)
    lines = open(path).read().strip().split("\n")
    check("alert history dedupes", len(lines) == 2, f"{len(lines)} lines")
    rec = json.loads(lines[-1])
    check("history rows carry time + set", rec["t"] == 1090.0
          and rec["alerts"] == ["a", "b"])

    # --- CLKIN soft verification (round-14) ------------------------------------
    # shared clock: both series ride ONE oscillator — a large common wander
    # cancels in the diff; only small independent measurement noise remains.
    t0, n, dt = 1000.0, 50, 30.0
    common = [0.30 * math.sin(i * dt / 1800.0) for i in range(n)]
    atsc = [[t0 + i * dt, common[i] + 0.002 * math.sin(i * 1.7)] for i in range(n)]
    waas = [[t0 + i * dt, common[i] + 0.008 * math.sin(i * 2.9 + 1.0)] for i in range(n)]
    v, d = sp.clkin_soft_verify(atsc, waas)
    check("shared clock verifies", v is True,
          f"rms={d['diff_rms_ppm']} slope={d['diff_slope_ppm_per_min']}")
    check("diag carries pairs/window/stats", d["pairs"] == n and d["window_s"] == sp.SOFT_WINDOW
          and d["diff_rms_ppm"] is not None and d["mean_diff_ppm"] is not None, f"{d}")

    # independent TCXOs: the One's own wander does NOT cancel — diff wanders
    # and drifts well beyond the gates (0.06 ppm amplitude, 240 s period).
    atsc_free = [[t0 + i * dt, common[i] + 0.06 * math.sin(i * dt / 240.0)] for i in range(n)]
    v, d = sp.clkin_soft_verify(atsc_free, waas)
    check("independent-TCXO wander does not verify", v is False,
          f"rms={d['diff_rms_ppm']} slope={d['diff_slope_ppm_per_min']}")

    # a constant per-transmitter offset is NOT clock information: offset-
    # invariant gates still verify (measured ch35 bias ≈ -0.05 ppm).
    atsc_bias = [[t, x - 0.05] for t, x in atsc]
    v, d = sp.clkin_soft_verify(atsc_bias, waas)
    check("constant transmitter offset still verifies", v is True,
          f"mean={d['mean_diff_ppm']} rms={d['diff_rms_ppm']}")

    # insufficient pairs -> None (unknown): the gate stays closed, never guesses
    v, d = sp.clkin_soft_verify(atsc[:10], waas[:10])
    check("insufficient pairs -> null", v is None and d["pairs"] == 10, f"{d}")
    sparse = [[t0 + i * 200.0, 0.0] for i in range(n)]        # 200-s grid
    off_grid = [[t0 + 100.0 + i * 200.0, 0.0] for i in range(n)]  # 100 s away
    v, d = sp.clkin_soft_verify(sparse, off_grid)
    check("unpairable epochs -> null", v is None and d["pairs"] == 0, f"{d}")

    # voter gate: hard probe primary, soft inference fallback (review's OR)
    check("probe True votes", sp.atsc_may_vote(True, None) is True)
    check("probe None + soft True votes", sp.atsc_may_vote(None, True) is True)
    check("probe None + soft False closed", sp.atsc_may_vote(None, False) is False)
    check("probe None + soft None closed", sp.atsc_may_vote(None, None) is False)
    check("probe False + soft True votes (spec OR)", sp.atsc_may_vote(False, True) is True)

    print(f"\n{len(FAILURES)} failure(s)" if FAILURES else "\nall tests passed")
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
