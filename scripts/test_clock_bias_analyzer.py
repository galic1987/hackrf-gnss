#!/usr/bin/env python3
"""Tests for scripts/clock_bias_analyzer.py (sub-ns claim gates, Leg 1 v2).

Fully synthetic — no radio, no live observations. Run:
    python3 scripts/test_clock_bias_analyzer.py
Also pytest-collectable (test_* functions, plain asserts)."""
import math
import random
import sys

from clock_bias_analyzer import analyze, detrend, tdev, verdict


def _white_pm(n, sigma, seed=7):
    """iid gaussian time-error series (white PM), no drift."""
    rng = random.Random(seed)
    return [rng.gauss(0.0, sigma) for _ in range(n)]


def test_white_pm_recovers_nist_tdev():
    # NIST SP 1065 derivation this test pins (the tau/m bookkeeping):
    #   ModAllanVar(m) = E[S_j^2] / (2 * tau^2 * m^2),  tau = m*dt,
    #   S_j = sum_{i=j}^{j+m-1} (x_{i+2m} - 2*x_{i+m} + x_i).
    # For iid x_k with variance sigma^2, each x_k enters S_j with block
    # weights (+1 for m indices, -2 for m indices, +1 for m indices), so
    #   E[S_j^2] = (m*1 + m*4 + m*1) * sigma^2 = 6*m*sigma^2
    #   ModAllanVar  = 6*m*sigma^2 / (2*tau^2*m^2) = 3*sigma^2 / (m*tau^2)
    #   TDEV = tau/sqrt(3) * sqrt(ModAllanVar) = sigma / sqrt(m)
    #        = sigma * sqrt(dt/tau)                    (white PM: tau^-1/2)
    # sigma = 0.4 ns, dt = 1 s, tau = 10 s -> TDEV = 0.4/sqrt(10) = 0.12649 ns.
    # The +/-30% bound on TDEV(10) fails loudly on the classic bookkeeping
    # errors: a stray sqrt(2) gives 1.414x (0.179), a stray sqrt(3) 1.732x
    # (0.219), both far outside [0.0885, 0.1644], while estimator scatter at
    # n = 7200 is a few percent. The slope pin kills the v1 statistic
    # outright: tau*ADEV/sqrt(3) from ordinary second differences is FLAT for
    # white PM (slope 0, TDEV ~= sigma), so v1 scores slope ~0 and TDEV(10)
    # ~0.4 — both outside the pins.
    n, sigma, dt = 7200, 0.4, 1.0
    res = _white_pm(n, sigma)
    tbl = tdev(res, dt, [10, 100, 1000])
    assert set(tbl) == {10, 100, 1000}, tbl  # 2 h span supports all three taus
    predicted_10 = sigma * math.sqrt(dt / 10.0)  # 0.12649 ns (derivation above)
    assert abs(tbl[10] / predicted_10 - 1.0) < 0.30, (tbl[10], predicted_10)
    slope = math.log(tbl[100] / tbl[10]) / math.log(100.0 / 10.0)
    assert -0.65 < slope < -0.35, slope


def test_detrend_removes_linear_drift():
    # kept from v1: least-squares detrend recovers the noise floor under drift.
    # Drift is 1.0 ns/s (span 7200 ns, rms ~2078 ns if NOT removed), so the
    # RMS bound alone fails loudly when the trend survives. The refit-slope
    # pin checks the LS residual is orthogonal to t (exact by construction,
    # up to fp) — endpoint differences are NOT pinned: they carry the noise.
    n, sigma = 7200, 0.4
    rng = random.Random(7)
    ep = [1_787_000_000.0 + k for k in range(n)]
    clk = [1.0 * k + rng.gauss(0.0, sigma) for k in range(n)]
    res = detrend(ep, clk)
    rms = math.sqrt(sum(r * r for r in res) / len(res))
    assert 0.35 < rms < 0.45, rms
    t0 = sum(ep) / n
    slope = sum((t - t0) * r for t, r in zip(ep, res)) / sum((t - t0) ** 2 for t in ep)
    assert abs(slope) < 1e-12, slope  # 1.0 ns/s drift is gone


def _rows(gen, t0, n, dt=1.0, rms_m=5.0):
    return [{"epoch": t0 + k * dt, "clock_ns": 0.01 * k, "clock_ns_uw": 0.01 * k,
             "residual_rms_m": rms_m, "residual_rms_m_uw": rms_m + 0.5,
             "n_sat": 8, "n_fresh": 2, "n_pred": 6, "slips": 0,
             "gen": gen, "source": "clock_bias"} for k in range(n)]


def test_gap_segmentation_longest_segment_analyzed():
    # 3000 rows @ 1 Hz, a 600 s hole, then 4000 rows @ 1 Hz
    rows = _rows("v2-1", 1_787_000_000.0, 3000)
    t1 = rows[-1]["epoch"] + 600.0
    rows += _rows("v2-1", t1, 4000)
    rep = analyze(rows)
    assert len(rep["segments"]) == 2, rep["segments"]
    assert rep["segments"][0]["rows"] == 3000
    assert rep["segments"][1]["rows"] == 4000
    # the gate report names the hole
    assert len(rep["holes"]) == 1 and abs(rep["holes"][0]["gap_s"] - 600.0) < 1.0, \
        rep["holes"]
    # the longest segment is the one analyzed, and it passes the gates
    assert rep["chosen_seg"] == 1, rep["chosen_seg"]
    assert rep["gate_fails"] == [], rep["gate_fails"]
    assert rep["segments"][1]["span_s"] >= 3600.0
    assert rep["verdict"] is not None
    assert rep["tdev"], rep["tdev"]


def test_continuity_gates_rows_do_not_equal_an_hour():
    # v1 defect: bare row count was treated as "a continuous hour".
    # Case A: rows >= 3400 but span < 3600 s (3700 rows @ 2 Hz = 1849.5 s)
    rep = analyze(_rows("v2-1", 1_787_000_000.0, 3700, dt=0.5))
    assert rep["verdict"] is None
    assert any("span" in f for f in rep["gate_fails"]), rep["gate_fails"]
    # Case B: span >= 3600 s but too few rows (1800 rows @ dt 4 s = 7196 s)
    rep = analyze(_rows("v2-1", 1_787_000_000.0, 1800, dt=4.0))
    assert rep["verdict"] is None
    assert any("rows" in f for f in rep["gate_fails"]), rep["gate_fails"]


def test_verdict_gates_unchanged():
    assert verdict(0.5, {10: 0.3, 100: 0.1, 1000: 0.05}) is True
    assert verdict(1.5, {10: 0.3, 100: 0.1, 1000: 0.05}) is False
    assert verdict(0.5, {10: 1.2, 100: 0.1, 1000: 0.05}) is False


def test_gen_latest_analyzed_by_default():
    # two gens; the newer gen is listed FIRST in the file to prove selection
    # is by epoch, not file position
    rows = _rows("v2-2000", 2_000_000.0, 100) + _rows("v2-1000", 1_000_000.0, 100)
    rep = analyze(rows)
    assert rep["gens"] == {"v2-1000": 100, "v2-2000": 100}, rep["gens"]
    assert rep["gen"] == "v2-2000", rep["gen"]
    assert rep["all_gens"] is False
    # --all-gens pools everything (forensics; warning flag set for main())
    rep = analyze(rows, all_gens=True)
    assert rep["gen"] is None and rep["all_gens"] is True
    assert rep["n_gen"] == 200, rep["n_gen"]


def main():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v)]
    failures = []
    for t in tests:
        try:
            t()
        except AssertionError as e:
            failures.append(t.__name__)
            print(f"FAIL {t.__name__}: {e}")
        else:
            print(f"PASS {t.__name__}")
    print(f"\n{len(failures)} failure(s)" if failures else "\nall tests passed")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
