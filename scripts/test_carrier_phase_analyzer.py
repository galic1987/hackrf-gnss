"""Unit test for scripts/carrier_phase_analyzer.py."""
import math
import os
import sys
import tempfile
import json
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import carrier_phase_analyzer as cpa


def test_mm_to_ns_conversion():
    # 299,792,458 mm in 1 second = 1e9 ns
    mm = 299792458.0 * 1000.0
    ns = cpa.mm_to_ns(mm)
    assert pytest.approx(ns, rel=1e-6) == 1e9
    # 1 mm should be ~3.3356 ps
    assert pytest.approx(cpa.mm_to_ns(1.0), rel=1e-4) == 0.00333564


def test_detrend_linear_recovery():
    epochs = [float(i) for i in range(100)]
    slope_truth = 0.05  # ns/s
    bias_truth = 10.0
    raw = [bias_truth + slope_truth * t + 0.1 * math.sin(t) for t in epochs]
    res, slope_fit = cpa.detrend(epochs, raw)
    assert pytest.approx(slope_fit, abs=1e-3) == slope_truth
    # residuals mean should be approximately 0
    assert pytest.approx(sum(res)/len(res), abs=1e-6) == 0.0


def test_tdev_white_phase_noise():
    # For white PM, TDEV(tau) = sigma / sqrt(m) where tau = m * dt
    dt = 1.0
    taus = [10, 20, 50]
    n = 1000
    # synthetic constant series
    residuals = [0.0] * n
    res = cpa.tdev(residuals, dt, taus)
    for tau in taus:
        assert tau in res
        assert pytest.approx(res[tau]["tdev_ns"], abs=1e-9) == 0.0


def test_carrier_phase_analyzer_e2e():
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as f:
        # Create 3650 rows at 1 Hz with 0.1 mm noise
        for i in range(3650):
            row = {
                "t": 1000.0 + i,
                "disp_mm": 0.5 * math.sin(i * 0.01),
                "sigma_mm": 0.5,
                "freq_off_hz": -31.5,
                "lock": True
            }
            f.write(json.dumps(row) + "\n")
        tmp_path = f.name

    try:
        rep = cpa.analyze_carrier_file(tmp_path, min_span=3600.0, min_rows=3400)
        assert rep["status"] == "PASS"
        assert rep["gates"]["span_ge_3600s"] is True
        assert rep["gates"]["rows_ge_3400"] is True
        assert rep["gates"]["max_gap_le_5s"] is True
        assert rep["gates"]["rms_lt_1ns"] is True
        assert rep["gates"]["tdev_lt_1ns"] is True
        assert rep["detrended_rms_ns"] < 0.01  # < 10 ps
    finally:
        os.remove(tmp_path)
