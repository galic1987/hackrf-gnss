"""Unit tests for scripts/iono_tid_analyzer.py."""
import json
import math
import os
import sys
import tempfile
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import iono_tid_analyzer as tid


def test_detrend_diurnal_removes_polynomial():
    t = np.linspace(0, 3600, 300)
    # Quadratic diurnal background + small ripple
    trend = 10.0 + 0.002 * t - 0.0000005 * t**2
    ripple = 0.05 * np.sin(2 * np.pi * t / 600)
    y = trend + ripple
    
    detrended = tid.detrend_diurnal(t, y, deg=2)
    # Mean should be zero, trend removed
    assert pytest.approx(float(np.mean(detrended)), abs=1e-6) == 0.0
    assert pytest.approx(float(np.std(detrended)), abs=0.01) == float(np.std(ripple))


def test_compute_tid_spectrum_synthetic_mstid():
    # Simulate 1 hour (3600s) of data with an injected 30-minute MSTID (T = 1800s, A = 0.40 TECU)
    t = np.linspace(0, 3600, 600)
    omega_mstid = 2.0 * np.pi / 1800.0
    mstid_signal = 0.40 * np.sin(omega_mstid * t)
    noise = np.random.normal(0, 0.03, len(t))
    y = 12.0 + 0.001 * t + mstid_signal + noise

    periods, powers, peak_per, peak_amp, snr_db = tid.compute_tid_spectrum(t, y)

    assert 27.0 <= peak_per <= 33.0  # recovered 30-min period
    assert 0.30 <= peak_amp <= 0.50  # recovered amplitude
    assert snr_db > 6.0              # clear spectral peak above noise floor


def test_classify_tid_thresholds():
    # Quiescent
    assert tid.classify_tid(peak_period_min=30.0, peak_amp_tecu=0.04, snr_db=2.0) == "QUIESCENT"

    # MSTID (15 - 60 min, amp >= 0.10)
    assert tid.classify_tid(peak_period_min=30.0, peak_amp_tecu=0.35, snr_db=12.0) == "MSTID"
    assert tid.classify_tid(peak_period_min=20.0, peak_amp_tecu=0.15, snr_db=8.0) == "MSTID"

    # LSTID (> 60 min, amp >= 0.20)
    assert tid.classify_tid(peak_period_min=75.0, peak_amp_tecu=0.50, snr_db=15.0) == "LSTID"


def test_tid_analyzer_pipeline():
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as tf:
        # Write 60 synthetic records
        for i in range(60):
            t = 1788470000.0 + i * 10.0
            rec = {
                "epoch": t,
                "weather": "QUIET",
                "max_sigma": 0.04,
                "max_s4": 0.05,
                "dtec": 2.0 + 0.2 * math.sin(i * 0.1),
                "roti": 0.05,
                "n_sats": 10
            }
            tf.write(json.dumps(rec) + "\n")
        tf_path = tf.name

    state_tmp = tf_path + ".state.json"
    try:
        analyzer = tid.TIDAnalyzer(history_file=tf_path, state_file=state_tmp)
        out = analyzer.analyze()
        assert out is not None
        assert "classification" in out
        assert out["n_samples"] == 60
        assert os.path.exists(state_tmp)
    finally:
        if os.path.exists(tf_path):
            os.remove(tf_path)
        if os.path.exists(state_tmp):
            os.remove(state_tmp)
