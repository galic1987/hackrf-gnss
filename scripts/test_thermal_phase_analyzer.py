"""Unit tests for scripts/thermal_phase_analyzer.py."""
import json
import math
import os
import sys
import tempfile
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import thermal_phase_analyzer as tpa


def test_fit_diurnal_harmonics_recovery():
    # Synthesize 48 hours of phase data with known secular drift and 24h harmonic
    t = np.linspace(0, 48 * 3600, 1000)
    v_true = 50.0 / 86400.0  # 50 mm/day
    amp24_true = 15.0        # 15 mm amplitude
    amp12_true = 5.0         # 5 mm amplitude
    
    y = (
        100.0
        + v_true * t
        + amp24_true * np.sin(2 * np.pi * t / 86400.0)
        + amp12_true * np.cos(4 * np.pi * t / 86400.0)
        + np.random.normal(0, 0.5, len(t))
    )

    fit = tpa.fit_diurnal_harmonics(t, y)

    assert pytest.approx(fit["secular_drift_mm_per_day"], abs=2.0) == 50.0
    assert pytest.approx(fit["diurnal_amp_24h_mm"], abs=1.0) == 15.0
    assert pytest.approx(fit["semidiurnal_amp_12h_mm"], abs=1.0) == 5.0
    assert fit["residual_rms_mm"] < 1.0


def test_thermal_phase_analyzer_pipeline():
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as tf:
        for i in range(120):
            rec = {
                "t": 1788000000.0 + i * 360.0,
                "disp_mm": 10.0 + 0.1 * i,
                "lock": True
            }
            tf.write(json.dumps(rec) + "\n")
        tf_path = tf.name

    state_tmp = tf_path + ".state.json"
    try:
        orig_state = tpa.STATE_THERMAL_PATH
        tpa.STATE_THERMAL_PATH = state_tmp
        res = tpa.analyze_phase_thermals(history_path=tf_path)
        assert res is not None
        assert res["n_samples_analyzed"] >= 10
        assert os.path.exists(state_tmp)
    finally:
        tpa.STATE_THERMAL_PATH = orig_state
        if os.path.exists(tf_path):
            os.remove(tf_path)
        if os.path.exists(state_tmp):
            os.remove(state_tmp)
