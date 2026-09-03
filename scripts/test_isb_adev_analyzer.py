"""Unit tests for scripts/isb_adev_analyzer.py."""
import json
import math
import os
import sys
import tempfile
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import isb_adev_analyzer as isb


def test_compute_adev_synthetic_white_phase():
    # Phase data with known white noise: ADEV should slope as 1/tau
    t = np.arange(0, 1000, 1.0)
    # White phase noise in seconds (1 ns sigma)
    y = np.random.normal(0, 1e-9, len(t))
    
    adev_map = isb.compute_adev(t, y, [10.0, 100.0])
    assert adev_map[10.0] is not None
    assert adev_map[100.0] is not None
    # ADEV at 100s should be lower than at 10s
    assert adev_map[100.0] < adev_map[10.0]


def test_isb_analyzer_pipeline():
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as tf:
        for i in range(20):
            rec = {
                "epoch": 1788000000.0 + i * 60.0,
                "mode": "3D(mixed GPS+BDS)",
                "isx_km": 0.003 + 0.0001 * (i % 3)  # ~10 ns
            }
            tf.write(json.dumps(rec) + "\n")
        tf_path = tf.name

    state_tmp = tf_path + ".state.json"
    try:
        # Patch paths
        orig_state = isb.STATE_ISB_PATH
        isb.STATE_ISB_PATH = state_tmp
        res = isb.analyze_isb(history_path=tf_path)
        assert res is not None
        assert res["n_samples"] == 20
        assert 9.0 <= res["isb_median_ns"] <= 12.0
        assert os.path.exists(state_tmp)
    finally:
        isb.STATE_ISB_PATH = orig_state
        if os.path.exists(tf_path):
            os.remove(tf_path)
        if os.path.exists(state_tmp):
            os.remove(state_tmp)
