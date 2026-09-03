"""Unit tests for scripts/iono_scintillation.py."""
import json
import math
import os
import sys
import tempfile
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import iono_scintillation as iono


def test_tec_physics_conversion_constant():
    # 40.3 * 10^16 * (f1^2 - f2^2) / (f1^2 * f2^2)
    f1 = 1575.42e6
    f2 = 1561.098e6
    expected = (1.0 / (40.3 * 1e16)) * (f1**2 * f2**2) / (f1**2 - f2**2)
    assert pytest.approx(iono.TEC_FACTOR_TECU_PER_M, rel=1e-5) == expected
    assert 330.0 < iono.TEC_FACTOR_TECU_PER_M < 340.0


def test_s4_amplitude_scintillation():
    # Constant C/N0 has 0 variance
    cn0_flat = [40.0] * 50
    assert iono.compute_s4(cn0_flat) == 0.0

    # Alternating C/N0 (fluctuating signal)
    cn0_fluct = [45.0 if i % 2 == 0 else 35.0 for i in range(50)]
    s4 = iono.compute_s4(cn0_fluct)
    assert s4 > 0.5  # strong scintillation signature


def test_sigma_phi_phase_scintillation():
    # Constant frequency (linear phase slope cyc = f0 * t) has 0 2nd-order polyfit residual
    times = [float(i) for i in range(60)]
    cycles_linear = [100.0 + 2.5 * i for i in range(60)]
    sigma = iono.compute_sigma_phi(times, cycles_linear)
    assert pytest.approx(sigma, abs=1e-6) == 0.0

    # Injected phase jitter
    jitter = [math.sin(i * 1.5) * 0.1 for i in range(60)]
    cycles_jitter = [100.0 + 2.5 * i + j for i, j in enumerate(jitter)]
    sigma_jit = iono.compute_sigma_phi(times, cycles_jitter)
    assert sigma_jit > 0.05


def test_roti_calculation():
    # Flat TEC
    dtec_flat = [10.0] * 30
    assert iono.compute_roti(dtec_flat, dt_s=1.0) == 0.0

    # Linear drift rate (e.g. constant plasma motion)
    dtec_ramp = [10.0 + 0.05 * i for i in range(30)]
    # Constant derivative has zero variance in ROT
    assert pytest.approx(iono.compute_roti(dtec_ramp, dt_s=1.0), abs=1e-6) == 0.0


def test_iono_monitor_pipeline():
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as tf:
        tracker_mock = {
            "epoch": 1788460000.0,
            "tracker": {
                "sats": [
                    {
                        "prn": 1,
                        "sys": "gps",
                        "carrier_cycles": 50000.0,
                        "cn0_proxy": 42.0,
                        "lock_s": 100.0,
                        "slip": False
                    },
                    {
                        "prn": 22,
                        "sys": "beidou",
                        "carrier_cycles": 49500.0,
                        "cn0_proxy": 39.0,
                        "lock_s": 80.0,
                        "slip": False
                    }
                ]
            }
        }
        json.dump(tracker_mock, tf)
        tf_path = tf.name

    state_tmp = tf_path + ".state.json"
    try:
        mon = iono.IonoMonitor(tracker_file=tf_path, sky_file="/tmp/nonexistent_sky.json", state_file=state_tmp)
        out = mon.update()
        assert out is not None
        assert out["weather"] == "QUIET"
        assert out["n_tracked"] == 2
        assert "GPS_1" in out["satellites"]
        assert "BEIDOU_22" in out["satellites"]
        assert os.path.exists(state_tmp)
    finally:
        if os.path.exists(tf_path):
            os.remove(tf_path)
        if os.path.exists(state_tmp):
            os.remove(state_tmp)
