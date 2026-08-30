import os
import sys
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))

from dual_radio_coherence import (
    compute_cross_correlation,
    abba_delay_separation,
)

def test_zero_lag_cross_correlation():
    # Synthesize identical 1024-sample chirp signal
    t = np.linspace(0, 1e-4, 2048)
    tone = np.exp(2j * np.pi * 1.5e6 * t)
    
    res = compute_cross_correlation(tone, tone, sample_rate_hz=16e6)
    assert res["coarse_lag_samples"] == 0.0
    assert abs(res["fine_lag_samples"]) < 1e-3
    assert abs(res["lag_ps"]) < 10.0
    assert res["coherence"] > 0.99
    assert abs(res["carrier_phase_rad"]) < 1e-3

def test_known_lag_cross_correlation():
    # Synthesize signal with 5 sample delay
    np.random.seed(42)
    s1 = np.random.randn(4096) + 1j * np.random.randn(4096)
    s2 = np.roll(s1, 5)
    
    res = compute_cross_correlation(s1, s2, sample_rate_hz=16e6)
    assert res["coarse_lag_samples"] == -5.0
    assert abs(res["fine_lag_samples"] - (-5.0)) < 0.05
    assert res["coherence"] > 0.95

def test_abba_delay_separation():
    # Suppose receiver delay difference is 120 ps, cable asymmetry is 35 ps
    tau_rx = 120.0
    tau_cable = 35.0
    
    delay_ab = tau_rx + tau_cable  # 155 ps
    delay_ba = tau_rx - tau_cable  # 85 ps
    
    recovered_rx, recovered_cable = abba_delay_separation(delay_ab, delay_ba)
    assert recovered_rx == pytest.approx(120.0, abs=1e-6)
    assert recovered_cable == pytest.approx(35.0, abs=1e-6)
