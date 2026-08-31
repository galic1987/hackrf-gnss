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
    assert res["lag_ps"] == pytest.approx(-312_500.0, abs=3_200.0)
    assert res["coherence"] > 0.95


def test_large_zero_filled_shift_uses_overlap_normalization():
    rng = np.random.default_rng(11)
    s1 = rng.normal(size=4096) + 1j * rng.normal(size=4096)
    s2 = np.zeros_like(s1)
    s2[768:] = s1[:-768]
    res = compute_cross_correlation(s1, s2, sample_rate_hz=16e6)
    assert res["coarse_lag_samples"] == -768.0
    assert res["peak_correlation"] > 0.99


def test_carrier_phase_convention():
    rng = np.random.default_rng(12)
    s1 = rng.normal(size=2048) + 1j * rng.normal(size=2048)
    phi = 0.37
    s2 = s1 * np.exp(1j * phi)
    res = compute_cross_correlation(s1, s2, sample_rate_hz=16e6)
    assert res["carrier_phase_rad"] == pytest.approx(-phi, abs=1e-10)
    assert "coarse" in res["carrier_phase_method"]


@pytest.mark.parametrize("rate", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_sample_rate_rejected(rate):
    x = np.arange(256, dtype=float) + 0j
    with pytest.raises(ValueError, match="sample_rate_hz"):
        compute_cross_correlation(x, x, sample_rate_hz=rate)


@pytest.mark.parametrize("bad", [
    np.ones(256, dtype=complex),
    np.full(256, np.nan + 0j),
    np.ones((16, 16), dtype=complex),
])
def test_invalid_or_degenerate_inputs_rejected(bad):
    good = np.arange(256, dtype=float) + 1j * np.arange(256, dtype=float)
    with pytest.raises(ValueError):
        compute_cross_correlation(bad, good, sample_rate_hz=16e6)


@pytest.mark.parametrize("delay", [0.25, -0.25])
def test_fractional_delay_has_correct_sign(delay):
    # Smooth, band-limited analytic fixture. s2(t)=s1(t-delay), so the
    # s1-versus-s2 correlation convention reports lag -delay.
    rng = np.random.default_rng(20260830)
    n = np.arange(4096, dtype=float)
    freqs = rng.uniform(-0.18, 0.18, 32)
    coeff = rng.normal(size=32) + 1j * rng.normal(size=32)

    def signal(at):
        return np.exp(2j * np.pi * np.outer(at, freqs)) @ coeff

    s1 = signal(n)
    s2 = signal(n - delay)
    res = compute_cross_correlation(s1, s2, sample_rate_hz=16e6)
    assert np.sign(res["fine_lag_samples"]) == np.sign(-delay), res
    assert res["fine_lag_samples"] == pytest.approx(-delay, abs=0.08)

def test_abba_delay_separation():
    # Suppose receiver delay difference is 120 ps, cable asymmetry is 35 ps
    tau_rx = 120.0
    tau_cable = 35.0
    
    delay_ab = tau_rx + tau_cable  # 155 ps
    delay_ba = tau_rx - tau_cable  # 85 ps
    
    recovered_rx, recovered_cable = abba_delay_separation(delay_ab, delay_ba)
    assert recovered_rx == pytest.approx(120.0, abs=1e-6)
    assert recovered_cable == pytest.approx(35.0, abs=1e-6)
