#!/usr/bin/env python3
"""Tests for scripts/clock_bias_analyzer.py (sub-ns claim gates).

Fully synthetic — no radio, no live observations. Run:
    python3 scripts/test_clock_bias_analyzer.py
Also pytest-collectable (test_* functions, plain asserts)."""
import math
import sys

from clock_bias_analyzer import detrend, oadev, verdict


def test_recovers_known_white_noise_adev():
    # synthetic: 2 h at 1 Hz, white phase noise sigma = 0.4 ns
    import random
    random.seed(7)
    n = 7200
    ep = [1_787_000_000.0 + k for k in range(n)]
    clk = [1e-6 * k + random.gauss(0, 0.4) for k in range(n)]  # drift + noise
    res = detrend(ep, clk)
    rms = math.sqrt(sum(r * r for r in res) / len(res))
    assert 0.35 < rms < 0.45, rms
    tbl = oadev(res, 1.0, [10, 100, 1000])
    for t, v in tbl.items():
        assert 0.2 < v < 0.6, (t, v)      # TDEV of white phase noise ~= sigma, flat


def test_verdict_gates():
    assert verdict(0.5, {10: 0.3, 100: 0.1, 1000: 0.05}) is True
    assert verdict(1.5, {10: 0.3, 100: 0.1, 1000: 0.05}) is False
    assert verdict(0.5, {10: 1.2, 100: 0.1, 1000: 0.05}) is False


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
