"""Unit tests for scripts/sidereal_multipath_analyzer.py."""
import json
import math
import os
import sys
import tempfile
import pytest
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import sidereal_multipath_analyzer as sma


def test_elevation_baseline_derivation():
    # Synthetic observations where cn0 = 30 + 10 * sin(el_rad)
    obs = []
    for el in np.linspace(5, 85, 100):
        expected_cn0 = 30.0 + 10.0 * math.sin(math.radians(el))
        obs.append((180.0, float(el), expected_cn0, "gps", 1))

    baseline = sma.compute_elevation_baseline(obs)
    assert len(baseline) > 0
    # Test monotonicity: baseline at 80 deg should be higher than at 10 deg
    high_el = max(baseline.keys())
    low_el = min(baseline.keys())
    assert baseline[high_el] > baseline[low_el]
    assert pytest.approx(sma.lookup_baseline(10.0, baseline), abs=1.5) == 30.0 + 10.0 * math.sin(math.radians(10.0))


def test_multipath_grid_and_octants():
    # Create clean sector (SW: az 225 deg, zero anomaly) and noisy sector (E: az 90 deg, large fluctuations)
    baseline = {el: 40.0 for el in range(5, 90, 5)}
    obs = []

    # Clean SW sector: 50 observations at cn0 = 40.0 (delta = 0)
    for _ in range(50):
        obs.append((225.0, 45.0, 40.0, "gps", 1))

    # Noisy E sector: 50 observations alternating between 30.0 and 50.0 (delta = +/- 10 dB)
    for i in range(50):
        val = 30.0 if i % 2 == 0 else 50.0
        obs.append((90.0, 45.0, val, "gps", 2))

    grid, sectors, overall_mpi = sma.build_multipath_grid(obs, baseline, az_step=10, el_step=5)

    assert "220_45" in grid
    assert "90_45" in grid
    assert grid["220_45"]["status"] == "CLEAN"
    assert grid["220_45"]["mpi_db"] < 0.1
    assert grid["90_45"]["status"] == "SEVERE"
    assert grid["90_45"]["mpi_db"] > 9.0

    # Verify sector rankings
    assert sectors["South-West (SW)"]["mpi_db"] < 0.5
    assert sectors["East (E)"]["mpi_db"] > 9.0


def test_analyzer_end_to_end_pipeline():
    with tempfile.TemporaryDirectory() as tmpdir:
        hist_path = os.path.join(tmpdir, "sky_history.jsonl")
        state_sky = os.path.join(tmpdir, "state.sky.json")
        cache_path = os.path.join(tmpdir, "cache.json")
        out_path = os.path.join(tmpdir, "state.multipath.json")

        # Write synthetic sky_history.jsonl
        with open(hist_path, "w") as f:
            for t_step in range(10):
                frame = {
                    "t": 1788000000.0 + t_step * 30.0,
                    "sats": [
                        {"sys": "gps", "prn": 3, "cls": "tracked", "lock_s": 50.0, "az_deg": 220.0, "el_deg": 45.0, "cn0": 40.0},
                        {"sys": "gps", "prn": 4, "cls": "tracked", "lock_s": 50.0, "az_deg": 90.0, "el_deg": 20.0, "cn0": 30.0}
                    ]
                }
                f.write(json.dumps(frame) + "\n")

        # Write synthetic state.sky.json
        with open(state_sky, "w") as f:
            json.dump({
                "sky": {
                    "sats": [
                        {"sys": "gps", "prn": 3, "cls": "tracked", "az_deg": 220.5, "el_deg": 44.8, "cn0": 40.2}
                    ]
                }
            }, f)

        analyzer = sma.SiderealMultipathAnalyzer(
            history_path=hist_path,
            state_sky_path=state_sky,
            cache_path=cache_path,
            out_path=out_path
        )

        res = analyzer.evaluate_live()
        assert res is not None
        assert os.path.exists(out_path)
        assert os.path.exists(cache_path)
        assert "overall_mpi_db" in res
        assert "GPS_3" in res["active_satellites"]
        sat = res["active_satellites"]["GPS_3"]
        assert sat["reflection_risk"] in ["LOW", "ELEVATED", "HIGH"]

        # Test cache reuse: second instantiation should load from cache without error
        analyzer2 = sma.SiderealMultipathAnalyzer(
            history_path=hist_path,
            state_sky_path=state_sky,
            cache_path=cache_path,
            out_path=out_path
        )
        assert analyzer2.load_or_rebuild_cache(force_rebuild=False) is True
        assert len(analyzer2.grid) == len(analyzer.grid)
        assert analyzer2.total_samples == analyzer.total_samples
