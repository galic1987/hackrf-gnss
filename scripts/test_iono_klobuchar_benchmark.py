"""Unit tests for scripts/iono_klobuchar_benchmark.py."""
import json
import math
import os
import sys
import tempfile
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import iono_klobuchar_benchmark as klob


def test_klobuchar_zenith_mapping_factor():
    # At 90 deg elevation (0.5 semi-circles): F(E) = 1 + 16 * (0.53 - 0.5)^3 = 1 + 16 * 0.000027 = 1.0004 ~ 1.0
    t_slant, t_vert, f_obliq, tec = klob.klobuchar_delay_s(
        lat_deg=40.0, lon_deg=-75.0, az_deg=0.0, el_deg=90.0, tow_s=50400.0
    )
    assert pytest.approx(f_obliq, abs=0.01) == 1.0
    assert pytest.approx(t_slant, rel=1e-3) == t_vert


def test_klobuchar_nighttime_floor():
    # At midnight (far from 14:00 local time = 50400s), delay should collapse to the 5 ns floor
    t_slant, t_vert, f_obliq, tec = klob.klobuchar_delay_s(
        lat_deg=40.0, lon_deg=-75.0, az_deg=0.0, el_deg=90.0, tow_s=0.0
    )
    assert pytest.approx(t_vert, abs=1e-11) == 5.0e-9
    assert pytest.approx(t_slant, abs=1e-11) == 5.0e-9


def test_klobuchar_elevation_scaling():
    # Slant delay at 15 deg elevation should be ~2.5x higher than at 90 deg
    t_90, _, f_90, _ = klob.klobuchar_delay_s(lat_deg=39.0, lon_deg=-77.0, az_deg=180.0, el_deg=90.0, tow_s=50400.0)
    t_15, _, f_15, _ = klob.klobuchar_delay_s(lat_deg=39.0, lon_deg=-77.0, az_deg=180.0, el_deg=15.0, tow_s=50400.0)
    assert f_15 > 2.3
    assert t_15 > 2.0 * t_90


def test_klobuchar_pipeline():
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as tf:
        mock_iono = {
            "epoch": 1788474000.0,
            "satellites": {
                "GPS_16": {
                    "sys": "gps",
                    "prn": 16,
                    "az_deg": 180.0,
                    "el_deg": 45.0,
                    "sigma_phi_rad": 0.05,
                    "s4": 0.04
                }
            }
        }
        json.dump(mock_iono, tf)
        tf_path = tf.name

    out_tmp = tf_path + ".klob.json"
    try:
        bench = klob.KlobucharBenchmark(state_iono_file=tf_path, out_file=out_tmp)
        res = bench.evaluate()
        assert res is not None
        assert res["n_benchmarked"] == 1
        assert "GPS_16" in res["satellites"]
        sat_data = res["satellites"]["GPS_16"]
        assert sat_data["klobuchar_delay_ns"] > 0.0
        assert sat_data["klobuchar_stec_tecu"] > 0.0
        assert os.path.exists(out_tmp)
    finally:
        if os.path.exists(tf_path):
            os.remove(tf_path)
        if os.path.exists(out_tmp):
            os.remove(out_tmp)
