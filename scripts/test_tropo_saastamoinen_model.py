"""Unit tests for scripts/tropo_saastamoinen_model.py."""
import json
import math
import os
import sys
import tempfile
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import tropo_saastamoinen_model as tsm


def test_saastamoinen_zhd_standard_atmosphere():
    # Saastamoinen standard reference: P0 = 1013.25 hPa at 45° latitude, sea level
    zhd = tsm.saastamoinen_zhd(p0_hpa=1013.25, lat_deg=45.0, height_km=0.0)
    assert pytest.approx(zhd, abs=1e-3) == 2.307


def test_saastamoinen_zwd_standard():
    # At 20°C, RH = 50%, e0 ~ 11.69 hPa
    e0 = tsm.water_vapor_pressure_hpa(t_c=20.0, rh_pct=50.0)
    assert 11.0 <= e0 <= 12.5
    zwd = tsm.saastamoinen_zwd(t0_c=20.0, e0_hpa=e0)
    assert 0.10 <= zwd <= 0.13


def test_niell_mapping_bounds():
    # At zenith (90°), mapping function must be 1.000
    mh_90 = tsm.niell_mapping_hydrostatic(90.0)
    mw_90 = tsm.niell_mapping_wet(90.0)
    assert pytest.approx(mh_90, abs=0.01) == 1.00
    assert pytest.approx(mw_90, abs=0.01) == 1.00

    # At 15° elevation, mapping factor is ~3.8x zenith
    mh_15 = tsm.niell_mapping_hydrostatic(15.0)
    mw_15 = tsm.niell_mapping_wet(15.0)
    assert 3.7 <= mh_15 <= 4.0
    assert 3.7 <= mw_15 <= 4.0

    # At 5° elevation, mapping factor is ~10x zenith
    mh_5 = tsm.niell_mapping_hydrostatic(5.0)
    assert 9.5 <= mh_5 <= 11.0


def test_tropo_pipeline():
    with tempfile.TemporaryDirectory() as tmpdir:
        state_klob = os.path.join(tmpdir, "state.klob.json")
        out_state = os.path.join(tmpdir, "state.tropo.json")

        # Mock Klobuchar state with 1 satellite
        with open(state_klob, "w") as f:
            json.dump({
                "satellites": {
                    "GPS_16": {
                        "az_deg": 180.0,
                        "el_deg": 45.0,
                        "klobuchar_delay_ns": 8.0,
                        "klobuchar_delay_m": 2.4
                    }
                }
            }, f)

        model = tsm.TroposphericModel(
            state_klob_path=state_klob,
            state_iono_path=state_klob,
            out_path=out_state
        )

        res = model.evaluate()
        assert res is not None
        assert os.path.exists(out_state)
        assert res["n_tropo_benchmarked"] == 1
        assert "GPS_16" in res["tropo_satellites"]

        sat = res["tropo_satellites"]["GPS_16"]
        assert sat["tropo_delay_m"] > 3.0
        assert sat["tropo_delay_ns"] > 10.0
        # Total atmospheric delay is sum of tropo + iono
        assert sat["total_atm_delay_ns"] == round(sat["tropo_delay_ns"] + 8.0, 2)
        assert sat["total_atm_delay_m"] > sat["tropo_delay_m"]
