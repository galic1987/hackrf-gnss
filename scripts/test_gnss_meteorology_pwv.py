#!/usr/bin/env python3
"""Unit tests for GNSS Meteorology & Precipitable Water Vapor (PWV) Inversion.

Verifies:
1. Bevis mean temperature relation Tm = 70.2 + 0.72 * Ts.
2. Bevis dimensionless conversion factor Pi = 1 / (1e-6 * rho_w * Rv * (k3/Tm + k2')).
3. Inversion of Zenith Wet Delay (ZWD) to Integrated Precipitable Water Vapor (PWV).
4. Vapor pressure and dew point calculations (Tetens / Bolton).
5. Slant Water Vapor (SWV) line-of-sight scaling.
6. Schema and boundary validations.
"""

import math
import unittest
import sys
import os

# Add scripts directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gnss_meteorology_pwv import (
    compute_bevis_tm,
    compute_bevis_pi,
    compute_pwv_mm,
    compute_vapor_pressures_and_dewpoint,
    compute_slant_water_vapor,
    evaluate_meteorology,
)

class TestGnssMeteorologyPWV(unittest.TestCase):

    def test_bevis_mean_temperature(self):
        """Verify Bevis mean atmospheric temperature relation."""
        # Ts = 20 C (293.15 K)
        t_c = 20.0
        tm_k = compute_bevis_tm(t_c)
        expected_tm = 70.2 + 0.72 * (t_c + 273.15)
        self.assertAlmostEqual(tm_k, expected_tm, places=3)
        self.assertAlmostEqual(tm_k, 281.268, delta=0.01)

        # Ts = 0 C (273.15 K)
        self.assertAlmostEqual(compute_bevis_tm(0.0), 266.868, delta=0.01)

    def test_bevis_pi_factor(self):
        """Verify Bevis dimensionless conversion factor Pi."""
        tm_k = 281.27
        pi_factor = compute_bevis_pi(tm_k)
        # Expected value is ~0.1588
        self.assertGreater(pi_factor, 0.150)
        self.assertLess(pi_factor, 0.170)
        self.assertAlmostEqual(pi_factor, 0.15878, delta=0.001)

    def test_pwv_inversion(self):
        """Verify conversion of Zenith Wet Delay (ZWD) into PWV (mm)."""
        zwd_m = 0.115  # 115 mm
        tm_k = 281.27
        pwv_mm = compute_pwv_mm(zwd_m, tm_k)
        # 115 mm * 0.15878 = 18.26 mm
        self.assertAlmostEqual(pwv_mm, 18.26, delta=0.1)

    def test_vapor_pressure_and_dewpoint(self):
        """Verify Magnus/Tetens saturation vapor pressure, actual vapor pressure, and dewpoint."""
        t_c = 20.0
        rh_pct = 50.0
        es, e0, td = compute_vapor_pressures_and_dewpoint(t_c, rh_pct)

        # At 20 C, es ~ 23.37 hPa, e0 ~ 11.69 hPa, Td ~ 9.27 C
        self.assertAlmostEqual(es, 23.37, delta=0.1)
        self.assertAlmostEqual(e0, 11.69, delta=0.1)
        self.assertAlmostEqual(td, 9.27, delta=0.2)

    def test_slant_water_vapor(self):
        """Verify Slant Water Vapor (SWV) projection along line-of-sight."""
        pwv_mm = 18.26
        map_wet = 2.292  # Elevation ~ 25.8 deg
        swv_mm = compute_slant_water_vapor(pwv_mm, map_wet)
        self.assertAlmostEqual(swv_mm, 41.85, delta=0.1)

    def test_evaluate_meteorology(self):
        """Verify full meteorological state dictionary."""
        tropo_state = {
            "surface_t0_c": 20.0,
            "surface_p0_hpa": 1010.8,
            "surface_rh_pct": 50.0,
            "zwd_m": 0.115,
            "zhd_m": 2.303,
            "ztd_m": 2.418,
            "tropo_satellites": {
                "GPS_4": {"map_wet": 1.1, "el_deg": 65.4, "sys": "gps", "prn": 4},
                "GPS_7": {"map_wet": 2.292, "el_deg": 25.8, "sys": "gps", "prn": 7},
            }
        }
        res = evaluate_meteorology(tropo_state)
        self.assertIn("meteorology_summary", res)
        self.assertIn("meteorology_satellites", res)

        summ = res["meteorology_summary"]
        self.assertAlmostEqual(summ["pwv_mm"], 18.26, delta=0.2)
        self.assertAlmostEqual(summ["bevis_tm_c"], 8.12, delta=0.2)
        self.assertAlmostEqual(summ["dew_point_c"], 9.27, delta=0.2)
        self.assertEqual(summ["convective_regime"], "NORMAL")

        sats = res["meteorology_satellites"]
        self.assertIn("GPS_7", sats)
        self.assertGreater(sats["GPS_7"]["swv_mm"], 40.0)

if __name__ == "__main__":
    unittest.main()
