#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/test_solar_noon_photochemistry_engine.py
================================================
Unit tests for Solar Noon Zenith Countdown & Chapman Diurnal Photochemistry Engine.
"""

import os
import sys
import unittest
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from solar_noon_photochemistry_engine import (
    compute_solar_ephemeris,
    compute_chapman_profile,
    evaluate_diurnal_lag,
    run_solar_noon_engine,
    STATE_FILE
)

class TestSolarNoonPhotochemistry(unittest.TestCase):
    def test_solar_ephemeris(self):
        utc_test = datetime(2026, 9, 4, 14, 0, 0, tzinfo=timezone.utc)
        eph = compute_solar_ephemeris(utc_test)
        self.assertIn("solar_elevation_deg", eph)
        self.assertIn("solar_zenith_deg", eph)
        self.assertIn("optical_airmass", eph)
        self.assertIn("solar_noon_utc", eph)
        self.assertGreater(eph["solar_elevation_deg"], 0.0)
        self.assertLess(eph["solar_elevation_deg"], 90.0)
        self.assertGreater(eph["optical_airmass"], 1.0)

    def test_chapman_profile(self):
        chap = compute_chapman_profile(zenith_deg=45.0)
        self.assertIn("production_ratio_q_over_q0", chap)
        self.assertAlmostEqual(chap["production_ratio_q_over_q0"], 0.7071, delta=0.01)
        self.assertIn("altitude_profile_q_m3_s", chap)
        self.assertIn("350km", chap["altitude_profile_q_m3_s"])
        self.assertGreater(chap["altitude_profile_q_m3_s"]["350km"], 1e8)

    def test_diurnal_lag(self):
        utc_now = datetime(2026, 9, 4, 14, 0, 0, tzinfo=timezone.utc)
        noon_dt = datetime(2026, 9, 4, 17, 9, 47, tzinfo=timezone.utc)
        lag = evaluate_diurnal_lag(utc_now, noon_dt)
        self.assertIn("countdown_to_solar_noon", lag)
        self.assertIn("predicted_vtec_peak_edt", lag)
        self.assertAlmostEqual(lag["recombination_time_constant_min"], 55.6, delta=1.0)

    def test_run_solar_noon_engine(self):
        res = run_solar_noon_engine()
        self.assertTrue(os.path.exists(STATE_FILE))
        self.assertIn("solar_geometry", res)
        self.assertIn("solar_noon_milestone", res)
        self.assertIn("chapman_photochemistry", res)
        self.assertIn("diurnal_continuity_model", res)

if __name__ == "__main__":
    unittest.main()
