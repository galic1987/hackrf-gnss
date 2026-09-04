#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/test_tropospheric_refractivity_ducting_sounder.py
========================================================
Unit tests for Tropospheric Refractivity Index & RF Ducting Sounder.
"""

import os
import sys
import math
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tropospheric_refractivity_ducting_sounder import (
    compute_water_vapor_pressure,
    compute_refractivity,
    run_refractivity_engine,
    STATE_FILE
)

class TestTropoRefractivitySounder(unittest.TestCase):
    def test_water_vapor_pressure(self):
        # At 0 deg C, saturation vapor pressure is ~6.11 hPa
        e0 = compute_water_vapor_pressure(0.0)
        self.assertAlmostEqual(e0, 6.1121, delta=0.1)

        # At 20 deg C, e ~ 23.38 hPa
        e20 = compute_water_vapor_pressure(20.0)
        self.assertAlmostEqual(e20, 23.38, delta=0.5)

    def test_refractivity_calculation(self):
        # P = 1013.25 hPa, T = 15 deg C (288.15 K), e = 10 hPa
        n_total, n_dry, n_wet = compute_refractivity(1013.25, 15.0, 10.0)
        # N_dry = 77.6 * (1013.25 / 288.15) ~ 272.87
        self.assertAlmostEqual(n_dry, 272.87, delta=1.0)
        # N_wet = 72 * (10/288.15) + 3.75e5 * (10 / 288.15^2) ~ 2.50 + 45.16 = 47.66
        self.assertAlmostEqual(n_wet, 47.66, delta=2.0)
        self.assertAlmostEqual(n_total, n_dry + n_wet, places=4)

    def test_run_refractivity_engine(self):
        res = run_refractivity_engine()
        self.assertTrue(os.path.exists(STATE_FILE))
        self.assertIn("refractivity_summary", res)
        self.assertIn("surface_meteorology", res)

        ref = res["refractivity_summary"]
        self.assertGreater(ref["surface_refractivity_n0"], 250.0)
        self.assertLess(ref["surface_refractivity_n0"], 450.0)

        # Effective Earth radius k-factor should be near 4/3 ~ 1.33
        self.assertGreater(ref["effective_earth_radius_k_factor"], 1.1)
        self.assertLess(ref["effective_earth_radius_k_factor"], 1.6)

        # Valid refraction regime
        self.assertIn(ref["refraction_regime"], [
            "STANDARD_REFRACTION", "SUPER_REFRACTION", "SUB_REFRACTION", "TRAPPING_DUCTING"
        ])
        self.assertGreater(ref["ground_radio_horizon_km"], 1.0)

        # Evidence Envelope
        self.assertIn("evidence_envelope", res)
        env = res["evidence_envelope"]
        self.assertEqual(env["claim_class"], "model")
        self.assertTrue(env["validity"])
        self.assertFalse(env["quarantined"])
        self.assertEqual(env["uncertainty"]["units"], "N-units")

if __name__ == "__main__":
    unittest.main()
