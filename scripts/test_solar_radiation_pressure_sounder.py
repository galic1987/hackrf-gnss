#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/test_solar_radiation_pressure_sounder.py
================================================
Unit tests for Solar Radiation Pressure (SRP) Sounder.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from solar_radiation_pressure_sounder import (
    compute_photon_pressure,
    evaluate_shadow_factor,
    run_srp_engine,
    STATE_FILE
)

class TestSRPSounder(unittest.TestCase):
    def test_photon_pressure(self):
        p_rad, s_flux = compute_photon_pressure(1.0)
        self.assertAlmostEqual(s_flux, 1361.0, delta=1.0)
        # P = S / c ~ 4.54 uPa
        self.assertAlmostEqual(p_rad * 1e6, 4.54, delta=0.05)

    def test_shadow_factor(self):
        # Day side: positive sun elevation -> full sunlight
        nu, state = evaluate_shadow_factor(20200.0, 45.0)
        self.assertEqual(nu, 1.0)
        self.assertEqual(state, "FULL_SUNLIGHT")

        # Deep night side: negative sun elevation -> eclipse
        nu_ecl, state_ecl = evaluate_shadow_factor(20200.0, -85.0)
        self.assertEqual(nu_ecl, 0.0)
        self.assertEqual(state_ecl, "UMBRA_TOTAL_ECLIPSE")

    def test_run_srp_engine(self):
        res = run_srp_engine()
        self.assertTrue(os.path.exists(STATE_FILE))
        self.assertIn("srp_summary", res)
        self.assertIn("srp_satellites", res)

        srp = res["srp_summary"]
        self.assertGreater(srp["mean_orbital_acceleration_nm_s2"], 10.0)
        self.assertLess(srp["mean_orbital_acceleration_nm_s2"], 200.0)

        # Evidence Envelope
        self.assertIn("evidence_envelope", res)
        env = res["evidence_envelope"]
        self.assertEqual(env["claim_class"], "model")
        self.assertTrue(env["validity"])
        self.assertFalse(env["quarantined"])
        self.assertEqual(env["uncertainty"]["units"], "nm/s^2")

if __name__ == "__main__":
    unittest.main()
