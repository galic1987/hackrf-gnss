#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/test_satellite_atomic_clock_analyzer.py
===============================================
Unit tests for Satellite In-Orbit Atomic Clock Stability & ADEV Analyzer.
"""

import os
import sys
import math
import unittest
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from satellite_atomic_clock_analyzer import (
    compute_allan_deviation,
    run_clock_adev_engine,
    SIM_STATE_FILE
)

class TestSatelliteClockAnalyzer(unittest.TestCase):
    def test_allan_deviation_linear_ramp(self):
        # A pure frequency offset (linear phase ramp x(t) = y0 * t) has zero second difference
        t = np.arange(500, dtype=np.float64)
        ramp = 1e-11 * t
        sigma_y = compute_allan_deviation(ramp, tau_m=10, dt_s=1.0)
        # Should be numerically zero (or floor 1e-18)
        self.assertLess(sigma_y, 1e-15)

    def test_allan_deviation_noise(self):
        np.random.seed(42)
        noise = np.random.normal(0, 1e-12, 500)
        sigma_y = compute_allan_deviation(noise, tau_m=10, dt_s=1.0)
        self.assertIsNotNone(sigma_y)
        self.assertGreater(sigma_y, 1e-15)
        self.assertLess(sigma_y, 1e-11)

    def test_run_clock_adev_engine(self):
        res = run_clock_adev_engine()
        self.assertTrue(os.path.exists(SIM_STATE_FILE))
        self.assertIn("satellite_atomic_clock_summary", res)
        self.assertIn("space_clocks", res)
        self.assertIn("evidence_envelope", res)

        env = res["evidence_envelope"]
        self.assertEqual(env["claim_class"], "simulation")
        self.assertTrue(env["quarantined"])
        self.assertFalse(env["validity"])
        self.assertIn("SYNTHETIC_NUMERICAL_SIMULATION", env["failure_reasons"])

        summ = res["satellite_atomic_clock_summary"]
        self.assertEqual(summ["claim_class"], "SIMULATION")
        self.assertEqual(summ["quarantine_status"], "QUARANTINED_SIMULATION")
        self.assertIn("Maser", summ["most_stable_clock_type"])
        # ADEV at 300s for maser should be ~ 1e-14
        self.assertLess(summ["best_adev_tau_300s"], 5e-14)
        self.assertGreater(summ["best_adev_tau_300s"], 1e-15)
        self.assertGreater(summ["n_space_clocks_benchmarked"], 0)

        # Check space clocks
        for sat_id, cinfo in res["space_clocks"].items():
            self.assertIn("constellation", cinfo)
            self.assertIn("atomic_oscillator", cinfo)
            self.assertIn("allan_deviation", cinfo)
            self.assertIn("tau_300s", cinfo["allan_deviation"])

if __name__ == "__main__":
    unittest.main()
