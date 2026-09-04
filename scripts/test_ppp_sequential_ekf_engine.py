#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/test_ppp_sequential_ekf_engine.py
=========================================
Unit tests for Precise Point Positioning (PPP) Sequential EKF Engine.
"""

import os
import sys
import json
import unittest
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ppp_sequential_ekf_engine import (
    geodetic_to_ecef,
    ecef_to_enu,
    compute_phase_windup,
    PrecisePointPositioner,
    run_ppp_engine,
    REF_LAT_DEG,
    REF_LON_DEG,
    REF_ALT_M,
    STATE_FILE
)

class TestPPPSequentialEKF(unittest.TestCase):
    def test_geodetic_to_ecef(self):
        # Known point: equator on prime meridian at h=0
        xyz = geodetic_to_ecef(0.0, 0.0, 0.0)
        self.assertAlmostEqual(xyz[0], 6378137.0, delta=1.0)
        self.assertAlmostEqual(xyz[1], 0.0, delta=1.0)
        self.assertAlmostEqual(xyz[2], 0.0, delta=1.0)

        # Station coordinates
        stn_xyz = geodetic_to_ecef(REF_LAT_DEG, REF_LON_DEG, REF_ALT_M)
        self.assertGreater(np.linalg.norm(stn_xyz), 6.3e6)
        self.assertLess(np.linalg.norm(stn_xyz), 6.4e6)

    def test_phase_windup(self):
        wu_0 = compute_phase_windup(0.0, 45.0, sat_yaw_deg=0.0)
        wu_90 = compute_phase_windup(90.0, 45.0, sat_yaw_deg=0.0)
        self.assertIsInstance(wu_0, float)
        self.assertIsInstance(wu_90, float)
        self.assertGreaterEqual(wu_90, 0.0)
        self.assertLessEqual(wu_90, 0.20)  # less than ~1 wavelength (0.19 m)

    def test_run_ppp_engine(self):
        res = run_ppp_engine()
        self.assertTrue(os.path.exists(STATE_FILE))
        self.assertIn("estimated_coordinates", res)
        self.assertIn("sigma_3d_m", res["estimated_coordinates"])
        self.assertIn("receiver_clock", res)
        self.assertIn("troposphere_estimation", res)
        self.assertIn("float_ambiguities", res)
        
        sig_3d = res["estimated_coordinates"]["sigma_3d_m"]
        self.assertGreater(sig_3d, 0.0)
        self.assertLess(sig_3d, 10.0)

if __name__ == "__main__":
    unittest.main()
