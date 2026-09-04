#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/test_earth_solid_tide_sounder.py
========================================
Unit tests for Earth Solid Body Tide & Crustal Deformation Sounder.
"""

import os
import sys
import math
import unittest
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from earth_solid_tide_sounder import (
    compute_local_basis,
    compute_sun_vector,
    compute_approx_moon_vector,
    dot3,
    run_earth_tide_engine,
    LOVE_H2,
    LOVE_L2,
    STATE_FILE
)

class TestEarthSolidTideSounder(unittest.TestCase):
    def test_local_basis_orthonormality(self):
        r, e, n = compute_local_basis(39.0029, -77.6058)
        # Norms
        self.assertAlmostEqual(math.sqrt(dot3(r, r)), 1.0, places=6)
        self.assertAlmostEqual(math.sqrt(dot3(e, e)), 1.0, places=6)
        self.assertAlmostEqual(math.sqrt(dot3(n, n)), 1.0, places=6)
        # Orthogonality
        self.assertAlmostEqual(dot3(r, e), 0.0, places=6)
        self.assertAlmostEqual(dot3(r, n), 0.0, places=6)
        self.assertAlmostEqual(dot3(e, n), 0.0, places=6)

    def test_moon_vector_and_distance(self):
        now = datetime.now(timezone.utc)
        r_moon, dist_m = compute_approx_moon_vector(now, 39.0029, -77.6058)
        norm = math.sqrt(dot3(r_moon, r_moon))
        self.assertAlmostEqual(norm, 1.0, places=4)
        # Lunar distance in meters should be between 350,000 km and 410,000 km
        self.assertGreater(dist_m, 350000000.0)
        self.assertLess(dist_m, 410000000.0)

    def test_sun_vector(self):
        r_sun = compute_sun_vector(45.0, 180.0, 39.0029, -77.6058)
        norm = math.sqrt(dot3(r_sun, r_sun))
        self.assertAlmostEqual(norm, 1.0, places=6)

    def test_run_earth_tide_engine(self):
        res = run_earth_tide_engine()
        self.assertTrue(os.path.exists(STATE_FILE))
        self.assertIn("solid_earth_tide_summary", res)
        self.assertIn("constituents_breakdown", res)

        tide = res["solid_earth_tide_summary"]
        # Total displacement typically within [-300 mm, +300 mm]
        self.assertGreater(tide["delta_up_mm"], -300.0)
        self.assertLess(tide["delta_up_mm"], 300.0)
        self.assertGreater(tide["total_3d_displacement_mm"], 0.0)
        self.assertLess(tide["total_3d_displacement_mm"], 500.0)

        # Evidence Envelope
        self.assertIn("evidence_envelope", res)
        env = res["evidence_envelope"]
        self.assertEqual(env["claim_class"], "model")
        self.assertTrue(env["validity"])
        self.assertFalse(env["quarantined"])
        self.assertEqual(env["uncertainty"]["units"], "mm")

if __name__ == "__main__":
    unittest.main()
