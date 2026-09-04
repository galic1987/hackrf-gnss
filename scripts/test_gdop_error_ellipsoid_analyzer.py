#!/usr/bin/env python3
"""Unit tests for Multi-Constellation GDOP & Horizontal Error Ellipsoid Metrology Engine.

Verifies:
1. Direction cosine geometry and unit vector norm preservation in local ENU frame.
2. Parkinson ideal 4-satellite tetrahedron theoretical limit (GDOP = sqrt(3) ~= 1.732).
3. Singularity detection for coplanar / rank-deficient satellite geometries.
4. Horizontal error ellipse eigenvalue decomposition, orientation, and 95% scaling.
5. Multi-clock design matrix construction and constellation separation.
6. End-to-end GDOP analysis cycle schema conformity and atomic publication.
"""

import math
import os
import sys
import unittest
import numpy as np

# Add scripts directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gdop_error_ellipsoid_analyzer import (
    CHI2_95_2DOF,
    NOMINAL_UERE_M,
    compute_direction_cosines,
    build_design_matrix,
    compute_dop_metrics,
    evaluate_dop_solution,
    run_gdop_analysis_cycle
)


class TestGdopErrorEllipsoidAnalyzer(unittest.TestCase):

    def test_direction_cosines(self):
        """Verify local ENU line-of-sight unit vector calculations."""
        # 1. Zenith: Az=0, El=90
        ue, un, uu = compute_direction_cosines(0.0, 90.0)
        self.assertAlmostEqual(ue, 0.0, places=6)
        self.assertAlmostEqual(un, 0.0, places=6)
        self.assertAlmostEqual(uu, 1.0, places=6)

        # 2. Due North on horizon: Az=0, El=0
        ue, un, uu = compute_direction_cosines(0.0, 0.0)
        self.assertAlmostEqual(ue, 0.0, places=6)
        self.assertAlmostEqual(un, 1.0, places=6)
        self.assertAlmostEqual(uu, 0.0, places=6)

        # 3. Due East on horizon: Az=90, El=0
        ue, un, uu = compute_direction_cosines(90.0, 0.0)
        self.assertAlmostEqual(ue, 1.0, places=6)
        self.assertAlmostEqual(un, 0.0, places=6)
        self.assertAlmostEqual(uu, 0.0, places=6)

        # 4. Due South on horizon: Az=180, El=0
        ue, un, uu = compute_direction_cosines(180.0, 0.0)
        self.assertAlmostEqual(ue, 0.0, places=6)
        self.assertAlmostEqual(un, -1.0, places=6)
        self.assertAlmostEqual(uu, 0.0, places=6)

        # 5. Arbitrary angle: El=37 deg, Az=215 deg -> norm must be 1.0
        ue, un, uu = compute_direction_cosines(215.0, 37.0)
        norm = math.sqrt(ue**2 + un**2 + uu**2)
        self.assertAlmostEqual(norm, 1.0, places=6)

    def test_parkinson_ideal_tetrahedron(self):
        """Verify theoretical minimum GDOP = sqrt(3) on Parkinson 4-sat geometry."""
        # 1 at Zenith, 3 on horizon separated by 120 deg
        sats = [
            {"az_deg": 0.0, "el_deg": 90.0},
            {"az_deg": 0.0, "el_deg": 0.0},
            {"az_deg": 120.0, "el_deg": 0.0},
            {"az_deg": 240.0, "el_deg": 0.0},
        ]
        res = evaluate_dop_solution(sats, mode="single_clock")
        self.assertIsNotNone(res)

        # Theoretical values:
        # Q_diag = [2/3, 2/3, 4/3, 1/3]
        # GDOP = sqrt(2/3 + 2/3 + 4/3 + 1/3) = sqrt(3) ~= 1.73205
        # PDOP = sqrt(8/3) ~= 1.63299
        # HDOP = sqrt(4/3) ~= 1.15470
        # VDOP = sqrt(4/3) ~= 1.15470
        # TDOP = sqrt(1/3) ~= 0.57735
        self.assertAlmostEqual(res["gdop"], math.sqrt(3.0), places=2)
        self.assertAlmostEqual(res["pdop"], math.sqrt(8.0 / 3.0), places=2)
        self.assertAlmostEqual(res["hdop"], math.sqrt(4.0 / 3.0), places=2)
        self.assertAlmostEqual(res["vdop"], math.sqrt(4.0 / 3.0), places=2)
        self.assertAlmostEqual(res["tdop"], math.sqrt(1.0 / 3.0), places=2)

        # Invariant: GDOP^2 == PDOP^2 + TDOP^2
        self.assertAlmostEqual(res["gdop"] ** 2, res["pdop"] ** 2 + res["tdop"] ** 2, delta=0.05)

    def test_coplanar_singularity(self):
        """Verify that coplanar / rank-deficient satellite sets return None."""
        # 4 satellites all exactly on the horizon at the same azimuth (collinear)
        bad_sats = [
            {"az_deg": 45.0, "el_deg": 10.0},
            {"az_deg": 45.0, "el_deg": 20.0},
            {"az_deg": 45.0, "el_deg": 30.0},
            {"az_deg": 45.0, "el_deg": 40.0},
        ]
        res = evaluate_dop_solution(bad_sats)
        self.assertIsNone(res)

        # Less than 4 satellites
        few_sats = [
            {"az_deg": 0.0, "el_deg": 45.0},
            {"az_deg": 90.0, "el_deg": 45.0},
            {"az_deg": 180.0, "el_deg": 45.0},
        ]
        self.assertIsNone(evaluate_dop_solution(few_sats))

    def test_horizontal_error_ellipse(self):
        """Verify horizontal error ellipse eigenvalue and orientation calculations."""
        # 6-satellite constellation with varied elevations and balanced azimuths
        sats = [
            {"az_deg": 0.0, "el_deg": 75.0},
            {"az_deg": 60.0, "el_deg": 45.0},
            {"az_deg": 120.0, "el_deg": 30.0},
            {"az_deg": 180.0, "el_deg": 50.0},
            {"az_deg": 240.0, "el_deg": 35.0},
            {"az_deg": 300.0, "el_deg": 40.0},
        ]
        res = evaluate_dop_solution(sats)
        self.assertIsNotNone(res)
        ellipse = res["error_ellipse_95"]

        # Valid error ellipse dimensions
        self.assertGreater(ellipse["semi_major_a_m"], 0.0)
        self.assertGreater(ellipse["semi_minor_b_m"], 0.0)
        self.assertGreaterEqual(ellipse["semi_major_a_m"], ellipse["semi_minor_b_m"])
        self.assertGreater(ellipse["area_m2"], 0.0)
        self.assertGreaterEqual(ellipse["azimuth_theta_deg"], 0.0)
        self.assertLessEqual(ellipse["azimuth_theta_deg"], 180.0)

        # Check 95% scaling factor definition
        self.assertAlmostEqual(CHI2_95_2DOF, 2.4477, places=3)

    def test_multi_clock_design_matrix(self):
        """Verify multi-clock design matrix construction for Multi-GNSS."""
        sats = [
            {"az_deg": 10.0, "el_deg": 40.0, "sys": "gps"},
            {"az_deg": 70.0, "el_deg": 50.0, "sys": "gps"},
            {"az_deg": 130.0, "el_deg": 45.0, "sys": "beidou"},
            {"az_deg": 190.0, "el_deg": 60.0, "sys": "beidou"},
            {"az_deg": 250.0, "el_deg": 35.0, "sys": "galileo"},
            {"az_deg": 310.0, "el_deg": 55.0, "sys": "galileo"},
        ]
        G, active_sys = build_design_matrix(sats, mode="multi_clock")
        # 6 sats, 3 coordinate states + 3 clock states = 6 columns
        self.assertEqual(G.shape, (6, 6))
        self.assertEqual(active_sys, ["beidou", "galileo", "gps"])

        # Check row 0 (GPS sat): clock column for gps (index 2) must be 1.0, others 0.0
        self.assertEqual(G[0, 3], 0.0)  # beidou
        self.assertEqual(G[0, 4], 0.0)  # galileo
        self.assertEqual(G[0, 5], 1.0)  # gps

        # Check row 2 (BDS sat): clock column for beidou (index 0) must be 1.0
        self.assertEqual(G[2, 3], 1.0)  # beidou
        self.assertEqual(G[2, 4], 0.0)  # galileo
        self.assertEqual(G[2, 5], 0.0)  # gps

        # Solve multi-clock solution
        dop_mc = compute_dop_metrics(G, mode="multi_clock")
        self.assertIsNotNone(dop_mc)
        self.assertGreater(dop_mc["pdop"], 0.0)

    def test_end_to_end_synthesis_cycle(self):
        """Verify live analysis cycle execution and schema conformity."""
        state = run_gdop_analysis_cycle()
        self.assertIn("epoch", state)
        self.assertIn("gdop_summary", state)
        self.assertIn("tracked_solutions", state)
        self.assertIn("in_view_solutions", state)

        summ = state["gdop_summary"]
        self.assertIn("primary_gdop", summ)
        self.assertIn("primary_hdop", summ)
        self.assertIn("primary_vdop", summ)
        self.assertIn("fix_status", summ)
        self.assertGreater(summ["primary_gdop"], 0.0)
        self.assertGreater(summ["primary_hdop"], 0.0)
        self.assertGreater(summ["primary_vdop"], 0.0)


if __name__ == "__main__":
    unittest.main()
