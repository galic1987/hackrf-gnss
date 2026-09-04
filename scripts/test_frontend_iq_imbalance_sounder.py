#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/test_frontend_iq_imbalance_sounder.py
=============================================
Unit tests for HackRF Front-End Direct Baseband I/Q Imbalance & Mixer Sounder.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from frontend_iq_imbalance_sounder import (
    IQImbalanceSounder,
    run_sounder,
    STATE_FILE
)

class TestFrontendIQImbalance(unittest.TestCase):
    def test_analyzer(self):
        sounder = IQImbalanceSounder()
        res = sounder.analyze()
        self.assertIn("dc_carrier_leakage", res)
        self.assertIn("quadrature_imbalance", res)
        self.assertIn("gram_schmidt_calibrator", res)

        q_imb = res["quadrature_imbalance"]
        self.assertIn("gain_ratio_alpha", q_imb)
        self.assertIn("phase_skew_deg", q_imb)
        self.assertIn("image_rejection_ratio_db", q_imb)
        self.assertGreater(q_imb["image_rejection_ratio_db"], 10.0)

        gs = res["gram_schmidt_calibrator"]
        self.assertEqual(len(gs["matrix_row_0"]), 2)
        self.assertEqual(len(gs["matrix_row_1"]), 2)
        self.assertEqual(gs["matrix_row_0"][0], 1.0)
        self.assertEqual(gs["matrix_row_0"][1], 0.0)

    def test_run_sounder(self):
        res = run_sounder()
        self.assertTrue(os.path.exists(STATE_FILE))
        self.assertIn("hardware_health_tier", res["quadrature_imbalance"])

if __name__ == "__main__":
    unittest.main()
