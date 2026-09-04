#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/test_lambda_ambiguity_resolution_engine.py
==================================================
Unit tests for LAMBDA Ambiguity Resolution Engine.
"""

import os
import sys
import unittest
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lambda_ambiguity_resolution_engine import (
    LambdaEngine,
    run_lambda_engine,
    STATE_FILE,
    SIM_STATE_FILE
)

class TestLambdaEngine(unittest.TestCase):
    def test_decorrelate_and_search(self):
        # 2D test problem
        a_hat = np.array([5.002, -3.998], dtype=np.float64)
        Q_aa = np.array([[0.04, 0.03], [0.03, 0.04]], dtype=np.float64)
        
        Z, L, D, z_hat = LambdaEngine.decorrelate(Q_aa, a_hat)
        self.assertEqual(Z.shape, (2, 2))
        det_Z = int(round(np.linalg.det(Z)))
        self.assertIn(abs(det_Z), [1])  # unimodular integer matrix

        z1, omega1, z2, omega2 = LambdaEngine.integer_search(z_hat, L, D)
        self.assertGreater(omega2, omega1)
        ratio = omega2 / omega1
        self.assertGreater(ratio, 1.0)

    def test_run_lambda_engine(self):
        res = run_lambda_engine()
        self.assertIn("lambda_summary", res)
        self.assertIn("geodetic_position_improvement", res)
        self.assertIn("fixed_satellites", res)
        
        l_sum = res["lambda_summary"]
        self.assertIn("ratio_test_statistic", l_sum)
        self.assertGreater(l_sum["ratio_test_statistic"], 1.0)
        self.assertIn(l_sum["validation_verdict"], ["ACCEPTED_FIXED", "REJECTED_FLOAT"])

        # Evidence Envelope
        self.assertIn("evidence_envelope", res)
        env = res["evidence_envelope"]
        if env["quarantined"]:
            self.assertEqual(env["claim_class"], "simulation")
            self.assertFalse(env["validity"])
            self.assertTrue(os.path.exists(SIM_STATE_FILE))
            self.assertFalse(os.path.exists(STATE_FILE))
        else:
            self.assertEqual(env["claim_class"], "derived")
            self.assertTrue(env["validity"])
            self.assertEqual(env["uncertainty"]["units"], "m")
            self.assertTrue(os.path.exists(STATE_FILE))
            self.assertFalse(os.path.exists(SIM_STATE_FILE))

if __name__ == "__main__":
    unittest.main()
