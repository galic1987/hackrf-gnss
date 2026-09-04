#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/test_evidence_envelope.py
=================================
Unit tests for Evidence Envelope & Provenance Contract.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from evidence_envelope import ClaimClass, make_evidence_envelope

class TestEvidenceEnvelope(unittest.TestCase):
    def test_observed_envelope(self):
        env = make_evidence_envelope(
            ClaimClass.OBSERVED,
            observation_epoch=1788500000.0,
            uncertainty={"value": 0.02, "units": "m", "confidence": "1_sigma"}
        )
        self.assertEqual(env["claim_class"], "observed")
        self.assertFalse(env["quarantined"])
        self.assertTrue(env["validity"])
        self.assertEqual(env["observation_epoch"], 1788500000.0)
        self.assertEqual(len(env["failure_reasons"]), 0)

    def test_simulation_envelope_quarantine(self):
        env = make_evidence_envelope(
            ClaimClass.SIMULATION,
            uncertainty={"value": 1e-14, "units": "adev", "confidence": "numerical"}
        )
        self.assertEqual(env["claim_class"], "simulation")
        self.assertTrue(env["quarantined"])
        self.assertFalse(env["validity"])
        self.assertIsNone(env["observation_epoch"])
        self.assertIn("SYNTHETIC_NUMERICAL_SIMULATION", env["failure_reasons"])

    def test_input_skew_detection(self):
        now = 1788500100.0
        stale_input = 1788500000.0  # 100 s ago, exceeds 5 s permitted skew
        env = make_evidence_envelope(
            ClaimClass.DERIVED,
            observation_epoch=now,
            input_epochs={"tracker": stale_input},
            permitted_skew_s=5.0
        )
        self.assertEqual(env["claim_class"], "derived")
        self.assertFalse(env["validity"])
        self.assertTrue(any("INPUT_EPOCH_SKEW_EXCEEDED" in r for r in env["failure_reasons"]))

if __name__ == "__main__":
    unittest.main()
