#!/usr/bin/env python3
"""Unit test for Opera Cake Multi-Band Scientific Observatory & Evidence Envelope."""

import os
import sys
import unittest
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from evidence_envelope import ClaimClass
from operacake_multiband_observatory import compute_cross_port_metrology

class TestOperacakeObservatory(unittest.TestCase):
    def test_compute_cross_port_metrology(self):
        atsc_dummy = {"snr_db": 30.0}
        adsb_dummy = {"messages_decoded": 5}
        fm_dummy = {"carrier_snr_db": 35.0}

        res = compute_cross_port_metrology(atsc_dummy, adsb_dummy, fm_dummy)
        self.assertIn("building_penetration_loss_db", res)
        self.assertIn("estimated_system_temp_k", res)
        self.assertIn("estimated_front_end_nf_db", res)
        self.assertGreater(res["building_penetration_loss_db"], 0)
        self.assertGreater(res["estimated_system_temp_k"], 100.0)
        self.assertLess(res["estimated_front_end_nf_db"], 10.0)

if __name__ == "__main__":
    unittest.main()
