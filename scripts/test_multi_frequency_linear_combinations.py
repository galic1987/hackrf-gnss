#!/usr/bin/env python3
"""Unit tests for multi_frequency_linear_combinations.py."""

import json
import os
import tempfile
import unittest

from multi_frequency_linear_combinations import process_linear_combinations

class TestMultiFrequencyLinearCombinations(unittest.TestCase):
    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.klob_path = os.path.join(self.test_dir, 'state.klobuchar.json')
        self.tropo_path = os.path.join(self.test_dir, 'state.tropo.json')

        klob_data = {
            'satellites': {
                'GPS_14': {'el_deg': 60.0, 'az_deg': 120.0, 'klobuchar_delay_m': 3.10},
                'BEIDOU_28': {'el_deg': 45.0, 'az_deg': 180.0, 'klobuchar_delay_m': 4.20}
            }
        }
        with open(self.klob_path, 'w') as f: json.dump(klob_data, f)
        with open(self.tropo_path, 'w') as f: json.dump({}, f)

    def test_linear_combinations_cancellation(self):
        res = process_linear_combinations(self.test_dir)
        sm = res['linear_combinations_summary']
        self.assertEqual(sm['n_synthesized_satellites'], 2)
        self.assertAlmostEqual(sm['wide_lane_wavelength_m'], 20.932, places=2)

        combs = res['combinations']
        self.assertIn('GPS_14', combs)
        self.assertIn('BEIDOU_28', combs)

        # L_IF ionosphere must be canceled down to sub-millimeter level
        self.assertAlmostEqual(combs['GPS_14']['l_if_iono_residual_mm'], 0.0, places=1)
        self.assertAlmostEqual(combs['BEIDOU_28']['l_if_iono_residual_mm'], 0.0, places=1)

if __name__ == '__main__':
    unittest.main()
