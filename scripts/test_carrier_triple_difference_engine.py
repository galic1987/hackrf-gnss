#!/usr/bin/env python3
"""Unit tests for carrier_triple_difference_engine.py."""

import json
import os
import tempfile
import unittest

import carrier_triple_difference_engine as td_engine

class TestCarrierTripleDifferenceEngine(unittest.TestCase):
    def setUp(self):
        td_engine.STATE_HISTORY.clear()
        self.test_dir = tempfile.mkdtemp()
        self.dd_path = os.path.join(self.test_dir, 'state.double_difference.json')
        dd_data = {
            'epoch': 1000.0,
            'double_difference_summary': {
                'pivot_satellite': 'GPS_5',
                'pivot_elevation_deg': 74.0
            },
            'dd_pairs': {
                'GPS_21_vs_GPS_5': {
                    'satellite': 'GPS_21',
                    'pivot_satellite': 'GPS_5',
                    'elevation_deg': 76.0,
                    'dd_phase_residual_mm': 1.25,
                    'integer_ambiguity_cycles': 3
                },
                'GPS_12_vs_GPS_5': {
                    'satellite': 'GPS_12',
                    'pivot_satellite': 'GPS_5',
                    'elevation_deg': 26.0,
                    'dd_phase_residual_mm': -0.85,
                    'integer_ambiguity_cycles': -5
                }
            }
        }
        with open(self.dd_path, 'w') as f:
            json.dump(dd_data, f)

    def test_first_epoch_initialization(self):
        res = td_engine.process_triple_differences(self.test_dir)
        sm = res['triple_difference_summary']
        self.assertEqual(sm['pivot_satellite'], 'GPS_5')
        self.assertEqual(sm['n_triple_differenced_pairs'], 2)
        self.assertEqual(sm['cycle_slips_flagged'], 0)

    def test_epoch_differencing_and_slip_detection(self):
        # First epoch
        td_engine.process_triple_differences(self.test_dir)

        # Second epoch with small smooth drift
        dd_data_2 = {
            'epoch': 1002.0,
            'double_difference_summary': {'pivot_satellite': 'GPS_5'},
            'dd_pairs': {
                'GPS_21_vs_GPS_5': {
                    'satellite': 'GPS_21',
                    'pivot_satellite': 'GPS_5',
                    'elevation_deg': 76.0,
                    'dd_phase_residual_mm': 1.30, # +0.05 mm delta
                    'integer_ambiguity_cycles': 3
                },
                'GPS_12_vs_GPS_5': {
                    'satellite': 'GPS_12',
                    'pivot_satellite': 'GPS_5',
                    'elevation_deg': 26.0,
                    'dd_phase_residual_mm': 189.44, # ~190.29 mm slip!
                    'integer_ambiguity_cycles': -4
                }
            }
        }
        with open(self.dd_path, 'w') as f:
            json.dump(dd_data_2, f)

        res2 = td_engine.process_triple_differences(self.test_dir)
        pairs = res2['td_pairs']

        # GPS_21 should be continuous
        self.assertFalse(pairs['GPS_21_vs_GPS_5']['cycle_slip_detected'])
        self.assertAlmostEqual(pairs['GPS_21_vs_GPS_5']['td_phase_residual_mm'], 0.05, places=2)

        # GPS_12 should flag a +1 cycle slip
        self.assertTrue(pairs['GPS_12_vs_GPS_5']['cycle_slip_detected'])
        self.assertEqual(pairs['GPS_12_vs_GPS_5']['cycle_slip_jump_cycles'], 1)

if __name__ == '__main__':
    unittest.main()
