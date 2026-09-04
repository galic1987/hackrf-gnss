#!/usr/bin/env python3
"""Unit tests for carrier_double_difference_engine.py."""

import json
import os
import tempfile
import unittest

from carrier_double_difference_engine import process_double_differences, LAMBDA_L1

class TestCarrierDoubleDifferenceEngine(unittest.TestCase):
    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.sd_path = os.path.join(self.test_dir, 'state.single_difference.json')
        self.klob_path = os.path.join(self.test_dir, 'state.klobuchar.json')

        sd_data = {
            'single_difference_summary': {
                'pivot_satellite': 'GPS_14',
                'pivot_elevation_deg': 68.0,
                'pivot_azimuth_deg': 110.0
            },
            'sd_pairs': {
                'GPS_03_vs_GPS_14': {
                    'satellite': 'GPS_03',
                    'pivot_satellite': 'GPS_14',
                    'el_deg': 32.0,
                    'az_deg': 45.0,
                    'cn0_dbhz': 39.0,
                    'baseline_angle_deg': 40.0
                },
                'GPS_17_vs_GPS_14': {
                    'satellite': 'GPS_17',
                    'pivot_satellite': 'GPS_14',
                    'el_deg': 22.0,
                    'az_deg': 280.0,
                    'cn0_dbhz': 35.0,
                    'baseline_angle_deg': 85.0
                }
            }
        }

        with open(self.sd_path, 'w') as f:
            json.dump(sd_data, f)

    def test_clock_and_atmosphere_cancellations(self):
        res = process_double_differences(self.test_dir)
        summary = res['double_difference_summary']
        self.assertEqual(summary['pivot_satellite'], 'GPS_14')
        self.assertEqual(summary['receiver_clock_bias_cancellation'], '100.0% CANCELED (0.000 ps)')
        self.assertEqual(summary['satellite_clock_bias_cancellation'], '100.0% CANCELED (0.000 ps)')
        self.assertEqual(summary['atmospheric_common_mode_rejection'], '100.0% CANCELED (0.000 mm)')

    def test_integer_ambiguity_properties(self):
        res = process_double_differences(self.test_dir)
        pairs = res['dd_pairs']
        self.assertIn('GPS_03_vs_GPS_14', pairs)
        p = pairs['GPS_03_vs_GPS_14']
        self.assertIsInstance(p['integer_ambiguity_cycles'], int)
        self.assertAlmostEqual(p['carrier_wavelength_mm'], 190.29, places=1)
        # In zero-baseline, phase residual must be sub-centimeter
        self.assertLess(abs(p['dd_phase_residual_mm']), 15.0)

    def test_high_ratio_fixed_solution(self):
        res = process_double_differences(self.test_dir)
        summary = res['double_difference_summary']
        self.assertGreater(summary['ambiguity_fix_rate_pct'], 50.0)

if __name__ == '__main__':
    unittest.main()
