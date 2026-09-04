#!/usr/bin/env python3
"""Unit tests for carrier_single_difference_engine.py."""

import json
import os
import tempfile
import unittest

from carrier_single_difference_engine import process_single_differences

class TestCarrierSingleDifferenceEngine(unittest.TestCase):
    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.klob_path = os.path.join(self.test_dir, 'state.klobuchar.json')
        self.tropo_path = os.path.join(self.test_dir, 'state.tropo.json')

        klob_data = {
            'satellites': {
                'G14': {'el_deg': 65.0, 'az_deg': 120.0, 'cn0': 42.0, 'klobuchar_delay_m': 2.15},
                'G03': {'el_deg': 25.0, 'az_deg': 45.0, 'cn0': 34.0, 'klobuchar_delay_m': 4.80},
                'G17': {'el_deg': 15.0, 'az_deg': 280.0, 'cn0': 30.0, 'klobuchar_delay_m': 7.20}
            }
        }
        tropo_data = {
            'tropo_satellites': {
                'G14': {'tropo_delay_m': 2.55},
                'G03': {'tropo_delay_m': 5.46},
                'G17': {'tropo_delay_m': 8.92}
            }
        }

        with open(self.klob_path, 'w') as f:
            json.dump(klob_data, f)
        with open(self.tropo_path, 'w') as f:
            json.dump(tropo_data, f)

    def test_pivot_selection(self):
        res = process_single_differences(self.test_dir)
        summary = res['single_difference_summary']
        self.assertEqual(summary['pivot_satellite'], 'G14')
        self.assertEqual(summary['pivot_elevation_deg'], 65.0)
        self.assertEqual(summary['n_differenced_pairs'], 2)

    def test_exact_receiver_clock_cancellation(self):
        res = process_single_differences(self.test_dir)
        pairs = res['sd_pairs']
        self.assertIn('G03_vs_G14', pairs)
        self.assertIn('G17_vs_G14', pairs)

        for pair_name, p in pairs.items():
            self.assertEqual(p['receiver_clock_bias_m'], 0.000)
            self.assertEqual(p['receiver_clock_bias_status'], 'EXACT_CANCELED_0_NS')

    def test_differential_atmospheric_delays(self):
        res = process_single_differences(self.test_dir)
        p_g03 = res['sd_pairs']['G03_vs_G14']
        self.assertAlmostEqual(p_g03['delta_iono_m'], 2.65, places=2)
        self.assertAlmostEqual(p_g03['delta_tropo_m'], 2.91, places=2)

    def test_noise_floor_scaling(self):
        res = process_single_differences(self.test_dir)
        g03_sigma = res['sd_pairs']['G03_vs_G14']['sd_noise_floor_sigma_mm']
        g17_sigma = res['sd_pairs']['G17_vs_G14']['sd_noise_floor_sigma_mm']
        self.assertGreater(g17_sigma, g03_sigma)

if __name__ == '__main__':
    unittest.main()
