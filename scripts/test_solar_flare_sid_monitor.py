#!/usr/bin/env python3
"""Unit tests for solar_flare_sid_monitor.py."""

import json
import os
import tempfile
import unittest

import solar_flare_sid_monitor as sid_monitor

class TestSolarFlareSidMonitor(unittest.TestCase):
    def setUp(self):
        sid_monitor.PREV_DTEC = None
        sid_monitor.PREV_EPOCH = None
        self.test_dir = tempfile.mkdtemp()
        self.solar_path = os.path.join(self.test_dir, 'state.solar.json')
        self.ramp_path = os.path.join(self.test_dir, 'state.solar_ramp.json')

        solar_data = {
            'solar_ephemeris': {
                'solar_el_apparent_deg': 36.0,
                'solar_az_deg': 112.0
            }
        }
        ramp_data = {
            'solar_ramp_summary': {
                'dtec_dt_tecu_per_hr': 1.05,
                'global_horizontal_irradiance_w_m2': 560.0
            }
        }
        with open(self.solar_path, 'w') as f: json.dump(solar_data, f)
        with open(self.ramp_path, 'w') as f: json.dump(ramp_data, f)

    def test_solar_flux_and_sid_calculation(self):
        res = sid_monitor.process_solar_flare_sid(self.test_dir)
        sm = res['solar_flare_sid_summary']
        self.assertEqual(sm['solar_elevation_deg'], 36.0)
        self.assertGreater(sm['f107_solar_flux_sfu'], 100.0)
        self.assertGreater(sm['delta_t_solar_k'], 0.0)
        self.assertEqual(sm['sid_event_classification'], 'QUIET_BACKGROUND')
        self.assertEqual(sm['sri_threat_level'], 'RF_QUIET')

if __name__ == '__main__':
    unittest.main()
