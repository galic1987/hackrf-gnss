#!/usr/bin/env python3
"""Unit tests for post_sunrise_flux_tracker.py."""

import json
import os
import tempfile
import unittest

from post_sunrise_flux_tracker import process_solar_ramp, kasten_young_airmass

class TestPostSunriseFluxTracker(unittest.TestCase):
    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.solar_path = os.path.join(self.test_dir, 'state.solar.json')
        solar_data = {
            'solar_ephemeris': {
                'solar_el_apparent_deg': 15.0,
                'solar_az_deg': 92.0
            },
            'twilight_milestones': {
                'ground_sunrise_utc': '10:40:32'
            }
        }
        with open(self.solar_path, 'w') as f:
            json.dump(solar_data, f)

    def test_airmass_calculation(self):
        am_zenith = kasten_young_airmass(90.0)
        self.assertAlmostEqual(am_zenith, 1.0, places=2)
        am_15 = kasten_young_airmass(15.0)
        self.assertGreater(am_15, 3.5)
        self.assertLess(am_15, 4.5)

    def test_solar_ramp_generation(self):
        res = process_solar_ramp(self.test_dir)
        sm = res['solar_ramp_summary']
        self.assertEqual(sm['solar_elevation_deg'], 15.0)
        self.assertGreater(sm['direct_normal_irradiance_w_m2'], 0.0)
        self.assertGreater(sm['global_horizontal_irradiance_w_m2'], 0.0)
        self.assertGreater(sm['dtec_dt_tecu_per_hr'], 0.0)
        self.assertGreater(sm['dtemp_dt_c_per_hr'], 0.0)

if __name__ == '__main__':
    unittest.main()
