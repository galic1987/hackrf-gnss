#!/usr/bin/env python3
"""Unit tests for Solar Terminator & Dawn Detection Engine."""

import datetime
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from solar_dawn_detector import (
    compute_solar_ephemeris,
    compute_layer_illumination,
    compute_overhead_shadow_height_km,
    compute_twilight_milestones,
    compute_satellite_sun_separation,
    classify_dawn_state,
    RE_KM
)

class TestSolarDawnDetector(unittest.TestCase):

    def setUp(self):
        self.lat = 39.0029
        self.lon = -77.6058
        self.alt = 20.0

    def test_solar_ephemeris_sanity(self):
        """Verify solar ephemeris calculations on vernal equinox at solar noon."""
        # March 20 ~12:00 UTC at 0 lon, 0 lat -> Sun should be near zenith
        dt_equinox = datetime.datetime(2026, 3, 20, 12, 0, 0, tzinfo=datetime.timezone.utc)
        res = compute_solar_ephemeris(0.0, 0.0, 0.0, dt_equinox)
        self.assertAlmostEqual(res["declination_deg"], 0.0, delta=1.5)
        self.assertGreater(res["solar_el_geom_deg"], 85.0)
        self.assertAlmostEqual(res["solar_distance_au"], 1.0, delta=0.03)

    def test_layer_illumination_geometry(self):
        """Verify layer horizon dip angle calculation."""
        # Ground should have 0 dip
        ground = compute_layer_illumination(0.0, 0.0)
        self.assertEqual(ground["horizon_dip_deg"], 0.0)
        self.assertFalse(ground["is_illuminated"])

        # 350 km shell should have dip = arccos(6371 / 6721) ~ 18.57 deg
        f2 = compute_layer_illumination(-10.0, 350.0)
        expected_dip = math.degrees(math.acos(6371.0 / 6721.0))
        self.assertAlmostEqual(f2["horizon_dip_deg"], expected_dip, delta=0.05)
        # When sun is at -10 deg, F2 layer (+18.57 deg dip) should be ILLUMINATED
        self.assertTrue(f2["is_illuminated"])
        self.assertEqual(f2["status"], "ILLUMINATED (DAYLIGHT)")

    def test_shadow_height_calculation(self):
        """Verify overhead shadow height."""
        # Sun above horizon -> shadow height is 0
        self.assertEqual(compute_overhead_shadow_height_km(5.0), 0.0)
        # Sun at -6 deg (civil dawn) -> shadow height h = Re * (1/cos(6 deg) - 1) ~ 35 km
        h_civil = compute_overhead_shadow_height_km(-6.0)
        self.assertAlmostEqual(h_civil, 35.1, delta=1.0)
        # Sun at -18 deg (astronomical dawn) -> h ~ 327 km
        h_astro = compute_overhead_shadow_height_km(-18.0)
        self.assertAlmostEqual(h_astro, 327.0, delta=5.0)

    def test_twilight_milestones_sequence(self):
        """Verify chronologic ordering of dawn milestones."""
        dt = datetime.datetime(2026, 9, 4, 10, 0, 0, tzinfo=datetime.timezone.utc)
        m = compute_twilight_milestones(self.lat, self.lon, dt)
        
        # Chronological order of UTC times:
        # iono_sunrise <= astronomical <= nautical <= civil <= ground_sunrise
        times = [
            m["iono_sunrise_350km_utc"],
            m["astronomical_dawn_utc"],
            m["nautical_dawn_utc"],
            m["civil_dawn_utc"],
            m["ground_sunrise_utc"]
        ]
        for t in times:
            self.assertIsNotNone(t)
        self.assertLessEqual(times[0], times[1])
        self.assertLessEqual(times[1], times[2])
        self.assertLessEqual(times[2], times[3])
        self.assertLessEqual(times[3], times[4])

    def test_satellite_sun_separation(self):
        """Verify angular separation between satellite and Sun."""
        # Identical positions -> 0 deg
        sep_zero = compute_satellite_sun_separation(75.0, 10.0, 75.0, 10.0)
        self.assertAlmostEqual(sep_zero, 0.0, delta=0.01)

        # Opposite hemispheres -> 180 deg
        sep_opp = compute_satellite_sun_separation(0.0, 0.0, 180.0, 0.0)
        self.assertAlmostEqual(sep_opp, 180.0, delta=0.01)

        # Orthogonal -> 90 deg
        sep_ortho = compute_satellite_sun_separation(0.0, 0.0, 90.0, 0.0)
        self.assertAlmostEqual(sep_ortho, 90.0, delta=0.01)

    def test_dawn_state_classification(self):
        """Verify dawn state classifier across elevation domains."""
        code, _ = classify_dawn_state(-20.0, False)
        self.assertEqual(code, "DEEP_NIGHT")

        code, _ = classify_dawn_state(-15.0, True)
        self.assertEqual(code, "IONOSPHERIC_SUNRISE_ACTIVE")

        code, _ = classify_dawn_state(-8.0, True)
        self.assertEqual(code, "NAUTICAL_DAWN")

        code, _ = classify_dawn_state(-3.0, True)
        self.assertEqual(code, "CIVIL_DAWN")

        code, _ = classify_dawn_state(1.0, True)
        self.assertEqual(code, "GROUND_SUNRISE")

        code, _ = classify_dawn_state(15.0, True)
        self.assertEqual(code, "FULL_DAYLIGHT")

if __name__ == "__main__":
    unittest.main()
