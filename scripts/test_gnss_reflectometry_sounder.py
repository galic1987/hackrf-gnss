#!/usr/bin/env python3
"""Unit tests for GNSS Interferometric Reflectometry (GNSS-R) & Chronometric Levelling Sounder."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from gnss_reflectometry_sounder import (
    compute_wavelength,
    compute_fresnel_ellipse,
    compute_fringe_frequency,
    invert_antenna_height,
    invert_soil_moisture_topp,
    compute_chronometric_levelling,
    LAMBDA_GPS_L1,
    LAMBDA_BDS_B1I
)

class TestGNSSReflectometrySounder(unittest.TestCase):

    def test_wavelength_selection(self):
        """Verify carrier wavelengths for GPS L1 vs BeiDou B1I."""
        self.assertAlmostEqual(compute_wavelength("gps"), LAMBDA_GPS_L1, places=5)
        self.assertAlmostEqual(compute_wavelength("beidou"), LAMBDA_BDS_B1I, places=5)
        self.assertAlmostEqual(compute_wavelength("B1I_C03"), LAMBDA_BDS_B1I, places=5)

    def test_fresnel_ellipse_geometry(self):
        """Verify First Fresnel Zone geometry at low vs high elevation."""
        # At 10 deg elevation, ellipse should be elongated along azimuth
        f_low = compute_fresnel_ellipse(1.842, 10.0, 90.0, LAMBDA_GPS_L1)
        self.assertGreater(f_low["semi_major_m"], f_low["semi_minor_m"])
        self.assertGreater(f_low["area_m2"], 10.0)
        # Check coordinates: Azimuth 90 deg (East) -> East > 0, North ~ 0
        self.assertGreater(f_low["center_east_m"], 5.0)
        self.assertAlmostEqual(f_low["center_north_m"], 0.0, delta=0.5)

        # At 60 deg elevation, ellipse should be much smaller and more circular
        f_high = compute_fresnel_ellipse(1.842, 60.0, 90.0, LAMBDA_GPS_L1)
        self.assertLess(f_high["semi_major_m"], f_low["semi_major_m"])
        self.assertLess(f_high["area_m2"], f_low["area_m2"])

    def test_fringe_frequency_roundtrip(self):
        """Verify that fringe frequency inverts to original antenna height."""
        h_true = 2.450 # meters
        f_x = compute_fringe_frequency(h_true, LAMBDA_GPS_L1)
        h_inverted = invert_antenna_height(f_x, LAMBDA_GPS_L1)
        self.assertAlmostEqual(h_true, h_inverted, places=6)

    def test_topp_soil_moisture_inversion(self):
        """Verify Topp's equation output across soil dielectric domain."""
        # Dry soil: eps ~ 4 -> VSM ~ 0.05 - 0.08
        vsm_dry = invert_soil_moisture_topp(4.0)
        self.assertGreaterEqual(vsm_dry, 0.02)
        self.assertLess(vsm_dry, 0.12)

        # Damp soil: eps ~ 12 -> VSM ~ 0.20 - 0.25
        vsm_damp = invert_soil_moisture_topp(12.0)
        self.assertGreater(vsm_damp, vsm_dry)

        # Saturated soil: eps ~ 25 -> VSM ~ 0.35 - 0.45
        vsm_sat = invert_soil_moisture_topp(25.0)
        self.assertGreater(vsm_sat, vsm_damp)
        self.assertLessEqual(vsm_sat, 0.55)

    def test_chronometric_levelling(self):
        """Verify general relativistic gravitational dilation."""
        # Station at 20 m AMSL + 1.842 m antenna
        c = compute_chronometric_levelling(20.0, 1.842)
        # Expected fractional shift ~ g * H / c^2 ~ 9.801 * 21.842 / (3e8)^2 ~ 2.38e-15
        self.assertAlmostEqual(c["chronometric_rate_fs_per_s"], 2.38, delta=0.05)
        # Drift per day ~ 2.38e-15 * 86400 * 1e9 ~ 0.205 ns/day
        self.assertAlmostEqual(c["chronometric_drift_ns_per_day"], 0.2058, delta=0.01)
        # Geopotential number = g * H ~ 214 m^2/s^2
        self.assertAlmostEqual(c["geopotential_number_m2_s2"], 214.07, delta=1.0)

if __name__ == "__main__":
    unittest.main()
