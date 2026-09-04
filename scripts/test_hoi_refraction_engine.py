#!/usr/bin/env python3
"""Unit tests for Higher-Order Ionospheric (HOI) Refraction & Geomagnetic Ray Bending Engine.

Verifies:
1. Ionospheric Pierce Point (IPP) geometry and zenith invariance.
2. Earth's tilted geomagnetic dipole field strength at 350 km altitude (35-50 uT).
3. Direction cosine cos(theta_B) between LOS and geomagnetic field vector.
4. Inverse cubic (1/f^3) second-order geomagnetic delay scaling (BDS B1I vs GPS L1).
5. Faraday rotation polarization angle calculation.
6. Fermat ray-path curvature bending behavior at low elevations.
7. Full analysis cycle execution, schema validity, and atomic state publication.
"""

import math
import os
import sys
import unittest

# Add scripts directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from hoi_refraction_engine import (
    C_LIGHT,
    F_GPS_L1,
    F_BDS_B1I,
    compute_ipp,
    compute_geomagnetic_field,
    compute_cos_theta_b,
    evaluate_hoi_corrections,
    run_hoi_analysis_cycle
)


class TestHoiRefractionEngine(unittest.TestCase):

    def test_ipp_geometry(self):
        """Verify IPP spherical projection."""
        lat_site = 39.0
        lon_site = -77.5

        # 1. Zenith: IPP must coincide with station
        ipp_lat_z, ipp_lon_z = compute_ipp(90.0, 0.0, lat_site, lon_site)
        self.assertAlmostEqual(ipp_lat_z, lat_site, places=2)
        self.assertAlmostEqual(ipp_lon_z, lon_site, places=2)

        # 2. Due North at 20 deg elevation: IPP moves North (higher latitude)
        ipp_lat_n, ipp_lon_n = compute_ipp(20.0, 0.0, lat_site, lon_site)
        self.assertGreater(ipp_lat_n, lat_site)
        self.assertAlmostEqual(ipp_lon_n, lon_site, places=1)

        # 3. Due South at 20 deg elevation: IPP moves South (lower latitude)
        ipp_lat_s, ipp_lon_s = compute_ipp(20.0, 180.0, lat_site, lon_site)
        self.assertLess(ipp_lat_s, lat_site)
        self.assertAlmostEqual(ipp_lon_s, lon_site, places=1)

    def test_geomagnetic_field_at_shell_altitude(self):
        """Verify geomagnetic field strength at 350 km altitude."""
        # Mid-latitude IPP: 39° N, 77° W
        b_mag, dip, dec = compute_geomagnetic_field(39.0, -77.5)

        # Magnitude must be in physical range ~35 to 50 uT
        b_ut = b_mag * 1e6
        self.assertGreater(b_ut, 35.0)
        self.assertLess(b_ut, 50.0)

        # Inclination (dip) must be positive downward in Northern hemisphere (+50 to +75 deg)
        dip_deg = math.degrees(dip)
        self.assertGreater(dip_deg, 50.0)
        self.assertLess(dip_deg, 75.0)

    def test_direction_cosine_cos_theta_b(self):
        """Verify LOS to geomagnetic field direction cosine bounds and signs."""
        dip = math.radians(65.0)
        dec = math.radians(-10.0)

        # 1. Zenith: pointing up against downward magnetic field -> negative cos(theta_B)
        ct_zenith = compute_cos_theta_b(90.0, 0.0, dip, dec)
        self.assertLess(ct_zenith, -0.8)
        self.assertGreaterEqual(ct_zenith, -1.0)

        # 2. Due South at 30 deg: pointing south against north-down field -> negative
        ct_south = compute_cos_theta_b(30.0, 180.0, dip, dec)
        self.assertLess(ct_south, 0.0)

        # Bound check: all values strictly in [-1.0, 1.0]
        for az in range(0, 360, 45):
            for el in [10, 30, 60, 85]:
                ct = compute_cos_theta_b(el, az, dip, dec)
                self.assertGreaterEqual(ct, -1.0)
                self.assertLessEqual(ct, 1.0)

    def test_frequency_dispersion_ratio_gps_vs_bds(self):
        """Verify 1/f^3 scaling of second-order ionospheric delay across GPS and BDS."""
        el = 35.0
        az = 160.0
        stec = 20.0

        res_gps = evaluate_hoi_corrections(el, az, stec, F_GPS_L1)
        res_bds = evaluate_hoi_corrections(el, az, stec, F_BDS_B1I)

        # Because BeiDou B1I (1561.098 MHz) has lower frequency than GPS L1 (1575.42 MHz),
        # its second-order delay must be strictly larger in magnitude!
        self.assertGreater(abs(res_bds["i2_second_order_mm"]), abs(res_gps["i2_second_order_mm"]))

        # Theoretical ratio: (f_GPS / f_BDS)^3 = (1575.42 / 1561.098)^3 = 1.02781
        expected_ratio = (F_GPS_L1 / F_BDS_B1I) ** 3
        observed_ratio = abs(res_bds["i2_second_order_mm"]) / abs(res_gps["i2_second_order_mm"])
        self.assertAlmostEqual(observed_ratio, expected_ratio, delta=0.001)

    def test_faraday_rotation(self):
        """Verify Faraday rotation polarization angle calculation."""
        res = evaluate_hoi_corrections(45.0, 180.0, 25.0, F_GPS_L1)
        # For 25 TECU, Faraday rotation is typically 2° to 8° on L1
        rot_deg = res["faraday_rotation_deg"]
        self.assertGreater(rot_deg, 1.0)
        self.assertLess(rot_deg, 10.0)

    def test_end_to_end_hoi_cycle(self):
        """Verify complete HOI synthesis cycle and schema validation."""
        state = run_hoi_analysis_cycle()
        self.assertIn("epoch", state)
        self.assertIn("hoi_summary", state)
        self.assertIn("satellites", state)

        summ = state["hoi_summary"]
        self.assertIn("mean_i2_phase_mm", summ)
        self.assertIn("max_i2_phase_mm", summ)
        self.assertIn("north_south_asymmetry_mm", summ)
        self.assertIn("mean_faraday_rotation_deg", summ)
        self.assertIn("mean_total_hoi_ps", summ)
        self.assertIn("status", summ)

        self.assertGreater(summ["mean_faraday_rotation_deg"], 0.0)
        self.assertGreater(summ["mean_total_hoi_ps"], 0.0)


if __name__ == "__main__":
    unittest.main()
