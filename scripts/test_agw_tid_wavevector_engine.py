#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/test_agw_tid_wavevector_engine.py
=========================================
Unit tests for AGW 2D Dispersion & TID Wavevector Inversion Engine.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from agw_tid_wavevector_engine import (
    compute_ipp,
    evaluate_hines_dispersion,
    run_agw_engine,
    STATION_LAT_DEG,
    STATION_LON_DEG,
    STATE_FILE
)

class TestAGWTIDWavevector(unittest.TestCase):
    def test_compute_ipp(self):
        # Zenith satellite: IPP directly overhead
        lat_ipp, lon_ipp, x_m, y_m = compute_ipp(STATION_LAT_DEG, STATION_LON_DEG, az_deg=0.0, el_deg=90.0)
        self.assertAlmostEqual(lat_ipp, STATION_LAT_DEG, delta=0.01)
        self.assertAlmostEqual(lon_ipp, STATION_LON_DEG, delta=0.01)
        self.assertAlmostEqual(x_m, 0.0, delta=100.0)
        self.assertAlmostEqual(y_m, 0.0, delta=100.0)

        # Low-elevation satellite: IPP displaced by hundreds of km
        lat_ipp_low, lon_ipp_low, x_low, y_low = compute_ipp(STATION_LAT_DEG, STATION_LON_DEG, az_deg=90.0, el_deg=20.0)
        self.assertGreater(x_low, 300000.0)  # > 300 km east

    def test_hines_dispersion(self):
        omega = 7.757e-3  # rad/s (~13.5 min)
        k_h = 4.19e-5     # rad/m (~150 km wavelength)
        disp = evaluate_hines_dispersion(omega, k_h)
        self.assertIn("acoustic_cutoff_omega_a_mrad_s", disp)
        self.assertIn("buoyancy_omega_b_mrad_s", disp)
        self.assertIn("vertical_propagation_mode", disp)

    def test_run_agw_engine(self):
        res = run_agw_engine()
        self.assertTrue(os.path.exists(STATE_FILE))
        self.assertIn("tid_wavevector_summary", res)
        self.assertIn("hines_atmospheric_gravity_wave_physics", res)
        self.assertIn("ionospheric_pierce_points_350km", res)

        t_sum = res["tid_wavevector_summary"]
        self.assertIn("horizontal_phase_speed_m_s", t_sum)
        self.assertGreater(t_sum["horizontal_phase_speed_m_s"], 50.0)
        self.assertLess(t_sum["horizontal_phase_speed_m_s"], 1000.0)

if __name__ == "__main__":
    unittest.main()
