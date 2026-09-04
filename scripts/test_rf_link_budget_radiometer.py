#!/usr/bin/env python3
"""Unit tests for RF Radiometry & Link Budget Metrology.

Verifies:
1. Free Space Path Loss (FSPL) calculations across GNSS carrier frequencies.
2. ITU-R P.676 atmospheric absorption model scaling with elevation.
3. Thermal noise spectral density (N0) and system noise temperature (Tsys).
4. Link margin evaluation relative to tracking thresholds.
5. RFI / Jamming threat classification based on noise floor elevation.
6. Complete radiometry state synthesis and schema conformity.
"""

import math
import unittest
import sys
import os

# Add scripts directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rf_link_budget_radiometer import (
    C_LIGHT,
    KB_BOLTZMANN,
    F_L1_GPS,
    F_B1I_BDS,
    compute_fspl_db,
    compute_atm_absorption_db,
    compute_noise_spectral_density_dbm_hz,
    compute_tsys_from_n0,
    compute_link_margin_db,
    classify_rfi_threat,
    evaluate_radiometry,
)

class TestRfLinkBudgetRadiometer(unittest.TestCase):

    def test_free_space_path_loss(self):
        """Verify FSPL calculations for GPS L1 and BDS B1I."""
        # Zenith GPS: d = 20,200 km, f = 1575.42 MHz
        d_zenith = 20200e3
        fspl_l1 = compute_fspl_db(d_zenith, F_L1_GPS)
        # Expected: ~182.5 dB
        self.assertAlmostEqual(fspl_l1, 182.51, delta=0.5)

        # Low-elevation GPS: d = 25,500 km
        d_low = 25500e3
        fspl_low = compute_fspl_db(d_low, F_L1_GPS)
        self.assertAlmostEqual(fspl_low, 184.54, delta=0.5)

        # BDS B1I (1561.098 MHz): should be slightly lower path loss due to lower frequency
        fspl_bds = compute_fspl_db(d_zenith, F_B1I_BDS)
        self.assertLess(fspl_bds, fspl_l1)
        self.assertAlmostEqual(fspl_l1 - fspl_bds, 0.08, delta=0.03)

    def test_atmospheric_absorption(self):
        """Verify ITU-R atmospheric absorption scaling with elevation."""
        # Zenith (90 deg): ~0.04 dB
        a_zenith = compute_atm_absorption_db(90.0)
        self.assertAlmostEqual(a_zenith, 0.04, delta=0.01)

        # 30 deg: ~0.08 dB
        a_30 = compute_atm_absorption_db(30.0)
        self.assertAlmostEqual(a_30, 0.08, delta=0.02)

        # 10 deg: ~0.23 dB
        a_10 = compute_atm_absorption_db(10.0)
        self.assertAlmostEqual(a_10, 0.23, delta=0.03)

    def test_system_noise_temperature(self):
        """Verify Boltzmann noise power density N0 and Tsys."""
        t_sys = 195.0  # Kelvin
        n0_dbm_hz = compute_noise_spectral_density_dbm_hz(t_sys)
        # -174 + 10*log10(195/290) = -175.72 dBm/Hz
        self.assertAlmostEqual(n0_dbm_hz, -175.72, delta=0.1)

        # Inversion: compute Tsys from N0
        inverted_tsys = compute_tsys_from_n0(n0_dbm_hz)
        self.assertAlmostEqual(inverted_tsys, t_sys, delta=0.5)

    def test_link_margin(self):
        """Verify link margin above receiver tracking threshold."""
        cn0 = 44.0  # dB-Hz
        threshold = 28.0  # dB-Hz
        margin = compute_link_margin_db(cn0, threshold)
        self.assertEqual(margin, 16.0)

    def test_rfi_threat_classification(self):
        """Verify RFI threat level classification based on Tsys."""
        self.assertEqual(classify_rfi_threat(250.0), "QUIET")
        self.assertEqual(classify_rfi_threat(420.0), "QUIET")
        self.assertEqual(classify_rfi_threat(550.0), "ELEVATED_NOISE")
        self.assertEqual(classify_rfi_threat(900.0), "RFI_INTERFERENCE")

    def test_full_radiometry_evaluation(self):
        """Verify full radiometry state evaluation with mock data."""
        tracked_sats = [
            {"sys": "gps", "prn": 4, "el_deg": 65.4, "az_deg": 224.5, "cn0_proxy": 44.1, "lock_s": 120.0},
            {"sys": "gps", "prn": 7, "el_deg": 25.8, "az_deg": 302.1, "cn0_proxy": 35.0, "lock_s": 80.0},
            {"sys": "beidou", "prn": 31, "el_deg": 24.9, "az_deg": 148.2, "cn0_proxy": 37.1, "lock_s": 90.0},
        ]
        res = evaluate_radiometry(tracked_sats)
        self.assertIn("radiometry_summary", res)
        self.assertIn("radiometry_satellites", res)

        summ = res["radiometry_summary"]
        self.assertGreater(summ["mean_cn0_db_hz"], 35.0)
        self.assertGreater(summ["mean_link_margin_db"], 7.0)
        self.assertGreater(summ["mean_fspl_db"], 182.0)
        self.assertEqual(summ["rfi_threat_level"], "QUIET")

        sats = res["radiometry_satellites"]
        self.assertIn("GPS_4", sats)
        self.assertGreater(sats["GPS_4"]["link_margin_db"], 10.0)

if __name__ == "__main__":
    unittest.main()
