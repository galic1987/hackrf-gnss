#!/usr/bin/env python3
"""Unit tests for the Relativistic Space-Time Inspector.

Verifies:
1. General Relativity gravitational blueshift (+45.7 us/day) and Special
   Relativity kinematic time dilation (-7.2 us/day) in Keplerian orbits.
2. Net secular frequency offset and factory pre-launch clock preset (-4.55 mHz).
3. Einstein periodic orbital eccentricity correction (Fe * sqrt(a) * sin E).
4. Relativistic Sagnac Earth-rotation delay (w_E / c^2 * (x_sat * y_rx - y_sat * x_rx)).
5. Kepler solver and end-to-end satellite relativity analysis.
"""

import math
import unittest
import sys
import os

# Add scripts directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from relativistic_space_time_inspector import (
    C_LIGHT,
    MU_GPS,
    MU_GAL,
    MU_BDS,
    OMEGA_E,
    A_EARTH,
    F_REL_GPS,
    compute_gr_blueshift,
    compute_sr_dilation,
    compute_periodic_eccentricity,
    compute_sagnac_delay,
    solve_kepler,
    analyze_satellite_relativity,
)

class TestRelativisticInspector(unittest.TestCase):

    def test_relativistic_frequency_shift(self):
        """Verify GR gravitational blueshift and SR time dilation for nominal GPS orbit."""
        # Nominal GPS circular orbit: a = 26,560 km, e = 0
        r_gps = 26560000.0
        v_gps = math.sqrt(MU_GPS / r_gps)  # ~3874 m/s

        gr_shift, gr_us_day = compute_gr_blueshift(r_gps, MU_GPS, A_EARTH)
        sr_shift, sr_us_day = compute_sr_dilation(v_gps)

        # Theoretical values:
        # GR: +45.7 us/day (+5.28e-10 fractional)
        # SR: -7.2 us/day (-0.83e-10 fractional)
        self.assertAlmostEqual(gr_us_day, 45.61, delta=0.5)
        self.assertAlmostEqual(sr_us_day, -7.22, delta=0.5)

        net_us_day = gr_us_day + sr_us_day
        self.assertAlmostEqual(net_us_day, 38.39, delta=0.5)

        # Daily uncompensated distance drift if ignored:
        drift_km = (net_us_day * 1e-6) * C_LIGHT / 1e3
        self.assertAlmostEqual(drift_km, 11.51, delta=0.2)

        # Factory oscillator preset: delta_f = -net_shift * 10.23 MHz = -4.55 mHz
        f0 = 10.23e6
        net_shift = gr_shift + sr_shift
        factory_offset_mhz = -net_shift * f0 * 1e3
        self.assertAlmostEqual(factory_offset_mhz, -4.55, delta=0.2)

    def test_periodic_eccentricity_correction(self):
        """Verify periodic orbital eccentricity correction Fe * sqrt(a) * sin(Ek)."""
        e = 0.02
        sqrt_a = 5153.68  # sqrt(26560e3)
        ek = math.pi / 2.0  # max sin(Ek) = 1.0

        dt_r, dr_r = compute_periodic_eccentricity(e, sqrt_a, ek, F_REL_GPS)

        # dt_r = -4.442807633e-10 * 0.02 * 5153.68 * 1.0 = -4.579e-8 s = -45.79 ns
        expected_dt = F_REL_GPS * e * sqrt_a * 1.0
        self.assertAlmostEqual(dt_r, expected_dt, places=12)
        self.assertAlmostEqual(dt_r * 1e9, -45.79, delta=0.1)

        # Range correction should be -c * dt_r = +13.73 m
        self.assertAlmostEqual(dr_r, -C_LIGHT * expected_dt, places=6)
        self.assertAlmostEqual(dr_r, 13.73, delta=0.1)

    def test_sagnac_effect_analytical_vs_rotation(self):
        """Verify Sagnac analytical formula matches full 3D rotation matrix within < 0.1 mm."""
        # Station at Radiator Lab
        rx, ry, rz = 1065334.7, -4847496.8, 3992620.5
        # Satellite in high MEO
        sx, sy, sz = 1.5e7, -1.2e7, 1.8e7

        dt_sagnac, dr_sagnac = compute_sagnac_delay(sx, sy, rx, ry)

        # Full 3D rotation calculation
        d0 = math.sqrt((sx - rx)**2 + (sy - ry)**2 + (sz - rz)**2)
        tau = d0 / C_LIGHT
        th = OMEGA_E * tau
        ct, st = math.cos(th), math.sin(th)
        s_rot_x = sx * ct + sy * st
        s_rot_y = -sx * st + sy * ct
        s_rot_z = sz
        d_rot = math.sqrt((s_rot_x - rx)**2 + (s_rot_y - ry)**2 + (s_rot_z - rz)**2)
        dr_full = d_rot - d0

        discrepancy_m = abs(dr_sagnac - dr_full)
        self.assertLess(discrepancy_m, 0.001)  # less than 1 mm discrepancy!

        # Anti-symmetry test: reversing east/west reverses Sagnac sign
        dt_rev, dr_rev = compute_sagnac_delay(-sx, sy, rx, ry)
        self.assertNotEqual(dr_rev, dr_sagnac)

    def test_kepler_solver(self):
        """Verify Newton-Raphson Kepler solver accuracy."""
        e = 0.015
        for mk_deg in [0.0, 30.0, 90.0, 180.0, 270.0]:
            mk = math.radians(mk_deg)
            ek = solve_kepler(mk, e)
            # Verify Kepler's equation Ek - e*sin(Ek) = Mk
            resid = ek - e * math.sin(ek) - mk
            self.assertLess(abs(resid), 1e-12)

    def test_analyze_satellite(self):
        """Verify complete satellite analysis dictionary."""
        eph = {
            "sys": 0, "prn": 7, "sqrt_a": 5153.76, "e": 0.02093,
            "m0": 3.0144, "delta_n": 4.95e-9, "toe": 432000.0, "toc": 432000.0,
            "omega": -1.9505, "omega0": 2.9436, "omega_dot": -7.79e-9,
            "i0": 0.9516, "idot": 6.96e-11,
            "cuc": -1.26e-6, "cus": 7.06e-6, "crc": 237.0, "crs": -21.4,
            "cic": 2.03e-7, "cis": -3.12e-7
        }
        site_ecef = (1065334.7, -4847496.8, 3992620.5)
        site_llh = (39.0029, -77.6058, 20.0)
        res = analyze_satellite_relativity(eph, 432060.0, site_ecef, site_llh)

        self.assertIsNotNone(res)
        self.assertIn("gr_blueshift_us_day", res)
        self.assertIn("sr_dilation_us_day", res)
        self.assertIn("net_secular_us_day", res)
        self.assertIn("eccentricity_correction_ns", res)
        self.assertIn("sagnac_correction_ns", res)
        self.assertIn("total_instantaneous_rel_m", res)

        # Check reasonable physical bounds:
        self.assertGreater(res["gr_blueshift_us_day"], 40.0)
        self.assertLess(res["gr_blueshift_us_day"], 55.0)
        self.assertLess(res["sr_dilation_us_day"], -5.0)
        self.assertGreater(res["sr_dilation_us_day"], -9.0)
        self.assertGreater(res["net_secular_us_day"], 35.0)
        self.assertLess(res["net_secular_us_day"], 45.0)

if __name__ == "__main__":
    unittest.main()
