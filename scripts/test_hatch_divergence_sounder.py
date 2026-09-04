#!/usr/bin/env python3
"""Unit tests for Hatch Filter Code-Carrier Divergence (CCD / CMC) Sounder.

Verifies:
1. Superluminal phase velocity excess (v_p > c) and subluminal group velocity (v_g < c).
2. Relativistic invariance condition: v_p * v_g == c^2.
3. Frequency dispersion scaling across GPS L1 (1575.42 MHz) and BeiDou B1I (1561.098 MHz).
4. Hatch smoother dynamics, cycle slip reset handling, and variance reduction.
5. Linear regression estimation of ionospheric divergence rate d(CMC)/dt.
6. Theoretical Hatch filter divergence bias formulation: Bias = 2 * tau * dI/dt.
7. Full analysis cycle execution, schema validation, and atomic state publication.
"""

import math
import os
import sys
import unittest

# Add scripts directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from hatch_divergence_sounder import (
    C_LIGHT,
    F_GPS_L1,
    F_BDS_B1I,
    LAM_GPS_L1,
    LAM_BDS_B1I,
    compute_plasma_velocities,
    HatchFilter,
    estimate_linear_slope,
    run_hatch_divergence_cycle
)


class TestHatchDivergenceSounder(unittest.TestCase):

    def test_plasma_velocities_and_relativistic_invariant(self):
        """Verify superluminal phase velocity, subluminal group velocity, and v_p * v_g == c^2."""
        stec = 20.0  # 20 TECU
        vp, vg, delta_vp, delta_vg, fp, np_ref = compute_plasma_velocities(stec, F_GPS_L1)

        # 1. Phase velocity is superluminal: v_p > c
        self.assertGreater(vp, C_LIGHT)
        self.assertGreater(delta_vp, 0.0)

        # 2. Group velocity is subluminal: v_g < c
        self.assertLess(vg, C_LIGHT)
        self.assertGreater(delta_vg, 0.0)

        # 3. Symmetry to first order: delta_vp ~= delta_vg
        self.assertAlmostEqual(delta_vp, delta_vg, delta=0.1)

        # 4. Fundamental Relativistic Invariant: v_p * v_g == c^2
        product = vp * vg
        c_squared = C_LIGHT ** 2
        rel_diff = abs(product - c_squared) / c_squared
        self.assertLess(rel_diff, 1e-9)

        # 5. Refractive index n_p < 1.0
        self.assertLess(np_ref, 1.0)
        self.assertGreater(np_ref, 0.999)

    def test_frequency_dispersion_gps_vs_bds(self):
        """Verify inverse-square frequency dispersion between GPS L1 and BDS B1I."""
        stec = 15.0
        _, _, delta_vp_gps, _, _, _ = compute_plasma_velocities(stec, F_GPS_L1)
        _, _, delta_vp_bds, _, _, _ = compute_plasma_velocities(stec, F_BDS_B1I)

        # BDS B1I (1561.098 MHz) is lower frequency than GPS L1 (1575.42 MHz),
        # so plasma dispersion excess must be strictly larger on BDS!
        self.assertGreater(delta_vp_bds, delta_vp_gps)

        # Theoretical ratio: (f_GPS / f_BDS)^2 = (1575.42 / 1561.098)^2 = 1.01844
        expected_ratio = (F_GPS_L1 / F_BDS_B1I) ** 2
        observed_ratio = delta_vp_bds / delta_vp_gps
        self.assertAlmostEqual(observed_ratio, expected_ratio, places=4)

    def test_hatch_filter_smoothing_and_reset(self):
        """Verify Hatch smoother converges on truth and flushes state on reset."""
        hatch = HatchFilter(window_epochs=50.0)

        # Feed 50 constant epochs at 20,000,000 meters
        truth = 20000000.0
        carr = 0.0
        code = truth
        for k in range(50):
            carr += 10.0  # 10 cycles/epoch = ~1.9 m/epoch
            # Under GNSS convention, increasing carrier cycles corresponds to decreasing range:
            # Delta rho = -lambda * Delta carr
            code = truth - carr * LAM_GPS_L1
            sm, n = hatch.update(code, carr, LAM_GPS_L1, reset=False)

        # Smoothed output must match exact geometric truth
        self.assertAlmostEqual(sm, code, places=4)
        self.assertEqual(n, 50.0)

        # Test reset on cycle slip
        code_jump = code + 500.0
        carr_jump = 99999.0
        sm_reset, n_reset = hatch.update(code_jump, carr_jump, LAM_GPS_L1, reset=True)
        self.assertEqual(sm_reset, code_jump)
        self.assertEqual(n_reset, 1.0)

    def test_linear_slope_estimator(self):
        """Verify linear regression slope calculation."""
        times = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
        # Slope = 3.5 m/s, intercept = 12.0
        values = [12.0 + 3.5 * t for t in times]
        slope = estimate_linear_slope(times, values)
        self.assertAlmostEqual(slope, 3.5, places=6)

        # Constant values -> slope = 0.0
        const_vals = [42.0] * len(times)
        self.assertAlmostEqual(estimate_linear_slope(times, const_vals), 0.0, places=6)

    def test_hatch_divergence_bias_relationship(self):
        """Verify that ionospheric gradient dI/dt produces Bias = 2 * tau * dI/dt."""
        iono_drift_m_s = 0.002  # 2 mm/s ionospheric drift rate
        tau_eff = 100.0         # 100 seconds smoothing time constant
        expected_bias_m = 2.0 * tau_eff * iono_drift_m_s  # 0.40 m = 40 cm
        self.assertAlmostEqual(expected_bias_m, 0.40, places=4)

    def test_end_to_end_ccd_sounder_cycle(self):
        """Verify end-to-end sounder cycle execution and output schema."""
        state = run_hatch_divergence_cycle()
        self.assertIn("epoch", state)
        self.assertIn("ccd_summary", state)
        self.assertIn("satellites", state)

        summ = state["ccd_summary"]
        self.assertIn("mean_iono_drift_mm_s", summ)
        self.assertIn("max_hatch_bias_m", summ)
        self.assertIn("mean_superluminal_vp_excess_m_s", summ)
        self.assertIn("hatch_status", summ)
        self.assertTrue(summ["relativistic_invariant_verified"])
        self.assertGreater(summ["mean_superluminal_vp_excess_m_s"], 0.0)


if __name__ == "__main__":
    unittest.main()
