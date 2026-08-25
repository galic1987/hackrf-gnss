#!/usr/bin/env python3
"""Unit tests for phase_drift_producer. Plain asserts, no pytest.

Synthetic fixtures only — never touches live observations. Covers:
  - slope recovery + honest sigma from fit residuals
  - slip rejection (window flushes, slip sample dropped)
  - reseed zero-jump rejection (carrier_cycles -> ~0 without a slip flag)
  - discipline-step rejection (correction_ppm change flushes the window)
  - window with < N samples -> no fit
  - lock_s / cn0 gates
  - epoch rollover (non-monotonic epochs, long gaps)
  - consensus weighting (sigma-weighted median, outlier down-weighted)
  - missing carrier-phase fields (pre-dcfcfaa state) -> no rows, no crash
  - end-to-end synthetic GEO fixture through process_state, including
    the corr-register add-back convention

  python3 scripts/test_phase_drift_producer.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase_drift_producer as pd

FAILURES = []


def check(name, cond, detail=""):
    print(f"{'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILURES.append(name)


def gen(t0, n, slope_hz, phase0=0.0, noise_cyc=0.0, seed=1):
    """Deterministic (t, cycles) series at 1 Hz with optional noise."""
    out = []
    x = seed
    for i in range(n):
        # tiny LCG so tests need no numpy/random state
        x = (1103515245 * x + 12345) % (2**31)
        nz = noise_cyc * (2.0 * (x / 2**31) - 1.0)
        out.append((t0 + i, phase0 + slope_hz * i + nz))
    return out


def main():
    # --- fit_drift: slope recovery + sigma ---------------------------------
    s = gen(1000.0, 40, slope_hz=-123.456)
    slope, sigma, n = pd.fit_drift(s)
    check("fit: clean slope recovered", abs(slope + 123.456) < 1e-9,
          f"slope={slope}")
    check("fit: clean sigma ~ 0", sigma < 1e-9, f"sigma={sigma}")
    check("fit: n reported", n == 40)

    noisy = gen(1000.0, 40, slope_hz=50.0, noise_cyc=0.02)
    slope_n, sigma_n, _ = pd.fit_drift(noisy)
    check("fit: noisy slope still recovered", abs(slope_n - 50.0) < 0.01,
          f"slope={slope_n}")
    check("fit: noise raises sigma", sigma_n > sigma > 0 or sigma_n > 1e-6,
          f"sigma={sigma_n}")
    # analytic scale: sigma_slope ~ sigma_cyc * sqrt(12/(n*T^2))
    expect = 0.02 * math.sqrt(12.0 / (40 * 39.0**2)) * (40.0 / 39.0)
    check("fit: sigma in the expected band", sigma_n < 5 * expect + 1e-6,
          f"sigma={sigma_n:.2e} expect~{expect:.2e}")

    check("fit: <3 points -> None", pd.fit_drift(s[:2]) is None)
    check("fit: zero time span -> None",
          pd.fit_drift([(1.0, 0.0), (1.0, 1.0), (1.0, 2.0)]) is None)

    # --- weighted_median ----------------------------------------------------
    med, sig = pd.weighted_median([(1.0, 0.01), (1.01, 0.01), (5.0, 10.0)])
    check("wmedian: loose outlier down-weighted", abs(med - 1.0) < 0.02,
          f"med={med}")
    med2, sig2 = pd.weighted_median([(2.0, 0.1)])
    check("wmedian: single voter passes through", med2 == 2.0)
    check("wmedian: single voter sigma = its sigma", abs(sig2 - 0.1) < 1e-12,
          f"sig={sig2}")
    tight = [(1.0, 1e-6), (1.0 + 1e-4, 1e-6)]   # voters disagree >> sigmas
    med3, sig3 = pd.weighted_median(tight)
    check("wmedian: scatter keeps consensus honest", sig3 > 1e-5,
          f"sig={sig3}")

    # --- SatWindow: continuity guards ---------------------------------------
    w = pd.SatWindow(40.0)
    for t, c in gen(1000.0, 45, slope_hz=-100.0):
        w.add(t, c, slip=False, cn0=40.0, lock_s=t - 900.0,
              corr=0.5)
    ev = w.evaluate(min_lock_s=40.0, min_cn0=30.0, min_samples=28)
    check("window: clean chain fits", ev is not None and
          abs(ev["slope_hz"] + 100.0) < 1e-9, f"ev={ev}")

    # slip flushes the window and the slip sample joins no chain
    w2 = pd.SatWindow(40.0)
    for t, c in gen(1000.0, 40, slope_hz=-100.0):
        w2.add(t, c, slip=False, cn0=40.0, lock_s=500.0,
               corr=0.5)
    joined = w2.add(1040.0, 0.0, slip=True, cn0=40.0,
                    lock_s=0.0, corr=0.5)
    check("slip: sample rejected", joined is False)
    check("slip: window flushed", len(w2.samples) == 0)
    check("slip: no fit right after", w2.evaluate(40.0, 30.0, 28) is None)
    for t, c in gen(1041.0, 40, slope_hz=-100.0, phase0=0.0):
        w2.add(t, c, slip=False, cn0=40.0, lock_s=500.0,
               corr=0.5)
    check("slip: chain re-fits on the new origin",
          w2.evaluate(40.0, 30.0, 28) is not None)

    # reseed WITHOUT a slip flag: cycles jump to ~0 mid-chain
    w3 = pd.SatWindow(40.0)
    for t, c in gen(1000.0, 30, slope_hz=-100.0):
        w3.add(t, c, slip=False, cn0=40.0, lock_s=500.0,
               corr=0.5)
    joined = w3.add(1030.0, 0.0, slip=False, cn0=40.0,
                    lock_s=500.0, corr=0.5)
    check("reseed: zero-jump detected without slip flag", joined is False
          and len(w3.samples) == 0)

    # honest per-second increment noise (the live NCO wanders ~1 cycle/s)
    # must NOT trip the break detector
    w3b = pd.SatWindow(40.0)
    ok = True
    for t, c in gen(1000.0, 40, slope_hz=-100.0, noise_cyc=1.5):
        ok &= w3b.add(t, c, slip=False, cn0=40.0,
                      lock_s=500.0, corr=0.5)
    check("reseed: ~1 cyc/s increment noise tolerated", ok)

    # a mid-chain jump too small for the collapse detector but far above
    # increment noise is caught vs the chain's own median increment
    w3c = pd.SatWindow(40.0)
    for t, c in gen(1000.0, 20, slope_hz=-5.0):   # small accumulator
        w3c.add(t, c, slip=False, cn0=40.0, lock_s=500.0, corr=0.5)
    joined = w3c.add(1020.0, -5.0 * 20 - 100.0, slip=False, cn0=40.0,
                     lock_s=500.0, corr=0.5)
    check("reseed: -100-cycle step caught vs median increment",
          joined is False and len(w3c.samples) == 0)

    # discipline step (corr register change) flushes the window
    w4 = pd.SatWindow(40.0)
    for t, c in gen(1000.0, 30, slope_hz=-100.0):
        w4.add(t, c, slip=False, cn0=40.0, lock_s=500.0,
               corr=0.5)
    joined = w4.add(1030.0, -100.0 * 30, slip=False,
                    cn0=40.0, lock_s=500.0, corr=0.53)
    check("disc-step: corr change flushes window", joined is False
          and len(w4.samples) == 0)

    # epoch rollover / non-monotonic epochs and long gaps
    w5 = pd.SatWindow(40.0)
    for t, c in gen(1000.0, 30, slope_hz=-100.0):
        w5.add(t, c, slip=False, cn0=40.0, lock_s=500.0,
               corr=0.5)
    check("epoch: backwards epoch flushes",
          w5.add(999.0, -100.0 * 29, cn0=40.0, lock_s=500.0,
                 corr=0.5) is False and len(w5.samples) == 0)
    w6 = pd.SatWindow(40.0)
    for t, c in gen(1000.0, 30, slope_hz=-100.0):
        w6.add(t, c, slip=False, cn0=40.0, lock_s=500.0,
               corr=0.5)
    check("epoch: >5 s gap flushes",
          w6.add(1040.0, -100.0 * 40, cn0=40.0, lock_s=500.0,
                 corr=0.5) is False and len(w6.samples) == 0)

    # < N samples -> no fit
    w7 = pd.SatWindow(40.0)
    for t, c in gen(1000.0, 20, slope_hz=-100.0):
        w7.add(t, c, cn0=40.0, lock_s=500.0, corr=0.5)
    check("window: 20 samples < min 28 -> None",
          w7.evaluate(40.0, 30.0, 28) is None)

    # gates: young lock, weak cn0
    w8 = pd.SatWindow(40.0)
    for t, c in gen(1000.0, 45, slope_hz=-100.0):
        w8.add(t, c, cn0=40.0, lock_s=10.0, corr=0.5)
    check("gate: lock_s < window -> None", w8.evaluate(40.0, 30.0, 28) is None)
    w9 = pd.SatWindow(40.0)
    for i, (t, c) in enumerate(gen(1000.0, 45, slope_hz=-100.0)):
        w9.add(t, c, cn0=25.0 if i == 20 else 40.0,
               lock_s=500.0, corr=0.5)
    check("gate: one weak-cn0 second in window -> None",
          w9.evaluate(40.0, 30.0, 28) is None)

    # --- process_state end-to-end -------------------------------------------
    # true raw clock = -0.471 ppm; corr register = -0.451 -> measured
    # residual slope = (raw - corr) * L1 = -0.020 ppm of L1
    raw_ppm, corr = -0.471, -0.451
    resid_hz = (raw_ppm - corr) * 1e-6 * pd.L1_HZ
    windows = {}
    rows = []
    for i in range(45):
        state = {
            "epoch": 2000.0 + i,
            "discipline": {"correction_ppm": corr},
            "tracker": {"sats": [
                {"sys": "sbas", "prn": 133, "epoch": 2000.0 + i,
                 "carrier_cycles": resid_hz * i, "doppler_hz": resid_hz,
                 "cn0_proxy": 42.0, "lock_s": 400.0 + i, "slip": False},
                {"sys": "sbas", "prn": 135, "epoch": 2000.0 + i,
                 "carrier_cycles": resid_hz * i + 7e5, "slip": False,
                 "doppler_hz": resid_hz, "cn0_proxy": 38.0,
                 "lock_s": 400.0 + i},
                {"sys": "gps", "prn": 8, "epoch": 2000.0 + i,
                 "carrier_cycles": 1800.0 * i, "doppler_hz": 1800.0,
                 "cn0_proxy": 45.0, "lock_s": 400.0 + i, "slip": False},
            ]},
        }
        rows, diag = pd.process_state(state, windows, 40.0, 40.0, 30.0, 28)
    bands = [r["band"] for r in rows]
    check("e2e: per-sat rows + consensus", "L1 / WAAS 133 (phase)" in bands
          and "L1 / WAAS 135 (phase)" in bands and pd.MY_BAND in bands,
          f"bands={bands}")
    check("e2e: GPS (MEO) contributes no row",
          not any("8" in b for b in bands))
    per = {r["band"]: r for r in rows if r["band"] != pd.MY_BAND}
    # each per-sat row must recover the RAW clock (residual + corr back)
    for b in ("L1 / WAAS 133 (phase)", "L1 / WAAS 135 (phase)"):
        check(f"e2e: {b} recovers raw clock ppm",
              abs(per[b]["value"] - raw_ppm) < 1e-9,
              f"value={per[b]['value']}")
    cons = [r for r in rows if r["band"] == pd.MY_BAND][0]
    check("e2e: consensus equals raw clock ppm",
          abs(cons["value"] - raw_ppm) < 1e-9, f"value={cons['value']}")
    check("e2e: honest small sigma", cons["sigma"] < 1e-6,
          f"sigma={cons['sigma']}")
    check("e2e: kind/ref/fields", cons["kind"] == "ClockDriftPpmComponent"
          and cons["ref_hz"] == pd.L1_HZ and cons["n_sats"] == 2)
    check("e2e: ALL phase rows are non-voting components",
          all(r["kind"] == "ClockDriftPpmComponent" for r in rows))

    # missing carrier-phase fields (pre-dcfcfaa state): skipped, no crash
    rows2, diag2 = pd.process_state(
        {"epoch": 3000.0, "tracker": {"sats": [
            {"sys": "sbas", "prn": 131, "doppler_hz": 50.0,
             "cn0_proxy": 40.0, "lock_s": 500.0}]}},
        {}, 40.0, 40.0, 30.0, 28)
    check("legacy: no carrier fields -> no rows", rows2 == [] and diag2 == {})
    rows3, _ = pd.process_state({"epoch": 3000.0}, {}, 40.0, 40.0, 30.0, 28)
    check("legacy: no tracker key -> no rows", rows3 == [])

    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILURES: {FAILURES}")
        sys.exit(1)
    print("all phase_drift_producer tests passed")


if __name__ == "__main__":
    main()
