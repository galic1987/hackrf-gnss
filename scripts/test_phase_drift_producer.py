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
  - P0b GeoCorrector: synthetic-GEO sign recovery, ephemeris-swap
    stitching, fail-closed gates, GPS-day wrap, slip re-anchor, and the
    process_state shadow wiring (diag keys + evidence jsonl; published
    rows proven to stay the uncorrected path)

  python3 scripts/test_phase_drift_producer.py
"""
import json
import math
import os
import sys
import tempfile

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

    # --- fit_drift: autocorrelated residuals — pinned calibration fact ------
    # Monte Carlo (deterministic LCG): on AR(1) rho=0.95 residuals the
    # residual-based OLS sigma UNDERSTATES the true slope scatter (measured
    # 2026-08-28: ~5x at n=40; a Newey-West HAC on the same residuals was
    # worse still and was reverted). This test pins the limitation so the
    # estimator is never "trusted" blindly again.
    def gen_ar1(t0, n, slope_hz, rho, innov_cyc, seed):
        out, x, e = [], seed, 0.0
        for i in range(n):
            x = (1103515245 * x + 12345) % (2**31)
            e = rho * e + innov_cyc * (2.0 * (x / 2**31) - 1.0)
            out.append((t0 + i, slope_hz * i + e))
        return out

    errs, sigs = [], []
    for k in range(300):
        ar = gen_ar1(1000.0, 40, 50.0, 0.95, 0.02, seed=29 + 131 * k)
        sl, sg, _ = pd.fit_drift(ar)
        errs.append(sl - 50.0)
        sigs.append(sg)
    emp = math.sqrt(sum(e * e for e in errs) / len(errs))
    sig_med = sorted(sigs)[len(sigs) // 2]
    check("fit: AR(1) OLS sigma understates (pinned, ~5x)",
          emp > 1.5 * sig_med,
          f"empirical={emp:.3e} median_sigma={sig_med:.3e}")
    check("fit: AR(1) slope stays unbiased", abs(sum(errs) / len(errs)) < emp,
          f"mean_err={sum(errs)/len(errs):.3e}")

    # --- scatter_sigma: calibrated where per-window OLS is not ---------------
    # 12 DISJOINT AR(1) windows (rho=0.95): the MAD scatter of their slopes
    # must show the inflation the per-window OLS sigma hides (MC: ~4.8x).
    check("scatter: needs >=5 fits",
          pd.scatter_sigma([1.0, 2.0, 3.0, 4.0]) is None)
    pairs = []
    for k in range(12):
        ar = gen_ar1(2000.0 + k * 40.0, 40, 50.0, 0.95, 0.02, seed=101 + 37 * k)
        pairs.append(pd.fit_drift(ar))
    sc = pd.scatter_sigma([sl for sl, _, _ in pairs])
    med_ols = sorted(sg for _, sg, _ in pairs)[len(pairs) // 2]
    check("scatter: exceeds per-window OLS on AR(1)",
          sc is not None and sc > 1.5 * med_ols,
          f"scatter={sc:.3e} med_ols={med_ols:.3e}")
    # and it stays sane on white noise (no fake inflation)
    wnp = []
    for k in range(12):
        wn = gen(2000.0 + k * 40.0, 40, 50.0, noise_cyc=0.02, seed=3 + k)
        wnp.append(pd.fit_drift(wn))
    sc_w = pd.scatter_sigma([sl for sl, _, _ in wnp])
    med_ols_w = sorted(sg for _, sg, _ in wnp)[len(wnp) // 2]
    check("scatter: ~OLS on white residuals",
          sc_w is not None and sc_w < 2.5 * med_ols_w,
          f"scatter={sc_w:.3e} med_ols={med_ols_w:.3e}")

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
    # residual slope = (raw - corr) * L1 = -0.020 ppm of L1. The discipline
    # state declares the correction APPLIED (actuate + corr_applied) — the
    # only case where the add-back may happen (review 2026-08-26).
    raw_ppm, corr = -0.471, -0.451
    resid_hz = (raw_ppm - corr) * 1e-6 * pd.L1_HZ
    windows = {}
    rows = []
    for i in range(45):
        state = {
            "epoch": 2000.0 + i,
            "discipline": {"correction_ppm": corr,
                           "actuate": True, "corr_applied": True},
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
    check("e2e: rows flagged corr_applied True",
          all(r.get("corr_applied") is True for r in rows))

    # SHADOW mode (actuate/corr_applied false): nothing was written to
    # hardware, so the cached correction intent must NOT be added back —
    # the published value is the measured residual, flagged False
    # (the 2026-08-26 bug: -0.3378 ppm intent was added unconditionally)
    windows = {}
    rows = []
    for i in range(45):
        state = {
            "epoch": 4000.0 + i,
            "discipline": {"correction_ppm": corr,
                           "actuate": False, "corr_applied": False},
            "tracker": {"sats": [
                {"sys": "sbas", "prn": 133, "epoch": 4000.0 + i,
                 "carrier_cycles": resid_hz * i, "doppler_hz": resid_hz,
                 "cn0_proxy": 42.0, "lock_s": 400.0 + i, "slip": False},
            ]},
        }
        rows, diag = pd.process_state(state, windows, 40.0, 40.0, 30.0, 28)
    per = {r["band"]: r for r in rows if r["band"] != pd.MY_BAND}
    check("e2e shadow: cached correction NOT added back",
          abs(per["L1 / WAAS 133 (phase)"]["value"] - (raw_ppm - corr)) < 1e-9,
          f"value={per['L1 / WAAS 133 (phase)']['value']}")
    check("e2e shadow: rows flagged corr_applied False",
          all(r.get("corr_applied") is False for r in rows))

    # missing carrier-phase fields (pre-dcfcfaa state): skipped, no crash
    rows2, diag2 = pd.process_state(
        {"epoch": 3000.0, "tracker": {"sats": [
            {"sys": "sbas", "prn": 131, "doppler_hz": 50.0,
             "cn0_proxy": 40.0, "lock_s": 500.0}]}},
        {}, 40.0, 40.0, 30.0, 28)
    check("legacy: no carrier fields -> no rows", rows2 == [] and diag2 == {})
    rows3, _ = pd.process_state({"epoch": 3000.0}, {}, 40.0, 40.0, 30.0, 28)
    check("legacy: no tracker key -> no rows", rows3 == [])

    # --- emitted GEO sigma: calibrated, never optimistic OLS (round 16) ---
    # Six disjoint windows whose TRUE slope wanders ±0.4 Hz across windows:
    # the cross-window scatter must dominate each window's tiny OLS sigma,
    # and the emitted row sigma must carry it (not the raw OLS value).
    def feed(n_win, win_s=60.0):
        windows = {}
        rows, diag = [], {}
        for k in range(n_win):
            base = 100000.0 + k * (win_s + 10.0)
            wander = 0.4 * math.sin(k * 2.1)
            for i in range(int(win_s) + 1):
                t = base + i
                nz = 0.004 * (((i * 37 + k * 11) % 11) - 5) / 5.0
                cyc = (-500.0 + wander) * i + nz
                st = {"epoch": t, "tracker": {"sats": [
                    {"sys": "sbas", "prn": 131, "carrier_cycles": cyc,
                     "slip": False, "cn0_proxy": 40.0,
                     "lock_s": 5000.0 + i, "epoch": t}]}}
                rows, diag = pd.process_state(st, windows, win_s, 10.0, 25.0, 20)
        return rows, diag

    rows, diag = feed(6)
    d = diag["sbas 131"]
    per = {r["band"]: r for r in rows}
    emitted = per["L1 / WAAS 131 (phase)"]["sigma"]
    check("sigma: scatter wired into emitted row",
          d["scatter_sigma_ppm"] is not None and not d["sigma_provisional"]
          and abs(emitted - d["emitted_sigma_ppm"]) < 1e-9
          and emitted >= d["scatter_sigma_ppm"] - 1e-12
          and emitted >= d["fit_sigma_ppm"],
          f"emitted={emitted} scatter={d['scatter_sigma_ppm']} ols={d['fit_sigma_ppm']}")
    check("sigma: scatter actually dominates tiny OLS here",
          d["scatter_sigma_ppm"] > 3.0 * d["fit_sigma_ppm"],
          f"ratio={d['scatter_sigma_ppm']/max(d['fit_sigma_ppm'],1e-15):.1f}")

    rows, diag = feed(2)   # fresh windows: <5 disjoint fits -> provisional
    d = diag["sbas 131"]
    check("sigma: provisional 5x inflation before 5 windows",
          d["sigma_provisional"] is True
          and abs(d["emitted_sigma_ppm"] - 5.0 * d["fit_sigma_ppm"]) < 1e-12,
          f"emitted={d['emitted_sigma_ppm']} ols={d['fit_sigma_ppm']}")

    # --- P0b GeoCorrector: synthetic GEO, sign, stitch, gates, wrap -------
    # Synthetic GEO over the real site-anchor coordinates. TRUTH is one
    # ephemeris (geo1) for the whole span; the measured carrier is
    #   cycles(t) = -rho(t)/lam + f_L1*dt_geo(t) + f_clock*t
    # (range shortening advances the phase — the tracker's f_D < 0 when
    # the satellite recedes — and the GEO clock polynomial rides on top).
    # The corrector must remove BOTH geometry terms and leave f_clock.
    site = pd.llh_to_ecef(39.0029556, -77.6051478, 77.1)
    lam = pd.LAM_L1_M
    f_clock = 1.3
    t0_s = 65216.0

    def mk_geo(t0, pos, vel, acc, agf0=-8.75e-8, agf1=-9.09e-13,
               iodn=47, ura=0):
        return {"iodn": iodn, "t0_s": t0, "ura": ura, "pos_m": list(pos),
                "vel_mps": list(vel), "acc_mps2": list(acc),
                "agf0_s": agf0, "agf1_sps": agf1, "applied_t": 0.0}

    def prop(geo, dt):
        return [geo["pos_m"][i] + geo["vel_mps"][i] * dt
                + 0.5 * geo["acc_mps2"][i] * dt * dt for i in range(3)]

    def rho_site(geo, dt):
        p = prop(geo, dt)
        return math.sqrt(sum((p[i] - site[i]) ** 2 for i in range(3)))

    def unix_of(tod):
        # any GPS day works — the corrector sees only time-of-day
        return pd.GPS_UNIX_EPOCH - pd.GPS_LEAP_S + 86400.0 * 17000 + tod

    def truth_cycles(geo, i):
        return (-rho_site(geo, i) / lam
                + pd.L1_HZ * (geo["agf0_s"] + geo["agf1_sps"] * i)
                + f_clock * i)

    geo1 = mk_geo(t0_s, [-19139594.24, -37569516.96, -2323.2],
                  [0.16125, 0.075, 0.104], [2.5e-5, -2.5e-5, 0.0])

    # sign + recovery: the corrected slope IS f_clock, the raw one is not
    # (it keeps the ~1 Hz class line-of-sight range rate)
    gc = pd.GeoCorrector(site)
    corr_s, raw_s = [], []
    ok1 = True
    for i in range(45):
        t = unix_of(t0_s + i)
        gc.update(geo1, t)
        cc, applied, _ = gc.correct(t, truth_cycles(geo1, i))
        ok1 &= applied
        corr_s.append((t, cc))
        raw_s.append((t, truth_cycles(geo1, i)))
    check("p0b: every sample applied", ok1)
    sl_c, _, _ = pd.fit_drift(corr_s)
    sl_u, _, _ = pd.fit_drift(raw_s)
    check("p0b: corrected slope recovers f_clock within 1e-3 Hz",
          abs(sl_c - f_clock) < 1e-3, f"slope={sl_c}")
    check("p0b: uncorrected slope keeps the range rate",
          abs(sl_u - f_clock) > 1e-3, f"slope={sl_u}")

    # ephemeris swap stitching: geo2 = geo1 propagated 128 s to its own t0
    # plus a decimetre-class refresh difference; the truth signal stays
    # geo1 (the satellite does not jump — the MODEL does), the corrector
    # swaps geo1 -> geo2 at i=40 and must stitch the series step-free
    dt2 = 128.0
    p2 = prop(geo1, dt2)
    p2[0] += 0.4
    p2[2] -= 0.3
    v2 = [geo1["vel_mps"][i] + geo1["acc_mps2"][i] * dt2 for i in range(3)]
    geo2 = mk_geo(t0_s + dt2, p2, v2, geo1["acc_mps2"], iodn=48)
    gc2 = pd.GeoCorrector(site)
    swap_s = []
    ok2 = True
    for i in range(80):
        t = unix_of(t0_s + i)
        gc2.update(geo2 if i >= 40 else geo1, t)   # swap fires at i=40
        cc, applied, _ = gc2.correct(t, truth_cycles(geo1, i))
        ok2 &= applied
        swap_s.append((t, cc))
    check("p0b swap: every sample applied", ok2)
    step = (swap_s[40][1] - swap_s[39][1]) - f_clock
    check("p0b swap: stitched series has NO STEP at the swap",
          abs(step) < 1e-6, f"step={step:.2e} cycles")
    sl2, _, _ = pd.fit_drift(swap_s)
    check("p0b swap: slope recovered across the swap",
          abs(sl2 - f_clock) < 1e-3, f"slope={sl2}")

    # slip re-anchor: a stitch exists, then a slip resets it (a step is
    # legal across a phase break — re-anchor, don't stitch)
    g6 = pd.GeoCorrector(site)
    g6.update(geo1, unix_of(t0_s))
    g6.update(geo2, unix_of(t0_s + 40))    # stitch -> offset nonzero
    stitched = g6.offset_cycles
    g6.update(geo1, unix_of(t0_s + 41), slip=True)
    check("p0b slip: slip re-anchors (stitch offset reset)",
          stitched != 0.0 and g6.offset_cycles == 0.0,
          f"offset {stitched:.3e} -> {g6.offset_cycles}")

    # gates: fail-closed with a reason
    g3 = pd.GeoCorrector(site)
    _, a, r = g3.correct(unix_of(t0_s), 0.0)
    check("p0b gate: missing geonav", not a and r == "no-geonav", f"{a} {r}")
    g3.update(mk_geo(t0_s, [-19139594.24, -37569516.96, -2323.2],
                     [0.16125, 0.075, 0.104], [2.5e-5, -2.5e-5, 0.0],
                     ura=15), unix_of(t0_s))
    _, a, r = g3.correct(unix_of(t0_s + 10), 0.0)
    check("p0b gate: ura 15 rejected", not a and r == "ura", f"{a} {r}")
    g4 = pd.GeoCorrector(site)
    g4.update(geo1, unix_of(t0_s + 3700))
    _, a, r = g4.correct(unix_of(t0_s + 3700), 0.0)
    check("p0b gate: |dt|>3600 -> stale", not a and r == "stale", f"{a} {r}")
    _, a, r = g4.correct(unix_of(t0_s + 3599), 0.0)
    check("p0b gate: |dt|=3599 still applies", a, f"{a} {r}")

    # GPS-day wrap: t0 at 86384 (16 s grid), evaluation 60 s past
    # midnight -> dt wraps to +76 s and the correction still applies
    gw = mk_geo(86384.0, [-19139594.24, -37569516.96, -2323.2],
                [0.16125, 0.075, 0.104], [2.5e-5, -2.5e-5, 0.0])
    g5 = pd.GeoCorrector(site)
    g5.update(gw, unix_of(60.0))
    _, a, r = g5.correct(unix_of(60.0), 0.0)
    check("p0b wrap: 60 s past midnight -> dt=+76 s, applied",
          a and abs(g5.last_dt - 76.0) < 1e-9, f"dt={g5.last_dt} {a} {r}")

    # --- P0b shadow wiring through process_state --------------------------
    # diag carries p0b_*; the shadow jsonl gains one line per GEO per
    # cycle; and the PUBLISHED row value stays the uncorrected slope
    shadow_path = os.path.join(tempfile.mkdtemp(prefix="p0b_test_"),
                               "shadow.jsonl")
    p0b = pd.P0bShadow(site, shadow_path)
    windows = {}
    rows = []
    for i in range(45):
        t = unix_of(t0_s + i)
        cyc = truth_cycles(geo1, i)
        st = {"epoch": t, "tracker": {"sats": [
            {"sys": "sbas", "prn": 131, "epoch": t, "carrier_cycles": cyc,
             "doppler_hz": 2.1, "cn0_proxy": 40.0, "lock_s": 400.0 + i,
             "slip": False, "sbas_geonav": geo1},
            {"sys": "sbas", "prn": 135, "epoch": t,
             "carrier_cycles": cyc + 7e5, "doppler_hz": 2.1,
             "cn0_proxy": 38.0, "lock_s": 400.0 + i, "slip": False,
             "sbas_geonav": geo1},
            {"sys": "gps", "prn": 8, "epoch": t,
             "carrier_cycles": 1800.0 * i, "doppler_hz": 1800.0,
             "cn0_proxy": 45.0, "lock_s": 400.0 + i, "slip": False},
        ]}}
        rows, diag = pd.process_state(st, windows, 40.0, 40.0, 30.0, 28, p0b)
    ppm_clock = f_clock / pd.L1_HZ * 1e6
    d131 = diag["sbas 131"]
    check("p0b wire: diag carries applied fit + provenance",
          d131.get("p0b_applied") is True
          and abs(d131["p0b_ppm"] - ppm_clock) < 1e-6
          and d131.get("p0b_sigma_ppm") is not None
          and d131.get("p0b_iodn") == 47
          and d131.get("p0b_age_s") is not None,
          f"ppm={d131.get('p0b_ppm')} expect~{ppm_clock}")
    per = {r["band"]: r for r in rows if r["band"] != pd.MY_BAND}
    unc = per["L1 / WAAS 131 (phase)"]["value"]
    check("p0b wire: published row is STILL the uncorrected value",
          abs(unc - sl_u / pd.L1_HZ * 1e6) < 1e-6
          and abs(unc - ppm_clock) > 1e-7,
          f"row={unc} uncorr_fit={sl_u / pd.L1_HZ * 1e6}")
    check("p0b wire: GPS sat carries no p0b keys",
          "p0b_applied" not in diag.get("gps 8", {}))
    # gated-out sbas sat still records applied:False + reason
    st = {"epoch": unix_of(t0_s + 50), "tracker": {"sats": [
        {"sys": "sbas", "prn": 133, "epoch": unix_of(t0_s + 50),
         "carrier_cycles": 1.0, "cn0_proxy": 40.0, "lock_s": 500.0,
         "slip": False}]}}
    _, diag_ng = pd.process_state(st, {}, 40.0, 40.0, 30.0, 28, p0b)
    check("p0b wire: no-geonav sat records applied False + reason",
          diag_ng["sbas 133"].get("p0b_applied") is False
          and diag_ng["sbas 133"].get("p0b_reason") == "no-geonav",
          f"{diag_ng['sbas 133']}")
    lines = [json.loads(l) for l in open(shadow_path)]
    check("p0b wire: shadow jsonl has both PRNs",
          {l["prn"] for l in lines} == {131, 135}, f"n={len(lines)}")
    last = lines[-1]
    check("p0b wire: shadow line schema",
          {"epoch", "prn", "p0b_ppm", "p0b_sigma_ppm", "uncorr_ppm",
           "iodn", "n"} <= set(last.keys()), f"{sorted(last.keys())}")
    check("p0b wire: shadow corrected/uncorrected differ by range rate",
          abs(last["p0b_ppm"] - last["uncorr_ppm"]) > 1e-7,
          f"{last['p0b_ppm']} vs {last['uncorr_ppm']}")

    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILURES: {FAILURES}")
        sys.exit(1)
    print("all phase_drift_producer tests passed")


if __name__ == "__main__":
    main()


def test_main():
    """pytest entry point: the suite is the plain-assert main() above
    (repo style — AGENTS.md runs it as a script); this wrapper makes
    `python3 -m pytest scripts/test_phase_drift_producer.py -x -q`
    execute the same checks (main() sys.exit(1)s on any failure)."""
    main()
