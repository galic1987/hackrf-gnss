#!/usr/bin/env python3
"""Tests for scripts/carrier_tdev_analyzer.py (exploratory carrier TDEV).

Fully synthetic — no radio, no live observations, no site.json read (the
GEO tests pass site_ecef explicitly). Run:
    python3 scripts/test_carrier_tdev_analyzer.py
Also pytest-collectable (test_* functions, plain asserts)."""
import math
import random
import sys

import geocorrector_helper
from carrier_tdev_analyzer import (
    CARRIER_GATE_PREREGISTERED, F_CARRIER_HZ, MIN_SEG_ROWS, TAUS,
    analyze, geo_correction_cycles, report_exit_status, tdev_grid,
    x_ns_from_cycles,
)

T0 = 1_788_000_000.0
F_L1 = F_CARRIER_HZ["gps"]


def _white_pm(n, sigma, seed=7):
    rng = random.Random(seed)
    return [rng.gauss(0.0, sigma) for _ in range(n)]


def test_white_pm_recovers_nist_tdev():
    # Same derivation pinned as test_clock_bias_analyzer (NIST SP 1065):
    # for iid x_k with std sigma, E[TDEV(tau=m*dt)] = sigma/sqrt(m)
    # (slope -1/2 on log-log). Pinning it THROUGH tdev_grid proves the
    # carrier analyzer's estimator path (shared clock_bias_analyzer.tdev)
    # keeps the exact tau/m bookkeeping: a stray sqrt(2) or sqrt(3) lands
    # far outside the +/-30% bound; a flat (v1-style) statistic fails the
    # slope pin.
    n, sigma, dt = 7200, 0.4, 1.0
    res = _white_pm(n, sigma)
    tbl, tau_eff, dropped = tdev_grid(res, dt, [10, 100, 1000])
    assert set(tbl) == {10, 100, 1000}, tbl
    assert dropped == [], dropped
    predicted_10 = sigma * math.sqrt(dt / 10.0)   # 0.12649
    assert abs(tbl[10] / predicted_10 - 1.0) < 0.30, (tbl[10], predicted_10)
    slope = math.log(tbl[100] / tbl[10]) / math.log(10.0)
    assert -0.65 < slope < -0.35, slope
    assert tau_eff == {10: 10.0, 100: 100.0, 1000: 1000.0}, tau_eff


def test_tau_below_cadence_is_dropped_not_remapped():
    # On the archive's 10 s grid, tau=1 s must be DROPPED (reason
    # "cadence"), never silently remapped to m=1 (which would report
    # TDEV(10 s) under the tau=1 label).
    res = _white_pm(400, 0.4)
    tbl, tau_eff, dropped = tdev_grid(res, 10.0, TAUS)
    assert 1 not in tbl and 1 not in tau_eff
    assert (1, "cadence") in dropped, dropped
    assert 10 in tbl and 100 in tbl and 1000 in tbl, tbl
    # and a span-limited tau is dropped with reason "span"
    tbl2, _, dropped2 = tdev_grid(res[:200], 10.0, TAUS)  # span 2000 s
    assert 1000 not in tbl2
    assert (1000, "span") in dropped2, dropped2


def test_sign_convention_clock_fast_maps_to_positive_x():
    # Station clock FAST -> LO high -> received carrier appears LOW ->
    # tracker integrates carrier_cycles DOWN. So negative cycles must map
    # to positive time error x, with |x| = |cycles|/f.
    cycles = -F_L1 * 5e-9          # the cycle count a +5 ns error produces
    x = x_ns_from_cycles(cycles, "gps")
    assert abs(x - 5.0) < 1e-9, x
    assert x_ns_from_cycles(+1000.0, "gps") < 0.0
    # per-system carrier: BeiDou B1I is 1561.098 MHz, not L1
    assert abs(x_ns_from_cycles(-1561.098e6 * 1e-9, "beidou") - 1.0) < 1e-9


def _snapshots(sat_series, dt=10.0, n=None):
    """Build collector-style snapshot rows.

    sat_series: {(sys, prn): callable k -> dict(cycles=..., slip=...,
    lock_s=..., geonav=...)} evaluated on the shared epoch grid."""
    if n is None:
        n = max(len(v) if isinstance(v, list) else 0
                for v in sat_series.values()) or 0
    rows = []
    for k in range(n):
        ep = T0 + k * dt
        sats = []
        for (sysname, prn), fn in sat_series.items():
            d = fn(k) if callable(fn) else (fn[k] if k < len(fn) else None)
            if d is None:
                continue
            sat = {"sys": sysname, "prn": prn, "epoch": ep,
                   "carrier_cycles": d["cycles"],
                   "slip": d.get("slip", False),
                   "lock_s": d.get("lock_s", 100.0 + k * dt)}
            if "geonav" in d:
                sat["sbas_geonav"] = d["geonav"]
            sats.append(sat)
        rows.append({"epoch": ep + 5.0, "sats": sats})
    return rows


def _clock_cycles(x_ns_series, f=F_L1):
    """cycles produced by a station time-error series (sign: analyzer)."""
    return [-x * 1e-9 * f for x in x_ns_series]


def test_reseed_segmentation_never_bridges():
    # 200 samples, then carrier_cycles re-zeroes and lock_s resets (the
    # re-seed signature), then 300 more. Must yield exactly 2 segments —
    # the re-seed is never bridged — and the longest (second) is analyzed.
    n1, n2, dt = 200, 300, 10.0
    noise = _white_pm(n1 + n2, 0.2)

    def sat(k):
        if k < n1:
            return {"cycles": 366.0 * k * dt + _clock_cycles([noise[k]])[0],
                    "lock_s": 100.0 + k * dt}
        j = k - n1
        return {"cycles": 366.0 * j * dt + _clock_cycles([noise[k]])[0],
                "lock_s": 1.0 + j * dt}
    rep = analyze(_snapshots({("gps", 7): sat}, dt=dt, n=n1 + n2))
    s = rep["sats"]["gps-7"]
    assert s["n_segments"] == 2, s
    assert len(s["breaks"]) == 1, s["breaks"]
    assert "reseed" in s["breaks"][0]["reason"], s["breaks"]
    assert s["chosen"]["rows"] == n2, s["chosen"]
    assert s["chosen"]["tdev"], s["chosen"]
    assert report_exit_status(rep) == 2


def test_fresh_reseed_lock_s_discontinuity_splits():
    # Adversarial-review probe C: a zeroing BETWEEN two freshly-seeded
    # samples leaves lock_s increased (3 -> 8 across a 10 s interval)
    # yet below the wall-rate floor, with no slip flag and a
    # sub-threshold cycles jump. Continuous lock ages at wall rate, so
    # lock_s < prev + 0.5*dt must split.
    dt = 10.0

    def sat(k):
        if k == 0:
            # single archived sample of the first seed (decimation)
            return {"cycles": 0.0, "lock_s": 3.0}
        j = k - 1
        # second seed: lock_s INCREASED (3 -> 8) so the plain reset rule
        # is silent, cycles offset below RESEED_JUMP_HZ*dt, no slip flag
        return {"cycles": 366.0 * k * dt + 500.0, "lock_s": 8.0 + j * dt}
    rep = analyze(_snapshots({("gps", 7): sat}, dt=dt, n=20))
    s = rep["sats"]["gps-7"]
    assert any("discontinuity" in b["reason"] for b in s["breaks"]), s["breaks"]
    # bridged would analyze all 20 rows; the split second seed has 19
    assert s["n_segments"] == 1 and s["chosen"]["rows"] == 19, s


def test_slip_flag_splits_segment():
    n, dt = 400, 10.0

    def sat(k):
        return {"cycles": 100.0 * k * dt, "slip": (k == 250)}
    rep = analyze(_snapshots({("gps", 8): sat}, dt=dt, n=n))
    s = rep["sats"]["gps-8"]
    assert s["n_segments"] == 2, s
    assert s["breaks"][0]["reason"] == "slip", s["breaks"]
    assert s["chosen"]["rows"] == 250, s["chosen"]


def test_gap_splits_and_longest_segment_wins():
    n, dt = 400, 10.0
    rows = _snapshots({("gps", 9): lambda k: {"cycles": 10.0 * k}},
                      dt=dt, n=n)
    rows = rows[:100] + rows[160:]     # 600 s hole in the epoch grid
    rep = analyze(rows)
    s = rep["sats"]["gps-9"]
    assert s["n_segments"] == 2, s
    assert s["n_grid_splits"] == 1, s
    assert s["chosen"]["rows"] == 240, s["chosen"]


def test_unlocked_and_legacy_rows_are_counted_excluded():
    n, dt = 50, 10.0
    rows = _snapshots({("gps", 5): lambda k: {"cycles": 1.0 * k,
                                              "lock_s": 0.0}}, dt=dt, n=n)
    for r in rows:                      # one legacy (pre-carrier) row each
        r["sats"].append({"sys": "gps", "prn": 30, "epoch": r["epoch"],
                          "lock_s": 5.0})
    rep = analyze(rows)
    assert rep["n_unlocked_excluded"] == n, rep["n_unlocked_excluded"]
    assert rep["n_legacy_sat_rows"] == n, rep["n_legacy_sat_rows"]
    assert not rep["integrity_fails"], rep["integrity_fails"]
    assert report_exit_status(rep) == 1   # nothing produced a TDEV table


def test_malformed_carrier_row_is_fatal():
    rows = _snapshots({("gps", 5): lambda k: {"cycles": 1.0 * k}}, n=50)
    del rows[10]["sats"][0]["slip"]       # carrier-bearing row, no slip flag
    rep = analyze(rows)
    assert rep["integrity_fails"], rep
    assert report_exit_status(rep) == 1


def test_parse_error_and_shrunk_input_are_fatal():
    rows = _snapshots({("gps", 5): lambda k: {"cycles": 1.0 * k}}, n=500)
    rep = analyze(rows, parse_errors=1)
    assert rep["integrity_fails"] and report_exit_status(rep) == 1
    rep = analyze(rows, shrunk=True)
    assert any("immutable snapshot" in f for f in rep["integrity_fails"])
    assert report_exit_status(rep) == 1


def test_duplicate_epochs_deduped_but_conflicts_poison_the_sat():
    rows = _snapshots({("gps", 5): lambda k: {"cycles": 366.0 * k * 10.0}},
                      n=400)
    dup = {"sys": "gps", "prn": 5, "epoch": rows[20]["sats"][0]["epoch"],
           "carrier_cycles": rows[20]["sats"][0]["carrier_cycles"],
           "slip": False, "lock_s": rows[20]["sats"][0]["lock_s"]}
    rows[20]["sats"].append(dict(dup))
    rep = analyze(rows)
    s = rep["sats"]["gps-5"]
    assert s["n_dup_dropped"] == 1 and s["conflict"] is None, s
    assert s["chosen"]["rows"] == 400, s["chosen"]
    # now a CONFLICTING same-epoch value: satellite excluded, reason kept
    rows[20]["sats"][-1]["carrier_cycles"] += 123.0
    rep = analyze(rows)
    assert rep["sats"]["gps-5"]["conflict"], rep["sats"]["gps-5"]
    assert rep["sats"]["gps-5"]["chosen"] is None


def _geo_fixture(n=350, dt=10.0, sigma_cyc=0.02):
    """SBAS GEO whose carrier is exactly the integral of the MT9-predicted
    Doppler (LOS motion incl. acceleration + agf1 clock drift) plus white
    noise. The acceleration term survives linear detrend, so an
    UNCORRECTED analysis shows a large residual while the MT9-corrected
    one recovers the noise floor — pinning the correction wiring AND its
    sign against the audited geocorrector_helper reference."""
    site = geocorrector_helper.llh_to_ecef(39.0, -77.6, 77.0)
    # t0_s must be the GPS time-of-day the helper derives from T0 so the
    # propagation interval starts near zero and grows with the segment
    t_tod0 = ((T0 - 315964800 + 18) % 604800) % 86400
    geonav = {"iodn": 82, "t0_s": t_tod0, "ura": 0,
              "pos_m": [site[0] * 6.0, site[1] * 6.0, 44.8],
              "vel_mps": [2.0, -1.5, 0.3],
              "acc_mps2": [1e-4, 0.0, 0.0],
              "agf0_s": 3e-8, "agf1_sps": -4.5e-12}
    f = F_CARRIER_HZ["sbas"]
    epochs = [T0 + k * dt for k in range(n)]
    corr, err = geo_correction_cycles(epochs, [geonav] * n, site, f)
    assert err is None, err
    noise = _white_pm(n, sigma_cyc, seed=11)
    series = [{"cycles": corr[k] + noise[k], "geonav": geonav}
              for k in range(n)]
    return series, corr, site, f


def test_geo_mt9_correction_flattens_los_motion():
    series, corr, site, f = _geo_fixture()
    n = len(series)
    # the LOS quadratic must be big enough that failing to remove it is
    # loud: check the raw ramp's detrended residual would dwarf the noise
    span_cycles = corr[-1] - corr[0]
    assert abs(span_cycles) > 1e4, span_cycles   # fixture sanity
    rep = analyze(_snapshots({("sbas", 135): series}, n=n),
                  site_ecef=site)
    s = rep["sats"]["sbas-135"]
    assert s["geo"] is True
    c = s["chosen"]
    assert c["geo_status"] == "corrected", c["geo_status"]
    # corrected residual is the injected white noise: sigma_cyc/f in ns
    noise_ns = 0.02 / f * 1e9    # ~0.0127 ns
    assert c["rms_ns"] < 8 * noise_ns, (c["rms_ns"], noise_ns)
    assert c["tdev"], c


def test_geo_uncorrected_residual_is_large_without_mt9():
    # Control for the previous test: same physics, geonav withheld -> the
    # analyzer must LABEL the segment motion-contaminated (fail closed,
    # never a silent zero-correction), and the LOS quadratic really is
    # huge compared to the corrected case.
    series, corr, site, f = _geo_fixture()
    stripped = [{"cycles": d["cycles"]} for d in series]   # no geonav key
    rep = analyze(_snapshots({("sbas", 135): stripped}, n=len(series)),
                  site_ecef=site)
    c = rep["sats"]["sbas-135"]["chosen"]
    assert c["geo_status"].startswith("motion-contaminated"), c["geo_status"]
    # the un-removed quadratic dominates: residual (~30 ns measured for
    # this fixture) sits ~3 orders above the corrected case's noise floor
    # (~0.01 ns in test_geo_mt9_correction_flattens_los_motion)
    assert c["rms_ns"] > 10.0, c["rms_ns"]
    # and a motion-contaminated GEO must not enter the cross-sat view
    cross = rep["cross"]
    assert "unavailable" in cross, cross


def test_geo_without_site_is_motion_contaminated():
    series, corr, site, f = _geo_fixture(n=60)
    rep = analyze(_snapshots({("sbas", 135): series}, n=60), site_ecef=None)
    c = rep["sats"]["sbas-135"]["chosen"]
    assert c["geo_status"].startswith("motion-contaminated"), c["geo_status"]


def test_cross_sat_common_mode_and_spread():
    # 3 GPS sats sharing one white-PM clock (sigma_c) plus independent
    # per-sat noise (sigma_s). On the shared grid:
    #   common = clock + mean(noise)  -> var sigma_c^2 + sigma_s^2/3
    #   dev_s  = noise_s - mean(noise) -> var sigma_s^2 * 2/3
    # TDEV at tau=dt (m=1) equals the series std for white PM, so both are
    # pinned to +/-35%.
    n, dt = 400, 10.0
    sigma_c, sigma_s = 1.0, 0.5
    clock = _white_pm(n, sigma_c, seed=3)
    per = {prn: _white_pm(n, sigma_s, seed=20 + prn) for prn in (1, 2, 3)}
    series = {}
    for prn in (1, 2, 3):
        x = [clock[k] + per[prn][k] for k in range(n)]
        cyc = _clock_cycles(x)
        series[("gps", prn)] = [{"cycles": cyc[k]} for k in range(n)]
    rep = analyze(_snapshots(series, dt=dt, n=n))
    cross = rep["cross"]
    assert "unavailable" not in cross, cross
    assert cross["sats"] == ["gps-1", "gps-2", "gps-3"], cross["sats"]
    assert cross["rows"] == n, cross
    exp_common = math.sqrt(sigma_c ** 2 + sigma_s ** 2 / 3.0)
    got_common = cross["common_tdev"][10]
    assert abs(got_common / exp_common - 1.0) < 0.35, (got_common, exp_common)
    exp_dev = sigma_s * math.sqrt(2.0 / 3.0)
    got_dev = cross["spread_median_tdev"][10]
    assert abs(got_dev / exp_dev - 1.0) < 0.35, (got_dev, exp_dev)
    # the spread is far below the common mode: that separation is the
    # noise-floor evidence this view exists to show
    assert got_dev < got_common


def test_geo_geonav_availability_transition_splits_not_poisons():
    # Real archive shape: ~0.5% of SBAS samples (tracker startup before
    # the first MT9 decode) lack sbas_geonav. Those must fragment off as
    # their own motion-contaminated segment; the long geonav-complete
    # remainder must still be MT9-corrected — one bad sample must not
    # poison an hours-long correctable segment.
    series, corr, site, f = _geo_fixture()
    head = 20
    mixed = ([{"cycles": series[k]["cycles"]} for k in range(head)]
             + series[head:])
    rep = analyze(_snapshots({("sbas", 135): mixed}, n=len(series)),
                  site_ecef=site)
    s = rep["sats"]["sbas-135"]
    assert any(b["reason"] == "geonav availability change"
               for b in s["breaks"]), s["breaks"]
    assert s["n_segments"] == 2, s
    c = s["chosen"]
    assert c["geo_status"] == "corrected", c["geo_status"]
    assert c["rows"] == len(series) - head, c


def test_cross_sat_window_ignores_transient_third_sat():
    # A third satellite covering only the middle of a long 2-sat overlap
    # must not shrink the window (the old constant-full-membership rule
    # broke the run at every rise/set): the 2-sat window keeps its full
    # span and the transient sat is simply not a member.
    n, dt = 400, 10.0
    series = {("gps", 1): lambda k: {"cycles": 366.0 * k * dt},
              ("gps", 2): lambda k: {"cycles": -250.0 * k * dt},
              ("gps", 3): lambda k: ({"cycles": 100.0 * k * dt}
                                     if 100 <= k < 200 else None)}
    rep = analyze(_snapshots(series, dt=dt, n=n))
    cross = rep["cross"]
    assert "unavailable" not in cross, cross
    assert cross["rows"] == n, cross
    assert cross["sats"] == ["gps-1", "gps-2"], cross["sats"]


def test_cross_sat_membership_breaks_at_reseed():
    # sat 2 re-seeds mid-window: the constant-membership run must not
    # bridge it — the reported window sits on one side of the re-seed.
    n, dt = 400, 10.0

    def plain(k):
        return {"cycles": 366.0 * k * dt}

    def reseeder(k):
        if k < 150:
            return {"cycles": 366.0 * k * dt, "lock_s": 100.0 + k * dt}
        return {"cycles": 366.0 * (k - 150) * dt,
                "lock_s": 1.0 + (k - 150) * dt}
    rep = analyze(_snapshots({("gps", 1): plain, ("gps", 2): reseeder},
                             dt=dt, n=n))
    cross = rep["cross"]
    assert "unavailable" not in cross, cross
    assert cross["rows"] == 250, cross   # the post-reseed side is longer
    assert cross["span_s"] <= 2490.0, cross


def test_exit_zero_is_unreachable():
    assert CARRIER_GATE_PREREGISTERED is False
    # a fully healthy run is still only exploratory (exit 2)
    rows = _snapshots({("gps", 5): lambda k: {"cycles": 366.0 * k * 10.0}},
                      n=400)
    rep = analyze(rows)
    assert report_exit_status(rep) == 2
    assert rep["exploratory_reasons"]
    # and an empty run fails (exit 1)
    assert report_exit_status(analyze([])) == 1


def test_short_fragments_are_counted_not_analyzed():
    n = MIN_SEG_ROWS - 2
    rep = analyze(_snapshots({("gps", 5): lambda k: {"cycles": 1.0 * k}},
                             n=n))
    s = rep["sats"]["gps-5"]
    assert s["n_segments"] == 0 and s["n_short_segments"] == 1, s
    assert s["chosen"] is None
    assert report_exit_status(rep) == 1


def main():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v)]
    failures = []
    for t in tests:
        try:
            t()
        except AssertionError as e:
            failures.append(t.__name__)
            print("FAIL %s: %s" % (t.__name__, e))
        else:
            print("PASS %s" % t.__name__)
    print("\n%d failure(s)" % len(failures) if failures
          else "\nall tests passed")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
