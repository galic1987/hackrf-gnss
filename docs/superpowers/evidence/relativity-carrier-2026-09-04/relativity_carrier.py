#!/usr/bin/env python3
"""Carrier-phase measurement of the GPS relativistic periodic clock term (v2).

Observable: between-satellite single difference of the tracker's integrated
carrier (telemetry_log.jsonl carrier_cycles, 10 s cadence), high-e sat H vs
near-circular reference R:  L = -(cyc_H - cyc_R) * c/f_L1  [m].
Model M = (rho_H - rho_R) at transmit time (Sagnac) - c(poly_H - poly_R)
          + ZTD (1/sin el_H - 1/sin el_R);  D = L - M.
Regressor Rreg = -c (dt_r_H - dt_r_R), dt_r = F e sqrtA sin Ek, so a = +1
means the carrier follows IS-GPS-200's term with its published sign.

Estimator: the archive's carrier carries per-channel STEPS at USB drop
holes (live.rs note_gap advances stream time but not the NCO phase; the
PLL re-settles with a net step ~ range-rate x hole), 6-12 % of 10 s
increments, metre-class, on a 7-10 cm core. Therefore the fit is done on
FIRST DIFFERENCES with iterative MAD outlier exclusion (steps drop out as
single-increment outliers; a smooth signal loses only those increments).
Nuisances (increment domain): slope (constant), receiver time-tag offset
delta0 (regressor d(rdot_H - rdot_R)), optional quadratic. A level-domain
fit (ACF n_eff) is reported for comparison, plus a phase-shifted null
regressor (cos Ek) that must fit ~0.
"""
import glob, json, math, os, sys, datetime
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rc_core import *   # constants, RINEX parser, orbit model, carrier segmentation, ols/n_eff/fit

REF_E_MAX_PRIMARY = 0.0030
REF_E_MAX_SECONDARY = 0.0060
MAD_K = 4.0
THR_M = 0.10          # fixed increment-residual exclusion threshold (m); drop steps are >= ~1 m
THR_VARIANTS = [0.05, 0.2, 0.5]
MIN_DIFF_SWING_M = 3.0
TCUT = datetime.datetime(2026, 8, 30, 20, 0, tzinfo=datetime.timezone.utc).timestamp()  # sample-accurate stream epoch (commit 3107967, 2026-08-30 18:01 UTC) + margin
DT_MIN, DT_MAX = 8.0, 12.0

def mad_sigma(r):
    return 1.4826 * np.median(np.abs(r - np.median(r)))

def robust_inc_fit(dD, cols, names, thr=None):
    """OLS on increments with annealed fixed-threshold exclusion (3 m -> 1 m
    -> 0.3 m -> thr, then iterate at thr). Returns dict + residuals + mask.
    Fails closed (NaN) if fewer than p+10 increments survive."""
    thr = THR_M if thr is None else thr
    X = np.stack(cols, axis=1)
    sc = np.abs(X).max(axis=0); sc[sc == 0] = 1
    Xs = X / sc
    n_all, p = X.shape
    keep = np.ones(n_all, bool)
    sched = [t for t in (3.0, 1.0, 0.3) if t > thr] + [thr] * 6
    r = np.zeros(n_all)
    for th in sched:
        if keep.sum() < p + 10:
            break
        b, *_ = np.linalg.lstsq(Xs[keep], dD[keep], rcond=None)
        r = dD - Xs @ b
        new = np.abs(r) <= th
        if th == thr and (new == keep).all():
            break
        keep = new
    if keep.sum() < p + 10:
        nan = float("nan")
        return dict(names=names, beta=[nan] * p, err=[nan] * p, n_inc=int(n_all), n_kept=int(keep.sum()),
                    frac_excluded=1.0, core_sigma_m=nan, n_eff=nan, acf_lags=0, status="failed: <p+10 increments kept"), r, keep
    b, *_ = np.linalg.lstsq(Xs[keep], dD[keep], rcond=None)
    r = dD - Xs @ b
    n = int(keep.sum())
    sigma2 = (r[keep] ** 2).sum() / max(n - p, 1)
    neff, lags = n_eff_acf(r[keep])
    cov = sigma2 * np.linalg.inv(Xs[keep].T @ Xs[keep]) * (n / max(neff, p + 1))
    err = np.sqrt(np.diag(cov)) / sc
    beta = b / sc
    out = dict(names=names, beta=[float(v) for v in beta], err=[float(v) for v in err],
               n_inc=int(n_all), n_kept=n, frac_excluded=float(1 - n / n_all),
               core_sigma_m=float(np.sqrt(sigma2)), n_eff=float(neff), acf_lags=int(lags), status="ok")
    return out, r, keep

def main():
    site = np.array(geocorrector_helper.get_site_ecef())
    sj = json.load(open("/Volumes/Radiator 8TB/gnss/observations/site.json"))
    lat, lon = sj["lat"], sj["lon"]
    SCR2 = os.path.dirname(os.path.abspath(__file__))
    eph = parse_rinex_gps(sorted(glob.glob(os.path.join(SCR2, "brdc", "*.rnx"))))
    ecc_mean = {p: float(np.mean([r["ecc"] for r in eph[p]])) for p in eph}
    hi = sorted([p for p in ecc_mean if ecc_mean[p] >= HI_E_MIN], key=lambda p: -ecc_mean[p])
    refs2 = sorted(ecc_mean)
    d = load_carrier()
    segs = {p: segments_for(d, p) for p in hi + refs2}
    u = lambda t: datetime.datetime.utcfromtimestamp(t).strftime("%Y-%m-%d %H:%M")
    print("high-e:", ["G%02d e=%.4f" % (p, ecc_mean[p]) for p in hi])
    print("refs  :", ["G%02d e=%.4f" % (p, ecc_mean[p]) for p in refs2])

    results = []; plots_data = []
    for H in hi:
        for (i0, i1) in segs[H]["segs"]:
            tH0, tH1 = segs[H]["ep"][i0], segs[H]["ep"][i1]
            if (tH1 - tH0) / 3600 < MIN_ARC_H:
                continue
            for R in refs2:
                for (j0, j1) in segs[R]["segs"]:
                    tR0, tR1 = segs[R]["ep"][j0], segs[R]["ep"][j1]
                    lo, hi_ = max(tH0, tR0), min(tH1, tR1)
                    if (hi_ - lo) / 3600 < MIN_ARC_H:
                        continue
                    epH = segs[H]["ep"][i0:i1 + 1]; cyH = segs[H]["cyc"][i0:i1 + 1]
                    epR = segs[R]["ep"][j0:j1 + 1]; cyR = segs[R]["cyc"][j0:j1 + 1]
                    common, ia, ib = np.intersect1d(epH, epR, return_indices=True)
                    m = (common >= lo) & (common <= hi_)
                    common, ia, ib = common[m], ia[m], ib[m]
                    if len(common) < 300 or common[0] < TCUT or R == H:
                        continue
                    t_rx = common - GPS_UNIX0 + LEAP
                    tmid = t_rx.mean()
                    eH = nearest_eph(eph[H], tmid); eR = nearest_eph(eph[R], tmid)
                    rH, pH, relH, posH = model_range(eH, t_rx, site)
                    rR, pR, relR, posR = model_range(eR, t_rx, site)
                    elH = elevation_deg(posH, site, lat, lon); elR = elevation_deg(posR, site, lat, lon)
                    ok = (elH >= MIN_EL_DEG) & (elR >= MIN_EL_DEG)
                    if ok.sum() < 300:
                        continue
                    sel = np.where(ok)[0]
                    t_rx, common, ia, ib = t_rx[sel], common[sel], ia[sel], ib[sel]
                    rH, pH, relH, elH = rH[sel], pH[sel], relH[sel], elH[sel]
                    rR, pR, relR, elR = rR[sel], pR[sel], relR[sel], elR[sel]
                    # null regressor: same amplitude, Ek phase-shifted by 90 deg
                    _, EH, _, _ = sat_state(eH, t_rx); _, ER, _, _ = sat_state(eR, t_rx)
                    nullH = F_REL * eH["ecc"] * eH["sqrt_a"] * np.cos(EH); nullR = F_REL * eR["ecc"] * eR["sqrt_a"] * np.cos(ER)
                    Rnull = -C * (nullH - nullR)
                    tropo = ZTD_M * (1 / np.sin(np.radians(elH)) - 1 / np.sin(np.radians(elR)))
                    L = -(cyH[ia] - cyR[ib]) * LAM
                    M = (rH - rR) - C * (pH - pR) + tropo
                    Rreg = -C * (relH - relR)
                    D = L - M
                    if Rreg.max() - Rreg.min() < MIN_DIFF_SWING_M:
                        continue
                    t = (t_rx - t_rx[0]) / 3600.0
                    rH2, _, _, _ = model_range(eH, t_rx + 0.5, site); rH1, _, _, _ = model_range(eH, t_rx - 0.5, site)
                    rR2, _, _, _ = model_range(eR, t_rx + 0.5, site); rR1, _, _, _ = model_range(eR, t_rx - 0.5, site)
                    drdot = (rH2 - rH1) - (rR2 - rR1)
                    dL = np.diff(L) / np.diff(t_rx); dM = np.diff((rH - rR) - C * (pH - pR)) / np.diff(t_rx)
                    slope = float(np.polyfit(dM, dL, 1)[0])
                    # ---- level-domain fits (for comparison)
                    lv = {}
                    lv["lin_tt"], _ = fit(D, t, Rreg, extra=[("dt_rx_s", drdot)])
                    lv["quad_tt"], _ = fit(D, t, Rreg, quad=True, extra=[("dt_rx_s", drdot)])
                    # ---- increment domain
                    dt = np.diff(t_rx); cad = (dt >= DT_MIN) & (dt <= DT_MAX)
                    dD = np.diff(D)[cad]; dR = np.diff(Rreg)[cad]; dN = np.diff(Rnull)[cad]; ddr = np.diff(drdot)[cad]
                    dts = dt[cad]; tm = (0.5 * (t[1:] + t[:-1]))[cad]
                    inc = {}
                    inc["lin"], r_lin, keep_lin = robust_inc_fit(dD, [dR, dts, ddr], ["a", "c1", "dt_rx_s"])
                    inc["quad"], r_quad, keep_quad = robust_inc_fit(dD, [dR, dts, ddr, tm * dts], ["a", "c1", "dt_rx_s", "c2"])
                    inc["lin_nott"], _, _ = robust_inc_fit(dD, [dR, dts], ["a", "c1"])
                    inc["null_lin"], _, _ = robust_inc_fit(dD, [dN, dts, ddr], ["a_null", "c1", "dt_rx_s"])
                    inc["null_quad"], _, _ = robust_inc_fit(dD, [dN, dts, ddr, tm * dts], ["a_null", "c1", "dt_rx_s", "c2"])
                    inc["both_lin"], _, _ = robust_inc_fit(dD, [dR, dN, dts, ddr], ["a", "a_null", "c1", "dt_rx_s"])
                    inc["norel_lin"], r_no, keep_no = robust_inc_fit(dD, [dts, ddr], ["c1", "dt_rx_s"])
                    thr_var = {}
                    for th in THR_VARIANTS:
                        fv, _, _ = robust_inc_fit(dD, [dR, dts, ddr], ["a", "c1", "dt_rx_s"], thr=th)
                        thr_var["%.2f" % th] = dict(a=round(fv["beta"][0], 4), a_err=round(fv["err"][0], 4), frac_excluded=round(fv["frac_excluded"], 4))
                    for k in ("lin", "quad", "lin_nott"):
                        f = inc[k]; f["a"] = f["beta"][0]; f["a_err"] = f["err"][0]
                        f["sig_a0"] = f["a"] / f["a_err"]; f["sig_a1"] = (f["a"] - 1) / f["a_err"]
                    for k in ("null_lin", "null_quad"):
                        f = inc[k]; f["a_null"] = f["beta"][0]; f["a_null_err"] = f["err"][0]
                    swing = float(Rreg.max() - Rreg.min())
                    # repaired level series (excluded increments bridged by the fitted model) for plots + RMS
                    b = inc["lin"]["beta"]; model_inc = b[0] * dR + b[1] * dts + b[2] * ddr
                    inc_rep = np.where(keep_lin, dD, model_inc)
                    Drep = np.concatenate([[0.0], np.cumsum(inc_rep)])
                    tk = np.concatenate([[t[0]], (t[1:][cad])])
                    Rk = np.concatenate([[Rreg[0]], Rreg[1:][cad]]); drk = np.concatenate([[drdot[0]], drdot[1:][cad]])
                    nuis = b[1] * (tk - tk[0]) * 3600 / 1.0 * 0 + b[1] * np.concatenate([[0.0], np.cumsum(dts)]) + b[2] * (drk - drk[0])
                    Dplot = Drep - nuis; Dplot -= (Dplot - b[0] * Rk).mean()
                    rms_rep = float(np.sqrt(((Dplot - b[0] * Rk) ** 2).mean()))
                    res = dict(H="G%02d" % H, R="G%02d" % R, e_H=round(eH["ecc"], 5), e_R=round(eR["ecc"], 5),
                               ref_class="primary" if eR["ecc"] <= REF_E_MAX_PRIMARY else ("secondary" if eR["ecc"] <= REF_E_MAX_SECONDARY else "any_e"),
                               local_hour_start=round(((common[0] / 3600.0) % 24 - 4) % 24, 1), threshold_variants=thr_var,
                               arc=[u(common[0]), u(common[-1])], arc_h=round((common[-1] - common[0]) / 3600, 2),
                               n=int(len(common)), status="fitted",
                               eph_H=dict(src=eH["src"], iode=eH["iode"], tk_range_h=[round((t_rx[0] - eH["toe_abs"]) / 3600, 2), round((t_rx[-1] - eH["toe_abs"]) / 3600, 2)]),
                               eph_R=dict(src=eR["src"], iode=eR["iode"], tk_range_h=[round((t_rx[0] - eR["toe_abs"]) / 3600, 2), round((t_rx[-1] - eR["toe_abs"]) / 3600, 2)]),
                               el_H=[round(float(elH.min()), 1), round(float(elH.max()), 1)],
                               el_R=[round(float(elR.min()), 1), round(float(elR.max()), 1)],
                               tropo_diff_swing_m=round(float(tropo.max() - tropo.min()), 2),
                               rel_swing_m=round(swing, 2),
                               rel_rate_max_mm_s=round(float(np.abs(np.diff(Rreg) / np.diff(t_rx)).max() * 1000), 2),
                               null_swing_m=round(float(Rnull.max() - Rnull.min()), 2),
                               doppler_sign_slope=round(slope, 4),
                               rms_repaired_level_m=round(rms_rep, 3),
                               inc={k: {kk: (round(vv, 4) if isinstance(vv, float) else vv) for kk, vv in v.items()} for k, v in inc.items()},
                               level={k: {kk: (round(vv, 4) if isinstance(vv, float) else vv) for kk, vv in v.items()} for k, v in lv.items()})
                    results.append(res)
                    plots_data.append((res, tk, Dplot, Rk, b[0], r_lin, keep_lin))
                    f = inc["lin"]; q = inc["quad"]
                    print("G%02d/G%02d %s %.2fh n=%d el_H %s el_R %s swing %5.1fm | a_inc lin %6.3f+/-%.3f quad %6.3f+/-%.3f | null %6.3f+/-%.3f | excl %.1f%% core %.3fm neff %.0f dt_rx %.2fs | a_lvl %.2f+/-%.2f | sign %.3f"
                          % (H, R, res["arc"][0], res["arc_h"], res["n"], res["el_H"], res["el_R"], swing, f["a"], f["a_err"], q["a"], q["a_err"],
                             inc["null_lin"]["a_null"], inc["null_lin"]["a_null_err"], 100 * f["frac_excluded"], f["core_sigma_m"], f["n_eff"], f["beta"][2],
                             lv["lin_tt"]["a"], lv["lin_tt"]["a_err"], slope))

    # ---- best ref per H arc (primary refs preferred), combination
    def best_of(rs):
        best = {}
        for r in rs:
            key = (r["H"], r["arc"][0][:13])
            cur = best.get(key)
            score = (r["arc_h"], -r["inc"]["lin"]["a_err"])
            if cur is None or score > (cur["arc_h"], -cur["inc"]["lin"]["a_err"]):
                best[key] = r
        return sorted(best.values(), key=lambda r: (r["inc"]["lin"]["a_err"] if np.isfinite(r["inc"]["lin"]["a_err"]) else 1e9))
    prim = best_of([r for r in results if r["ref_class"] in ("primary", "secondary")])
    allb = best_of(results)
    def combine(rs, key="lin", field="a"):
        if not rs:
            return None
        rs = [r for r in rs if r["inc"][key].get("status") == "ok" and np.isfinite(r["inc"][key][field + "_err"])]
        if not rs:
            return None
        a = np.array([r["inc"][key][field] for r in rs]); s = np.array([r["inc"][key][field + "_err"] for r in rs])
        w = 1 / s ** 2; am = (w * a).sum() / w.sum(); ae = 1 / math.sqrt(w.sum())
        chi2 = float((((a - am) / s) ** 2).sum()); dof = max(len(a) - 1, 1)
        sf = math.sqrt(max(chi2 / dof, 1))
        return dict(a=round(am, 4), err=round(ae, 4), n_arcs=len(a), chi2=round(chi2, 1), dof=dof, err_scaled=round(ae * sf, 4),
                    median_a=round(float(np.median(a)), 4), sig_a0=round(am / ae, 1), sig_a1=round((am - 1) / ae, 1),
                    sig_a0_scaled=round(am / (ae * sf), 1), sig_a1_scaled=round((am - 1) / (ae * sf), 1))
    combined = {}
    for label, rs in (("lowe_ref_best", prim), ("all_best", allb), ("night_best", [r for r in allb if r["local_hour_start"] >= 20 or r["local_hour_start"] < 5])):
        for key, field in (("lin", "a"), ("quad", "a"), ("lin_nott", "a"), ("null_lin", "a_null"), ("null_quad", "a_null")):
            c = combine(rs, key, field)
            if c:
                combined["%s/%s" % (label, key)] = c
                print("COMBINED %-24s a=%8.4f +/- %.4f (scaled %.4f; chi2 %.1f/%d; n=%d; median %.3f)  sig(a-0)=%.1f sig(a-1)=%.1f [scaled %.1f / %.1f]"
                      % ("%s/%s" % (label, key), c["a"], c["err"], c["err_scaled"], c["chi2"], c["dof"], c["n_arcs"], c["median_a"], c["sig_a0"], c["sig_a1"], c["sig_a0_scaled"], c["sig_a1_scaled"]))

    # ---- plots
    plotted = []
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        order = sorted(plots_data, key=lambda x: x[0]["inc"]["lin"]["a_err"])
        done = set()
        for (r, tk, Dplot, Rk, a_fit, r_lin, keep_lin) in order:
            key = (r["H"], r["arc"][0][:13])
            if key in done or len(done) >= 6:
                continue
            done.add(key)
            fig, ax = plt.subplots(2, 1, figsize=(8, 6.4), sharex=False)
            ax[0].plot(tk, Dplot, ".", ms=2.5, color="#555", label="D(t): step-bridged carrier SD minus fitted slope/time-tag")
            ax[0].plot(tk, Rk, "-", color="#d62728", lw=1.6, label="a = 1 (IS-GPS-200 relativistic term)")
            ax[0].plot(tk, a_fit * Rk, "--", color="#1f77b4", lw=1.2, label="fit a = %.3f +/- %.3f" % (r["inc"]["lin"]["a"], r["inc"]["lin"]["a_err"]))
            ax[0].set_ylabel("metres"); ax[0].set_xlabel("hours from arc start"); ax[0].legend(fontsize=8)
            ax[0].set_title("%s - %s  %s UTC  %.2f h  e_H=%.4f  predicted swing %.1f m" % (r["H"], r["R"], r["arc"][0], r["arc_h"], r["e_H"], r["rel_swing_m"]), fontsize=10)
            ax[1].plot(np.arange(len(r_lin))[keep_lin], r_lin[keep_lin], ".", ms=2, color="#1f77b4", label="kept increment residuals (sigma %.3f m)" % r["inc"]["lin"]["core_sigma_m"])
            ax[1].plot(np.arange(len(r_lin))[~keep_lin], np.clip(r_lin[~keep_lin], -3, 3), "x", ms=4, color="#d62728", label="excluded step increments (clipped to +/-3 m), %.1f%%" % (100 * r["inc"]["lin"]["frac_excluded"]))
            ax[1].set_ylabel("metres per 10 s"); ax[1].set_xlabel("increment index"); ax[1].legend(fontsize=8); ax[1].set_ylim(-3.2, 3.2)
            fn = os.path.join(SCR2, "relativity_carrier_%s_%s_%s.png" % (r["H"], r["R"], r["arc"][0].replace(" ", "T").replace(":", "")))
            fig.tight_layout(); fig.savefig(fn, dpi=110); plt.close(fig); plotted.append(fn)
    except Exception as ex:
        print("plot skipped:", ex)

    out = dict(generated=datetime.datetime.utcnow().isoformat() + "Z",
               source="observations/telemetry_log.jsonl (carrier_cycles, 10 s; %d GPS samples 2026-08-25..09-04) + BKG BRDC00WRD daily RINEX DOY 237-246 + observations/brdc_latest.rnx" % len(d["prn"]),
               constants=dict(F=F_REL, mu=MU, c=C, f_L1=F_L1, ztd_m=ZTD_M, leap_s=LEAP, threshold_m=THR_M),
               sign_convention=("L = -(cycles_H - cycles_R) * c/f_L1 [m]. carrier_cycles rate == doppler_hz (archive check: ratio 0.9996, corr 0.9992); "
                                "tracker Doppler is positive for closing range, so -lambda*cycles tracks +range: L = (rho_H-rho_R) - c(dt_sv_H-dt_sv_R) + tropo - iono + const. "
                                "Empirical check per arc: doppler_sign_slope = slope of d(L)/dt against d(M)/dt, expected +1. "
                                "M uses the broadcast polynomial clock only; Rreg = -c(dt_r_H - dt_r_R) with dt_r = F e sqrtA sin(Ek); a = +1 <=> IS-GPS-200 sign."),
               estimator=("first differences of D at the native 10 s cadence (8-12 s only), OLS with iterative fixed |residual| <= %.2f m exclusion (removes per-channel carrier steps at USB drop holes); "
                          "nuisances: slope, receiver time-tag offset dt_rx via d(rdot_H - rdot_R), optional quadratic; errors from kept-residual scatter x n/n_eff(ACF). "
                          "Level-domain fits (ACF n_eff, with dt_rx nuisance) are reported for comparison only." % THR_M),
               selection=dict(hi_e_min=HI_E_MIN, ref_e_max_primary=REF_E_MAX_PRIMARY, ref_e_max_secondary=REF_E_MAX_SECONDARY, min_arc_h=MIN_ARC_H, min_el_deg=MIN_EL_DEG,
                              hi_sats=["G%02d" % p for p in hi], ref_sats="any PRN with differential swing >= %.1f m; class primary e<=%.4f, secondary e<=%.4f, else any_e" % (MIN_DIFF_SWING_M, REF_E_MAX_PRIMARY, REF_E_MAX_SECONDARY), tcut_utc=datetime.datetime.utcfromtimestamp(TCUT).isoformat(), threshold_m=THR_M),
               n_pair_arcs_fitted=len(results), n_best_lowe_ref=len(prim), n_best_all=len(allb),
               combined=combined, best_lowe_ref=prim, best_all=allb, all_pair_fits=results, plots=plotted)
    json.dump(out, open(os.path.join(SCR2, "relativity_carrier.json"), "w"), indent=1)
    print("wrote relativity_carrier.json; pair-arcs %d; plots %d" % (len(results), len(plotted)))

if __name__ == "__main__":
    main()
