#!/usr/bin/env python3
"""Phase A null test: is the GPS relativistic periodic clock term present in
the per-satellite residuals of clock_bias.jsonl?  Pure python, read-only."""
import json, math, sys, os
from collections import defaultdict

OBS = "/Volumes/Radiator 8TB/gnss/observations"
CB = os.path.join(OBS, "clock_bias.jsonl")
RNX = os.path.join(OBS, "brdc_latest.rnx")
OUT = "/private/tmp/claude-501/-Volumes-Radiator-8TB/43a4bc6f-9c07-4172-8270-58a5d832053e/scratchpad/relativity_null_test.json"

C = 299792458.0
MU = 3.986005e14
F_REL = -4.442807633e-10
GPS_UNIX0 = 315964800
LEAP = 18
WEEK = 604800.0
T_ORB = 11 * 3600 + 58 * 60  # 43080 s
MIN_SAMPLES = 1500
DECOR = 60.0
REJ_CLIP_M = 1000.0  # 'rej' rows can be ~1e6 m; clip to the gate ceiling

# ---------------------------------------------------------------- RINEX parse
def fnum(s):
    s = s.strip().replace("D", "E").replace("d", "e")
    return float(s) if s else 0.0

def parse_rinex_gps(path):
    eph = defaultdict(list)
    with open(path) as f:
        for line in f:
            if "END OF HEADER" in line:
                break
        lines = f.read().splitlines()
    i = 0
    while i < len(lines):
        l = lines[i]
        if not l.startswith("G"):
            i += 1
            continue
        rec = lines[i:i + 8]
        if len(rec) < 8:
            break
        try:
            prn = l[0:3]
            yy, mo, dd, hh, mi, ss = int(l[4:8]), int(l[9:11]), int(l[12:14]), int(l[15:17]), int(l[18:20]), int(l[21:23])
            def fld(k, j):
                return fnum(rec[k][4 + 19 * j:4 + 19 * (j + 1)])
            e = dict(prn=prn, af0=fnum(l[23:42]), af1=fnum(l[42:61]), af2=fnum(l[61:80]),
                     iode=fld(1, 0), crs=fld(1, 1), delta_n=fld(1, 2), m0=fld(1, 3),
                     cuc=fld(2, 0), ecc=fld(2, 1), cus=fld(2, 2), sqrt_a=fld(2, 3),
                     toe=fld(3, 0), week=fld(5, 2), health=fld(6, 1))
            e["toe_abs"] = e["week"] * WEEK + e["toe"]
            if e["sqrt_a"] > 1000 and e["health"] == 0.0:
                eph[prn].append(e)
        except Exception as ex:
            print("rinex parse skip", i, ex, file=sys.stderr)
        i += 8
    return eph

def kepler_E(M, e):
    E = M
    for _ in range(12):
        E = E - (E - e * math.sin(E) - M) / (1 - e * math.cos(E))
    return E

def wrap_tk(tk):
    if tk > 302400: tk -= WEEK
    elif tk < -302400: tk += WEEK
    return tk

def dt_rel(e, gps_abs):
    """Relativistic periodic clock term (s) from a broadcast record at absolute GPS time."""
    tow = gps_abs % WEEK
    tk = wrap_tk(tow - e["toe"])
    A = e["sqrt_a"] ** 2
    n = math.sqrt(MU / A ** 3) + e["delta_n"]
    M = e["m0"] + n * tk
    E = kepler_E(M, e["ecc"])
    return F_REL * e["ecc"] * e["sqrt_a"] * math.sin(E)

def nearest_eph(recs, gps_abs):
    return min(recs, key=lambda r: abs(r["toe_abs"] - gps_abs))

# ---------------------------------------------------------------- load rows
def load_rows():
    rows = []
    with open(CB) as f:
        for line in f:
            if '"residuals"' not in line:
                continue
            try:
                d = json.loads(line)
            except Exception:
                continue
            rows.append(d)
    rows.sort(key=lambda d: d["epoch"])
    return rows

# ---------------------------------------------------------------- linear algebra
def lstsq(X, y):
    """Normal-equation LS. Returns beta, cov_unit (X^T X)^-1, sse, n, p."""
    n, p = len(X), len(X[0])
    XtX = [[sum(X[k][i] * X[k][j] for k in range(n)) for j in range(p)] for i in range(p)]
    Xty = [sum(X[k][i] * y[k] for k in range(n)) for i in range(p)]
    inv = invert(XtX)
    beta = [sum(inv[i][j] * Xty[j] for j in range(p)) for i in range(p)]
    sse = sum((y[k] - sum(X[k][j] * beta[j] for j in range(p))) ** 2 for k in range(n))
    return beta, inv, sse, n, p

def invert(A):
    n = len(A)
    M = [row[:] + [1.0 if i == j else 0.0 for j in range(n)] for i, row in enumerate(A)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[piv] = M[piv], M[c]
        d = M[c][c]
        if abs(d) < 1e-300:
            raise ZeroDivisionError("singular")
        M[c] = [v / d for v in M[c]]
        for r in range(n):
            if r != c and M[r][c] != 0.0:
                f = M[r][c]
                M[r] = [a - f * b for a, b in zip(M[r], M[c])]
    return [row[n:] for row in M]

def n_eff_bins(ts, width=DECOR):
    return len({int(t // width) for t in ts})

def acf_tau(ts, resid, width=DECOR, maxlag=90):
    """Integrated autocorrelation time (in units of `width` bins) of the fit
    residual, from 60 s bin means; summed to the first negative lag."""
    bins = defaultdict(list)
    for t, r in zip(ts, resid):
        bins[int(t // width)].append(r)
    keys = sorted(bins)
    if len(keys) < 8:
        return 1.0, len(keys)
    k0 = keys[0]
    series = {k - k0: sum(v) / len(v) for k, v in bins.items()}
    m = sum(series.values()) / len(series)
    var = sum((v - m) ** 2 for v in series.values()) / len(series)
    if var <= 0:
        return 1.0, len(keys)
    tau = 1.0
    for lag in range(1, maxlag + 1):
        pairs = [(series[k], series[k + lag]) for k in series if (k + lag) in series]
        if len(pairs) < 10:
            break
        rho = sum((a - m) * (b - m) for a, b in pairs) / (len(pairs) * var)
        if rho <= 0:
            break
        tau += 2 * rho
    return tau, len(keys)

def fit_model(ts, rs, ss, t0, quad=False):
    """r = a*s + b + d*(t-t0) [+ q*(t-t0)^2].  Errors: iid, 60 s decorrelation,
    and ACF-integrated correlation time of the fit residual."""
    X = [[s, 1.0, (t - t0) / 3600.0] + ([((t - t0) / 3600.0) ** 2] if quad else []) for t, s in zip(ts, ss)]
    beta, inv, sse, n, p = lstsq(X, rs)
    rms = math.sqrt(sse / max(n - p, 1))
    sig = rms * math.sqrt(inv[0][0])
    neff = n_eff_bins(ts)
    sig_c = sig * math.sqrt(max(n - p, 1) / max(neff - p, 1))
    resid = [y - sum(x * b for x, b in zip(row, beta)) for row, y in zip(X, rs)]
    tau, nb = acf_tau(ts, resid)
    neff_acf = nb / tau
    sig_acf = sig * math.sqrt(max(n - p, 1) / max(neff_acf - p, 0.5))
    return dict(a=beta[0], b=beta[1], d_m_per_h=beta[2], sigma_iid=sig, sigma_60s=sig_c,
                sigma_acf=sig_acf, acf_tau_min=tau, n_eff_acf=neff_acf,
                rms_m=rms, n=n, n_eff_60s=neff)

def fit_sinusoid(ts, rs, t0, period=T_ORB):
    w = 2 * math.pi / period
    X = [[math.cos(w * (t - t0)), math.sin(w * (t - t0)), 1.0, (t - t0) / 3600.0] for t in ts]
    beta, inv, sse, n, p = lstsq(X, rs)
    rms = math.sqrt(sse / max(n - p, 1))
    A, B = beta[0], beta[1]
    amp = math.hypot(A, B)
    # amplitude error: propagate via gradient
    if amp > 0:
        gA, gB = A / amp, B / amp
        var = rms ** 2 * (gA * gA * inv[0][0] + 2 * gA * gB * inv[0][1] + gB * gB * inv[1][1])
    else:
        var = rms ** 2 * inv[0][0]
    sig = math.sqrt(max(var, 0))
    neff = n_eff_bins(ts)
    sig_c = sig * math.sqrt(max(n - p, 1) / max(neff - p, 1))
    resid = [y - sum(x * b for x, b in zip(row, beta)) for row, y in zip(X, rs)]
    tau, nb = acf_tau(ts, resid)
    sig_acf = sig * math.sqrt(max(n - p, 1) / max(nb / tau - p, 0.5))
    return dict(amp_m=amp, phase_rad=math.atan2(B, A), sigma_iid=sig, sigma_60s=sig_c,
                sigma_acf=sig_acf, acf_tau_min=tau, rms_m=rms, n=n)

def robust_sigma(v):
    v = sorted(v)
    n = len(v)
    med = v[n // 2]
    dev = sorted(abs(x - med) for x in v)
    return 1.4826 * dev[n // 2], med

# ---------------------------------------------------------------- main
def main():
    eph = parse_rinex_gps(RNX)
    rows = load_rows()
    if not rows:
        print("no residual rows"); return
    t_first, t_last = rows[0]["epoch"], rows[-1]["epoch"]
    t0 = t_first
    # gaps
    gaps = []
    for a, b in zip(rows, rows[1:]):
        dt = b["epoch"] - a["epoch"]
        if dt > 60:
            gaps.append(dict(start=a["epoch"], end=b["epoch"], seconds=round(dt, 1)))
    cadence = sorted(b["epoch"] - a["epoch"] for a, b in zip(rows, rows[1:]))
    med_cadence = cadence[len(cadence) // 2]

    # per-PRN sample collection: (t, r, w, fresh, ok, s_raw, s_proj)
    per = defaultdict(list)
    n_pred = 0
    gps_ok_resid = []
    for d in rows:
        t = d["epoch"]
        gps_abs = t - GPS_UNIX0 + LEAP
        res = d["residuals"]
        # signature per member (non-GPS -> 0)
        sig = {}
        for lab, r, w, fr, ok in res:
            if lab.startswith("G") and lab in eph:
                sig[lab] = -C * dt_rel(nearest_eph(eph[lab], gps_abs), gps_abs)
            else:
                sig[lab] = 0.0
        # clock projection over the accepted (ok) set, weighted as in solve_clock_only
        okm = [(lab, w) for lab, r, w, fr, ok in res if ok == "ok"]
        sw = sum(w for _, w in okm)
        smean = sum(sig[lab] * w for lab, w in okm) / sw if sw > 0 else 0.0
        for lab, r, w, fr, ok in res:
            if not lab.startswith("G") or lab not in eph:
                continue
            if fr == "pred": n_pred += 1
            per[lab].append((t, r, w, fr, ok, sig[lab], sig[lab] - smean))
            if ok == "ok": gps_ok_resid.append(r)

    gps_sig, gps_med = robust_sigma(gps_ok_resid)
    gps_rms = math.sqrt(sum(x * x for x in gps_ok_resid) / len(gps_ok_resid))

    results = {}
    table = []
    for prn in sorted(per):
        v = per[prn]
        if len(v) < MIN_SAMPLES:
            continue
        rec = nearest_eph(eph[prn], (t_first + t_last) / 2 - GPS_UNIX0 + LEAP)
        ecc, sqa = rec["ecc"], rec["sqrt_a"]
        peak = abs(F_REL * ecc * sqa) * C
        ok = [x for x in v if x[4] == "ok"]
        fresh_ok = [x for x in ok if x[3] == "fresh"]
        allc = [(x[0], max(-REJ_CLIP_M, min(REJ_CLIP_M, x[1])), x[2], x[3], x[4], x[5], x[6]) for x in v]
        ts = [x[0] for x in ok]; rs = [x[1] for x in ok]
        s_raw = [x[5] for x in ok]; s_proj = [x[6] for x in ok]
        hours = n_eff_bins(ts) * DECOR / 3600.0
        span_h = (ts[-1] - ts[0]) / 3600.0
        # how much of the signature survives the clock solve
        proj_frac = (sum(a * b for a, b in zip(s_proj, s_raw)) / sum(a * a for a in s_raw)) if any(s_raw) else 0.0
        sig_range = (min(s_raw), max(s_raw))
        r = dict(prn=prn, e=ecc, sqrt_a=sqa, predicted_peak_m=peak,
                 n_all=len(v), n_ok=len(ok), n_rej=len(v) - len(ok), n_fresh_ok=len(fresh_ok),
                 hours_ok_60s_bins=hours, span_hours=span_h,
                 signature_range_m=sig_range, signature_swing_m=sig_range[1] - sig_range[0],
                 clock_projection_retained_frac=proj_frac,
                 passes=count_passes(ts))
        try:
            r["fit_ok_raw_s"] = fit_model(ts, rs, s_raw, t0)
            r["fit_ok_proj_s"] = fit_model(ts, rs, s_proj, t0)
            r["fit_ok_proj_s_quad"] = fit_model(ts, rs, s_proj, t0, quad=True)
            if len(fresh_ok) >= 300:
                r["fit_fresh_ok_proj_s"] = fit_model([x[0] for x in fresh_ok], [x[1] for x in fresh_ok], [x[6] for x in fresh_ok], t0)
            ta = [x[0] for x in allc]; ra = [x[1] for x in allc]
            r["fit_all_raw_s"] = fit_model(ta, ra, [x[5] for x in allc], t0)
            r["fit_all_proj_s"] = fit_model(ta, ra, [x[6] for x in allc], t0)
            r["free_sinusoid_ok"] = fit_sinusoid(ts, rs, t0)
        except ZeroDivisionError as ex:
            r["error"] = str(ex)
        rob, med = robust_sigma(rs)
        r["resid_robust_sigma_m"] = rob; r["resid_median_m"] = med
        results[prn] = r
        table.append(r)

    # combined a (inverse-variance weighted) over decisive PRNs
    def combine(key, sigkey):
        num = den = 0.0
        for r in table:
            f = r.get(key)
            if not f: continue
            s = f[sigkey]
            if s <= 0: continue
            num += f["a"] / s ** 2; den += 1 / s ** 2
        return (num / den, math.sqrt(1 / den)) if den > 0 else (None, None)
    comb = {}
    for key in ("fit_ok_raw_s", "fit_ok_proj_s", "fit_ok_proj_s_quad", "fit_fresh_ok_proj_s", "fit_all_raw_s", "fit_all_proj_s"):
        a1, s1 = combine(key, "sigma_iid"); a2, s2 = combine(key, "sigma_60s"); a3, s3 = combine(key, "sigma_acf")
        c = dict(a_ivw_iid=a1, sigma_iid=s1, a_ivw_60s=a2, sigma_60s=s2, a_ivw_acf=a3, sigma_acf=s3)
        # consistency of per-PRN a's under the ACF error, Birge-ratio inflation
        fs = [r[key] for r in table if key in r]
        if a3 is not None and len(fs) > 1:
            chi2 = sum(((f["a"] - a3) / f["sigma_acf"]) ** 2 for f in fs)
            dof = len(fs) - 1
            birge = math.sqrt(max(chi2 / dof, 1.0))
            c.update(chi2_acf=chi2, dof=dof, birge_ratio=birge, sigma_acf_birge=s3 * birge)
            av = [f["a"] for f in fs]
            m = sum(av) / len(av)
            sd = math.sqrt(sum((x - m) ** 2 for x in av) / (len(av) - 1))
            c.update(a_unweighted_mean=m, a_unweighted_std=sd, a_unweighted_sem=sd / math.sqrt(len(av)))
            # signal-strength weighted: weight by signature swing^2 (metres of leverage)
            sw = [(r[key]["a"], r["signature_swing_m"] ** 2) for r in table if key in r]
            c["a_swing2_weighted"] = sum(a * w for a, w in sw) / sum(w for _, w in sw)
        comb[key] = c

    # Phase B prediction for G07
    g07 = nearest_eph(eph["G07"], t_last - GPS_UNIX0 + LEAP) if "G07" in eph else None
    phaseb = {}
    if g07:
        peak = abs(F_REL * g07["ecc"] * g07["sqrt_a"]) * C
        pass_h = 5.0
        n_iid = pass_h * 3600 / med_cadence
        n_60 = pass_h * 3600 / DECOR
        # over a ~5 h arc of a 12 h sinusoid, after offset+trend removal, the
        # detectable RMS of the signature is roughly amp/sqrt(2) * ~0.5..0.7
        # depending on phase; use 0.6*amp/sqrt(2) as the effective signal rms.
        eff_rms = 0.6 * peak / math.sqrt(2)
        taus = sorted(r["fit_ok_proj_s"]["acf_tau_min"] for r in table if "fit_ok_proj_s" in r)
        tau_med = taus[len(taus) // 2]
        n_acf = n_60 / tau_med
        # 60 s-binned residual scatter (what the ACF error actually sees)
        g07r = results.get("G07", {})
        phaseb = dict(g07_e=g07["ecc"], g07_sqrt_a=g07["sqrt_a"], predicted_peak_m=peak,
                      gps_resid_robust_sigma_m=gps_sig, gps_resid_rms_m=gps_rms,
                      median_cadence_s=med_cadence, assumed_pass_hours=pass_h,
                      effective_signal_rms_m_after_detrend=eff_rms,
                      snr_amplitude_iid=eff_rms * math.sqrt(n_iid) / gps_sig,
                      snr_amplitude_60s_decorrelated=eff_rms * math.sqrt(n_60) / gps_sig,
                      median_acf_tau_min=tau_med,
                      snr_amplitude_acf_decorrelated=eff_rms * math.sqrt(n_acf) / gps_sig,
                      g07_phase_a_a_proj=g07r.get("fit_ok_proj_s", {}).get("a"),
                      g07_phase_a_sigma_acf=g07r.get("fit_ok_proj_s", {}).get("sigma_acf"),
                      g07_phase_a_resid_sigma_m=g07r.get("resid_robust_sigma_m"),
                      note="SNR = effective signal rms * sqrt(N) / per-epoch robust sigma; G07 solved-around so no clock absorption (retained frac 1.0). The ACF figure is the realistic one: residuals are correlated on ~tau minutes.")

    out = dict(
        generated="2026-09-04", source=CB, rinex=RNX, rows_with_residuals=len(rows),
        first_epoch=t_first, last_epoch=t_last, span_hours=(t_last - t_first) / 3600.0,
        median_cadence_s=med_cadence, gaps_over_60s=gaps,
        total_gap_hours=sum(g["seconds"] for g in gaps) / 3600.0,
        gps_pred_flag_samples=n_pred,
        gps_ok_residual_robust_sigma_m=gps_sig, gps_ok_residual_median_m=gps_med, gps_ok_residual_rms_m=gps_rms,
        sign_convention="residual = observed - modelled; model pseudorange = range - c*dt_sv; s(t) = -c*dt_r. Spurious term -> a=-1; correct -> a=0; applied with wrong sign vs physics -> a=-2 (twice the term left); a=+1 would mean physics twice the applied term.",
        clock_projection="proj regressor = s_i - sum(w_j s_j)/sum(w_j) over the epoch's accepted members (exactly what solve_clock_only removes); raw regressor ignores absorption.",
        per_prn=results, combined=comb, phase_b_g07=phaseb,
    )
    with open(OUT, "w") as f:
        json.dump(out, f, indent=1)

    # ---- console table
    print(f"rows={len(rows)} span={out['span_hours']:.2f} h cadence={med_cadence:.2f} s gaps>60s={len(gaps)} ({out['total_gap_hours']:.2f} h)")
    for g in gaps: print("  gap", g)
    print(f"GPS ok residuals: robust sigma={gps_sig:.1f} m  median={gps_med:.1f}  rms={gps_rms:.1f} m; pred-flag samples={n_pred}")
    hdr = "PRN    e      peak_m swing_m hours span_h pass n_ok n_frsh n_rej ret | a_raw ±iid ±60s ±acf | a_proj ±iid ±60s ±acf tau_min | a_quad ±acf | a_fresh ±acf | a_all ±acf | sinA_m ±60s ±acf | rsig"
    print(hdr)
    for r in table:
        f1, f2, f3, f4, fs = r["fit_ok_raw_s"], r["fit_ok_proj_s"], r["fit_all_proj_s"], r["fit_ok_proj_s_quad"], r["free_sinusoid_ok"]
        f5 = r.get("fit_fresh_ok_proj_s")
        f5s = f"{f5['a']:+7.2f} {f5['sigma_acf']:6.2f}" if f5 else "      -      -"
        print(f"{r['prn']} {r['e']:.5f} {r['predicted_peak_m']:5.2f} {r['signature_swing_m']:6.2f} {r['hours_ok_60s_bins']:5.2f} {r['span_hours']:5.2f} {r['passes']:3d} {r['n_ok']:5d} {r['n_fresh_ok']:5d} {r['n_rej']:5d} {r['clock_projection_retained_frac']:4.2f} | "
              f"{f1['a']:+7.2f} {f1['sigma_iid']:5.2f} {f1['sigma_60s']:6.2f} {f1['sigma_acf']:6.2f} | {f2['a']:+7.2f} {f2['sigma_iid']:5.2f} {f2['sigma_60s']:6.2f} {f2['sigma_acf']:6.2f} {f2['acf_tau_min']:5.1f} | "
              f"{f4['a']:+7.2f} {f4['sigma_acf']:6.2f} | {f5s} | {f3['a']:+7.2f} {f3['sigma_acf']:6.2f} | "
              f"{fs['amp_m']:8.1f} {fs['sigma_60s']:7.1f} {fs['sigma_acf']:7.1f} | {r['resid_robust_sigma_m']:.1f}")
    for k, v in comb.items():
        print("combined", k, json.dumps({kk: (round(vv, 3) if isinstance(vv, float) else vv) for kk, vv in v.items()}))
    print("phaseB", json.dumps(phaseb, indent=1))

def count_passes(ts, gap=1800):
    p = 1
    for a, b in zip(ts, ts[1:]):
        if b - a > gap: p += 1
    return p

if __name__ == "__main__":
    main()
