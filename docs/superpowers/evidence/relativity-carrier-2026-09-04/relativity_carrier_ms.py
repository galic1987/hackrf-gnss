#!/usr/bin/env python3
"""Multi-satellite carrier-phase measurement of the GPS relativistic periodic
clock term (per-epoch common-mode elimination; increment domain).

Per window (a merged span of high-e slip-free arcs), all GPS satellites with
slip-free carrier segments enter.  For sat s at tagged epoch t:
    D_s = L_s - M_s,  L_s = -cycles_s * lambda,
    M_s = rho_s(t_tag + delta0) - c*poly_s + ZTD/sin(el_s)
First differences between consecutive samples (8-12 s) of one segment:
    dD_s,k = dClock_k + rdot_s * g_k + a * dR_s,k + slope_seg * dt
             (+ ddelta0 * d(rdot_s))  + eps
dClock_k (receiver clock, common) and g_k (common time-tag / USB-hole
increment, proportional to each sat's range-rate) are free per epoch pair
and eliminated exactly by per-epoch projection (Frisch-Waugh-Lovell),
which needs >= 3 satellites per epoch.  Common-mode hole steps of ANY size
are thereby removed; per-channel cycle slips remain and are handled by
excluding epoch pairs whose max |residual| exceeds THR.  delta0 (tagged
time -> true receive time) is solved and the model re-evaluated at
t_tag + delta0 (2 passes) so second-order range terms are exact.
a = +1 <=> IS-GPS-200 relativistic term with its published sign
(regressor R_s = -c * F e sqrtA sin Ek).
"""
import glob, json, math, os, sys, datetime
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rc_core import *

SCR = os.path.dirname(os.path.abspath(__file__))
TCUT = datetime.datetime(2026, 8, 30, 20, 0, tzinfo=datetime.timezone.utc).timestamp()
THR_M = 0.06
THR_VARIANTS = [0.04, 0.10, 0.20]
MIN_SEG_SAMPLES = 60
MIN_ARC_H = 1.25
DT_MIN, DT_MAX = 8.0, 12.0
LAT, LON = 39.0029556, -77.6051478
EPH_MAX_AGE_H = 2.5
MIN_WINDOW_H = 3.0

_site = None; _eph = None; _d = None; _segs = {}
_KLOB = {}   # doy -> (alpha[4], beta[4])
USE_KLOBUCHAR = True

def load_klobuchar():
    import re
    for fn in glob.glob(os.path.join(SCR, "brdc", "BRDC00WRD_R_2026*_01D_MN.rnx")):
        doy = int(os.path.basename(fn)[16:19])
        al = be = None
        with open(fn) as f:
            for line in f:
                if "END OF HEADER" in line:
                    break
                if line.startswith("GPSA"):
                    al = [float(line[5 + 12 * i:5 + 12 * (i + 1)].replace("D", "E")) for i in range(4)]
                elif line.startswith("GPSB"):
                    be = [float(line[5 + 12 * i:5 + 12 * (i + 1)].replace("D", "E")) for i in range(4)]
        if al and be:
            _KLOB[doy] = (al, be)

def klobuchar_m(t_gps_abs, el_deg, az_deg, lat_deg, lon_deg):
    """IS-GPS-200 20.3.3.5.2.5 single-frequency L1 ionospheric delay [m], vectorized."""
    doy = ((t_gps_abs + GPS_UNIX0 - LEAP) / 86400.0)
    doy = np.array([datetime.datetime.utcfromtimestamp(v * 86400.0).timetuple().tm_yday for v in doy])
    out = np.zeros(len(t_gps_abs))
    for dy in np.unique(doy):
        key = dy if dy in _KLOB else (min(_KLOB, key=lambda k: abs(k - dy)) if _KLOB else None)
        if key is None:
            return out
        al, be = _KLOB[key]
        mm = doy == dy
        E = np.radians(el_deg[mm]) / math.pi; A = np.radians(az_deg[mm])
        phi_u = lat_deg / 180.0; lam_u = lon_deg / 180.0
        psi = 0.0137 / (E + 0.11) - 0.022
        phi_i = np.clip(phi_u + psi * np.cos(A), -0.416, 0.416)
        lam_i = lam_u + psi * np.sin(A) / np.cos(phi_i * math.pi)
        phi_m = phi_i + 0.064 * np.cos((lam_i - 1.617) * math.pi)
        t = np.mod(4.32e4 * lam_i + np.mod(t_gps_abs[mm], WEEK), 86400.0)
        F = 1.0 + 16.0 * (0.53 - E) ** 3
        AMP = np.maximum(al[0] + al[1] * phi_m + al[2] * phi_m ** 2 + al[3] * phi_m ** 3, 0.0)
        PER = np.maximum(be[0] + be[1] * phi_m + be[2] * phi_m ** 2 + be[3] * phi_m ** 3, 72000.0)
        x = 2 * math.pi * (t - 50400.0) / PER
        T = np.where(np.abs(x) < 1.57, F * (5e-9 + AMP * (1 - x ** 2 / 2 + x ** 4 / 24)), F * 5e-9)
        out[mm] = C * T
    return out

def azimuth_deg(prot, site, lat, lon):
    d = prot - site
    sl, cl = math.sin(math.radians(lat)), math.cos(math.radians(lat))
    so, co = math.sin(math.radians(lon)), math.cos(math.radians(lon))
    east = -so * d[:, 0] + co * d[:, 1]
    north = -sl * co * d[:, 0] - sl * so * d[:, 1] + cl * d[:, 2]
    return np.degrees(np.arctan2(east, north))

def init():
    global _site, _eph, _d
    _site = np.array(geocorrector_helper.get_site_ecef())
    _eph = parse_rinex_gps(sorted(glob.glob(os.path.join(SCR, "brdc", "*.rnx"))))
    _d = load_carrier()
    load_klobuchar()
    print("Klobuchar coefficient days:", sorted(_KLOB))
    for p in range(1, 33):
        _segs[p] = segments_for(_d, p)

def sat_table(p):
    """All locked samples of PRN p sorted by epoch, exact-duplicate epochs removed
    (conflicting duplicates dropped), with slip / lock-reset flags kept."""
    m = (_d["prn"] == p) & (_d["lock_s"] > 0)
    ep = _d["sat_epoch"][m]; cy = _d["cycles"][m]; lk = _d["lock_s"][m]; sl = _d["slip"][m]
    o = np.argsort(ep, kind="stable"); ep, cy, lk, sl = ep[o], cy[o], lk[o], sl[o]
    keep = np.ones(len(ep), bool)
    for i in range(1, len(ep)):
        if ep[i] == ep[i - 1]:
            keep[i] = False
            if cy[i] != cy[i - 1]:
                keep[i - 1] = False
    return ep[keep], cy[keep], lk[keep], sl[keep]

def window_obs(t0, t1, delta0=0.0):
    """Per-sample table for all GPS sats in [t0,t1]; model at t_tag+delta0.
    One 'segment' per PRN per window (slips are bridged: the slip increment
    is excluded, the smooth model continues)."""
    rows = []
    for p in range(1, 33):
        if p not in _eph:
            continue
        ep, cy, lk, sl = sat_table(p)
        m = (ep >= t0) & (ep <= t1)
        if m.sum() < MIN_SEG_SAMPLES:
            continue
        ep, cy, lk, sl = ep[m], cy[m], lk[m], sl[m]
        trx = ep - GPS_UNIX0 + LEAP + delta0
        # per-sample nearest broadcast record (|t - toe| minimal); record
        # switches are flagged as breaks so the level jump never enters an
        # increment. Samples > EPH_MAX_AGE_H from every toe are dropped.
        toes = np.array([r["toe_abs"] for r in _eph[p]])
        ridx = np.argmin(np.abs(trx[:, None] - toes[None, :]), axis=1)
        age_ok = np.abs(trx - toes[ridx]) <= EPH_MAX_AGE_H * 3600
        r = np.zeros(len(ep)); pc = np.zeros(len(ep)); rel = np.zeros(len(ep)); pos = np.zeros((len(ep), 3))
        r2 = np.zeros(len(ep)); r1 = np.zeros(len(ep)); E = np.zeros(len(ep)); Nn = np.zeros(len(ep))
        for ri in np.unique(ridx):
            e = _eph[p][ri]; mm = ridx == ri
            r[mm], pc[mm], rel[mm], pos[mm] = model_range(e, trx[mm], _site)
            r2[mm], _, _, _ = model_range(e, trx[mm] + 0.5, _site); r1[mm], _, _, _ = model_range(e, trx[mm] - 0.5, _site)
            _, E[mm], _, _ = sat_state(e, trx[mm])
            Nn[mm] = -C * F_REL * e["ecc"] * e["sqrt_a"] * np.cos(E[mm])
        e = _eph[p][ridx[len(ridx) // 2]]
        el = elevation_deg(pos, _site, LAT, LON)
        ok = (el >= MIN_EL_DEG) & age_ok
        if ok.sum() < MIN_SEG_SAMPLES:
            continue
        M = r - C * pc + ZTD_M / np.sin(np.radians(el))
        if USE_KLOBUCHAR and _KLOB:
            az = azimuth_deg(pos, _site, LAT, LON)
            M = M - klobuchar_m(trx, el, az, LAT, LON)     # carrier: phase ADVANCE, -I
        L = -cy * LAM
        R = -C * rel
        N = Nn
        switch = np.zeros(len(ep), bool); switch[1:] = ridx[1:] != ridx[:-1]
        mf = 1.0 / np.sqrt(1.0 - (6371.0 / (6371.0 + 350.0) * np.cos(np.radians(el))) ** 2)   # thin-shell (350 km) iono mapping
        brk = np.zeros(len(ep), bool)   # break BEFORE sample i (slip / lock reset / zeroing)
        brk[1:] = sl[1:] | (lk[1:] < lk[:-1]) | (lk[1:] < lk[:-1] + 0.75 * np.diff(ep)) | (np.abs(np.diff(cy)) / np.maximum(np.diff(ep), 1e-3) > RESEED_JUMP_HZ)
        brk |= switch
        for k in np.where(ok)[0]:
            rows.append((ep[k], p, p, L[k] - M[k], r2[k] - r1[k], R[k], N[k], el[k], e["ecc"], brk[k], mf[k]))
    a = np.array(rows, dtype=[("ep", float), ("prn", int), ("seg", int), ("D", float), ("rdot", float),
                              ("R", float), ("N", float), ("el", float), ("ecc", float), ("brk", bool), ("mf", float)])
    a.sort(order=["seg", "ep"])
    return a

def increments(a):
    """Consecutive-sample increments within a PRN; epoch pair keyed (ep0, ep1).
    'bad' marks increments across a flagged break (excluded a priori)."""
    out = []
    for seg in np.unique(a["seg"]):
        b = a[a["seg"] == seg]
        for i in range(len(b) - 1):
            dt = b["ep"][i + 1] - b["ep"][i]
            if DT_MIN <= dt <= DT_MAX:
                out.append((b["ep"][i + 1], b["ep"][i], b["prn"][i], seg, b["D"][i + 1] - b["D"][i],
                            0.5 * (b["rdot"][i + 1] + b["rdot"][i]), b["rdot"][i + 1] - b["rdot"][i],
                            b["R"][i + 1] - b["R"][i], b["N"][i + 1] - b["N"][i], dt, 0.5 * (b["ep"][i + 1] + b["ep"][i]), bool(b["brk"][i + 1]),
                            b["mf"][i + 1] - b["mf"][i], 0.5 * (b["mf"][i + 1] + b["mf"][i])))
    inc = np.array(out, dtype=[("k", float), ("k0", float), ("prn", int), ("seg", int), ("dD", float), ("rdot", float),
                               ("drdot", float), ("dR", float), ("dN", float), ("dt", float), ("tm", float), ("bad", bool), ("dmf", float), ("mf", float)])
    inc.sort(order=["k", "prn"])
    return inc

def _project(X, y, inc, keep, idx_inv, n_pairs, min_sats):
    """FWL projection of span{1, rdot} per epoch pair over KEPT members only;
    pairs with < min_sats kept members are dropped (returned mask)."""
    Xp = X.copy(); yp = y.copy(); use = np.zeros(len(y), bool)
    order = np.argsort(idx_inv, kind="stable"); bounds = np.searchsorted(idx_inv[order], np.arange(n_pairs + 1))
    for g in range(n_pairs):
        ii = order[bounds[g]:bounds[g + 1]]
        ii = ii[keep[ii]]
        if len(ii) < min_sats:
            continue
        Z = np.stack([np.ones(len(ii)), inc["rdot"][ii]], 1)
        Q, _ = np.linalg.qr(Z)
        Xp[ii] -= Q @ (Q.T @ X[ii]); yp[ii] -= Q @ (Q.T @ y[ii])
        use[ii] = True
    return Xp, yp, use

def solve(inc, thr=THR_M, use_rel=True, use_null=False, quad=False, tt_order=1, min_sats=3, per_sat_excl=True, per_sat_a=None, iono=0):
    keys = np.stack([inc["k0"], inc["k"]], 1)
    uniq, idx_inv = np.unique(keys, axis=0, return_inverse=True)
    n_pairs = len(uniq)
    segs = np.unique(inc["seg"])
    cols = []; names = []
    if use_rel and per_sat_a:
        rest = np.ones(len(inc), bool)
        for p_ in per_sat_a:
            mm = inc["prn"] == p_; rest &= ~mm
            cols.append(inc["dR"] * mm); names.append("a_G%02d" % p_)
        cols.append(inc["dR"] * rest); names.append("a_rest")
    elif use_rel:
        cols.append(inc["dR"]); names.append("a")
    if use_null:
        cols.append(inc["dN"]); names.append("a_null")
    tc = 0.5 * (inc["tm"].min() + inc["tm"].max()); th_ = (inc["tm"] - tc) / 3600.0
    for m_ in range(tt_order + 1):
        cols.append(inc["drdot"] * th_ ** m_); names.append("ddelta%d_s" % m_)
    for s in segs:
        ind = (inc["seg"] == s).astype(float)
        cols.append(ind * inc["dt"]); names.append("slope_%d" % s)
        if quad:
            cols.append(ind * inc["dt"] * th_); names.append("quad_%d" % s)
        if iono >= 1:
            cols.append(ind * inc["dmf"]); names.append("vtec0_%d" % s)
        if iono >= 2:
            cols.append(ind * (inc["dmf"] * th_ + inc["mf"] * inc["dt"] / 3600.0)); names.append("vtec1_%d" % s)
    X = np.stack(cols, 1); y = inc["dD"].copy(); p = X.shape[1]
    keep = ~inc["bad"]
    keep &= np.abs(y - np.median(y[keep])) < 3000.0   # gross (re-seed) increments
    sched = [t for t in (3.0, 1.0, 0.3) if t > thr] + [thr] * 12
    beta = None
    for th in sched:
        Xp, yp, use = _project(X, y, inc, keep, idx_inv, n_pairs, min_sats)
        m = keep & use
        if m.sum() < p + 20:
            return None
        sc = np.abs(Xp[m]).max(0); sc[sc == 0] = 1; Xs = Xp / sc
        b, *_ = np.linalg.lstsq(Xs[m], yp[m], rcond=None)
        r = yp - Xs @ b
        # exclusion: per pair, if max|r| > th: drop worst member when >= min_sats+1 kept, else drop the pair
        newkeep = keep.copy()
        order = np.argsort(idx_inv, kind="stable"); bounds = np.searchsorted(idx_inv[order], np.arange(n_pairs + 1))
        changed = False
        for g in range(n_pairs):
            ii = order[bounds[g]:bounds[g + 1]]; ii = ii[m[ii]]
            if len(ii) == 0:
                continue
            ra = np.abs(r[ii]); j = np.argmax(ra)
            if ra[j] > th:
                changed = True
                if per_sat_excl and len(ii) > min_sats:
                    newkeep[ii[j]] = False
                else:
                    newkeep[ii] = False
        if th == thr and not changed:
            beta = b; break
        keep = newkeep
        beta = b
    Xp, yp, use = _project(X, y, inc, keep, idx_inv, n_pairs, min_sats)
    m = keep & use
    n = int(m.sum())
    if n < p + 20:
        return None
    sc = np.abs(Xp[m]).max(0); sc[sc == 0] = 1; Xs = Xp / sc
    b, *_ = np.linalg.lstsq(Xs[m], yp[m], rcond=None)
    r = yp - Xs @ b
    n_pairs_used = len(np.unique(idx_inv[m]))
    dof = max(n - p - 2 * n_pairs_used, 1)
    sigma2 = (r[m] ** 2).sum() / dof
    ordk = np.argsort(inc["k"][m], kind="stable")
    neff, lags = n_eff_acf(r[m][ordk])
    cov = sigma2 * np.linalg.pinv(Xs[m].T @ Xs[m]) * (n / max(neff, p + 1))
    err = np.sqrt(np.abs(np.diag(cov))) / sc; beta = b / sc
    n_cand = int((~inc["bad"]).sum())
    res = dict(names=names[:4], beta=[float(v) for v in beta[:4]], err=[float(v) for v in err[:4]],
               n_inc=int(len(inc)), n_flagged_breaks=int(inc["bad"].sum()), n_kept=n, frac_excluded=float(1 - n / max(n_cand, 1)),
               n_pairs=int(n_pairs), n_pairs_used=int(n_pairs_used), core_sigma_m=float(math.sqrt(sigma2)),
               n_eff=float(neff), acf_lags=int(lags), n_sats=int(len(segs)), sats=sorted(int(v) for v in segs))
    for nm, bv, ev in zip(names, beta, err):
        if nm in ("a", "a_null", "a_rest") or nm.startswith("ddelta") or nm.startswith("a_G"):
            res[nm] = float(bv); res[nm + "_err"] = float(ev)
    res["_resid"] = (inc["k"][m], inc["prn"][m], r[m])
    # per-epoch nuisance recovery (dClock_k, g_k) from unprojected residual y - X beta over kept members
    ymod = y - X @ beta
    nuis = {}
    order = np.argsort(idx_inv, kind="stable"); bounds = np.searchsorted(idx_inv[order], np.arange(n_pairs + 1))
    for g in range(n_pairs):
        ii = order[bounds[g]:bounds[g + 1]]; ii = ii[m[ii]]
        if len(ii) < min_sats:
            continue
        Z = np.stack([np.ones(len(ii)), inc["rdot"][ii]], 1)
        cg, *_ = np.linalg.lstsq(Z, ymod[ii], rcond=None)
        nuis[float(inc["k"][ii[0]])] = (float(cg[0]), float(cg[1]))
    res["_nuis"] = nuis
    res["_beta_full"] = (names, beta)
    res["_keep"] = m
    return res

def analyze_window(t0, t1, label):
    u = lambda t: datetime.datetime.utcfromtimestamp(t).strftime("%Y-%m-%d %H:%M")
    delta0 = 0.0; passes = []
    for it in range(4):
        a = window_obs(t0, t1, delta0)
        if len(a) == 0:
            return dict(label=label, status="no data")
        inc = increments(a)
        f = solve(inc, tt_order=2, thr=0.3 if it == 0 else THR_M)
        if f is None:
            return dict(label=label, status="insufficient (>=3-sat epochs)")
        passes.append(dict(delta0_in=round(delta0, 4), ddelta0=round(f["ddelta0_s"], 4), ddelta1=round(f["ddelta1_s"], 4), a=round(f["a"], 4), a_err=round(f["a_err"], 4), frac_excluded=round(f["frac_excluded"], 3)))
        delta0 += f["ddelta0_s"]
        if abs(f["ddelta0_s"]) < 0.01:
            break
    a = window_obs(t0, t1, delta0); inc = increments(a)
    fits = {}
    fits["lin"] = solve(inc, tt_order=1)                      # default: no per-sat iono term (see lin_iono1/2 diagnostics)
    fits["lin_iono1"] = solve(inc, tt_order=1, iono=1)
    global USE_KLOBUCHAR
    if USE_KLOBUCHAR and _KLOB:
        USE_KLOBUCHAR = False
        a_nk = window_obs(t0, t1, delta0); inc_nk = increments(a_nk)
        fits["lin_noklob"] = solve(inc_nk, tt_order=1)
        fits["null_noklob"] = solve(inc_nk, tt_order=1, use_rel=False, use_null=True)
        USE_KLOBUCHAR = True
    fits["lin_iono2"] = solve(inc, tt_order=1, iono=2)
    # jackknife over satellites (leave-one-out) -> empirical error of a
    jk = []
    for s_ in sorted(set(int(v) for v in inc["prn"])):
        fj = solve(inc[inc["prn"] != s_], tt_order=1)
        if fj:
            jk.append((s_, fj["a"]))
    if len(jk) >= 4:
        ja = np.array([v for _, v in jk]); nj = len(ja)
        fits["jackknife"] = dict(n=nj, a_mean=float(ja.mean()), a_err_jackknife=float(math.sqrt((nj - 1) / nj * ((ja - ja.mean()) ** 2).sum())),
                                 leave_one_out={"G%02d" % p_: round(v, 3) for p_, v in jk})
    fits["lin_tt0"] = solve(inc, tt_order=0)
    fits["lin_tt2"] = solve(inc, tt_order=2)
    fits["quad"] = solve(inc, tt_order=1, quad=True)
    fits["null_lin"] = solve(inc, tt_order=1, use_rel=False, use_null=True)
    fits["both_lin"] = solve(inc, tt_order=1, use_null=True)
    fits["lin_min4"] = solve(inc, tt_order=1, min_sats=4)
    fits["lin_pairexcl"] = solve(inc, tt_order=1, per_sat_excl=False)
    big = [int(p) for p in np.unique(a["prn"]) if (a[a["prn"] == p]["R"].max() - a[a["prn"] == p]["R"].min()) >= 8.0]
    fits["per_sat"] = None
    per_sat_fixed = {}
    for s_ in big:
        inc2 = inc.copy(); others = inc2["prn"] != s_
        inc2["dD"][others] -= inc2["dR"][others]; inc2["dR"][others] = 0.0
        f2 = solve(inc2, tt_order=1)
        f3 = solve(inc2, tt_order=1, iono=1)
        if f2:
            per_sat_fixed["G%02d" % s_] = dict(a=round(f2["a"], 4), a_err=round(f2["a_err"], 4), a_iono1=round(f3["a"], 4) if f3 else None, a_iono1_err=round(f3["a_err"], 4) if f3 else None)
    # level reconstruction for plots: walk the PRN's samples; kept increments -> cleaned
    # increment (common-mode, tag, slope removed); dropped/excluded increments -> bridged
    # with the fitted model a*dR so the series stays continuous
    recon = {}
    fl = fits["lin"]
    if fl is not None:
        names_b, beta_b = fl["_beta_full"]; nuis = fl["_nuis"]; keepm = fl["_keep"]
        bmap = dict(zip(names_b, beta_b))
        tc = 0.5 * (inc["tm"].min() + inc["tm"].max())
        for p_ in big:
            ii = np.where(inc["prn"] == p_)[0]
            if len(ii) < 100:
                continue
            th_ = (inc["tm"][ii] - tc) / 3600.0
            tt = sum(bmap.get("ddelta%d_s" % k_, 0.0) * inc["drdot"][ii] * th_ ** k_ for k_ in range(3))
            nu = np.array([nuis.get(float(k_), (np.nan, np.nan)) for k_ in inc["k"][ii]])
            clean = inc["dD"][ii] - nu[:, 0] - inc["rdot"][ii] * nu[:, 1] - bmap["slope_%d" % p_] * inc["dt"][ii] - tt
            ok_ = keepm[ii] & np.isfinite(clean)
            cmap = {float(k_): float(c_) for k_, c_, o_ in zip(inc["k"][ii], clean, ok_) if o_}
            b = a[a["prn"] == p_]; b = b[np.argsort(b["ep"], kind="stable")]
            lvl = np.zeros(len(b))
            for i in range(1, len(b)):
                c_ = cmap.get(float(b["ep"][i]))
                lvl[i] = lvl[i - 1] + (c_ if c_ is not None else bmap["a"] * (b["R"][i] - b["R"][i - 1]))
            recon["G%02d" % p_] = (b["ep"].copy(), lvl, b["R"].copy(), float(ok_.mean()))
    thr_var = {}
    for th in THR_VARIANTS:
        fv = solve(inc, thr=th, tt_order=1)
        if fv:
            thr_var["%.2f" % th] = dict(a=round(fv["a"], 4), a_err=round(fv["a_err"], 4), frac_excluded=round(fv["frac_excluded"], 3))
    swings = {}
    for p in np.unique(a["prn"]):
        b = a[a["prn"] == p]; swings["G%02d" % p] = round(float(b["R"].max() - b["R"].min()), 2)
    hi_sw = {k: v for k, v in swings.items() if v >= 3.0}
    resid = fits["lin"].pop("_resid") if fits["lin"] else None
    for k in list(fits):
        if fits[k] is None:
            fits[k] = dict(status="failed")
        else:
            for kk in ("_resid", "_nuis", "_beta_full", "_keep"):
                fits[k].pop(kk, None)
            fits[k] = {kk: (round(vv, 5) if isinstance(vv, float) else vv) for kk, vv in fits[k].items()}
    f = fits["lin"]
    status = "fitted" if "a" in f else "failed"
    if "a" in f and f["frac_excluded"] > 0.6:
        status = "failed: >60%% increments excluded (tag/session pathology)"
    out = dict(label=label, window=[u(t0), u(t1)], span_h=round((t1 - t0) / 3600, 2), status=status,
               delta0_s=round(delta0, 3), delta0_passes=passes, local_hour_start=round(((t0 / 3600.0) % 24 - 4) % 24, 1),
               n_samples=int(len(a)), sats=sorted(set(int(v) for v in a["prn"])), rel_swing_by_sat_m=swings, rel_swing_high_m=hi_sw,
               fits=fits, threshold_variants=thr_var)
    if status == "fitted":
        out.update(a=f["a"], a_err=f["a_err"], sig_a0=round(f["a"] / f["a_err"], 2), sig_a1=round((f["a"] - 1) / f["a_err"], 2))
        def fmt(k, fld="a"):
            g = fits[k]; return ("%.2f+/-%.2f" % (g[fld], g[fld + "_err"])) if fld in g else "-"
        jks = ("jackknife %.3f+/-%.3f" % (fits["jackknife"]["a_mean"], fits["jackknife"]["a_err_jackknife"])) if "jackknife" in fits and "a_mean" in fits["jackknife"] else ""
        print("%-24s %s %.2fh nsat %d d0 %.2fs | a=%.3f+/-%.3f %s (noklob %s tt0 %s tt2 %s quad %s null %s null_noklob %s min4 %s pairexcl %s) core %.4f excl %.2f neff %.0f"
              % (label, out["window"][0], out["span_h"], len(out["sats"]), delta0, f["a"], f["a_err"], jks, fmt("lin_noklob") if "lin_noklob" in fits else "-", fmt("lin_tt0"), fmt("lin_tt2"), fmt("quad"),
                 fmt("null_lin", "a_null"), fmt("null_noklob", "a_null") if "null_noklob" in fits else "-", fmt("lin_min4"), fmt("lin_pairexcl"), f["core_sigma_m"], f["frac_excluded"], f["n_eff"]))
    else:
        print("%-24s failed" % label)
    out["per_sat_a_others_fixed"] = per_sat_fixed
    if per_sat_fixed:
        print("      per-sat a (others fixed at 1):", {k: "%.2f+/-%.2f" % (v["a"], v["a_err"]) for k, v in per_sat_fixed.items()})
    return out, resid, a, recon

def main():
    init()
    ecc = {p: float(np.mean([r["ecc"] for r in _eph[p]])) for p in _eph}
    hi = [p for p in ecc if ecc[p] >= HI_E_MIN]
    # windows: high-e arcs >= MIN_ARC_H after TCUT, merged when overlapping
    arcs = []
    for H in hi:
        for (i0, i1) in _segs[H]["segs"]:
            t0, t1 = _segs[H]["ep"][i0], _segs[H]["ep"][i1]
            if t0 >= TCUT and (t1 - t0) / 3600 >= MIN_ARC_H:
                arcs.append([t0, t1, ["G%02d" % H]])
    arcs.sort()
    for a in arcs:
        a[0] -= 2 * 3600.0; a[1] += 2 * 3600.0
    merged = []
    for a in arcs:
        if merged and a[0] <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], a[1]); merged[-1][2] += a[2]
        else:
            merged.append(a)
    # tracker restarts: row epochs where every locked GPS channel has lock_s < 60 s
    ep_all = _d["sat_epoch"]; lk_all = _d["lock_s"]; mlock = lk_all > 0
    ue, inv = np.unique(ep_all[mlock], return_inverse=True)
    mx = np.zeros(len(ue)); np.maximum.at(mx, inv, lk_all[mlock])
    restarts = ue[(mx < 60) & (ue >= TCUT)]
    # collapse restart clusters (< 10 min apart)
    restart_list = []
    for t in restarts:
        if not restart_list or t - restart_list[-1] > 600:
            restart_list.append(float(t))
    print("tracker restarts after TCUT:", [datetime.datetime.utcfromtimestamp(t).strftime("%m-%d %H:%M") for t in restart_list])
    split = []
    for (t0, t1, hs) in merged:
        cuts = [t0] + [t + 120 for t in restart_list if t0 < t < t1] + [t1]
        for i in range(len(cuts) - 1):
            if (cuts[i + 1] - cuts[i]) / 3600 >= MIN_WINDOW_H:
                split.append([cuts[i], cuts[i + 1] - (120 if i + 1 < len(cuts) - 1 else 0), hs])
    merged = split
    print("windows:", len(merged))
    results = []; plots = []
    for (t0, t1, hs) in merged:
        label = "+".join(sorted(set(hs))) + " " + datetime.datetime.utcfromtimestamp(t0).strftime("%m-%d %H:%M")
        r = analyze_window(t0, t1, label)
        if isinstance(r, tuple):
            res, resid, a, recon = r
            results.append(res)
            if res["status"] == "fitted":
                plots.append((res, resid, a, recon))
        else:
            results.append(r)
    fitted = [r for r in results if r.get("status") == "fitted"]
    def combine(rs, key="lin", field="a"):
        rs = [r for r in rs if "a" in r["fits"][key] or field in r["fits"][key]]
        if not rs:
            return None
        av = np.array([r["fits"][key][field] for r in rs]); s = np.array([r["fits"][key][field + "_err"] for r in rs])
        w = 1 / s ** 2; am = (w * av).sum() / w.sum(); ae = 1 / math.sqrt(w.sum())
        chi2 = float((((av - am) / s) ** 2).sum()); dof = max(len(av) - 1, 1); sf = math.sqrt(max(chi2 / dof, 1))
        return dict(a=round(am, 4), err=round(ae, 4), n_windows=len(av), chi2=round(chi2, 1), dof=dof, err_scaled=round(ae * sf, 4),
                    median=round(float(np.median(av)), 4), sig_a0=round(am / ae, 1), sig_a1=round((am - 1) / ae, 1),
                    sig_a0_scaled=round(am / (ae * sf), 1), sig_a1_scaled=round((am - 1) / (ae * sf), 1))
    combined = {}
    head = [r for r in fitted if abs(r["delta0_s"]) < 120 and r["fits"]["lin"]["frac_excluded"] < 0.4]
    for label, rs in (("headline", head), ("all", fitted)):
        js = [r for r in rs if "jackknife" in r["fits"] and "a_mean" in r["fits"]["jackknife"]]
        if js:
            av = np.array([r["fits"]["lin"]["a"] for r in js]); se = np.array([r["fits"]["jackknife"]["a_err_jackknife"] for r in js])
            w = 1 / se ** 2; am = (w * av).sum() / w.sum(); ae = 1 / math.sqrt(w.sum())
            chi2 = float((((av - am) / se) ** 2).sum())
            combined["%s/lin/a_jackknife_weighted" % label] = dict(a=round(am, 4), err=round(ae, 4), n_windows=len(av), chi2=round(chi2, 1), dof=len(av) - 1,
                                                                   sig_a0=round(am / ae, 1), sig_a1=round((am - 1) / ae, 1), per_window=[[round(x, 3), round(y, 3)] for x, y in zip(av, se)])
            print("COMBINED %s/lin jackknife-weighted: a = %.3f +/- %.3f (chi2 %.1f/%d) sig(a-0)=%.1f sig(a-1)=%.1f" % (label, am, ae, chi2, len(av) - 1, am / ae, (am - 1) / ae))
    for label, rs in (("headline", head), ("all", fitted)):
        for key, field in (("lin", "a"), ("lin_noklob", "a"), ("null_noklob", "a_null"), ("lin_iono1", "a"), ("lin_iono2", "a"), ("lin_tt0", "a"), ("lin_tt2", "a"), ("quad", "a"), ("lin_min4", "a"), ("lin_pairexcl", "a"), ("null_lin", "a_null"), ("both_lin", "a"), ("both_lin", "a_null")):
            c = combine(rs, key, field)
            if c:
                combined["%s/%s/%s" % (label, key, field)] = c
                print("COMBINED %-16s a=%8.4f +/- %.4f (scaled %.4f; chi2 %.1f/%d; n=%d; median %.3f) sig(a-0)=%.1f sig(a-1)=%.1f [scaled %.1f / %.1f]"
                      % ("%s/%s" % (label, key), c["a"], c["err"], c["err_scaled"], c["chi2"], c["dof"], c["n_windows"], c["median"], c["sig_a0"], c["sig_a1"], c["sig_a0_scaled"], c["sig_a1_scaled"]))
    # plots: per window, the high-e sat's cumulative projected signal is not directly plottable; plot instead
    # per-sat D minus (per-epoch common fit) minus slope, versus a=1 curve -- level reconstruction from kept increments
    plotted = []
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        for (res, resid, a, recon) in sorted(plots, key=lambda x: x[0]["a_err"]):
            if not recon:
                continue
            keys = sorted(recon, key=lambda k: (0 if k in ("G07", "G24", "G15", "G02", "G16") else 1, -(recon[k][2].max() - recon[k][2].min())))[:3]
            fig, axs = plt.subplots(len(keys), 1, figsize=(8.5, 2.9 * len(keys)), sharex=True, squeeze=False)
            for ax, k in zip(axs[:, 0], keys):
                tl, lvl, Rl, fk = recon[k]
                t = (tl - tl[0]) / 3600
                cl = np.polyfit(t, Rl, 1); trend = np.polyval(cl, t)   # linear part of the a=1 curve removed from everything (slope is a nuisance)
                gap = np.concatenate([[False], np.diff(tl) > 60])
                y_d = lvl - trend; y_1 = Rl - trend; y_f = res["a"] * (Rl - trend)
                off = (y_d - y_f).mean(); y_d = y_d - off
                y_1 = y_1 - (y_1.mean() - y_f.mean()) + 0 * y_1
                y1m = np.where(gap, np.nan, y_1); yfm = np.where(gap, np.nan, y_f)
                ax.plot(t, y_d, ".", ms=2, color="#555", label="%s cleaned carrier D(t), minus linear trend" % k)
                ax.plot(t, y1m + (y_f.mean() - y_1.mean()), "-", color="#d62728", lw=1.5, label="a = 1 (IS-GPS-200), same trend removed")
                ax.plot(t, yfm, "--", color="#1f77b4", lw=1.1, label="window fit a = %.2f +/- %.2f" % (res["a"], res["a_err"]))
                ax.set_ylabel("metres"); ax.legend(fontsize=7, loc="best")
                ax.set_title("%s   relativistic swing %.1f m   kept increments %.0f%%" % (k, Rl.max() - Rl.min(), 100 * fk), fontsize=9)
            axs[-1, 0].set_xlabel("hours from window start  (%s UTC, %.1f h, delta0 %.1f s)" % (res["window"][0], res["span_h"], res["delta0_s"]))
            fig.suptitle("Relativistic periodic clock term, carrier phase: window %s" % res["label"], fontsize=10)
            fn = os.path.join(SCR, "relativity_ms_%s.png" % res["label"].replace(" ", "_").replace(":", "").replace("+", "_"))
            fig.tight_layout(); fig.savefig(fn, dpi=110); plt.close(fig); plotted.append(fn)
    except Exception as ex:
        import traceback; traceback.print_exc()
        print("plot skipped:", ex)
    out = dict(generated=datetime.datetime.utcnow().isoformat() + "Z", method=__doc__, threshold_m=THR_M, tcut_utc=datetime.datetime.utcfromtimestamp(TCUT).isoformat(),
               eph_max_age_h=EPH_MAX_AGE_H, tracker_restarts_utc=[datetime.datetime.utcfromtimestamp(t).isoformat() for t in restart_list],
               windows=results, combined=combined, plots=plotted)
    json.dump(out, open(os.path.join(SCR, "relativity_carrier_ms.json"), "w"), indent=1)
    print("wrote relativity_carrier_ms.json")

if __name__ == "__main__":
    main()
