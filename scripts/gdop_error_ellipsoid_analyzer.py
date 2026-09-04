#!/usr/bin/env python3
"""Multi-Constellation GDOP & Horizontal Error Ellipsoid Metrology Engine.

Evaluates geometric dilution of precision and spatial error ellipsoids:
1. Direction Cosines & Normal Equations:
   u_i = [cos(El_i)*sin(Az_i), cos(El_i)*cos(Az_i), sin(El_i)]^T (local ENU)
   G_i = [-u_E, -u_N, -u_U, 1.0] (single clock) or [-u_E, -u_N, -u_U, delta_sys] (multi-clock)
   Q = (G^T G)^-1 = Covariance cofactor matrix

2. Dilution of Precision Decomposition:
   GDOP = sqrt(tr(Q)) = sqrt(PDOP^2 + TDOP^2)
   PDOP = sqrt(Q_EE + Q_NN + Q_UU) (3D Position Dilution)
   HDOP = sqrt(Q_EE + Q_NN) (Horizontal Dilution)
   VDOP = sqrt(Q_UU) (Vertical Dilution)
   TDOP = sqrt(Q_tt) (Time Dilution)
   VDOP/HDOP ratio: geometric vertical deficit due to half-space visibility (horizon mask)

3. 2D Horizontal Error Ellipsoid (95% Confidence):
   Q_H = [[Q_EE, Q_EN], [Q_NE, Q_NN]]
   Eigenvalues lambda_1, lambda_2 (major and minor variance axes)
   a_95 = 2.4477 * sigma_UERE * sqrt(lambda_1) (semi-major axis in meters)
   b_95 = 2.4477 * sigma_UERE * sqrt(lambda_2) (semi-minor axis in meters)
   theta = 0.5 * atan2(2*Q_EN, Q_NN - Q_EE) (orientation azimuth, deg from North)
   Area_95 = pi * a_95 * b_95 (m^2)

4. Multi-Constellation vs Single-System Fusion Benchmark:
   - Evaluates live tracked satellites and complete in-view sky constellation
   - Evaluates ISB calibration gain factor: PDOP_multi_clock / PDOP_single_clock
   - Computes Multi-GNSS advantage vs single GPS (GDOP reduction %, error ellipse area reduction %)

Publishes state to:
  /Volumes/Radiator 8TB/gnss/observations/state.gdop.json
"""

import argparse
import json
import math
import os
import sys
import time
import numpy as np

# Nominal User Equivalent Range Error (UERE) in meters
NOMINAL_UERE_M = 2.0
# 95% 2D confidence scale factor: sqrt(-2 * ln(1 - 0.95)) = sqrt(2 * ln(20))
CHI2_95_2DOF = math.sqrt(-2.0 * math.log(0.05))  # ~2.44774

SYS_MAP = {
    0: "gps", "gps": "gps",
    1: "beidou", "beidou": "beidou", "bds": "beidou",
    2: "galileo", "galileo": "galileo", "gal": "galileo",
    3: "glonass", "glonass": "glonass", "glo": "glonass",
    "sbas": "sbas"
}


def compute_direction_cosines(az_deg, el_deg):
    """Compute local ENU line-of-sight unit vector components.
    
    az_deg: Azimuth measured clockwise from North (degrees).
    el_deg: Elevation measured above local horizon (degrees).
    
    Returns (ue, un, uu).
    """
    el = math.radians(el_deg)
    az = math.radians(az_deg)
    ue = math.cos(el) * math.sin(az)
    un = math.cos(el) * math.cos(az)
    uu = math.sin(el)
    return ue, un, uu


def build_design_matrix(sat_list, mode="single_clock", sys_list=None):
    """Construct GNSS linear observation design matrix G.
    
    mode:
      - 'single_clock': 4 columns [dx_E, dx_N, dx_U, c*dt]
      - 'multi_clock': 3 + M columns [dx_E, dx_N, dx_U, c*dt_sys1, ..., c*dt_sysM]
    
    sat_list: list of dicts with 'az_deg', 'el_deg', and optionally 'sys'
    sys_list: list of unique constellation names for multi_clock mode
    
    Returns (G, active_sys_list).
    """
    if not sat_list:
        return np.empty((0, 4)), []

    if mode == "multi_clock":
        if not sys_list:
            sys_list = sorted(list(set(s.get("sys", "gps").lower() for s in sat_list)))
        rows = []
        for s in sat_list:
            ue, un, uu = compute_direction_cosines(s["az_deg"], s["el_deg"])
            s_sys = s.get("sys", "gps").lower()
            clock_vec = [1.0 if s_sys == active_s else 0.0 for active_s in sys_list]
            rows.append([-ue, -un, -uu] + clock_vec)
        return np.array(rows, dtype=np.float64), sys_list
    else:
        rows = []
        for s in sat_list:
            ue, un, uu = compute_direction_cosines(s["az_deg"], s["el_deg"])
            rows.append([-ue, -un, -uu, 1.0])
        return np.array(rows, dtype=np.float64), ["common"]


def compute_dop_metrics(G, mode="single_clock", uere_m=NOMINAL_UERE_M):
    """Invert normal equations G^T G to compute full DOP metrics and error ellipse.
    
    Returns a dict of metrics, or None if rank deficient / ill-conditioned.
    """
    n_rows, n_cols = G.shape
    if n_rows < n_cols:
        return None

    # Condition number check
    cond = np.linalg.cond(G)
    if cond > 1e7 or np.isnan(cond):
        return None

    try:
        GTG = G.T @ G
        Q = np.linalg.inv(GTG)
    except np.linalg.LinAlgError:
        return None

    # Diagonal variances
    q_ee = Q[0, 0]
    q_nn = Q[1, 1]
    q_uu = Q[2, 2]

    if q_ee < 0 or q_nn < 0 or q_uu < 0:
        return None

    hdop = math.sqrt(q_ee + q_nn)
    vdop = math.sqrt(q_uu)
    pdop = math.sqrt(q_ee + q_nn + q_uu)

    if mode == "single_clock":
        q_tt = Q[3, 3] if Q.shape[0] > 3 else 0.0
        tdop = math.sqrt(max(0.0, q_tt))
        gdop = math.sqrt(pdop ** 2 + tdop ** 2)
    else:
        # Multi-clock: TDOP per system, primary system TDOP is Q[3,3]
        q_tt = Q[3, 3] if Q.shape[0] > 3 else 0.0
        tdop = math.sqrt(max(0.0, q_tt))
        # Total GDOP defined with mean clock variance
        clock_vars = [Q[i, i] for i in range(3, n_cols)]
        mean_q_tt = sum(clock_vars) / len(clock_vars) if clock_vars else 0.0
        gdop = math.sqrt(pdop ** 2 + mean_q_tt)

    vdop_hdop_ratio = (vdop / hdop) if hdop > 1e-6 else None

    # Horizontal Error Ellipse (2D eigenvalue decomposition)
    q_en = Q[0, 1]

    # Analytical eigenvalues for 2x2 symmetric matrix
    delta = math.sqrt(((q_ee - q_nn) / 2.0) ** 2 + q_en ** 2)
    lambda_1 = max(0.0, (q_ee + q_nn) / 2.0 + delta)
    lambda_2 = max(0.0, (q_ee + q_nn) / 2.0 - delta)

    # 95% Confidence axes (scaled by UERE and chi-square factor)
    a_95 = CHI2_95_2DOF * uere_m * math.sqrt(lambda_1)
    b_95 = CHI2_95_2DOF * uere_m * math.sqrt(lambda_2)
    area_95 = math.pi * a_95 * b_95

    # Orientation angle of semi-major axis (azimuth clockwise from North)
    # theta = 0.5 * atan2(2 * Q_en, Q_nn - Q_ee)
    theta_rad = 0.5 * math.atan2(2.0 * q_en, q_nn - q_ee)
    theta_deg = math.degrees(theta_rad) % 180.0
    if theta_deg < 0:
        theta_deg += 180.0

    return {
        "n_sat": n_rows,
        "mode": mode,
        "condition_number": round(float(cond), 2),
        "gdop": round(gdop, 2),
        "pdop": round(pdop, 2),
        "hdop": round(hdop, 2),
        "vdop": round(vdop, 2),
        "tdop": round(tdop, 2),
        "vdop_hdop_ratio": round(vdop_hdop_ratio, 2) if vdop_hdop_ratio is not None else None,
        "error_ellipse_95": {
            "semi_major_a_m": round(a_95, 2),
            "semi_minor_b_m": round(b_95, 2),
            "azimuth_theta_deg": round(theta_deg, 1),
            "area_m2": round(area_95, 2),
            "aspect_ratio": round(a_95 / b_95, 2) if b_95 > 1e-4 else 1.0,
            "uere_m": uere_m
        }
    }


def evaluate_dop_solution(sat_list, mode="single_clock", sys_list=None):
    """Helper to build G and solve DOP for a given satellite set."""
    if not sat_list or len(sat_list) < 4:
        return None
    G, active_sys = build_design_matrix(sat_list, mode=mode, sys_list=sys_list)
    return compute_dop_metrics(G, mode=mode)


def run_gdop_analysis_cycle():
    """Execute one full GDOP and error ellipsoid metrology evaluation."""
    # 1. Load sky state
    sky_file = "/Volumes/Radiator 8TB/gnss/observations/state.sky.json"
    sky_sats = []
    epoch = time.time()

    if os.path.exists(sky_file):
        try:
            with open(sky_file) as f:
                sdata = json.load(f)
                epoch = sdata.get("epoch", epoch)
                sky_sats = sdata.get("sky", {}).get("sats", [])
        except Exception:
            sky_sats = []

    # 2. Extract tracked satellites and in-view satellites
    tracked_sats = []
    in_view_sats = []

    for s in sky_sats:
        az = s.get("az_deg")
        el = s.get("el_deg")
        if az is None or el is None:
            continue
        
        sys_name = s.get("sys", "gps").lower()
        if sys_name == "bds":
            sys_name = "beidou"
        elif sys_name == "gal":
            sys_name = "galileo"
        elif sys_name == "glo":
            sys_name = "glonass"

        s_clean = {
            "sys": sys_name,
            "prn": s.get("prn"),
            "az_deg": az,
            "el_deg": el,
            "cls": s.get("cls"),
            "cn0": s.get("cn0"),
            "lock_s": s.get("lock_s", 0.0)
        }

        # Filter tracked: marked as 'tracked' or has positive lock duration
        if s.get("cls") == "tracked" or (s.get("lock_s") and s["lock_s"] > 0):
            tracked_sats.append(s_clean)

        # In-view sky: elevation >= 5.0 degrees
        if el >= 5.0:
            in_view_sats.append(s_clean)

    # 3. Compute Tracked Solutions
    tracked_all_single = evaluate_dop_solution(tracked_sats, mode="single_clock")
    tracked_all_multi = evaluate_dop_solution(tracked_sats, mode="multi_clock")

    tracked_gps = evaluate_dop_solution([s for s in tracked_sats if s["sys"] == "gps"])
    tracked_bds = evaluate_dop_solution([s for s in tracked_sats if s["sys"] == "beidou"])
    tracked_gal = evaluate_dop_solution([s for s in tracked_sats if s["sys"] == "galileo"])

    # ISB Gain Factor on tracked fix
    isb_gain_factor = None
    if tracked_all_multi and tracked_all_single:
        if tracked_all_single["pdop"] > 0:
            isb_gain_factor = round(tracked_all_multi["pdop"] / tracked_all_single["pdop"], 2)

    # 4. Compute Complete Sky In-View Solutions (Mask >= 5 deg)
    inview_all = evaluate_dop_solution(in_view_sats, mode="single_clock")
    inview_gps = evaluate_dop_solution([s for s in in_view_sats if s["sys"] == "gps"])
    inview_bds = evaluate_dop_solution([s for s in in_view_sats if s["sys"] == "beidou"])
    inview_gal = evaluate_dop_solution([s for s in in_view_sats if s["sys"] == "galileo"])
    inview_glo = evaluate_dop_solution([s for s in in_view_sats if s["sys"] == "glonass"])

    # Multi-GNSS Advantage vs Single GPS (In-View)
    gdop_reduction_pct = None
    ellipse_area_reduction_pct = None
    if inview_all and inview_gps:
        if inview_gps["gdop"] > 0:
            gdop_reduction_pct = round((1.0 - inview_all["gdop"] / inview_gps["gdop"]) * 100.0, 1)
        area_all = inview_all["error_ellipse_95"]["area_m2"]
        area_gps = inview_gps["error_ellipse_95"]["area_m2"]
        if area_gps > 0:
            ellipse_area_reduction_pct = round((1.0 - area_all / area_gps) * 100.0, 1)

    # Synthesis summary
    primary_solution = tracked_all_single or inview_all
    primary_gdop = primary_solution["gdop"] if primary_solution else 1.0
    primary_hdop = primary_solution["hdop"] if primary_solution else 0.5
    primary_vdop = primary_solution["vdop"] if primary_solution else 0.8
    primary_ellipse = primary_solution["error_ellipse_95"] if primary_solution else {}

    output_state = {
        "epoch": round(epoch, 2),
        "ttl_s": 30.0,
        "gdop_summary": {
            "primary_gdop": primary_gdop,
            "primary_hdop": primary_hdop,
            "primary_vdop": primary_vdop,
            "fix_status": "3D_MULTI_GNSS_FIX" if tracked_all_single else ("VISIBLE_SKY_ONLY" if inview_all else "NO_FIX"),
            "tracked_sats_count": len(tracked_sats),
            "in_view_sats_count": len(in_view_sats),
            "isb_gain_factor": isb_gain_factor,
            "gdop_reduction_vs_gps_pct": gdop_reduction_pct,
            "area_reduction_vs_gps_pct": ellipse_area_reduction_pct,
            "semi_major_a_m": primary_ellipse.get("semi_major_a_m"),
            "semi_minor_b_m": primary_ellipse.get("semi_minor_b_m"),
            "azimuth_theta_deg": primary_ellipse.get("azimuth_theta_deg")
        },
        "tracked_solutions": {
            "all_gnss_single_clock": tracked_all_single,
            "all_gnss_multi_clock": tracked_all_multi,
            "gps_only": tracked_gps or {"status": f"INSUFFICIENT_SATS (N={len([s for s in tracked_sats if s['sys'] == 'gps'])})"},
            "beidou_only": tracked_bds or {"status": f"INSUFFICIENT_SATS (N={len([s for s in tracked_sats if s['sys'] == 'beidou'])})"},
            "galileo_only": tracked_gal or {"status": f"INSUFFICIENT_SATS (N={len([s for s in tracked_sats if s['sys'] == 'galileo'])})"}
        },
        "in_view_solutions": {
            "all_gnss": inview_all,
            "gps_only": inview_gps,
            "beidou_only": inview_bds,
            "galileo_only": inview_gal,
            "glonass_only": inview_glo
        },
        "tracked_satellites_list": tracked_sats
    }

    # Write atomically
    target_path = "/Volumes/Radiator 8TB/gnss/observations/state.gdop.json"
    tmp_path = target_path + f".tmp.{os.getpid()}"
    with open(tmp_path, "w") as f:
        json.dump(output_state, f, indent=2)
    os.replace(tmp_path, target_path)

    return output_state


def main():
    parser = argparse.ArgumentParser(description="Multi-Constellation GDOP & Error Ellipsoid Metrology Daemon")
    parser.add_argument("--loop", action="store_true", help="Run continuously in a loop")
    parser.add_argument("--interval", type=float, default=2.0, help="Loop interval in seconds (default: 2.0)")
    parser.add_argument("--once", action="store_true", help="Run a single evaluation cycle")
    args = parser.parse_args()

    print(f"Starting GDOP & Error Ellipsoid Analyzer (interval={args.interval}s, loop={args.loop})...")

    while True:
        try:
            state = run_gdop_analysis_cycle()
            summ = state["gdop_summary"]
            print(f"[{time.strftime('%H:%M:%S')}] GDOP={summ['primary_gdop']} (HDOP={summ['primary_hdop']}, VDOP={summ['primary_vdop']}) | "
                  f"Ellipse: {summ['semi_major_a_m']}m x {summ['semi_minor_b_m']}m @ {summ['azimuth_theta_deg']} deg | "
                  f"Tracked={summ['tracked_sats_count']} sats ({summ['fix_status']})")
        except Exception as e:
            print(f"[{time.strftime('%H:%M:%S')}] Error in GDOP cycle: {e}")

        if args.once or not args.loop:
            break
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
