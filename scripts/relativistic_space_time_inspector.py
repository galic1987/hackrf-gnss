#!/usr/bin/env python3
"""Relativistic Space-Time Inspector.

Evaluates Einstein's General and Special Relativity on GNSS signals in real time:
1. General Relativity (Gravitational Blueshift / Potential Dilation):
   Clocks at MEO altitudes tick faster than clocks on the geoid by +45.7 us/day.
2. Special Relativity (Kinematic Time Dilation / Transverse Doppler):
   High orbital speed (~3.87 km/s) causes kinematic slowdown by -7.2 us/day.
3. Net Secular Clock Drift & Pre-launch Frequency Offset:
   Satellites gain +38.5 us/day (+11.5 km/day uncompensated); factory oscillators
   are pre-compensated on Earth to 10.22999999543 MHz (-4.55 mHz shift).
4. Einstein Periodic Orbital Eccentricity Correction:
   dt_r = F * e * sqrt(a) * sin(E_k) (oscillating up to +/-70 ns / +/-21 m).
5. Relativistic Sagnac Delay:
   Earth's rotation during signal flight (dt = w_E / c^2 * (x_sat*y_rx - y_sat*x_rx))
   shifts pseudoranges by up to +/-35 m (+/-120 ns).

Publishes state to:
  /Volumes/Radiator 8TB/gnss/observations/state.relativity.json
"""

import argparse
import json
import math
import os
import sys
import time

# Constants
C_LIGHT = 299792458.0              # Speed of light (m/s)
MU_GPS = 3.986005e14              # Earth gravitational parameter GPS (m^3/s^2)
MU_GAL = 3.986004418e14           # Earth gravitational parameter Galileo
MU_BDS = 3.986004418e14           # Earth gravitational parameter BeiDou
OMEGA_E = 7.2921151467e-5         # Earth rotation rate (rad/s)
A_EARTH = 6378137.0               # WGS-84 Earth equatorial radius (m)
E2_EARTH = 0.00669437999014       # WGS-84 first eccentricity squared

F_REL_GPS = -4.442807633e-10      # Relativistic constant for GPS (s/m^0.5)
F_REL_GAL = -4.442807309e-10      # Relativistic constant for Galileo (s/m^0.5)
F_REL_BDS = -2.0 * math.sqrt(MU_BDS) / (C_LIGHT ** 2)

WEEK_S = 604800.0

SYS_NAMES = {0: "gps", 1: "beidou", 2: "galileo", 3: "glonass"}
SYS_CODES = {0: "GPS", 1: "BEIDOU", 2: "GALILEO", 3: "GLONASS"}

def llh_to_ecef(lat_deg, lon_deg, h_m):
    """Convert geodetic latitude, longitude, and height to ECEF coordinates (meters)."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    n = A_EARTH / math.sqrt(1.0 - E2_EARTH * math.sin(lat) ** 2)
    x = (n + h_m) * math.cos(lat) * math.cos(lon)
    y = (n + h_m) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - E2_EARTH) + h_m) * math.sin(lat)
    return x, y, z

def ecef_to_azel(sat_ecef, site_ecef, lat_deg, lon_deg):
    """Compute azimuth and elevation of satellite seen from ground station."""
    dx = sat_ecef[0] - site_ecef[0]
    dy = sat_ecef[1] - site_ecef[1]
    dz = sat_ecef[2] - site_ecef[2]
    la = math.radians(lat_deg)
    lo = math.radians(lon_deg)
    e = -math.sin(lo) * dx + math.cos(lo) * dy
    n = -math.sin(la) * math.cos(lo) * dx - math.sin(la) * math.sin(lo) * dy + math.cos(la) * dz
    u = math.cos(la) * math.cos(lo) * dx + math.cos(la) * math.sin(lo) * dy + math.sin(la) * dz
    az = math.degrees(math.atan2(e, n)) % 360.0
    el = math.degrees(math.atan2(u, math.hypot(e, n)))
    return az, el

def solve_kepler(mk, e, tol=1e-12, max_iter=15):
    """Solve Kepler's equation M = E - e*sin(E) using Newton-Raphson."""
    ek = mk
    for _ in range(max_iter):
        f = ek - e * math.sin(ek) - mk
        f_prime = 1.0 - e * math.cos(ek)
        step = f / f_prime
        ek -= step
        if abs(step) < tol:
            break
    return ek

def compute_gr_blueshift(r_sat, mu=MU_GPS, r_geoid=A_EARTH):
    """General Relativity gravitational blueshift:
    (delta_f / f)_GR = (Phi_sat - Phi_geoid) / c^2 = mu/c^2 * (1/r_geoid - 1/r_sat).
    Returns (fractional_shift, us_per_day).
    """
    shift = (mu / (C_LIGHT ** 2)) * (1.0 / r_geoid - 1.0 / r_sat)
    us_per_day = shift * 86400.0 * 1e6
    return shift, us_per_day

def compute_sr_dilation(v_orbital):
    """Special Relativity kinematic time dilation (transverse Doppler):
    (delta_f / f)_SR = -v^2 / (2 * c^2).
    Returns (fractional_shift, us_per_day).
    """
    shift = - (v_orbital ** 2) / (2.0 * (C_LIGHT ** 2))
    us_per_day = shift * 86400.0 * 1e6
    return shift, us_per_day

def compute_periodic_eccentricity(e, sqrt_a, ek, f_rel=F_REL_GPS):
    """Einstein periodic orbital eccentricity clock correction:
    dt_r = F_rel * e * sqrt(a) * sin(E_k)  (seconds).
    dr_r = -c * dt_r  (meters of pseudorange correction).
    """
    dt_r = f_rel * e * sqrt_a * math.sin(ek)
    dr_r = -C_LIGHT * dt_r
    return dt_r, dr_r

def compute_sagnac_delay(sat_x, sat_y, rx_x, rx_y):
    """Relativistic Sagnac Earth-rotation correction during signal transit:
    dt_sagnac = w_E / c^2 * (x_sat * y_rx - y_sat * x_rx)  (seconds).
    dr_sagnac = c * dt_sagnac  (meters).
    """
    dt_sagnac = (OMEGA_E / (C_LIGHT ** 2)) * (sat_x * rx_y - sat_y * rx_x)
    dr_sagnac = dt_sagnac * C_LIGHT
    return dt_sagnac, dr_sagnac

def analyze_satellite_relativity(eph, t_sow, site_ecef, site_llh):
    """Perform full relativistic space-time analysis on a single satellite."""
    sys_id = eph.get("sys", 0)
    mu = MU_GPS if sys_id == 0 else (MU_GAL if sys_id == 2 else MU_BDS)
    f_rel = F_REL_GPS if sys_id == 0 else (F_REL_GAL if sys_id == 2 else F_REL_BDS)

    sqrt_a = eph.get("sqrt_a")
    if not sqrt_a or sqrt_a <= 0:
        return None
    a = sqrt_a ** 2
    e = eph.get("e", 0.0)

    n0 = math.sqrt(mu / (a ** 3))
    tk = t_sow - eph.get("toe", 0.0)
    if tk > 302400.0:
        tk -= WEEK_S
    elif tk < -302400.0:
        tk += WEEK_S

    mk = eph.get("m0", 0.0) + (n0 + eph.get("delta_n", 0.0)) * tk
    ek = solve_kepler(mk, e)

    # Satellite radial distance
    se, ce = math.sin(ek), math.cos(ek)
    vk = math.atan2(math.sqrt(max(0.0, 1.0 - e ** 2)) * se, ce - e)
    phik = vk + eph.get("omega", 0.0)
    s2, c2 = math.sin(2.0 * phik), math.cos(2.0 * phik)
    uk = phik + eph.get("cus", 0.0) * s2 + eph.get("cuc", 0.0) * c2
    rk = a * (1.0 - e * ce) + eph.get("crs", 0.0) * s2 + eph.get("crc", 0.0) * c2
    ik = eph.get("i0", 0.0) + eph.get("cis", 0.0) * s2 + eph.get("cic", 0.0) * c2 + eph.get("idot", 0.0) * tk

    # Orbital speed in inertial frame
    v_orbital = math.sqrt(max(0.0, mu * (2.0 / rk - 1.0 / a)))

    # ECEF coordinates
    xp, yp = rk * math.cos(uk), rk * math.sin(uk)
    om_ = eph.get("omega0", 0.0) + (eph.get("omega_dot", 0.0) - OMEGA_E) * tk - OMEGA_E * eph.get("toe", 0.0)
    co, so, ci, si = math.cos(om_), math.sin(om_), math.cos(ik), math.sin(ik)
    sx = xp * co - yp * ci * so
    sy = xp * so + yp * ci * co
    sz = yp * si

    # Azimuth and Elevation
    az, el = ecef_to_azel((sx, sy, sz), site_ecef, site_llh[0], site_llh[1])

    # 1. General Relativity (Gravitational blueshift)
    gr_shift, gr_us_day = compute_gr_blueshift(rk, mu, A_EARTH)

    # 2. Special Relativity (Kinematic time dilation)
    sr_shift, sr_us_day = compute_sr_dilation(v_orbital)

    # 3. Combined net secular drift
    net_shift = gr_shift + sr_shift
    net_us_day = gr_us_day + sr_us_day
    drift_km_day = (net_us_day * 1e-6) * C_LIGHT / 1e3

    # 4. Periodic eccentricity correction
    dt_ecc, dr_ecc = compute_periodic_eccentricity(e, sqrt_a, ek, f_rel)

    # 5. Sagnac effect
    dt_sagnac, dr_sagnac = compute_sagnac_delay(sx, sy, site_ecef[0], site_ecef[1])

    # 6. Total instantaneous deformation
    total_inst_rel_m = dr_ecc + dr_sagnac

    return {
        "sys": SYS_NAMES.get(sys_id, "unknown"),
        "prn": eph.get("prn"),
        "az_deg": round(az, 1),
        "el_deg": round(el, 1),
        "eccentricity": round(e, 5),
        "semi_major_axis_km": round(a / 1e3, 1),
        "altitude_km": round((rk - A_EARTH) / 1e3, 1),
        "orbital_speed_km_s": round(v_orbital / 1e3, 3),
        "gr_blueshift_us_day": round(gr_us_day, 2),
        "sr_dilation_us_day": round(sr_us_day, 2),
        "net_secular_us_day": round(net_us_day, 2),
        "uncompensated_drift_km_day": round(drift_km_day, 2),
        "eccentricity_correction_ns": round(dt_ecc * 1e9, 2),
        "eccentricity_correction_m": round(dr_ecc, 2),
        "sagnac_correction_ns": round(dt_sagnac * 1e9, 2),
        "sagnac_correction_m": round(dr_sagnac, 2),
        "total_instantaneous_rel_m": round(total_inst_rel_m, 2)
    }

def run_inspector_cycle():
    """Execute one inspection cycle and write state.relativity.json atomically."""
    # 1. Load site anchor
    site_file = "/Volumes/Radiator 8TB/gnss/observations/site.json"
    site_llh = (39.0029556, -77.6051478, 77.1)
    if os.path.exists(site_file):
        try:
            with open(site_file) as f:
                sdata = json.load(f)
                site_llh = (sdata["lat"], sdata["lon"], sdata.get("h_m", 20.0))
        except Exception:
            pass
    site_ecef = llh_to_ecef(site_llh[0], site_llh[1], site_llh[2])

    # 2. Load ephemerides
    sys.path.insert(0, "/Volumes/Radiator 8TB/gnss/hackrf_gnss/scripts")
    try:
        from sky_producer import load_ephemeris
        ephemerides, leap_s, notes = load_ephemeris()
    except Exception as e:
        ephemerides = {}

    # 3. Load active tracker state
    tracker_file = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json"
    tracked_sats = []
    epoch = time.time()
    if os.path.exists(tracker_file):
        try:
            with open(tracker_file) as f:
                tdata = json.load(f)
                epoch = tdata.get("epoch", epoch)
                tracked_sats = tdata.get("tracker", {}).get("sats", [])
        except Exception:
            pass

    # GPS time of week from unix epoch
    t_sow = (epoch - 315964800.0 + 18.0) % WEEK_S

    # Analyze all tracked satellites (or visible ephemerides)
    sat_results = {}
    gr_list = []
    sr_list = []
    net_list = []
    ecc_m_list = []
    sagnac_m_list = []

    # Map tracked sats
    tracked_map = {}
    for ts in tracked_sats:
        sys_str = ts.get("sys", "gps").lower()
        sys_id = 0 if sys_str == "gps" else (1 if sys_str in ("beidou", "bds") else (2 if sys_str in ("galileo", "gal") else 3))
        prn = ts.get("prn")
        if prn:
            tracked_map[(sys_id, prn)] = ts

    # If we have tracked sats, analyze them; also include visible satellites from ephemeris
    target_keys = set(tracked_map.keys())
    if not target_keys:
        # Fall back to first 12 active ephemerides
        target_keys = set(list(ephemerides.keys())[:12])

    for (sys_id, prn) in target_keys:
        eph = ephemerides.get((sys_id, prn))
        if not eph:
            continue
        res = analyze_satellite_relativity(eph, t_sow, site_ecef, site_llh)
        if not res:
            continue

        key = f"{SYS_CODES.get(sys_id, 'SAT')}_{prn}"
        ts = tracked_map.get((sys_id, prn))
        if ts:
            res["cn0"] = ts.get("cn0_proxy", 0.0)
            res["lock_s"] = ts.get("lock_s", 0.0)
            res["doppler_hz"] = ts.get("doppler_hz", 0.0)

        sat_results[key] = res
        gr_list.append(res["gr_blueshift_us_day"])
        sr_list.append(res["sr_dilation_us_day"])
        net_list.append(res["net_secular_us_day"])
        ecc_m_list.append(abs(res["eccentricity_correction_m"]))
        sagnac_m_list.append(abs(res["sagnac_correction_m"]))

    mean_gr = sum(gr_list) / len(gr_list) if gr_list else 45.7
    mean_sr = sum(sr_list) / len(sr_list) if sr_list else -7.2
    mean_net = sum(net_list) / len(net_list) if net_list else 38.5
    mean_drift_km = (mean_net * 1e-6) * C_LIGHT / 1e3
    max_ecc = max(ecc_m_list) if ecc_m_list else 0.0
    max_sagnac = max(sagnac_m_list) if sagnac_m_list else 0.0

    # Factory oscillator offset: -delta_f/f * 10.23 MHz (in mHz)
    factory_offset_mhz = - (mean_net * 1e-6 / 86400.0) * 10.23e6 * 1e3

    output_state = {
        "epoch": round(epoch, 2),
        "ttl_s": 30.0,
        "station_lat": site_llh[0],
        "station_lon": site_llh[1],
        "station_h_m": site_llh[2],
        "relativity_summary": {
            "mean_gr_rate_us_day": round(mean_gr, 2),
            "mean_sr_rate_us_day": round(mean_sr, 2),
            "mean_net_secular_us_day": round(mean_net, 2),
            "uncompensated_drift_km_day": round(mean_drift_km, 2),
            "factory_frequency_offset_mhz": round(factory_offset_mhz, 2),
            "max_periodic_ecc_m": round(max_ecc, 2),
            "max_sagnac_m": round(max_sagnac, 2),
            "n_sat_analyzed": len(sat_results)
        },
        "relativity_satellites": sat_results
    }

    # Write atomically
    target_path = "/Volumes/Radiator 8TB/gnss/observations/state.relativity.json"
    tmp_path = target_path + f".tmp.{os.getpid()}"
    with open(tmp_path, "w") as f:
        json.dump(output_state, f, indent=2)
    os.replace(tmp_path, target_path)

    return output_state

def main():
    parser = argparse.ArgumentParser(description="Relativistic Space-Time Inspector Daemon")
    parser.add_argument("--loop", action="store_true", help="Run continuously in a loop")
    parser.add_argument("--interval", type=float, default=2.0, help="Loop poll interval in seconds (default: 2.0)")
    parser.add_argument("--once", action="store_true", help="Run a single evaluation cycle")
    args = parser.parse_args()

    print(f"Starting Relativistic Space-Time Inspector (interval={args.interval}s, loop={args.loop})...")

    while True:
        try:
            state = run_inspector_cycle()
            summ = state["relativity_summary"]
            n = summ["n_sat_analyzed"]
            print(f"[{time.strftime('%H:%M:%S')}] Relativity: {n} sats | GR={summ['mean_gr_rate_us_day']:+.2f} us/d, "
                  f"SR={summ['mean_sr_rate_us_day']:+.2f} us/d, Net={summ['mean_net_secular_us_day']:+.2f} us/d "
                  f"({summ['uncompensated_drift_km_day']:+.2f} km/d uncompensated) | Max Sagnac={summ['max_sagnac_m']:.1f} m")
        except Exception as e:
            print(f"Error in relativity cycle: {e}", file=sys.stderr)

        if not args.loop or args.once:
            break
        time.sleep(args.interval)

if __name__ == "__main__":
    main()
