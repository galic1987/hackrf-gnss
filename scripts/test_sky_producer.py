#!/usr/bin/env python3
"""Unit tests for sky_producer az/el + orbit math. Plain asserts, no pytest.

Known cases:
  - satellite at the site zenith            -> el = +90 deg
  - satellite due east on the horizon       -> el = 0, az = 90 deg
  - circular equatorial orbit at toe        -> ECEF = (a, 0, 0) exactly
  - live BRDC cross-check (if the real observations dir is present):
    every currently-locked GPS sat must be above the horizon (sign check)

  python3 scripts/test_sky_producer.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sky_producer as sp

FAILURES = []


def check(name, cond, detail=""):
    print(f"{'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILURES.append(name)


def main():
    site = sp.geodetic_to_ecef(sp.SITE_LAT, sp.SITE_LON, sp.SITE_H)
    la, lo = math.radians(sp.SITE_LAT), math.radians(sp.SITE_LON)
    up = (math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la))
    east = (-math.sin(lo), math.cos(lo), 0.0)
    north = (-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la))

    # --- zenith -> el 90 --------------------------------------------------
    R = 20200e3
    sat = tuple(site[i] + up[i] * R for i in range(3))
    az, el = sp.ecef_to_azel(sat, site, sp.SITE_LAT, sp.SITE_LON)
    check("zenith satellite -> el +90", abs(el - 90.0) < 1e-9, f"el={el}")

    # --- east horizon -> el 0, az 90 ---------------------------------------
    sat = tuple(site[i] + east[i] * R for i in range(3))
    az, el = sp.ecef_to_azel(sat, site, sp.SITE_LAT, sp.SITE_LON)
    check("east horizon -> el 0", abs(el) < 1e-6, f"el={el}")
    check("east horizon -> az 90", abs(az - 90.0) < 1e-6, f"az={az}")

    sat = tuple(site[i] + north[i] * R for i in range(3))
    az, el = sp.ecef_to_azel(sat, site, sp.SITE_LAT, sp.SITE_LON)
    check("north horizon -> az 0", abs(az) < 1e-6 or abs(az - 360) < 1e-6, f"az={az}")

    # below the site -> el -90
    sat = tuple(site[i] - up[i] * R for i in range(3))
    _, el = sp.ecef_to_azel(sat, site, sp.SITE_LAT, sp.SITE_LON)
    check("nadir satellite -> el -90", abs(el + 90.0) < 1e-9, f"el={el}")

    # --- circular equatorial orbit at toe -> (a, 0, 0) ----------------------
    e = {"sqrt_a": 5153.6, "e": 0.0, "m0": 0.0, "delta_n": 0.0, "omega0": 0.0,
         "omega": 0.0, "i0": 0.0, "idot": 0.0, "omega_dot": 0.0, "cuc": 0.0,
         "cus": 0.0, "crc": 0.0, "crs": 0.0, "cic": 0.0, "cis": 0.0,
         "toe": 0.0, "toc": 0.0}
    p = sp.sat_pos_ecef(e, 0.0)
    a = 5153.6 ** 2
    check("circular orbit at toe -> (a,0,0)",
          abs(p[0] - a) < 1e-6 and abs(p[1]) < 1e-6 and abs(p[2]) < 1e-6,
          f"p={p} a={a}")
    r = math.sqrt(sum(x * x for x in p))
    check("radius is GPS-nominal", abs(r - 26_560_000.0) < 30_000.0, f"r={r:.0f}")

    # 60 s later the orbit must have moved ~3.9 km/s * 60 s (cross-check
    # against the Rust test in broadcast.rs: 100-300 km window)
    p1 = sp.sat_pos_ecef(e, 60.0)
    d = math.sqrt(sum((p1[i] - p[i]) ** 2 for i in range(3)))
    check("60 s displacement ~234 km", 100_000.0 < d < 300_000.0, f"d={d:.0f}")
    check("orbit stays equatorial (z=0)", abs(p1[2]) < 1e-3, f"z={p1[2]}")

    # --- wrap_tk across the week boundary ------------------------------------
    check("wrap_tk +/- half week",
          sp._wrap_tk(604800.0 - 10.0 - 604800.0) == -10.0
          and abs(sp._wrap_tk(-400000.0) - 204800.0) < 1e-9)

    # --- classification logic on a synthetic pass -----------------------------
    mask = {"exp": 100, "lock": 0}
    check("learned mask bin flags masked", sp.bin_masked(mask))
    check("thin bin not masked", not sp.bin_masked({"exp": 3, "lock": 0}))
    check("healthy bin not masked", not sp.bin_masked({"exp": 100, "lock": 40}))

    # --- roller parser mapping -------------------------------------------------
    import archive_roller as roller
    line = {"t": 1787614546.0, "sats": [
        {"sys": "gps", "prn": 3, "az_deg": 123.4, "el_deg": 45.6,
         "cls": "tracked", "cn0": 36.8, "lock_s": 381.0,
         "doppler_hz": -3087.1, "rho_m": None, "t_tx": 171336.0, "ppm": -1.96}]}
    rows = roller.parse_sky(line)["satellite"]
    check("parse_sky fills reserved az/el columns",
          len(rows) == 1 and rows[0]["az_deg"] == 123.4
          and rows[0]["el_deg"] == 45.6 and rows[0]["cn0"] == 36.8,
          f"row={rows[0] if rows else None}")
    check("parse_sky survives junk", roller.parse_sky({}) == {})

    # --- live cross-check against the real sky (skipped in sandboxes) -----------
    if os.path.exists(sp.BRDC) and os.path.exists(sp.TRACKER_STATE):
        import time as _t
        st = sp.pass_once(_t.time())
        sky = st["sky"]
        print(f"  live: eph={sky['eph']} counts={sky['counts']}")
        locked_low = [s for s in sky["sats"]
                      if s["cls"] == "unexpected" and s["el_deg"] < -2.0]
        check("no locked sat deep below horizon (sign check)", not locked_low,
              f"{[(s['sys'], s['prn'], s['el_deg']) for s in locked_low]}")
        check("some GPS sats modeled", sky["counts"]["modeled"] >= 4,
              f"modeled={sky['counts']['modeled']}")
    else:
        print("skip  live cross-check (no observations dir)")

    print(f"\n{len(FAILURES)} failure(s)" if FAILURES else "\nall tests passed")
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
