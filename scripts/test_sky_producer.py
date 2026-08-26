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
    # anchor comes from observations/site.json via sp.SITE (None sandboxed);
    # the geometry cases work for any site, so a fixture stands in then
    site_ll = sp.SITE if sp.SITE else (39.0032, -77.6058, 20.0)
    site = sp.geodetic_to_ecef(*site_ll)
    la, lo = math.radians(site_ll[0]), math.radians(site_ll[1])
    up = (math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la))
    east = (-math.sin(lo), math.cos(lo), 0.0)
    north = (-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la))

    # --- zenith -> el 90 --------------------------------------------------
    R = 20200e3
    sat = tuple(site[i] + up[i] * R for i in range(3))
    az, el = sp.ecef_to_azel(sat, site, site_ll[0], site_ll[1])
    check("zenith satellite -> el +90", abs(el - 90.0) < 1e-9, f"el={el}")

    # --- east horizon -> el 0, az 90 ---------------------------------------
    sat = tuple(site[i] + east[i] * R for i in range(3))
    az, el = sp.ecef_to_azel(sat, site, site_ll[0], site_ll[1])
    check("east horizon -> el 0", abs(el) < 1e-6, f"el={el}")
    check("east horizon -> az 90", abs(az - 90.0) < 1e-6, f"az={az}")

    sat = tuple(site[i] + north[i] * R for i in range(3))
    az, el = sp.ecef_to_azel(sat, site, site_ll[0], site_ll[1])
    check("north horizon -> az 0", abs(az) < 1e-6 or abs(az - 360) < 1e-6, f"az={az}")

    # below the site -> el -90
    sat = tuple(site[i] - up[i] * R for i in range(3))
    _, el = sp.ecef_to_azel(sat, site, site_ll[0], site_ll[1])
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

    # --- motion fields (alt/speed/track for the panel, 2026-08-25) ----------
    alt_km, speed_mps, track_deg = sp.sat_motion(e, 3600.0, site_ll[0], site_ll[1])
    check("GPS-nominal alt ~20180 km", abs(alt_km - 20180.0) < 50.0,
          f"alt={alt_km:.0f}")
    # prograde equatorial case: ECEF speed = orbital sqrt(mu/a) minus the
    # frame's rotation rate at orbital radius (OMEGA_E * a) = 3874 - 1937
    check("ECEF speed = orbital minus rotation", abs(speed_mps - 1937.0) < 10.0,
          f"speed={speed_mps:.0f}")
    check("track heading in [0, 360)", 0.0 <= track_deg < 360.0,
          f"track={track_deg:.1f}")

    # --- classification logic on a synthetic pass -----------------------------
    mask = {"exp": 100, "lock": 0}
    check("learned mask bin flags masked", sp.bin_masked(mask))
    check("thin bin not masked", not sp.bin_masked({"exp": 3, "lock": 0}))
    check("healthy bin not masked", not sp.bin_masked({"exp": 100, "lock": 40}))

    # --- Galileo / GLONASS parsing + propagation -------------------------------
    def rnx_rec(letter, prn, fields8):
        hdr = f"{letter}{prn:2d} 2026 08 26 12 00 00" + "".join(
            f"{v:19.9e}" for v in fields8[0])
        body = ["    " + "".join(f"{v:19.9e}" for v in row)
                for row in fields8[1:]]
        return "\n".join([hdr] + body)

    # Galileo E record: circular a=29600 km (alt ~23222), i0=0.96 rad
    gal = rnx_rec("E", 11, [
        (1e-5, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, math.sqrt(29600e3)),
        (430000.0, 0.0, 1.0, 0.0),
        (0.96, 0.0, 0.0, 0.0),
        (0.0, 0.0, 2433.0, 0.0),
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0)])
    # GLONASS R records: circular equatorial r=25508 km; ECEF-frame speed
    # (n - Omega)*r (the broadcast frame is the rotating PZ-90)
    r_glo = 25508.0
    n_glo = math.sqrt(sp.MU_GLO / (r_glo * 1e3) ** 3)
    v_glo = (n_glo - sp.OMEGA_GLO) * r_glo  # km/s
    glo_a = rnx_rec("R", 5, [
        (1e-4, 1e-12, 43200.0),
        (r_glo, 0.0, 0.0, 0.0),
        (0.0, v_glo, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0)])
    glo_b = rnx_rec("R", 5, [       # same PRN, tb 30 min later
        (1e-4, 1e-12, 45000.0),
        (r_glo * math.cos(0.224), -v_glo * math.sin(0.224), 0.0, 0.0),
        (r_glo * math.sin(0.224), v_glo * math.cos(0.224), 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0)]).replace("12 00 00", "12 30 00")
    glo_sick = rnx_rec("R", 6, [    # Bn != 0 -> must be skipped
        (1e-4, 1e-12, 43200.0),
        (r_glo, 0.0, 0.0, 1.0),
        (0.0, v_glo, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0)])
    mini = ("     3.05           NAVIGATION DATA     MIXED               "
            "RINEX VERSION / TYPE\n"
            "                                                            "
            "END OF HEADER\n"
            + gal + "\n" + glo_a + "\n" + glo_b + "\n" + glo_sick + "\n")
    days = sp.jdn(2026, 8, 26) - sp.jdn(1980, 1, 6)
    now_sow = (days * 86400 + 12 * 3600 + 28 * 60 + 18.0) % sp.WEEK_S
    eph = sp.parse_rinex_nav(mini, now_sow=now_sow, leap_s=18.0)
    check("galileo E record parsed", (2, 11) in eph)
    check("glonass R record parsed, sick one skipped",
          (3, 5) in eph and (3, 6) not in eph, f"keys={sorted(eph)}")
    eg = eph.get((2, 11))
    check("galileo sqrt_a right field",
          eg and abs(eg["sqrt_a"] - math.sqrt(29600e3)) < 1e-3,
          f"sqrt_a={eg and eg['sqrt_a']}")
    pg = sp.sat_pos_ecef(eg, eg["toe"], sp.MU_GAL, sp.OMEGA_GAL)
    rg = math.sqrt(sum(v * v for v in pg))
    check("galileo radius ~29600 km", abs(rg - 29600e3) < 1e3, f"r={rg:.0f}")
    alt_gal = (rg - sp.A_E) / 1e3
    check("galileo alt ~23222 km", abs(alt_gal - 23222.0) < 50.0,
          f"alt={alt_gal:.0f}")
    gr = eph.get((3, 5))
    check("glonass units km->m, nearest tb picked (12:30 not 12:00)",
          gr and abs(gr["pos"][0] - r_glo * math.cos(0.224) * 1e3) < 1.0
          and abs(gr["tb_sow"] - ((days * 86400 + 45000.0 + 18.0)
                                  % sp.WEEK_S)) < 1e-6,
          f"pos0={gr and gr['pos'][0]:.1f} tb={gr and gr['tb_sow']:.1f}")
    # propagate the 12:30 record 30 s forward: radius must hold, motion sane
    p1, v1 = sp.glo_state(gr, gr["tb_sow"] + 30.0)
    r1 = math.sqrt(sum(v * v for v in p1))
    s1 = math.sqrt(sum(v * v for v in v1))
    check("glo RK4 holds radius (+30 s)", abs(r1 - r_glo * 1e3) < 200.0,
          f"r={r1:.0f}")
    check("glo ECEF speed ~ (n-Omega)*r", abs(s1 - v_glo * 1e3) < 50.0,
          f"v={s1:.0f}")
    moved = math.dist(p1, gr["pos"])
    check("glo 30 s displacement ~63 km", 50e3 < moved < 80e3, f"d={moved:.0f}")
    # inertial-frame speed must come out orbital (~3.95 km/s): v + Omega x r
    wxr = (-sp.OMEGA_GLO * p1[1], sp.OMEGA_GLO * p1[0], 0.0)
    vi = math.sqrt(sum((v1[k] + wxr[k]) ** 2 for k in range(3)))
    check("glo inertial speed ~3.95 km/s", abs(vi - 3953.0) < 60.0, f"vi={vi:.0f}")
    # long arc stability: a full orbit must not blow up (J2-only model)
    old_fit = sp.GLO_FIT_S
    sp.GLO_FIT_S = 1e9
    p2, _ = sp.glo_state(eph[(3, 5)], gr["tb_sow"] + 40544.0)
    sp.GLO_FIT_S = old_fit
    r2 = math.sqrt(sum(v * v for v in p2))
    check("glo RK4 stable over one full period",
          abs(r2 - r_glo * 1e3) < 500e3, f"r={r2:.0f}")
    try:
        sp.glo_state(gr, gr["tb_sow"] + sp.GLO_FIT_S + 1.0)
        check("glo fit window enforced", False)
    except ValueError:
        check("glo fit window enforced", True)


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
        if st is None:
            check("live pass has an anchor", False, "pass_once returned None")
            st = {"sky": {"sats": [], "counts": {}, "eph": "?"}}
        sky = st["sky"]
        print(f"  live: eph={sky['eph']} counts={sky['counts']}")
        locked_low = [s for s in sky["sats"]
                      if s["cls"] == "unexpected" and s["el_deg"] < -2.0]
        check("no locked sat deep below horizon (sign check)", not locked_low,
              f"{[(s['sys'], s['prn'], s['el_deg']) for s in locked_low]}")
        check("some GPS sats modeled", sky["counts"]["modeled"] >= 4,
              f"modeled={sky['counts']['modeled']}")
        gal = [s for s in sky["sats"] if s["sys"] == "galileo"]
        glo = [s for s in sky["sats"] if s["sys"] == "glonass"]
        check("galileo modeled from BRDC", len(gal) >= 4, f"gal={len(gal)}")
        check("glonass modeled from BRDC", len(glo) >= 4, f"glo={len(glo)}")
        check("glonass rows all predicted-only",
              bool(glo) and all(s["cls"] == "predicted" for s in glo))
        check("glonass alt ~19130 km",
              bool(glo) and all(18500 <= s["alt_km"] <= 19800 for s in glo),
              f"{[s['alt_km'] for s in glo][:6]}")
        check("galileo alt plausible (E14/E18 are eccentric)",
              bool(gal) and all(17000 <= s["alt_km"] <= 27000
                                for s in gal))
        cov = sum(1 for s in sky["sats"] if s["cls"] in ("tracked", "absent"))
        check("predicted-only sats stay out of coverage counts",
              sky["counts"]["expected"] == cov
              and sky["counts"]["predicted"] == len(glo),
              f"expected={sky['counts']['expected']} cov={cov} "
              f"predicted={sky['counts']['predicted']} glo={len(glo)}")
        gal_low = [s for s in gal
                   if s["cls"] == "unexpected" and s["el_deg"] < -2.0]
        check("no locked galileo deep below horizon", not gal_low,
              f"{[(s['prn'], s['el_deg']) for s in gal_low]}")
    else:
        print("skip  live cross-check (no observations dir)")

    print(f"\n{len(FAILURES)} failure(s)" if FAILURES else "\nall tests passed")
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
