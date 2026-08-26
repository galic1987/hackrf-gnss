#!/usr/bin/env python3
"""Unit tests for sky_producer az/el + orbit math. Plain asserts, no pytest.

Known cases:
  - satellite at the site zenith            -> el = +90 deg
  - satellite due east on the horizon       -> el = 0, az = 90 deg
  - circular equatorial orbit at toe        -> ECEF = (a, 0, 0) exactly
  - gated mask learning: stale tracker / fresh realign era / lock floor all
    gate learning OFF; a healthy tracker teaches "expected but not acquired"
  - versioned mask load: schema-less / site- / rig-mismatched masks are
    quarantined aside, matching provenance loads
  - live BRDC cross-check (if the real observations dir is present): the
    live INPUTS are copied into a tmp sandbox and pass_once runs there —
    production state/mask/history are NEVER touched (round-10 finding)

  python3 scripts/test_sky_producer.py
"""
import json
import math
import os
import shutil
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sky_producer as sp

REAL_OBS = sp.OBS  # live obs dir captured at import — pass_once NEVER runs
                   # against it; every pass below is rebound to a tmp sandbox

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

    # --- round-11: strict RINEX parsing (mirrors src/gps/broadcast.rs) ------
    def g_rec(prn, i0=0.30, sqrt_a=5153.6):
        return rnx_rec("G", prn, [
            (1e-5, 0.0, 0.0),
            (100.0, 0.0, 0.0, 0.3),
            (0.0, 0.0, 0.0, sqrt_a),
            (430000.0, 0.0, 1.0, 0.0),
            (i0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 2433.0, 0.0),
            (0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0)])

    hdr2 = ("     3.05           NAVIGATION DATA     MIXED               "
            "RINEX VERSION / TYPE\n" + " " * 60 + "END OF HEADER\n")

    # malformed core field (sqrt_a blanked): reject, never zero-fill;
    # the well-formed sibling survives
    bad = g_rec(9).replace(f"{5153.6:19.9e}", " " * 19)
    st = {}
    eph2 = sp.parse_rinex_nav(hdr2 + g_rec(7) + "\n" + bad + "\n", stats=st)
    check("malformed record rejected, sibling kept",
          st.get("rejected") == 1 and (0, 7) in eph2 and (0, 9) not in eph2,
          f"stats={st} keys={sorted(eph2)}")
    check("unit verdict surfaced in stats",
          (st.get("units") or {}).get("G") == "semicircles")

    # grey-band i0 (0.5: physically impossible under either unit) rejects the
    # record but must NOT flip the constellation's unit
    st = {}
    eph2 = sp.parse_rinex_nav(hdr2 + g_rec(7) + "\n" + g_rec(9, i0=0.5) + "\n",
                              stats=st)
    check("grey-band i0 rejected, unit not flipped",
          st.get("rejected") == 1 and (0, 7) in eph2 and (0, 9) not in eph2
          and abs(eph2[(0, 7)]["m0"] - 0.3 * math.pi) < 1e-9,
          f"stats={st}")

    # contradictory content (one sc-like + one rad-like record): fail closed
    st = {}
    eph2 = sp.parse_rinex_nav(hdr2 + g_rec(7) + "\n" + g_rec(9, i0=0.96) + "\n",
                              stats=st)
    check("contradictory unit content fails closed",
          (st.get("units") or {}).get("G") == "ambiguous"
          and st.get("rejected") == 2 and not eph2,
          f"stats={st}")

    # NaN spelling parses in Python but is not RINEX content
    nan_rec = g_rec(9).replace(f"{5153.6:19.9e}", f"{'NaN':>19}")
    st = {}
    eph2 = sp.parse_rinex_nav(hdr2 + nan_rec + "\n", stats=st)
    check("NaN field rejected", st.get("rejected") == 1 and not eph2,
          f"stats={st}")

    # high-inclination BDS IGSO (live regression: C09 at i0 = 1.0523 rad =
    # 60.3 deg in the 2026-08-26 BRDC) — the sanity range must not gate
    # legitimate satellites
    c09 = rnx_rec("C", 9, [
        (1e-5, 0.0, 0.0),
        (100.0, 0.0, 0.0, 0.3),
        (0.0, 0.0, 0.0, 5283.0),
        (430000.0, 0.0, 1.0, 0.0),
        (1.0523, 0.0, 0.0, 0.0),
        (0.0, 0.0, 1077.0, 0.0),
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0)])
    st = {}
    eph2 = sp.parse_rinex_nav(hdr2 + c09 + "\n", stats=st)
    check("60.3-deg IGSO (live C09) accepted under radians",
          st.get("rejected") == 0 and (1, 9) in eph2
          and abs(eph2[(1, 9)]["i0"] - 1.0523) < 1e-9
          and (st.get("units") or {}).get("C") == "radians",
          f"stats={st} keys={sorted(eph2)}")

    # --- round-14: minority-unit records are quarantined -------------------
    # a record whose raw i0 voted for the LOSING unit is rejected, never
    # scaled by the winner's factor into a silently garbage orbit
    txt = (hdr2 + g_rec(7) + "\n" + g_rec(8, i0=0.31) + "\n"
           + g_rec(9, i0=0.32) + "\n" + g_rec(10, i0=0.96) + "\n")
    st = {}
    eph2 = sp.parse_rinex_nav(txt, stats=st)
    check("semicircles winner: rad-band record quarantined",
          (st.get("units") or {}).get("G") == "semicircles"
          and st.get("rejected") == 1 and (0, 10) not in eph2
          and (0, 7) in eph2
          and abs(eph2[(0, 7)]["m0"] - 0.3 * math.pi) < 1e-9,
          f"stats={st} keys={sorted(eph2)}")
    txt = (hdr2 + g_rec(7, i0=0.95) + "\n" + g_rec(8, i0=0.96) + "\n"
           + g_rec(9, i0=0.97) + "\n" + g_rec(10, i0=0.30) + "\n")
    st = {}
    eph2 = sp.parse_rinex_nav(txt, stats=st)
    check("radians winner: sc-band record quarantined",
          (st.get("units") or {}).get("G") == "radians"
          and st.get("rejected") == 1 and (0, 10) not in eph2
          and abs(eph2[(0, 7)]["m0"] - 0.3) < 1e-12,
          f"stats={st} keys={sorted(eph2)}")
    st = {}
    eph2 = sp.parse_rinex_nav(hdr2 + g_rec(7, i0=0.95) + "\n"
                            + g_rec(8, i0=0.10) + "\n", stats=st)
    check("unit-neutral record still passes under the winner",
          st.get("rejected") == 0 and (0, 8) in eph2
          and (st.get("units") or {}).get("G") == "radians",
          f"stats={st} keys={sorted(eph2)}")


    # --- THE J2-sign regression test (round-9b) -----------------------------
    # Real consecutive broadcast records (BRDC 2026-08-26, GLONASS PRN 1):
    # propagate the first record exactly one 30-min record interval forward
    # and compare against the second record's broadcast position. With the
    # correct J2 term sign this lands within metres; with the wrong sign it
    # lands ~200 m out — the failure the shipped code once REPORTED as its
    # accuracy. This experiment, not an internal consistency check, is what
    # validates the model.
    glo_real_a = {
        "sys": 3, "prn": 1,
        "pos": (850913.5742188001, 13389219.72656, 21689637.20703),
        "vel": (-2062.1538162230004, 2142.408370972, -1240.293502808),
        "acc": (2.793967723846e-06, 0.0, -9.313225746155e-07),
        "tb_sow": 292518.0,
    }
    glo_truth_b = (-2343950.683594, 17243422.36328, 18647217.77344)
    pT, _ = sp.glo_state(glo_real_a, glo_real_a["tb_sow"] + 1800.0)
    errT = math.dist(pT, glo_truth_b)
    check("glo J2 sign: real record -> next record 30 min, < 10 m",
          errT < 10.0, f"err={errT:.2f} m (wrong sign gives ~200 m)")



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

    # --- sandboxed pass_once: gated mask learning + mask versioning ----------
    # Every pass_once below runs with paths rebound to a tmp sandbox via
    # sp.bind_obs — production state.sky.json / sky_mask.json /
    # sky_history.jsonl are NEVER touched (the old live cross-check mutated
    # the production mask and appended a production history row: round-10).
    def sandbox_site():
        d = tempfile.mkdtemp(prefix="sky_test_")
        json.dump({"lat": site_ll[0], "lon": site_ll[1], "h_m": site_ll[2]},
                  open(os.path.join(d, "site.json"), "w"))
        sp.bind_obs(d)
        return d

    def sat_row(prn, lock_s, sysn="gps"):
        return {"prn": prn, "sys": sysn, "lock_s": lock_s,
                "cn0_proxy": 40.0, "slip": False}

    def tracker_file(d, sats, mtime, ttl=1200.0):
        p = os.path.join(d, "state.tracker.json")
        json.dump({"epoch": mtime, "ttl_s": ttl, "tracker": {"sats": sats}},
                  open(p, "w"))
        os.utime(p, (mtime, mtime))

    # one GPS record whose sat sits above the site at fix_now (Kepler,
    # radians — i0=0.96 > 0.6 flips the parser's unit detection; row layout
    # mirrors the Galileo fixture above: sqrt_a on body line 2, toe/omega0 on
    # body line 3)
    days70 = sp.jdn(2026, 8, 26) - sp.jdn(1970, 1, 1)
    fix_now = float(days70 * 86400 + 12 * 3600 + 28 * 60)   # 2026-08-26T12:28Z
    toe = sp.gps_sow_unix(fix_now, 18.0)
    omega0 = (math.radians(site_ll[1]) + sp.OMEGA_E * toe) % (2 * math.pi)
    nav_txt = ("     3.05           NAVIGATION DATA     MIXED               "
               "RINEX VERSION / TYPE\n"
               + " " * 60 + "END OF HEADER\n"
               + rnx_rec("G", 3, [
                   (1e-5, 0.0, 0.0),
                   (0.0, 0.0, 0.0, 0.0),
                   (0.0, 0.0, 0.0, 5153.6),
                   (toe, 0.0, omega0, 0.0),
                   (0.96, 0.0, 0.0, 0.0),
                   (0.0, 0.0, 2433.0, 0.0),
                   (0.0, 0.0, 0.0, 0.0),
                   (0.0, 0.0, 0.0, 0.0)]) + "\n")

    healthy = [sat_row(p, 300.0) for p in (3, 5, 7, 9, 11, 13, 15, 17, 19, 21)]

    # stale tracker: "receiver unavailable" — nothing may be taught
    d = sandbox_site()
    open(sp.BRDC, "w").write(nav_txt)
    tracker_file(d, healthy, fix_now - 3600.0)          # mtime way past ttl
    st = sp.pass_once(fix_now)
    m = json.load(open(sp.MASK_PATH))
    check("stale tracker gates learning off",
          st["sky"]["mask"]["learning"] == "gated:tracker-stale"
          and m["bins"] == {})
    check("gated pass counted as gated, not learned",
          m["provenance"]["passes_gated"] == 1
          and m["provenance"]["passes_learned"] == 0)
    check("stale pass still classifies (panel picture)",
          any(s["cls"] == "absent" for s in st["sky"]["sats"]))

    # fresh realign era: locks younger than the covered window (post-restart)
    d = sandbox_site()
    open(sp.BRDC, "w").write(nav_txt)
    young = [sat_row(p, 10.0) for p in (3, 5, 7, 9, 11, 13, 15, 17, 19, 21)]
    tracker_file(d, young, fix_now)
    st = sp.pass_once(fix_now)
    check("young locks (realign era) gate learning off",
          st["sky"]["mask"]["learning"] == "gated:locks-young"
          and json.load(open(sp.MASK_PATH))["bins"] == {})

    # fresh but below the healthy lock floor
    tracker_file(d, [sat_row(p, 300.0) for p in (3, 5, 7)], fix_now)
    st = sp.pass_once(fix_now)
    check("below lock floor gates learning off",
          st["sky"]["mask"]["learning"] == "gated:few-locks"
          and json.load(open(sp.MASK_PATH))["bins"] == {})

    # healthy tracker: learning ON; modeled+locked sat tracked
    d = sandbox_site()
    open(sp.BRDC, "w").write(nav_txt)
    tracker_file(d, healthy, fix_now)
    st = sp.pass_once(fix_now)
    g3 = [s for s in st["sky"]["sats"] if s["sys"] == "gps" and s["prn"] == 3]
    m = json.load(open(sp.MASK_PATH))
    check("healthy tracker teaches the mask (exp+lock)",
          st["sky"]["mask"]["learning"] == "learning"
          and sum(b["exp"] for b in m["bins"].values()) == 1
          and sum(b["lock"] for b in m["bins"].values()) == 1,
          f"bins={m['bins']}")
    check("provenance learn-start + pass counts set",
          m["provenance"]["learn_start_epoch"] == fix_now
          and m["provenance"]["passes_learned"] == 1)
    check("modeled+locked sat classified tracked", bool(g3)
          and g3[0]["cls"] == "tracked", f"{g3}")

    # healthy tracker, modeled sat unobserved: "expected but not acquired" IS
    # taught (exp without lock) — the distinction from the gated cases above
    d = sandbox_site()
    open(sp.BRDC, "w").write(nav_txt)
    tracker_file(d, [sat_row(p, 300.0)
                     for p in (5, 7, 9, 11, 13, 15, 17, 19, 21, 23)], fix_now)
    st = sp.pass_once(fix_now)
    g3 = [s for s in st["sky"]["sats"] if s["sys"] == "gps" and s["prn"] == 3]
    m = json.load(open(sp.MASK_PATH))
    check("not-acquired sat taught as absent (exp without lock)",
          bool(g3) and g3[0]["cls"] == "absent"
          and sum(b["exp"] for b in m["bins"].values()) == 1
          and sum(b["lock"] for b in m["bins"].values()) == 0)

    # --- versioned load: old schema / mismatched provenance are retired ------
    d = sandbox_site()
    old = {"bin_deg": 5, "site": [39.0, -77.6, 20.0], "epoch": 1,
           "bins": {"A000E+00": {"exp": 100, "lock": 0}}}
    json.dump(old, open(sp.MASK_PATH, "w"))
    m = sp.load_mask(site_ll, "test-rig")
    q = [f for f in os.listdir(d) if f.startswith("sky_mask.json.quarantine-")]
    check("schema-less mask retired to fresh schema 2",
          m["schema"] == sp.MASK_SCHEMA and m["bins"] == {})
    check("retired mask quarantined, not deleted",
          len(q) == 1
          and json.load(open(os.path.join(d, q[0])))["bins"] == old["bins"],
          f"quarantine={q}")
    check("fresh mask carries provenance (site/rig/epochs/counts)",
          m["provenance"]["site"]["lat"] == site_ll[0]
          and m["provenance"]["rig"] == "test-rig"
          and m["provenance"]["created_epoch"] > 0
          and m["provenance"]["passes_learned"] == 0)

    moved = {"schema": sp.MASK_SCHEMA, "bin_deg": 5, "epoch": 1,
             "provenance": {"site": {"lat": 10.0, "lon": 10.0, "h_m": 0.0},
                            "rig": "test-rig", "created_epoch": 1,
                            "learn_start_epoch": 1, "passes_learned": 5,
                            "passes_gated": 0},
             "bins": {"A000E+00": {"exp": 50, "lock": 1}}}
    json.dump(moved, open(sp.MASK_PATH, "w"))
    m = sp.load_mask(site_ll, "test-rig")
    check("site-mismatched mask retired", m["bins"] == {})

    json.dump(moved, open(sp.MASK_PATH, "w"))
    m = sp.load_mask(site_ll, "other-rig")
    check("rig-mismatched mask retired", m["bins"] == {})

    ok_mask = dict(moved)
    ok_mask["provenance"] = dict(moved["provenance"],
                                 site={"lat": site_ll[0], "lon": site_ll[1],
                                       "h_m": site_ll[2]})
    json.dump(ok_mask, open(sp.MASK_PATH, "w"))
    m = sp.load_mask(site_ll, "test-rig")
    check("matching provenance mask kept (bins intact)",
          m["bins"].get("A000E+00") == {"exp": 50, "lock": 1})

    # --- live cross-check in a sandbox: copies the live INPUTS (site anchor,
    # BRDC, tracker eph + state — mtimes preserved so ttl semantics hold) and
    # runs pass_once against the copies. Writes land in the sandbox only. -----
    live_inputs = [os.path.join(REAL_OBS, f) for f in
                   ("site.json", "brdc_latest.rnx", "tracker_eph.json",
                    "state.tracker.json")]
    if all(os.path.exists(p) for p in live_inputs):
        d = tempfile.mkdtemp(prefix="sky_live_")
        for p in live_inputs:
            shutil.copy2(p, os.path.join(d, os.path.basename(p)))
        sp.bind_obs(d)
        st = sp.pass_once(time.time())
        if st is None:
            check("live pass has an anchor", False, "pass_once returned None")
            st = {"sky": {"sats": [], "counts": {}, "eph": "?"}}
        sky = st["sky"]
        print(f"  live: eph={sky['eph']} counts={sky['counts']} "
              f"learning={sky['mask']['learning']}")
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
        check("live pass wrote only into the sandbox",
              all(os.path.exists(os.path.join(d, f)) for f in
                  ("state.sky.json", "sky_mask.json", "sky_history.jsonl"))
              and sp.MASK_PATH.startswith(d))
    else:
        print("skip  live cross-check (no observations dir)")

    print(f"\n{len(FAILURES)} failure(s)" if FAILURES else "\nall tests passed")
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
