#!/usr/bin/env python3
"""Sky-visibility producer: expected vs observed satellites, ~30 s cadence.

For every GPS (+ BeiDou, where ephemeris exists) satellite with a current
broadcast ephemeris, computes az/el from the surveyed site (ECEF -> ENU,
IS-GPS-200 Table 20-IV orbit math — a pure-Python port of
src/gps/broadcast.rs, dependency-free), diffs against the tracker's live
state and classifies each satellite:

  tracked     observed (locked) and above the mask      — nominal
  absent      above horizon/mask but not observed        — blockage/fade
  unexpected  observed but BELOW the current mask        — reflection/spoof candidate
  below       not observed and below the mask            — not expected
  unmodeled   tracked by the tracker but no ephemeris    — galileo/sbas today

Ephemeris sources (freshest wins per PRN):
  - observations/brdc_latest.rnx   (BKG BRDC, full GPS constellation + BDS)
  - observations/tracker_eph.json  (self-decoded live: GPS + BeiDou)

Writes (sole writer of each; tmp + os.replace, never read-modify-write of
other producers' state):
  observations/state.sky.json      merged by the panel server at read time
  observations/sky_mask.json       persistent 5°x5° az/el lock histogram —
                                   over days this paints the antenna's true
                                   visibility, including reflection islands
  observations/sky_history.jsonl   append-only per-sat az/el/visibility rows
                                   (rolled into the archive by
                                   scripts/archive_roller.py)

Touches NO radio, signals NO process. Read-only on every other file.
Respects the station ttl rule: a state file older (mtime) than its ttl_s is
dead and its values don't count.

Usage:
  sky_producer.py            # one pass (also: --once)
  sky_producer.py --loop     # forever, 30 s cadence
"""
import json
import math
import os
import sys
import time

OBS = os.environ.get("HACKRF_GNSS_OBS", "/Volumes/Radiator 8TB/gnss/observations")
STATE = os.path.join(OBS, "state.sky.json")
MASK_PATH = os.path.join(OBS, "sky_mask.json")
HIST = os.path.join(OBS, "sky_history.jsonl")
TRACKER_STATE = os.path.join(OBS, "state.tracker.json")
TRACKER_EPH = os.path.join(OBS, "tracker_eph.json")
BRDC = os.path.join(OBS, "brdc_latest.rnx")

SITE_LAT, SITE_LON, SITE_H = 39.0032, -77.6058, 20.0  # mast by the window
CADENCE_S = 30.0
TTL_S = 90.0                       # 3 missed cycles before we tombstone out
EL_MASK_DEG = 5.0                  # static horizon mask until learned
BIN_DEG = 5                        # az/el histogram resolution
MASK_MIN_SAMPLES = 30              # bin needs this many expected samples ...
MASK_MAX_FRAC = 0.05               # ... and <5% lock rate to count as masked

# WGS-84 / IS-GPS-200 constants (match src/gps/broadcast.rs)
A_E = 6378137.0
F_E = 1.0 / 298.257223563
E2 = F_E * (2.0 - F_E)
MU_E = 3.986005e14
OMEGA_E = 7.2921151467e-5
WEEK_S = 604800.0

SYS_NAME = {0: "gps", 1: "beidou"}
SYS_ID = {"gps": 0, "beidou": 1}
BDS_GEO_PRNS = set(range(1, 6)) | set(range(59, 64))  # GEO: ICD MEO math wrong


# --- time ------------------------------------------------------------------

def jdn(y, m, d):
    a = (14 - m) // 12
    yy = y + 4800 - a
    mm = m + 12 * a - 3
    return d + (153 * mm + 2) // 5 + 365 * yy + yy // 4 - yy // 100 + yy // 400 - 32045


def sow_from_calendar(y, mo, d, h, mi, s, offset_s):
    """Seconds-of-week treating the calendar fields as (XDT) and adding
    offset_s to reach GPST-equivalent: UTC needs +leap(18), BDT needs +14.
    Mirrors gps_sow() in src/gps/broadcast.rs."""
    days = jdn(y, mo, d) - jdn(1980, 1, 6)
    secs = days * 86400 + h * 3600 + mi * 60 + s + offset_s
    return secs % WEEK_S


GPS_EPOCH_UNIX = 315964800.0      # 1980-01-06 00:00:00 UTC


def gps_sow_unix(unix_t, leap_s=18.0):
    return (unix_t + leap_s - GPS_EPOCH_UNIX) % WEEK_S


# --- orbit math (port of src/gps/broadcast.rs) -------------------------------

def _kepler_e(m, ecc):
    ek = m
    for _ in range(12):
        ek -= (ek - ecc * math.sin(ek) - m) / (1.0 - ecc * math.cos(ek))
    return ek


def _wrap_tk(tk):
    if tk > WEEK_S / 2.0:
        tk -= WEEK_S
    elif tk < -WEEK_S / 2.0:
        tk += WEEK_S
    return tk


def sat_pos_ecef(e, t):
    """Satellite ECEF (m) at GPST-equivalent time-of-week t (s)."""
    a = e["sqrt_a"] ** 2
    n0 = math.sqrt(MU_E / a ** 3)
    tk = _wrap_tk(t - e["toe"])
    mk = e["m0"] + (n0 + e["delta_n"]) * tk
    ek = _kepler_e(mk, e["e"])
    se, ce = math.sin(ek), math.cos(ek)
    vk = math.atan2(math.sqrt(1.0 - e["e"] ** 2) * se, ce - e["e"])
    phik = vk + e["omega"]
    s2, c2 = math.sin(2 * phik), math.cos(2 * phik)
    uk = phik + e["cus"] * s2 + e["cuc"] * c2
    rk = a * (1.0 - e["e"] * ce) + e["crs"] * s2 + e["crc"] * c2
    ik = e["i0"] + e["cis"] * s2 + e["cic"] * c2 + e["idot"] * tk
    xp, yp = rk * math.cos(uk), rk * math.sin(uk)
    om = e["omega0"] + (e["omega_dot"] - OMEGA_E) * tk - OMEGA_E * e["toe"]
    co, so, ci, si = math.cos(om), math.sin(om), math.cos(ik), math.sin(ik)
    return (xp * co - yp * ci * so, xp * so + yp * ci * co, yp * si)


# --- geodesy ------------------------------------------------------------------

def geodetic_to_ecef(lat_deg, lon_deg, h):
    la, lo = math.radians(lat_deg), math.radians(lon_deg)
    n = A_E / math.sqrt(1.0 - E2 * math.sin(la) ** 2)
    return ((n + h) * math.cos(la) * math.cos(lo),
            (n + h) * math.cos(la) * math.sin(lo),
            (n * (1.0 - E2) + h) * math.sin(la))


def ecef_to_azel(sat, site, lat_deg, lon_deg):
    """(az_deg, el_deg) of sat ECEF seen from site ECEF at geodetic lat/lon."""
    dx, dy, dz = sat[0] - site[0], sat[1] - site[1], sat[2] - site[2]
    la, lo = math.radians(lat_deg), math.radians(lon_deg)
    e = -math.sin(lo) * dx + math.cos(lo) * dy
    n = -math.sin(la) * math.cos(lo) * dx - math.sin(la) * math.sin(lo) * dy + math.cos(la) * dz
    u = math.cos(la) * math.cos(lo) * dx + math.cos(la) * math.sin(lo) * dy + math.sin(la) * dz
    az = math.degrees(math.atan2(e, n)) % 360.0
    el = math.degrees(math.atan2(u, math.hypot(e, n)))
    return az, el


# --- ephemeris sources ---------------------------------------------------------

def _df(s):
    t = s.strip()
    if not t:
        return 0.0
    try:
        return float(t.replace("D", "E").replace("d", "E"))
    except ValueError:
        return 0.0


def _fld(line, a, b):
    return line[a:min(b, len(line))] if a < len(line) else ""


def parse_rinex_nav(text):
    """GPS (G) + BeiDou (C) records of a RINEX-3 MIXED nav file -> {(sys,prn): eph}.
    Port of parse_rinex_gps/parse_rinex_bds (src/gps/broadcast.rs,
    src/beidou_d1.rs): latest toe wins per PRN; BKG files carry angles in
    RADIANS (detected from first record's i0); BDS times shifted BDT->GPST
    (+14 s) so one timescale serves all constellations. BDS GEOs skipped."""
    lines = text.splitlines()
    hdr = 0
    while hdr < len(lines) and "END OF HEADER" not in lines[hdr]:
        hdr += 1
    hdr += 1
    # unit detection per constellation: take the MAX |i0| over the early
    # records — a GEO's small inclination (i0 ~ 0.02 rad) would slip under
    # the radians threshold and flip the whole file to semicircles
    # (mirrors parse_rinex_gps / parse_rinex_bds in src/).
    ang = {}
    for want in ("G", "C"):
        a = math.pi  # spec default: semicircles
        max_i0 = 0.0
        j = hdr
        scanned = 0
        while j + 4 < len(lines) and scanned < 64:
            if lines[j].startswith(want) and len(lines[j]) > 4:
                max_i0 = max(max_i0, abs(_df(_fld(lines[j + 4], 4, 23))))
                scanned += 1
                j += 8
                continue
            j += 1
        if max_i0 > 0.6:
            a = 1.0  # radians (BKG)
        ang[want] = a
    out = {}
    i = hdr
    while i < len(lines):
        ln = lines[i]
        if not ln or ln[:1] not in ("G", "C") or i + 7 >= len(lines):
            i += 1
            continue
        is_bds = ln[0] == "C"
        try:
            prn = int(_fld(ln, 1, 3))
        except ValueError:
            i += 1
            continue
        if is_bds and prn in BDS_GEO_PRNS:
            i += 8
            continue
        try:
            y, mo, d = (int(_fld(ln, 4, 8)), int(_fld(ln, 9, 11)), int(_fld(ln, 12, 14)))
            h, mi_, s = (int(_fld(ln, 15, 17)), int(_fld(ln, 18, 20)), int(_fld(ln, 21, 23)))
        except ValueError:
            i += 1
            continue
        b = lines[i + 1:i + 8]
        f = lambda l, k: _df(_fld(b[l], 4 + k * 19, 4 + (k + 1) * 19))
        an = ang[ln[0]]
        dt = 14.0 if is_bds else 18.0      # BDT+14=GPST ; UTC+leap(18)=GPST
        e = {
            "sys": 1 if is_bds else 0, "prn": prn,
            "af0": _df(_fld(ln, 23, 42)), "af1": _df(_fld(ln, 42, 61)),
            "af2": _df(_fld(ln, 61, 80)),
            "crs": f(0, 1), "delta_n": f(0, 2) * an, "m0": f(0, 3) * an,
            "cuc": f(1, 0), "e": f(1, 1), "cus": f(1, 2), "sqrt_a": f(1, 3),
            "toe": f(2, 0) + (14.0 if is_bds else 0.0),
            "cic": f(2, 1), "omega0": f(2, 2) * an, "cis": f(2, 3),
            "i0": f(3, 0) * an, "crc": f(3, 1),
            "omega": f(3, 2) * an, "omega_dot": f(3, 3) * an,
            "idot": f(4, 0) * an, "week": f(4, 2), "tgd": f(5, 2),
            "toc": sow_from_calendar(y, mo, d, h, mi_, s, dt),
        }
        key = (e["sys"], prn)
        if key not in out or e["toe"] > out[key]["toe"]:
            out[key] = e
        i += 8
    return out


def load_ephemeris():
    """{(sys_id, prn): eph}; BRDC first, self-decoded tracker eph overrides
    (it is the freshest for those PRNs). Returns (eph, leap_s, notes)."""
    eph, notes = {}, []
    leap_s = 18.0
    try:
        with open(BRDC) as fobj:
            text = fobj.read()
        for ln in text.splitlines()[:60]:
            if "LEAP SECONDS" in ln:
                try:
                    leap_s = float(ln.split()[0])
                except (ValueError, IndexError):
                    pass
                break
        brdc = parse_rinex_nav(text)
        eph.update(brdc)
        notes.append(f"brdc:{len(brdc)}")
    except Exception as ex:
        notes.append(f"brdc:none({ex})")
    try:
        with open(TRACKER_EPH) as fobj:
            live = json.load(fobj).get("ephemeris") or []
        n = 0
        for e in live:
            sysid, prn = int(e.get("sys", 0)), int(e.get("prn", 0))
            if sysid not in SYS_NAME or not prn:
                continue
            if sysid == 1 and prn in BDS_GEO_PRNS:
                continue
            if not e.get("sqrt_a"):
                continue
            eph[(sysid, prn)] = {k: float(e.get(k) or 0.0) for k in
                                 ("toe", "toc", "sqrt_a", "e", "m0", "delta_n",
                                  "omega0", "omega", "i0", "idot", "omega_dot",
                                  "cuc", "cus", "crc", "crs", "cic", "cis",
                                  "af0", "af1", "af2", "tgd")} | {"sys": sysid, "prn": prn}
            n += 1
        notes.append(f"live:{n}")
    except Exception as ex:
        notes.append(f"live:none({ex})")
    return eph, leap_s, ",".join(notes)


# --- live tracker state ---------------------------------------------------------

def load_tracked(now):
    """{(sys_id, prn): sat-row} from the tracker state, honouring its ttl
    (mtime rule, same as the server). Empty dict when the tracker is dead."""
    try:
        st = os.stat(TRACKER_STATE)
        with open(TRACKER_STATE) as fobj:
            d = json.load(fobj)
        ttl = float(d.get("ttl_s", 1200))
        if now - st.st_mtime > ttl:
            return {}, True
        out = {}
        for s in (d.get("tracker") or {}).get("sats") or []:
            sysid = SYS_ID.get(s.get("sys"), s.get("sys"))
            if sysid is None:
                continue
            out[(sysid, int(s["prn"]))] = s
        return out, False
    except Exception:
        return {}, True


# --- learned mask -----------------------------------------------------------------

def bin_key(az, el):
    a = int(az // BIN_DEG) * BIN_DEG
    e = int(max(-90.0, min(89.999, el)) // BIN_DEG) * BIN_DEG
    return f"A{a:03d}E{e:+03d}"


def load_mask():
    try:
        with open(MASK_PATH) as fobj:
            d = json.load(fobj)
        if isinstance(d.get("bins"), dict):
            return d
    except Exception:
        pass
    return {"bin_deg": BIN_DEG, "site": [SITE_LAT, SITE_LON, SITE_H],
            "epoch": 0, "bins": {}}


def bin_masked(binc):
    exp = binc.get("exp", 0)
    return exp >= MASK_MIN_SAMPLES and binc.get("lock", 0) / exp < MASK_MAX_FRAC


def save_mask(mask):
    mask["epoch"] = time.time()
    tmp = f"{MASK_PATH}.tmp.{os.getpid()}"
    with open(tmp, "w") as fobj:
        json.dump(mask, fobj)
    os.replace(tmp, MASK_PATH)


def atomic_json(path, obj):
    tmp = f"{path}.tmp.{os.getpid()}"
    with open(tmp, "w") as fobj:
        json.dump(obj, fobj)
    os.replace(tmp, path)


# --- one pass ----------------------------------------------------------------------

def pass_once(now=None):
    now = now if now is not None else time.time()
    site = geodetic_to_ecef(SITE_LAT, SITE_LON, SITE_H)
    eph, leap_s, eph_notes = load_ephemeris()
    t_sow = gps_sow_unix(now, leap_s)
    tracked, tracker_stale = load_tracked(now)
    mask = load_mask()

    sats = []
    counts = {"modeled": 0, "tracked": 0, "absent": 0,
              "unexpected": 0, "below": 0, "unmodeled": 0}
    for key in sorted(eph):
        sysid, prn = key
        e = eph[key]
        if now and abs(_wrap_tk(t_sow - e["toe"])) > 4 * 3600:
            continue                       # ephemeris past its fit window
        try:
            pos = sat_pos_ecef(e, t_sow)
        except (ValueError, OverflowError, ZeroDivisionError):
            continue
        az, el = ecef_to_azel(pos, site, SITE_LAT, SITE_LON)
        row = tracked.get(key)
        observed = bool(row and (row.get("lock_s") or 0) > 0)
        bk = bin_key(az, el)
        binc = mask["bins"].setdefault(bk, {"exp": 0, "lock": 0})
        if el >= 0.0:
            binc["exp"] += 1
            if observed:
                binc["lock"] += 1
        above = el >= EL_MASK_DEG and not bin_masked(binc)
        if observed and above:
            cls, counts["tracked"] = "tracked", counts["tracked"] + 1
        elif observed:
            cls, counts["unexpected"] = "unexpected", counts["unexpected"] + 1
        elif above:
            cls, counts["absent"] = "absent", counts["absent"] + 1
        else:
            cls, counts["below"] = "below", counts["below"] + 1
        counts["modeled"] += 1
        sats.append({
            "sys": SYS_NAME[sysid], "prn": prn,
            "az_deg": round(az, 1), "el_deg": round(el, 1), "cls": cls,
            "cn0": row.get("cn0_proxy") if row else None,
            "lock_s": row.get("lock_s") if row else None,
            "doppler_hz": row.get("doppler_hz") if row else None,
            "rho_m": row.get("rho_m") if row else None,
            "t_tx": row.get("t_tx") if row else None,
            "ppm": row.get("ppm") if row else None,
        })
    modeled_keys = set(eph)
    unmodeled = []
    for key, row in tracked.items():
        if key not in modeled_keys and (row.get("lock_s") or 0) > 0:
            sysname = key[0] if isinstance(key[0], str) else SYS_NAME.get(key[0], "?")
            unmodeled.append(f"{sysname} {key[1]}")
    counts["unmodeled"] = len(unmodeled)
    counts["expected"] = counts["tracked"] + counts["absent"]
    counts["observed"] = counts["tracked"] + counts["unexpected"]

    state = {
        "epoch": now, "ttl_s": TTL_S,
        "sky": {
            "epoch": now,
            "site": [SITE_LAT, SITE_LON, SITE_H],
            "mask": {"el_min_deg": EL_MASK_DEG, "bin_deg": BIN_DEG,
                     "learned_bins": sum(1 for b in mask["bins"].values()
                                         if bin_masked(b))},
            "counts": counts,
            "unmodeled_sats": unmodeled,
            "tracker_stale": tracker_stale,
            "eph": eph_notes,
            "sats": sats,
        },
    }
    atomic_json(STATE, state)
    save_mask(mask)

    hist = {"t": now, "sats": [{k: s[k] for k in
                                ("sys", "prn", "az_deg", "el_deg", "cls",
                                 "cn0", "lock_s", "doppler_hz", "rho_m",
                                 "t_tx", "ppm")} for s in sats]}
    with open(HIST, "a") as fobj:
        fobj.write(json.dumps(hist) + "\n")
    return state


def main():
    args = sys.argv[1:]
    loop = "--loop" in args
    while True:
        t0 = time.time()
        try:
            st = pass_once(t0)
            c = st["sky"]["counts"]
            print(f"[{time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(t0))}] "
                  f"modeled={c['modeled']} tracked={c['tracked']} "
                  f"absent={c['absent']} unexpected={c['unexpected']} "
                  f"unmodeled={c['unmodeled']} eph={st['sky']['eph']}")
        except Exception as ex:
            print(f"[{time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}] "
                  f"pass failed: {ex}", file=sys.stderr)
        if not loop:
            break
        time.sleep(max(5.0, CADENCE_S - (time.time() - t0)))


if __name__ == "__main__":
    main()
