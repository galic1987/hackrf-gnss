#!/usr/bin/env python3
"""Sky-visibility producer: expected vs observed satellites, ~30 s cadence.

For every GPS + BeiDou + Galileo satellite with a current broadcast
ephemeris, computes az/el from the surveyed site (ECEF -> ENU,
IS-GPS-200 Table 20-IV orbit math — a pure-Python port of
src/gps/broadcast.rs, dependency-free), diffs against the tracker's live
state and classifies each satellite:

  tracked     observed (locked) and above the mask      — nominal
  absent      above horizon/mask but not observed        — blockage/fade
  unexpected  observed but BELOW the current mask        — reflection/spoof candidate
  below       not observed and below the mask            — not expected
  predicted   GLONASS: ephemeris-modeled but NEVER receivable here (G1
              1602 MHz FDMA lies outside the 1568.25 MHz L1 tune) — shown
              on the sky map, kept OUT of the tracked/absent/expected
              tracker-coverage accounting and out of the learned mask
  unmodeled   tracked by the tracker but no ephemeris    — sbas today

Galileo E records are Keplerian like GPS (GM and Omega_e from the Galileo
SIS-ICD; GST ~= GPST — the sub-second offset is arc-minute-irrelevant on
the sky). GLONASS R records are PZ-90 ECEF state vectors propagated with
the ICD 4th-order Runge-Kutta model (J2 + broadcast luni-solar point
acceleration; mirrors GloEphemeris in src/glonass_nav.rs). PZ-90 vs WGS84
frame difference is meter-class — irrelevant at sky resolution.

Ephemeris sources (freshest wins per PRN):
  - observations/brdc_latest.rnx   (BKG BRDC: GPS + BDS + GAL + GLO)
  - observations/tracker_eph.json  (self-decoded live: GPS + BeiDou)

Writes (sole writer of each; tmp + os.replace, never read-modify-write of
other producers' state):
  observations/state.sky.json      merged by the panel server at read time
  observations/sky_mask.json       persistent 5°x5° az/el lock histogram —
                                   over days this paints the antenna's true
                                   visibility, including reflection islands.
                                   Schema 2: provenance block (site / rig /
                                   epochs / pass counts) and learning GATED
                                   on tracker health — bins are taught only
                                   from fresh, healthy, non-realigning
                                   tracker state; a stale-schema or
                                   site/rig-mismatched mask is quarantined
                                   aside on load, never silently kept
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

def _load_site(obs):
    """Canonical site anchor: <obs>/site.json. NO hardcoded
    coordinates: a missing anchor is an operator-visible error state, never
    a guessed location. Returns (lat, lon, h_m) or None."""
    try:
        with open(os.path.join(obs, "site.json")) as f:
            s = json.load(f)
        return float(s["lat"]), float(s["lon"]), float(s.get("h_m", 20.0))
    except Exception:
        return None


def bind_obs(obs_dir):
    """(Re)bind every observation path to obs_dir. HACKRF_GNSS_OBS sets this
    at import; tests rebind to a tmp sandbox so they NEVER touch live state
    (state.sky.json / sky_mask.json / sky_history.jsonl writes included)."""
    global OBS, STATE, MASK_PATH, HIST, TRACKER_STATE, TRACKER_EPH, BRDC, SITE
    OBS = obs_dir
    STATE = os.path.join(OBS, "state.sky.json")
    MASK_PATH = os.path.join(OBS, "sky_mask.json")
    HIST = os.path.join(OBS, "sky_history.jsonl")
    TRACKER_STATE = os.path.join(OBS, "state.tracker.json")
    TRACKER_EPH = os.path.join(OBS, "tracker_eph.json")
    BRDC = os.path.join(OBS, "brdc_latest.rnx")
    SITE = _load_site(OBS)  # mast by the window — None when site.json absent


bind_obs(os.environ.get("HACKRF_GNSS_OBS",
                        "/Volumes/Radiator 8TB/gnss/observations"))
CADENCE_S = 30.0
TTL_S = 90.0                       # 3 missed cycles before we tombstone out
EL_MASK_DEG = 5.0                  # static horizon mask until learned
BIN_DEG = 5                        # az/el histogram resolution
MASK_MIN_SAMPLES = 30              # bin needs this many expected samples ...
MASK_MAX_FRAC = 0.05               # ... and <5% lock rate to count as masked

MASK_SCHEMA = 2           # 2 = provenance block + tracker-health-gated
                          # learning (2026-08-26); anything older/foreign is
                          # retired to a .quarantine-* file on load, never
                          # silently kept — the pre-gate mask taught itself
                          # receiver outages/realigns as sky blockage
MASK_MIN_LOCKED = 8       # healthy-track floor: this station holds 9-14 L1
                          # locks (GPS+BDS+GAL) on a normal pass; below it the
                          # receiver is degraded (post-restart acquisition, USB
                          # churn, fade) and absences are receiver artifacts,
                          # not sky truth
MASK_LOCK_AGE_FRAC = 0.8  # most locks must predate the covered window (age >=
                          # CADENCE_S): a realign reseeds every lock age to ~0,
                          # so this rejects fresh all-zero realign eras
MASK_SITE_TOL_DEG = 0.005 # ~550 m: anchor self-tightening must not reset the
                          # mask; a physical antenna move is far larger

# WGS-84 / IS-GPS-200 constants (match src/gps/broadcast.rs)
A_E = 6378137.0
F_E = 1.0 / 298.257223563
E2 = F_E * (2.0 - F_E)
MU_E = 3.986005e14
OMEGA_E = 7.2921151467e-5
WEEK_S = 604800.0

# Galileo SIS-ICD constants (Kepler model like GPS; GST ~= GPST)
MU_GAL = 3.986004418e14
OMEGA_GAL = 7.2921151467e-5

# PZ-90.02 constants for the GLONASS ICD state-vector model
MU_GLO = 398600.44e9          # m^3/s^2
AE_GLO = 6378136.0            # m
C20_GLO = -1082.63e-6         # zonal harmonic (negative value; the J2 TERM
                              # below carries a PLUS sign — see _glo_deriv)
OMEGA_GLO = 7.292115e-5       # rad/s
GLO_STEP_S = 30.0             # RK4 step; ~2 m over a 30-min arc (measured by
                              # propagating real BRDC records to the next
                              # record's epoch: median 2.2 m, p90 3.5 m)
GLO_FIT_S = 7200.0            # max |t - tb| (BRDC records are 30 min apart,
                              # so <= 15 min when the HOURLY file is fresh;
                              # 2 h tolerance rides out file lag at km-class
                              # error — still sub-arcminute on the sky)

SYS_NAME = {0: "gps", 1: "beidou", 2: "galileo", 3: "glonass"}
SYS_ID = {"gps": 0, "beidou": 1, "galileo": 2, "glonass": 3}
BDS_GEO_PRNS = set(range(1, 6)) | set(range(59, 64))  # GEO: ICD MEO math wrong

# Rolling in-memory az/el trails for the panel's sky log map: the last
# TRAIL_WINDOW_S of (az, el, t) per satellite, rebuilt from scratch after a
# restart (30 s cadence -> <= 60 points per trail).
TRAIL_WINDOW_S = 1800.0
TRAILS = {}  # (sysid, prn) -> [[az_deg, el_deg, unix_t], ...]


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


def sat_pos_ecef(e, t, mu=MU_E, om=OMEGA_E):
    """Satellite ECEF (m) at GPST-equivalent time-of-week t (s).

    Galileo passes mu=MU_GAL, om=OMEGA_GAL (GST ~= GPST: the sub-second
    offset is invisible at arc-minute sky resolution)."""
    a = e["sqrt_a"] ** 2
    n0 = math.sqrt(mu / a ** 3)
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
    om_ = e["omega0"] + (e["omega_dot"] - om) * tk - om * e["toe"]
    co, so, ci, si = math.cos(om_), math.sin(om_), math.cos(ik), math.sin(ik)
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


def ecef_to_enu_vec(vec, lat_deg, lon_deg):
    """Rotate an ECEF vector into (east, north, up) at geodetic lat/lon."""
    la, lo = math.radians(lat_deg), math.radians(lon_deg)
    dx, dy, dz = vec
    e = -math.sin(lo) * dx + math.cos(lo) * dy
    n = (-math.sin(la) * math.cos(lo) * dx - math.sin(la) * math.sin(lo) * dy
         + math.cos(la) * dz)
    u = (math.cos(la) * math.cos(lo) * dx + math.cos(la) * math.sin(lo) * dy
         + math.sin(la) * dz)
    return e, n, u


def sat_motion(e, t, lat_deg, lon_deg, mu=MU_E, om=OMEGA_E):
    """(alt_km, speed_mps, track_deg) for the panel's satellite table.

    alt is height above the spherical Earth radius A_E (educational, not
    geodetic); speed is the ECEF-frame magnitude (central difference over
    1 s); track is the heading of the velocity projected into the site's
    local horizon plane — the direction the sat is moving across our sky,
    degrees clockwise from north.
    """
    pos = sat_pos_ecef(e, t, mu, om)
    p0 = sat_pos_ecef(e, t - 0.5, mu, om)
    p1 = sat_pos_ecef(e, t + 0.5, mu, om)
    vel = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    alt_km = (math.sqrt(pos[0] ** 2 + pos[1] ** 2 + pos[2] ** 2) - A_E) / 1e3
    speed = math.sqrt(sum(v * v for v in vel))
    ev, nv, _ = ecef_to_enu_vec(vel, lat_deg, lon_deg)
    track = math.degrees(math.atan2(ev, nv)) % 360.0
    return alt_km, speed, track


# --- GLONASS state-vector propagation (ICD 5.1 / glonass_nav.rs fields) ------
#
# Broadcast R records carry the PZ-90 ECEF state (pos km, vel km/s — ECEF
# frame, so the Coriolis/centrifugal terms below are mandatory) plus the
# luni-solar point acceleration, valid at tb. Integrate with 4th-order
# Runge-Kutta; PZ-90 vs WGS84 differs by meters, invisible on the sky.

def _glo_deriv(st, acc):
    x, y, z, vx, vy, vz = st
    r2 = x * x + y * y + z * z
    r = math.sqrt(r2)
    zz = z * z / r2
    j2 = 1.5 * C20_GLO * MU_GLO * AE_GLO * AE_GLO / (r2 * r2 * r)
    # J2 term sign: C20 is NEGATIVE and the term enters with a PLUS here —
    # at the equator (zz=0) it then points inward (-|1.5*C20|... * x), the
    # standard J2 acceleration. Empirically decisive: propagating 202 real
    # BRDC R-records to the next record's epoch gives median 2.2 m with this
    # sign and 201 m with the opposite one (round-9b review ablation,
    # reproduced 2026-08-26). The Coriolis/centrifugal terms are likewise
    # mandatory (dropping them: ~600 km over the same arc).
    ax = -MU_GLO * x / (r2 * r) + j2 * x * (1.0 - 5.0 * zz) \
        + OMEGA_GLO ** 2 * x + 2.0 * OMEGA_GLO * vy + acc[0]
    ay = -MU_GLO * y / (r2 * r) + j2 * y * (1.0 - 5.0 * zz) \
        + OMEGA_GLO ** 2 * y - 2.0 * OMEGA_GLO * vx + acc[1]
    az = -MU_GLO * z / (r2 * r) + j2 * z * (3.0 - 5.0 * zz) + acc[2]
    return (vx, vy, vz, ax, ay, az)


def glo_state(e, t):
    """(pos_m, vel_ms) PZ-90 ECEF state at GPST-equivalent sow t, RK4 from
    tb. Raises ValueError outside the fit window (BRDC R records are 30 min
    apart, so |t-tb| <= 15 min in normal operation)."""
    dt = _wrap_tk(t - e["tb_sow"])
    if abs(dt) > GLO_FIT_S:
        raise ValueError(f"glo eph stale: |dt|={dt:.0f}s")
    st = list(e["pos"]) + list(e["vel"])
    acc = e["acc"]
    n = max(1, int(abs(dt) / GLO_STEP_S + 0.5))
    h = dt / n
    for _ in range(n):
        k1 = _glo_deriv(st, acc)
        s2 = [st[i] + 0.5 * h * k1[i] for i in range(6)]
        k2 = _glo_deriv(s2, acc)
        s3 = [st[i] + 0.5 * h * k2[i] for i in range(6)]
        k3 = _glo_deriv(s3, acc)
        s4 = [st[i] + h * k3[i] for i in range(6)]
        k4 = _glo_deriv(s4, acc)
        st = [st[i] + h * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0
              for i in range(6)]
    return tuple(st[:3]), tuple(st[3:])


def glo_motion(e, t, lat_deg, lon_deg):
    """(pos, alt_km, speed_mps, track_deg) — same panel fields as
    sat_motion, velocity taken straight from the integrated state."""
    pos, vel = glo_state(e, t)
    alt_km = (math.sqrt(sum(v * v for v in pos)) - A_E) / 1e3
    speed = math.sqrt(sum(v * v for v in vel))
    ev, nv, _ = ecef_to_enu_vec(vel, lat_deg, lon_deg)
    track = math.degrees(math.atan2(ev, nv)) % 360.0
    return pos, alt_km, speed, track


# --- ephemeris sources ---------------------------------------------------------

def _df(s):
    """Strict RINEX-3 D/E-exponent float field: None on blank OR malformed
    (round-11 review: the old 0.0 zero-fill fabricated plausible
    ephemerides out of corrupt lines). Non-finite spellings parse in Python
    but are not RINEX content. Mirrors df_strict in src/gps/broadcast.rs."""
    t = s.strip()
    if not t:
        return None
    try:
        v = float(t.replace("D", "E").replace("d", "E"))
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def _fld(line, a, b):
    return line[a:min(b, len(line))] if a < len(line) else ""


# Inclination evidence bands for the radians-vs-semicircles unit decision
# (mirror src/gps/broadcast.rs). Live MEO GNSS inclinations span 53-59 deg
# and drifting BDS IGSOs exceed 60 deg (live BRDC 2026-08-26: C09 at i0 =
# 1.0523 rad = 60.3 deg): 0.93-1.06 rad or 0.294-0.336 semicircles. A
# record's raw |i0| is rad-like in [0.85, 1.10], sc-like in [0.25, 0.36],
# neutral below 0.25 (BDS GEOs ~0.02-0.12 carry no unit evidence), and
# physically impossible under either unit anywhere else (no vote; the
# per-record sanity check rejects it).
I0_RAD_LIKE = (0.85, 1.10)
I0_SC_LIKE = (0.25, 0.36)
I0_GREY = (0.36, 0.85)   # impossible under either unit: reject outright


def _detect_ang(lines, hdr, want):
    """Radians-vs-semicircles verdict for one constellation:
    "radians" | "semicircles" | "ambiguous". Per-record i0 votes over the
    WHOLE constellation (no early-exit cap: BKG files sort records by PRN
    and a capped early scan of a BDS constellation can see nothing but
    unit-neutral GEOs). Both vote kinds present -> "ambiguous": the file is
    internally inconsistent — fail closed, every record of the constellation
    is rejected rather than guessed. No decided votes -> semicircles (spec
    default; a GEO-only BDS constellation lands here). One corrupt record
    earns no vote and cannot flip the file."""
    rad = sc = 0
    j = hdr
    while j + 4 < len(lines):
        ln = lines[j]
        if ln.startswith(want) and len(ln) > 4:
            v = _df(_fld(lines[j + 4], 4, 23))
            if v is not None:
                m = abs(v)
                if I0_RAD_LIKE[0] <= m <= I0_RAD_LIKE[1]:
                    rad += 1
                elif I0_SC_LIKE[0] <= m <= I0_SC_LIKE[1]:
                    sc += 1
            j += 8
            continue
        j += 1
    if rad and sc:
        return "ambiguous"
    return "radians" if rad else "semicircles"


def _i0_sane(i0_raw, unit):
    """Per-record i0 sanity against the chosen unit (mirror of i0_sane in
    src/gps/broadcast.rs): [0, 1] semicircles / [0, pi] radians, never
    inside the grey band. The sanity range deliberately does NOT gate
    inclination — drifting BDS IGSOs legitimately exceed 60 deg (live
    BRDC 2026-08-26 C09); garbage protection is the grey band plus the
    vote detection."""
    if i0_raw < 0.0 or I0_GREY[0] < i0_raw < I0_GREY[1]:
        return False
    return i0_raw <= (1.0 if unit == "semicircles" else math.pi)


def parse_rinex_nav(text, now_sow=None, leap_s=18.0, stats=None):
    """GPS (G) + BeiDou (C) + Galileo (E) + GLONASS (R) records of a RINEX-3
    MIXED nav file -> {(sys,prn): eph}.
    Port of parse_rinex_gps_nav/parse_rinex_bds_nav (src/gps/broadcast.rs,
    src/beidou_d1.rs), extended: Galileo is the same 8-line Kepler record
    (GST ~= GPST, so one timescale serves all Kepler constellations);
    GLONASS is a 4-line PZ-90 state-vector record (fields mirror
    GloEphemeris in src/glonass_nav.rs).

    STRICT (round-11 review): any blank/malformed CONSUMED field rejects
    the record (never a silent 0.0); the angle unit is decided per
    constellation by per-record i0 votes (_detect_ang) — contradictory
    content fails closed ("ambiguous", every record of the constellation
    rejected); each record's raw i0 must then pass the chosen unit's
    sanity band (_i0_sane). When `stats` is a dict it receives the ledger:
    stats["rejected"] (count) and stats["units"] (per-constellation
    verdict).

    Newest VALID issue per PRN wins: (week, toe) tuple compare — RINEX
    weeks are continuous (3.05 §4.1.1/§4.1.4), rollover-exact across the
    week boundary (the old bare `toe > toe` kept last week's record over a
    fresh one). For GLONASS the record with tb nearest now_sow wins
    (latest tb when now_sow is None). BDS epochs/toe/toc are BDT, shifted
    BDT->GPST (+14 s, wrapped); G-record epochs are ALREADY GPST (RINEX-3
    time-system code G — no leap term; the pre-round-11 port added +18 s,
    putting toc 18 s past toe on every record); R epochs are UTC(SU),
    shifted +leap to GPST and tb rounded to its 15-min grid (the +3 h
    Moscow labeling of tb in the ICD is a multiple of 15 min, so the
    UTC(SU) rounding lands on the same instant). BDS GEOs and unhealthy
    (Bn != 0) GLONASS records skipped."""
    lines = text.splitlines()
    hdr = 0
    while hdr < len(lines) and "END OF HEADER" not in lines[hdr]:
        hdr += 1
    hdr += 1
    ang = {want: _detect_ang(lines, hdr, want) for want in ("G", "C", "E")}
    if stats is not None:
        stats["units"] = dict(ang)
        stats["rejected"] = 0

    def rejected():
        if stats is not None:
            stats["rejected"] += 1

    out = {}
    i = hdr
    while i < len(lines):
        ln = lines[i]
        if not ln or ln[:1] not in ("G", "C", "E", "R") or len(ln) < 23:
            i += 1
            continue
        if ln[0] == "R":
            # GLONASS: 4-line state-vector record (km, km/s, km/s^2 -> SI)
            if i + 3 >= len(lines):
                break
            try:
                prn = int(_fld(ln, 1, 3))
                y, mo, d = (int(_fld(ln, 4, 8)), int(_fld(ln, 9, 11)),
                            int(_fld(ln, 12, 14)))
                h, mi_, s = (int(_fld(ln, 15, 17)), int(_fld(ln, 18, 20)),
                             int(_fld(ln, 21, 23)))
            except ValueError:
                rejected()
                i += 1
                continue
            b = lines[i + 1:i + 4]
            f = lambda l, k: _df(_fld(b[l], 4 + k * 19, 4 + (k + 1) * 19))
            vals = {(l, k): f(l, k) for l in range(3) for k in range(3)}
            vals[(0, 3)] = f(0, 3)                 # Bn health flag
            tau_n = _df(_fld(ln, 23, 42))          # line carries -tau_n
            gamma_n = _df(_fld(ln, 42, 61))
            if any(v is None for v in vals.values()) \
                    or tau_n is None or gamma_n is None:
                rejected()
                i += 4
                continue
            if vals[(0, 3)] != 0.0:                # Bn: 0 = healthy
                i += 4
                continue
            days = jdn(y, mo, d) - jdn(1980, 1, 6)
            sod = h * 3600 + mi_ * 60 + s
            tb_sod = round(sod / 900.0) * 900.0   # tb on its 15-min grid
            e = {
                "sys": 3, "prn": prn,
                "pos": tuple(vals[(l, 0)] * 1e3 for l in range(3)),
                "vel": tuple(vals[(l, 1)] * 1e3 for l in range(3)),
                "acc": tuple(vals[(l, 2)] * 1e3 for l in range(3)),
                "tau_n_s": -tau_n,
                "gamma_n": gamma_n,
                "tb_sow": (days * 86400 + tb_sod + leap_s) % WEEK_S,
            }
            key = (3, prn)
            cur = out.get(key)
            if cur is None:
                out[key] = e
            elif now_sow is not None:
                if abs(_wrap_tk(e["tb_sow"] - now_sow)) < \
                        abs(_wrap_tk(cur["tb_sow"] - now_sow)):
                    out[key] = e
            elif e["tb_sow"] > cur["tb_sow"]:
                out[key] = e
            i += 4
            continue
        if i + 7 >= len(lines):
            break
        is_bds = ln[0] == "C"
        unit = ang[ln[0]]
        if unit == "ambiguous":
            # fail closed: contradictory unit content — never guessed
            rejected()
            i += 8
            continue
        try:
            prn = int(_fld(ln, 1, 3))
        except ValueError:
            rejected()
            i += 1
            continue
        if is_bds and prn in BDS_GEO_PRNS:
            i += 8
            continue
        try:
            y, mo, d = (int(_fld(ln, 4, 8)), int(_fld(ln, 9, 11)), int(_fld(ln, 12, 14)))
            h, mi_, s = (int(_fld(ln, 15, 17)), int(_fld(ln, 18, 20)), int(_fld(ln, 21, 23)))
        except ValueError:
            rejected()
            i += 1
            continue
        b = lines[i + 1:i + 8]
        f = lambda l, k: _df(_fld(b[l], 4 + k * 19, 4 + (k + 1) * 19))
        raw = {
            "af0": _df(_fld(ln, 23, 42)), "af1": _df(_fld(ln, 42, 61)),
            "af2": _df(_fld(ln, 61, 80)),
            "crs": f(0, 1), "delta_n": f(0, 2), "m0": f(0, 3),
            "cuc": f(1, 0), "e": f(1, 1), "cus": f(1, 2), "sqrt_a": f(1, 3),
            "toe": f(2, 0), "cic": f(2, 1), "omega0": f(2, 2), "cis": f(2, 3),
            "i0": f(3, 0), "crc": f(3, 1),
            "omega": f(3, 2), "omega_dot": f(3, 3),
            "idot": f(4, 0), "week": f(4, 2), "tgd": f(5, 2),
        }
        if any(v is None for v in raw.values()) or not _i0_sane(raw["i0"], unit):
            rejected()
            i += 8
            continue
        an = 1.0 if unit == "radians" else math.pi
        # BDT+14=GPST (wrapped) ; GST~=GPST and G-epochs ARE GPST already
        dt = 14.0 if is_bds else 0.0
        toe = raw["toe"] + 14.0 if is_bds else raw["toe"]
        e = {
            "sys": 1 if is_bds else 0 if ln[0] == "G" else 2, "prn": prn,
            "af0": raw["af0"], "af1": raw["af1"], "af2": raw["af2"],
            "crs": raw["crs"], "delta_n": raw["delta_n"] * an,
            "m0": raw["m0"] * an,
            "cuc": raw["cuc"], "e": raw["e"], "cus": raw["cus"],
            "sqrt_a": raw["sqrt_a"],
            "toe": toe % WEEK_S,
            "cic": raw["cic"], "omega0": raw["omega0"] * an,
            "cis": raw["cis"], "i0": raw["i0"] * an, "crc": raw["crc"],
            "omega": raw["omega"] * an, "omega_dot": raw["omega_dot"] * an,
            "idot": raw["idot"] * an, "week": raw["week"], "tgd": raw["tgd"],
            "toc": sow_from_calendar(y, mo, d, h, mi_, s, dt),
        }
        key = (e["sys"], prn)
        cur = out.get(key)
        if cur is None or (e["week"], e["toe"]) > (cur["week"], cur["toe"]):
            out[key] = e
        i += 8
    return out


def load_ephemeris(now=None):
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
        now_sow = gps_sow_unix(now, leap_s) if now else None
        st = {}
        brdc = parse_rinex_nav(text, now_sow=now_sow, leap_s=leap_s, stats=st)
        eph.update(brdc)
        per = {}
        for sid, _ in brdc:
            per[SYS_NAME[sid][:3]] = per.get(SYS_NAME[sid][:3], 0) + 1
        notes.append("brdc:" + ",".join(f"{k}={v}" for k, v in sorted(per.items())))
        # round-11: the parser's rejection ledger is operator-visible —
        # a malformed-record storm must show up in the state file's notes
        if st.get("rejected"):
            amb = ",".join(k for k, v in (st.get("units") or {}).items()
                           if v == "ambiguous")
            notes.append(f"brdc-rej:{st['rejected']}"
                         + (f"(fail-closed:{amb})" if amb else ""))
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


def rig_identity():
    """Antenna/radio config string for mask provenance: the tracker's GPS L1
    source name carries rig + antenna + tune (e.g. 'C/A live track ·
    Pro+AA.250, 16 Msps @ 1568.25 · Presence'). None when unpublished — a
    mask created then adopts the rig once it appears (see load_mask)."""
    try:
        with open(TRACKER_STATE) as fobj:
            d = json.load(fobj)
        for s in d.get("sources") or []:
            if "GPS L1" in (s.get("band") or ""):
                return s.get("name")
    except Exception:
        pass
    return None


def tracker_health(tracked, tracker_stale):
    """May this pass TEACH the mask? -> (ok, "learning" | "gated:<reason>").

    A dead or realigning receiver reports absences that are receiver
    artifacts, not sky truth, so gated passes still classify (the panel
    wants the picture) but never increment the bins. Gates: tracker state
    fresh (its ttl honoured), >= MASK_MIN_LOCKED channels locked, and
    >= MASK_LOCK_AGE_FRAC of those locks older than the covered window
    (a realign reseeds every lock age to ~0; per-sat slip flags are
    momentary loop events and deliberately NOT a veto — only the wholesale
    age reset marks a realign era)."""
    if tracker_stale:
        return False, "gated:tracker-stale"
    locked = [r for r in tracked.values() if (r.get("lock_s") or 0) > 0]
    if len(locked) < MASK_MIN_LOCKED:
        return False, "gated:few-locks"
    mature = sum(1 for r in locked if r["lock_s"] >= CADENCE_S)
    if mature < MASK_LOCK_AGE_FRAC * len(locked):
        return False, "gated:locks-young"
    return True, "learning"


# --- learned mask -----------------------------------------------------------------

def bin_key(az, el):
    a = int(az // BIN_DEG) * BIN_DEG
    e = int(max(-90.0, min(89.999, el)) // BIN_DEG) * BIN_DEG
    return f"A{a:03d}E{e:+03d}"


def new_mask(site_ll, rig, now=None):
    """Empty schema-2 mask with its provenance block. Per-bin sample counts
    live in bins (exp = expected passes, lock = observed passes)."""
    now = time.time() if now is None else now
    return {"schema": MASK_SCHEMA, "bin_deg": BIN_DEG, "epoch": now,
            "provenance": {
                "site": {"lat": site_ll[0], "lon": site_ll[1], "h_m": site_ll[2]}
                        if site_ll else None,
                "rig": rig,
                "created_epoch": now,
                "learn_start_epoch": None,   # first gated-IN pass
                "passes_learned": 0,
                "passes_gated": 0,
            },
            "bins": {}}


def _quarantine_mask(reason):
    """Move an unusable mask aside — stale knowledge is retired, never
    silently kept, but also never destroyed (auditability)."""
    try:
        ts = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        dst = f"{MASK_PATH}.quarantine-{ts}-{os.getpid()}-{time.time_ns() % 10**9}"
        os.replace(MASK_PATH, dst)
        print(f"sky_mask retired ({reason}) -> {os.path.basename(dst)}",
              file=sys.stderr)
    except OSError:
        pass


def load_mask(site_ll, rig):
    """Load the learned mask, enforcing schema + provenance. A mask whose
    schema predates the gated-learning change, or whose site/rig identity
    mismatches the current station, is quarantined and learning restarts
    from an empty mask."""
    try:
        with open(MASK_PATH) as fobj:
            d = json.load(fobj)
    except FileNotFoundError:
        return new_mask(site_ll, rig)
    except Exception:
        _quarantine_mask("unreadable")
        return new_mask(site_ll, rig)
    if d.get("schema") != MASK_SCHEMA or not isinstance(d.get("bins"), dict):
        _quarantine_mask(f"schema {d.get('schema')!r} != {MASK_SCHEMA}")
        return new_mask(site_ll, rig)
    prov = d.setdefault("provenance", {})
    psite = prov.get("site")
    if site_ll and psite:
        if abs(psite.get("lat", 1e9) - site_ll[0]) > MASK_SITE_TOL_DEG or \
                abs(psite.get("lon", 1e9) - site_ll[1]) > MASK_SITE_TOL_DEG:
            _quarantine_mask(f"site moved: mask {psite} != current {site_ll}")
            return new_mask(site_ll, rig)
    prig = prov.get("rig")
    if rig and prig and rig != prig:
        _quarantine_mask(f"rig changed: {prig!r} != {rig!r}")
        return new_mask(site_ll, rig)
    # adopt identity fields once known (mask may have been born while the
    # tracker was down / site anchor briefly absent)
    if rig and not prig:
        prov["rig"] = rig
    if site_ll and not psite:
        prov["site"] = {"lat": site_ll[0], "lon": site_ll[1], "h_m": site_ll[2]}
    return d


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
    # re-read the anchor every pass: operator edits propagate live, and a
    # missing anchor is an honest error heartbeat, never a guessed location
    site_ll = _load_site(OBS)
    if site_ll is None:
        atomic_json(STATE, {"epoch": now, "ttl_s": TTL_S, "sky": {
            "epoch": now, "site": None,
            "error": "no site anchor: observations/site.json missing/invalid",
            "sats": [],
            "counts": {k: 0 for k in ("modeled", "tracked", "absent",
                                      "unexpected", "below", "unmodeled",
                                      "predicted", "expected", "observed")}}})
        return None
    site_lat, site_lon, site_h = site_ll
    site = geodetic_to_ecef(site_lat, site_lon, site_h)
    eph, leap_s, eph_notes = load_ephemeris(now)
    t_sow = gps_sow_unix(now, leap_s)
    tracked, tracker_stale = load_tracked(now)
    rig = rig_identity()
    mask = load_mask(site_ll, rig)
    learn_ok, learn_why = tracker_health(tracked, tracker_stale)

    sats = []
    counts = {"modeled": 0, "tracked": 0, "absent": 0,
              "unexpected": 0, "below": 0, "unmodeled": 0, "predicted": 0}
    for key in sorted(eph):
        sysid, prn = key
        e = eph[key]
        try:
            if sysid == 3:
                # PZ-90 state vector, RK4 to t (raises outside the fit window)
                pos, alt_km, speed_mps, track_deg = glo_motion(
                    e, t_sow, site_lat, site_lon)
            else:
                if now and abs(_wrap_tk(t_sow - e["toe"])) > 4 * 3600:
                    continue                   # ephemeris past its fit window
                mu, om = (MU_GAL, OMEGA_GAL) if sysid == 2 else (MU_E, OMEGA_E)
                pos = sat_pos_ecef(e, t_sow, mu, om)
                alt_km, speed_mps, track_deg = sat_motion(
                    e, t_sow, site_lat, site_lon, mu, om)
        except (ValueError, OverflowError, ZeroDivisionError):
            continue
        az, el = ecef_to_azel(pos, site, site_lat, site_lon)
        if el > -10.0:  # trail only the near-sky region (below-horizon is noise)
            tr = TRAILS.setdefault(key, [])
            tr.append([round(az, 1), round(el, 1), round(now)])
            cutoff = now - TRAIL_WINDOW_S
            while tr and tr[0][2] < cutoff:
                tr.pop(0)
        if sysid == 3:
            # GLONASS is never observable by this station's L1 tracker (G1
            # 1602 MHz FDMA lies outside the 1568.25 MHz tune), so it must
            # not judge tracker coverage: no mask learning, no tracked/
            # absent/expected accounting — predicted-only, honestly labeled.
            cls, counts["predicted"] = "predicted", counts["predicted"] + 1
            counts["modeled"] += 1
            sats.append({
                "sys": "glonass", "prn": prn,
                "az_deg": round(az, 1), "el_deg": round(el, 1), "cls": cls,
                "alt_km": round(alt_km), "speed_mps": round(speed_mps),
                "track_deg": round(track_deg, 1),
                "cn0": None, "lock_s": None, "doppler_hz": None,
                "rho_m": None, "t_tx": None, "ppm": None,
            })
            continue
        row = tracked.get(key)
        observed = bool(row and (row.get("lock_s") or 0) > 0)
        bk = bin_key(az, el)
        binc = mask["bins"].get(bk)
        # Mask learning is GATED on tracker health: a stale/dead/realigning
        # receiver must not teach absences — those are receiver artifacts,
        # not sky truth. Gated passes classify but record nothing, which is
        # the "receiver unavailable" vs "expected but not acquired"
        # distinction. Bins are only created by a taught sample (no more
        # all-zero keys for below-horizon sats).
        if el >= 0.0 and learn_ok:
            if binc is None:
                binc = mask["bins"][bk] = {"exp": 0, "lock": 0}
            binc["exp"] += 1
            if observed:
                binc["lock"] += 1
        above = el >= EL_MASK_DEG and not bin_masked(binc or {})
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
            "alt_km": round(alt_km), "speed_mps": round(speed_mps),
            "track_deg": round(track_deg, 1),
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

    # Prune departed sats' trails fully (in-loop pruning only covers sats
    # seen this pass); survivors with >= 2 points feed the sky log map.
    cutoff = now - TRAIL_WINDOW_S
    for k in list(TRAILS):
        TRAILS[k] = [pt for pt in TRAILS[k] if pt[2] >= cutoff]
        if not TRAILS[k]:
            del TRAILS[k]
    recent_trails = [{"sys": SYS_NAME[k[0]], "prn": k[1], "trail": v}
                     for k, v in sorted(TRAILS.items()) if len(v) >= 2]

    prov = mask.setdefault("provenance", {})
    if learn_ok:
        prov["passes_learned"] = int(prov.get("passes_learned", 0)) + 1
        if not prov.get("learn_start_epoch"):
            prov["learn_start_epoch"] = now
    else:
        prov["passes_gated"] = int(prov.get("passes_gated", 0)) + 1

    state = {
        "epoch": now, "ttl_s": TTL_S,
        "sky": {
            "epoch": now,
            "site": [site_lat, site_lon, site_h],
            "mask": {"el_min_deg": EL_MASK_DEG, "bin_deg": BIN_DEG,
                     "schema": MASK_SCHEMA,
                     "learning": learn_why,
                     "learned_bins": sum(1 for b in mask["bins"].values()
                                         if bin_masked(b))},
            "counts": counts,
            "unmodeled_sats": unmodeled,
            "tracker_stale": tracker_stale,
            "eph": eph_notes,
            "recent_trails": recent_trails,
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


# Nominal altitude bands (km) for the startup self-check — a parse or
# unit-conversion bug shows up here before the panel ever sees the file.
# Galileo's band is wide on purpose: E14/E18 are the eccentric FOC pair
# (e ~ 0.16, altitude swings ~17,500-26,000 km).
ALT_BAND_KM = {"gps": (19500, 21000), "beidou": (20000, 38000),
               "galileo": (17000, 27000), "glonass": (18500, 19800)}


def selfcheck(sats):
    """One startup log line: per-sys modeled counts + altitude sanity."""
    per = {}
    bad = []
    for s in sats:
        lo, hi = ALT_BAND_KM.get(s["sys"], (0, 1e9))
        n, mn, mx = per.get(s["sys"], (0, 1e9, -1e9))
        per[s["sys"]] = (n + 1, min(mn, s["alt_km"]), max(mx, s["alt_km"]))
        if not lo <= s["alt_km"] <= hi:
            bad.append(f"{s['sys']}{s['prn']}:{s['alt_km']}km")
    parts = [f"{k}={v[0]}(alt {v[1]:.0f}-{v[2]:.0f}km)"
             for k, v in sorted(per.items())]
    print(f"selfcheck: {' '.join(parts)}"
          + (f"  WARN alt out of band: {','.join(bad)}" if bad else " (alt ok)"))


def main():
    args = sys.argv[1:]
    loop = "--loop" in args
    checked = False
    while True:
        t0 = time.time()
        try:
            st = pass_once(t0)
            c = st["sky"]["counts"]
            print(f"[{time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(t0))}] "
                  f"modeled={c['modeled']} tracked={c['tracked']} "
                  f"absent={c['absent']} unexpected={c['unexpected']} "
                  f"predicted={c['predicted']} unmodeled={c['unmodeled']} "
                  f"mask={st['sky']['mask']['learning']} "
                  f"eph={st['sky']['eph']}")
            if not checked:
                checked = True
                selfcheck(st["sky"]["sats"])
        except Exception as ex:
            print(f"[{time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}] "
                  f"pass failed: {ex}", file=sys.stderr)
        if not loop:
            break
        time.sleep(max(5.0, CADENCE_S - (time.time() - t0)))


if __name__ == "__main__":
    main()
