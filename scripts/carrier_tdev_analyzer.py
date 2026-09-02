#!/usr/bin/env python3
"""Carrier-phase TDEV analyzer (EXPLORATORY — no carrier gate is registered).

The re-registered Leg 1 claim gate is carrier-phase TDEV stability. This
tool computes carrier-phase TDEV; it does NOT judge the gate, because no
carrier-phase gate has been pre-registered. Exit 0 is therefore
UNREACHABLE by construction (CARRIER_GATE_PREREGISTERED is a False
constant, same mechanism as clock_bias_analyzer.POISON_GATE_CLAIM_GRADE):
the best possible outcome is exit 2 (exploratory numbers, honestly
labeled). Exit 1 means failed / insufficient / integrity violation.

DATA SOURCE (surveyed 2026-09-02):
  archive/YYYY-MM-DD/satellite.parquet  — NO carrier columns (epoch, sys,
      prn, cn0, doppler_hz, lock_s, rho_m, t_tx, ppm, az/el, residual, cls).
  archive/YYYY-MM-DD/phase.parquet and phase_history.jsonl — ATSC ch35
      carrier phase, not GNSS.
  state.tracker.json — the tracker's live 1 Hz per-sat publication
      (carrier_cycles, phase_frac, slip), but a 30 s-TTL snapshot: no
      history.
  telemetry_log.jsonl — the ONLY durable archive of the carrier
      observable: the 10 s collector merges the tracker snapshot into each
      row's "sats" list verbatim (carrier_cycles, slip, lock_s, epoch, and
      per-row sbas_geonav for SBAS sats). THIS is what we read.
  Consequence: the archived stream is the tracker's 1 s grid decimated by
      ~10 with +/-1 sample jitter (observed dt histogram: 9/10/11 s), so
      tau = 1 s TDEV is NOT computable from the archive (reported as
      dropped, reason "cadence"); a 1 Hz carrier archive is future work.

OBSERVABLE AND SIGN CONVENTION:
  carrier_cycles is the integrated replica carrier phase in cycles,
  zeroed at channel (re)seed, rate == Doppler (tracker_producer.py:22-29),
  carrying the Costas 180-degree (half-cycle) ambiguity as a constant
  offset per segment (constant offsets vanish in detrend + TDEV).
  Time error:  x(t) = -carrier_cycles / f_carrier.
  Justification: every locally generated frequency — the downconversion
  LO and the replica NCO time base — derives from the station clock. If
  the station clock runs FAST (time error x increasing), the LO is high,
  the incoming carrier appears LOW, and the tracker integrates
  carrier_cycles DOWN; hence x = -cycles/f. Geometry (LOS Doppler) and
  satellite-clock terms ride on top: the segment-mean rate is removed by
  linear detrend, and SBAS GEO LOS motion + clock drift is removed by the
  MT9 correction below. TDEV itself is sign-invariant; the convention
  fixes only the interpretation of common-mode drift direction.
  f_carrier per system: L1/E1/SBAS 1575.42 MHz, BeiDou B1I 1561.098 MHz.

METHOD (mirrors clock_bias_analyzer.py v2 idioms):
  1. Snapshot read: capture st_size FIRST, read exactly that prefix (an
     append-only log's fixed-size prefix is an immutable snapshot; the
     file having SHRUNK during the read is an integrity failure). A tail
     window (--max-bytes, default 200 MB) drops its first partial line.
  2. Validation is fail-closed and counted: sat rows without a
     carrier_cycles key are the pre-dcfcfaa legacy schema (counted,
     excluded); a row that HAS carrier_cycles but malformed/missing
     epoch/slip/lock_s/sys/prn is a global integrity failure (exit 1).
     Unknown systems are excluded and counted, never silently dropped.
  3. Quality: samples with lock_s <= 0 (channel unlocked / just seeded)
     are excluded and counted — segment discipline below is the rest of
     the quality gate (junk channels re-seed constantly and never form a
     segment >= MIN_SEG_ROWS).
  4. Segmentation NEVER bridges a phase break: split before any sample
     with slip == true; split on re-seed (lock_s decrease, or
     |d(carrier_cycles)/dt| > RESEED_JUMP_HZ — far beyond any physical
     LOS+clock rate, a zeroing shows as ~1e5-1e6 Hz apparent); then
     clock_bias_analyzer.split_segments splits each chunk at nonpositive
     intervals, gaps > 1.5x median dt, and cadence error > 15%. The 15%
     (vs clock_bias's 2%) exists because the collector's decimation
     aliases the 1 s tracker grid to 9/10/11 s: +/-1 sample on a 10 s
     spacing is a <=10% tau bookkeeping error, folded into the honest
     label "tau +/-10%" on every result; materially different cadences
     still split. Exact-duplicate epochs (collector double-read of one
     tracker second) are deduplicated and counted; same-epoch rows with
     DIFFERENT carrier values are a conflict — the satellite is excluded
     with a printed reason.
  5. Per-segment linear detrend (clock_bias_analyzer.detrend): removes
     the segment-mean rate — receiver clock frequency offset AND per-sat
     mean LOS Doppler — and, exactly as in clock_bias_analyzer, CONCEALS
     instability below ~1/span. It does NOT remove LOS Doppler curvature:
     MEO L1 Doppler rates reach ~1 Hz/s, so per-satellite TDEV at
     tau >~ 100 s is GEOMETRY-DOMINATED — an upper bound on clock
     stability, never a measurement of it. The cross-satellite view is
     where clock (common mode) separates from geometry (per-sat spread).
  6. TDEV: clock_bias_analyzer.tdev — uniform-grid NIST SP 1065
     ModAllanVar-based TDEV (white PM: E[TDEV] = sigma/sqrt(m), slope
     -1/2; pinned by the synthetic tests). Requested taus {1,10,100,1000}
     s; a tau with round(tau/dt) < 1 is dropped as "cadence" (never
     silently remapped to a coarser effective tau), taus with span
     < 3*tau are dropped as "span".
  7. GEO handling (sys sbas / PRN >= GEO_PRN_MIN): the archive carries
     the tracker's MT9 sbas_geonav on every SBAS sample. When every
     sample of a segment has a usable geonav and site.json resolves, the
     LOS motion + GEO clock drift is REMOVED before TDEV: per-sample
     Doppler from geocorrector_helper.calc_geo_doppler_hz (the audited
     DO-229 A.4.5.1 2nd-order Taylor reference, GPS time-of-day fold
     included), integrated by trapezoid and subtracted from
     carrier_cycles. The helper fails OPEN (returns 0.0 on bad input), so
     geonav is validated HERE first and the predicted Doppler is bounded
     (|f| <= GEO_MAX_ABS_DOPPLER_HZ; WAAS GEO LOS range rates are
     0.5-3 m/s). GEO segments are additionally split at geonav
     availability transitions so each segment is geonav-homogeneous: the
     rare no-geonav samples (tracker startup before the first MT9
     decode; ~0.5% of real SBAS rows) fragment off instead of poisoning
     an hours-long correctable segment. A segment that still cannot be
     corrected is MOTION-CONTAMINATED (excluded from the cross-sat view,
     labeled in the per-sat report) — fail closed. Known uncorrected
     residuals, stated not hidden: MT9 IODN swaps inject decimetre-class
     (<~1 ns) steps; the ~10 m-class GEO ranging bias is constant-class
     and vanishes in detrend (irrelevant to TDEV).
  8. Cross-satellite consistency view: the best >= 2-satellite
     co-coverage window (pairwise segment intersections ranked by span
     then member count; a segment — not a satellite — is the membership
     unit, so a re-seed inside the window disqualifies by construction,
     and membership is fixed over the whole window so the common-mode
     mean has no membership steps). Each member is detrended over the
     window; common = per-epoch mean (station clock + mean geometry
     curvature); deviations d_s = x_s - common. TDEV(common) and the
     per-sat TDEV(d_s) spread are reported — the spread IS the carrier
     noise-floor evidence at small tau.

There is no gen field in this stream; re-seed segmentation (4) is the
session-identity mechanism here.

Usage:
  python3 scripts/carrier_tdev_analyzer.py [path] [--max-bytes N]
Exit codes: 0 RESERVED (unreachable until a carrier gate is
pre-registered), 1 failed/insufficient/integrity, 2 exploratory numbers
produced.
"""
import json
import math
import os
import sys

from clock_bias_analyzer import detrend, tdev, split_segments, _median
import geocorrector_helper

TAUS = [1, 10, 100, 1000]
F_CARRIER_HZ = {
    "gps": 1575.42e6,
    "sbas": 1575.42e6,
    "galileo": 1575.42e6,
    "beidou": 1561.098e6,   # B1I
}
GEO_PRN_MIN = 120
GAP_FACTOR = 1.5            # same rule as clock_bias_analyzer
CADENCE_TOL_FRAC = 0.15     # see docstring item 4 (collector decimation
                            # jitter; results labeled "tau +/-10%")
RESEED_JUMP_HZ = 20e3       # |d cycles/dt| bound: MEO Doppler <= ~5 kHz
                            # + station clock ~1.5 ppm (~2.4 kHz) + drift
                            # stay far below; a re-zeroing jump is
                            # ~1e5-1e6 Hz apparent
GEO_MAX_ABS_DOPPLER_HZ = 200.0  # sanity bound on the MT9 prediction:
                            # GEO LOS 0.5-3 m/s -> ~3-16 Hz at L1, plus
                            # agf1 clock drift; 200 Hz rejects a garbage
                            # geonav without clipping any physical GEO
MIN_SEG_ROWS = 12           # ~2 min at the 10 s archive cadence; shorter
                            # fragments are counted, never analyzed
CARRIER_GATE_PREREGISTERED = False  # exit 0 unreachable until a spec
                            # pre-registers a carrier-phase gate (mirror
                            # of POISON_GATE_CLAIM_GRADE)
DEFAULT_PATH = "/Volumes/Radiator 8TB/gnss/observations/telemetry_log.jsonl"
DEFAULT_MAX_BYTES = 200 * 1024 * 1024


def x_ns_from_cycles(cycles, sysname):
    """x(t) = -carrier_cycles / f_carrier, in ns (sign: module docstring)."""
    return -cycles / F_CARRIER_HZ[sysname] * 1e9


def is_geo(sysname, prn):
    return sysname == "sbas" or prn >= GEO_PRN_MIN


def _finite(v):
    return type(v) in (int, float) and math.isfinite(v)


def _finite3(v):
    return (isinstance(v, list) and len(v) == 3 and all(_finite(c) for c in v))


def valid_geonav(g):
    """Pre-validate an MT9 vector: calc_geo_doppler_hz fails OPEN (returns
    0.0 on any bad input), so the fail-closed check must happen here."""
    if not isinstance(g, dict):
        return False
    if not (_finite3(g.get("pos_m")) and _finite3(g.get("vel_mps"))):
        return False
    if not _finite(g.get("t0_s")):
        return False
    for opt in ("acc_mps2",):
        if opt in g and not _finite3(g[opt]):
            return False
    for opt in ("agf0_s", "agf1_sps", "agf1"):
        if opt in g and not _finite(g[opt]):
            return False
    return True


def geo_correction_cycles(epochs, geonavs, site_ecef, f_carrier):
    """Cumulative predicted GEO carrier cycles (LOS motion + clock drift)
    over one segment, or (None, reason) if any sample is unusable.

    Trapezoid integral of calc_geo_doppler_hz per sample (each sample uses
    its own archived geonav — the freshest broadcast at that instant).
    Returns (list_of_cycles starting at 0.0, None) on success."""
    if site_ecef is None:
        return None, "no usable site.json"
    dopp = []
    for t, g in zip(epochs, geonavs):
        if not valid_geonav(g):
            return None, "missing/invalid MT9 geonav on a sample"
        d = geocorrector_helper.calc_geo_doppler_hz(g, site_ecef, f_carrier,
                                                    t_unix=t)
        if not (math.isfinite(d) and abs(d) <= GEO_MAX_ABS_DOPPLER_HZ):
            return None, "MT9 prediction out of physical bounds"
        dopp.append(d)
    out = [0.0]
    for i in range(1, len(epochs)):
        out.append(out[-1] + 0.5 * (dopp[i] + dopp[i - 1])
                   * (epochs[i] - epochs[i - 1]))
    return out, None


def tdev_grid(x_ns, dt_s, taus):
    """clock_bias_analyzer.tdev with honest tau bookkeeping for a coarse
    grid: a tau with round(tau/dt) < 1 is DROPPED (reason "cadence")
    instead of being silently remapped to m=1 (tdev()'s max(1, ...) would
    report TDEV(dt) under the requested-tau label); taus omitted by tdev()
    for span are reported dropped (reason "span").

    Returns (table {tau: tdev_ns}, tau_eff {tau: m*dt}, dropped
    [(tau, reason)])."""
    usable, dropped = [], []
    for tau in taus:
        if dt_s <= 0 or int(round(tau / dt_s)) < 1:
            dropped.append((tau, "cadence"))
        else:
            usable.append(tau)
    tbl = tdev(x_ns, dt_s, usable)
    tau_eff = {tau: max(1, int(round(tau / dt_s))) * dt_s for tau in tbl}
    for tau in usable:
        if tau not in tbl:
            dropped.append((tau, "span"))
    return tbl, tau_eff, dropped


def _extract_sat_samples(rows, rep):
    """Validate snapshot rows -> per-sat ordered sample lists.

    Returns {satkey: [sample dict]} with sample = {epoch, cycles, slip,
    lock_s, geonav}. Counting/fail-closed policy: module docstring 2-3."""
    per_sat = {}
    for r in rows:
        if not isinstance(r, dict) or not _finite(r.get("epoch")):
            rep["integrity_fails"].append("malformed snapshot row (not a "
                                          "dict with a finite epoch)")
            return per_sat
        sats = r.get("sats")
        if sats is None:
            rep["n_no_sats_snapshots"] += 1
            continue
        if not isinstance(sats, list):
            rep["integrity_fails"].append("snapshot 'sats' is not a list")
            return per_sat
        for s in sats:
            if not isinstance(s, dict):
                rep["integrity_fails"].append("sat entry is not a dict")
                return per_sat
            if "carrier_cycles" not in s:
                rep["n_legacy_sat_rows"] += 1   # pre-dcfcfaa schema
                continue
            sysname = s.get("sys")
            prn = s.get("prn")
            if not (isinstance(sysname, str) and type(prn) is int):
                rep["integrity_fails"].append(
                    "carrier-bearing sat row with malformed sys/prn")
                return per_sat
            if sysname not in F_CARRIER_HZ:
                rep["n_unknown_sys"] += 1
                rep["unknown_sys"].add(sysname)
                continue
            if not (_finite(s.get("epoch")) and _finite(s["carrier_cycles"])
                    and type(s.get("slip")) is bool
                    and _finite(s.get("lock_s"))):
                rep["integrity_fails"].append(
                    "carrier-bearing sat row with malformed "
                    "epoch/carrier_cycles/slip/lock_s "
                    "(%s prn %s)" % (sysname, prn))
                return per_sat
            if float(s["lock_s"]) <= 0.0:
                rep["n_unlocked_excluded"] += 1
                continue
            key = "%s-%d" % (sysname, prn)
            per_sat.setdefault(key, []).append({
                "epoch": float(s["epoch"]),
                "cycles": float(s["carrier_cycles"]),
                "slip": s["slip"],
                "lock_s": float(s["lock_s"]),
                "geonav": s.get("sbas_geonav"),
                "sys": sysname, "prn": prn,
            })
    return per_sat


def _dedupe(samples, srep):
    """Drop exact-duplicate epochs (collector double-read of one tracker
    second); a same-epoch carrier CONFLICT poisons the satellite."""
    out = []
    for s in samples:
        if out and s["epoch"] == out[-1]["epoch"]:
            if s["cycles"] == out[-1]["cycles"] and s["slip"] == out[-1]["slip"]:
                srep["n_dup_dropped"] += 1
                continue
            srep["conflict"] = ("same-epoch samples with different "
                                "carrier_cycles/slip at epoch %.3f"
                                % s["epoch"])
            return None
        out.append(s)
    return out


def _segment_sat(samples, srep):
    """Split at slip / re-seed / gap / cadence; NEVER bridge a re-seed.

    Returns list of index lists (into samples), each a final segment."""
    n = len(samples)
    # phase-break boundaries: boundary[i] == True means a break BETWEEN
    # sample i-1 and sample i (segment must not span it)
    chunks, start = [], 0
    for i in range(1, n):
        a, b = samples[i - 1], samples[i]
        dtv = b["epoch"] - a["epoch"]
        breaks = None
        if b["slip"]:
            breaks = "slip"
        elif b["lock_s"] < a["lock_s"]:
            breaks = "reseed (lock_s reset)"
        elif b["lock_s"] < a["lock_s"] + 0.75 * dtv:
            # A zeroing BETWEEN two freshly-seeded samples can leave
            # lock_s increased yet smaller than continuous tracking
            # implies (e.g. 3 -> 8 across a 10 s archive gap with the
            # slip flag lost to decimation and a sub-threshold jump):
            # continuous lock must age at wall rate, so anything less
            # proves an intervening re-seed. Adversarial-review probe C.
            breaks = "reseed (lock_s discontinuity)"
        elif dtv > 0 and abs(b["cycles"] - a["cycles"]) / dtv > RESEED_JUMP_HZ:
            breaks = "reseed (carrier zeroing jump)"
        elif (is_geo(b["sys"], b["prn"])
                and valid_geonav(b["geonav"]) != valid_geonav(a["geonav"])):
            # GEO segments must be geonav-homogeneous so the MT9
            # correction applies to a whole segment or none of it: the
            # rare no-geonav samples (tracker startup before the first
            # MT9 decode; ~0.5% of real SBAS rows) fragment off instead
            # of poisoning an hours-long correctable segment
            breaks = "geonav availability change"
        if breaks:
            chunks.append((start, i - 1))
            srep["breaks"].append({"at_epoch": b["epoch"], "reason": breaks})
            start = i
    chunks.append((start, n - 1))
    final = []
    for c0, c1 in chunks:
        ep = [samples[k]["epoch"] for k in range(c0, c1 + 1)]
        segs, holes, _ = split_segments(ep, GAP_FACTOR, CADENCE_TOL_FRAC)
        srep["n_grid_splits"] += len(holes)
        for i0, i1 in segs:
            idx = list(range(c0 + i0, c0 + i1 + 1))
            if len(idx) >= MIN_SEG_ROWS:
                final.append(idx)
            else:
                srep["n_short_segments"] += 1
    return final


def _build_segment_series(samples, idx, site_ecef, srep):
    """(epochs, x_ns, geo_status) for one final segment.

    geo_status: None (not GEO), "corrected" (MT9 LOS+clock-drift removed),
    or "motion-contaminated: <reason>"."""
    epochs = [samples[k]["epoch"] for k in idx]
    cycles = [samples[k]["cycles"] for k in idx]
    s0 = samples[idx[0]]
    geo_status = None
    if is_geo(s0["sys"], s0["prn"]):
        corr, err = geo_correction_cycles(
            epochs, [samples[k]["geonav"] for k in idx], site_ecef,
            F_CARRIER_HZ[s0["sys"]])
        if corr is None:
            geo_status = "motion-contaminated: " + err
        else:
            cycles = [c - g for c, g in zip(cycles, corr)]
            geo_status = "corrected"
    x_ns = [x_ns_from_cycles(c, s0["sys"]) for c in cycles]
    return epochs, x_ns, geo_status


def _cross_sat_view(seg_table):
    """Best >= 2-satellite co-coverage window: pairwise segment
    intersections, ranked by span then member count.

    Every segment is a contiguous run of the shared snapshot-epoch grid
    (sats in one snapshot NEARLY always share the tracker epoch — ~2%
    of real snapshots carry mixed epochs, which the matcher rejects
    fail-closed rather than aligning; a per-sat hole splits that sat's
    segment), so the intersection of two segments is itself contiguous. For each cross-sat segment pair the candidate
    window is their epoch intersection; its members are ALL segments that
    cover every window epoch — a segment (not a satellite) is the unit,
    so a re-seed inside the window disqualifies by construction (never
    bridged). Membership is fixed over the whole window, so the
    common-mode mean has no membership steps. seg_table:
    [(satkey, seg_id, {epoch: x_ns})], GEO only when MT9-corrected.
    Returns report dict or {"unavailable": reason}."""
    entries = []
    for satkey, seg_id, exmap in seg_table:
        eps = sorted(exmap)
        entries.append((satkey, seg_id, eps[0], eps[-1], set(eps), exmap))
    best = None  # (span, n_members, win_epochs, member_entries)
    for i in range(len(entries)):
        for j in range(i + 1, len(entries)):
            if entries[i][0] == entries[j][0]:
                continue           # same satellite: not a cross-sat pair
            lo = max(entries[i][2], entries[j][2])
            hi = min(entries[i][3], entries[j][3])
            if hi <= lo:
                continue
            win = sorted(e for e in entries[i][4] if lo <= e <= hi)
            if len(win) < MIN_SEG_ROWS:
                continue
            wset = set(win)
            members = [ent for ent in entries if wset <= ent[4]]
            sats = {ent[0] for ent in members}
            if len(sats) < 2 or len(members) != len(sats):
                continue           # fail closed on any coverage anomaly
            span = win[-1] - win[0]
            cand = (span, len(members), win, members)
            if best is None or (span, len(members)) > (best[0], best[1]):
                best = cand
    if best is None:
        return {"unavailable": "no >=2-satellite co-coverage window "
                               ">= %d epochs" % MIN_SEG_ROWS}
    span, _, win, members = best
    detr = {}
    for satkey, seg_id, _, _, _, em in members:
        detr[satkey] = detrend(win, [em[e] for e in win])
    dt_s = _median([b - a for a, b in zip(win, win[1:]) if b > a])
    nsat = len(detr)
    common = [sum(detr[k][i] for k in detr) / nsat for i in range(len(win))]
    ctbl, ceff, cdrop = tdev_grid(common, dt_s, TAUS)
    devs = {}
    for k in detr:
        d = [detr[k][i] - common[i] for i in range(len(win))]
        devs[k], _, _ = tdev_grid(d, dt_s, TAUS)
    spread_median = {}
    for tau in TAUS:
        vals = [devs[k][tau] for k in devs if tau in devs[k]]
        if vals:
            spread_median[tau] = _median(vals)
    return {"span_s": span, "rows": len(win), "dt_s": dt_s,
            "sats": sorted(detr),
            "common_tdev": ctbl, "common_tau_eff": ceff,
            "common_dropped": cdrop,
            "dev_tdev": devs, "spread_median_tdev": spread_median}


def analyze(rows, parse_errors=0, shrunk=False, site_ecef=None):
    """Full pipeline on parsed snapshot rows -> report dict (main prints).

    Fail-closed: any integrity failure (parse errors, shrinking input,
    malformed carrier-bearing row) yields integrity_fails and no numbers."""
    rep = {"n_in": len(rows), "n_parse_errors": parse_errors,
           "n_no_sats_snapshots": 0, "n_legacy_sat_rows": 0,
           "n_unknown_sys": 0, "unknown_sys": set(),
           "n_unlocked_excluded": 0,
           "integrity_fails": [], "sats": {}, "cross": None,
           "exploratory_reasons": []}
    if parse_errors:
        rep["integrity_fails"].append(
            "%d unparseable JSON line(s)" % parse_errors)
    if shrunk:
        rep["integrity_fails"].append(
            "input file shrank during the read (append-only prefix "
            "assumption violated); analyze an immutable snapshot")
    if rep["integrity_fails"]:
        return rep
    per_sat = _extract_sat_samples(rows, rep)
    if rep["integrity_fails"]:
        return rep

    seg_table = []   # cross-sat eligible: (satkey, seg_id, {epoch: x_ns})
    for key in sorted(per_sat):
        srep = {"n_samples": len(per_sat[key]), "n_dup_dropped": 0,
                "conflict": None, "breaks": [], "n_grid_splits": 0,
                "n_short_segments": 0, "n_segments": 0,
                "geo": is_geo(per_sat[key][0]["sys"], per_sat[key][0]["prn"]),
                "chosen": None}
        rep["sats"][key] = srep
        samples = _dedupe(per_sat[key], srep)
        if samples is None:
            continue   # conflict: satellite excluded, reason printed
        segs = _segment_sat(samples, srep)
        srep["n_segments"] = len(segs)
        if not segs:
            continue
        built = []
        for seg_id, idx in enumerate(segs):
            epochs, x_ns, geo_status = _build_segment_series(
                samples, idx, site_ecef, srep)
            built.append((seg_id, epochs, x_ns, geo_status))
            if not (geo_status or "").startswith("motion-contaminated"):
                seg_table.append((key, seg_id, dict(zip(epochs, x_ns))))
        # per-sat headline: the longest segment (by span, then rows)
        seg_id, epochs, x_ns, geo_status = max(
            built, key=lambda b: (b[1][-1] - b[1][0], len(b[1])))
        res = detrend(epochs, x_ns)
        dt_s = _median([b - a for a, b in zip(epochs, epochs[1:]) if b > a])
        tbl, tau_eff, dropped = tdev_grid(res, dt_s, TAUS)
        srep["chosen"] = {
            "seg_id": seg_id, "rows": len(epochs),
            "span_s": epochs[-1] - epochs[0], "dt_s": dt_s,
            "rms_ns": math.sqrt(sum(r * r for r in res) / len(res)),
            "tdev": tbl, "tau_eff": tau_eff, "dropped": dropped,
            "geo_status": geo_status,
        }
    rep["cross"] = _cross_sat_view(seg_table)

    rep["exploratory_reasons"] = [
        "no carrier-phase gate is pre-registered (spec required before "
        "any pass/fail verdict; exit 0 unreachable by construction)",
        "archived cadence is the collector's 10 s decimation with +/-1 s "
        "grid jitter: every tau carries ~10% sampling-time uncertainty",
        "linear detrend leaves LOS Doppler curvature: per-sat MEO TDEV "
        "at tau >~ 100 s is geometry-dominated (upper bound only)",
        "GEO MT9 IODN swaps inject uncorrected decimetre-class (<~1 ns) "
        "steps; the ~10 m GEO ranging bias is constant and detrends away",
    ]
    return rep


def report_exit_status(rep):
    """1 = failed/insufficient/integrity, 2 = exploratory numbers produced.
    0 stays unreachable until a carrier gate is pre-registered."""
    if rep["integrity_fails"]:
        return 1
    produced = any(s.get("chosen") and s["chosen"]["tdev"]
                   for s in rep["sats"].values())
    if not produced:
        return 1
    if CARRIER_GATE_PREREGISTERED:
        raise AssertionError(
            "no pre-registered carrier gate exists; a True constant here "
            "requires the gate spec and a deliberate code change")
    return 2


def load_rows(path, max_bytes=DEFAULT_MAX_BYTES):
    """Immutable-prefix read of an append-only jsonl (docstring item 1).

    Returns (rows, parse_errors, shrunk)."""
    with open(path, "rb") as f:
        size0 = os.fstat(f.fileno()).st_size
        start = 0
        tail_window = bool(max_bytes) and size0 > max_bytes
        if tail_window:
            start = size0 - max_bytes
            f.seek(start)
        payload = f.read(size0 - start)
        shrunk = os.fstat(f.fileno()).st_size < size0
    text = payload.decode("utf-8", errors="replace")
    lines = text.split("\n")
    if tail_window:
        lines = lines[1:]          # first line of a tail window is partial
    if lines and lines[-1] != "":
        lines = lines[:-1]         # unterminated final line: still being
                                   # appended, not part of the snapshot
    rows, bad = [], 0
    for line in lines:
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            bad += 1
    return rows, bad, shrunk


def _print_tdev_block(indent, tbl, tau_eff, dropped):
    for tau in TAUS:
        if tau in tbl:
            print("%sTDEV(requested %4d s, effective %.1f s): %.3f ns"
                  % (indent, tau, tau_eff[tau], tbl[tau]))
    for tau, reason in sorted(dropped):
        print("%sTDEV(requested %4d s): dropped (%s)" % (indent, tau,
              "cadence too coarse for this tau" if reason == "cadence"
              else "span < 3*tau"))


def main():
    argv = sys.argv[1:]
    max_bytes = DEFAULT_MAX_BYTES
    if "--max-bytes" in argv:
        i = argv.index("--max-bytes")
        max_bytes = int(argv[i + 1])
        del argv[i:i + 2]
    path = argv[0] if argv and not argv[0].startswith("-") else DEFAULT_PATH
    try:
        rows, bad, shrunk = load_rows(path, max_bytes)
    except FileNotFoundError:
        rows, bad, shrunk = [], 0, False
    site = geocorrector_helper.get_site_ecef()
    rep = analyze(rows, parse_errors=bad, shrunk=shrunk, site_ecef=site)

    print("file: %s (%d snapshot rows read%s)" % (
        path, rep["n_in"],
        ", %d unparseable lines" % bad if bad else ""))
    print("source: telemetry_log.jsonl sats rows — the only durable "
          "archive of the tracker carrier observable (10 s collector "
          "decimation of the 1 s tracker grid; tau=1 s needs a future "
          "1 Hz archive)")
    print("counts: %d legacy (pre-carrier-schema) sat rows excluded, "
          "%d unknown-system rows excluded%s, %d unlocked (lock_s<=0) "
          "samples excluded, %d snapshots without sats" % (
              rep["n_legacy_sat_rows"], rep["n_unknown_sys"],
              " (%s)" % ", ".join(sorted(rep["unknown_sys"]))
              if rep["unknown_sys"] else "",
              rep["n_unlocked_excluded"], rep["n_no_sats_snapshots"]))
    if rep["integrity_fails"]:
        print("integrity failure: " + "; ".join(rep["integrity_fails"]))
        print("INSUFFICIENT DATA")
        sys.exit(report_exit_status(rep))

    print("per-satellite TDEV (x = -carrier_cycles/f, linear detrend per "
          "segment; segments split at slip/re-seed/gap/cadence, never "
          "bridged; tau +/-10% from archive grid jitter):")
    for key in sorted(rep["sats"]):
        s = rep["sats"][key]
        if s["conflict"]:
            print("  %s: EXCLUDED — %s" % (key, s["conflict"]))
            continue
        head = ("  %s: %d samples, %d segment(s) (>=%d rows), "
                "%d short fragment(s), %d break(s), %d grid split(s), "
                "%d duplicate(s) dropped"
                % (key, s["n_samples"], s["n_segments"], MIN_SEG_ROWS,
                   s["n_short_segments"], len(s["breaks"]),
                   s["n_grid_splits"], s["n_dup_dropped"]))
        print(head)
        c = s["chosen"]
        if not c:
            print("    no segment >= %d rows — insufficient" % MIN_SEG_ROWS)
            continue
        label = ""
        if s["geo"]:
            label = ("GEO, MT9 LOS+clock-drift removed"
                     if c["geo_status"] == "corrected"
                     else "GEO, %s" % c["geo_status"])
        else:
            label = ("MEO/IGSO — tau >~100 s geometry-dominated "
                     "(upper bound only)")
        print("    longest seg: span %.0f s, %d rows, dt %.1f s — %s"
              % (c["span_s"], c["rows"], c["dt_s"], label))
        print("    detrended RMS: %.3f ns" % c["rms_ns"])
        _print_tdev_block("    ", c["tdev"], c["tau_eff"], c["dropped"])

    cross = rep["cross"]
    print("cross-satellite consistency view (common mode = station clock "
          "+ mean geometry curvature; per-sat spread IS the noise-floor "
          "evidence):")
    if cross is None or "unavailable" in (cross or {}):
        print("  unavailable: %s" % (cross or {}).get(
            "unavailable", "no data"))
    else:
        print("  window: span %.0f s, %d epochs, dt %.1f s, sats: %s"
              % (cross["span_s"], cross["rows"], cross["dt_s"],
                 ", ".join(cross["sats"])))
        print("  common-mode TDEV:")
        _print_tdev_block("    ", cross["common_tdev"],
                          cross["common_tau_eff"], cross["common_dropped"])
        print("  per-sat deviation-from-common TDEV (median across sats):")
        for tau in TAUS:
            if tau in cross["spread_median_tdev"]:
                print("    tau %4d s: %.3f ns"
                      % (tau, cross["spread_median_tdev"][tau]))
        for k in sorted(cross["dev_tdev"]):
            parts = ["tau %d: %.3f ns" % (tau, v) for tau, v in
                     sorted(cross["dev_tdev"][k].items())]
            print("    %s: %s" % (k, "; ".join(parts)))

    status = report_exit_status(rep)
    if status == 2:
        print("VERDICT: EXPLORATORY CARRIER-PHASE TDEV REPORT — NO GATE "
              "EXISTS, NO CLAIM IS SUPPORTED")
        for reason in rep["exploratory_reasons"]:
            print("  - " + reason)
    else:
        print("VERDICT: insufficient data (no satellite produced a TDEV "
              "table)")
    sys.exit(status)


if __name__ == "__main__":
    main()
