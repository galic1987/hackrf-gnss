#!/usr/bin/env python3
"""Position history watcher: turns state.position.json snapshots into a
position-over-time record for the panel.

position_producer.py runs examples/live_fix every 5 min, which overwrites
observations/state.position.json with the LATEST fix — no history. This
watcher polls that file every 30 s (cheap stat + parse; never opens a radio,
never signals a process) and, when the fix epoch changes:

  1. appends one line to observations/position_history.jsonl (append-only,
     durable; one JSON object per fix) — ONLY when the solver marked the
     fix trusted_for_history (redundant geometry AND plausibility-passing;
     a missing field defaults to false). Sane-but-untrusted fixes go to
     observations/position_history_diagnostic.jsonl instead: visible,
     but never feeding consumers of the trusted history. Physically
     impossible fixes (plausibility gate below) are not ingested at all.
     Publication law (round-13): "position" is TRUSTED-only — a plausible
     but EXACT solve (unverifiable by construction) publishes as
     "position_candidate", a plausibility-FAILING solve as
     "position_diagnostic", and in both untrusted classes the last trusted
     fix survives in "position", honestly aging. This watcher reads all
     three channels and appends fresh candidate/diagnostic solves to the
     diagnostic history under the same sanity gate; without it those
     solves would go dark.
     Each history row also carries the solve's trust/quality metadata
     (geometry_redundant, plausibility_pass, trusted_for_history,
     residual_rms_m, loo, gdop, source); rows written before the round-12
     rename still carry it under the former integrity-valid key, and
  2. rewrites observations/state.position_history.json (tmp + os.replace,
     per the merge architecture) carrying the last WINDOW_S of fixes under
     the "position_history" key, so the panel server merges it into
     /api/sync at read time with NO server change. mtime expiry (ttl_s)
     tombstones it if this watcher dies.

Standalone so it goes live without touching the running position_producer.
Sandboxable for tests via HACKRF_GNSS_OBS (observations dir override).
"""
import json
import os
import time

OBS = os.environ.get("HACKRF_GNSS_OBS", "/Volumes/Radiator 8TB/gnss/observations")
STATE_IN = f"{OBS}/state.position.json"
HISTORY = f"{OBS}/position_history.jsonl"
# sane-but-untrusted solves (trusted_for_history != true): visible here,
# never in the trusted history above that consumers draw from
HISTORY_DIAG = f"{OBS}/position_history_diagnostic.jsonl"
STATE_OUT = f"{OBS}/state.position_history.json"

POLL_S = 30
WINDOW_S = 3 * 3600          # keep what the panel draws: last 3 h of fixes
TTL_S = 15 * 60              # panel drops us 15 min after our last heartbeat

# Canonical site anchor (observations/site.json), re-read on every state
# write; the panel converts fixes to ENU relative to it. NO hardcoded
# coordinates: a missing anchor omits the site key (the panel shows its
# no-site state) rather than publishing a guessed location.
def load_site():
    try:
        with open(f"{OBS}/site.json") as f:
            s = json.load(f)
        return {"lat": float(s["lat"]), "lon": float(s["lon"]),
                "alt_m": float(s.get("h_m", 20.0))}
    except Exception:
        return None


# Plausibility gate: fixes outside this altitude band are physically
# impossible for this station (roof / road) and historically entered
# position_history ungated (review round 4). Applied at INGEST so the
# durable log never records them.
ALT_MIN_KM, ALT_MAX_KM = -1.0, 30.0


def fix_sane(fix):
    """True unless the fix is physically impossible for this station."""
    alt = fix.get("alt_km")
    if alt is None or not isinstance(alt, (int, float)):
        return False
    if not (ALT_MIN_KM <= alt <= ALT_MAX_KM):
        return False
    return abs(fix.get("lat", 999.0)) <= 90.0 and abs(fix.get("lon", 999.0)) <= 180.0


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def read_fix(key="position"):
    """Latest fix from state.position.json, flattened, or None.

    `key` selects the channel: "position" (the trusted position of record),
    "position_candidate" (plausible-but-EXACT solves — the round-13 law
    keeps them out of `position`), or "position_diagnostic"
    (plausibility-failing solves).
    """
    with open(STATE_IN) as f:
        st = json.load(f)
    pos = st.get(key)
    if not isinstance(pos, dict) or pos.get("lat") is None:
        return None
    # The publication gate (round-11) preserves the last VALID fix across
    # invalid eras under a fresh file epoch — the fix's own nested epoch is
    # the truth for freshness/dedupe; fall back to the file epoch when absent.
    return {
        "epoch": pos.get("epoch") or st.get("epoch"),
        "lat": pos["lat"],
        "lon": pos["lon"],
        "alt_km": pos.get("alt_km"),
        "mode": pos.get("mode"),
        "gate": pos.get("gate"),
        "gdop": pos.get("gdop"),
        "n_sats": pos.get("n_sat"),
        "isx_km": pos.get("isx_km"),      # mixed GPS+BDS solves only
        "bds_quarantined": pos.get("bds_quarantined"),  # reason string when the mixed solve failed the plausibility law
        # PVT trust fields + quality/provider metadata from live_fix —
        # carried through so history rows keep the solve's provenance
        "residual_rms_m": pos.get("residual_rms_m"),
        "geometry_redundant": pos.get("geometry_redundant"),
        "plausibility_pass": pos.get("plausibility_pass"),
        "trusted_for_history": pos.get("trusted_for_history"),
        "loo": pos.get("loo"),            # LOO-exclusion note, when present
        "source": pos.get("source"),
    }


def last_history_epoch(path=HISTORY):
    """Epoch of the last appended line, so a restart doesn't duplicate it."""
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            if f.tell() == 0:
                return None
            # history lines are small; a 4 KB tail always holds the last one
            f.seek(max(0, f.tell() - 4096))
            tail = f.read().decode("utf-8", "replace").strip().splitlines()
        return json.loads(tail[-1]).get("epoch") if tail else None
    except (OSError, ValueError):
        return None


def load_window(now):
    """Recent fixes for the state file: JSONL tail within WINDOW_S."""
    out = []
    try:
        with open(HISTORY) as f:
            for line in f:
                try:
                    fix = json.loads(line)
                except ValueError:
                    continue
                ep = fix.get("epoch")
                if isinstance(ep, (int, float)) and now - ep <= WINDOW_S:
                    out.append(fix)
    except OSError:
        pass
    return out


def write_state(fixes):
    tmp = STATE_OUT + ".tmp"
    doc = {
        "position_history": fixes,
        "epoch": time.time(),
        "ttl_s": TTL_S,
    }
    site = load_site()
    if site:
        doc["site"] = site
    with open(tmp, "w") as f:
        json.dump(doc, f)
    os.replace(tmp, STATE_OUT)


def cycle(seen_epoch, seen_cand_epoch, seen_diag_epoch):
    """One poll. Returns the epochs now considered seen (unchanged if none)."""
    fix = read_fix()
    if fix and fix["epoch"] and (seen_epoch is None or fix["epoch"] > seen_epoch):
        # STRICTLY-newer dedupe (round-13 9b): `position` always holds the
        # LATEST trusted fix — older ones are merely PRESERVED across
        # candidate/failing eras — so anything not newer than the restart
        # cursor was already recorded. Plain `!=` would re-ingest a
        # preserved older fix whenever the diagnostic tail is newer.
        if not fix_sane(fix):
            # physically impossible for this station: logged, NOT ingested
            log(f"fix REJECTED as impossible (alt={fix.get('alt_km')} km, "
                f"gate={fix.get('gate')}) — kept out of history")
        elif fix.get("trusted_for_history") is True:
            # the trusted history takes ONLY solver-trusted fixes
            # (redundant geometry + plausibility-pass; missing field = false)
            with open(HISTORY, "a") as f:
                f.write(json.dumps(fix) + "\n")
            log(f"fix {fix['mode']} gate={fix['gate']} "
                f"lat={fix['lat']:.6f} lon={fix['lon']:.6f} "
                f"alt={fix['alt_km'] * 1000:.0f} m -> history")
        else:
            # sane but untrusted (legacy producers only — round-13 keeps
            # untrusted solves out of "position"): stays visible in the
            # diagnostic log, never feeds consumers of the trusted history
            with open(HISTORY_DIAG, "a") as f:
                f.write(json.dumps(fix) + "\n")
            log(f"fix {fix['mode']} gate={fix['gate']} UNTRUSTED "
                f"(rms={fix.get('residual_rms_m')} m, "
                f"isx={fix.get('isx_km')} km) -> diagnostic history")
        seen_epoch = fix["epoch"]
    # Candidate channel (round-13): plausible-but-EXACT solves publish ONLY
    # as "position_candidate" — one home, never the position of record, no
    # diagnostic mirror. They drain to the diagnostic history under the
    # same sanity gate.
    cand = read_fix("position_candidate")
    if cand and cand["epoch"] and cand["epoch"] != seen_cand_epoch:
        if not fix_sane(cand):
            log(f"candidate fix REJECTED as impossible (alt={cand.get('alt_km')} km, "
                f"gate={cand.get('gate')}) — kept out of history")
        else:
            with open(HISTORY_DIAG, "a") as f:
                f.write(json.dumps(cand) + "\n")
            log(f"candidate (exact) fix {cand['mode']} gate={cand['gate']} "
                f"(rms={cand.get('residual_rms_m')} m, "
                f"isx={cand.get('isx_km')} km) -> diagnostic history")
        seen_cand_epoch = cand["epoch"]
    # Diagnostic channel: plausibility-FAILING solves live ONLY in
    # "position_diagnostic" (the publication gate keeps them out of
    # "position") and would go dark without a reader. Same ingest law as the
    # main channel — sane ones land in the diagnostic history, impossible
    # ones are logged and dropped. A diagnostic row that merely MIRRORS the
    # current main fix (pre-round-13 producers mirrored plausible-but-
    # untrusted solves) was already routed above, so epoch equality skips
    # it here.
    diag = read_fix("position_diagnostic")
    if (diag and diag["epoch"] and diag["epoch"] != seen_diag_epoch
            and diag["epoch"] != (fix or {}).get("epoch")):
        if not fix_sane(diag):
            log(f"diagnostic fix REJECTED as impossible (alt={diag.get('alt_km')} km, "
                f"gate={diag.get('gate')}) — kept out of history")
        else:
            with open(HISTORY_DIAG, "a") as f:
                f.write(json.dumps(diag) + "\n")
            log(f"diagnostic (plausibility-failed) fix {diag['mode']} gate={diag['gate']} "
                f"(rms={diag.get('residual_rms_m')} m, "
                f"isx={diag.get('isx_km')} km) -> diagnostic history")
        seen_diag_epoch = diag["epoch"]
    write_state(load_window(time.time()))
    return seen_epoch, seen_cand_epoch, seen_diag_epoch


def seed_cursors():
    """Initial (main, candidate, diagnostic) cursors from the history tails.

    Round-13 restart dedupe: the main cursor takes the NEWEST epoch across
    the trusted AND diagnostic tails — the last event before a restart may
    be an untrusted or candidate row that only ever landed in the
    diagnostic history, and seeding from the trusted tail alone re-ingests
    the still-current doc and appends it again. A trusted fix's epoch never
    appears ONLY in the diagnostic tail, and the main channel ingests only
    STRICTLY newer epochs, so a recorded trusted row is neither skipped
    nor duplicated. The candidate/diagnostic cursors seed from the
    diagnostic history both channels drain into.
    """
    seen_trusted = last_history_epoch(HISTORY)
    seen_diag = last_history_epoch(HISTORY_DIAG)
    tails = [e for e in (seen_trusted, seen_diag) if e is not None]
    return (max(tails) if tails else None), seen_diag, seen_diag


def main():
    log(f"position watch starting (obs={OBS})")
    seen, seen_cand, seen_diag = seed_cursors()
    while True:
        try:
            seen, seen_cand, seen_diag = cycle(seen, seen_cand, seen_diag)
        except Exception as e:
            # a missing/corrupt state file must never kill the watcher
            log(f"cycle failed: {e}")
        time.sleep(POLL_S)


if __name__ == "__main__":
    main()
