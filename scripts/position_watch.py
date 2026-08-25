#!/usr/bin/env python3
"""Position history watcher: turns state.position.json snapshots into a
position-over-time record for the panel.

position_producer.py runs examples/live_fix every 5 min, which overwrites
observations/state.position.json with the LATEST fix — no history. This
watcher polls that file every 30 s (cheap stat + parse; never opens a radio,
never signals a process) and, when the fix epoch changes:

  1. appends one line to observations/position_history.jsonl (append-only,
     durable; one JSON object per fix), and
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


def read_fix():
    """Latest fix from state.position.json, flattened, or None."""
    with open(STATE_IN) as f:
        st = json.load(f)
    pos = st.get("position")
    if not isinstance(pos, dict) or pos.get("lat") is None:
        return None
    return {
        "epoch": st.get("epoch"),
        "lat": pos["lat"],
        "lon": pos["lon"],
        "alt_km": pos.get("alt_km"),
        "mode": pos.get("mode"),
        "gate": pos.get("gate"),
        "gdop": pos.get("gdop"),
        "n_sats": pos.get("n_sat"),
        "isx_km": pos.get("isx_km"),      # mixed GPS+BDS solves only
    }


def last_history_epoch():
    """Epoch of the last appended line, so a restart doesn't duplicate it."""
    try:
        with open(HISTORY, "rb") as f:
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


def cycle(seen_epoch):
    """One poll. Returns the epoch now considered seen (unchanged if none)."""
    fix = read_fix()
    if fix and fix["epoch"] and fix["epoch"] != seen_epoch:
        if not fix_sane(fix):
            # physically impossible for this station: logged, NOT ingested
            log(f"fix REJECTED as impossible (alt={fix.get('alt_km')} km, "
                f"gate={fix.get('gate')}) — kept out of history")
        else:
            with open(HISTORY, "a") as f:
                f.write(json.dumps(fix) + "\n")
            log(f"fix {fix['mode']} gate={fix['gate']} "
                f"lat={fix['lat']:.6f} lon={fix['lon']:.6f} "
                f"alt={fix['alt_km'] * 1000:.0f} m -> history")
        seen_epoch = fix["epoch"]
    write_state(load_window(time.time()))
    return seen_epoch


def main():
    log(f"position watch starting (obs={OBS})")
    seen = last_history_epoch()
    while True:
        try:
            seen = cycle(seen)
        except Exception as e:
            # a missing/corrupt state file must never kill the watcher
            log(f"cycle failed: {e}")
        time.sleep(POLL_S)


if __name__ == "__main__":
    main()
