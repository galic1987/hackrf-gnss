#!/usr/bin/env python3
"""10-s merged telemetry snapshot collector -> observations/telemetry_log.jsonl.

The always-on half of the archive (see observations/archive/README.md).
Every 10 s, reads each producer's observations/state.<name>.json READ-ONLY
and appends one merged JSON line that archive_roller.py later compacts into
the partitioned Parquet archive.

Merge mirrors the server's tombstone rule: a file whose `epoch` is older
than its `ttl_s` (default 20 min) is a dead producer and is skipped, so the
archive records dropouts as absence — the dropout-cause signal. The `files`
list in every row records exactly which producers were alive.

Owns telemetry_log.jsonl the way producers own their state files (nobody
else appends). Append is a single write() of one line, atomic enough for
<64 KiB lines on local filesystems. Any failure (missing/corrupt state
file, disk hiccup) is caught, logged, and the loop continues — this
collector must never die because a producer did.

CPU is negligible: a few small JSON reads per 10 s.

  telemetry_collector.py           # run forever
  telemetry_collector.py --once    # one snapshot to stdout (testing)
"""
import json
import os
import sys
import time
import traceback

OBS = os.environ.get("HACKRF_GNSS_OBS", "/Volumes/Radiator 8TB/gnss/observations")
OUT = os.path.join(OBS, "telemetry_log.jsonl")
PERIOD = 10.0
DEFAULT_TTL = 1200.0             # mirrors the server's 20-min default
STATE_FILES = ("tracker", "phase", "band", "series", "tick", "position")


def read_state(name, now):
    """Fresh state dict, or None if missing/corrupt/expired."""
    try:
        with open(os.path.join(OBS, f"state.{name}.json")) as f:
            d = json.load(f)
    except Exception:
        return None
    ttl = d.get("ttl_s", DEFAULT_TTL)
    if now - d.get("epoch", now) > ttl:
        return None                # tombstone: dead producer's values expire
        
    # Prevent laundering of fresh but unlocked tombstones into the archive
    if name == "phase" and d.get("phase", {}).get("lock") is False:
        return None
        
    return d


def snapshot(now=None):
    now = time.time() if now is None else now
    row = {"epoch": round(now, 3), "files": []}
    for name in STATE_FILES:
        d = read_state(name, now)
        if d is None:
            continue
        row["files"].append(name)
        if name == "tracker":
            row["sats"] = d.get("tracker", {}).get("sats") or []
            if isinstance(d.get("discipline"), dict):
                row["discipline"] = d["discipline"]
        elif name == "phase" and isinstance(d.get("phase"), dict):
            # drop the embedded 60 Hz series — phase_history.jsonl has it
            row["phase"] = {k: v for k, v in d["phase"].items() if k != "series"}
        elif name == "tick":
            row["tick_hz"] = (d.get("clock") or {}).get("live_tick_hz")
        elif name == "position" and isinstance(d.get("position"), dict):
            row["position"] = d["position"]
        for s in d.get("sources") or []:
            if isinstance(s, dict):
                s = dict(s)
                s["producer"] = name
                row.setdefault("sources", []).append(s)
    return row


def main():
    if "--once" in sys.argv:
        print(json.dumps(snapshot(), indent=1, sort_keys=True))
        return
    while True:
        t0 = time.time()
        try:
            line = json.dumps(snapshot(t0), separators=(",", ":"))
            with open(OUT, "a") as f:
                f.write(line + "\n")
        except Exception:
            traceback.print_exc()   # never let a bad cycle kill the loop
        time.sleep(max(1.0, PERIOD - (time.time() - t0)))


if __name__ == "__main__":
    main()
