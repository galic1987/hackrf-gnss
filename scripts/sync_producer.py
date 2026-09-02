#!/usr/bin/env python3
"""Live drift producer for the /sync panel: polls the FPGA timestamp counter
over SPI (--ts-read now) and publishes tick-rate-vs-PC drift to its OWN file,
observations/state.tick.json (keys: epoch, seq, clock). The Rust server
deep-merges all observations/state.*.json with the legacy sync_state.json at
/api/sync read time, so this producer never touches shared state and no
lost-update race is possible. Runs until killed. Radio-safe: read-only, and
tolerates transient failures (flash windows) by skipping cycles.
"""
import json, re, subprocess, time, os, sys

print(
    "QUARANTINED: legacy SPI poller targets the dead Pro #1 and can contend "
    "with the production tracker; no radio was opened.",
    file=sys.stderr,
)
raise SystemExit(78)

PRO = "/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_pro"
ENV = dict(os.environ, DYLD_LIBRARY_PATH="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/libhackrf/src")
SERIAL = "QUARANTINED_NO_SERIAL"
STATE = "/Volumes/Radiator 8TB/gnss/observations/state.tick.json"
POLL_S = 2.0

def read_now():
    try:
        out = subprocess.run([PRO, "-d", SERIAL, "--ts-read", "now"],
                             capture_output=True, text=True, timeout=5, env=ENV).stdout
        m = re.search(r"ts\.now = (\d+) ticks", out)
        return (time.time(), int(m.group(1))) if m else None
    except Exception:
        return None

pairs = []          # (host_t, ticks)
rates = []          # (host_t, rate_hz) for the panel sparkline
seq = 0
rate0 = None        # first good rate estimate, Hz
while True:
    r = read_now()
    now = time.time()
    if r:
        pairs.append(r)
        pairs = [p for p in pairs if now - p[0] < 180]
        if len(pairs) > 10:
            (t0, k0), (t1, k1) = pairs[0], pairs[-1]
            dt = t1 - t0
            if dt > 30:
                rate = (k1 - k0) / dt
                if rate0 is None and rate > 1e6:
                    rate0 = rate
                drift_ppm = (rate - rate0) / rate0 * 1e6 if rate0 else 0.0
                rates.append((now, rate))
                rates = [x for x in rates if now - x[0] < 3600]
                seq += 1
                # ONLY this producer's keys — the server merges the rest
                state = {
                    "epoch": now,
                    "seq": seq,
                    "clock": {
                        "live_tick_hz": round(rate, 1),
                        "tick_rate_drift_ppm_vs_session_ref": round(drift_ppm, 3),
                        "samples": len(pairs),
                        "note": "live SPI poll; ref = first rate this session; idle counter ~32.0 MHz domain",
                        "recent": [[round(t, 1), round(hz, 1)] for t, hz in rates],
                    },
                }
                tmp = STATE + ".tmp"
                json.dump(state, open(tmp, "w"), indent=1)
                os.replace(tmp, STATE)
                # history for all-night stats (one line per write); keep the
                # historical filename even though the live file moved
                with open(STATE.replace("state.tick.json", "sync_state_history.jsonl"), "a") as h:
                    h.write(json.dumps({"t": now, "tick_hz": round(rate, 1),
                                        "drift_ppm": round(drift_ppm, 3),
                                        "n": len(pairs)}) + "\n")
    time.sleep(POLL_S)
