#!/usr/bin/env python3
"""Rolling 1-hour per-band precision series for the /sync panel.

Every 30 s, reads the MERGED panel state from http://localhost:8090/api/sync
(the Rust server merges the legacy sync_state.json with every producer's
observations/state.*.json, so the API is the single always-correct view —
chosen over re-implementing the merge locally), lifts the current value of
every live ClockDriftPpm source plus the consensus, and maintains a rolling
1-h window per band, published to its OWN file observations/state.series.json
(keys: epoch, ttl_s, band_series, sources=[the "PC clock" row]). Presence
bands contribute a "sats seen" count series ("n:<band>"). Backfills on
startup from band_drift_history.jsonl and phase_history.jsonl so the chart
has an instant hour of context.

Race-free: this file is written by nobody else (tmp + os.replace). ttl_s=120
tells the server to drop this file's contribution ~2 min (4 missed polls)
after this producer dies — the band_series chart must vanish, not freeze.
"""
import json, os, time, urllib.request

OBS = "/Volumes/Radiator 8TB/gnss/observations"
STATE = f"{OBS}/state.series.json"
API = "http://localhost:8090/api/sync"
HIST = f"{OBS}/band_drift_history.jsonl"
PHASE = f"{OBS}/phase_history.jsonl"
WINDOW = 3600.0
POLL = 30.0
MAXPTS = 150            # per series, decimated
CH35_HZ = 602.30944e6

series = {}             # label -> [[t, value], ...]


def add(label, t, v):
    pts = series.setdefault(label, [])
    if pts and t <= pts[-1][0]:
        return                       # dedupe / out-of-order guard
    pts.append([round(t, 1), round(v, 4)])
    del pts[:-MAXPTS]
    cutoff = time.time() - WINDOW
    while pts and pts[0][0] < cutoff:
        pts.pop(0)


def backfill():
    try:
        with open(HIST) as f:
            for line in f:
                try:
                    h = json.loads(line)
                except Exception:
                    continue
                t = h.get("t", 0)
                if time.time() - t > WINDOW:
                    continue
                for k, label in (("ATSC ch23", "ATSC ch23"), ("ATSC ch35", "ATSC ch35"),
                                 ("waas", "L1 / WAAS")):
                    if k in h:
                        add(label, t, h[k])
    except Exception:
        pass
    try:
        with open(PHASE) as f:
            for line in f:
                try:
                    h = json.loads(line)
                except Exception:
                    continue
                t = h.get("t", 0)
                if time.time() - t > WINDOW or "freq_off_hz" not in h:
                    continue
                add("ch35 phase", t, h["freq_off_hz"] / CH35_HZ * 1e6)
    except Exception:
        pass
    for label in series:            # coarse decimation of the 1 Hz phase feed
        pts = series[label]
        if len(pts) > MAXPTS:
            step = len(pts) // MAXPTS + 1
            series[label] = pts[::step][-MAXPTS:]


def main():
    backfill()
    while True:
        try:
            with urllib.request.urlopen(API, timeout=5) as r:
                st = json.load(r)
            if "error" in st:
                raise ValueError(st["error"])
        except Exception:
            time.sleep(POLL)
            continue
        now = time.time()
        for s in st.get("sources", []):
            if s.get("kind") == "ClockDriftPpm" and s.get("value") is not None:
                if now - s.get("epoch", 0) < 900:          # live rows only
                    add(s["band"], s["epoch"], s["value"])
            if s.get("kind") == "Presence" and s.get("sats"):
                if now - s.get("epoch", 0) < 3600:
                    add("n:" + s["band"], s["epoch"], float(len(s["sats"])))
        # Live tracker constellations: GPS L1 / E1 / B1I / SBAS all sit INSIDE
        # the 16 Msps capture at 1568.25 MHz, so the 1 Hz tracker sees them
        # continuously — no snapshot radio time needed. band_producer's
        # out-of-band rotation (L5, L2C, G1/G2, E5b, E6) can only open the
        # Pro when live_radio releases it, so those rows refresh rarely; the
        # in-band ones should NEVER go stale while the tracker runs.
        trk = st.get("tracker") or {}
        if trk.get("sats") and now - st.get("epoch", now) < 30:
            counts = {}
            for s in trk["sats"]:
                if s.get("lock_s", 0) > 0:
                    counts[s.get("sys", "?")] = counts.get(s.get("sys", "?"), 0) + 1
            label = {"gps": "GPS L1", "sbas": "SBAS/WAAS",
                     "galileo": "Galileo E1", "beidou": "BeiDou B1I"}
            for sysname, n in counts.items():
                add("n:" + label.get(sysname, sysname) + " (live)", now, float(n))

        # cross-producer consensus: weighted mean over ALL live drift rows,
        # regardless of which producer wrote them
        voters = [(s["value"], max(s.get("sigma") or 0.05, 1e-3))
                  for s in st.get("sources", [])
                  if s.get("kind") == "ClockDriftPpm" and s.get("value") is not None
                  and s.get("band") != "PC clock"      # client of the reference, not a voter
                  and now - s.get("epoch", 0) < 1800]  # rotation-slowed voter window
        cons = None
        if len(voters) >= 2:
            w = [1.0 / (sig * sig) for _, sig in voters]
            cons = sum(v * wi for (v, _), wi in zip(voters, w)) / sum(w)
            add("consensus", now, cons)

        # cross-producer divergence alarm: the ONLY voter-wide integrity check
        # (band_producer can only see its own rows). PC clock is a client of
        # the reference, not a reference — it always "diverges", so it's out.
        xalerts = []
        if cons is not None:
            for s in st.get("sources", []):
                if (s.get("kind") == "ClockDriftPpm" and s.get("value") is not None
                        and s.get("band") != "PC clock"
                        and now - s.get("epoch", 0) < 1800):
                    sig = max(s.get("sigma") or 0.05, 1e-3)
                    z = (s["value"] - cons) / sig
                    if abs(z) > 3:
                        xalerts.append(f"{s['band']}: {s['value']-cons:+.3f} ppm from "
                                       f"cross-producer consensus ({z:+.1f} sigma) — spoof/fault candidate")

        # PC clock drift: the 2-s SPI tick polls are a transfer oscillator.
        # tick rate error over the last hour = tcxo_drift - pc_drift, so
        # pc_drift = consensus - measured tick-rate error. Windowed mean kills
        # the per-read USB jitter (~±30 ppm per sample, ~0.05 ppm over 1 h).
        clk = st.get("clock", {})
        recent = clk.get("recent") or []
        pc_row = None
        if cons is not None and len(recent) >= 60:
            import statistics
            tick_nom = 32.0e6
            rates = [h for _, h in recent if 30e6 < h < 42e6]   # drop transport glitches
            if rates:
                err_ppm = (statistics.median(rates) / tick_nom - 1.0) * 1e6
                pc_ppm = cons - err_ppm
                pc_row = {
                    "band": "PC clock", "name": "macOS system clock vs 1-h tick-rate window",
                    "kind": "ClockDriftPpm", "value": round(pc_ppm, 3), "sigma": 0.05,
                    "ref_hz": None, "epoch": now,
                    "sats": ["via FPGA tick counter"], "anchor": "NTP-disciplined by macOS",
                    "ns_per_s": round(pc_ppm * 1000.0, 1),
                    "m_per_s": round(pc_ppm * 1e-6 * 299792458.0, 2)}
                add("PC clock", now, pc_ppm)
        # ONLY this producer's keys — the server merges the rest
        out = {
            "epoch": now,
            "ttl_s": 120,
            "band_series": {k: v for k, v in sorted(series.items())},
            "alerts": xalerts,
        }
        if cons is not None:
            out["consensus_ppm"] = round(cons, 4)
        if pc_row is not None:
            out["sources"] = [pc_row]
        tmp = STATE + ".series.tmp"
        json.dump(out, open(tmp, "w"), indent=1)
        os.replace(tmp, STATE)
        time.sleep(POLL)


if __name__ == "__main__":
    main()
