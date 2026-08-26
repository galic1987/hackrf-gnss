#!/usr/bin/env python3
"""Archive roller: JSONL histories + telemetry snapshots -> partitioned Parquet.

Reads the append-only station logs (band_drift_history.jsonl,
phase_history.jsonl, crate clock_loop_log.jsonl / fused_loop_log.jsonl,
and the collector's telemetry_log.jsonl) and rolls them into
observations/archive/YYYY-MM-DD/<stream>.parquet (UTC day partitions).

- Atomic: each partition is written to a tmp file then os.replace()d.
- Idempotent: rows are deduped on their natural key against the existing
  partition before rewrite; byte offsets in _roller_state.json are only an
  incremental-read optimization, never the correctness mechanism.
- pyarrow if available; otherwise csv.gz partitions (see archive README
  for the tradeoff). No new dependencies; macOS system python3.9.

Read-only on every state/log file. Never touches the radios or any
running process.

Usage:
  archive_roller.py            # one incremental pass (backfill on 1st run)
  archive_roller.py --full     # ignore offsets, re-read everything
  archive_roller.py --loop 300 # roll every N seconds forever
"""
import csv
import gzip
import json
import os
import sys
import time

OBS = os.environ.get("HACKRF_GNSS_OBS", "/Volumes/Radiator 8TB/gnss/observations")
CRATE = os.environ.get("HACKRF_GNSS_CRATE", "/Volumes/Radiator 8TB/gnss/hackrf_gnss")
ARCHIVE = os.path.join(OBS, "archive")
OFFSETS_PATH = os.path.join(ARCHIVE, "_roller_state.json")

try:
    import pyarrow as pa
    import pyarrow.parquet as pq
    HAVE_PARQUET = True
except ImportError:
    pa = pq = None
    HAVE_PARQUET = False

EXT = ".parquet" if HAVE_PARQUET else ".csv.gz"

# --- stream schemas -------------------------------------------------------
# Every stream carries `epoch` (float unix seconds, UTC). Reserved/placeholder
# columns (az/el, residual, temp, gain, tdc stream) exist from day one so
# future producers fill columns instead of migrating schemas.
# cls (satellite) was added after archives already existed; no migration is
# needed — load_partition reads old partitions as plain dicts and
# write_partition re-projects every merged row via r.get(n) against the
# current schema, so pre-cls rows read back as NULL and the next rewrite of
# that partition adds the column.

FIELDS = {
    "satellite": [("epoch", "f"), ("sys", "s"), ("prn", "i"), ("cn0", "f"),
                  ("doppler_hz", "f"), ("lock_s", "f"), ("rho_m", "f"),
                  ("t_tx", "f"), ("ppm", "f"),
                  ("az_deg", "f"), ("el_deg", "f"), ("residual_m", "f"),
                  ("cls", "s")],
    "clock_drift": [("epoch", "f"), ("source", "s"), ("band", "s"),
                    ("ppm", "f"), ("sigma_ppm", "f"), ("anchor", "s"),
                    ("n_sats", "i"), ("sats", "s"), ("kind", "s"),
                    ("producer", "s")],
    "discipline": [("epoch", "f"), ("correction_ppm", "f"), ("residual_ppm", "f"),
                   ("clamp_ppm", "f"), ("step_limit_ppm", "f"), ("sign", "f"),
                   ("stalled", "b"), ("waas_locked", "i"), ("note", "s")],
    "phase": [("epoch", "f"), ("disp_mm", "f"), ("freq_off_hz", "f"),
              ("sigma_mm", "f"), ("lock", "b")],
    "loop_log": [("epoch", "f"), ("loop", "s"), ("measured_ppm", "f"),
                 ("delta_ppm", "f"), ("correction_ppm", "f"), ("n_bursts", "i"),
                 ("n_detected", "i"), ("n_attributed", "i"), ("sats", "s")],
    "telemetry": [("epoch", "f"), ("tick_hz", "f"), ("n_sats_tracked", "i"),
                  ("lat", "f"), ("lon", "f"), ("alt_km", "f"), ("gdop", "f"),
                  ("pos_n_sat", "i"), ("files_present", "s"),
                  ("temp_c", "f"), ("gain_db", "f"), ("radio", "s")],
    "presence": [("epoch", "f"), ("band", "s"), ("n_sats", "i"),
                 ("sats", "s"), ("anchor", "s")],
    "position": [("epoch", "f"), ("lat", "f"), ("lon", "f"), ("alt_km", "f"),
                 ("mode", "s"), ("gate", "s"), ("gdop", "f"), ("n_sats", "i"),
                 ("isx_km", "f")],
    "tdc": [("epoch", "f"), ("seq", "i"), ("popcount", "i"), ("thermo", "s")],
}

STREAMS = list(FIELDS)

# natural dedupe keys (field names) per stream
KEYS = {
    "satellite": ("epoch", "sys", "prn"),
    "clock_drift": ("epoch", "source", "producer"),
    "discipline": ("epoch",),
    "phase": ("epoch",),
    "loop_log": ("epoch", "loop"),
    "telemetry": ("epoch",),
    "presence": ("epoch", "band"),
    "position": ("epoch",),
    "tdc": ("epoch", "seq"),
}

# wide-row keys in band_drift_history.jsonl that are ppm drift values
DRIFT_KEYS = {"waas": "L1 / WAAS", "ATSC ch23": "ATSC ch23", "ATSC ch35": "ATSC ch35"}


def _schema(stream):
    if not HAVE_PARQUET:
        return None
    tmap = {"f": pa.float64(), "i": pa.int64(), "s": pa.string(), "b": pa.bool_()}
    return pa.schema([(name, tmap[t]) for name, t in FIELDS[stream]])


def _epoch_key(epoch):
    return round(float(epoch or 0.0), 3)


def key_of(stream, row):
    return tuple(_epoch_key(row[f]) if f == "epoch" else row.get(f)
                 for f in KEYS[stream])


def date_of(epoch):
    return time.strftime("%Y-%m-%d", time.gmtime(float(epoch)))


# --- parsers: one source line -> {stream: [rows]} --------------------------

def parse_band_drift(d):
    t = d.get("t")
    if t is None:
        return {}
    rows = []
    for k, label in DRIFT_KEYS.items():
        v = d.get(k)
        if v is None:
            continue
        rows.append({"epoch": t, "source": label, "band": label, "ppm": v,
                     "kind": "ClockDriftPpm", "producer": "band_drift_history"})
    return {"clock_drift": rows}


def parse_phase(d):
    t = d.get("t")
    if t is None:
        return {}
    return {"phase": [{"epoch": t, "disp_mm": d.get("disp_mm"),
                       "freq_off_hz": d.get("freq_off_hz"),
                       "sigma_mm": d.get("sigma_mm"), "lock": d.get("lock")}]}


def parse_loop(d, loop_name):
    t = d.get("t")
    if t is None:
        return {}
    sats = d.get("sats") or []
    return {"loop_log": [{"epoch": t, "loop": loop_name,
                          "measured_ppm": d.get("measured_ppm"),
                          "delta_ppm": d.get("delta_ppm"),
                          "correction_ppm": d.get("correction_ppm"),
                          "n_bursts": d.get("n_bursts"),
                          "n_detected": d.get("n_detected"),
                          "n_attributed": d.get("n_attributed"),
                          "sats": ",".join(str(s) for s in sats)}]}


def parse_telemetry(d):
    """One collector snapshot -> rows for several streams."""
    t = d.get("epoch")
    if t is None:
        return {}
    out = {}
    sats = d.get("sats") or []
    if sats:
        def rho_phys(v):
            # rho_m from an unanchored channel is stream-offset dominated
            # (up to a week of TOW vs stream-time — ±1.8e14 m observed),
            # not a pseudorange. Archive only physical-class values
            # (GNSS geometric range plus anchored-clock margin); the rest
            # reads NULL instead of poisoning analysis (round-14: ~44k
            # impossible rows/day were landing in the satellite stream).
            return v if isinstance(v, (int, float)) and 1.5e7 <= v <= 5.0e7 else None
        out["satellite"] = [
            {"epoch": s.get("epoch", t), "sys": s.get("sys"), "prn": s.get("prn"),
             "cn0": s.get("cn0_proxy", s.get("cn0")), "doppler_hz": s.get("doppler_hz"),
             "lock_s": s.get("lock_s"), "rho_m": rho_phys(s.get("rho_m")),
             "t_tx": s.get("t_tx"), "ppm": s.get("ppm"),
             "az_deg": None, "el_deg": None, "residual_m": None, "cls": None}
            for s in sats]
    disc = d.get("discipline")
    if disc:
        out["discipline"] = [
            {"epoch": disc.get("epoch", t), "correction_ppm": disc.get("correction_ppm"),
             "residual_ppm": disc.get("residual_ppm"), "clamp_ppm": disc.get("clamp_ppm"),
             "step_limit_ppm": disc.get("step_limit_ppm"), "sign": disc.get("sign"),
             "stalled": disc.get("stalled"), "waas_locked": disc.get("waas_locked"),
             "note": disc.get("note")}]
    drift, presence = [], []
    for s in d.get("sources") or []:
        if not isinstance(s, dict):
            continue
        sats_s = ",".join(str(x) for x in (s.get("sats") or []))
        if s.get("kind") == "ClockDriftPpm" and s.get("value") is not None:
            drift.append({"epoch": s.get("epoch", t), "source": s.get("band"),
                          "band": s.get("band"), "ppm": s.get("value"),
                          "sigma_ppm": s.get("sigma"), "anchor": s.get("anchor"),
                          "n_sats": len(s.get("sats") or []), "sats": sats_s,
                          "kind": s.get("kind"), "producer": s.get("producer")})
        elif s.get("kind") == "Presence":
            presence.append({"epoch": s.get("epoch", t), "band": s.get("band"),
                             "n_sats": len(s.get("sats") or []), "sats": sats_s,
                             "anchor": s.get("anchor")})
    if drift:
        out["clock_drift"] = drift
    if presence:
        out["presence"] = presence
    ph = d.get("phase")
    if ph:
        out["phase"] = [{"epoch": ph.get("epoch", t), "disp_mm": ph.get("disp_mm"),
                         "freq_off_hz": ph.get("freq_off_hz"),
                         "sigma_mm": ph.get("sigma_mm"), "lock": ph.get("lock")}]
    pos = d.get("position") or {}
    out["telemetry"] = [
        {"epoch": t, "tick_hz": d.get("tick_hz"), "n_sats_tracked": len(sats),
         "lat": pos.get("lat"), "lon": pos.get("lon"), "alt_km": pos.get("alt_km"),
         "gdop": pos.get("gdop"), "pos_n_sat": pos.get("n_sat"),
         "files_present": ",".join(d.get("files") or []),
         "temp_c": None, "gain_db": None, "radio": None}]
    return out


def parse_sky(d):
    """One sky_producer cycle line -> satellite-stream rows with the reserved
    az_deg/el_deg columns filled, plus the sky classification (cls:
    predicted/tracked/unexpected/absent/below) from sky_history.jsonl."""
    t = d.get("t")
    if t is None:
        return {}
    rows = []
    for s in d.get("sats") or []:
        if not isinstance(s, dict) or s.get("prn") is None:
            continue
        rows.append({"epoch": t, "sys": s.get("sys"), "prn": s.get("prn"),
                     "cn0": s.get("cn0"), "doppler_hz": s.get("doppler_hz"),
                     "lock_s": s.get("lock_s"), "rho_m": s.get("rho_m"),
                     "t_tx": s.get("t_tx"), "ppm": s.get("ppm"),
                     "az_deg": s.get("az_deg"), "el_deg": s.get("el_deg"),
                     "residual_m": None, "cls": s.get("cls")})
    return {"satellite": rows} if rows else {}


def parse_position(d):
    """One position_watch history line -> position-stream row."""
    t = d.get("epoch")
    if t is None:
        return {}
    return {"position": [{"epoch": t, "lat": d.get("lat"), "lon": d.get("lon"),
                          "alt_km": d.get("alt_km"), "mode": d.get("mode"),
                          "gate": d.get("gate"), "gdop": d.get("gdop"),
                          "n_sats": d.get("n_sats"),
                          "isx_km": d.get("isx_km")}]}


# (path, parser) — parser takes the decoded JSON object
SOURCES = [
    (os.path.join(OBS, "band_drift_history.jsonl"), parse_band_drift),
    (os.path.join(OBS, "phase_history.jsonl"), parse_phase),
    (os.path.join(CRATE, "clock_loop_log.jsonl"),
     lambda d: parse_loop(d, "clock")),
    (os.path.join(CRATE, "fused_loop_log.jsonl"),
     lambda d: parse_loop(d, "fused")),
    (os.path.join(OBS, "telemetry_log.jsonl"), parse_telemetry),
    (os.path.join(OBS, "sky_history.jsonl"), parse_sky),
    (os.path.join(OBS, "position_history.jsonl"), parse_position),
]


# --- partition IO -----------------------------------------------------------

def load_partition(path, stream):
    """Existing partition rows as {key: row}; {} if unreadable/missing."""
    if not os.path.exists(path):
        return {}
    try:
        if HAVE_PARQUET:
            rows = pq.read_table(path).to_pylist()
        else:
            with gzip.open(path, "rt") as f:
                rows = list(csv.DictReader(f))
        return {key_of(stream, r): r for r in rows}
    except Exception as e:
        print(f"warn: unreadable partition {path}: {e}; rewriting from scratch",
              file=sys.stderr)
        return {}


def write_partition(path, stream, rows):
    names = [n for n, _ in FIELDS[stream]]
    ordered = sorted(rows, key=lambda r: (float(r.get("epoch") or 0.0),
                                          str(key_of(stream, r))))
    tmp = f"{path}.tmp.{os.getpid()}"
    if HAVE_PARQUET:
        table = pa.Table.from_pylist(
            [{n: r.get(n) for n in names} for r in ordered], schema=_schema(stream))
        pq.write_table(table, tmp, compression="zstd")
    else:
        with gzip.open(tmp, "wt", newline="") as f:
            w = csv.DictWriter(f, fieldnames=names)
            w.writeheader()
            for r in ordered:
                w.writerow({n: r.get(n) for n in names})
    os.replace(tmp, path)


# --- offsets -----------------------------------------------------------------

def load_offsets():
    try:
        with open(OFFSETS_PATH) as f:
            return json.load(f)
    except Exception:
        return {}


def save_offsets(offsets):
    tmp = f"{OFFSETS_PATH}.tmp.{os.getpid()}"
    with open(tmp, "w") as f:
        json.dump(offsets, f, indent=1)
    os.replace(tmp, OFFSETS_PATH)


def read_new_lines(path, offset):
    """Return (complete-lines, new-offset); resets to 0 if the file shrank."""
    try:
        size = os.path.getsize(path)
    except OSError:
        return [], offset            # source down/missing — not an error
    if size < offset:
        offset = 0                   # rotated/truncated: re-read
    if size == offset:
        return [], offset
    with open(path, "rb") as f:
        f.seek(offset)
        data = f.read()
    lines = data.split(b"\n")
    if lines and lines[-1] == b"":
        lines.pop()
        return lines, offset + len(data)
    # last line incomplete (producer mid-append) — hold it for next pass
    tail = lines.pop() if lines else b""
    return lines, offset + len(data) - len(tail)


# --- main pass -----------------------------------------------------------------

def roll(full=False):
    os.makedirs(ARCHIVE, exist_ok=True)
    offsets = {} if full else load_offsets()
    new = {s: {} for s in STREAMS}   # stream -> {date -> {key: row}}
    parsed = skipped = 0
    for path, parser in SOURCES:
        off = offsets.get(path, 0)
        lines, new_off = read_new_lines(path, off)
        offsets[path] = new_off
        for raw in lines:
            try:
                d = json.loads(raw)
            except Exception:
                skipped += 1
                continue
            try:
                out = parser(d)
            except Exception:
                skipped += 1
                continue
            parsed += 1
            for stream, rows in out.items():
                for r in rows:
                    if r.get("epoch") is None:
                        continue
                    new[stream].setdefault(date_of(r["epoch"]), {})[
                        key_of(stream, r)] = r
    written = {}
    for stream in STREAMS:
        by_date = new[stream]
        if not by_date:
            continue
        for date, rows in sorted(by_date.items()):
            d = os.path.join(ARCHIVE, date)
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, stream + EXT)
            merged = load_partition(path, stream)
            before = len(merged)
            merged.update(rows)
            write_partition(path, stream, list(merged.values()))
            written[f"{date}/{stream}"] = (len(rows), before, len(merged))
    save_offsets(offsets)
    return parsed, skipped, written


def main():
    args = sys.argv[1:]
    full = "--full" in args
    loop = None
    if "--loop" in args:
        loop = float(args[args.index("--loop") + 1])
    fmt = "parquet" if HAVE_PARQUET else "csv.gz (pyarrow missing — fallback)"
    while True:
        t0 = time.time()
        parsed, skipped, written = roll(full=full)
        full = False
        total_new = sum(v[0] for v in written.values())
        print(f"[{time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}] "
              f"format={fmt} parsed={parsed} skipped={skipped} "
              f"new_rows={total_new} partitions_touched={len(written)}")
        for part, (nnew, before, after) in sorted(written.items()):
            print(f"  {part}: +{nnew} (partition {before} -> {after})")
        if loop is None:
            break
        time.sleep(max(5.0, loop - (time.time() - t0)))


if __name__ == "__main__":
    main()
