#!/usr/bin/env python3
"""Sandboxed tests for archive_roller + telemetry_collector.

Builds a temp OBS/CRATE tree, points HACKRF_GNSS_OBS / HACKRF_GNSS_CRATE at
it, and checks: backfill counts, idempotent re-run, parquet readback vs
source counts, and collector survival with missing/corrupt/expired state
files. Never touches the live observations dir. Plain asserts, no pytest.

  python3 scripts/test_archive_roller.py
"""
import importlib
import json
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FAILURES = []


def check(name, cond, detail=""):
    print(f"{'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILURES.append(name)


def main():
    tmp = tempfile.mkdtemp(prefix="archive_test_")
    obs, crate = os.path.join(tmp, "obs"), os.path.join(tmp, "crate")
    os.makedirs(obs)
    os.makedirs(crate)
    os.environ["HACKRF_GNSS_OBS"] = obs
    os.environ["HACKRF_GNSS_CRATE"] = crate
    try:
        import archive_roller as roller
        import telemetry_collector as collector
        importlib.reload(collector)          # pick up the env override
        importlib.reload(roller)

        now = 1787600000.0                   # fixed fake epoch (2026-08-24)

        # --- fixture sources ------------------------------------------------
        with open(os.path.join(obs, "band_drift_history.jsonl"), "w") as f:
            f.write(json.dumps({"t": now, "waas": -0.42, "ATSC ch35": -0.49,
                                "l1_std": 16.7}) + "\n")
            f.write(json.dumps({"t": now + 30, "ATSC ch23": -0.36}) + "\n")
            f.write("not json\n")
        with open(os.path.join(obs, "phase_history.jsonl"), "w") as f:
            for i in range(3):
                f.write(json.dumps({"t": now + i, "disp_mm": float(i),
                                    "freq_off_hz": -282.0, "sigma_mm": None,
                                    "lock": True}) + "\n")
        with open(os.path.join(crate, "clock_loop_log.jsonl"), "w") as f:
            f.write(json.dumps({"t": now, "measured_ppm": -0.8,
                                "delta_ppm": 0.8, "correction_ppm": 26.8,
                                "n_bursts": 3, "sats": ["IRIDIUM 107"]}) + "\n")
        with open(os.path.join(crate, "fused_loop_log.jsonl"), "w") as f:
            f.write(json.dumps({"t": now, "measured_ppm": -0.4,
                                "delta_ppm": 0.4, "correction_ppm": 26.4,
                                "n_detected": 101, "n_attributed": 7,
                                "sats": ["IRIDIUM 120"]}) + "\n")
        with open(os.path.join(obs, "telemetry_log.jsonl"), "w") as f:
            f.write(json.dumps({
                "epoch": now, "files": ["tracker", "tick"],
                "sats": [{"sys": "gps", "prn": 26, "doppler_hz": 2041.4,
                          "cn0_proxy": 42.6, "lock_s": 574.0, "epoch": now,
                          "rho_m": -1.0, "t_tx": 163860.0, "ppm": 1.296}],
                "discipline": {"correction_ppm": -0.45, "epoch": now,
                               "stalled": False, "waas_locked": 0,
                               "note": "holding"},
                "tick_hz": 31999713.9,
                "sources": [{"band": "ATSC ch35", "kind": "ClockDriftPpm",
                             "value": -0.40, "sigma": 0.005, "epoch": now,
                             "sats": ["GPS-disciplined Tx"], "anchor": "CLKIN",
                             "producer": "phase"},
                            {"band": "GPS L5", "kind": "Presence",
                             "value": None, "epoch": now, "sats": ["~PRN 25"],
                             "anchor": "Pro snapshot", "producer": "band"}]}) + "\n")

        # --- pass 1: backfill ----------------------------------------------
        parsed, skipped, written = roller.roll()
        check("backfill parses good lines", parsed == 8, f"parsed={parsed}")
        check("backfill skips bad lines", skipped == 1, f"skipped={skipped}")

        date = "2026-08-24"
        adir = os.path.join(obs, "archive", date)
        ext = roller.EXT

        def nrows(stream):
            p = os.path.join(adir, stream + ext)
            if roller.HAVE_PARQUET:
                return roller.pq.read_table(p).num_rows
            return sum(1 for _ in roller.gzip.open(p, "rt")) - 1

        check("clock_drift rows", nrows("clock_drift") == 4,
              "(3 history + 1 telemetry source)")
        check("phase rows", nrows("phase") == 3)
        check("loop_log rows", nrows("loop_log") == 2)
        check("satellite rows", nrows("satellite") == 1)
        check("discipline rows", nrows("discipline") == 1)
        check("telemetry rows", nrows("telemetry") == 1)
        check("presence rows", nrows("presence") == 1)

        if roller.HAVE_PARQUET:
            t = roller.pq.read_table(os.path.join(adir, "satellite" + ext))
            row = t.to_pylist()[0]
            check("reserved columns exist and are null",
                  row["az_deg"] is None and row["el_deg"] is None
                  and row["residual_m"] is None)
            tt = roller.pq.read_table(os.path.join(adir, "telemetry" + ext))
            trow = tt.to_pylist()[0]
            check("telemetry reserved temp/gain/radio null",
                  trow["temp_c"] is None and trow["gain_db"] is None
                  and trow["radio"] is None)
            check("telemetry tick_hz landed",
                  abs(trow["tick_hz"] - 31999713.9) < 1.0)

        # --- pass 2: idempotency ---------------------------------------------
        parsed2, _, written2 = roller.roll()
        check("second run parses nothing new", parsed2 == 0,
              f"parsed={parsed2}")
        check("second run touches no partitions", written2 == {})

        # --- pass 3: --full re-read still dedupes ------------------------------
        parsed3, _, _ = roller.roll(full=True)
        check("full re-read reparses all", parsed3 == 8, f"parsed={parsed3}")
        check("full re-read does not duplicate",
              nrows("clock_drift") == 4 and nrows("phase") == 3
              and nrows("loop_log") == 2 and nrows("satellite") == 1)

        # --- incremental append ------------------------------------------------
        with open(os.path.join(obs, "phase_history.jsonl"), "a") as f:
            f.write(json.dumps({"t": now + 5, "disp_mm": 9.0,
                                "freq_off_hz": -282.0, "sigma_mm": 1.0,
                                "lock": False}) + "\n")
        roller.roll()
        check("incremental append lands", nrows("phase") == 4)

        # --- collector: missing / corrupt / expired state files ----------------
        # obs has NO state.*.json at all -> row with empty files, no crash
        row = collector.snapshot(now)
        check("collector survives all producers down", row["files"] == [])

        with open(os.path.join(obs, "state.tracker.json"), "w") as f:
            f.write("{corrupt")                       # mid-write / corrupt
        row = collector.snapshot(now)
        check("collector survives corrupt file", row["files"] == [])

        with open(os.path.join(obs, "state.tracker.json"), "w") as f:
            json.dump({"epoch": now, "ttl_s": 30,
                       "tracker": {"sats": [{"sys": "gps", "prn": 1}]},
                       "discipline": {"correction_ppm": 1.0}}, f)
        row = collector.snapshot(now)
        check("collector reads live tracker",
              row["files"] == ["tracker"] and len(row["sats"]) == 1)

        stale = {"epoch": now - 9999, "ttl_s": 30,
                 "position": {"lat": 39.0, "lon": -77.6}}
        with open(os.path.join(obs, "state.position.json"), "w") as f:
            json.dump(stale, f)
        row = collector.snapshot(now)
        check("collector drops ttl-expired file (tombstone)",
              "position" not in row["files"] and "position" not in row)

        line = json.dumps(collector.snapshot(now))     # must be serializable
        check("snapshot serializes", json.loads(line)["epoch"] == now)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print(f"\n{len(FAILURES)} failure(s)" if FAILURES else "\nall tests passed")
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
