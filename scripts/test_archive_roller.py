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
        with open(os.path.join(obs, "sky_history.jsonl"), "w") as f:
            f.write(json.dumps({"t": now, "sats": [
                {"sys": "glonass", "prn": 5, "cls": "predicted",
                 "az_deg": 10.0, "el_deg": 25.0},
                {"sys": "gps", "prn": 30, "cls": "tracked", "cn0": 44.0,
                 "lock_s": 300.0, "az_deg": 200.0, "el_deg": 60.0}]}) + "\n")

        # --- pass 1: backfill ----------------------------------------------
        parsed, skipped, written = roller.roll()
        check("backfill parses good lines", parsed == 9, f"parsed={parsed}")
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
        check("satellite rows", nrows("satellite") == 3,
              "(1 telemetry + 2 sky)")
        check("discipline rows", nrows("discipline") == 1)
        check("telemetry rows", nrows("telemetry") == 1)
        check("presence rows", nrows("presence") == 1)

        if roller.HAVE_PARQUET:
            t = roller.pq.read_table(os.path.join(adir, "satellite" + ext))
            by_sat = {(r["sys"], r["prn"]): r for r in t.to_pylist()}
            row = by_sat[("gps", 26)]       # the telemetry-sourced row
            check("reserved columns exist and are null",
                  row["az_deg"] is None and row["el_deg"] is None
                  and row["residual_m"] is None)
            check("sky cls lands in satellite rows",
                  by_sat[("glonass", 5)]["cls"] == "predicted"
                  and by_sat[("gps", 30)]["cls"] == "tracked")
            check("telemetry-sourced row keeps cls null", row["cls"] is None)
            check("impossible rho_m archived as NULL (round-14)",
                  row["rho_m"] is None, "fixture carries rho_m=-1.0")
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
        check("full re-read reparses all", parsed3 == 9, f"parsed={parsed3}")
        check("full re-read does not duplicate",
              nrows("clock_drift") == 4 and nrows("phase") == 3
              and nrows("loop_log") == 2 and nrows("satellite") == 3)

        # --- incremental append ------------------------------------------------
        with open(os.path.join(obs, "phase_history.jsonl"), "a") as f:
            f.write(json.dumps({"t": now + 5, "disp_mm": 9.0,
                                "freq_off_hz": -282.0, "sigma_mm": 1.0,
                                "lock": False}) + "\n")
        roller.roll()
        check("incremental append lands", nrows("phase") == 4)

        # --- schema evolution: satellite partition written BEFORE cls existed ---
        if roller.HAVE_PARQUET:
            old_epoch = now + 86400.0
            odir = os.path.join(obs, "archive", roller.date_of(old_epoch))
            os.makedirs(odir, exist_ok=True)
            old_row = {"epoch": old_epoch, "sys": "glonass", "prn": 9,
                       "cn0": None, "doppler_hz": None, "lock_s": None,
                       "rho_m": None, "t_tx": None, "ppm": None,
                       "az_deg": 5.0, "el_deg": 6.0, "residual_m": None}
            # no cls key at all: pyarrow infers a schema without the column,
            # exactly like the live partitions that predate the fix
            roller.pq.write_table(roller.pa.Table.from_pylist([old_row]),
                                  os.path.join(odir, "satellite" + ext),
                                  compression="zstd")
            with open(os.path.join(obs, "sky_history.jsonl"), "a") as f:
                f.write(json.dumps({"t": old_epoch, "sats": [
                    {"sys": "glonass", "prn": 10, "cls": "predicted",
                     "az_deg": 1.0, "el_deg": 2.0}]}) + "\n")
            roller.roll()
            orows = roller.pq.read_table(
                os.path.join(odir, "satellite" + ext)).to_pylist()
            by_prn = {r["prn"]: r for r in orows}
            check("pre-cls partition merges without error", len(orows) == 2)
            check("pre-cls row reads back cls NULL", by_prn[9]["cls"] is None)
            check("new row in pre-cls partition carries cls",
                  by_prn[10]["cls"] == "predicted")

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
