import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from tdc_ts_analyze import COUNTER_MODULUS, analyze


def write_capture(path, *, hz=32_000_000, n=12, gap_at=None,
                  mismatch_at=None, complete_metadata=True):
    config = {
        "kind": "config",
        "schema": "tdc-ts-latch-capture-v2",
        "capture_protocol": "double-read-equal-v1",
        "adclk_hz": hz,
        "serial": "0000000000000000645061de252d6613",
        "slot": 1,
        "build_id": "0x03ba",
        "clock_source": "Bodnar 10 MHz split",
        "clock_source_readback": "external selected",
        "trigger_source": "Bodnar 1 PPS split to P28.16",
        "counter_bits": 48,
    }
    if not complete_metadata:
        config.pop("build_id")
    t = 1_000.0
    latch = COUNTER_MODULUS - hz // 2
    rows = [config, {"kind": "event", "seq": 0, "t": t,
                     "latch_read_1": latch, "latch_read_2": latch,
                     "latch": latch}]
    for i in range(1, n):
        step = 2 if gap_at == i else 1
        t += step
        delta = step * hz
        latch = (latch + delta) % COUNTER_MODULUS
        stored = delta + (1 if mismatch_at == i else 0)
        rows.append({"kind": "event", "seq": i, "t": t,
                     "latch_read_1": latch, "latch_read_2": latch,
                     "latch": latch, "delta": stored})
    rows.append({"kind": "stop", "n": n, "last_seq": n - 1})
    with open(path, "w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row) + "\n")


def test_modular_latch_deltas_and_claim_grade_ratio(tmp_path):
    path = tmp_path / "good.jsonl"
    write_capture(path)
    result = analyze(str(path))
    assert result["relative_ratio_valid"] is True, result["gate_fails"]
    assert result["valid_intervals"] == 11
    assert result["mean_ticks_per_observed_pps"] == 32_000_000
    assert result["conditional_adclk_ppm_if_pps_exact"] == 0.0


def test_irregular_cadence_is_excluded_and_blocks_verdict(tmp_path):
    path = tmp_path / "gap.jsonl"
    write_capture(path, gap_at=6)
    result = analyze(str(path))
    assert result["irregular_intervals"] == 1
    assert result["valid_intervals"] == 10
    assert result["relative_ratio_valid"] is False
    assert any("irregular/missed" in s for s in result["gate_fails"])


def test_stored_delta_mismatch_blocks_verdict(tmp_path):
    path = tmp_path / "mismatch.jsonl"
    write_capture(path, mismatch_at=4)
    result = analyze(str(path))
    assert result["stored_delta_mismatches"] == 1
    assert result["relative_ratio_valid"] is False


def test_missing_provenance_remains_exploratory(tmp_path):
    path = tmp_path / "old.jsonl"
    write_capture(path, complete_metadata=False)
    result = analyze(str(path))
    assert result["relative_ratio_valid"] is False
    assert any("build_id" in s for s in result["gate_fails"])


def test_sample_rate_is_not_clock_telemetry(tmp_path):
    path = tmp_path / "sample_rate.jsonl"
    path.write_text(
        json.dumps({"metadata": "legacy", "sample_rate": 16_000_000}) + "\n"
        + json.dumps({"t": 1.0, "latch": 10}) + "\n"
        + json.dumps({"t": 2.0, "latch": 32_000_010}) + "\n"
    )
    result = analyze(str(path))
    assert "error" in result and "sample_rate*2" in result["error"]


def test_cli_clock_mismatch_is_refused(tmp_path):
    path = tmp_path / "good.jsonl"
    write_capture(path)
    result = analyze(str(path), adclk_hz=40_000_000)
    assert "error" in result and "mismatch" in result["error"]


def test_cli_clock_cannot_launder_invalid_metadata(tmp_path):
    path = tmp_path / "bad_clock.jsonl"
    write_capture(path)
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows[0]["adclk_hz"] = "bogus"
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    result = analyze(str(path), adclk_hz=32_000_000)
    assert result["relative_ratio_valid"] is False
    assert any("metadata" in reason and "adclk_hz" in reason
               for reason in result["gate_fails"])


def test_measurement_integers_are_not_coerced(tmp_path):
    path = tmp_path / "fractional_seq.jsonl"
    write_capture(path)
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows[4]["seq"] = 3.0
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    result = analyze(str(path))
    assert result["relative_ratio_valid"] is False
    assert any("malformed" in reason for reason in result["gate_fails"])


def test_regular_non_pps_period_is_rejected(tmp_path):
    path = tmp_path / "not_pps.jsonl"
    write_capture(path, hz=32_000_000)
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    latch = rows[1]["latch"]
    t = rows[1]["t"]
    step = int(1.4 * 32_000_000)
    for row in rows[2:-1]:
        t += 1.4
        latch = (latch + step) % COUNTER_MODULUS
        row.update({"t": t, "latch_read_1": latch,
                    "latch_read_2": latch, "latch": latch,
                    "delta": step})
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    result = analyze(str(path))
    assert result["relative_ratio_valid"] is False
    assert result["valid_intervals"] == 0


def test_events_after_stop_are_rejected(tmp_path):
    path = tmp_path / "outside.jsonl"
    write_capture(path)
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    event = dict(rows[-2])
    event["seq"] += 1
    event["t"] += 1.0
    event["latch"] = (event["latch"] + 32_000_000) % COUNTER_MODULUS
    event["latch_read_1"] = event["latch"]
    event["latch_read_2"] = event["latch"]
    rows.append(event)
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    result = analyze(str(path))
    assert result["relative_ratio_valid"] is False
    assert result["outside_segment_rows"] == 1


def test_unequal_double_read_is_rejected(tmp_path):
    path = tmp_path / "torn.jsonl"
    write_capture(path)
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows[5]["latch_read_2"] += 1
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    result = analyze(str(path))
    assert result["relative_ratio_valid"] is False
    assert result["incoherent_reads"] >= 1


def test_unrepresentably_large_numbers_fail_closed(tmp_path):
    path = tmp_path / "huge.jsonl"
    write_capture(path)
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows[3]["t"] = 10 ** 1000
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    result = analyze(str(path))
    assert result["relative_ratio_valid"] is False
    assert any("malformed" in reason for reason in result["gate_fails"])
    assert "error" in analyze(str(path), adclk_hz=10 ** 1000)
