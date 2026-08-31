import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from tdc_pps_analyze import load, segment_events


def event(t, k, status):
    return {"t": t, "k": k, "popcount": k, "status_toggle": status}


def test_repeated_status_breaks_and_excludes_boundary_step():
    events = [
        event(0.0, 10, 0),
        event(1.0, 11, 1),
        event(2.0, 40, 1),  # repeated: must not form 11 -> 40 step
        event(3.0, 12, 0),
        event(4.0, 13, 1),
    ]
    segments = segment_events(events)
    assert [[row["k"] for row in segment] for segment in segments] == [
        [10, 11], [12, 13]
    ]


def test_missing_status_invalid_code_and_bad_cadence_split():
    events = [
        event(0.0, 10, 0),
        event(1.0, 11, 1),
        event(2.0, 12, None),
        event(3.0, 13, 0),
        event(5.0, 14, 1),
        event(6.0, None, 0),
        event(7.0, 15, 1),
    ]
    segments = segment_events(events)
    assert [[row["k"] for row in segment] for segment in segments] == [
        [10, 11], [13], [15]
    ]


def test_unrepresentably_large_timestamp_is_rejected(tmp_path):
    path = tmp_path / "huge.jsonl"
    path.write_text(json.dumps({
        "t": 10 ** 1000,
        "bytes": "ff,00,00,00,00,00",
        "reg31": "0x01",
    }) + "\n")
    events, stats = load(path)
    assert events == []
    assert stats["invalid"] == 1
