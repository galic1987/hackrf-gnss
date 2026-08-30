import os
import sys
import pytest

sys.path.insert(0, os.path.dirname(__file__))

from tdc_density_cal import (
    parse_thermo_bytes,
    decode_thermometer,
    parse_jsonl_event,
    compute_code_density,
)

def test_parse_thermo_bytes():
    taps = parse_thermo_bytes("0x01,0x00,0x00,0x00,0x00,0x00")
    assert len(taps) == 48
    assert taps[0] == 1
    assert taps[1] == 0
    assert sum(taps) == 1

def test_decode_thermometer():
    taps = [1] * 16 + [0] * 32
    assert decode_thermometer(taps) == 16

    sat = [1] * 48
    assert decode_thermometer(sat) == 48

    zeros = [0] * 48
    assert decode_thermometer(zeros) == 0

def test_parse_jsonl_event():
    line1 = '{"t": 123.456, "bytes": "0xff,0x00,0x00,0x00,0x00,0x00"}'
    ev1 = parse_jsonl_event(line1)
    assert ev1 is not None
    assert ev1["t"] == 123.456
    assert ev1["k"] == 8

    line2 = '{"seq": 5, "thermo": "010000000000"}'
    ev2 = parse_jsonl_event(line2)
    assert ev2 is not None
    assert ev2["k"] == 1

def test_compute_code_density():
    ks = [k for k in range(49)] * 10  # 10 counts in every bin 0..48
    res = compute_code_density(ks, num_taps=48, t_clock_ns=25.0)
    assert res["total_events"] == 490
    assert len(res["bin_widths_ps"]) == 49
    assert len(res["calibrated_lut_ps"]) == 49
    assert res["calibrated_lut_ps"][0] == 0.0
    assert sum(res["bin_widths_ps"]) == pytest.approx(25000.0, rel=1e-3)
