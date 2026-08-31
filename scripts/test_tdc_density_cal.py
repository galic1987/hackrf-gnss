import json
import os
import random
import subprocess
import sys
import pytest

sys.path.insert(0, os.path.dirname(__file__))

from tdc_density_cal import (
    parse_thermo_bytes,
    decode_thermometer,
    parse_jsonl_event,
    compute_code_density,
)

SCRIPT = os.path.join(os.path.dirname(__file__), "tdc_density_cal.py")


def run_cli(*args):
    return subprocess.run(
        [sys.executable, SCRIPT] + list(args), capture_output=True, text=True
    )


def write_jsonl(path, ks, header=None):
    with open(path, "w", encoding="utf-8") as f:
        if header is not None:
            f.write(header + "\n")
        for i, k in enumerate(ks):
            f.write(json.dumps({"seq": i, "popcount": k}) + "\n")


def incoherent_ks(n=2000, p=0.203, num_taps=48, seed=7):
    """Synthetic TCXO-like capture: in-window hits ~Bernoulli(p), uniform bins."""
    rng = random.Random(seed)
    return [rng.randrange(num_taps) if rng.random() < p else num_taps for _ in range(n)]

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


def test_compute_code_density_requires_clock():
    # Fail-closed: no clock period, no result.
    res = compute_code_density([1, 2, 3], num_taps=48)
    assert "error" in res
    assert "25.0" in res["error"] and "31.25" in res["error"]


def test_ideal_tap_divisor():
    # Uniform in-window occupancy across the 48 in-window bins 0..47 at the
    # run1-lineage ~79.7% saturation MUST give DNL ~0. The old /(num_taps-1)
    # divisor biased every bin by ~-2.25 ps.
    ks = [k for k in range(48)] * 10 + [48] * 1890
    res = compute_code_density(ks, num_taps=48, t_clock_ns=25.0)
    window_ps = 25000.0 * (1.0 - res["saturation_rate"])
    assert res["ideal_tap_ps"] == pytest.approx(window_ps / 48, abs=1e-6)
    assert max(abs(d) for d in res["dnl_ps"]) < 0.5  # /47 gives ~2.25 ps bias
    assert max(abs(d) for d in res["dnl_lsb"]) < 0.005


def test_missing_clock_refused(tmp_path):
    # No metadata row, no --clock-ns: must exit nonzero naming both station
    # values, never guess.
    path = str(tmp_path / "noclock.jsonl")
    write_jsonl(path, incoherent_ks())
    r = run_cli(path)
    assert r.returncode != 0
    assert "25.0" in r.stderr and "31.25" in r.stderr
    assert "TCXO" in r.stderr and "CLKIN" in r.stderr
    assert "forbidden" in r.stderr


def test_explicit_clock_ns_accepted(tmp_path):
    path = str(tmp_path / "cli_clock.jsonl")
    write_jsonl(path, incoherent_ks())
    r = run_cli(path, "--clock-ns", "25.0")
    assert r.returncode == 0, r.stderr
    assert "source: --clock-ns" in r.stdout


def test_metadata_clock_used(tmp_path):
    # A config/metadata row in the file supplies the clock; no --clock-ns needed.
    path = str(tmp_path / "meta_clock.jsonl")
    header = json.dumps({"kind": "config", "run": "tdc_sweep", "clock_ns": 31.25})
    write_jsonl(path, incoherent_ks(p=0.162), header=header)
    r = run_cli(path)
    assert r.returncode == 0, r.stderr
    assert "31.25 ns (source: run metadata)" in r.stdout

    # ts_latch_run3-style metadata row: adclk = 2 x sample_rate (tdc_ts_analyze.py).
    path2 = str(tmp_path / "meta_sr.jsonl")
    header2 = json.dumps({"metadata": "TDC latch capture", "sample_rate": 16000000})
    write_jsonl(path2, incoherent_ks(p=0.162), header=header2)
    r2 = run_cli(path2)
    assert r2.returncode == 0, r2.stderr
    assert "31.25 ns (source: run metadata)" in r2.stdout


def test_metadata_cli_clock_mismatch_refused(tmp_path):
    path = str(tmp_path / "mismatch.jsonl")
    header = json.dumps({"kind": "config", "clock_ns": 31.25})
    write_jsonl(path, incoherent_ks(p=0.162), header=header)
    r = run_cli(path, "--clock-ns", "25.0")
    assert r.returncode != 0
    assert "mismatch" in r.stderr


def test_coherent_capture_all_saturated_refused(tmp_path):
    # Coherent capture parked past the window: every event saturates.
    path = str(tmp_path / "parked.jsonl")
    write_jsonl(path, [48] * 500)
    r = run_cli(path, "--clock-ns", "25.0")
    assert r.returncode != 0
    assert "COHERENT-CAPTURE" in r.stderr


def test_coherent_capture_clustered_refused_and_forced(tmp_path):
    # Coherent capture crawling through the window: one long block of
    # consecutive in-window hits (fraction itself looks plausible).
    path = str(tmp_path / "clustered.jsonl")
    write_jsonl(path, [20] * 300 + [48] * 1200)
    r = run_cli(path, "--clock-ns", "25.0")
    assert r.returncode != 0
    assert "COHERENT-CAPTURE" in r.stderr
    assert "--force" in r.stderr

    # --force overrides with a loud warning banner but produces output.
    rf = run_cli(path, "--clock-ns", "25.0", "--force")
    assert rf.returncode == 0, rf.stderr
    assert "WARNING" in rf.stderr and "force" in rf.stderr.lower()
    assert "Total events" in rf.stdout


def test_incoherent_capture_passes_guard():
    # A quasi-uniform TCXO-like capture must NOT trip the coherence guard.
    res = compute_code_density(incoherent_ks(), num_taps=48, t_clock_ns=25.0)
    assert res["coherence"]["coherent_suspect"] is False


def test_zero_events_clean_exit(tmp_path):
    # A file with no parseable events must exit nonzero with a message,
    # not a KeyError traceback.
    path = str(tmp_path / "empty.jsonl")
    with open(path, "w", encoding="utf-8") as f:
        f.write("# tdc_pps empty header only\n")
    r = run_cli(path, "--clock-ns", "25.0")
    assert r.returncode != 0
    assert "no events" in r.stderr
    assert "Traceback" not in r.stderr
