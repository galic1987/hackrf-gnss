import hashlib
import json
import os
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from tdc_density_cal import (
    EVIDENCE_SCHEMA,
    analyze_file,
    classify_a2_word,
    compute_code_density,
    decode_thermometer,
    parse_jsonl_event,
    parse_thermo_bytes,
)
from tdc_capture_seal import seal_capture


SCRIPT = os.path.join(os.path.dirname(__file__), "tdc_density_cal.py")


def run_cli(*args):
    return subprocess.run(
        [sys.executable, SCRIPT, *map(str, args)], capture_output=True, text=True
    )


def balanced_codes(cycles=60, taps=48):
    # Both halves have exactly the same occupancy distribution.
    block = list(range(1, taps)) + [taps]
    return block * cycles


def raw_word(k, taps=48):
    word = (1 << k) - 1 if k else 0
    return ",".join(f"{(word >> (8 * i)) & 0xff:02x}"
                    for i in range((taps + 7) // 8))


def complete_capture(path, ks, **config_overrides):
    bitstream_sha256 = "ab" * 32
    raw_path = str(path) + ".raw.jsonl"
    names = ["0_timing", "1_halfprec", "2_extprec_rx", "3_extprec_tx"]
    image_hashes = [bitstream_sha256, "bc" * 32, "cd" * 32, "de" * 32]
    manifest = {
        "schema": "hackrf-fpga-manifest-v2",
        "build_id": 0x48E,
        "provenance": {
            "source_sha": "1" * 40,
            "source_tree": "firmware/fpga",
            "dirty": False,
            "diff_sha256": None,
            "toolchain": {"yosys": "Yosys test", "nextpnr-ice40": "nextpnr test"},
        },
        "images": [
            {
                "slot": slot,
                "name": names[slot],
                "reg_3e": (slot << 4) | 0x04,
                "reg_3f": 0x8E,
                "bitstream_sha256": image_hashes[slot],
                **({"tdc_abi": "0xA2"} if slot == 0 else {}),
            }
            for slot in range(4)
        ],
        "timing": {
            name: {
                "achieved_mhz": {
                    "clk": 40.0,
                    **({"adclk_clk_$glb_clk": 40.0} if slot == 0 else {}),
                },
                "tim_sha256": f"{slot + 1:x}" * 64,
                "manual_adclk_path_audit": False,
            }
            for slot, name in enumerate(names)
        },
        "blob_sha256": "ef" * 32,
    }
    manifest_path = str(path) + ".manifest.json"
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, sort_keys=True)
    with open(manifest_path, "rb") as f:
        manifest_sha256 = hashlib.sha256(f.read()).hexdigest()
    config = {
        "kind": "config",
        "schema": "hackrf-pro-tdc-trigger-read-v2",
        "serial": "0000000000000000645061de252d6613",
        "slot": 0,
        "build_id": "0x48e",
        "usb_api_version": "0x0117",
        "manifest_sha256": manifest_sha256,
        "bitstream_sha256": bitstream_sha256,
        "raw_capture_sha256": None,
        "provenance_binding": "manifest-and-build-tag-smoke-test",
        "clock_source": "measured external swept-delay fixture",
        "trigger_source": "fixture output to P28.16",
        "adclk_hz": 40_000_000,
        "capture_protocol": "held-mailbox-handshake-v2",
        "tdc_abi": "0xA2",
        "baseline_status": 0x11,
        "pre_begin_mailbox_status": 0x00,
        "begin_mailbox_status": 0x21,
        "timeout_s": 2.5,
        "ready_timeout_s": 1.5,
        "command_timeout_s": 1.5,
        "selftest": False,
        "time_basis": "host-monotonic-after-mailbox-ready",
    }
    config.update(config_overrides)
    event_rows = []
    mailbox_tokens = 0x20
    for seq, k in enumerate(ks):
        final = seq == len(ks) - 1
        completion = "close" if final else "ack"
        ready = mailbox_tokens | 0x03
        mailbox_tokens ^= 0x40 if final else 0x10
        complete = mailbox_tokens | (0x00 if final else 0x01)
        byte_text = raw_word(k)
        valid_toggle = (config["baseline_status"] & 0x01) ^ ((seq + 1) & 0x01)
        event_rows.append({
            "kind": "event", "seq": seq, "fpga_event_seq": seq & 0xffff,
            "wait_start_t": seq + 0.75, "t": seq + 1.0,
            "reg31_before": 0x10 | valid_toggle,
            "reg31_after": 0x10 | valid_toggle,
            "mailbox_status_ready": ready,
            "mailbox_status_complete": complete,
            "completion": completion,
            "bytes": byte_text, "thermo": byte_text.replace(",", ""),
            "popcount": k,
            "code_class": "composite-full-scale" if k == 48 else "interior",
            "atomic_snapshot_passed": True,
            "overflow_seen": False,
            "protocol_fault": False,
        })
    stop = {
        "kind": "stop", "n": len(ks),
        "last_seq": len(ks) - 1,
        "last_fpga_event_seq": (len(ks) - 1) & 0xffff,
        "protocol_closed": True,
        "overflow_seen": False,
        "protocol_fault": False,
        "postflight_artifact_ok": True,
        "selftest_safe_off": True,
        "trigger_rearmed": True,
    }
    raw_config = dict(config)
    for field in (
        "raw_capture_sha256", "manifest_sha256", "bitstream_sha256",
        "clock_source", "trigger_source", "adclk_hz",
    ):
        raw_config[field] = None
    raw_config.pop("provenance_binding", None)
    raw_rows = [raw_config, *event_rows, stop]
    with open(raw_path, "w", encoding="utf-8") as f:
        f.write("\n".join(json.dumps(row) for row in raw_rows) + "\n")
    with open(raw_path, "rb") as f:
        config["raw_capture_sha256"] = hashlib.sha256(f.read()).hexdigest()
    sealed_rows = [config, *event_rows, stop]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(json.dumps(row) for row in sealed_rows) + "\n")
    return config


def analyze_capture(path, **kwargs):
    return analyze_file(
        str(path), artifact_manifest=str(path) + ".manifest.json",
        raw_capture=str(path) + ".raw.jsonl", **kwargs
    )


def evidence_for(dataset, path, clock_ns=25.0, **overrides):
    with open(dataset, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    evidence = {
        "schema": EVIDENCE_SCHEMA,
        "uniform_phase_verified": True,
        "method": "external-swept-delay",
        "dataset_sha256": digest,
        "clock_ns": clock_ns,
    }
    evidence.update(overrides)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(evidence, f)
    return evidence


def rewrite_manifest_and_rebind(data, mutate):
    manifest_path = str(data) + ".manifest.json"
    with open(manifest_path, encoding="utf-8") as f:
        manifest = json.load(f)
    mutate(manifest)
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, sort_keys=True)
    with open(manifest_path, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[0]["manifest_sha256"] = digest
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")


def rebind_raw_hash(data):
    raw_path = str(data) + ".raw.jsonl"
    with open(raw_path, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[0]["raw_capture_sha256"] = digest
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")


def replace_event_word(data, event_index, byte_text, code_class):
    """Change one hash-bound raw/sealed event while retaining its exact word."""
    popcount = sum(parse_thermo_bytes(byte_text))
    for path in (data, str(data) + ".raw.jsonl"):
        with open(path, encoding="utf-8") as f:
            rows = [json.loads(line) for line in f]
        row = rows[event_index + 1]
        row["bytes"] = byte_text
        row["thermo"] = byte_text.replace(",", "")
        row["popcount"] = popcount
        row["code_class"] = code_class
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(json.dumps(item) for item in rows) + "\n")
    rebind_raw_hash(data)


def test_parse_thermo_bytes_exact_word():
    taps = parse_thermo_bytes("01,00,00,00,00,00")
    assert len(taps) == 48
    assert taps[0] == 1 and sum(taps) == 1


def test_decode_thermometer_is_strict_prefix_only():
    assert decode_thermometer([1] * 16 + [0] * 32) == 16
    assert decode_thermometer([1] * 48) == 48
    assert decode_thermometer([0] * 48) == 0
    assert decode_thermometer([0, 1] + [0] * 46) is None
    assert decode_thermometer([1, 1, 0, 1] + [0] * 44) is None
    assert decode_thermometer([1] * 47) is None
    assert decode_thermometer([1] * 49) is None


def test_a2_word_classifier_accepts_only_tap0_anchor_and_preserves_bubbles():
    interior = classify_a2_word([1] * 16 + [0] * 32)
    assert interior == {
        "code_class": "interior", "popcount": 16, "density_code": 16,
        "density_eligible": True,
    }
    full = classify_a2_word([1] * 48)
    assert full["code_class"] == "composite-full-scale"
    assert full["density_code"] == 48
    bubbled = classify_a2_word([1, 0, 1] + [0] * 45)
    assert bubbled == {
        "code_class": "bubbled", "popcount": 2, "density_code": None,
        "density_eligible": False,
    }
    assert classify_a2_word([0] * 48) is None
    assert classify_a2_word([0, 0, 1] + [0] * 45) is None


def test_parse_event_rejects_duplicate_and_short_word():
    good = parse_jsonl_event(
        '{"t":123.456,"bytes":"ff,00,00,00,00,00",'
        '"thermo":"ff0000000000","popcount":8}'
    )
    assert good == {
        "t": 123.456,
        "k": 8,
        "popcount": 8,
        "code_class": "interior",
        "raw_word": "ff,00,00,00,00,00",
    }
    bubbled = parse_jsonl_event(
        '{"t":2,"bytes":"05,00,00,00,00,00",'
        '"thermo":"050000000000","popcount":2}'
    )
    assert bubbled == {
        "t": 2.0,
        "k": None,
        "popcount": 2,
        "code_class": "bubbled",
        "raw_word": "05,00,00,00,00,00",
    }
    assert parse_jsonl_event(
        '{"t":2,"bytes":"04,00,00,00,00,00",'
        '"thermo":"040000000000","popcount":1}'
    ) is None
    assert parse_jsonl_event(
        '{"t":1,"bytes":"01,00","thermo":"0100","popcount":1,"dup":false}'
    ) is None
    assert parse_jsonl_event(
        '{"t":1,"bytes":"01,00,00,00,00,00",'
        '"thermo":"010000000000","popcount":1,"dup":true}'
    ) is None


def test_histogram_without_uniform_evidence_is_not_calibration():
    res = compute_code_density(balanced_codes(), t_clock_ns=25.0)
    assert res["total_events"] == 48 * 60
    assert res["code0_invalid_events"] == 0
    assert res["overflow_composite_events"] == 60
    assert res["absolute_calibration_valid"] is False
    assert "interior_bin_widths_ps" not in res
    assert any("uniform input phase" in s for s in res["calibration_gate_fails"])


def test_uniform_declaration_cannot_overcome_first_tap_transfer_blocker():
    res = compute_code_density(
        balanced_codes(), t_clock_ns=25.0, uniform_phase_verified=True
    )
    assert res["absolute_calibration_valid"] is False
    assert res["interior_codes"] == list(range(1, 48))
    assert "interior_bin_widths_ps" not in res
    assert "dnl_lsb" not in res and "inl_lsb" not in res
    assert any("tap-0 event-selection" in s
               for s in res["calibration_gate_fails"])


def test_code_zero_and_low_counts_fail_absolute_gates():
    res = compute_code_density(
        [0] + balanced_codes(cycles=2), 48, 25.0, uniform_phase_verified=True
    )
    assert res["absolute_calibration_valid"] is False
    assert any("code 0" in s for s in res["calibration_gate_fails"])
    assert any("below 50" in s for s in res["calibration_gate_fails"])


def test_invalid_code_and_clock_are_rejected():
    assert "error" in compute_code_density([1, 49], 48, 25.0)
    assert "error" in compute_code_density([1], 48, 0.0)
    assert "error" in compute_code_density([1], 1, 25.0)


def test_complete_capture_stays_characterization_without_evidence(tmp_path):
    data = tmp_path / "capture.jsonl"
    complete_capture(data, balanced_codes())
    res = analyze_capture(data)
    assert res["capture"]["config_rows"] == 1
    assert res["capture"]["stop_rows"] == 1
    assert res["absolute_calibration_valid"] is False
    assert res["capture_integrity_valid"] is True
    assert res["artifact_manifest_status"].startswith("manifest/tag-bound")
    assert res["t_clock_source"] == "run metadata"


def test_bubbled_a2_event_is_integrity_valid_but_never_a_density_bin(tmp_path):
    data = tmp_path / "bubbled.jsonl"
    codes = balanced_codes()
    complete_capture(data, codes)
    # Replace the first code-1 prefix with tap0+t2. Its popcount is 2, but it
    # must not inflate histogram bin 2 or become a calibrated code.
    replace_event_word(data, 0, "05,00,00,00,00,00", "bubbled")
    res = analyze_capture(data)
    assert res["capture_integrity_valid"] is True
    assert res["capture_total_events"] == len(codes)
    assert res["density_input_events"] == len(codes) - 1
    assert res["bubbled_capture_events"] == 1
    assert res["bubbled_excluded_from_density_events"] == 1
    assert res["capture"]["bubbled_events"] == 1
    assert res["capture"]["mailbox_protocol_errors"] == 0
    assert res["hist"][2] == 60
    assert res["absolute_calibration_valid"] is False
    assert any("popcount is not a calibrated tap-bin code" in reason
               for reason in res["calibration_gate_fails"])


def test_sample_rate_is_not_promoted_to_tdc_clock(tmp_path):
    data = tmp_path / "sample_rate_only.jsonl"
    complete_capture(data, balanced_codes(), adclk_hz=None)
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[0].pop("adclk_hz")
    rows[0]["sample_rate"] = 16_000_000
    data.write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    res = analyze_capture(data)
    assert "error" in res and "sample_rate" in res["error"]


def test_matching_declaration_is_recorded_but_does_not_unlock(tmp_path):
    data = tmp_path / "capture.jsonl"
    ev = tmp_path / "uniform.json"
    complete_capture(data, balanced_codes())
    evidence_for(data, ev)
    res = analyze_capture(data, uniform_phase_evidence=str(ev))
    assert res["absolute_calibration_valid"] is False
    assert res["uniform_phase_evidence_status"].startswith("verified;")
    assert any("tap-0 event-selection" in s
               for s in res["calibration_gate_fails"])


def test_popcount_only_capture_is_not_integrity_auditable(tmp_path):
    data = tmp_path / "popcount.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    for row in rows:
        row.pop("bytes", None)
        row.pop("thermo", None)
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    assert res["capture"]["raw_word_events"] == 0
    assert res["capture"]["popcount_only_events"] == len(balanced_codes())
    assert any("popcount-only" in s for s in res["calibration_gate_fails"])


def test_evidence_hash_mismatch_stays_fail_closed(tmp_path):
    data = tmp_path / "capture.jsonl"
    ev = tmp_path / "uniform.json"
    complete_capture(data, balanced_codes())
    evidence_for(data, ev, dataset_sha256="0" * 64)
    res = analyze_capture(data, uniform_phase_evidence=str(ev))
    assert res["absolute_calibration_valid"] is False
    assert "matching dataset_sha256" in res["uniform_phase_evidence_status"]


def test_abort_duplicate_and_missing_provenance_are_claim_blockers(tmp_path):
    data = tmp_path / "bad_capture.jsonl"
    ks = balanced_codes()
    complete_capture(data, ks)
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[0].pop("build_id")
    rows.insert(2, {"seq": 0, "popcount": 1, "dup": True})
    rows.insert(-1, {"kind": "abort", "reason": "test"})
    data.write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    ev = tmp_path / "uniform.json"
    evidence_for(data, ev)
    res = analyze_capture(data, uniform_phase_evidence=str(ev))
    assert res["absolute_calibration_valid"] is False
    joined = " ".join(res["calibration_gate_fails"])
    assert "abort" in joined and "duplicate" in joined and "build_id" in joined


def test_unknown_reordered_and_missing_stop_count_are_capture_failures(tmp_path):
    data = tmp_path / "bad_framing.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows.insert(2, {"kind": "mystery", "note": "must not be ignored"})
    rows[3]["seq"] = rows[1]["seq"]
    rows[-1].pop("n")
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    joined = " ".join(res["calibration_gate_fails"])
    assert "unknown/non-event" in joined
    assert "contiguous" in joined
    assert "requires an exact" in joined


def test_checkpoint_and_rows_after_stop_are_rejected(tmp_path):
    data = tmp_path / "nonterminal_stop.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows.insert(2, {"kind": "checkpoint", "n": 1})
    rows.append({"kind": "checkpoint", "n": len(balanced_codes())})
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    joined = " ".join(res["calibration_gate_fails"])
    assert "unknown/non-event" in joined
    assert "final nonblank row" in joined


def test_capture_protocol_config_invariants_are_exact(tmp_path):
    data = tmp_path / "bad_protocol_config.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[0]["baseline_status"] = 0x12  # busy plus first event no longer fresh
    rows[0]["timeout_s"] = 3.0
    rows[0]["ready_timeout_s"] = 1.0
    rows[0]["command_timeout_s"] = 1.0
    rows[0]["selftest"] = True
    rows[0]["time_basis"] = "wall-clock"
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    joined = " ".join(res["calibration_gate_fails"])
    assert "baseline_status must prove" in joined
    assert "timeout_s must be exactly 2.5" in joined
    assert "ready_timeout_s must be exactly 1.5" in joined
    assert "command_timeout_s must be exactly 1.5" in joined
    assert "selftest must be false" in joined
    assert "host-monotonic-after-mailbox-ready" in joined


def test_event_kind_and_integer_sequence_are_strict(tmp_path):
    data = tmp_path / "bad_types.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[1].pop("kind")
    rows[2]["seq"] = 1.0
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    joined = " ".join(res["calibration_gate_fails"])
    assert "without kind='event'" in joined
    assert "invalid event" in joined


def test_held_mailbox_snapshot_proofs_are_strict(tmp_path):
    data = tmp_path / "bad_status.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[1]["atomic_snapshot_passed"] = False
    rows[2]["overflow_seen"] = True
    rows[3]["mailbox_status_ready"] |= 0x80
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    assert res["capture"]["mailbox_protocol_errors"] >= 3
    assert any("held-mailbox" in reason
               for reason in res["calibration_gate_fails"])


def test_mailbox_token_chain_and_valid_toggle_are_independently_proved(tmp_path):
    data = tmp_path / "bad_token_chain.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[1]["mailbox_status_ready"] ^= 0x10  # not the BEGIN-complete token state
    rows[2]["mailbox_status_complete"] ^= 0x40  # ACK also moved CLOSE_DONE
    rows[3]["reg31_after"] ^= 0x01  # producer token changed during held read
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    assert res["capture"]["mailbox_protocol_errors"] >= 3
    assert res["capture_integrity_valid"] is False


def test_stop_requires_postflight_safety_proofs(tmp_path):
    data = tmp_path / "bad_stop_safety.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[-1].pop("selftest_safe_off")
    rows[-1]["trigger_rearmed"] = False
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    joined = " ".join(res["calibration_gate_fails"])
    assert "selftest_safe_off=true" in joined
    assert "trigger_rearmed=true" in joined


def test_event_monotonic_deadline_timestamps_are_mandatory(tmp_path):
    data = tmp_path / "bad_timestamps.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[1].pop("wait_start_t")
    rows[2]["t"] = rows[1]["t"]
    rows[3]["wait_start_t"] = rows[3]["t"] - 2.5
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    assert res["capture"]["timestamp_errors"] >= 3
    assert res["capture_integrity_valid"] is False
    assert any("timestamp pair" in reason
               for reason in res["calibration_gate_fails"])


def test_oversized_json_numbers_fail_closed_without_exception(tmp_path):
    data = tmp_path / "oversized.jsonl"
    complete_capture(data, balanced_codes())
    lines = data.read_text().splitlines()
    lines[1] = lines[1][:-1] + ',"oversized":' + ("9" * 1000) + "}"
    data.write_text("\n".join(lines) + "\n")
    res = analyze_capture(data)
    assert res["capture_integrity_valid"] is False
    assert res["capture"]["malformed_rows"] == 1


def test_manifest_hash_and_slot_identity_are_fail_closed(tmp_path):
    data = tmp_path / "artifact_tamper.jsonl"
    complete_capture(data, balanced_codes())
    manifest_path = str(data) + ".manifest.json"
    with open(manifest_path, encoding="utf-8") as f:
        manifest = json.load(f)
    manifest["images"][0]["reg_3f"] ^= 1
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, sort_keys=True)
    res = analyze_capture(data)
    assert res["capture_integrity_valid"] is False
    assert "artifact manifest sha256 does not match config" in (
        " ".join(res["calibration_gate_fails"])
    )


def test_cli_exit_contract_distinguishes_integrity_and_calibration(tmp_path):
    data = tmp_path / "capture.jsonl"
    complete_capture(data, balanced_codes())
    manifest = str(data) + ".manifest.json"
    raw = str(data) + ".raw.jsonl"
    ok = run_cli(
        data, "--artifact-manifest", manifest, "--raw-capture", raw, "--quiet"
    )
    assert ok.returncode == 0
    uncalibrated = run_cli(
        data, "--artifact-manifest", manifest, "--raw-capture", raw,
        "--require-calibration", "--quiet"
    )
    assert uncalibrated.returncode == 3
    no_manifest = run_cli(data, "--quiet")
    assert no_manifest.returncode == 2


def test_non_utf8_capture_is_rejected_as_an_error(tmp_path):
    data = tmp_path / "binary.jsonl"
    data.write_bytes(b"\xff\xfe\x00")
    res = analyze_file(str(data))
    assert "error" in res and "UTF-8" in res["error"]


def test_out_json_refused_even_with_declaration_while_transfer_is_unknown(tmp_path):
    data = tmp_path / "capture.jsonl"
    ev = tmp_path / "uniform.json"
    out = tmp_path / "cal.json"
    complete_capture(data, balanced_codes())
    manifest = str(data) + ".manifest.json"
    raw = str(data) + ".raw.jsonl"
    r = run_cli(
        data, "--artifact-manifest", manifest, "--raw-capture", raw,
        "--out-json", out,
    )
    assert r.returncode != 0
    assert "Refusing --out-json" in r.stderr
    assert not out.exists()

    evidence_for(data, ev)
    r = run_cli(
        data, "--artifact-manifest", manifest, "--raw-capture", raw,
        "--uniform-phase-evidence", ev, "--out-json", out,
    )
    assert r.returncode == 3
    assert "Refusing --out-json" in r.stderr
    assert not out.exists()


def test_v1_schema_protocol_and_abi_are_rejected(tmp_path):
    data = tmp_path / "v1.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[0]["schema"] = "hackrf-pro-tdc-trigger-read-v1"
    rows[0]["capture_protocol"] = "valid-toggle-bracket-v1"
    rows[0]["tdc_abi"] = "0xA1"
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    joined = " ".join(res["calibration_gate_fails"])
    assert res["capture_integrity_valid"] is False
    assert "v1 is not claim-grade" in joined
    assert "held-mailbox-handshake-v2" in joined
    assert "tdc_abi must be 0xA2" in joined


def test_exact_48_tap_abi_is_enforced_before_histogram_allocation(tmp_path):
    data = tmp_path / "capture.jsonl"
    complete_capture(data, balanced_codes())
    assert "exactly 48" in compute_code_density([1], 10**9, 25.0)["error"]
    assert "exactly 48" in analyze_capture(data, num_taps=49)["error"]


def test_code_zero_is_capture_integrity_failure_and_cli_exit_two(tmp_path):
    data = tmp_path / "code0.jsonl"
    complete_capture(data, [0] + balanced_codes())
    res = analyze_capture(data)
    assert res["capture_integrity_valid"] is False
    assert res["capture"]["code0_events"] == 1
    assert res["code0_invalid_events"] == 1
    out = tmp_path / "must_not_exist.json"
    cli = run_cli(
        data,
        "--artifact-manifest", str(data) + ".manifest.json",
        "--raw-capture", str(data) + ".raw.jsonl",
        "--out-json", out,
        "--quiet",
    )
    assert cli.returncode == 2
    assert not out.exists()


def test_nonzero_tap0_clear_word_is_not_an_a2_event(tmp_path):
    data = tmp_path / "unanchored.jsonl"
    complete_capture(data, balanced_codes())
    replace_event_word(data, 0, "04,00,00,00,00,00", "bubbled")
    res = analyze_capture(data)
    assert res["capture_integrity_valid"] is False
    assert res["capture"]["unanchored_events"] == 1
    assert res["capture"]["invalid_event_rows"] >= 1
    assert any("tap-0-clear" in reason
               for reason in res["calibration_gate_fails"])


def test_fpga_sequence_atomicity_overflow_and_protocol_closure_are_mandatory(tmp_path):
    data = tmp_path / "bad_mailbox.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[2]["fpga_event_seq"] += 1
    rows[3]["atomic_snapshot_passed"] = False
    rows[4]["overflow_seen"] = True
    rows[-1]["last_fpga_event_seq"] -= 1
    rows[-1]["protocol_closed"] = False
    rows[-1]["protocol_fault"] = True
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    joined = " ".join(res["calibration_gate_fails"])
    assert res["capture"]["sequence_errors"] >= 1
    assert res["capture"]["mailbox_protocol_errors"] >= 2
    assert "last_fpga_event_seq" in joined
    assert "protocol_closed=true" in joined
    assert "protocol_fault=false" in joined


def test_redundant_thermometer_and_popcount_must_match_bytes(tmp_path):
    data = tmp_path / "contradictory_raw.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[1]["thermo"] = "ff" * 6
    rows[2]["popcount"] += 1
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    assert res["capture_integrity_valid"] is False
    assert res["capture"]["invalid_event_rows"] >= 2


def test_adclk_is_authoritative_and_period_aliases_cannot_contradict_it(tmp_path):
    data = tmp_path / "clock_conflict.jsonl"
    complete_capture(data, balanced_codes(), clock_ns=31.25, t_clock_ns=25.0)
    res = analyze_capture(data, t_clock_ns=25.0)
    assert res["capture_integrity_valid"] is False
    assert any("period aliases must agree" in reason
               for reason in res["calibration_gate_fails"])


def test_raw_capture_sidecar_is_required_and_hash_verified(tmp_path):
    data = tmp_path / "raw_binding.jsonl"
    complete_capture(data, balanced_codes())
    without_raw = analyze_file(
        str(data), artifact_manifest=str(data) + ".manifest.json"
    )
    assert without_raw["capture_integrity_valid"] is False
    assert "sidecar is required" in " ".join(without_raw["calibration_gate_fails"])
    with open(str(data) + ".raw.jsonl", "ab") as f:
        f.write(b'{"kind":"tamper"}\n')
    tampered = analyze_capture(data)
    assert tampered["capture_integrity_valid"] is False
    assert tampered["raw_capture_status"] == "raw capture sha256 does not match config"


def test_raw_binding_rejects_sealed_event_or_nonowned_config_changes(tmp_path):
    data = tmp_path / "sealed_tamper.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[0]["serial"] = "0000000000000000645061de252d6614"
    rows[1]["bytes"] = "03,00,00,00,00,00"
    rows[1]["thermo"] = "030000000000"
    rows[1]["popcount"] = 2
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    assert res["capture_integrity_valid"] is False
    assert res["raw_capture_status"] == (
        "raw/sealed config differs outside sealer-owned provenance fields"
    )

    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[1]["bytes"] = "03,00,00,00,00,00"
    rows[1]["thermo"] = "030000000000"
    rows[1]["popcount"] = 2
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    event_only = analyze_capture(data)
    assert event_only["raw_capture_status"] == "raw/sealed logical row 2 differs"

    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[1]["t"] = 1  # Numerically equal to raw 1.0, but a changed JSON type.
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    type_changed = analyze_capture(data)
    assert type_changed["raw_capture_status"] == "raw/sealed logical row 2 differs"


def test_raw_binding_rejects_preclaimed_placeholders_and_row_count_changes(tmp_path):
    data = tmp_path / "raw_tamper.jsonl"
    complete_capture(data, balanced_codes())
    raw_path = str(data) + ".raw.jsonl"
    with open(raw_path, encoding="utf-8") as f:
        raw_rows = [json.loads(line) for line in f]
    raw_rows[0]["manifest_sha256"] = "11" * 32
    with open(raw_path, "w", encoding="utf-8") as f:
        f.write("\n".join(json.dumps(row) for row in raw_rows) + "\n")
    rebind_raw_hash(data)
    preclaimed = analyze_capture(data)
    assert preclaimed["capture_integrity_valid"] is False
    assert "null provenance placeholders" in preclaimed["raw_capture_status"]

    complete_capture(data, balanced_codes())
    with open(raw_path, encoding="utf-8") as f:
        raw_rows = [json.loads(line) for line in f]
    raw_rows.pop(1)
    with open(raw_path, "w", encoding="utf-8") as f:
        f.write("\n".join(json.dumps(row) for row in raw_rows) + "\n")
    rebind_raw_hash(data)
    missing_row = analyze_capture(data)
    assert missing_row["capture_integrity_valid"] is False
    assert "raw/sealed logical row 2 differs" == missing_row["raw_capture_status"]


def test_raw_binding_uses_strict_duplicate_key_parser(tmp_path):
    data = tmp_path / "raw_duplicate.jsonl"
    complete_capture(data, balanced_codes())
    raw_path = str(data) + ".raw.jsonl"
    with open(raw_path, encoding="utf-8") as f:
        lines = f.read().splitlines()
    lines[1] = lines[1][:-1] + ',"seq":0}'
    with open(raw_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    rebind_raw_hash(data)
    res = analyze_capture(data)
    assert res["capture_integrity_valid"] is False
    assert "duplicate JSON key: seq" in res["raw_capture_status"]


def test_duplicate_keys_nan_and_missing_terminal_newline_fail_closed(tmp_path):
    data = tmp_path / "strict_json.jsonl"
    complete_capture(data, balanced_codes())
    lines = data.read_text().splitlines()
    lines[1] = lines[1][:-1] + ',"seq":0}'
    lines[2] = lines[2][:-1] + ',"nan":NaN}'
    data.write_text("\n".join(lines))
    res = analyze_capture(data)
    assert res["capture_integrity_valid"] is False
    assert res["capture"]["malformed_rows"] == 2
    joined = " ".join(res["calibration_gate_fails"])
    assert "must end with a newline" in joined


def test_manifest_must_cover_four_slots_and_reject_timing_waivers(tmp_path):
    data = tmp_path / "manifest_structure.jsonl"
    complete_capture(data, balanced_codes())

    def remove_slot(manifest):
        removed = manifest["images"].pop()
        manifest["timing"].pop(removed["name"])

    rewrite_manifest_and_rebind(data, remove_slot)
    missing = analyze_capture(data)
    assert missing["capture_integrity_valid"] is False
    assert "exactly four flash slots" in " ".join(missing["calibration_gate_fails"])

    complete_capture(data, balanced_codes())

    def unaudited_slow_adclk(manifest):
        entry = manifest["timing"][manifest["images"][0]["name"]]
        entry["achieved_mhz"]["adclk_clk_$glb_clk"] = 39.0
        entry["manual_adclk_path_audit"] = False

    rewrite_manifest_and_rebind(data, unaudited_slow_adclk)
    slow = analyze_capture(data)
    assert slow["capture_integrity_valid"] is False
    assert "aggregate adclk timing must be at least 40 MHz" in (
        " ".join(slow["calibration_gate_fails"])
    )

    complete_capture(data, balanced_codes())

    def manual_waiver(manifest):
        entry = manifest["timing"][manifest["images"][0]["name"]]
        entry["manual_adclk_path_audit"] = True

    rewrite_manifest_and_rebind(data, manual_waiver)
    waived = analyze_capture(data)
    assert waived["capture_integrity_valid"] is False
    assert "manual_adclk_path_audit must be false" in (
        " ".join(waived["calibration_gate_fails"])
    )

    complete_capture(data, balanced_codes())

    def legacy_receipt(manifest):
        entry = manifest["timing"][manifest["images"][0]["name"]]
        entry["manual_adclk_audit_receipt"] = {"claimed": "legacy waiver"}

    rewrite_manifest_and_rebind(data, legacy_receipt)
    receipt = analyze_capture(data)
    assert receipt["capture_integrity_valid"] is False
    assert "retired manual timing waiver receipt" in (
        " ".join(receipt["calibration_gate_fails"])
    )


def test_manifest_status_does_not_claim_deployed_bitstream_verification(tmp_path):
    data = tmp_path / "honest_manifest.jsonl"
    complete_capture(data, balanced_codes())
    res = analyze_capture(data)
    assert res["capture_integrity_valid"] is True
    assert res["artifact_manifest_status"] == (
        "manifest/tag-bound; deployed identity limited to BUILD_ID/ABI postflight"
    )


def test_official_sealer_output_is_accepted_with_raw_sidecar(tmp_path):
    template = tmp_path / "template.jsonl"
    raw = tmp_path / "raw.jsonl"
    sealed = tmp_path / "sealed.jsonl"
    complete_capture(template, balanced_codes())
    rows = [json.loads(line) for line in template.read_text().splitlines()]
    for field in (
        "raw_capture_sha256", "manifest_sha256", "bitstream_sha256",
        "clock_source", "trigger_source", "adclk_hz",
    ):
        rows[0][field] = None
    rows[0].pop("provenance_binding")
    raw.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    manifest = str(template) + ".manifest.json"
    seal_capture(
        str(raw), str(sealed), manifest, "0000000000000000645061de252d6613",
        "Bodnar 10 MHz equal-length split",
        "Bodnar 1 PPS equal-length P28.16",
        40_000_000,
    )
    res = analyze_file(
        str(sealed), artifact_manifest=manifest, raw_capture=str(raw)
    )
    assert res["capture_integrity_valid"] is True
    assert res["raw_capture_status"] == "verified hash and exact sealed derivative"


def test_serial_provenance_is_canonical_full_nonzero_device_serial(tmp_path):
    data = tmp_path / "serial.jsonl"
    complete_capture(data, balanced_codes())
    rows = [json.loads(line) for line in data.read_text().splitlines()]
    rows[0]["serial"] = "A" * 32
    data.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    res = analyze_capture(data)
    assert res["capture_integrity_valid"] is False
    assert "canonical 32-digit device serial" in (
        " ".join(res["calibration_gate_fails"])
    )


def test_evidence_and_manifest_non_utf8_are_rejected_without_exception(tmp_path):
    data = tmp_path / "nonutf_documents.jsonl"
    evidence = tmp_path / "evidence.json"
    complete_capture(data, balanced_codes())
    evidence.write_bytes(b"\xff")
    res = analyze_capture(data, uniform_phase_evidence=str(evidence))
    assert res["capture_integrity_valid"] is True
    assert res["uniform_phase_evidence_status"].startswith("unreadable evidence")
    (tmp_path / "nonutf_documents.jsonl.manifest.json").write_bytes(b"\xff")
    bad_manifest = analyze_capture(data)
    assert bad_manifest["capture_integrity_valid"] is False
    assert "unreadable artifact manifest" in (
        " ".join(bad_manifest["calibration_gate_fails"])
    )
