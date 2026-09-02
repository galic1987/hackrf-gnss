import hashlib
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from tdc_capture_seal import seal_capture


SERIAL = "0000000000000000645061de252d6613"


def write_manifest(path, build_id=0x48E):
    images = []
    for slot, name in enumerate((
        "0_standard", "1_halfprec", "2_extprec_rx", "3_extprec_tx"
    )):
        images.append({
            "slot": slot,
            "name": name,
            "reg_3e": (slot << 4) | (build_id >> 8),
            "reg_3f": build_id & 0xff,
            "bitstream_sha256": f"{slot + 1:02x}" * 32,
            "tdc_abi": "0xA2" if slot == 0 else None,
        })
    manifest = {
        "schema": "hackrf-fpga-manifest-v2",
        "build_id": build_id,
        "blob_sha256": "aa" * 32,
        "provenance": {
            "source_sha": "1" * 40,
            "source_tree": "firmware/fpga",
            "dirty": False,
            "diff_sha256": None,
            "toolchain": {"yosys": "test", "nextpnr-ice40": "test"},
        },
        "images": images,
        "timing": {
            name: {
                "achieved_mhz": {
                    "clk": 48.0,
                    "adclk_clk_$glb_clk": 40.0,
                    "daclk_clk_$glb_clk": 40.0,
                },
                "tim_sha256": f"{slot + 5:02x}" * 32,
                "manual_adclk_path_audit": False,
            }
            for slot, name in enumerate((
                "0_standard", "1_halfprec", "2_extprec_rx", "3_extprec_tx"
            ))
        },
    }
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return manifest


def write_raw(path, serial=SERIAL, build_id="0x48e"):
    config = {
        "kind": "config",
        "schema": "hackrf-pro-tdc-trigger-read-v2",
        "capture_protocol": "held-mailbox-handshake-v2",
        "serial": serial,
        "slot": 0,
        "build_id": build_id,
        "tdc_abi": "0xA2",
        "usb_api_version": "0x0117",
        "manifest_sha256": None,
        "bitstream_sha256": None,
        "clock_source": None,
        "trigger_source": None,
        "adclk_hz": None,
        "raw_capture_sha256": None,
    }
    terminal = {"kind": "abort", "reason": "fixture"}
    path.write_text(
        json.dumps(config) + "\n" + json.dumps(terminal) + "\n",
        encoding="utf-8",
    )
    return config


def test_seal_binds_raw_manifest_and_physical_metadata(tmp_path):
    raw = tmp_path / "raw.jsonl"
    out = tmp_path / "sealed.jsonl"
    manifest_path = tmp_path / "manifest.json"
    write_raw(raw)
    manifest = write_manifest(manifest_path)
    original = raw.read_bytes()
    result = seal_capture(
        str(raw), str(out), str(manifest_path), SERIAL,
        "Bodnar 10 MHz via equal-length split",
        "Bodnar 1 PPS via equal-length P28.16 cables",
        40_000_000,
    )
    assert raw.read_bytes() == original
    rows = [json.loads(line) for line in out.read_text().splitlines()]
    config = rows[0]
    assert config["raw_capture_sha256"] == hashlib.sha256(original).hexdigest()
    assert config["manifest_sha256"] == hashlib.sha256(
        manifest_path.read_bytes()
    ).hexdigest()
    assert config["bitstream_sha256"] == manifest["images"][0]["bitstream_sha256"]
    assert config["adclk_hz"] == 40_000_000
    assert result["sealed_capture_sha256"] == hashlib.sha256(out.read_bytes()).hexdigest()


def test_seal_refuses_overwrite_and_identity_mismatch(tmp_path):
    raw = tmp_path / "raw.jsonl"
    out = tmp_path / "sealed.jsonl"
    manifest_path = tmp_path / "manifest.json"
    write_raw(raw)
    write_manifest(manifest_path)
    out.write_text("owned by user", encoding="utf-8")
    with pytest.raises(FileExistsError):
        seal_capture(
            str(raw), str(out), str(manifest_path), SERIAL,
            "clock", "trigger", 40_000_000,
        )
    assert out.read_text() == "owned by user"
    out.unlink()
    with pytest.raises(ValueError, match="serial"):
        seal_capture(
            str(raw), str(out), str(manifest_path), "0" * 32,
            "clock", "trigger", 40_000_000,
        )


def test_seal_rejects_dirty_or_partial_manifest(tmp_path):
    raw = tmp_path / "raw.jsonl"
    out = tmp_path / "sealed.jsonl"
    manifest_path = tmp_path / "manifest.json"
    write_raw(raw)
    manifest = write_manifest(manifest_path)
    manifest["provenance"]["dirty"] = True
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="clean provenance"):
        seal_capture(
            str(raw), str(out), str(manifest_path), SERIAL,
            "clock", "trigger", 40_000_000,
        )


def test_seal_rejects_preclaimed_raw_hash_sub40_and_manual_waiver(tmp_path):
    raw = tmp_path / "raw.jsonl"
    out = tmp_path / "sealed.jsonl"
    manifest_path = tmp_path / "manifest.json"
    config = write_raw(raw)
    manifest = write_manifest(manifest_path)

    config["raw_capture_sha256"] = "11" * 32
    raw.write_text(
        json.dumps(config) + "\n" + json.dumps({"kind": "abort", "reason": "fixture"}) + "\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="raw_capture_sha256 must be null"):
        seal_capture(
            str(raw), str(out), str(manifest_path), SERIAL,
            "clock", "trigger", 40_000_000,
        )

    write_raw(raw)
    manifest["timing"]["0_standard"]["achieved_mhz"]["adclk_clk_$glb_clk"] = 39.0
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="at least 40 MHz"):
        seal_capture(
            str(raw), str(out), str(manifest_path), SERIAL,
            "clock", "trigger", 40_000_000,
        )

    manifest = write_manifest(manifest_path)
    manifest["timing"]["0_standard"]["manual_adclk_path_audit"] = True
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="manual_adclk_path_audit=false"):
        seal_capture(
            str(raw), str(out), str(manifest_path), SERIAL,
            "clock", "trigger", 40_000_000,
        )

    manifest = write_manifest(manifest_path)
    manifest["timing"]["0_standard"]["manual_adclk_audit_receipt"] = {
        "claimed": "legacy waiver"
    }
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="retired manual timing waiver"):
        seal_capture(
            str(raw), str(out), str(manifest_path), SERIAL,
            "clock", "trigger", 40_000_000,
        )


def test_seal_rejects_nonfinite_json_number(tmp_path):
    raw = tmp_path / "raw.jsonl"
    out = tmp_path / "sealed.jsonl"
    manifest_path = tmp_path / "manifest.json"
    write_raw(raw)
    write_manifest(manifest_path)
    raw.write_text(
        raw.read_text(encoding="utf-8").replace('"slot": 0', '"slot": 1e999'),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="finite"):
        seal_capture(
            str(raw), str(out), str(manifest_path), SERIAL,
            "clock", "trigger", 40_000_000,
        )


def test_seal_rejects_predeclared_binding_and_missing_final_newline(tmp_path):
    raw = tmp_path / "raw.jsonl"
    out = tmp_path / "sealed.jsonl"
    manifest_path = tmp_path / "manifest.json"
    config = write_raw(raw)
    write_manifest(manifest_path)
    config["provenance_binding"] = "self-asserted"
    raw.write_text(
        json.dumps(config) + "\n" + json.dumps({"kind": "abort", "reason": "fixture"}) + "\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="must not predeclare"):
        seal_capture(
            str(raw), str(out), str(manifest_path), SERIAL,
            "clock", "trigger", 40_000_000,
        )

    write_raw(raw)
    raw.write_bytes(raw.read_bytes().rstrip(b"\n"))
    with pytest.raises(ValueError, match="end with a newline"):
        seal_capture(
            str(raw), str(out), str(manifest_path), SERIAL,
            "clock", "trigger", 40_000_000,
        )
