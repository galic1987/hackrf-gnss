#!/usr/bin/env python3
"""Seal a raw HackRF Pro A2 TDC JSONL capture with bench provenance.

This is deliberately offline: it never opens a radio or executes another
program.  The raw producer cannot know the physical reference, cable, ADC
clock, or release-manifest path, so it emits null placeholders.  This tool
fills those placeholders in a new, no-overwrite file while preserving a
SHA-256 link to the exact raw bytes.

The seal binds the capture to a manifest and its slot-0 bitstream hash.  It is
not a bitstream readback proof; the FPGA exposes only its 12-bit BUILD_ID and
TDC ABI as smoke-test identities.
"""

import argparse
import hashlib
import json
import os
import re
import stat
import tempfile
from typing import Any, Dict, List, Tuple


CAPTURE_SCHEMA = "hackrf-pro-tdc-trigger-read-v2"
PROTOCOL = "held-mailbox-handshake-v2"
MANIFEST_SCHEMA = "hackrf-fpga-manifest-v2"
TDC_ABI = "0xA2"
SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")
SERIAL_RE = re.compile(r"^[0-9a-fA-F]{32}$")
BUILD_RE = re.compile(r"^0x[0-9a-fA-F]{3}$")
MAX_RAW_BYTES = 128 * 1024 * 1024
MAX_MANIFEST_BYTES = 4 * 1024 * 1024
MAX_JSON_LINE_BYTES = 64 * 1024
MAX_JSON_ROWS = 1_000_002


def _reject_duplicate_keys(pairs: List[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def _strict_json(text: str) -> Any:
    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON number {value!r}")

    def parse_int(value: str) -> int:
        if len(value.lstrip("-")) > 19:
            raise ValueError("JSON integer exceeds the signed 64-bit token limit")
        parsed = int(value)
        if not -(1 << 63) <= parsed < (1 << 63):
            raise ValueError("JSON integer is outside signed 64-bit range")
        return parsed

    def parse_float(value: str) -> float:
        if len(value) > 64:
            raise ValueError("JSON float token is too long")
        parsed = float(value)
        if not (-float("inf") < parsed < float("inf")):
            raise ValueError("JSON float must be finite")
        return parsed

    try:
        return json.loads(
            text,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=reject_constant,
            parse_int=parse_int,
            parse_float=parse_float,
        )
    except RecursionError as exc:
        raise ValueError("JSON nesting is too deep") from exc


def _read_stable_bytes(path: str, max_bytes: int, label: str) -> bytes:
    """Read one bounded regular-file snapshot and reject concurrent mutation."""
    with open(path, "rb") as f:
        before = os.fstat(f.fileno())
        if not stat.S_ISREG(before.st_mode):
            raise ValueError(f"{label} is not a regular file")
        if before.st_size > max_bytes:
            raise ValueError(f"{label} exceeds the {max_bytes}-byte limit")
        raw = f.read(max_bytes + 1)
        after = os.fstat(f.fileno())
    if len(raw) > max_bytes:
        raise ValueError(f"{label} exceeds the {max_bytes}-byte limit")
    stable_fields = ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")
    if any(getattr(before, field) != getattr(after, field) for field in stable_fields):
        raise ValueError(f"{label} changed while it was being read")
    if len(raw) != before.st_size:
        raise ValueError(f"{label} size changed while it was being read")
    return raw


def _read_jsonl_snapshot(path: str) -> Tuple[bytes, List[Dict[str, Any]]]:
    raw = _read_stable_bytes(path, MAX_RAW_BYTES, "raw capture")
    if raw and not raw.endswith(b"\n"):
        raise ValueError("raw JSONL capture must end with a newline")
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"raw capture is not UTF-8: {exc}") from exc
    rows: List[Dict[str, Any]] = []
    for lineno, line in enumerate(text.splitlines(), 1):
        if len(line.encode("utf-8")) > MAX_JSON_LINE_BYTES:
            raise ValueError(
                f"raw capture line {lineno} exceeds the {MAX_JSON_LINE_BYTES}-byte limit"
            )
        if not line.strip():
            continue
        try:
            row = _strict_json(line)
        except (json.JSONDecodeError, ValueError) as exc:
            raise ValueError(f"raw capture line {lineno}: {exc}") from exc
        if not isinstance(row, dict):
            raise ValueError(f"raw capture line {lineno} is not an object")
        rows.append(row)
        if len(rows) > MAX_JSON_ROWS:
            raise ValueError(f"raw capture exceeds the {MAX_JSON_ROWS}-row limit")
    if not rows:
        raise ValueError("raw capture is empty")
    return raw, rows


def _load_manifest(path: str) -> Tuple[bytes, Dict[str, Any], Dict[str, Any]]:
    raw = _read_stable_bytes(path, MAX_MANIFEST_BYTES, "manifest")
    try:
        manifest = _strict_json(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise ValueError(f"manifest is not strict UTF-8 JSON: {exc}") from exc
    if not isinstance(manifest, dict):
        raise ValueError("manifest root is not an object")
    if manifest.get("schema") != MANIFEST_SCHEMA:
        raise ValueError(f"manifest schema must be {MANIFEST_SCHEMA}")
    images = manifest.get("images")
    if not isinstance(images, list) or len(images) != 4:
        raise ValueError("manifest must contain exactly four image records")
    slots = [image.get("slot") for image in images if isinstance(image, dict)]
    if len(slots) != 4 or set(slots) != {0, 1, 2, 3} or any(type(s) is not int for s in slots):
        raise ValueError("manifest image slots must be the integers 0,1,2,3 exactly once")
    slot0 = next(image for image in images if image["slot"] == 0)
    if slot0.get("tdc_abi") != TDC_ABI:
        raise ValueError(f"manifest slot 0 must declare tdc_abi={TDC_ABI}")
    if not isinstance(slot0.get("bitstream_sha256"), str) or not SHA256_RE.fullmatch(
        slot0["bitstream_sha256"]
    ):
        raise ValueError("manifest slot-0 bitstream_sha256 is not canonical")
    if not isinstance(manifest.get("blob_sha256"), str) or not SHA256_RE.fullmatch(
        manifest["blob_sha256"]
    ):
        raise ValueError("manifest blob_sha256 is not canonical")
    if not isinstance(manifest.get("timing"), dict) or set(manifest["timing"]) != {
        image["name"] for image in images
    }:
        raise ValueError("manifest timing evidence must cover all four images")
    for image in images:
        evidence = manifest["timing"][image["name"]]
        if not isinstance(evidence, dict):
            raise ValueError(f"manifest timing evidence for {image['name']} is not an object")
        if not isinstance(evidence.get("tim_sha256"), str) or not SHA256_RE.fullmatch(
            evidence["tim_sha256"]
        ):
            raise ValueError(f"manifest timing evidence for {image['name']} has no canonical tim_sha256")
        if type(evidence.get("manual_adclk_path_audit")) is not bool:
            raise ValueError(
                f"manifest timing evidence for {image['name']} has no boolean "
                "manual_adclk_path_audit"
            )
        achieved = evidence.get("achieved_mhz")
        if not isinstance(achieved, dict) or not achieved:
            raise ValueError(f"manifest timing evidence for {image['name']} has no achieved_mhz map")
        if any(type(value) not in (int, float) or value <= 0 for value in achieved.values()):
            raise ValueError(f"manifest timing evidence for {image['name']} has invalid frequencies")
    slot0_timing = manifest["timing"][slot0["name"]]
    slot0_adclk = slot0_timing["achieved_mhz"].get("adclk_clk_$glb_clk")
    if type(slot0_adclk) not in (int, float):
        raise ValueError("manifest slot-0 timing evidence has no adclk domain")
    if slot0_adclk < 40.0 and slot0_timing["manual_adclk_path_audit"] is not True:
        raise ValueError(
            "manifest slot-0 sub-40-MHz aggregate adclk requires the "
            "report-hash-bound manual path audit"
        )
    provenance = manifest.get("provenance")
    if not isinstance(provenance, dict) or provenance.get("dirty") is not False:
        raise ValueError("manifest must carry clean provenance (dirty=false)")
    return raw, manifest, slot0


def seal_capture(
    raw_path: str,
    output_path: str,
    manifest_path: str,
    expected_serial: str,
    clock_source: str,
    trigger_source: str,
    adclk_hz: int,
) -> Dict[str, str]:
    """Validate and seal one raw capture, refusing to overwrite output."""
    if not SERIAL_RE.fullmatch(expected_serial):
        raise ValueError("expected serial must be exactly 32 hexadecimal digits")
    if not isinstance(clock_source, str) or not clock_source.strip():
        raise ValueError("clock source must be a nonempty description")
    if not isinstance(trigger_source, str) or not trigger_source.strip():
        raise ValueError("trigger source must be a nonempty description")
    if type(adclk_hz) is not int or not 1_000_000 <= adclk_hz <= 100_000_000:
        raise ValueError("adclk_hz must be an integer in [1000000, 100000000]")
    if os.path.abspath(raw_path) == os.path.abspath(output_path):
        raise ValueError("output must differ from the immutable raw capture")
    if os.path.exists(output_path):
        raise FileExistsError(f"refusing to overwrite {output_path}")

    raw, rows = _read_jsonl_snapshot(raw_path)
    manifest_raw, manifest, slot0 = _load_manifest(manifest_path)
    config = rows[0]
    terminals = [row for row in rows if row.get("kind") in {"stop", "abort"}]
    if config.get("kind") != "config" or config.get("schema") != CAPTURE_SCHEMA:
        raise ValueError(f"first row must be a {CAPTURE_SCHEMA} config")
    if config.get("capture_protocol") != PROTOCOL or config.get("tdc_abi") != TDC_ABI:
        raise ValueError(f"raw capture must use {PROTOCOL} and ABI {TDC_ABI}")
    if len(terminals) != 1 or rows[-1] is not terminals[0]:
        raise ValueError("raw capture must have exactly one final stop/abort row")
    if str(config.get("serial", "")).lower() != expected_serial.lower():
        raise ValueError("raw capture serial does not match --expected-serial")
    build_text = config.get("build_id")
    if not isinstance(build_text, str) or not BUILD_RE.fullmatch(build_text):
        raise ValueError("raw capture build_id is not canonical 0xNNN")
    build_id = int(build_text, 16)
    if build_id == 0 or type(manifest.get("build_id")) is not int or manifest["build_id"] != build_id:
        raise ValueError("raw capture BUILD_ID does not match manifest")
    reg_3e, reg_3f = slot0.get("reg_3e"), slot0.get("reg_3f")
    if (type(reg_3e) is not int or type(reg_3f) is not int
            or reg_3e != (build_id >> 8) or reg_3f != (build_id & 0xff)):
        raise ValueError("manifest slot-0 BUILD_ID registers are inconsistent")

    for field in (
        "raw_capture_sha256", "manifest_sha256", "bitstream_sha256", "clock_source",
        "trigger_source", "adclk_hz",
    ):
        if config.get(field) is not None:
            raise ValueError(f"raw config {field} must be null before sealing")
    if "provenance_binding" in config:
        raise ValueError("raw config must not predeclare provenance_binding")

    config["raw_capture_sha256"] = hashlib.sha256(raw).hexdigest()
    config["manifest_sha256"] = hashlib.sha256(manifest_raw).hexdigest()
    config["bitstream_sha256"] = slot0["bitstream_sha256"].lower()
    config["clock_source"] = clock_source.strip()
    config["trigger_source"] = trigger_source.strip()
    config["adclk_hz"] = adclk_hz
    config["provenance_binding"] = "manifest-and-build-tag-smoke-test"

    sealed = ("\n".join(
        json.dumps(row, sort_keys=True, separators=(",", ":")) for row in rows
    ) + "\n").encode("utf-8")
    output_dir = os.path.dirname(os.path.abspath(output_path)) or "."
    os.makedirs(output_dir, exist_ok=True)
    fd, temp_path = tempfile.mkstemp(prefix=".tdc-seal-", dir=output_dir)
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(sealed)
            f.flush()
            os.fsync(f.fileno())
        os.link(temp_path, output_path)  # atomic and refuses an existing target
    finally:
        try:
            os.unlink(temp_path)
        except FileNotFoundError:
            pass

    return {
        "raw_capture_sha256": config["raw_capture_sha256"],
        "sealed_capture_sha256": hashlib.sha256(sealed).hexdigest(),
        "manifest_sha256": config["manifest_sha256"],
        "bitstream_sha256": config["bitstream_sha256"],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_capture")
    parser.add_argument("sealed_output")
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--expected-serial", required=True)
    parser.add_argument("--clock-source", required=True)
    parser.add_argument("--trigger-source", required=True)
    parser.add_argument("--adclk-hz", required=True, type=int)
    args = parser.parse_args()
    try:
        result = seal_capture(
            args.raw_capture,
            args.sealed_output,
            args.manifest,
            args.expected_serial,
            args.clock_source,
            args.trigger_source,
            args.adclk_hz,
        )
    except (OSError, ValueError) as exc:
        parser.exit(2, f"tdc_capture_seal: {exc}\n")
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
