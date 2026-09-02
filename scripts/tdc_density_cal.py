#!/usr/bin/env python3
"""Offline TDC occupancy characterization and gated code-density analysis.

The A2 external-trigger gateware freezes the raw 48-tap word whenever tap 0
anchors an event. A strict prefix of length 1..47 is an interior code, all 48
ones is composite full scale, and a tap-0-anchored word with internal holes is
``bubbled``. Bubbled words are genuine capture/protocol evidence, but their
popcount is descriptive only: it is not a calibrated bin or code-density
input. Tap-0-clear words, including all-zero words, are invalid A2 events.

A histogram alone cannot prove that input phase was uniform: source phase
visitation and TDC DNL are confounded. The tap-0 event-selection transfer is
also not independently measured or bounded, so a marginal first-tap sample
can still be represented late. Therefore this program reports occupancy only.
A hash-bound stimulus declaration is retained as provenance but cannot by
itself unlock absolute widths, DNL/INL, or a LUT.
"""

import argparse
import hashlib
import json
import math
import os
import re
import sys
from typing import Any, Dict, List, Optional, Tuple


DEFAULT_NUM_TAPS = 48
MIN_INTERIOR_COUNT = 50
MAX_STATIONARITY_Z = 5.0
EVIDENCE_SCHEMA = "tdc-uniform-phase-evidence-v1"
ALLOWED_UNIFORM_METHODS = {
    "external-swept-delay",
    "randomized-delay",
    "phase-tagged-model",
}
MISSING_CLOCK_ERROR = (
    "clock period unknown: provide direct clock_ns/t_clock_ns/adclk_hz "
    "metadata or --clock-ns. A commanded sample_rate is not clock-source "
    "telemetry and is never converted into an AFE/TDC clock."
)
REQUIRED_CONFIG_FIELDS = {
    "schema", "serial", "slot", "build_id", "clock_source", "trigger_source",
    "adclk_hz", "capture_protocol", "tdc_abi", "baseline_status",
    "pre_begin_mailbox_status", "begin_mailbox_status", "timeout_s",
    "ready_timeout_s", "command_timeout_s", "selftest", "time_basis",
    "usb_api_version", "manifest_sha256", "bitstream_sha256",
    "raw_capture_sha256", "provenance_binding",
}
CAPTURE_SCHEMA = "hackrf-pro-tdc-trigger-read-v2"
CAPTURE_PROTOCOL = "held-mailbox-handshake-v2"
MANIFEST_SCHEMA = "hackrf-fpga-manifest-v2"
TDC_ABI = "0xA2"
RETIRED_MANUAL_TIMING_FIELDS = (
    "manual_adclk_audit_receipt",
    "manual_adclk_timing_report",
    "manual_adclk_asc",
)
MAX_JSON_LINE_BYTES = 64 * 1024
MAX_DOCUMENT_BYTES = 4 * 1024 * 1024
MAX_JSON_NUMBER_CHARS = 128
FIRST_TAP_TRANSFER_BLOCKER = (
    "the A2 tap-0 event-selection transfer is not independently measured or "
    "bounded; a marginal tap-0 sample can be represented late, so absolute "
    "interior widths are not yet identifiable"
)
SERIAL_RE = re.compile(r"^[0-9a-f]{32}$")
BUILD_ID_RE = re.compile(r"^0x[0-9a-f]{3}$")
USB_API_RE = re.compile(r"^0x[0-9a-f]{4}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
SOURCE_SHA_RE = re.compile(r"^[0-9a-f]{40}$")
RAW_BYTES_RE = re.compile(r"^[0-9a-f]{2}(?:,[0-9a-f]{2}){5}$")
THERMO_RE = re.compile(r"^[0-9a-f]{12}$")

CONFIG_ALLOWED_FIELDS = REQUIRED_CONFIG_FIELDS | {"kind", "clock_ns", "t_clock_ns"}
EVENT_REQUIRED_FIELDS = {
    "kind", "seq", "fpga_event_seq", "wait_start_t", "t",
    "reg31_before", "reg31_after", "mailbox_status_ready",
    "mailbox_status_complete", "completion", "bytes", "thermo", "popcount",
    "code_class", "atomic_snapshot_passed", "overflow_seen", "protocol_fault",
}
STOP_REQUIRED_FIELDS = {
    "kind", "n", "last_seq", "last_fpga_event_seq", "protocol_closed",
    "overflow_seen", "protocol_fault", "postflight_artifact_ok",
    "selftest_safe_off", "trigger_rearmed",
}
SEALER_OWNED_CONFIG_FIELDS = {
    "raw_capture_sha256", "manifest_sha256", "bitstream_sha256",
    "clock_source", "trigger_source", "adclk_hz", "provenance_binding",
}
RAW_NULL_PLACEHOLDER_FIELDS = SEALER_OWNED_CONFIG_FIELDS - {"provenance_binding"}


class _DuplicateKeyError(ValueError):
    pass


def _strict_object(pairs: List[Tuple[str, Any]]) -> Dict[str, Any]:
    obj: Dict[str, Any] = {}
    for key, value in pairs:
        if key in obj:
            raise _DuplicateKeyError(f"duplicate JSON key: {key}")
        obj[key] = value
    return obj


def _bounded_json_int(text: str) -> int:
    if len(text.lstrip("-")) > MAX_JSON_NUMBER_CHARS:
        raise ValueError("JSON integer is too long")
    return int(text)


def _bounded_json_float(text: str) -> float:
    if len(text) > MAX_JSON_NUMBER_CHARS:
        raise ValueError("JSON float is too long")
    value = float(text)
    if not math.isfinite(value):
        raise ValueError("JSON float is nonfinite or overflowed")
    return value


def _reject_json_constant(text: str) -> None:
    raise ValueError(f"non-standard JSON constant: {text}")


def _strict_json_loads(text: str) -> Any:
    return json.loads(
        text,
        object_pairs_hook=_strict_object,
        parse_int=_bounded_json_int,
        parse_float=_bounded_json_float,
        parse_constant=_reject_json_constant,
    )


def _read_strict_json_document(path: str) -> Tuple[bytes, Any]:
    with open(path, "rb") as f:
        raw = f.read(MAX_DOCUMENT_BYTES + 1)
    if len(raw) > MAX_DOCUMENT_BYTES:
        raise ValueError(f"JSON document exceeds {MAX_DOCUMENT_BYTES} bytes")
    return raw, _strict_json_loads(raw.decode("utf-8"))


def _json_exact_equal(left: Any, right: Any) -> bool:
    """Compare parsed JSON values without Python's bool/int or int/float coercion."""
    if type(left) is not type(right):
        return False
    if isinstance(left, dict):
        return set(left) == set(right) and all(
            _json_exact_equal(left[key], right[key]) for key in left
        )
    if isinstance(left, list):
        return len(left) == len(right) and all(
            _json_exact_equal(a, b) for a, b in zip(left, right)
        )
    return left == right


def _verify_raw_capture_rows(
    path: str, sealed_rows: List[Tuple[int, Dict[str, Any]]]
) -> Tuple[str, int, bool, bool, str]:
    """Hash and strictly compare a raw host stream with its sealed derivative."""
    digest = hashlib.sha256()
    size = 0
    logical_index = 0
    comparison_ok = True
    comparison_status = "raw and sealed logical rows match"
    final_newline = False
    with open(path, "rb") as f:
        before = os.fstat(f.fileno())
        for lineno, raw_line in enumerate(f, 1):
            digest.update(raw_line)
            size += len(raw_line)
            final_newline = raw_line.endswith(b"\n")
            if not raw_line.strip():
                continue
            if len(raw_line) > MAX_JSON_LINE_BYTES:
                raise ValueError(
                    f"raw capture line {lineno} exceeds {MAX_JSON_LINE_BYTES} bytes"
                )
            try:
                row = _strict_json_loads(raw_line.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError,
                    OverflowError, RecursionError) as exc:
                raise ValueError(f"raw capture line {lineno}: {exc}") from exc
            if not isinstance(row, dict):
                raise ValueError(f"raw capture line {lineno} is not an object")

            if logical_index >= len(sealed_rows):
                if comparison_ok:
                    comparison_ok = False
                    comparison_status = (
                        f"raw capture has extra logical row {logical_index + 1}"
                    )
            else:
                sealed = sealed_rows[logical_index][1]
                if logical_index == 0:
                    raw_base = {
                        key: value for key, value in row.items()
                        if key not in SEALER_OWNED_CONFIG_FIELDS
                    }
                    sealed_base = {
                        key: value for key, value in sealed.items()
                        if key not in SEALER_OWNED_CONFIG_FIELDS
                    }
                    raw_placeholders_ok = all(
                        field in row and row[field] is None
                        for field in RAW_NULL_PLACEHOLDER_FIELDS
                    ) and "provenance_binding" not in row
                    if not _json_exact_equal(raw_base, sealed_base) and comparison_ok:
                        comparison_ok = False
                        comparison_status = (
                            "raw/sealed config differs outside sealer-owned provenance fields"
                        )
                    elif not raw_placeholders_ok and comparison_ok:
                        comparison_ok = False
                        comparison_status = (
                            "raw config must contain null provenance placeholders and no "
                            "provenance_binding"
                        )
                elif not _json_exact_equal(row, sealed) and comparison_ok:
                    comparison_ok = False
                    comparison_status = (
                        f"raw/sealed logical row {logical_index + 1} differs"
                    )
            logical_index += 1
        after = os.fstat(f.fileno())
    stable = (
        before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns
    ) == (
        after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns
    ) and size == before.st_size
    if not final_newline:
        raise ValueError("raw JSONL capture must end with a newline")
    if logical_index != len(sealed_rows) and comparison_ok:
        comparison_ok = False
        comparison_status = (
            f"raw capture has {logical_index} logical row(s), sealed capture has "
            f"{len(sealed_rows)}"
        )
    return digest.hexdigest(), size, stable, comparison_ok, comparison_status


def _finite_float(value: Any) -> Optional[float]:
    """Convert a JSON number to finite float without leaking overflow."""
    if type(value) not in (int, float):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def parse_thermo_bytes(hexbytes: str) -> List[int]:
    """Parse exactly the bytes supplied, LSB first within each byte."""
    if not isinstance(hexbytes, str):
        return []
    try:
        bs = [int(b.strip(), 16) for b in hexbytes.split(",") if b.strip()]
    except ValueError:
        return []
    if any(b < 0 or b > 255 for b in bs):
        return []
    return [(b >> j) & 1 for b in bs for j in range(8)]


def decode_thermometer(taps: List[int],
                       num_taps: int = DEFAULT_NUM_TAPS) -> Optional[int]:
    """Decode one exact strict-prefix word; reject short, long, or bubbled data."""
    if num_taps <= 0 or len(taps) != num_taps:
        return None
    if any(t not in (0, 1) for t in taps):
        return None
    k = 0
    while k < num_taps and taps[k] == 1:
        k += 1
    return k if all(t == 0 for t in taps[k:]) else None


def classify_a2_word(taps: List[int],
                     num_taps: int = DEFAULT_NUM_TAPS) -> Optional[Dict[str, Any]]:
    """Classify one exact A2 raw word without normalizing away its bubbles.

    Tap 0 is the hardware event anchor. An anchored nonzero word is valid even
    when later taps contain holes. Only exact strict prefixes have a
    ``density_code``; a bubbled word's popcount remains descriptive metadata.
    """
    if num_taps <= 0 or len(taps) != num_taps:
        return None
    if any(tap not in (0, 1) for tap in taps) or taps[0] != 1:
        return None
    popcount = sum(taps)
    prefix = decode_thermometer(taps, num_taps)
    if prefix == num_taps:
        code_class = "composite-full-scale"
        density_code: Optional[int] = prefix
    elif prefix is not None:
        code_class = "interior"
        density_code = prefix
    else:
        code_class = "bubbled"
        density_code = None
    return {
        "code_class": code_class,
        "popcount": popcount,
        "density_code": density_code,
        "density_eligible": density_code is not None,
    }


def _decode_row(r: Dict[str, Any],
                num_taps: int) -> Tuple[Optional[Dict[str, Any]], str]:
    """Return (event, reason); metadata/control rows are not errors."""
    if r.get("dup") is True:
        return None, "duplicate"
    if r.get("kind") in {"config", "checkpoint", "stop", "abort"}:
        return None, "control"
    if "bytes" in r:
        byte_text = r["bytes"]
        if not isinstance(byte_text, str) or not RAW_BYTES_RE.fullmatch(byte_text):
            return None, "invalid-event"
        taps = parse_thermo_bytes(byte_text)
        raw_word = True
        raw_word_hex = byte_text
        thermo_text = r.get("thermo")
        if (not isinstance(thermo_text, str)
                or not THERMO_RE.fullmatch(thermo_text)
                or thermo_text != byte_text.replace(",", "")):
            return None, "invalid-event"
    elif "thermo" in r:
        text = r["thermo"]
        if not isinstance(text, str) or len(text) != 2 * ((num_taps + 7) // 8):
            return None, "invalid-event"
        try:
            bs = [int(text[i:i + 2], 16) for i in range(0, len(text), 2)]
        except ValueError:
            return None, "invalid-event"
        taps = [(b >> j) & 1 for b in bs for j in range(8)][:num_taps]
        raw_word = True
        raw_word_hex = ",".join(f"{byte:02x}" for byte in bs)
    elif "popcount" in r:
        value = r["popcount"]
        if type(value) is not int:
            return None, "invalid-event"
        if not 0 <= value <= num_taps:
            return None, "invalid-event"
        raw_word = False
        classification = {
            "code_class": "popcount-only",
            "popcount": value,
            "density_code": None,
            "density_eligible": False,
        }
        raw_word_hex = None
    else:
        return None, "non-event"
    if raw_word:
        if not taps or taps[0] != 1:
            return None, "code0-event" if not any(taps) else "unanchored-event"
        classification = classify_a2_word(taps, num_taps)
        if classification is None:
            return None, "invalid-event"
        stored = r.get("popcount")
        if type(stored) is not int:
            return None, "invalid-event"
        if stored != classification["popcount"]:
            return None, "invalid-event"
    t = _finite_float(r.get("t"))
    if t is None:
        return None, "invalid-event"
    return {
        "t": t,
        "k": classification["density_code"],
        "popcount": classification["popcount"],
        "code_class": classification["code_class"],
        "density_eligible": classification["density_eligible"],
        "raw_word": raw_word,
        "raw_word_hex": raw_word_hex,
    }, "event"


def parse_jsonl_event(line: str,
                      num_taps: int = DEFAULT_NUM_TAPS) -> Optional[Dict[str, Any]]:
    """Parse one valid event while retaining class, popcount, and raw word."""
    try:
        r = _strict_json_loads(line.strip())
    except (json.JSONDecodeError, TypeError, ValueError, OverflowError, RecursionError):
        return None
    if not isinstance(r, dict):
        return None
    event, _ = _decode_row(r, num_taps)
    if event is None:
        return None
    return {
        "t": event["t"],
        "k": event["k"],
        "popcount": event["popcount"],
        "code_class": event["code_class"],
        "raw_word": event["raw_word_hex"],
    }


def metadata_clock_ns(r: Dict[str, Any]) -> Optional[float]:
    """Derive the TDC clock from adclk_hz and reject contradictory aliases."""
    if not isinstance(r, dict):
        return None
    try:
        raw = _finite_float(r.get("adclk_hz"))
        if raw is None or raw <= 0.0:
            return None
        value = 1e9 / raw
        for field in ("clock_ns", "t_clock_ns"):
            if field not in r:
                continue
            alias = _finite_float(r[field])
            if (alias is None or not math.isclose(
                    alias, value, rel_tol=1e-9, abs_tol=1e-9)):
                return None
    except (TypeError, ValueError, OverflowError, ZeroDivisionError):
        return None
    return value if value is not None and 1.0 <= value <= 1000.0 else None


def _stationarity(ks: List[int], num_taps: int) -> Dict[str, Any]:
    """First/second-half occupancy screen; it detects drift, not uniformity."""
    mid = len(ks) // 2
    halves = (ks[:mid], ks[mid:])
    counts = []
    for half in halves:
        h = [0] * (num_taps + 1)
        for k in half:
            h[k] += 1
        counts.append(h)
    max_z = 0.0
    worst_code = None
    n1, n2 = len(halves[0]), len(halves[1])
    if n1 and n2:
        for code in range(1, num_taps + 1):
            c1, c2 = counts[0][code], counts[1][code]
            pooled = (c1 + c2) / (n1 + n2)
            variance = pooled * (1.0 - pooled) * (1.0 / n1 + 1.0 / n2)
            z = abs(c1 / n1 - c2 / n2) / math.sqrt(variance) if variance else 0.0
            if z > max_z:
                max_z, worst_code = z, code
    return {
        "first_half_events": n1,
        "second_half_events": n2,
        "max_two_proportion_z": max_z,
        "worst_code": worst_code,
        "threshold_z": MAX_STATIONARITY_Z,
        "passes": n1 > 0 and n2 > 0 and max_z <= MAX_STATIONARITY_Z,
        "limitation": "stationarity does not prove uniform input phase",
    }


def compute_code_density(
    ks: List[int],
    num_taps: int = DEFAULT_NUM_TAPS,
    t_clock_ns: Optional[float] = None,
    uniform_phase_verified: bool = False,
    capture_gate_fails: Optional[List[str]] = None,
    capture_total_events: Optional[int] = None,
    bubbled_events: int = 0,
    popcount_only_events: int = 0,
    code0_invalid_events: int = 0,
) -> Dict[str, Any]:
    """Characterize strict-prefix occupancy and gate absolute calibration.

    ``ks`` contains only strict-prefix A2 words. Bubbled raw words remain valid
    capture events, but callers report them separately and never map their
    popcount into this histogram.
    """
    if type(num_taps) is not int or num_taps != DEFAULT_NUM_TAPS:
        return {"error": f"--taps must be exactly {DEFAULT_NUM_TAPS} for ABI {TDC_ABI}"}
    if t_clock_ns is None:
        return {"error": MISSING_CLOCK_ERROR}
    try:
        t_clock_ns = float(t_clock_ns)
    except (TypeError, ValueError, OverflowError):
        return {"error": "clock period must be numeric"}
    if not math.isfinite(t_clock_ns) or t_clock_ns <= 0.0:
        return {"error": "clock period must be finite and > 0"}
    if any(isinstance(k, bool) or not isinstance(k, int)
           or k < 0 or k > num_taps for k in ks):
        return {"error": f"event codes must be integers in [0, {num_taps}]"}
    for label, value in (
        ("bubbled_events", bubbled_events),
        ("popcount_only_events", popcount_only_events),
        ("code0_invalid_events", code0_invalid_events),
    ):
        if type(value) is not int or value < 0:
            return {"error": f"{label} must be a nonnegative integer"}
    if capture_total_events is None:
        capture_total_events = len(ks) + bubbled_events + popcount_only_events
    if (type(capture_total_events) is not int or capture_total_events < 0
            or capture_total_events < len(ks) + bubbled_events
                               + popcount_only_events):
        return {"error": "capture_total_events is inconsistent with event counts"}
    if not ks and capture_total_events == 0 and code0_invalid_events == 0:
        return {"error": "no events provided"}

    hist = [0] * (num_taps + 1)
    for k in ks:
        hist[k] += 1
    total = len(ks)
    occupancy = [c / total for c in hist] if total else [0.0] * len(hist)
    interior_codes = list(range(1, num_taps))
    interior_counts = [hist[k] for k in interior_codes]
    stationarity = _stationarity(ks, num_taps)

    gate_fails = list(capture_gate_fails or [])
    if not uniform_phase_verified:
        gate_fails.append(
            "no dataset-hash-bound independent proof of uniform input phase"
        )
    gate_fails.append(FIRST_TAP_TRANSFER_BLOCKER)
    if bubbled_events:
        gate_fails.append(
            f"{bubbled_events} tap-0-anchored bubbled event(s) were preserved "
            "as raw capture evidence but excluded from code-density input; "
            "their popcount is not a calibrated tap-bin code"
        )
    if popcount_only_events:
        gate_fails.append(
            f"{popcount_only_events} popcount-only event(s) were excluded from "
            "code-density input because their raw tap-0 anchor/class is unavailable"
        )
    if code0_invalid_events or hist[0]:
        invalid_code0 = code0_invalid_events + hist[0]
        gate_fails.append(
            f"code 0 occurred {invalid_code0} time(s); tap 0 is clear, so it "
            "is not a valid A2 external-trigger event"
        )
    low = [(code, hist[code]) for code in interior_codes
           if hist[code] < MIN_INTERIOR_COUNT]
    if low:
        gate_fails.append(
            f"{len(low)} interior code(s) below {MIN_INTERIOR_COUNT} events"
        )
    if not stationarity["passes"]:
        gate_fails.append(
            "first/second-half occupancy is not stationary at the declared gate"
        )

    result: Dict[str, Any] = {
        "schema": "tdc-occupancy-characterization-v2",
        "total_events": total,
        "capture_total_events": capture_total_events,
        "density_input_events": total,
        "excluded_from_density_events": bubbled_events + popcount_only_events,
        "bubbled_capture_events": bubbled_events,
        "bubbled_excluded_from_density_events": bubbled_events,
        "popcount_only_excluded_from_density_events": popcount_only_events,
        "density_input_policy": (
            "strict-prefix raw A2 words only; bubbled raw words are preserved "
            "but their popcount is never used as a timing code"
        ),
        "t_clock_ns": t_clock_ns,
        "hist": hist,
        "occupancy": occupancy,
        "interior_codes": interior_codes,
        "interior_events": sum(interior_counts),
        "code0_invalid_events": code0_invalid_events + hist[0],
        "overflow_composite_events": hist[num_taps],
        "overflow_composite_rate": hist[num_taps] / total if total else None,
        "edge_code_semantics": {
            "code_0": "invalid A2 event because tap 0 is clear",
            f"code_{num_taps}": (
                "all taps high: composite full scale/possible late capture; "
                "occupancy edge only, not a tap-width bin"
            ),
            "bubbled": (
                "valid tap-0-anchored raw capture with internal holes; retained "
                "for integrity/classification and excluded from code density"
            ),
        },
        "stationarity": stationarity,
        "uniform_phase_evidence_declared": bool(uniform_phase_verified),
        "absolute_calibration_valid": not gate_fails,
        "calibration_gate_fails": gate_fails,
    }

    if gate_fails:
        return result

    period_ps = t_clock_ns * 1000.0
    widths = [hist[k] / total * period_ps for k in interior_codes]
    uncertainties = [
        period_ps * math.sqrt((hist[k] / total) * (1.0 - hist[k] / total) / total)
        for k in interior_codes
    ]
    ideal = sum(widths) / len(widths)
    dnl_ps = [w - ideal for w in widths]
    dnl_lsb = [d / ideal for d in dnl_ps]
    inl_lsb = [0.0]
    for d in dnl_lsb:
        inl_lsb.append(inl_lsb[-1] + d)
    inl_ps = [v * ideal for v in inl_lsb]
    lut = [0.0]
    for width in widths:
        lut.append(lut[-1] + width)

    result.update({
        "calibration_scope": "interior codes 1..num_taps-1 only",
        "interior_bin_widths_ps": widths,
        "interior_bin_sigma_ps": uncertainties,
        "ideal_interior_lsb_ps": ideal,
        "dnl_ps": dnl_ps,
        "dnl_lsb": dnl_lsb,
        "inl_convention": (
            "cumulative interior DNL, zero at the left edge of code 1; "
            "final endpoint is zero by the interior-mean convention"
        ),
        "inl_ps": inl_ps,
        "inl_lsb": inl_lsb,
        "interior_code_edges_ps": lut,
        "max_abs_dnl_lsb": max(abs(v) for v in dnl_lsb),
        "max_abs_inl_lsb": max(abs(v) for v in inl_lsb),
        "lut_limitation": (
            "relative to the left edge of code 1; codes 0 and num_taps are "
            "unresolved and no full coarse+fine timestamp mapping is claimed"
        ),
    })
    return result


def _load_uniform_evidence(path: Optional[str], dataset_sha256: str,
                           clock_ns: float) -> Tuple[bool, Optional[Dict[str, Any]], str]:
    if not path:
        return False, None, "not supplied"
    try:
        _, evidence = _read_strict_json_document(path)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError,
            ValueError, OverflowError, RecursionError) as exc:
        return False, None, f"unreadable evidence: {exc}"
    if not isinstance(evidence, dict):
        return False, None, "evidence root is not an object"
    checks = [
        evidence.get("schema") == EVIDENCE_SCHEMA,
        evidence.get("uniform_phase_verified") is True,
        evidence.get("method") in ALLOWED_UNIFORM_METHODS,
        evidence.get("dataset_sha256") == dataset_sha256,
    ]
    evidence_clock = _finite_float(evidence.get("clock_ns"))
    checks.append(
        evidence_clock is not None
        and abs(evidence_clock - clock_ns) <= 1e-3 * clock_ns
    )
    if not all(checks):
        return False, evidence, (
            "evidence must have the v1 schema, an allowed independent method, "
            "uniform_phase_verified=true, matching dataset_sha256, and matching clock_ns"
        )
    return True, evidence, "verified"


def _load_artifact_manifest(
    path: Optional[str], config: Optional[Dict[str, Any]]
) -> Tuple[bool, Optional[Dict[str, Any]], str, Optional[str]]:
    """Validate a complete release manifest without overstating deployment proof."""
    if not path:
        return False, None, "artifact manifest not supplied", None
    try:
        raw, manifest = _read_strict_json_document(path)
        digest = hashlib.sha256(raw).hexdigest()
    except (OSError, UnicodeDecodeError, json.JSONDecodeError,
            ValueError, OverflowError, RecursionError) as exc:
        return False, None, f"unreadable artifact manifest: {exc}", None
    if not isinstance(manifest, dict) or not isinstance(config, dict):
        return False, manifest if isinstance(manifest, dict) else None, (
            "artifact manifest/config root is not an object"
        ), digest

    failures: List[str] = []
    if manifest.get("schema") != MANIFEST_SCHEMA:
        failures.append(f"manifest schema must be {MANIFEST_SCHEMA}")
    build_text = config.get("build_id")
    manifest_sha = config.get("manifest_sha256")
    bitstream_sha = config.get("bitstream_sha256")
    if not isinstance(manifest_sha, str) or not SHA256_RE.fullmatch(manifest_sha):
        failures.append("config manifest_sha256 is not canonical")
    elif digest.lower() != manifest_sha.lower():
        failures.append("artifact manifest sha256 does not match config")
    if not isinstance(bitstream_sha, str) or not SHA256_RE.fullmatch(bitstream_sha):
        failures.append("config bitstream_sha256 is not canonical")

    build_id = int(build_text, 16) if (
        isinstance(build_text, str) and BUILD_ID_RE.fullmatch(build_text)
    ) else None
    if build_id is None or build_id == 0:
        failures.append("config build_id is not a canonical nonzero 12-bit tag")
    elif type(manifest.get("build_id")) is not int or manifest["build_id"] != build_id:
        failures.append("manifest build_id does not match config")

    images = manifest.get("images")
    indexed: Dict[int, Dict[str, Any]] = {}
    if not isinstance(images, list) or len(images) != 4:
        failures.append("manifest images must contain exactly four flash slots")
    else:
        for image in images:
            if not isinstance(image, dict) or type(image.get("slot")) is not int:
                failures.append("each manifest image requires an integer slot")
                continue
            slot = image["slot"]
            if slot in indexed:
                failures.append(f"manifest contains duplicate slot {slot}")
            else:
                indexed[slot] = image
        if set(indexed) != {0, 1, 2, 3}:
            failures.append("manifest image slots must be exactly 0,1,2,3")

    image_names: List[str] = []
    for slot in sorted(indexed):
        image = indexed[slot]
        name = image.get("name")
        if not isinstance(name, str) or not name.strip():
            failures.append(f"manifest slot-{slot} name must be nonempty")
        else:
            image_names.append(name)
        reg_3e, reg_3f = image.get("reg_3e"), image.get("reg_3f")
        regs_ok = (
            type(reg_3e) is int and type(reg_3f) is int
            and 0 <= reg_3e <= 0xff and 0 <= reg_3f <= 0xff
            and (reg_3e >> 4) == slot
            and build_id is not None
            and (((reg_3e & 0x0f) << 8) | reg_3f) == build_id
        )
        if not regs_ok:
            failures.append(
                f"manifest slot-{slot} BUILD_ID registers do not match slot/config"
            )
        image_sha = image.get("bitstream_sha256")
        if not isinstance(image_sha, str) or not SHA256_RE.fullmatch(image_sha):
            failures.append(f"manifest slot-{slot} bitstream_sha256 is not canonical")
        if slot == 0:
            if (not isinstance(bitstream_sha, str)
                    or not isinstance(image_sha, str)
                    or image_sha != bitstream_sha):
                failures.append(
                    "manifest slot-0 bitstream sha256 does not match config"
                )
            if image.get("tdc_abi") != TDC_ABI:
                failures.append(f"manifest slot-0 tdc_abi must be {TDC_ABI}")
    if len(image_names) != len(set(image_names)):
        failures.append("manifest image names must be unique")

    provenance = manifest.get("provenance")
    if not isinstance(provenance, dict):
        failures.append("manifest provenance is missing")
    else:
        if provenance.get("dirty") is not False:
            failures.append("manifest provenance must record dirty=false")
        if provenance.get("source_tree") != "firmware/fpga":
            failures.append("manifest provenance source_tree must be firmware/fpga")
        if provenance.get("diff_sha256") is not None:
            failures.append("clean manifest provenance diff_sha256 must be null")
        source_sha = provenance.get("source_sha")
        if not isinstance(source_sha, str) or not SOURCE_SHA_RE.fullmatch(source_sha):
            failures.append("manifest provenance source_sha is not canonical")
        toolchain = provenance.get("toolchain")
        if not isinstance(toolchain, dict):
            failures.append("manifest provenance toolchain is missing")
        else:
            for tool in ("yosys", "nextpnr-ice40"):
                if (not isinstance(toolchain.get(tool), str)
                        or not toolchain[tool].strip()):
                    failures.append(f"manifest toolchain {tool} must be nonempty")

    blob_sha = manifest.get("blob_sha256")
    if not isinstance(blob_sha, str) or not SHA256_RE.fullmatch(blob_sha):
        failures.append("manifest blob_sha256 is not canonical")

    timing = manifest.get("timing")
    if not isinstance(timing, dict) or set(timing) != set(image_names):
        failures.append("manifest timing entries must exactly match all image names")
    else:
        for slot, name in enumerate(image_names):
            entry = timing[name]
            if not isinstance(entry, dict):
                failures.append(f"manifest timing entry {name} is not an object")
                continue
            tim_sha = entry.get("tim_sha256")
            if not isinstance(tim_sha, str) or not SHA256_RE.fullmatch(tim_sha):
                failures.append(f"manifest timing {name} tim_sha256 is not canonical")
            clocks = entry.get("achieved_mhz")
            if not isinstance(clocks, dict) or not clocks:
                failures.append(f"manifest timing {name} achieved_mhz is empty")
            elif any(
                not isinstance(clock, str) or not clock
                or _finite_float(value) is None or float(value) <= 0.0
                for clock, value in clocks.items()
            ):
                failures.append(
                    f"manifest timing {name} achieved_mhz values must be finite and > 0"
                )
            manual_audit = entry.get("manual_adclk_path_audit")
            if type(manual_audit) is not bool:
                failures.append(
                    f"manifest timing {name} manual_adclk_path_audit must be boolean"
                )
            elif manual_audit is not False:
                failures.append(
                    f"manifest timing {name} manual_adclk_path_audit must be false"
                )
            if any(entry.get(field) is not None
                   for field in RETIRED_MANUAL_TIMING_FIELDS):
                failures.append(
                    f"manifest timing {name} carries a retired manual timing "
                    "waiver receipt/artifact"
                )
            if slot == 0 and isinstance(clocks, dict):
                adclk_mhz = _finite_float(clocks.get("adclk_clk_$glb_clk"))
                if adclk_mhz is None or adclk_mhz <= 0.0:
                    failures.append(
                        "manifest slot-0 timing must record adclk_clk_$glb_clk"
                    )
                elif adclk_mhz < 40.0:
                    failures.append(
                        "manifest slot-0 aggregate adclk timing must be at least "
                        "40 MHz; manual waivers are not release evidence"
                    )

    if failures:
        return False, manifest, "; ".join(failures), digest
    return (
        True,
        manifest,
        "manifest/tag-bound; deployed identity limited to BUILD_ID/ABI postflight",
        digest,
    )


def analyze_file(
    path: str,
    num_taps: int = DEFAULT_NUM_TAPS,
    t_clock_ns: Optional[float] = None,
    uniform_phase_evidence: Optional[str] = None,
    artifact_manifest: Optional[str] = None,
    raw_capture: Optional[str] = None,
) -> Dict[str, Any]:
    """Analyze one sealed A2 config→events→stop segment, fail closed."""
    if type(num_taps) is not int or num_taps != DEFAULT_NUM_TAPS:
        return {"error": f"--taps must be exactly {DEFAULT_NUM_TAPS} for ABI {TDC_ABI}"}
    rows: List[Tuple[int, Dict[str, Any]]] = []
    malformed_lines = 0
    input_digest = hashlib.sha256()
    input_size = 0
    final_newline = False
    try:
        with open(path, "rb") as f:
            before_stat = os.fstat(f.fileno())
            for lineno, raw_line in enumerate(f, 1):
                input_digest.update(raw_line)
                input_size += len(raw_line)
                final_newline = raw_line.endswith(b"\n")
                if not raw_line.strip():
                    continue
                if len(raw_line) > MAX_JSON_LINE_BYTES:
                    malformed_lines += 1
                    continue
                line = raw_line.decode("utf-8")
                try:
                    row = _strict_json_loads(line)
                except (json.JSONDecodeError, ValueError, OverflowError,
                        RecursionError):
                    malformed_lines += 1
                    continue
                if isinstance(row, dict):
                    rows.append((lineno, row))
                else:
                    malformed_lines += 1
            after_stat = os.fstat(f.fileno())
        capture_snapshot_stable = (
            before_stat.st_dev,
            before_stat.st_ino,
            before_stat.st_size,
            before_stat.st_mtime_ns,
        ) == (
            after_stat.st_dev,
            after_stat.st_ino,
            after_stat.st_size,
            after_stat.st_mtime_ns,
        ) and input_size == before_stat.st_size
        input_sha256 = input_digest.hexdigest()
    except (OSError, UnicodeDecodeError) as exc:
        return {"error": f"cannot snapshot strict UTF-8 capture: {exc}"}

    configs = [(n, r) for n, r in rows if r.get("kind") == "config"]
    stops = [(n, r) for n, r in rows if r.get("kind") == "stop"]
    aborts = [(n, r) for n, r in rows if r.get("kind") == "abort"]
    config = configs[0][1] if len(configs) == 1 else None
    meta_clock = metadata_clock_ns(config) if config else None
    arg_clock = _finite_float(t_clock_ns) if t_clock_ns is not None else None
    if (t_clock_ns is not None
            and (arg_clock is None or not 1.0 <= arg_clock <= 1000.0)):
        return {"error": "--clock-ns must be finite and in [1, 1000] ns"}
    if meta_clock is not None:
        if arg_clock is not None and not math.isclose(
                meta_clock, arg_clock, rel_tol=1e-9, abs_tol=1e-9):
            return {"error": (
                f"clock mismatch: run metadata records {meta_clock:g} ns but "
                f"--clock-ns says {arg_clock:g} ns"
            )}
        resolved_clock, clock_source = meta_clock, "run metadata"
    elif arg_clock is not None:
        resolved_clock, clock_source = arg_clock, "--clock-ns"
    else:
        return {"error": MISSING_CLOCK_ERROR}

    events: List[Dict[str, Any]] = []
    duplicates = invalid_events = outside_segment = 0
    unknown_rows = event_kind_errors = sequence_errors = timestamp_errors = 0
    event_schema_errors = mailbox_protocol_errors = code0_events = 0
    unanchored_events = 0
    first_config_line = configs[0][0] if len(configs) == 1 else None
    stop_line = stops[0][0] if len(stops) == 1 else None
    previous_event_t: Optional[float] = None
    declared_timeout = _finite_float(config.get("timeout_s")) if config else None
    begin_status = config.get("begin_mailbox_status") if config else None
    baseline_status = config.get("baseline_status") if config else None
    expected_completion_tokens = (
        begin_status & 0x70 if type(begin_status) is int else None
    )
    expected_valid_toggle = (
        (baseline_status & 0x01) ^ 0x01
        if type(baseline_status) is int else None
    )
    if declared_timeout is None or declared_timeout <= 0.0:
        declared_timeout = 2.5
    for lineno, row in rows:
        if row.get("kind") == "event" and set(row) != EVENT_REQUIRED_FIELDS:
            event_schema_errors += 1
        event, reason = _decode_row(row, num_taps)
        if reason == "duplicate":
            duplicates += 1
        elif reason == "code0-event":
            invalid_events += 1
            code0_events += 1
        elif reason == "unanchored-event":
            invalid_events += 1
            unanchored_events += 1
        elif reason == "invalid-event":
            invalid_events += 1
        elif event is not None:
            if row.get("kind") != "event":
                event_kind_errors += 1
            try:
                seq = row["seq"]
                if type(seq) is not int or seq < 0:
                    raise ValueError
            except (KeyError, TypeError, ValueError):
                invalid_events += 1
                continue
            event["seq"] = seq
            fpga_event_seq = row.get("fpga_event_seq")
            if (type(fpga_event_seq) is not int
                    or not 0 <= fpga_event_seq <= 0xffff):
                invalid_events += 1
                continue
            event["fpga_event_seq"] = fpga_event_seq
            event["completion"] = row.get("completion")
            if (first_config_line is None or stop_line is None
                    or not (first_config_line < lineno < stop_line)):
                outside_segment += 1
            else:
                wait_start_t = _finite_float(row.get("wait_start_t"))
                event_t = _finite_float(row.get("t"))
                timing_ok = (
                    wait_start_t is not None and event_t is not None
                    and wait_start_t >= 0.0 and event_t >= 0.0
                    and wait_start_t <= event_t
                    and event_t - wait_start_t < declared_timeout
                    and (previous_event_t is None
                         or previous_event_t <= wait_start_t)
                    and (previous_event_t is None or event_t > previous_event_t)
                )
                if not timing_ok:
                    timestamp_errors += 1
                if event_t is not None:
                    previous_event_t = event_t
                reg31_before = row.get("reg31_before")
                reg31_after = row.get("reg31_after")
                mailbox_ready = row.get("mailbox_status_ready")
                mailbox_complete = row.get("mailbox_status_complete")
                completion = row.get("completion")
                protocol_ok = (
                    type(reg31_before) is int and 0 <= reg31_before <= 0xff
                    and type(reg31_after) is int and 0 <= reg31_after <= 0xff
                    and (reg31_before & 0xe6) == 0
                    and (reg31_after & 0xe6) == 0
                    and type(mailbox_ready) is int and 0 <= mailbox_ready <= 0xff
                    and type(mailbox_complete) is int
                    and 0 <= mailbox_complete <= 0xff
                    and (mailbox_ready & 0x8f) == 0x03
                    and completion in {"ack", "close"}
                    and row.get("atomic_snapshot_passed") is True
                    and row.get("overflow_seen") is False
                    and row.get("protocol_fault") is False
                )
                protocol_ok = (
                    protocol_ok
                    and (reg31_before & 0x01) == (reg31_after & 0x01)
                    and (expected_valid_toggle is None
                         or (reg31_before & 0x01) == expected_valid_toggle)
                    and (expected_completion_tokens is None
                         or (mailbox_ready & 0x70) == expected_completion_tokens)
                )
                if completion == "ack":
                    protocol_ok = (
                        protocol_ok
                        and (mailbox_complete & 0x8f) == 0x01
                        and ((mailbox_ready ^ mailbox_complete) & 0x70) == 0x10
                    )
                elif completion == "close":
                    protocol_ok = (
                        protocol_ok
                        and (mailbox_complete & 0x8f) == 0x00
                        and ((mailbox_ready ^ mailbox_complete) & 0x70) == 0x40
                    )
                if row.get("code_class") != event["code_class"]:
                    protocol_ok = False
                if not protocol_ok:
                    mailbox_protocol_errors += 1
                if type(mailbox_complete) is int:
                    expected_completion_tokens = mailbox_complete & 0x70
                if expected_valid_toggle is not None:
                    expected_valid_toggle ^= 0x01
                events.append(event)
        elif reason == "control":
            if row.get("kind") not in {"config", "stop", "abort"}:
                unknown_rows += 1
        else:
            unknown_rows += 1

    capture_fails = []
    if not capture_snapshot_stable:
        capture_fails.append("sealed capture changed while it was hashed/read")
    if input_size and not final_newline:
        capture_fails.append("sealed JSONL capture must end with a newline")
    if len(configs) != 1:
        capture_fails.append(f"expected exactly one config row, found {len(configs)}")
    if len(stops) != 1:
        capture_fails.append(f"expected exactly one stop row, found {len(stops)}")
    if aborts:
        capture_fails.append(f"capture contains {len(aborts)} abort row(s)")
    if rows and (len(configs) != 1 or rows[0][0] != configs[0][0]):
        capture_fails.append("config row must be the first nonblank row")
    if rows and (len(stops) != 1 or rows[-1][0] != stops[0][0]):
        capture_fails.append("stop row must be the final nonblank row")
    if malformed_lines:
        capture_fails.append(f"capture contains {malformed_lines} malformed/truncated row(s)")
    if duplicates:
        capture_fails.append(f"capture contains {duplicates} duplicate-tagged row(s)")
    if invalid_events:
        capture_fails.append(f"capture contains {invalid_events} invalid event row(s)")
    if unknown_rows:
        capture_fails.append(f"capture contains {unknown_rows} unknown/non-event row(s)")
    if event_kind_errors:
        capture_fails.append(
            f"capture contains {event_kind_errors} event row(s) without kind='event'"
        )
    if event_schema_errors:
        capture_fails.append(
            f"capture contains {event_schema_errors} event row(s) outside the exact v2 schema"
        )
    if mailbox_protocol_errors:
        capture_fails.append(
            f"capture contains {mailbox_protocol_errors} invalid held-mailbox "
            "snapshot/completion proof(s) or invalid code-class metadata"
        )
    if timestamp_errors:
        capture_fails.append(
            f"capture contains {timestamp_errors} missing, nonfinite, regressing, "
            "or over-deadline monotonic timestamp pair(s)"
        )
    if outside_segment:
        capture_fails.append(f"capture contains {outside_segment} event(s) outside config→stop")
    if code0_events:
        capture_fails.append(
            f"capture contains {code0_events} invalid all-zero/code-0 event(s)"
        )
    if unanchored_events:
        capture_fails.append(
            f"capture contains {unanchored_events} nonzero tap-0-clear event(s)"
        )
    seqs = [event["seq"] for event in events]
    if seqs != list(range(len(events))):
        sequence_errors += 1
        capture_fails.append("event sequence must be contiguous 0..n-1 in file order")
    fpga_seqs = [event["fpga_event_seq"] for event in events]
    expected_fpga_seqs = [i & 0xffff for i in range(len(events))]
    if fpga_seqs != expected_fpga_seqs:
        sequence_errors += 1
        capture_fails.append(
            "FPGA event sequence must start at 0 and increment modulo 65536"
        )
    completions = [event["completion"] for event in events]
    expected_completions = (
        ["ack"] * (len(events) - 1) + ["close"] if events else []
    )
    if completions != expected_completions:
        sequence_errors += 1
        capture_fails.append(
            "event completion must be ack except for one final close"
        )
    raw_word_events = sum(bool(event["raw_word"]) for event in events)
    popcount_only_events = len(events) - raw_word_events
    bubbled_events = sum(
        event["code_class"] == "bubbled" for event in events
    )
    density_codes = [
        event["k"] for event in events if event["density_eligible"]
    ]
    if popcount_only_events:
        capture_fails.append(
            f"capture contains {popcount_only_events} popcount-only event(s); "
            "raw six-byte words are required to audit the tap-0 anchor and A2 class"
        )
    if config:
        missing = sorted(k for k in REQUIRED_CONFIG_FIELDS
                         if k not in config or config[k] in (None, ""))
        if missing:
            capture_fails.append("config missing provenance fields: " + ", ".join(missing))
        unknown_config = sorted(set(config) - CONFIG_ALLOWED_FIELDS)
        if unknown_config:
            capture_fails.append(
                "config contains fields outside the exact v2 schema: "
                + ", ".join(unknown_config)
            )
        if type(config.get("slot")) is not int or config.get("slot") != 0:
            capture_fails.append("occupancy capture config requires integer slot 0")
        if config.get("schema") != CAPTURE_SCHEMA:
            capture_fails.append(
                f"config schema must be {CAPTURE_SCHEMA}; v1 is not claim-grade"
            )
        if config.get("capture_protocol") != CAPTURE_PROTOCOL:
            capture_fails.append(
                f"config capture_protocol must be {CAPTURE_PROTOCOL}"
            )
        if config.get("tdc_abi") != TDC_ABI:
            capture_fails.append(f"config tdc_abi must be {TDC_ABI}")
        if config.get("provenance_binding") != "manifest-and-build-tag-smoke-test":
            capture_fails.append(
                "config provenance_binding must disclose manifest/tag smoke-test scope"
            )
        baseline = config.get("baseline_status")
        if (type(baseline) is not int or not 0 <= baseline <= 0xff
                or (baseline & 0xfe) != 0x10):
            capture_fails.append(
                "config baseline_status must prove busy=0, selftest=0, "
                "trigger-low, armed=1, and reserved bits zero"
            )
        pre_begin = config.get("pre_begin_mailbox_status")
        begin = config.get("begin_mailbox_status")
        if (type(pre_begin) is not int or not 0 <= pre_begin <= 0xff
                or (pre_begin & 0x8f) != 0x00):
            capture_fails.append(
                "config pre_begin_mailbox_status must be clean and inactive"
            )
        if (type(begin) is not int or not 0 <= begin <= 0xff
                or (begin & 0x8f) != 0x01):
            capture_fails.append(
                "config begin_mailbox_status must be clean and active"
            )
        if (type(pre_begin) is int and type(begin) is int
                and ((pre_begin ^ begin) & 0x70) != 0x20):
            capture_fails.append(
                "config begin_mailbox_status must prove only BEGIN_DONE changed"
            )
        timeout_s = _finite_float(config.get("timeout_s"))
        if (timeout_s is None
                or not math.isclose(timeout_s, 2.5, rel_tol=0.0, abs_tol=1e-12)):
            capture_fails.append("config timeout_s must be exactly 2.5")
        ready_timeout_s = _finite_float(config.get("ready_timeout_s"))
        if (ready_timeout_s is None
                or not math.isclose(ready_timeout_s, 1.5,
                                    rel_tol=0.0, abs_tol=1e-12)):
            capture_fails.append("config ready_timeout_s must be exactly 1.5")
        command_timeout_s = _finite_float(config.get("command_timeout_s"))
        if (command_timeout_s is None
                or not math.isclose(command_timeout_s, 1.5,
                                    rel_tol=0.0, abs_tol=1e-12)):
            capture_fails.append("config command_timeout_s must be exactly 1.5")
        if config.get("selftest") is not False:
            capture_fails.append("config selftest must be false")
        if config.get("time_basis") != "host-monotonic-after-mailbox-ready":
            capture_fails.append(
                "config time_basis must be host-monotonic-after-mailbox-ready"
            )
        for field in ("clock_source", "trigger_source"):
            if not isinstance(config.get(field), str) or not config[field].strip():
                capture_fails.append(f"config {field} must be a nonempty string")
        if (not isinstance(config.get("serial"), str)
                or not SERIAL_RE.fullmatch(config["serial"])
                or set(config["serial"]) == {"0"}):
            capture_fails.append(
                "config serial must be the nonzero canonical 32-digit device serial"
            )
        if (not isinstance(config.get("build_id"), str)
                or not BUILD_ID_RE.fullmatch(config["build_id"])
                or int(config["build_id"], 16) == 0):
            capture_fails.append(
                "config build_id must be canonical 0xNNN and nonzero"
            )
        usb_api = config.get("usb_api_version")
        if (not isinstance(usb_api, str) or not USB_API_RE.fullmatch(usb_api)
                or int(usb_api, 16) < 0x0117):
            capture_fails.append(
                "config usb_api_version must be canonical and at least 0x0117"
            )
        if metadata_clock_ns(config) is None:
            capture_fails.append(
                "config adclk_hz must be valid and any direct period aliases must agree"
            )
        for field in ("manifest_sha256", "bitstream_sha256", "raw_capture_sha256"):
            if (not isinstance(config.get(field), str)
                    or not SHA256_RE.fullmatch(config[field])):
                capture_fails.append(f"config {field} must be canonical lowercase SHA-256")
    if len(stops) == 1:
        stop = stops[0][1]
        if set(stop) != STOP_REQUIRED_FIELDS:
            capture_fails.append("stop row must use the exact v2 terminal schema")
        try:
            stop_n = stop["n"]
            if type(stop_n) is not int or stop_n < 0:
                raise ValueError
            if stop_n != len(events):
                capture_fails.append(
                    f"stop count {stop['n']} != accepted events {len(events)}"
                )
        except (KeyError, TypeError, ValueError):
            capture_fails.append("stop row requires an exact nonnegative integer n")
        stop_last_seq = stop.get("last_seq")
        if type(stop_last_seq) is not int or stop_last_seq != len(events) - 1:
            capture_fails.append(
                "stop row last_seq must exactly equal accepted event count minus one"
            )
        stop_fpga_seq = stop.get("last_fpga_event_seq")
        expected_stop_fpga_seq = fpga_seqs[-1] if fpga_seqs else -1
        if type(stop_fpga_seq) is not int or stop_fpga_seq != expected_stop_fpga_seq:
            capture_fails.append(
                "stop row last_fpga_event_seq must equal the final accepted FPGA sequence"
            )
        for field in (
            "protocol_closed", "postflight_artifact_ok", "selftest_safe_off",
            "trigger_rearmed",
        ):
            if stop.get(field) is not True:
                capture_fails.append(f"stop row requires {field}=true")
        for field in ("overflow_seen", "protocol_fault"):
            if stop.get(field) is not False:
                capture_fails.append(f"stop row requires {field}=false")

    raw_capture_digest = None
    raw_capture_size = None
    raw_capture_status = "not supplied"
    if raw_capture is None:
        capture_fails.append(
            "raw capture sidecar is required to verify config raw_capture_sha256"
        )
    else:
        try:
            (raw_capture_digest, raw_capture_size, raw_stable,
             raw_rows_match, raw_rows_status) = _verify_raw_capture_rows(
                 raw_capture, rows
             )
            if not raw_stable:
                raw_capture_status = "raw capture changed while being hashed"
                capture_fails.append(raw_capture_status)
            elif (not isinstance(config, dict)
                    or config.get("raw_capture_sha256") != raw_capture_digest):
                raw_capture_status = "raw capture sha256 does not match config"
                capture_fails.append(raw_capture_status)
            elif not raw_rows_match:
                raw_capture_status = raw_rows_status
                capture_fails.append("raw/sealed provenance: " + raw_rows_status)
            else:
                raw_capture_status = "verified hash and exact sealed derivative"
        except (OSError, ValueError) as exc:
            raw_capture_status = f"cannot hash raw capture: {exc}"
            capture_fails.append(raw_capture_status)

    artifact_ok, manifest, artifact_status, manifest_digest = _load_artifact_manifest(
        artifact_manifest, config
    )
    if not artifact_ok:
        capture_fails.append("artifact provenance: " + artifact_status)

    uniform_ok, evidence, evidence_status = _load_uniform_evidence(
        uniform_phase_evidence, input_sha256, float(resolved_clock)
    )
    result = compute_code_density(
        density_codes, num_taps, float(resolved_clock), uniform_ok,
        capture_fails, capture_total_events=len(events),
        bubbled_events=bubbled_events,
        popcount_only_events=popcount_only_events,
        code0_invalid_events=code0_events,
    )
    if "error" in result:
        return result
    result.update({
        "input": os.path.abspath(path),
        "input_sha256": input_sha256,
        "t_clock_source": clock_source,
        "capture_integrity_valid": not capture_fails,
        "artifact_manifest_status": artifact_status,
        "artifact_manifest_sha256": manifest_digest,
        "artifact_manifest": manifest,
        "raw_capture": os.path.abspath(raw_capture) if raw_capture else None,
        "raw_capture_sha256": raw_capture_digest,
        "raw_capture_size": raw_capture_size,
        "raw_capture_status": raw_capture_status,
        "capture": {
            "config_rows": len(configs),
            "stop_rows": len(stops),
            "abort_rows": len(aborts),
            "duplicate_rows": duplicates,
            "invalid_event_rows": invalid_events,
            "unknown_rows": unknown_rows,
            "event_kind_errors": event_kind_errors,
            "event_schema_errors": event_schema_errors,
            "mailbox_protocol_errors": mailbox_protocol_errors,
            "code0_events": code0_events,
            "unanchored_events": unanchored_events,
            "bubbled_events": bubbled_events,
            "density_input_events": len(density_codes),
            "timestamp_errors": timestamp_errors,
            "sequence_errors": sequence_errors,
            "malformed_rows": malformed_lines,
            "events_outside_segment": outside_segment,
            "raw_word_events": raw_word_events,
            "popcount_only_events": popcount_only_events,
            "raw_word_preservation": (
                "immutable hash-bound JSONL retained verbatim; bubbled words "
                "are classified from the raw bits and never normalized to a bin"
            ),
            "config": config,
        },
        "uniform_phase_evidence_status": (
            evidence_status + "; declaration alone is not a calibration proof"
            if evidence_status == "verified" else evidence_status
        ),
        "uniform_phase_evidence": evidence,
    })
    return result


def main() -> None:
    parser = argparse.ArgumentParser(
        description="TDC occupancy characterization with fail-closed calibration gates"
    )
    parser.add_argument("input", help="input JSONL capture")
    parser.add_argument("--taps", type=int, default=DEFAULT_NUM_TAPS)
    parser.add_argument("--clock-ns", type=float, default=None)
    parser.add_argument(
        "--uniform-phase-evidence",
        help=(
            "hash-bound stimulus declaration retained as provenance; it cannot "
            "overcome the current tap-0 transfer calibration blocker"
        ),
    )
    parser.add_argument(
        "--artifact-manifest",
        help="release manifest whose hash and slot-0 bitstream bind the capture",
    )
    parser.add_argument(
        "--raw-capture",
        help="immutable raw host JSONL whose SHA-256 is declared by the sealed config",
    )
    parser.add_argument(
        "--out-json",
        help="reserved calibrated-output path; refused while any absolute gate fails",
    )
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument(
        "--require-calibration",
        action="store_true",
        help="exit 3 unless absolute calibration (not just capture integrity) is valid",
    )
    args = parser.parse_args()

    if not os.path.exists(args.input):
        sys.exit(f"Error: input file '{args.input}' not found")
    result = analyze_file(
        args.input,
        args.taps,
        args.clock_ns,
        args.uniform_phase_evidence,
        args.artifact_manifest,
        args.raw_capture,
    )
    if "error" in result:
        sys.exit(f"Error: {result['error']}")

    if not args.quiet:
        print(f"=== TDC occupancy characterization: {args.input} ===")
        print(f"Clock period: {result['t_clock_ns']:g} ns "
              f"(source: {result.get('t_clock_source', 'unknown')})")
        print(f"Protocol-valid capture events: {result['capture_total_events']}")
        print(f"Strict-prefix density inputs: {result['density_input_events']}")
        print(f"Bubbled raw events (excluded from density): "
              f"{result['bubbled_capture_events']}")
        print(f"Interior codes 1..{args.taps - 1}: {result['interior_events']}")
        print(f"Code 0 invalid: {result['code0_invalid_events']}")
        print(f"Code {args.taps} composite full scale: "
              f"{result['overflow_composite_events']}")
        print("Absolute calibration: " + (
            "VALID" if result["absolute_calibration_valid"] else "NOT ESTABLISHED"
        ))
        print("Capture integrity: " + (
            "VALID" if result["capture_integrity_valid"] else "FAILED"
        ))
        for reason in result["calibration_gate_fails"]:
            print(f"  - {reason}")
        if result["absolute_calibration_valid"]:
            print(f"Interior ideal LSB: {result['ideal_interior_lsb_ps']:.3f} ps")
            print(f"Max |DNL|: {result['max_abs_dnl_lsb']:.3f} LSB")
            print(f"Max |INL|: {result['max_abs_inl_lsb']:.3f} LSB")

    if not result["capture_integrity_valid"]:
        raise SystemExit(2)
    if ((args.require_calibration or args.out_json)
            and not result["absolute_calibration_valid"]):
        if args.out_json:
            print(
                "Refusing --out-json: absolute calibration gates did not pass. "
                "Relative occupancy was characterized, but no LUT is valid.",
                file=sys.stderr,
            )
        raise SystemExit(3)
    if args.out_json:
        try:
            with open(args.out_json, "w", encoding="utf-8") as f:
                json.dump(result, f, indent=2, allow_nan=False)
        except (OSError, ValueError) as exc:
            print(f"Error: cannot write calibrated output: {exc}", file=sys.stderr)
            raise SystemExit(1)
        if not args.quiet:
            print(f"Calibrated interior result written to {args.out_json}")


if __name__ == "__main__":
    main()
