#!/usr/bin/env python3
"""Unit tests for carrier_phase_analyzer gap-bridging and trailing window logic.

Verifies Order 4 remediation:
1. Audited single-row exclusion bridge rule:
   - If an A/B membership mismatch lasts <= 1 epoch (<= 2.5 s) with phase lock
     resumed immediately, continuity ID preserved, and no cycle slip, the gap
     is bridged rather than resetting continuous lock counter to zero.
2. Invariant enforcement:
   - Multi-row mismatches (> 1 epoch / > 2.5 s) refuse bridge and split.
   - Discontinuity in continuity_id refuses bridge and splits.
   - Cycle slip during or after gap refuses bridge and splits.
   - Loss of phase lock (lock=False) refuses bridge and splits.
   - Gaps exceeding 2.5 s threshold refuse bridge and split.
3. Deterministic trailing rolling window:
   - Evaluates strictly against trailing 3600s window without historical cherry-picking.
"""
import json
import math
import os
import sys
import tempfile
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import carrier_phase_analyzer as cpa


def test_single_row_ab_mismatch_bridged_reaches_3600s():
    """Verify that ~23 single-row A/B mismatches per hour are bridged,

    resolving the 1787 s ceiling to reach a continuous 3600 s quality hour.
    """
    n_epochs = 3650
    # Inject 23 single-row A/B mismatches periodically (~every 150s)
    mismatch_indices = set(range(150, n_epochs - 50, 150))
    assert len(mismatch_indices) >= 23

    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as f:
        for i in range(n_epochs):
            t = 1000.0 + float(i)
            is_mismatch = (i in mismatch_indices)
            row = {
                "t": t,
                "disp_mm": 0.3 * math.sin(i * 0.005),
                "sigma_mm": 0.5,
                "freq_off_hz": -31.5,
                "lock": True,
                "ab_membership_match": not is_mismatch,
                "continuity_id": "arc_waas_001",
                "slip": False
            }
            f.write(json.dumps(row) + "\n")
        tmp_path = f.name

    try:
        # 1. Without bridging: should split into ~24 short segments (~150 s each),
        # FAILING the 3600s span gate (reproducing the pre-fix 1787s ceiling blocker)
        rep_unbridged = cpa.analyze_carrier_file(
            tmp_path, min_span=3600.0, min_rows=3400, bridge_ab_mismatches=False
        )
        assert rep_unbridged["status"] == "FAIL"
        assert rep_unbridged["gates"]["span_ge_3600s"] is False
        assert rep_unbridged["span_s"] < 200.0  # bounded by ~150s gap spacing

        # 2. With audited single-row bridging: bridges all 23 exclusions,
        # yielding a continuous 3600s quality hour
        rep_bridged = cpa.analyze_carrier_file(
            tmp_path, min_span=3600.0, min_rows=3400, bridge_ab_mismatches=True
        )
        assert rep_bridged["status"] == "PASS"
        assert rep_bridged["gates"]["span_ge_3600s"] is True
        assert rep_bridged["gates"]["rows_ge_3400"] is True
        assert rep_bridged["gates"]["max_gap_le_5s"] is True
        assert rep_bridged["gates"]["rms_lt_1ns"] is True
        assert rep_bridged["gates"]["tdev_lt_1ns"] is True
        assert rep_bridged["span_s"] >= 3600.0

        # Audit trail verification
        audit = rep_bridged["audit"]
        assert audit["n_bridged_gaps"] >= 23
        assert audit["n_ab_mismatch_excluded"] >= 23
        assert audit["cherry_picking_prohibited"] is True
        assert audit["window_mode"] == "deterministic_trailing_rolling_window"
        # Each bridged entry has audit metadata
        for b in audit["bridges"]:
            assert b["bridged"] is True
            assert b["continuity_id"] == "arc_waas_001"
            assert b["reason"] == "ab_membership_mismatch_single_row"
            assert b["gap_s"] <= 2.5
    finally:
        os.remove(tmp_path)


def test_multi_row_mismatch_fails_bridge_and_splits():
    """Verify that multi-epoch A/B mismatches (>1 epoch / >2.5s) are NOT bridged."""
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as f:
        # 100 rows before gap
        for i in range(100):
            f.write(json.dumps({
                "t": 1000.0 + i, "disp_mm": 0.1, "lock": True,
                "ab_membership_match": True, "continuity_id": "arc_001", "slip": False
            }) + "\n")
        # 2 consecutive mismatch rows (>1 epoch)
        f.write(json.dumps({
            "t": 1100.0, "disp_mm": 0.1, "lock": True,
            "ab_membership_match": False, "continuity_id": "arc_001", "slip": False
        }) + "\n")
        f.write(json.dumps({
            "t": 1101.0, "disp_mm": 0.1, "lock": True,
            "ab_membership_match": False, "continuity_id": "arc_001", "slip": False
        }) + "\n")
        # 100 rows after gap (gap = 3.0s between t=1099 and t=1102)
        for i in range(100):
            f.write(json.dumps({
                "t": 1102.0 + i, "disp_mm": 0.1, "lock": True,
                "ab_membership_match": True, "continuity_id": "arc_001", "slip": False
            }) + "\n")
        tmp_path = f.name

    try:
        rep = cpa.analyze_carrier_file(tmp_path, min_span=150.0, min_rows=80, bridge_ab_mismatches=True)
        # Should split because multi-row mismatch exceeds single-row bridge invariant
        # Segment 0: 1000..1099 (span 99s); Segment 1: 1102..1201 (span 99s)
        # Neither segment alone reaches 150s
        assert rep["gates"]["span_ge_3600s"] is False
        assert rep["audit"]["n_bridged_gaps"] == 0
        assert any("multi_epoch" in s["reason"] or "exceeds" in s["reason"] for s in rep["audit"]["splits"])
    finally:
        os.remove(tmp_path)


def test_continuity_id_mismatch_blocks_bridge():
    """Verify that a change in continuity_id across a gap prevents bridging."""
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as f:
        for i in range(100):
            f.write(json.dumps({
                "t": 1000.0 + i, "disp_mm": 0.1, "lock": True,
                "ab_membership_match": True, "continuity_id": "arc_001", "slip": False
            }) + "\n")
        # 1 mismatch row with arc_001
        f.write(json.dumps({
            "t": 1100.0, "disp_mm": 0.1, "lock": True,
            "ab_membership_match": False, "continuity_id": "arc_001", "slip": False
        }) + "\n")
        # Resume with different continuity ID arc_002
        for i in range(100):
            f.write(json.dumps({
                "t": 1101.0 + i, "disp_mm": 0.1, "lock": True,
                "ab_membership_match": True, "continuity_id": "arc_002", "slip": False
            }) + "\n")
        tmp_path = f.name

    try:
        rep = cpa.analyze_carrier_file(tmp_path, min_span=150.0, min_rows=80, bridge_ab_mismatches=True)
        assert rep["audit"]["n_bridged_gaps"] == 0
        assert any("continuity_id" in s["reason"] for s in rep["audit"]["splits"])
    finally:
        os.remove(tmp_path)


def test_cycle_slip_blocks_bridge():
    """Verify that a cycle slip flag (slip=True) prevents bridging."""
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as f:
        for i in range(100):
            f.write(json.dumps({
                "t": 1000.0 + i, "disp_mm": 0.1, "lock": True,
                "ab_membership_match": True, "continuity_id": "arc_001", "slip": False
            }) + "\n")
        # 1 mismatch row
        f.write(json.dumps({
            "t": 1100.0, "disp_mm": 0.1, "lock": True,
            "ab_membership_match": False, "continuity_id": "arc_001", "slip": False
        }) + "\n")
        # Resume with cycle slip flagged
        f.write(json.dumps({
            "t": 1101.0, "disp_mm": 0.1, "lock": True,
            "ab_membership_match": True, "continuity_id": "arc_001", "slip": True
        }) + "\n")
        for i in range(1, 100):
            f.write(json.dumps({
                "t": 1101.0 + i, "disp_mm": 0.1, "lock": True,
                "ab_membership_match": True, "continuity_id": "arc_001", "slip": False
            }) + "\n")
        tmp_path = f.name

    try:
        rep = cpa.analyze_carrier_file(tmp_path, min_span=150.0, min_rows=80, bridge_ab_mismatches=True)
        assert rep["audit"]["n_bridged_gaps"] == 0
    finally:
        os.remove(tmp_path)


def test_loss_of_phase_lock_blocks_bridge():
    """Verify that loss of phase lock (lock=False) is treated as a real break, not an A/B bridge."""
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as f:
        for i in range(100):
            f.write(json.dumps({
                "t": 1000.0 + i, "disp_mm": 0.1, "lock": True,
                "ab_membership_match": True, "continuity_id": "arc_001", "slip": False
            }) + "\n")
        # 1 row with lock lost (lock=False, ab_membership_match=True)
        f.write(json.dumps({
            "t": 1100.0, "disp_mm": 0.1, "lock": False,
            "ab_membership_match": True, "continuity_id": "arc_001", "slip": False
        }) + "\n")
        # Resume
        for i in range(100):
            f.write(json.dumps({
                "t": 1101.0 + i, "disp_mm": 0.1, "lock": True,
                "ab_membership_match": True, "continuity_id": "arc_001", "slip": False
            }) + "\n")
        tmp_path = f.name

    try:
        rep = cpa.analyze_carrier_file(tmp_path, min_span=150.0, min_rows=80, bridge_ab_mismatches=True)
        assert rep["audit"]["n_bridged_gaps"] == 0
        assert any("loss_of_phase_lock" in s["reason"] or "not_ab_mismatch" in s["reason"] for s in rep["audit"]["splits"])
    finally:
        os.remove(tmp_path)


def test_gap_exceeding_2_5s_blocks_bridge():
    """Verify that a time hole > 2.5 s is rejected for bridging."""
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as f:
        for i in range(100):
            f.write(json.dumps({
                "t": 1000.0 + i, "disp_mm": 0.1, "lock": True,
                "ab_membership_match": True, "continuity_id": "arc_001", "slip": False
            }) + "\n")
        # Resume with 2.8 s gap (1099.0 to 1101.8)
        for i in range(100):
            f.write(json.dumps({
                "t": 1101.8 + i, "disp_mm": 0.1, "lock": True,
                "ab_membership_match": True, "continuity_id": "arc_001", "slip": False
            }) + "\n")
        tmp_path = f.name

    try:
        rep = cpa.analyze_carrier_file(tmp_path, min_span=150.0, min_rows=80, bridge_ab_mismatches=True)
        assert rep["audit"]["n_bridged_gaps"] == 0
        assert any("exceeds_max_bridge" in s["reason"] for s in rep["audit"]["splits"])
    finally:
        os.remove(tmp_path)


def test_prefiltered_single_epoch_gap_bridged():
    """Verify that pre-filtered single-row holes (2.08s <= 2.5s) are bridged when continuity preserved."""
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as f:
        # Pre-filtered data: row at 1100 is omitted, gap is 2.0 s
        for i in range(100):
            f.write(json.dumps({
                "t": 1000.0 + i, "disp_mm": 0.1, "lock": True,
                "ab_membership_match": True, "continuity_id": "arc_001", "slip": False
            }) + "\n")
        for i in range(100):
            f.write(json.dumps({
                "t": 1101.0 + i, "disp_mm": 0.1, "lock": True,
                "ab_membership_match": True, "continuity_id": "arc_001", "slip": False
            }) + "\n")
        tmp_path = f.name

    try:
        rep = cpa.analyze_carrier_file(tmp_path, min_span=150.0, min_rows=140, bridge_ab_mismatches=True)
        assert rep["status"] == "PASS"
        assert rep["span_s"] >= 150.0
        assert rep["audit"]["n_bridged_gaps"] == 1
    finally:
        os.remove(tmp_path)


def test_deterministic_trailing_rolling_window_no_cherry_picking():
    """Verify that the analyzer evaluates strictly against trailing 3600s,

    refusing to cherry-pick a quieter earlier slice.
    """
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as f:
        # Hour 1 (0..3600): super clean (0.01 mm noise -> ~0.03 ps RMS)
        for i in range(3600):
            f.write(json.dumps({
                "t": 1000.0 + i, "disp_mm": 0.01 * math.sin(i * 0.01),
                "lock": True, "ab_membership_match": True, "continuity_id": "arc_001"
            }) + "\n")
        # Hour 2 (3600..7200): higher noise (0.15 mm noise -> ~0.50 ps RMS)
        for i in range(3600, 7200):
            f.write(json.dumps({
                "t": 1000.0 + i, "disp_mm": 0.15 * math.sin(i * 0.01),
                "lock": True, "ab_membership_match": True, "continuity_id": "arc_001"
            }) + "\n")
        tmp_path = f.name

    try:
        rep = cpa.analyze_carrier_file(tmp_path, min_span=3600.0, min_rows=3400)
        assert rep["status"] == "PASS"
        assert rep["span_s"] == 3600.0
        # The evaluated window must be the trailing 3600s (epochs 4600..8199)
        # Its RMS will be ~0.35..0.50 ps, NOT the earlier ~0.03 ps
        assert rep["detrended_rms_ps"] > 0.20
        assert rep["audit"]["window_mode"] == "deterministic_trailing_rolling_window"
        assert rep["audit"]["cherry_picking_prohibited"] is True
    finally:
        os.remove(tmp_path)
