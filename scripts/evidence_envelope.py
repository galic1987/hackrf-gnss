#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/evidence_envelope.py
============================
Unified Evidence Envelope & Provenance Contract for GNSS Metrology Suite.

Implements the 4-layer architectural contract:
  1. Immutable instrument observations (OBSERVED)
  2. Versioned calibration records (CALIBRATED)
  3. Derived scientific products with complete lineage (DERIVED / MODEL)
  4. Presentation contract that quarantines or rejects results failing verification (SIMULATION / QUARANTINED)

Author: Antigravity Agent & Metrological Architecture Team
"""

import time
from enum import Enum
from typing import Dict, Any, List, Optional

class ClaimClass(str, Enum):
    OBSERVED = "observed"       # Direct physical measurement from hardware (HackRF, ADC, NCO)
    DERIVED = "derived"         # Algorithmic combination of direct observations (e.g. DD, Hatch, EKF)
    MODEL = "model"             # Astrodynamic/geophysical equations evaluated at station coords/time
    SIMULATION = "simulation"   # Synthetic/numerical simulation (QUARANTINED from live evidence)

DEFAULT_RECEIVER_TOPOLOGY = (
    "HackRF One (S/N 00000000000000006450...) + HackRF Pro (S/N 0000000000000000922c...) "
    "Matched-Cable Split-Star 10MHz/1PPS (CLKIN + P28.16 TRIGGER.IN)"
)
DEFAULT_FIRMWARE_REV = "fpga_slot2_extprec_v2.1"
DEFAULT_SOFTWARE_VERSION = "hackrf_gnss 0.1.0"

def make_evidence_envelope(
    claim_class: ClaimClass,
    generation_epoch: Optional[float] = None,
    observation_epoch: Optional[float] = None,
    input_epochs: Optional[Dict[str, float]] = None,
    permitted_skew_s: float = 5.0,
    receiver_topology: str = DEFAULT_RECEIVER_TOPOLOGY,
    firmware_revision: str = DEFAULT_FIRMWARE_REV,
    software_version: str = DEFAULT_SOFTWARE_VERSION,
    continuity_id: Optional[str] = None,
    slip_counter: int = 0,
    sample_sequence: Optional[int] = None,
    calibration_id: Optional[str] = None,
    uncertainty: Optional[Dict[str, Any]] = None,
    validity: bool = True,
    failure_reasons: Optional[List[str]] = None,
    quarantined: Optional[bool] = None,
    source_artifact_hashes: Optional[Dict[str, str]] = None
) -> Dict[str, Any]:
    """
    Construct a validated evidence envelope adhering to the station metrological contract.
    """
    now = time.time()
    gen_epoch = generation_epoch if generation_epoch is not None else now
    
    # Classify claim and enforce quarantine invariants
    is_simulation = (claim_class == ClaimClass.SIMULATION)
    if is_simulation:
        if quarantined is None:
            quarantined = True
        if failure_reasons is None:
            failure_reasons = []
        if "SYNTHETIC_NUMERICAL_SIMULATION" not in failure_reasons:
            failure_reasons.append("SYNTHETIC_NUMERICAL_SIMULATION")
        validity = False
        obs_epoch = None
    else:
        obs_epoch = observation_epoch if observation_epoch is not None else gen_epoch
        if quarantined is None:
            quarantined = False
        if failure_reasons is None:
            failure_reasons = []

    # Check input epoch skew if inputs are provided
    if input_epochs and obs_epoch is not None:
        for src_name, in_epoch in input_epochs.items():
            skew = abs(obs_epoch - in_epoch)
            if skew > permitted_skew_s:
                validity = False
                failure_reasons.append(f"INPUT_EPOCH_SKEW_EXCEEDED_{src_name}_{skew:.1f}s")

    return {
        "claim_class": claim_class.value if isinstance(claim_class, ClaimClass) else str(claim_class),
        "quarantined": quarantined,
        "validity": validity,
        "generation_epoch": round(gen_epoch, 4),
        "observation_epoch": round(obs_epoch, 4) if obs_epoch is not None else None,
        "input_epochs": input_epochs or {},
        "permitted_skew_s": permitted_skew_s,
        "receiver_topology": receiver_topology,
        "firmware_revision": firmware_revision,
        "software_version": software_version,
        "continuity_id": continuity_id or "arc_001",
        "slip_counter": slip_counter,
        "sample_sequence": sample_sequence if sample_sequence is not None else 1,
        "calibration_id": calibration_id or "cal_split_star_20260830",
        "uncertainty": uncertainty or {"value": 0.0, "units": "dimensionless", "confidence": "unspecified"},
        "failure_reasons": failure_reasons,
        "source_artifact_hashes": source_artifact_hashes or {}
    }
