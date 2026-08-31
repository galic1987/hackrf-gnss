#!/usr/bin/env python3
"""Offline shared-RF correlation math prototype.

This module does not open a radio, set gains, verify CLKIN, or validate capture
provenance. `check_usb_separation` is a best-effort macOS inventory heuristic.
The three-point peak interpolation is descriptive and does not establish
picosecond uncertainty; a claim-grade path still needs manifest-bound captures,
GCC-PHAT/known-signal validation, bandwidth/SNR gates, and Monte Carlo or
repeatability uncertainty. ABBA separation is algebra only.
"""

import math
import subprocess
import sys
import numpy as np
from typing import Dict, Tuple, Optional, Any

PRO_SERIAL = "0000000000000000645061de252d6613"
ONE_SERIAL = "0000000000000000922c63dc21748847"


def check_usb_separation(serial1: str, serial2: str) -> Dict[str, Any]:
    """Check if serial1 and serial2 reside on separate USB host controllers."""
    try:
        out = subprocess.check_output(["system_profiler", "SPUSBDataType"], text=True, timeout=5)
        # Parse USB trees
        s1_bus = None
        s2_bus = None
        current_bus = None

        for line in out.splitlines():
            line_str = line.strip()
            if "Host Controller Location:" in line_str or "USB" in line_str and "Bus:" in line_str:
                current_bus = line_str
            if serial1 in line_str:
                s1_bus = current_bus
            if serial2 in line_str:
                s2_bus = current_bus

        separated = (s1_bus != s2_bus) if (s1_bus and s2_bus) else None
        return {
            "serial1": serial1,
            "serial2": serial2,
            "s1_bus": s1_bus,
            "s2_bus": s2_bus,
            "separated": separated,
        }
    except Exception as e:
        return {"error": str(e), "separated": None}


def compute_cross_correlation(
    s1: np.ndarray,
    s2: np.ndarray,
    sample_rate_hz: float = 16.0e6,
) -> Dict[str, Any]:
    """Compute a descriptive sub-sample correlation peak and carrier phase.

    s1, s2: complex64/complex128 baseband I/Q arrays.

    Lag convention: a delayed ``s2`` produces a negative lag.  The reported
    carrier phase is the phase of ``sum(s1 * conj(s2_aligned))`` at the
    integer peak (so ``s2 = s1 * exp(+j*phi)`` reports ``-phi``).  It is not
    phase-compensated to the parabolic fractional-lag estimate.
    """
    try:
        sample_rate_hz = float(sample_rate_hz)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("sample_rate_hz must be finite and > 0") from None
    if not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0:
        raise ValueError("sample_rate_hz must be finite and > 0")

    s1 = np.asarray(s1)
    s2 = np.asarray(s2)
    if s1.ndim != 1 or s2.ndim != 1:
        raise ValueError("cross-correlation inputs must be one-dimensional")
    n = min(len(s1), len(s2))
    if n < 128:
        raise ValueError(f"Sample length {n} is too short for cross-correlation")
    if not np.all(np.isfinite(s1[:n])) or not np.all(np.isfinite(s2[:n])):
        raise ValueError("cross-correlation inputs must be finite")

    x = s1[:n] - np.mean(s1[:n])
    y = s2[:n] - np.mean(s2[:n])
    ex = float(np.sum(np.abs(x) ** 2))
    ey = float(np.sum(np.abs(y) ** 2))
    sx = max(float(np.max(np.abs(s1[:n]))), 1.0)
    sy = max(float(np.max(np.abs(s2[:n]))), 1.0)
    eps = np.finfo(float).eps * n
    if ex <= eps * sx * sx or ey <= eps * sy * sy:
        raise ValueError("cross-correlation inputs must have positive AC energy")

    # FFT cross-correlation
    n_fft = 1 << (2 * n - 1).bit_length()
    X = np.fft.fft(x, n_fft)
    Y = np.fft.fft(y, n_fft)
    R_xy = np.fft.ifft(X * np.conj(Y))

    # Shift zero lag to center
    R_xy = np.fft.fftshift(R_xy)
    lags = np.arange(-n_fft // 2, n_fft // 2)

    # Only -(n-1)..(n-1) are linear-correlation lags. The rest of the padded
    # FFT contains numerical zero and must not participate in peak selection.
    valid_indices = np.flatnonzero((lags >= -(n - 1)) & (lags <= n - 1))
    valid_mag = np.abs(R_xy[valid_indices])
    peak_local = int(np.argmax(valid_mag))
    peak_idx = int(valid_indices[peak_local])
    coarse_lag_samples = lags[peak_idx]

    # Sub-sample peak refinement using parabolic interpolation on magnitude
    mag = np.abs(R_xy)
    if 0 < peak_idx < len(mag) - 1:
        alpha = float(mag[peak_idx - 1])
        beta = float(mag[peak_idx])
        gamma = float(mag[peak_idx + 1])
        denom = 2.0 * (2.0 * beta - alpha - gamma)
        delta = (gamma - alpha) / denom if abs(denom) > 1e-12 else 0.0
    else:
        delta = 0.0

    fine_lag_samples = coarse_lag_samples + delta
    fine_lag_sec = fine_lag_samples / sample_rate_hz
    fine_lag_ps = fine_lag_sec * 1e12

    # Carrier phase difference at the COARSE peak; see the convention in the
    # docstring.  The fractional-delay parabola refines only the lag estimate.
    carrier_phase_rad = float(np.angle(R_xy[peak_idx]))
    carrier_phase_deg = math.degrees(carrier_phase_rad)

    # Normalized peak correlation over the samples that actually overlap at
    # the selected lag. This is not spectral coherence.
    lag = int(coarse_lag_samples)
    if lag < 0:
        x_overlap, y_overlap = x[:n + lag], y[-lag:]
    else:
        x_overlap, y_overlap = x[lag:], y[:n - lag]
    overlap_norm = math.sqrt(float(np.sum(np.abs(x_overlap) ** 2))
                             * float(np.sum(np.abs(y_overlap) ** 2)))
    if overlap_norm <= 0.0:
        raise ValueError("selected peak has zero-energy overlap")
    peak_correlation = min(1.0, float(np.abs(R_xy[peak_idx]) / overlap_norm))

    # A simple ambiguity diagnostic: strongest valid sidelobe outside the
    # peak and its immediate interpolation neighbours.
    sidelobes = valid_mag.copy()
    sidelobes[max(0, peak_local - 1):peak_local + 2] = 0.0
    second_peak = float(np.max(sidelobes)) if sidelobes.size else 0.0
    peak_to_sidelobe = (float(valid_mag[peak_local]) / second_peak
                        if second_peak > 0.0 else math.inf)

    return {
        "coarse_lag_samples": float(coarse_lag_samples),
        "fine_lag_samples": float(fine_lag_samples),
        "lag_sec": float(fine_lag_sec),
        "lag_ps": float(fine_lag_ps),
        "lag_uncertainty_ps": None,
        "lag_claim_grade": False,
        "lag_method": "FFT correlation + three-point magnitude parabola",
        "carrier_phase_rad": float(carrier_phase_rad),
        "carrier_phase_deg": float(carrier_phase_deg),
        "carrier_phase_method": "coarse integer correlation peak",
        "carrier_phase_convention": "arg(sum(s1 * conj(s2_aligned)))",
        "peak_correlation": float(peak_correlation),
        # Compatibility alias; callers should migrate to peak_correlation.
        "coherence": float(peak_correlation),
        "peak_to_sidelobe_ratio": float(peak_to_sidelobe),
    }


def abba_delay_separation(delay_ab_ps: float, delay_ba_ps: float) -> Tuple[float, float]:
    """Decouple receiver front-end delay from cable/splitter asymmetry using ABBA swap.

    delay_ab = tau_rx_diff + tau_cable_diff
    delay_ba = tau_rx_diff - tau_cable_diff

    Returns: (tau_rx_diff_ps, tau_cable_diff_ps)
    """
    tau_rx_diff_ps = (delay_ab_ps + delay_ba_ps) / 2.0
    tau_cable_diff_ps = (delay_ab_ps - delay_ba_ps) / 2.0
    return tau_rx_diff_ps, tau_cable_diff_ps


if __name__ == "__main__":
    print("=== Dual-Radio Offline Math Prototype ===")
    usb_info = check_usb_separation(PRO_SERIAL, ONE_SERIAL)
    print("USB inventory heuristic:", usb_info)
    print("No radios were configured or captured; no delay calibration was performed.")
