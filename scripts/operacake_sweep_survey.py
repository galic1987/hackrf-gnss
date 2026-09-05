#!/usr/bin/env python3
"""Opera Cake 8-Port RF Spectrum Survey & Characterization Engine.

Sequentially routes HackRF Pro Primary Port A to all 8 secondary ports:
  A1, A2, A3, A4, B1, B2, B3, B4
Performs non-emitting calibrated wideband sweeps across 50 MHz - 2000 MHz.
Computes band-specific power profiles to classify antenna types and sensitivity.
"""

import os
import subprocess
import sys
import time
import numpy as np

PRO_SERIAL = "0000000000000000645061de252d6613"
PORTS = ["A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4"]

BANDS = {
    "FM (88-108 MHz)": (88e6, 108e6),
    "VHF Airband (118-137 MHz)": (118e6, 137e6),
    "UHF TV / ATSC (470-608 MHz)": (470e6, 608e6),
    "ATSC Ch35 Pilot (599-605 MHz)": (599e6, 605e6),
    "Cellular 700/850 MHz": (700e6, 894e6),
    "GNSS L5/E5a (1176.45 MHz)": (1164e6, 1188e6),
    "GNSS L2/B2 (1207-1228 MHz)": (1200e6, 1235e6),
    "GNSS L1/B1 (1561-1575 MHz)": (1555e6, 1610e6),
    "Iridium (1616-1626 MHz)": (1616e6, 1626e6),
    "Cellular PCS/AWS (1850-1990 MHz)": (1850e6, 1990e6),
}


def switch_operacake(port):
    """Switch Opera Cake Port A0 to target port."""
    cmd = ["hackrf_operacake", "-d", PRO_SERIAL, "-a", port]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"Failed to switch Opera Cake to {port}: {res.stderr}")
    time.sleep(0.1)  # Settling time


def run_sweep(port, csv_path):
    """Run wideband non-emitting sweep on current port."""
    # -a 0 (no amp), -p 0 (no DC power), safe LNA=32, VGA=30, 250kHz bins, 3 sweeps
    cmd = [
        "hackrf_sweep",
        "-d", PRO_SERIAL,
        "-a", "0",
        "-p", "0",
        "-l", "32",
        "-g", "30",
        "-f", "50:2000",
        "-w", "250000",
        "-N", "3",
        "-r", csv_path
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"Sweep failed on port {port}: {res.stderr}")


def parse_sweep_csv(csv_path):
    """Parse hackrf_sweep CSV and extract (freq, dbm) array."""
    freqs = []
    powers = []
    
    with open(csv_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 7:
                continue
            try:
                hz_low = float(parts[2])
                hz_high = float(parts[3])
                hz_bin_width = float(parts[4])
                num_samples = int(parts[5])
                db_vals = [float(v) for v in parts[6:]]
                
                n_bins = len(db_vals)
                for i, p in enumerate(db_vals):
                    f_bin = hz_low + (i + 0.5) * hz_bin_width
                    freqs.append(f_bin)
                    powers.append(p)
            except Exception:
                continue

    freqs = np.array(freqs)
    powers = np.array(powers)
    
    # Sort by frequency
    idx = np.argsort(freqs)
    return freqs[idx], powers[idx]


def analyze_port(port, freqs, powers):
    """Calculate band metrics for a port."""
    metrics = {
        "port": port,
        "mean_power_db": float(np.mean(powers)),
        "median_power_db": float(np.median(powers)),
        "max_power_db": float(np.max(powers)),
        "max_freq_mhz": float(freqs[np.argmax(powers)] / 1e6),
        "bands": {}
    }
    
    for band_name, (f_min, f_max) in BANDS.items():
        mask = (freqs >= f_min) & (freqs <= f_max)
        if np.any(mask):
            band_powers = powers[mask]
            metrics["bands"][band_name] = {
                "mean_db": float(np.mean(band_powers)),
                "max_db": float(np.max(band_powers)),
                "max_freq_mhz": float(freqs[mask][np.argmax(band_powers)] / 1e6)
            }
        else:
            metrics["bands"][band_name] = {"mean_db": -999.0, "max_db": -999.0, "max_freq_mhz": 0.0}

    return metrics


def main():
    print("========================================================================")
    print("      OPERA CAKE 8-PORT RF ANTENNA SURVEY & CHANNEL SWEEP (RX-ONLY)    ")
    print("========================================================================")
    print(f"Device: HackRF Pro ({PRO_SERIAL})")
    print("Reference: Bodnar LBE-1421 GPSDO 10 MHz & 1PPS")
    print("RF Mode: STRICT RX ONLY (No TX, Amp=OFF, Antenna DC Power=OFF)")
    print(f"Ports: {', '.join(PORTS)}")
    print("Sweep Range: 50 MHz to 2000 MHz (250 kHz resolution)\n")

    results = {}

    for port in PORTS:
        csv_path = f"/tmp/operacake_sweep_{port}.csv"
        print(f"[{time.strftime('%H:%M:%S')}] Switching to Port {port}...", end="", flush=True)
        try:
            switch_operacake(port)
            print(" [LATCHED] -> Running 50-2000 MHz sweep...", end="", flush=True)
            run_sweep(port, csv_path)
            freqs, powers = parse_sweep_csv(csv_path)
            metrics = analyze_port(port, freqs, powers)
            results[port] = metrics
            print(f" [DONE] Mean={metrics['mean_power_db']:.1f} dB, Peak={metrics['max_power_db']:.1f} dB @ {metrics['max_freq_mhz']:.2f} MHz")
        except Exception as e:
            print(f" [ERROR: {e}]")

    print("\n========================================================================")
    print("                     CROSS-PORT ANTENNA COMPARISON                      ")
    print("========================================================================")
    header = f"{'Band':<32} | " + " | ".join([f"{p:>6}" for p in PORTS])
    print(header)
    print("-" * len(header))

    for band_name in BANDS.keys():
        row = f"{band_name:<32} | "
        vals = []
        for port in PORTS:
            if port in results and band_name in results[port]["bands"]:
                m = results[port]["bands"][band_name]["max_db"]
                vals.append(f"{m:>6.1f}")
            else:
                vals.append("   -- ")
        row += " | ".join(vals)
        print(row)

    print("-" * len(header))
    
    # Overall summary
    row_mean = f"{'Total Mean Power (50-2000 MHz)':<32} | "
    row_mean += " | ".join([f"{results[p]['mean_power_db']:>6.1f}" if p in results else "   -- " for p in PORTS])
    print(row_mean)
    
    row_peak = f"{'Peak Carrier Power':<32} | "
    row_peak += " | ".join([f"{results[p]['max_power_db']:>6.1f}" if p in results else "   -- " for p in PORTS])
    print(row_peak)

    row_peak_f = f"{'Peak Carrier Freq (MHz)':<32} | "
    row_peak_f += " | ".join([f"{results[p]['max_freq_mhz']:>6.0f}" if p in results else "   -- " for p in PORTS])
    print(row_peak_f)
    print("========================================================================\n")

    # Save summary JSON
    summary_path = "/Volumes/Radiator 8TB/gnss/observations/operacake_sweep_summary.json"
    import json
    with open(summary_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Full spectral survey saved to: {summary_path}")


if __name__ == "__main__":
    main()
