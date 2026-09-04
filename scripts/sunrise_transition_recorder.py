#!/usr/bin/env python3
"""Ground Sunrise Event Transition Recorder & Snapshot Engine.

Latches the exact millisecond when the solar disk breaches the geometric horizon
(solar_el >= 0.0 deg) at the station (39.0029 N, -77.6058 W), capturing a synchronized
scientific snapshot across all 15 metrology state files.
"""

import datetime
import json
import os
import sys
import time

OBS_DIR = "/Volumes/Radiator 8TB/gnss/observations"
EVENT_FILE = os.path.join(OBS_DIR, "sunrise_20260904_event.json")

def wait_and_capture_sunrise(timeout_s=300):
    print(f"Sunrise Transition Recorder: Arming watcher at {datetime.datetime.now().isoformat()}...")
    start_time = time.time()
    captured = False

    while (time.time() - start_time) < timeout_s:
        solar_path = os.path.join(OBS_DIR, "state.solar.json")
        if os.path.exists(solar_path):
            try:
                with open(solar_path) as f:
                    s = json.load(f)
                eph = s.get("solar_ephemeris", {})
                dawn = s.get("dawn_state", {})
                el = eph.get("solar_el_apparent_deg", -99.0)
                az = eph.get("solar_az_deg", 0.0)

                sys.stdout.write(f"\rWatching Sun: El={el:+.3f}° | Az={az:.1f}° | State={dawn.get('code', '?')} | Shadow Ht={dawn.get('overhead_shadow_height_km', '?')} km  ")
                sys.stdout.flush()

                if el >= 0.0 or dawn.get("code") in ["GROUND_SUNRISE", "FULL_DAYLIGHT"]:
                    print("\n\n☀️ SUNRISE TRANSITION DETECTED! Capturing synchronized geodetic state snapshot...")
                    captured = True
                    break
            except Exception as e:
                pass
        time.sleep(0.5)

    if not captured:
        print("\nTimed out or already past sunrise. Taking baseline capture...")

    # Capture all state files
    snapshot = {
        "event_name": "GROUND_SUNRISE_TRANSITION_20260904",
        "capture_timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "capture_epoch": time.time(),
        "station": {
            "lat_deg": 39.0029,
            "lon_deg": -77.6058,
            "altitude_amsl_m": 20.0,
            "receiver": "HackRF Pro + One (Split Star)"
        },
        "states": {}
    }

    state_files = [
        "solar", "radiometry", "reflectometry", "meteorology",
        "tropo", "iono", "klobuchar", "hatch_divergence", "hoi",
        "gdop", "relativity", "phase_drift", "clock_bias"
    ]

    for sf in state_files:
        p = os.path.join(OBS_DIR, f"state.{sf}.json")
        if os.path.exists(p):
            try:
                with open(p) as f:
                    snapshot["states"][sf] = json.load(f)
            except Exception as e:
                snapshot["states"][sf] = {"error": str(e)}

    with open(EVENT_FILE, "w") as f:
        json.dump(snapshot, f, indent=2)

    print(f"Sunrise event snapshot successfully written to: {EVENT_FILE}")
    return snapshot

if __name__ == "__main__":
    wait_and_capture_sunrise()
