#!/usr/bin/env bash
# supervisor.sh — Keep the station alive.
# Checks if core services (tracker_producer.py, clock_bias, band_producer.py) are running.
# If they are not, it initiates a clean restart.

set -euo pipefail

PRO_SERIAL="0000000000000000645061de252d6613"
HD="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_debug"
SPIFLASH="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_spiflash"
WORKDIR="/Volumes/Radiator 8TB/gnss/hackrf_gnss"

cd "$WORKDIR"

echo "Station Supervisor started. Monitoring processes..."

while true; do
    # Check if core processes are running
    if ! pgrep -f "tracker_producer.py" > /dev/null || ! pgrep -f "band_producer.py" > /dev/null || ! pgrep -f "examples/clock_bias" > /dev/null; then
        # Prevent concurrent supervisors or rapid crash loops with flock on a lock file
        exec 9>/tmp/supervisor_reboot.lock
        if flock -n 9; then
            echo "$(date) — Outage detected! Core services down. Initiating recovery..."
            
            # Kill any remaining zombies (including live_radio)
            echo "$(date) — Terminating lingering processes..."
            pkill -f "tracker_producer.py" || true
            pkill -f "band_producer.py" || true
            pkill -f "examples/clock_bias" || true
            pkill -f "clock_bias_shadow.py" || true
            pkill -f "gpsdo_probe.py" || true
            pkill -f "live_radio" || true
            sleep 2
            
            echo "$(date) — Resetting HackRF hardware state..."
            # Removed the hackrf_debug -P 0 line to avoid enforcing bitstream zero unnecessarily
            "$SPIFLASH" -d "$PRO_SERIAL" -R || true
            sleep 6
            
            echo "$(date) — Booting core services..."
            nohup python3 scripts/tracker_producer.py >> /tmp/tracker_producer.log 2>&1 &
            sleep 3
            nohup ./target/release/examples/clock_bias >> /tmp/clock_bias.log 2>&1 &
            nohup python3 scripts/band_producer.py >> /tmp/band_producer.log 2>&1 &
            nohup python3 scripts/gpsdo_probe.py >> /tmp/gpsdo_probe.log 2>&1 &
            nohup python3 scripts/clock_bias_shadow.py >> /tmp/clock_bias_shadow.log 2>&1 &
            
            echo "$(date) — Station reboot complete. Entering 60s backoff..."
            sleep 60
            
            flock -u 9
        else
            echo "$(date) — Reboot already in progress or lock held."
        fi
        exec 9>&-
    fi
    
    # Throttle daemon for acquisition workers
    for p in $(pgrep -f "(_acq|acq_)" || true); do 
        taskpolicy -b -p $p 2>/dev/null || true
    done
    
    sleep 30
done
