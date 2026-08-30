#!/usr/bin/env bash
set -euo pipefail

echo "=== Latch Capture Calibration Script ==="
PRO_SERIAL="0000000000000000645061de252d6613"
HACKRF_INFO="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_info"
HACKRF_CLOCK="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_clock"
HACKRF_TRANSFER="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_transfer"
HACKRF_PRO="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_pro"
HACKRF_DEBUG="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_debug"

echo "Activating Slot 1 (coarse trigger latch image)..."
"$HACKRF_DEBUG" -d "$PRO_SERIAL" -P 1 || true
sleep 5

echo "Forcing RX streaming at 16 Msps (1 second) to lock Si5351 to CLKIN..."
"$HACKRF_TRANSFER" -d "$PRO_SERIAL" -r /dev/null -s 16000000 -n 16000000 || true

echo "Verifying CLKIN signal..."
"$HACKRF_CLOCK" -d "$PRO_SERIAL" -i || true

echo "Fetching device info..."
INFO=$("$HACKRF_INFO" -d "$PRO_SERIAL" || true)
FW_VERSION=$(echo "$INFO" | grep "Firmware Version" | awk '{print $3}' || echo "unknown")

OUT_FILE="/Volumes/Radiator 8TB/gnss/observations/ts_latch_run3.jsonl"
echo "Recording metadata to $OUT_FILE..."
echo '{"metadata": "TDC latch capture", "sample_rate": 16000000, "firmware": "'"$FW_VERSION"'"}' > "$OUT_FILE"

echo "Running TDC measurement for 20 minutes (1200 seconds)..."
echo "Starting latch capture loop..."

prev=""
end=$((SECONDS + 1200))
while [ $SECONDS -lt $end ]; do
  line=$("$HACKRF_PRO" -d "$PRO_SERIAL" --ts-read trigger 2>/dev/null | grep -o '[0-9]*' | head -1)
  [ -z "$line" ] && continue
  if [ -n "$prev" ] && [ "$line" != "$prev" ]; then
    echo "{\"t\": $(date +%s.%N), \"latch\": $line, \"delta\": $((line - prev))}" >> "$OUT_FILE"
  fi
  prev="$line"
done
echo "Capture complete."
