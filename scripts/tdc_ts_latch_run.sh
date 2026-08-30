#!/usr/bin/env bash
# tdc_ts_latch_run.sh — ticks-per-GPS-second capture.
# Requires slot 1 active (coarse trigger latch image): hackrf_debug -P 1
# Polls the 48-bit trigger latch via hackrf_pro --ts-read trigger; every
# PPS rising edge updates it. Each CHANGE of the latch value is one GPS
# second; consecutive differences = sample clocks per GPS second.
# Usage: tdc_ts_latch_run.sh SECONDS OUTFILE
set -u
PRO=0000000000000000645061de252d6613
HP="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_pro"
DUR="${1:-900}"
OUT="${2:-/Volumes/Radiator 8TB/gnss/observations/ts_latch_run1.jsonl}"
echo "# ts_latch run $(date -u +%Y-%m-%dT%H:%M:%SZ) pro=$PRO slot1 latch deltas" > "$OUT"
prev=""
end=$((SECONDS + DUR))
while [ $SECONDS -lt $end ]; do
  line=$("$HP" -d "$PRO" --ts-read trigger 2>/dev/null | grep -o '[0-9]*' | head -1)
  [ -z "$line" ] && continue
  if [ -n "$prev" ] && [ "$line" != "$prev" ]; then
    echo "{\"t\": $(date +%s.%N), \"latch\": $line, \"delta\": $((line - prev))}" >> "$OUT"
  fi
  prev="$line"
done
echo "latch run complete: $(grep -c '{' "$OUT") seconds"
