#!/bin/bash
# QUARANTINED: this legacy entry point does not select a radio by immutable
# serial, broadly kills every hackrf_transfer/recorder process, and enables
# antenna power at high gain.  On the current station that can seize either
# the tracker-owned Pro or the ClearStream-owned One and can energize the
# wrong RF path.  Retain the old body below only as historical context.
echo "QUARANTINED: unsafe unaddressed GPS capture; no process or radio was touched." >&2
exit 78

# Open-sky GPS L1 capture for a real meter-level position fix.
#
# BEFORE running: move the Taoglas AA.250 to a spot with a CLEAR VIEW OF THE SKY
# (window ledge facing up, balcony rail, or roof). A partial west-window view
# gives ~1 satellite; an open sky gives the 4+ the PVT solver needs. Put a small
# steel plate under the magnetic mount as a ground plane.
#
# usage: capture_gps_opensky.sh [outfile] [seconds]
set -e
OUT="${1:-gps_opensky.iq}"
SECS="${2:-120}"
FS=8000000            # 8 Msps
FC=1573000000         # centre; GPS L1 (1575.42) lands at +2.42 MHz IF, clear of DC

echo ">> releasing the radio from the recorder"
pkill -f recorder.py 2>/dev/null || true
pkill -f hackrf_transfer 2>/dev/null || true
sleep 3
hackrf_info 2>/dev/null | grep -q "Found HackRF" || { echo "!! no HackRF found"; exit 1; }

echo ">> capturing ${SECS}s of GPS L1  (bias-tee ON to power the antenna, RF amp OFF)"
hackrf_transfer -r "$OUT" -f "$FC" -s "$FS" -n "$((SECS*FS))" -l 32 -g 44 -a 0 -p 1

echo ">> ADC level check (want sigma > 4, clipping ~0%)"
python3 - "$OUT" <<'PY'
import numpy as np,sys
r=np.fromfile(sys.argv[1],dtype=np.int8,count=2*8000000).astype(np.float32)
i,q=r[0::2],r[1::2]
print("   sigma %.1f counts, clipping %.3f%%, max|I| %d"%(i.std(),100*np.mean((abs(i)>=127)|(abs(q)>=127)),int(max(abs(i).max(),abs(q).max()))))
PY

echo ">> restarting the recorder"
( cd "$(dirname "$0")/../../validation" && nohup python3 recorder.py >> ../observations/recorder.log 2>&1 & disown ) || true
echo ">> done -> $OUT   (next: acquire, then track, then PVT)"
