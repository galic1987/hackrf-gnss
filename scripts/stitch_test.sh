#!/bin/bash
# QUARANTINED: defaults to dead Pro #1 and uses the superseded timestamp/CLI
# contract while changing antenna power. Retained as historical source only.
echo "QUARANTINED: legacy stitch HIL; no radio was opened." >&2
exit 78

# stitch_test.sh — T8 step 6 live stitch test: two 8 Msps std captures taken
# ~GAP s apart on the same tuning, each bracketed by FPGA timestamp reads, a
# timestamp sidecar per capture, then the phase-continuity check in
# examples/stitch_check (tone phase advance vs counter elapsed ticks).
#
# Counter is authoritative on image 0/1 (std); tick = 4/sample at 8 Msps.
# hackrf_pro exits 0 even when a read fails, so a MISSING 'ts.' line on stdout
# is the failure signal — never trust the exit code.
#
# usage: stitch_test.sh <fc_hz> [secs] [gap_s] [outdir] [f_min f_max]
set -e
FC="$1"; SECS="${2:-8}"; GAP="${3:-60}"; OUTDIR="${4:-.}"
FMIN="${5:-300000}"; FMAX="${6:-3000000}"
GNSS="$(cd "$(dirname "$0")/.." && pwd)"
HACKRF_TOOLS="${HACKRF_TOOLS:-/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src}"
S="${PRO_SERIAL:-QUARANTINED_NO_SERIAL}"
FS=8000000
TS="$(date +%Y%m%d_%H%M%S)"

ts_read() { # ts_read <start|trigger|now> -> tick value, or nothing on failure
	"$HACKRF_TOOLS/hackrf_pro" -d "$S" --ts-read "$1" 2>/dev/null \
		| grep -oE 'ts\.[a-z]+ = [0-9]+' | grep -oE '[0-9]+' || true
}

capture() { # capture <tag> -> prints "<raw> <sidecar>" on stdout
	local tag="$1"
	local raw="$OUTDIR/stitch_${tag}_${TS}.iq"
	local sc="$OUTDIR/l1_ts_stitch_${tag}_${TS}.json"
	local tb te tstart tick_hz
	tb="$(ts_read now)"
	if [ -z "$tb" ]; then echo "!! FAIL: ts-read now before $tag produced no ts. line" >&2; return 1; fi
	echo ">> [$tag] capturing ${SECS}s @ $FC, $FS sps" >&2
	"$HACKRF_TOOLS/hackrf_transfer" -d "$S" -r "$raw" -f "$FC" -s "$FS" \
		-n $((SECS * FS)) -l 40 -g 46 -a 0 -p 1 2>&1 | tail -1 >&2
	te="$(ts_read now)"
	tstart="$(ts_read start)"
	if [ -z "$te" ] || [ -z "$tstart" ]; then
		echo "!! FAIL: ts-read after $tag produced no ts. line" >&2; return 1
	fi
	tick_hz="$(python3 -c "print(($te - $tb) * $FS / ($SECS * $FS))")"
	echo ">> [$tag] bracket: ts-begin=$tb ts-end=$te (feed these to examples/discipline)" >&2
	python3 - "$sc" "$tstart" "$tick_hz" "$FS" <<'PY'
import json, sys
path, start, tick_hz, fs = sys.argv[1:5]
json.dump({
    "stream_start_ticks": int(start),
    "tick_hz": float(tick_hz),
    "fs": float(fs),
    "image": 0,
    "ticks_source": "spi",
    "utc_known": False,
}, open(path, "w"), indent=2)
PY
	echo ">> [$tag] start=$tstart ticks, tick_hz=$tick_hz -> $sc" >&2
	echo "$raw $sc"
}

T0=$(date +%s)
read RAW1 SC1 <<<"$(capture cap1)"
ELAPSED=$(( $(date +%s) - T0 ))
REM=$(( GAP - ELAPSED ))
if [ "$REM" -gt 0 ]; then echo ">> waiting ${REM}s so starts are ~${GAP}s apart"; sleep "$REM"; fi
read RAW2 SC2 <<<"$(capture cap2)"

echo ">> stitch check ($RAW1 + $RAW2, tone window $FMIN..$FMAX Hz)"
"$GNSS/target/release/examples/stitch_check" "$RAW1" "$SC1" "$RAW2" "$SC2" "$FMIN" "$FMAX"
