#!/bin/bash
# run_l1_session.sh — one-command open-sky L1/E1 session: capture -> convert
# -> acquire -> verdict.  Validated end-to-end 2026-08-20 (6/6 sim control,
# std/ext A/B equivalence, bitstream switching reliable without reboot).
#
# BEFORE running: AA.250 on a ground plane with CLEAR SKY VIEW (roof, balcony,
# window ledge facing up). Indoor/desk placement yields 0 satellites — verified.
#
# usage: run_l1_session.sh [ext|std] [seconds] [outdir]
set -e
MODE="${1:-ext}"            # ext = 12-bit extended precision @2.5Msps | std = 8-bit @8Msps
SECS="${2:-60}"
OUTDIR="${3:-.}"
GNSS="$(cd "$(dirname "$0")/.." && pwd)"
HACKRF_TOOLS="${HACKRF_TOOLS:-/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src}"
S="${PRO_SERIAL:-0000000000000000977c64de2b557213}"
TS="$(date +%Y%m%d_%H%M%S)"

# hackrf_pro's exit code is 0 even on failure — a failed read is detected by
# the MISSING ts.* line on stdout. Prints the tick value or nothing.
ts_read() { # ts_read <start|trigger|now>
	"$HACKRF_TOOLS/hackrf_pro" -d "$S" --ts-read "$1" 2>/dev/null | grep -oE 'ts\.[a-z]+ = [0-9]+' | grep -oE '[0-9]+' || true
}

write_sidecar() { # write_sidecar <path> <start> <tick_hz> <fs> <image> <source>
	python3 - "$@" <<'PY'
import json, sys
path, start, tick_hz, fs, image, source = sys.argv[1:7]
json.dump({
    "stream_start_ticks": int(start),
    "tick_hz": float(tick_hz),
    "fs": float(fs),
    "image": int(image),
    "ticks_source": source,
    "utc_known": False,
}, open(path, "w"), indent=2)
print(f">> sidecar: {path}  (start={start} tick_hz={float(tick_hz):.2f} source={source})")
PY
}

if [ "$MODE" = "ext" ]; then
	FS=2500000; FC=1574420000; IF=1000000; L=40; G=44
	RAW="$OUTDIR/l1_ext_${TS}.rawiq"
	echo ">> [ext] switching to ext_precision_rx (runtime switch is safe: reload fix flashed)"
	DYLD_LIBRARY_PATH="$HACKRF_TOOLS/../../libhackrf/src" "$HACKRF_TOOLS/hackrf_debug" -d "$S" -P 2
	echo ">> [ext] capturing ${SECS}s @ $FC, $FS sps (l=$L g=$G)"
	"$HACKRF_TOOLS/hackrf_transfer" -d "$S" -r "$RAW" -f "$FC" -s "$FS" -n $((SECS*FS*2)) -l $L -g $G -a 0 -p 1 | tail -1
	DYLD_LIBRARY_PATH="$HACKRF_TOOLS/../../libhackrf/src" "$HACKRF_TOOLS/hackrf_debug" -d "$S" -P 0
	echo ">> [ext] stats + baseband conversion (mix -$IF Hz)"
	python3 "$GNSS/scripts/extprec_convert.py" "$RAW" --stats-only
	python3 "$GNSS/scripts/extprec_convert.py" "$RAW" "$OUTDIR/l1_bb.f32" --format cf32 --mix-if $IF --fs $FS | tail -1
	# On image 2 SPI timestamp reads return 0 — the counter travels in-stream
	# in the nibble channel, so the sidecar comes from the stream itself.
	echo ">> [ext] timestamp sidecar (nibble-stream)"
	"$GNSS/target/release/examples/discipline" "$RAW" "$FS" --image 2 \
		--sidecar "$OUTDIR/l1_ts_${TS}.json" \
		|| echo ">> sidecar skipped (build with: cargo build --release --example discipline)"
else
	FS=8000000; FC=1573000000; IF=2420000; L=32; G=44
	RAW="$OUTDIR/l1_std_${TS}.iq"
	# bracket the capture with counter reads for the tick-rate calibration
	TSB="$(ts_read now)"
	if [ -z "$TSB" ]; then echo ">> WARNING: ts-read now failed (no ts.* line) — sidecar will be skipped"; fi
	echo ">> [std] capturing ${SECS}s @ $FC, $FS sps (l=$L g=$G)"
	"$HACKRF_TOOLS/hackrf_transfer" -d "$S" -r "$RAW" -f "$FC" -s "$FS" -n $((SECS*FS)) -l $L -g $G -a 0 -p 1 | tail -1
	TSE="$(ts_read now)"; TSTART="$(ts_read start)"
	if [ -n "$TSB" ] && [ -n "$TSE" ] && [ -n "$TSTART" ]; then
		TICK_HZ="$(python3 -c "print(($TSE-$TSB)*$FS/($SECS*$FS))")"
		write_sidecar "$OUTDIR/l1_ts_${TS}.json" "$TSTART" "$TICK_HZ" "$FS" 0 spi
	else
		echo ">> WARNING: timestamp read failed — no sidecar written"
	fi
	echo ">> [std] baseband conversion (mix -$IF Hz)"
	python3 - "$RAW" "$OUTDIR/l1_bb.f32" <<'PY'
import numpy as np, sys
raw=np.fromfile(sys.argv[1],dtype=np.int8)
fs=8e6
sig=(raw[0::2].astype(np.float32)+1j*raw[1::2].astype(np.float32))
t=np.arange(len(sig))/fs
bb=sig*np.exp(-2j*np.pi*2.42e6*t).astype(np.complex64)
out=np.empty(2*len(bb),dtype=np.float32); out[0::2]=bb.real; out[1::2]=bb.imag
out.tofile(sys.argv[2]); print("sigma I: %.1f"%raw[0::2].std())
PY
fi

echo ">> acquiring (GPS + SBAS, +-20 kHz Doppler)"
"$GNSS/target/release/examples/acquire_file" "$OUTDIR/l1_bb.f32" "$FS" -20000 20000 250 8000 \
	> "$OUTDIR/l1_acq_${TS}.json"
python3 - "$OUTDIR/l1_acq_${TS}.json" <<'PY'
import json,sys
res=json.load(open(sys.argv[1])); res.sort(key=lambda r:-r['metric'])
hits=[r for r in res if r['metric']>2.5]
print(f">> {len(hits)} satellites acquired:")
for r in hits: print(f"   PRN {r['prn']:3} metric {r['metric']:.1f} dopp {r['doppler']:+7.0f}")
if not hits:
    print("   (none — check sky view, ground plane, bias-tee)")
PY
echo ">> verdict JSON: $OUTDIR/l1_acq_${TS}.json  raw: $RAW"
