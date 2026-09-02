#!/bin/bash
# QUARANTINED: defaults to dead Pro #1 and switches FPGA images without the
# reviewed maintenance lease and verified restoration transaction.
echo "QUARANTINED: legacy ext-precision capture cannot prove safe image restore; no radio was opened." >&2
exit 78

# capture_extprec_l1.sh — extended-precision (12-bit) GPS/Galileo L1 capture
# on the HackRF Pro with the Taoglas AA.250.
#
# The 2_extprec_rx gateware decimates 4x-32x (the halfband tail was
# removed), so the usable range is 32 MHz AFE / 32 .. 32 MHz / 4 =
# 1-8 Msps complex (12-bit I/Q, int16 lanes). Default here is 2.5 Msps
# (16x on 40 MHz), centred so L1 (1575.42) sits at +1.0 MHz IF with the DC
# spike 1 MHz away from the carrier; GPS L1 C/A needs >=2.046 MHz. For the
# full-rate mode: FS=8000000 (4x on 32 MHz) — 8 Msps int16 = 32 MB/s, the
# byte rate the live tracker already sustains daily at 16 Msps 8-bit with
# ~55 ms processing against the ~190 ms USB queue (no host buffer changes
# needed; the queue time-depth argument is identical at equal byte rate).
# Narrowband slice: no BeiDou B1 here
# (use capture_dualband_gnss.sh for the wide multi-constellation setup).
#
# Why bother: 12-bit vs 8-bit gives ~20 dB more dynamic range below full
# scale — weaker satellites survive next to interference.
#
# usage: capture_extprec_l1.sh [outfile_base] [seconds]
set -e
BASE="${1:-extprec_l1}"
SECS="${2:-60}"

PRO_SERIAL="${PRO_SERIAL:-QUARANTINED_NO_SERIAL}"
FS="${FS:-2500000}"      # up to 8000000 (4x decimation); see header
FC=1574420000          # L1 at +1.0 MHz IF
L="${L:-40}"           # IF gain — ext chain runs quiet, needs more than std
G="${G:-44}"           # BB gain
TOOLS="$(cd "$(dirname "$0")" && pwd)"
HACKRF_TOOLS="${HACKRF_TOOLS:-/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src}"

RAW="${BASE}.rawiq"
echo ">> switching Pro to ext_precision_rx bitstream (index 2)"
DYLD_LIBRARY_PATH="$HACKRF_TOOLS/../../libhackrf/src" "$HACKRF_TOOLS/hackrf_debug" -d "$PRO_SERIAL" -P 2

echo ">> capturing ${SECS}s L1/E1 @ 2.5 Msps 12-bit (l=$L g=$G, bias-tee from persistent default)"
"$HACKRF_TOOLS/hackrf_transfer" -d "$PRO_SERIAL" -r "$RAW" -f "$FC" -s "$FS" \
	-n "$((SECS*FS))" -l "$L" -g "$G" -a 0 2>&1 | tail -2

echo ">> restoring standard bitstream (index 0)"
DYLD_LIBRARY_PATH="$HACKRF_TOOLS/../../libhackrf/src" "$HACKRF_TOOLS/hackrf_debug" -d "$PRO_SERIAL" -P 0

echo ">> stats:"
python3 "$TOOLS/extprec_convert.py" "$RAW" --stats-only

echo ">> converting for hackrf_gnss (cs8 auto-scaled): ${BASE}.iq"
python3 "$TOOLS/extprec_convert.py" "$RAW" "${BASE}.iq" --format cs8 --scale auto
echo ">> done. Acquire with:"
echo "   cargo run --release --example acquire_file -- ${BASE}.iq"
