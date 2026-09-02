#!/bin/bash
# QUARANTINED: obsolete dual-active-antenna procedure. It broadly kills
# transfers, defaults to the dead Pro #1, and enables persistent bias on both
# receivers. Retained as historical source only.
echo "QUARANTINED: unsafe legacy dual-band capture; no radio was opened." >&2
exit 78

# Dual-device, dual-band parallel GNSS capture for the Taoglas AA.250.
#
# Slices captured simultaneously:
#   Pro  : L1/E1/B1 cluster  — BeiDou B1 (1561.098), GPS L1 + Galileo E1 (1575.42)
#   One  : L5/E5a/B2a        — all at 1176.45 MHz
#
# Hardware setup:
#   * AA.250 on a ground plane (steel plate / car roof) with clear sky view.
#   * AA.250 -> 2-way SMA power divider -> Pro ANT and One ANT.
#     (Single-device fallback: set DEVICES=pro or DEVICES=one below.)
#   * Bias-tee is REQUIRED — the AA.250's dual-stage LNA draws power from it.
#     This script sets persistent defaults (RX=on, OFF=off) on both devices,
#     so captures are powered even without -p on the command line.
#
# Sample alignment: without a trigger wire the two streams start a few ms
# apart; align in post (dsp_calib.rs). With a trigger wire (One P28 pin15->16,
# or Pro P1/P2 routed trigger) add -H to both hackrf_transfer calls and fire
# the trigger after both are armed for sample-aligned starts (sync-start API).
#
# usage: capture_dualband_gnss.sh [outdir] [seconds]
set -e
OUTDIR="${1:-.}"
SECS="${2:-120}"

PRO_SERIAL="${PRO_SERIAL:-QUARANTINED_NO_SERIAL}"
ONE_SERIAL="${ONE_SERIAL:-0000000000000000922c63dc21748847}"
DEVICES="${DEVICES:-both}"        # both | pro | one

# --- Pro: L1/E1/B1 cluster -------------------------------------------------
PRO_FS="${PRO_FS:-20000000}"      # 20 Msps
PRO_FC="${PRO_FC:-1568300000}"    # clean zone (0.75*fs) ~1560.8-1575.8:
                                  #   B1 at -7.2 MHz, L1/E1 at +7.1 MHz (both in)
PRO_L="${PRO_L:-32}"              # IF gain  (validated on AA.250)
PRO_G="${PRO_G:-44}"              # BB gain

# --- One: L5/E5a/B2a -------------------------------------------------------
ONE_FS="${ONE_FS:-10000000}"      # 10 Msps
ONE_FC="${ONE_FC:-1174450000}"    # L5/E5a/B2a at +2.0 MHz IF, clear of DC spike
ONE_L="${ONE_L:-32}"
ONE_G="${ONE_G:-44}"

TS="$(date +%Y%m%d_%H%M%S)"
PRO_OUT="$OUTDIR/l1e1b1_${TS}.iq"
ONE_OUT="$OUTDIR/l5e5ab2a_${TS}.iq"

echo ">> releasing radios"
pkill -f hackrf_transfer 2>/dev/null || true
sleep 2

for S in "$PRO_SERIAL" "$ONE_SERIAL"; do
	[ "$DEVICES" = "both" ] || [ "$S" = "$PRO_SERIAL" -a "$DEVICES" = "pro" ] || \
	[ "$S" = "$ONE_SERIAL" -a "$DEVICES" = "one" ] || continue
	hackrf_info -d "$S" 2>/dev/null | grep -q "Serial number" || { echo "!! device $S not found"; exit 1; }
	echo ">> $S: persistent bias-tee defaults RX=on, OFF=off (powers AA.250 LNA)"
	hackrf_biast -d "$S" -r on -t off -o off
done

PIDS=""
if [ "$DEVICES" = "both" ] || [ "$DEVICES" = "pro" ]; then
	echo ">> Pro  : ${SECS}s @ $PRO_FC Hz, $((PRO_FS/1000000)) Msps  -> $PRO_OUT"
	hackrf_transfer -d "$PRO_SERIAL" -r "$PRO_OUT" -f "$PRO_FC" -s "$PRO_FS" \
		-n "$((SECS*PRO_FS))" -l "$PRO_L" -g "$PRO_G" -a 0 &
	PIDS="$PIDS $!"
fi
if [ "$DEVICES" = "both" ] || [ "$DEVICES" = "one" ]; then
	echo ">> One  : ${SECS}s @ $ONE_FC Hz, $((ONE_FS/1000000)) Msps  -> $ONE_OUT"
	hackrf_transfer -d "$ONE_SERIAL" -r "$ONE_OUT" -f "$ONE_FC" -s "$ONE_FS" \
		-n "$((SECS*ONE_FS))" -l "$ONE_L" -g "$ONE_G" -a 0 &
	PIDS="$PIDS $!"
fi

wait $PIDS

echo ">> ADC level checks (want sigma > 4, clipping ~0%)"
for F in "$PRO_OUT" "$ONE_OUT"; do
	[ -f "$F" ] || continue
	python3 - "$F" <<'PY'
import numpy as np,sys,os
f=sys.argv[1]
r=np.fromfile(f,dtype=np.int8,count=20000000).astype(np.float32)
i,q=r[0::2],r[1::2]
print("   %-28s sigma %.1f, clipping %.3f%%, max %d, %.1f MB"%(
    os.path.basename(f), i.std(),
    100*np.mean((np.abs(i)>=127)|(np.abs(q)>=127)),
    int(max(np.abs(i).max(),np.abs(q).max())), os.path.getsize(f)/1e6))
PY
done

echo ">> M0 shortfall check (0 = no USB throughput loss)"
hackrf_debug -S -d "$PRO_SERIAL" 2>/dev/null | grep -i shortfall || true

echo ">> done. Feed to decoders, e.g.:"
echo "   cargo run --example acquire_file -- $PRO_OUT"
