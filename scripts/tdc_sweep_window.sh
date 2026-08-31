#!/usr/bin/env bash
# tdc_sweep_window.sh — TDC code-density (DNL) calibration sweep, TCXO mode.
#
# Executes the coherence-trap-breaking calibration from
# docs/superpowers/plans/2026-08-30-tdc-code-density-sweep-window.md
# (step 6 of docs/superpowers/plans/2026-08-30-shared-rf-calibration-plan.md).
#
# ############################################################################
# ##  PHYSICS CONSTRAINT — READ BEFORE TOUCHING ANYTHING BELOW              ##
# ##                                                                        ##
# ##  Code-density calibration requires INCOHERENT sampling. TCXO mode      ##
# ##  gives it free: the -0.667 ppm offset slews the Bodnar PPS phase at    ##
# ##  ~667 ns/s, wrapping the 25 ns adclk period every ~37.5 ms, so 1 Hz    ##
# ##  samples land quasi-uniformly across the 48-tap window.                ##
# ##                                                                        ##
# ##  BUT: activate_best_clock_source (firmware usb_api_transceiver.c:411)  ##
# ##  runs on ANY streaming transfer — including OFF-mode transfers — and   ##
# ##  latches CLKIN whenever the Bodnar 10 MHz is present on the           ##
# ##  connector (it always is at this station). One transfer and the       ##
# ##  sweep silently goes Bodnar-COHERENT (run3 measured the null:         ##
# ##  +0.026 ppb) and every sample lands on the same phase: the histogram  ##
# ##  collapses and the dataset is garbage WITH NO ERROR MESSAGE.          ##
# ##                                                                        ##
# ##  Therefore, while this script runs:                                    ##
# ##    - NO hackrf_transfer, NO live_radio, NO band_producer snapshots.    ##
# ##    - This script itself invokes ONLY register-level control reads      ##
# ##      (hackrf_pro --read-reg / --tdc-selftest) and hackrf_spiflash -R.  ##
# ##      Register reads are proven safe: run1 (tdc_pps_run1.jsonl) was     ##
# ##      captured this way and stayed at -0.667 ppm (TCXO) throughout.     ##
# ##    - NO hackrf_debug anywhere (it can silently revert the FPGA slot).  ##
# ############################################################################
#
# What it does:
#   1. Refuses to run if supervisor v1 is alive (verify: pgrep -f supervisor.sh).
#   2. Creates observations/maintenance.lock (supervisor_v2 stands down on it).
#      trap EXIT/INT ALWAYS restores: reset board, relaunch tracker, SIGCONT
#      band_producer, remove lock.
#   3. SIGSTOPs band_producer FIRST (it must not snapshot mid-window).
#   4. pkill -TERM the tracker (pattern-broken, per
#      docs/superpowers/specs/2026-08-25-clock-write-continuity-experiment.md:127-133),
#      sleep 3, VERIFIES live_radio exited (TERMs an orphan; never -9; aborts
#      if the radio stays held).
#   5. hackrf_spiflash -R, exit code CHECKED — abort on failure. sleep 6.
#      After reset with no stream started, the Si5351 references the internal
#      TCXO (run1 lineage): this IS the calibration source.
#   6. Verifies the slot-0 timing image TDC registers respond (0x31 status,
#      0x30 selftest must read 0x00 — RO self-test OFF during capture).
#   7. Captures at 1 Hz: each second read 0x31 then the frozen 6-byte
#      thermometer map 0x20-0x25 (idiom of tdc_pps_poll.py / run1), appending
#      run1-schema lines to OUTFILE, with a full config header line first.
#      Periodic checkpoint lines every 300 s. Bounded duration.
#   8. Restores the station and prints the analysis command
#      (scripts/tdc_density_cal.py).
#
# Usage: tdc_sweep_window.sh [DURATION_S] [OUTFILE]
#   DURATION_S default 14400 (4 h => ~60 in-window hits/bin), min 600,
#   max 28800 (8 h). >=100/bin wants ~6.6 h; minimum publishable 50/bin ~3.3 h.
#   START_TEMP_C=xx.x (env) records the bench temperature — measurements
#   without recorded configuration are this station's recurring sin (P5:
#   TCXO tempco 0.014-0.1 ppm/degC and the run1 bench temp went unlogged).

set -uo pipefail

PRO_SERIAL="0000000000000000645061de252d6613"
TOOLS="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src"
HP="$TOOLS/hackrf_pro"
SPIFLASH="$TOOLS/hackrf_spiflash"
WORKDIR="/Volumes/Radiator 8TB/gnss/hackrf_gnss"
OBS="/Volumes/Radiator 8TB/gnss/observations"
LOCKFILE="$OBS/maintenance.lock"

DUR="${1:-14400}"
OUT="${2:-$OBS/tdc_sweep_run1.jsonl}"
START_TEMP_C="${START_TEMP_C:-MANUAL-ENTRY-REQUIRED}"

MIN_DUR=600
MAX_DUR=28800
MAX_CONSEC_READ_ERRS=30
CHECKPOINT_EVERY_S=300

log() { echo "$(date -u +%Y-%m-%dT%H:%M:%SZ) tdc_sweep[$$] $*"; }

case "$DUR" in
    ''|*[!0-9]*) log "FATAL: DURATION_S must be an integer (got '$DUR')"; exit 2 ;;
esac
if [ "$DUR" -lt "$MIN_DUR" ] || [ "$DUR" -gt "$MAX_DUR" ]; then
    log "FATAL: DURATION_S $DUR outside bounds [$MIN_DUR, $MAX_DUR]"
    exit 2
fi

cd "$WORKDIR" || { log "FATAL: cannot cd $WORKDIR"; exit 2; }

# --- 1. arbitration ---------------------------------------------------------
# Broadened v1 guard: any supervisor.sh cmdline (not only scripts/-prefixed),
# explicitly excluding _v2 (v2 honors maintenance.lock and may keep running).
if pgrep -fl 'supervisor\.sh' 2>/dev/null | grep -v '_v2' | grep -q 'supervisor\.sh'; then
    log "FATAL: supervisor v1 is running — it ignores maintenance.lock and will fight this window within 30 s. Stop it first (pkill -TERM -f 'scripts/supervisor.sh')."
    exit 1
fi
if [ -f "$LOCKFILE" ]; then
    log "FATAL: $LOCKFILE already exists — another maintenance window is open. Not stacking windows."
    exit 1
fi

# --- 2. always-restore trap FIRST, then lock --------------------------------
# Traps are installed BEFORE the lock exists (cleanup tolerates a
# nothing-to-restore run); this closes the crash window between lock
# creation and trap installation.
RADIO_HELD=0
CLEANED=0

cleanup() {
    [ "$CLEANED" -eq 1 ] && return
    CLEANED=1
    log "RESTORE: beginning station restore (trap)"

    if [ "$RADIO_HELD" -eq 1 ]; then
        log "RESTORE: an orphan live_radio still held the radio — skipping board reset and tracker relaunch. HUMAN NEEDED."
    else
        # Restart law (AGENTS.md ~:100-114): board reset must precede the
        # tracker start. Checked exit, 3 attempts, exponential backoff.
        local attempt backoff rc ok
        ok=0; backoff=10
        for attempt in 1 2 3; do
            "$SPIFLASH" -d "$PRO_SERIAL" -R
            rc=$?
            if [ "$rc" -eq 0 ]; then ok=1; break; fi
            log "RESTORE: board reset FAILED rc=$rc (attempt $attempt/3)"
            [ "$attempt" -lt 3 ] && { sleep "$backoff"; backoff=$((backoff*2)); }
        done
        if [ "$ok" -eq 1 ]; then
            sleep 6
            log "RESTORE: board reset OK — relaunching tracker (documented order: tracker first)"
            nohup python3 scripts/tracker_producer.py >> /tmp/tracker_producer.log 2>&1 &
            sleep 3
        else
            log "RESTORE: board reset failed 3x — NOT relaunching the tracker onto an un-reset board. HUMAN NEEDED (likely physical replug, AGENTS.md 2026-08-29 incident). Lock will still be removed; run supervisor_v2 manually once the board re-enumerates."
        fi
    fi

    log "RESTORE: SIGCONT band_producer"
    pkill -CONT -f 'band''_producer.py' 2>/dev/null

    rm -f "$LOCKFILE"
    log "RESTORE: maintenance.lock removed — window closed"
}
trap cleanup EXIT
trap 'log "interrupted"; exit 130' INT TERM

echo "{\"holder\": \"tdc_sweep_window.sh\", \"pid\": $$, \"since\": \"$(date -u +%Y-%m-%dT%H:%M:%SZ)\", \"purpose\": \"TDC code-density TCXO sweep — NO streaming transfers allowed\"}" > "$LOCKFILE"
log "maintenance.lock created"

# --- 3. freeze band_producer FIRST ------------------------------------------
log "SIGSTOP band_producer (must not snapshot / start a transfer mid-window)"
pkill -STOP -f 'band''_producer.py' 2>/dev/null

# --- 4. stop the tracker, verify the radio is free --------------------------
log "pkill -TERM tracker (pattern-broken)"
pkill -TERM -f 'tracker''_producer.py' 2>/dev/null
sleep 3

if pgrep -f 'live''_radio' > /dev/null 2>&1 || pgrep -f 'hackrf''_transfer' > /dev/null 2>&1; then
    log "live_radio/hackrf_transfer still alive — TERM the orphan (never -9)"
    pkill -TERM -f 'live''_radio' 2>/dev/null
    pkill -TERM -f 'hackrf''_transfer' 2>/dev/null
    sleep 3
fi
if pgrep -f 'live''_radio' > /dev/null 2>&1 || pgrep -f 'hackrf''_transfer' > /dev/null 2>&1; then
    RADIO_HELD=1
    log "FATAL: orphan live_radio/hackrf_transfer refuses TERM — radio not free, aborting without reset"
    exit 1
fi
log "radio verified free of orphans"

# --- 5. board reset (checked) -----------------------------------------------
log "hackrf_spiflash -R (checked exit)"
"$SPIFLASH" -d "$PRO_SERIAL" -R
rc=$?
if [ "$rc" -ne 0 ]; then
    log "FATAL: board reset failed rc=$rc — aborting sweep (trap will retry once during restore)"
    exit 1
fi
sleep 6
log "board reset OK — Si5351 now references the internal TCXO (no stream has run)"

# --- 6. verify slot-0 timing image TDC registers ----------------------------
read_reg() {
    # read_reg ADDR -> echoes byte value 0-255, or empty on failure (3 tries)
    local addr="$1" try out hex
    for try in 1 2 3; do
        out=$("$HP" -d "$PRO_SERIAL" --read-reg "$addr" 2>/dev/null)
        hex=$(echo "$out" | grep -oE '0x[0-9a-fA-F]{2}' | head -1)
        if [ -n "$hex" ]; then
            echo $(( hex ))
            return 0
        fi
        sleep 0.2
    done
    echo ""
    return 1
}

status=$(read_reg 0x31)
if [ -z "$status" ]; then
    log "FATAL: TDC status reg 0x31 unreadable — slot-0 timing image not responding. DO NOT use hackrf_debug -P to 'fix' the slot; investigate manually."
    exit 1
fi
selftest=$(read_reg 0x30)
if [ -z "$selftest" ]; then
    log "FATAL: reg 0x30 unreadable — aborting"
    exit 1
fi
if [ "$selftest" -ne 0 ]; then
    log "reg 0x30 = $selftest (RO self-test ON) — gating it off before capture"
    "$HP" -d "$PRO_SERIAL" --tdc-selftest off
    selftest=$(read_reg 0x30)
    if [ -z "$selftest" ] || [ "$selftest" -ne 0 ]; then
        log "FATAL: could not gate RO self-test off (0x30=${selftest:-unreadable}) — an active RO corrupts the external-edge capture. Aborting."
        exit 1
    fi
fi
log "slot-0 TDC verified: 0x31=$status 0x30=0x00 (external-trigger mode, RO off)"

# --- 7. config header --------------------------------------------------------
# Measurements without recorded configuration are this station's recurring sin.
if [ -f "$OUT" ]; then
    log "NOTE: $OUT exists — appending (resume); a fresh config header marks the new segment"
fi
{
    printf '{"kind": "config", "t": %s, ' "$(date +%s)"
    printf '"run": "tdc_sweep", "serial": "%s", ' "$PRO_SERIAL"
    printf '"slot": 0, "image": "timing.py CarryChainTDC 48-tap", '
    printf '"clock_source": "TCXO (XTAL) free-running, nominal adclk 40 MHz / 25 ns period; expected offset -0.667 +/- 0.004 ppm vs Bodnar PPS (P5)", '
    printf '"clock_ns": 25.0, "adclk_hz": 40000000, '
    printf '"pps_source": "Bodnar LBE-1421 OUT1 via P28.16 (external trigger, 0x30=0x00)", '
    printf '"firmware_provenance": "post hackrf_spiflash -R cold boot, NO streaming transfer since reset (usb_api_transceiver.c:411 CLKIN-latch trap avoided); register-poll capture per tdc_pps_run1 lineage", '
    printf '"start_temp_c": "%s", ' "$START_TEMP_C"
    printf '"duration_s": %s, "poll_hz": 1, ' "$DUR"
    printf '"analyzer": "scripts/tdc_density_cal.py"}\n'
} >> "$OUT"
log "config header written to $OUT (start_temp_c=$START_TEMP_C)"

# --- 8. 1 Hz capture loop ----------------------------------------------------
log "capture: ${DUR}s at 1 Hz -> $OUT"
T0=$(date +%s)
i=0
n_lines=0
consec_errs=0
last_ckpt=$T0
prev_valid=""   # 0x31 bit0 (valid toggle) from the previous iteration

while :; do
    i=$(( i + 1 ))
    target=$(( T0 + i ))
    now=$(date +%s)
    [ $(( now - T0 )) -ge "$DUR" ] && break
    if [ "$now" -lt "$target" ]; then
        sleep $(( target - now ))
    fi

    # Coherence sentry (once per iteration): ANY streaming process during the
    # window means activate_best_clock_source may have latched CLKIN (see plan
    # doc, THE COHERENCE TRAP) — the data is void from this instant. Abort via
    # the normal cleanup path (trap restores the station).
    if pgrep -f 'live''_radio' > /dev/null 2>&1 || pgrep -f 'hackrf''_transfer' > /dev/null 2>&1; then
        log "FATAL: live_radio/hackrf_transfer observed mid-sweep — CLKIN may have latched (coherence broken, see plan doc). Aborting sweep; data void from this instant."
        printf '{"kind": "abort", "t": %s, "n": %s, "aborted_coherence_risk": true, "reason": "streaming process (live_radio/hackrf_transfer) observed during window"}\n' \
            "$(date +%s)" "$n_lines" >> "$OUT"
        exit 1
    fi

    s31=$(read_reg 0x31)
    if [ -z "$s31" ]; then
        consec_errs=$(( consec_errs + 1 ))
        if [ "$consec_errs" -ge "$MAX_CONSEC_READ_ERRS" ]; then
            log "FATAL: $consec_errs consecutive register-read failures — board gone? aborting (trap restores)"
            exit 1
        fi
        continue
    fi

    bytes=""
    read_ok=1
    for addr in 0x20 0x21 0x22 0x23 0x24 0x25; do
        b=$(read_reg "$addr")
        if [ -z "$b" ]; then read_ok=0; break; fi
        hexb=$(printf '0x%02x' "$b")
        if [ -z "$bytes" ]; then bytes="$hexb"; else bytes="$bytes,$hexb"; fi
    done
    if [ "$read_ok" -ne 1 ]; then
        consec_errs=$(( consec_errs + 1 ))
        continue
    fi
    consec_errs=0

    # Valid-toggle tracking: 0x31 bit0 flips on each fresh PPS capture. If it
    # did not toggle since the previous iteration, this is a re-read of the
    # same capture — tag the line "dup":true instead of counting it as a
    # fresh sample.
    valid=$(( s31 & 1 ))
    dup=""
    if [ -n "$prev_valid" ] && [ "$valid" -eq "$prev_valid" ]; then
        dup=', "dup": true'
    fi
    prev_valid=$valid

    printf '{"t": %s, "reg31": "0x%02x", "bytes": "%s"%s}\n' \
        "$(date +%s.%N)" "$s31" "$bytes" "$dup" >> "$OUT"
    if [ -z "$dup" ]; then
        n_lines=$(( n_lines + 1 ))
    fi

    now=$(date +%s)
    if [ $(( now - last_ckpt )) -ge "$CHECKPOINT_EVERY_S" ]; then
        last_ckpt=$now
        printf '{"kind": "checkpoint", "t": %s, "n": %s, "elapsed_s": %s}\n' \
            "$now" "$n_lines" "$(( now - T0 ))" >> "$OUT"
        log "checkpoint: $n_lines samples, $(( now - T0 ))s elapsed of ${DUR}s"
    fi
done

printf '{"kind": "stop", "t": %s, "n": %s, "end_temp_c": "MANUAL-ENTRY-REQUIRED"}\n' \
    "$(date +%s)" "$n_lines" >> "$OUT"
log "capture complete: $n_lines samples in $OUT"
log "analyze with: python3 scripts/tdc_density_cal.py '$OUT' --clock-ns 25.0 --taps 48 --out-json '$OBS/tdc_sweep_run1_dnl.json'"
log "expected saturation ~79.7 +/- 2 percent; a collapsed histogram (few distinct codes) means the board went CLKIN-coherent — check nothing streamed"

# trap cleanup restores the station on exit
exit 0
