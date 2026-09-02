#!/usr/bin/env bash
# supervisor_v2.sh — observe-only station health monitor.
#
# ############################################################################
# ##  MANUAL START ONLY.  This supervisor must be started BY A HUMAN, on    ##
# ##  purpose, after reading this header. AUTOMATIC HARDWARE RECOVERY IS    ##
# ##  DISABLED: production tracker/band clients now share pro_lease.py,     ##
# ##  but legacy/manual libhackrf tools do not and recovery remains human.  ##
# ##  It must NEVER run alongside                                           ##
# ##  scripts/supervisor.sh (v1): two supervisors fight over the radio.     ##
# ##  This script refuses to start if v1 is running, but that check is a    ##
# ##  seatbelt, not a license.                                              ##
# ############################################################################
#
# Fixes over v1 (each was an adversarially confirmed defect):
#   a. Monitors the complete producer set v1 attempted to manage (v1 detected
#      3 but relaunched 5, so gpsdo_probe and clock_bias_shadow could die
#      silently forever).
#      Detection = pgrep liveness AND state-file mtime age where a state file
#      exists (observations/state.<name>.json, per AGENTS.md merge table).
#   b. Performs no TERM, reset, relaunch, image change, taskpolicy mutation, or
#      other hardware/process recovery action. Enumerating process names and
#      serial substrings cannot prove exclusive radio ownership, and a
#      maintenance-lock check cannot make a later reset atomic.
#   c. Maintenance-window arbitration: if observations/maintenance.lock
#      exists, report it and suppress state-file freshness alarms.
#   d. Own observability: atomic instance directory and a heartbeat every cycle.
#
# Usage:
#   scripts/supervisor_v2.sh
#
# SUPERVISOR_MODE=recover is intentionally rejected. A future recovery
# controller must make every libhackrf client acquire the same atomic Pro lock;
# process-name scans are not an ownership protocol.  pro_lease.py makes the
# production handoff atomic; it does not authorize an automatic reset.

set -uo pipefail   # deliberately NOT -e: a supervisor must not die on a probe

WORKDIR="/Volumes/Radiator 8TB/gnss/hackrf_gnss"
OBS="/Volumes/Radiator 8TB/gnss/observations"
LOCKFILE="$OBS/maintenance.lock"
FAILED_MARKER="$OBS/supervisor.FAILED"
INSTANCE_LOCK="$OBS/supervisor_v2.lock.d"
PIDFILE="$INSTANCE_LOCK/pid"
LOGFILE="/tmp/supervisor_v2.log"
CYCLE_S=30
LOCK_GRACE_S=1800          # 4: post-maintenance grace, pgrep-only health checks
SUPERVISOR_MODE="${SUPERVISOR_MODE:-observe}"

log() {
    local line
    line="$(date -u +%Y-%m-%dT%H:%M:%SZ) supervisor_v2[$$] $*"
    echo "$line"
    echo "$line" >> "$LOGFILE"
}

case "$SUPERVISOR_MODE" in
    observe) ;;
    recover)
        log "FATAL: automatic recovery is disabled until all Pro clients share one atomic ownership lock"
        exit 78
        ;;
    *)
        log "FATAL: SUPERVISOR_MODE must be 'observe' (got '$SUPERVISOR_MODE')"
        exit 2
        ;;
esac

# --- refuse to coexist with v1 (or a second v2) -----------------------------
# Broadened v1 guard: any supervisor.sh cmdline (not only scripts/-prefixed),
# explicitly excluding _v2 (a second v2 is caught by the pidfile below).
if pgrep -fl 'supervisor\.sh' 2>/dev/null | grep -v '_v2' | grep -q 'supervisor\.sh'; then
    log "FATAL: supervisor v1 (supervisor.sh) is running. Refusing to start."
    exit 1
fi
if ! mkdir "$INSTANCE_LOCK" 2>/dev/null; then
    oldpid=$(cat "$PIDFILE" 2>/dev/null || echo "")
    if [[ "$oldpid" =~ ^[0-9]+$ ]] && ps -p "$oldpid" -o pid= 2>/dev/null | grep -q '[0-9]'; then
        log "FATAL: another supervisor_v2 (pid $oldpid) owns $INSTANCE_LOCK. Refusing to start."
        exit 1
    fi
    # Never auto-remove a stale lock: two simultaneous takeover attempts can
    # otherwise remove each other's newly-created directory in the gap before
    # the owner PID is written.  Recovery control fails closed; a human must
    # inspect the owner and remove this exact lock directory during a
    # maintenance window.
    log "FATAL: stale or malformed instance lock at $INSTANCE_LOCK (owner ${oldpid:-missing}); refusing automatic takeover"
    exit 1
fi
if ! (set -C; printf '%s\n' "$$" > "$PIDFILE") 2>/dev/null; then
    rmdir "$INSTANCE_LOCK" 2>/dev/null
    log "FATAL: acquired $INSTANCE_LOCK but could not record ownership"
    exit 1
fi
release_instance_lock() {
    local owner
    owner=$(cat "$PIDFILE" 2>/dev/null || echo "")
    if [ "$owner" = "$$" ]; then
        rm -f "$PIDFILE"
        rmdir "$INSTANCE_LOCK" 2>/dev/null
    fi
}
trap release_instance_lock EXIT

cd "$WORKDIR" || { log "FATAL: cannot cd to $WORKDIR"; exit 1; }
log "started mode=observe (MANUAL start assumed). cycle=${CYCLE_S}s pidfile=$PIDFILE"

# --- health model -----------------------------------------------------------
# producer table: pgrep pattern | state file (or "-") | max state age seconds
# Patterns are written broken-up so no invoking cmdline can self-match.
# State ages are generous vs producer cadence (tracker 1 Hz, gpsdo 5 s,
# band heartbeats each rotation, clock_bias ~30 s) to avoid flapping.
PRODUCERS=(
    "tracker_producer.py|state.tracker.json|120"
    "band_producer.py|state.band.json|900"
    "examples/clock_bias|state.clock_bias.json|900"
    "clock_bias_shadow.py|-|0"
    "gpsdo_probe.py|state.gpsdo.json|300"
)

file_age_s() {
    # mtime age in seconds, or -1 if missing
    local f="$1" m now
    m=$(stat -f %m "$f" 2>/dev/null) || { echo -1; return; }
    now=$(date +%s)
    echo $(( now - m ))
}

health_check() {
    # echoes failing check names; empty output == healthy
    # $1 == 1: post-maintenance grace (4) — pgrep liveness only; skip state-file
    # checks so just-restored producers' stale state files can't trigger a
    # spurious recovery.
    local grace="${1:-0}" entry pat sf max age
    for entry in "${PRODUCERS[@]}"; do
        pat="${entry%%|*}"
        sf="$(echo "$entry" | cut -d'|' -f2)"
        max="$(echo "$entry" | cut -d'|' -f3)"
        if ! pgrep -f "$pat" > /dev/null 2>&1; then
            echo "dead:$pat"
            continue
        fi
        if [ "$grace" -eq 1 ]; then
            continue
        fi
        if [ "$sf" != "-" ]; then
            age=$(file_age_s "$OBS/$sf")
            if [ "$age" -lt 0 ]; then
                echo "nostate:$sf"
            elif [ "$age" -gt "$max" ]; then
                echo "stale:$sf:${age}s"
            fi
        fi
    done
}

# --- main loop --------------------------------------------------------------
cycle=0
lock_last_seen=0      # 4: epoch when maintenance.lock was last observed present
while true; do
    cycle=$(( cycle + 1 ))

    # A historical/manual failure marker is surfaced prominently. This
    # monitor never clears it and never acts on the station.
    if [ -f "$FAILED_MARKER" ]; then
        log "heartbeat cycle=$cycle status=FAILED marker=$FAILED_MARKER — standing down (remove the marker after manual recovery)"
        sleep "$CYCLE_S"
        continue
    fi

    # Operator maintenance window wins, always.
    if [ -f "$LOCKFILE" ]; then
        lock_last_seen=$(date +%s)   # 4: remember for the post-window grace
        log "heartbeat cycle=$cycle status=MAINTENANCE $LOCKFILE present — not acting"
        sleep "$CYCLE_S"
        continue
    fi

    # 4: post-window grace — for LOCK_GRACE_S after the lock disappears the
    # just-restored producers' state files are legitimately stale; check pgrep
    # liveness only so expected stale files do not create noisy alarms.
    grace=0
    if [ "$lock_last_seen" -gt 0 ] && [ $(( $(date +%s) - lock_last_seen )) -lt "$LOCK_GRACE_S" ]; then
        grace=1
    fi

    failures=$(health_check "$grace")
    if [ -n "$failures" ]; then
        log "heartbeat cycle=$cycle status=DEGRADED failures=$(echo "$failures" | tr '\n' ' ') — observe-only; no process or hardware action"
    else
        if [ "$grace" -eq 1 ]; then
            log "heartbeat cycle=$cycle status=OK all producers live (post-window grace: state-file staleness suppressed)"
        else
            log "heartbeat cycle=$cycle status=OK all producers live, state files fresh"
        fi
    fi

    sleep "$CYCLE_S"
done
