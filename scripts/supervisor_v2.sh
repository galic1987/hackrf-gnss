#!/usr/bin/env bash
# supervisor_v2.sh — Station keep-alive, second generation.
#
# ############################################################################
# ##  MANUAL START ONLY.  This supervisor must be started BY A HUMAN, on    ##
# ##  purpose, after reading this header.  It must NEVER run alongside      ##
# ##  scripts/supervisor.sh (v1): two supervisors fight over the radio.     ##
# ##  This script refuses to start if v1 is running, but that check is a    ##
# ##  seatbelt, not a license.                                              ##
# ############################################################################
#
# Fixes over v1 (each was an adversarially confirmed defect):
#   a. Detects EVERY producer it relaunches (v1 detected 3 but relaunched 5 —
#      gpsdo_probe and clock_bias_shadow could die silently forever).
#      Detection = pgrep liveness AND state-file mtime age where a state file
#      exists (observations/state.<name>.json, per AGENTS.md merge table).
#   b. Kill list includes live_radio; the radio must be verifiably free of
#      orphans BEFORE any board reset. Pattern-broken pkill only (see
#      docs/superpowers/specs/2026-08-25-clock-write-continuity-experiment.md:127-133
#      — pkill self-matched its wrapper once and live_radio kept streaming).
#      NEVER -9 (AGENTS.md: never pkill -9 live_radio). sleep 3, not 2.
#   c. NO hackrf_debug anywhere (v1 once carried `hackrf_debug -P 0`, which
#      silently reverts the FPGA slot). The ONLY reset is `hackrf_spiflash -R`
#      with its exit code CHECKED: on failure we do NOT relaunch the tracker;
#      we retry with exponential backoff (max 3 attempts), then touch
#      observations/supervisor.FAILED and stop acting. No `|| true` swallowing.
#   d. Maintenance-window arbitration: if observations/maintenance.lock
#      exists, log and sleep — never act (v1 fought the operator within 30 s).
#   e. Own observability: PID file, heartbeat log line every cycle.
#
# Restart law (AGENTS.md ~:100-131): pkill -TERM → sleep 3 → board reset
# IMMEDIATELY (before anything else) → sleep 6 → start tracker. Never run
# cargo builds while the tracker streams.

set -uo pipefail   # deliberately NOT -e: a supervisor must not die on a probe

PRO_SERIAL="0000000000000000645061de252d6613"
TOOLS="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src"
SPIFLASH="$TOOLS/hackrf_spiflash"
WORKDIR="/Volumes/Radiator 8TB/gnss/hackrf_gnss"
OBS="/Volumes/Radiator 8TB/gnss/observations"
LOCKFILE="$OBS/maintenance.lock"
FAILED_MARKER="$OBS/supervisor.FAILED"
PIDFILE="$OBS/supervisor_v2.pid"
LOGFILE="/tmp/supervisor_v2.log"
CYCLE_S=30
POST_RECOVERY_BACKOFF_S=60
MAX_CONSEC_RECOVERIES=5    # 3: recoveries without a healthy cycle before give-up
LOCK_GRACE_S=1800          # 4: post-maintenance grace, pgrep-only health checks

log() {
    local line
    line="$(date -u +%Y-%m-%dT%H:%M:%SZ) supervisor_v2[$$] $*"
    echo "$line"
    echo "$line" >> "$LOGFILE"
}

# --- refuse to coexist with v1 (or a second v2) -----------------------------
# Broadened v1 guard: any supervisor.sh cmdline (not only scripts/-prefixed),
# explicitly excluding _v2 (a second v2 is caught by the pidfile below).
if pgrep -fl 'supervisor\.sh' 2>/dev/null | grep -v '_v2' | grep -q 'supervisor\.sh'; then
    log "FATAL: supervisor v1 (supervisor.sh) is running. Refusing to start."
    exit 1
fi
if [ -f "$PIDFILE" ]; then
    oldpid=$(cat "$PIDFILE" 2>/dev/null || echo "")
    if [ -n "$oldpid" ] && kill -0 "$oldpid" 2>/dev/null; then
        log "FATAL: another supervisor_v2 (pid $oldpid) is alive per $PIDFILE. Refusing to start."
        exit 1
    fi
    log "stale pidfile (pid ${oldpid:-none} dead) — taking over"
fi
echo $$ > "$PIDFILE"
trap 'rm -f "$PIDFILE"' EXIT

cd "$WORKDIR" || { log "FATAL: cannot cd to $WORKDIR"; exit 1; }
log "started (MANUAL start assumed). serial=$PRO_SERIAL cycle=${CYCLE_S}s pidfile=$PIDFILE"

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

# --- recovery ---------------------------------------------------------------
# PIDs holding (or able to claim) the PRO's radio. live_radio is Pro-only.
# hackrf_transfer blocks only when it targets $PRO_SERIAL or names no -d
# device — the One legitimately runs its own hackrf_transfer all day
# (phase_producer pilot capture); unscoped matching would loop on its
# respawning child until the consecutive-recovery give-up bricked us.
pro_radio_holders() {
    pgrep -f 'live''_radio' 2>/dev/null
    pgrep -fl 'hackrf''_transfer' 2>/dev/null | while read -r pid args; do
        case "$args" in
            *"$PRO_SERIAL"*) echo "$pid" ;;
            *' -d '*)        : ;;
            *)               echo "$pid" ;;
        esac
    done
}

kill_everything() {
    # Pattern-broken pkill -TERM only. NEVER -9. Covers live_radio explicitly.
    pkill -TERM -f 'tracker''_producer.py'   2>/dev/null
    pkill -TERM -f 'band''_producer.py'      2>/dev/null
    pkill -TERM -f 'examples/clock''_bias'   2>/dev/null
    pkill -TERM -f 'clock_bias''_shadow.py'  2>/dev/null
    pkill -TERM -f 'gpsdo''_probe.py'        2>/dev/null
    local holders
    holders=$(pro_radio_holders)
    [ -n "$holders" ] && kill -TERM $holders 2>/dev/null
    sleep 3
}

radio_is_free() {
    # true only when nothing holds (or could claim) the Pro
    [ -z "$(pro_radio_holders)" ]
}

reset_board() {
    # hackrf_spiflash -R, exit code CHECKED, exponential backoff, max 3 tries.
    # Returns 0 on success; on total failure touches FAILED marker, returns 1.
    # NO hackrf_debug here, ever — it can silently revert the FPGA slot.
    local attempt backoff rc
    backoff=10
    for attempt in 1 2 3; do
        log "board reset attempt $attempt/3: hackrf_spiflash -R"
        "$SPIFLASH" -d "$PRO_SERIAL" -R
        rc=$?
        if [ "$rc" -eq 0 ]; then
            log "board reset OK (attempt $attempt)"
            sleep 6
            return 0
        fi
        log "board reset FAILED rc=$rc (attempt $attempt)"
        if [ "$attempt" -lt 3 ]; then
            log "backing off ${backoff}s before retry"
            sleep "$backoff"
            backoff=$(( backoff * 2 ))
        fi
    done
    log "board reset failed 3x — touching $FAILED_MARKER and standing down. HUMAN NEEDED (likely a physical replug, per AGENTS.md 2026-08-29 incident)."
    touch "$FAILED_MARKER"
    return 1
}

relaunch_producers() {
    # Documented order (v1 lineage + AGENTS restart law): tracker first,
    # after the board reset; then the rest.
    log "relaunching producers (tracker first)"
    nohup python3 scripts/tracker_producer.py    >> /tmp/tracker_producer.log   2>&1 &
    sleep 3
    nohup ./target/release/examples/clock_bias   >> /tmp/clock_bias.log         2>&1 &
    nohup python3 scripts/band_producer.py       >> /tmp/band_producer.log      2>&1 &
    nohup python3 scripts/gpsdo_probe.py         >> /tmp/gpsdo_probe.log        2>&1 &
    nohup python3 scripts/clock_bias_shadow.py   >> /tmp/clock_bias_shadow.log  2>&1 &
}

operator_window_opened() {
    # 1: maintenance.lock / FAILED marker can appear MID-recovery; honor them
    # immediately, not only at the top of the next cycle. Returns 0 (and logs)
    # when recovery must abort. Producers stay down — the operator owns the
    # window and restores in their own order.
    local where="$1"
    if [ -f "$LOCKFILE" ]; then
        log "ABORTING recovery ($where): $LOCKFILE appeared mid-recovery — operator owns the window; leaving producers down"
        return 0
    fi
    if [ -f "$FAILED_MARKER" ]; then
        log "ABORTING recovery ($where): $FAILED_MARKER appeared mid-recovery — standing down; leaving producers down"
        return 0
    fi
    return 1
}

recover() {
    log "OUTAGE: $1 — initiating recovery"
    kill_everything

    # b: the radio must be verifiably free before any reset
    local tries=0
    while ! radio_is_free; do
        tries=$(( tries + 1 ))
        if [ "$tries" -gt 3 ]; then
            log "orphan live_radio survived ${tries}x TERM — will NOT reset or relaunch (never -9). Retrying next cycle."
            return 1
        fi
        log "PRO-radio holder(s) [$(echo $(pro_radio_holders))] — TERM again (try $tries)"
        kill -TERM $(pro_radio_holders) 2>/dev/null
        sleep 3
    done
    log "radio verified free of orphans"

    # 1: re-check operator markers immediately before the board reset
    if operator_window_opened "before reset_board"; then
        return 1
    fi

    # c: checked reset; on failure do NOT relaunch the tracker
    if ! reset_board; then
        return 1
    fi

    # 1: and again before relaunch — the reset takes seconds; a window can open
    if operator_window_opened "before relaunch_producers"; then
        return 1
    fi

    relaunch_producers
    log "recovery complete — ${POST_RECOVERY_BACKOFF_S}s backoff"
    sleep "$POST_RECOVERY_BACKOFF_S"
    return 0
}

# --- main loop --------------------------------------------------------------
cycle=0
consec_recoveries=0   # 3: recover() invocations without an intervening healthy cycle
lock_last_seen=0      # 4: epoch when maintenance.lock was last observed present
while true; do
    cycle=$(( cycle + 1 ))

    # c: after a declared failure, never act again until a human clears it
    if [ -f "$FAILED_MARKER" ]; then
        log "heartbeat cycle=$cycle status=FAILED marker=$FAILED_MARKER — standing down (remove the marker after manual recovery)"
        sleep "$CYCLE_S"
        continue
    fi

    # d: operator maintenance window wins, always
    if [ -f "$LOCKFILE" ]; then
        lock_last_seen=$(date +%s)   # 4: remember for the post-window grace
        log "heartbeat cycle=$cycle status=MAINTENANCE $LOCKFILE present — not acting"
        sleep "$CYCLE_S"
        continue
    fi

    # 4: post-window grace — for LOCK_GRACE_S after the lock disappears the
    # just-restored producers' state files are legitimately stale; check pgrep
    # liveness only so staleness cannot trigger a spurious recovery.
    grace=0
    if [ "$lock_last_seen" -gt 0 ] && [ $(( $(date +%s) - lock_last_seen )) -lt "$LOCK_GRACE_S" ]; then
        grace=1
    fi

    failures=$(health_check "$grace")
    if [ -n "$failures" ]; then
        # 3: consecutive-recovery give-up — same standdown path as reset failure
        if [ "$consec_recoveries" -ge "$MAX_CONSEC_RECOVERIES" ]; then
            log "GIVE-UP: $consec_recoveries consecutive recoveries without a healthy cycle — touching $FAILED_MARKER and standing down. HUMAN NEEDED."
            echo "$(date -u +%Y-%m-%dT%H:%M:%SZ) supervisor_v2[$$] gave up: $consec_recoveries consecutive recover() invocations without an intervening healthy cycle (last failures: $(echo "$failures" | tr '\n' ' '))" >> "$FAILED_MARKER"
            sleep "$CYCLE_S"
            continue
        fi
        consec_recoveries=$(( consec_recoveries + 1 ))
        recover "$(echo "$failures" | tr '\n' ' ')"
    else
        consec_recoveries=0
        if [ "$grace" -eq 1 ]; then
            log "heartbeat cycle=$cycle status=OK all producers live (post-window grace: state-file staleness suppressed)"
        else
            log "heartbeat cycle=$cycle status=OK all producers live, state files fresh"
        fi
    fi

    # background-throttle acquisition workers (carried over from v1)
    for p in $(pgrep -f "(_acq|acq_)" 2>/dev/null); do
        taskpolicy -b -p "$p" 2>/dev/null
    done

    sleep "$CYCLE_S"
done
