#!/usr/bin/env bash
# ==============================================================================
# scripts/guard_live_radio.sh
# ==============================================================================
# Station Safety Guard: Live Radio & Device Lease Protection
#
# Enforces Article IV of docs/GOVERNANCE.md (The Host Process Peace Treaty).
#
# 1. Checks if `live_radio` or any process holding a HackRF device lease is active.
# 2. Refuses to allow `cargo test`, `cargo build --release`, or broad `pkill`
#    commands while `live_radio` holds the hardware lease.
#
# Usage:
#   scripts/guard_live_radio.sh status
#   scripts/guard_live_radio.sh check
#   scripts/guard_live_radio.sh exec <command...>
#   scripts/guard_live_radio.sh <command...>
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [ -d "${REPO_ROOT}/../observations" ]; then
    OBS_DIR="$(cd "${REPO_ROOT}/../observations" && pwd)"
elif [ -d "${REPO_ROOT}/observations" ]; then
    OBS_DIR="$(cd "${REPO_ROOT}/observations" && pwd)"
else
    OBS_DIR="/Volumes/Radiator 8TB/gnss/observations"
fi
LEASE_DIR="${OBS_DIR}/pro.radio.lock.d"
OWNER_JSON="${LEASE_DIR}/owner.json"
MAINT_LOCK="${OBS_DIR}/maintenance.lock"

# Color formatting (if terminal)
if [ -t 2 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    BOLD='\033[1m'
    NC='\033[0m' # No Color
else
    RED=''
    GREEN=''
    YELLOW=''
    BLUE=''
    BOLD=''
    NC=''
fi

# Detect active live_radio processes and HackRF leases
find_active_radio_processes() {
    # Returns PIDs of live_radio or active HackRF stream tools
    # Exclude grep and this guard script itself
    local pids=()
    
    # 1. live_radio binary
    while IFS= read -r pid; do
        [ -n "$pid" ] && pids+=("$pid")
    done < <(pgrep -f "target/release/examples/live_radio" 2>/dev/null || true)

    # 2. fallback search for live_radio if full path differed
    while IFS= read -r pid; do
        [ -n "$pid" ] && [[ ! " ${pids[*]:-} " =~ " ${pid} " ]] && pids+=("$pid")
    done < <(pgrep -x "live_radio" 2>/dev/null || true)

    # 3. hackrf_transfer or hackrf_rx
    while IFS= read -r pid; do
        [ -n "$pid" ] && [[ ! " ${pids[*]:-} " =~ " ${pid} " ]] && pids+=("$pid")
    done < <(pgrep -f "hackrf_transfer" 2>/dev/null || true)

    # 4. Check lease file PID from owner.json
    if [ -f "$OWNER_JSON" ]; then
        local lease_pid
        lease_pid=$(python3 -c '
import json, sys
try:
    with open(sys.argv[1]) as f:
        d = json.load(f)
        pid = d.get("pid")
        if pid:
            print(pid)
except Exception:
    pass
' "$OWNER_JSON" 2>/dev/null || true)
        if [ -n "$lease_pid" ] && kill -0 "$lease_pid" 2>/dev/null; then
            if [[ ! " ${pids[*]:-} " =~ " ${lease_pid} " ]]; then
                pids+=("$lease_pid")
            fi
        fi
    fi

    echo "${pids[@]:-}"
}

is_radio_active() {
    local active_pids
    active_pids=$(find_active_radio_processes)
    if [ -n "${active_pids}" ]; then
        return 0 # Active
    fi
    if [ -d "$LEASE_DIR" ] && [ -f "$OWNER_JSON" ]; then
        return 0 # Lease directory exists and owner.json is present
    fi
    return 1 # Idle
}

get_lease_details() {
    if [ -f "$OWNER_JSON" ]; then
        python3 -c '
import json, sys
try:
    with open(sys.argv[1]) as f:
        d = json.load(f)
        role = d.get("role", "unknown")
        pid = d.get("pid", "unknown")
        serial = d.get("serial", "unknown")
        print(f"Role:   {role}")
        print(f"PID:    {pid}")
        print(f"Serial: {serial}")
except Exception as e:
    print(f"Malformed owner.json: {e}")
' "$OWNER_JSON" 2>/dev/null || echo "Unable to parse ${OWNER_JSON}"
    else
        echo "No owner.json found at ${OWNER_JSON}"
    fi
}

print_active_pids_info() {
    local active_pids
    active_pids=$(find_active_radio_processes)
    if [ -n "$active_pids" ]; then
        echo -e "${YELLOW}Active Radio Processes:${NC}" >&2
        for pid in $active_pids; do
            ps -p "$pid" -o pid,user,%cpu,%mem,comm,args | tail -n +2 | sed 's/^/  /' >&2 || true
        done
    fi
}

check_forbidden_command() {
    local cmd_str="$*"
    
    # Check 1: cargo test
    if [[ "$cmd_str" =~ (^|[[:space:]])cargo([[:space:]]+.*)?[[:space:]]+test([[:space:]]|$) ]]; then
        return 0 # Forbidden
    fi

    # Check 2: cargo build --release or cargo build -r
    if [[ "$cmd_str" =~ (^|[[:space:]])cargo([[:space:]]+.*)?[[:space:]]+build([[:space:]]+.*)?(--release|-r)([[:space:]]|$) ]]; then
        return 0 # Forbidden
    fi

    # Check 3: broad un-niced cargo build
    if [[ "$cmd_str" =~ (^|[[:space:]])cargo[[:space:]]+build([[:space:]]|$) ]]; then
        return 0 # Forbidden
    fi

    # Check 4: broad pkill or killall targeting radio or general processes
    if [[ "$cmd_str" =~ (^|[[:space:]])pkill([[:space:]]+.*)?(live_radio|hackrf|cargo|python)([[:space:]]|$) ]]; then
        return 0 # Forbidden
    fi
    if [[ "$cmd_str" =~ (^|[[:space:]])killall([[:space:]]+.*)?(live_radio|hackrf|cargo|python)([[:space:]]|$) ]]; then
        return 0 # Forbidden
    fi
    if [[ "$cmd_str" =~ (^|[[:space:]])pkill[[:space:]]+-[0-9]+[[:space:]]+live_radio([[:space:]]|$) ]]; then
        return 0 # Forbidden
    fi

    return 1 # Not forbidden
}

# Main Command Dispatcher
if [ $# -eq 0 ]; then
    echo "Usage: $0 {status|check|require-idle|exec <cmd...>|<cmd...>}" >&2
    exit 1
fi

ACTION="$1"

case "$ACTION" in
    status)
        echo -e "${BOLD}=== Station Live Radio Guard Status ===${NC}"
        if is_radio_active; then
            echo -e "Radio Status: ${RED}${BOLD}ACTIVE (Hardware Lease Held)${NC}"
            echo -e "\n${BOLD}Lease Registration:${NC}"
            get_lease_details
            echo ""
            print_active_pids_info
            echo -e "\n${YELLOW}Policy:${NC} Heavy compilations, test suites, and broad pkill commands are ${RED}${BOLD}REFUSED${NC}."
            exit 0
        else
            echo -e "Radio Status: ${GREEN}${BOLD}IDLE (No Hardware Lease Active)${NC}"
            echo -e "Safe for compilations, tests, and maintenance operations."
            exit 0
        fi
        ;;

    check)
        if is_radio_active; then
            echo -e "${RED}[GUARD REFUSAL] Live radio or HackRF device lease is ACTIVE.${NC}" >&2
            print_active_pids_info
            exit 1
        else
            echo -e "${GREEN}[GUARD OK] Live radio is IDLE. Safe to proceed.${NC}"
            exit 0
        fi
        ;;

    require-idle)
        if is_radio_active; then
            echo -e "\n${RED}${BOLD}==============================================================================${NC}" >&2
            echo -e "${RED}${BOLD} [VIOLATION] REFUSED BY STATION SAFETY GUARD: LIVE RADIO LEASE HELD           ${NC}" >&2
            echo -e "${RED}${BOLD}==============================================================================${NC}" >&2
            echo -e "A live radio process or exclusive HackRF device lease is currently active.\n" >&2
            get_lease_details >&2
            echo "" >&2
            print_active_pids_info
            echo -e "\n${BOLD}Authority:${NC} docs/GOVERNANCE.md (Article IV: The Host Process Peace Treaty)" >&2
            echo -e "${YELLOW}Host stalls >115 ms cause HackRF FIFO overflow and catastrophic loss of carrier lock.${NC}" >&2
            echo -e "\nTo perform this operation, you must first transition the station into a maintenance window:" >&2
            echo -e "  1. python3 scripts/pro_lease.py gate --serial <SERIAL> --token-file /tmp/pro-maint.token" >&2
            echo -e "  2. Gracefully stop tracker_producer/live_radio" >&2
            echo -e "  3. python3 scripts/pro_lease.py acquire --token-file /tmp/pro-maint.token --wait-seconds 30" >&2
            echo -e "  4. Reset the board immediately: hackrf_spiflash -R" >&2
            echo -e "  5. Execute your compilation, tests, or maintenance" >&2
            echo -e "  6. python3 scripts/pro_lease.py release --token-file /tmp/pro-maint.token\n" >&2
            exit 78 # EX_CONFIG
        fi
        exit 0
        ;;

    exec)
        shift
        COMMAND=("$@")
        COMMAND_STR="${COMMAND[*]}"
        
        if is_radio_active && check_forbidden_command "$COMMAND_STR"; then
            echo -e "\n${RED}${BOLD}==============================================================================${NC}" >&2
            echo -e "${RED}${BOLD} [BLOCKED] FORBIDDEN COMMAND WHILE LIVE RADIO HOLDS HARDWARE LEASE            ${NC}" >&2
            echo -e "${RED}${BOLD}==============================================================================${NC}" >&2
            echo -e "${RED}Command blocked:${NC} ${BOLD}${COMMAND_STR}${NC}\n" >&2
            echo -e "This command violates ${BOLD}Article IV of docs/GOVERNANCE.md (The Host Process Peace Treaty).${NC}" >&2
            echo -e "Executing 'cargo test', 'cargo build --release', or broad 'pkill' commands during" >&2
            echo -e "live streaming overflows the ~190ms USB queue and destroys carrier-phase continuity.\n" >&2
            get_lease_details >&2
            echo "" >&2
            print_active_pids_info
            echo -e "\n${YELLOW}To run this command, use a token-gated maintenance window:${NC}" >&2
            echo -e "  python3 scripts/pro_lease.py gate --serial <SERIAL> --token-file /tmp/pro-maint.token" >&2
            echo -e "  (stop tracker gracefully, acquire lease, reset board, then build/test)" >&2
            exit 78 # EX_CONFIG
        fi
        exec "${COMMAND[@]}"
        ;;

    *)
        # Default fallback: Treat as command to run under exec check
        COMMAND=("$@")
        COMMAND_STR="${COMMAND[*]}"
        
        if is_radio_active && check_forbidden_command "$COMMAND_STR"; then
            echo -e "\n${RED}${BOLD}==============================================================================${NC}" >&2
            echo -e "${RED}${BOLD} [BLOCKED] FORBIDDEN COMMAND WHILE LIVE RADIO HOLDS HARDWARE LEASE            ${NC}" >&2
            echo -e "${RED}${BOLD}==============================================================================${NC}" >&2
            echo -e "${RED}Command blocked:${NC} ${BOLD}${COMMAND_STR}${NC}\n" >&2
            echo -e "This command violates ${BOLD}Article IV of docs/GOVERNANCE.md (The Host Process Peace Treaty).${NC}" >&2
            echo -e "Executing 'cargo test', 'cargo build --release', or broad 'pkill' commands during" >&2
            echo -e "live streaming overflows the ~190ms USB queue and destroys carrier-phase continuity.\n" >&2
            get_lease_details >&2
            echo "" >&2
            print_active_pids_info
            echo -e "\n${YELLOW}To run this command, use a token-gated maintenance window:${NC}" >&2
            echo -e "  python3 scripts/pro_lease.py gate --serial <SERIAL> --token-file /tmp/pro-maint.token" >&2
            echo -e "  (stop tracker gracefully, acquire lease, reset board, then build/test)" >&2
            exit 78 # EX_CONFIG
        fi
        exec "${COMMAND[@]}"
        ;;
esac
