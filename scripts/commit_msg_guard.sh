#!/usr/bin/env bash
# ==============================================================================
# scripts/commit_msg_guard.sh
# ==============================================================================
# Commit Message Hook & Trailer Enforcer for Station Governance
#
# Enforces Article V of docs/GOVERNANCE.md (Automated Agent Governance):
# - Every automated or agent commit must include:
#     Agent-Role: <Role Name>
#     Audited-By: <Operator / Auditor Identity>
#
# Can be installed to .git/hooks/commit-msg
# ==============================================================================

set -euo pipefail

COMMIT_MSG_FILE="$1"

# Color formatting
if [ -t 2 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BOLD='\033[1m'
    NC='\033[0m'
else
    RED=''
    GREEN=''
    YELLOW=''
    BOLD=''
    NC=''
fi

# Detect if the commit is automated / agent-assisted
# Signals:
# 1. Environment variables (AGENT_ROLE, ANTIGRAVITY_AGENT, CI)
# 2. Git author name/email contains "agent", "bot", "claude", "antigravity", "gemini"
# 3. Message explicitly contains "Agent-Role:"
# 4. In interactive development by human operator, warning is emitted if trailers omitted.

IS_AGENT_COMMIT=0

if [ -n "${AGENT_ROLE:-}" ] || [ -n "${ANTIGRAVITY_AGENT:-}" ]; then
    IS_AGENT_COMMIT=1
fi

GIT_AUTHOR_NAME=$(git var GIT_AUTHOR_IDENT 2>/dev/null | cut -d'>' -f1 || true)
if echo "$GIT_AUTHOR_NAME" | grep -E -i -q "agent|bot|claude|antigravity|gemini"; then
    IS_AGENT_COMMIT=1
fi

if grep -E -q "^[[:space:]]*Agent-Role:" "$COMMIT_MSG_FILE"; then
    IS_AGENT_COMMIT=1
fi

if [ "$IS_AGENT_COMMIT" -eq 1 ]; then
    # Must have both Agent-Role: and Audited-By:
    HAS_ROLE=0
    HAS_AUDITOR=0

    if grep -E -q "^[[:space:]]*Agent-Role:[[:space:]]+[^[:space:]]+" "$COMMIT_MSG_FILE"; then
        HAS_ROLE=1
    fi

    if grep -E -q "^[[:space:]]*Audited-By:[[:space:]]+[^[:space:]]+" "$COMMIT_MSG_FILE"; then
        HAS_AUDITOR=1
    fi

    if [ "$HAS_ROLE" -eq 0 ] || [ "$HAS_AUDITOR" -eq 0 ]; then
        echo -e "\n${RED}${BOLD}==============================================================================${NC}" >&2
        echo -e "${RED}${BOLD} [REJECTED] COMMIT MESSAGE MISSING MANDATORY AGENT TRAILERS                   ${NC}" >&2
        echo -e "${RED}${BOLD}==============================================================================${NC}" >&2
        echo -e "Per ${BOLD}Article V of docs/GOVERNANCE.md${NC}, every automated/agent commit must include:" >&2
        echo -e "  ${YELLOW}Agent-Role: <Role Name>${NC}" >&2
        echo -e "  ${YELLOW}Audited-By: <Auditor Name and Email>${NC}\n" >&2
        echo -e "Missing trailers in commit message:" >&2
        [ "$HAS_ROLE" -eq 0 ] && echo -e "  ${RED}✗ Missing: Agent-Role:${NC}" >&2
        [ "$HAS_AUDITOR" -eq 0 ] && echo -e "  ${RED}✗ Missing: Audited-By:${NC}" >&2
        echo -e "\nExample commit trailer format:" >&2
        echo -e "  Agent-Role: Governance & Guards Architect" >&2
        echo -e "  Audited-By: Ivo Galic <galic1987@gmail.com>\n" >&2
        exit 1
    fi

    echo -e "${GREEN}✓ Commit message complies with Agent Governance trailers.${NC}"
fi

exit 0
