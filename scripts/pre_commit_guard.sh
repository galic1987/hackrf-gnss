#!/usr/bin/env bash
# ==============================================================================
# scripts/pre_commit_guard.sh
# ==============================================================================
# Pre-commit Hook & Safety Guard for Station Governance
#
# Enforces:
# 1. Host Process Peace Treaty (docs/GOVERNANCE.md Article IV):
#    Guarantees pre-commit checks never invoke cargo test, release builds, or
#    broad process kills while live_radio or HackRF lease is active.
# 2. Epistemic Mandate (docs/GOVERNANCE.md Article II):
#    Validates that any staged `state.*.json` strictly contains `source`,
#    `input_counts` (>= 0), and `synthetic: false`.
# 3. Specification Gate (docs/GOVERNANCE.md Article III):
#    Rejects un-registered verdict fields (`pass`/`fail`/`ok`) in staged state.
# 4. Quarantine Integrity (docs/GOVERNANCE.md Article VI):
#    Ensures quarantined engines remain inert with exit 78 stubs.
#
# Can be installed to .git/hooks/pre-commit
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

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

echo -e "${BOLD}[Station Guard] Running pre-commit verification...${NC}"

# ------------------------------------------------------------------------------
# 1. Check Live Radio Safety Guard
# ------------------------------------------------------------------------------
if [ -x "${REPO_ROOT}/scripts/guard_live_radio.sh" ]; then
    if "${REPO_ROOT}/scripts/guard_live_radio.sh" check >/dev/null 2>&1; then
        echo -e "${GREEN}  ✓ Live radio status: IDLE (safe for workspace tasks)${NC}"
    else
        echo -e "${YELLOW}  ! Live radio status: ACTIVE (Hardware lease held)${NC}"
        echo -e "    Enforcing Host Process Peace Treaty: Suppressing heavy builds & test suites during commit."
    fi
fi

# ------------------------------------------------------------------------------
# 2. Validate Staged State Files against Epistemic Mandate & Specification Gate
# ------------------------------------------------------------------------------
STAGED_STATE_FILES=$(git diff --cached --name-only --diff-filter=ACM | grep -E '(^|/)state\.[^/]+\.json$' || true)

if [ -n "$STAGED_STATE_FILES" ]; then
    echo -e "${BOLD}  Checking staged state files against Epistemic Mandate...${NC}"
    for file in $STAGED_STATE_FILES; do
        # Extract staged content from git index
        python3 -c '
import json, sys

path = sys.argv[1]
try:
    content = sys.stdin.read()
    if not content.strip():
        sys.exit(0)
    data = json.loads(content)
except Exception as e:
    print(f"ERROR: {path} is not valid JSON: {e}", file=sys.stderr)
    sys.exit(1)

# Epistemic Mandate Checks
if "source" not in data or not data["source"]:
    print(f"REJECTED: {path} violates Epistemic Mandate: missing \"source\" identifier.", file=sys.stderr)
    sys.exit(1)

if "input_counts" not in data:
    print(f"REJECTED: {path} violates Epistemic Mandate: missing \"input_counts\" field.", file=sys.stderr)
    sys.exit(1)

if not isinstance(data["input_counts"], int) or data["input_counts"] < 0:
    ic = data.get("input_counts")
    print(f"REJECTED: {path} violates Epistemic Mandate: \"input_counts\" must be non-negative integer (got {ic}).", file=sys.stderr)
    sys.exit(1)

if "synthetic" not in data:
    print(f"REJECTED: {path} violates Epistemic Mandate: missing explicit \"synthetic: false\" flag.", file=sys.stderr)
    sys.exit(1)

if data["synthetic"] is not False:
    print(f"REJECTED: {path} violates Epistemic Mandate: \"synthetic\" is not false. Simulated data must be prefixed sim.* and quarantined.", file=sys.stderr)
    sys.exit(1)

# Specification Gate Checks
forbidden_verdicts = ["verdict", "pass", "fail", "ok"]
found_verdicts = [k for k in forbidden_verdicts if k in data]
if found_verdicts:
    # Check if specification metadata is provided
    has_spec = ("specification_ref" in data or "spec" in data or "audited_by" in data)
    if not has_spec:
        print(f"REJECTED: {path} violates Specification Gate: verdict keys {found_verdicts} present without audited specification reference.", file=sys.stderr)
        sys.exit(1)

print(f"  ✓ {path} complies with Epistemic Mandate (source, input_counts, synthetic: false).")
' "$file" < <(git show ":$file") || {
            echo -e "${RED}[PRE-COMMIT REFUSAL] Commit blocked due to Epistemic Mandate / Specification Gate violation.${NC}" >&2
            exit 1
        }
    done
fi

# ------------------------------------------------------------------------------
# 3. Quarantine Integrity: Ensure Quarantined Engines Are Not Re-armed
# ------------------------------------------------------------------------------
QUARANTINED_ENGINES=(
    "carrier_single_difference_engine.py"
    "carrier_double_difference_engine.py"
    "carrier_triple_difference_engine.py"
    "lambda_ambiguity_resolution_engine.py"
    "hatch_divergence_sounder.py"
    "hoi_refraction_engine.py"
    "iono_tid_analyzer.py"
    "agw_tid_wavevector_engine.py"
    "solar_dawn_detector.py"
    "solar_flare_sid_monitor.py"
    "solar_noon_photochemistry_engine.py"
    "multi_frequency_linear_combinations.py"
    "tropo_saastamoinen_model.py"
    "tropospheric_refractivity_ducting_sounder.py"
    "gnss_meteorology_pwv.py"
    "earth_solid_tide_sounder.py"
    "solar_radiation_pressure_sounder.py"
    "relativistic_space_time_inspector.py"
    "rf_link_budget_radiometer.py"
    "frontend_iq_imbalance_sounder.py"
    "gnss_reflectometry_sounder.py"
    "ppp_sequential_ekf_engine.py"
    "gdop_error_ellipsoid_analyzer.py"
    "satellite_atomic_clock_analyzer.py"
    "post_sunrise_flux_tracker.py"
    "metrology_suite_daemon.sh"
)

STAGED_QUARANTINE_FILES=$(git diff --cached --name-only --diff-filter=ACM || true)
for qf in "${QUARANTINED_ENGINES[@]}"; do
    if echo "$STAGED_QUARANTINE_FILES" | grep -q "scripts/${qf}$"; then
        echo -e "${BOLD}  Checking quarantine integrity for scripts/${qf}...${NC}"
        staged_code=$(git show ":scripts/${qf}")
        if ! echo "$staged_code" | grep -E -q "exit\(?78\)?|EX_CONFIG|QUARANTINED"; then
            echo -e "${RED}[PRE-COMMIT REFUSAL] Quarantined engine scripts/${qf} cannot be re-armed with active execution code.${NC}" >&2
            echo -e "Quarantined engines must remain exit-78 stubs per Order 1 & Station Governance Charter." >&2
            exit 1
        fi
        echo -e "${GREEN}  ✓ scripts/${qf} remains quarantined (exit 78).${NC}"
    fi
done

echo -e "${GREEN}✓ Pre-commit guards passed successfully.${NC}"
exit 0
