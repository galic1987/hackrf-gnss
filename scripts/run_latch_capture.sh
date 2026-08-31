#!/usr/bin/env bash
# Legacy capture entry point — intentionally quarantined.
set -u
cat >&2 <<'EOF'
run_latch_capture.sh is QUARANTINED; no device was touched.

The retired procedure masked slot/stream/clock failures, switched images with
hackrf_debug, overwrote a fixed evidence file, and recorded no claim-grade
build/source/return-code provenance or restoration state. It must not be used
for final calibration. Design a maintenance-window capture with atomic lock
ownership and explicit manifest/source attestation first.
EOF
exit 78
