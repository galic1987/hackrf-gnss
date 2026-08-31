#!/usr/bin/env bash
# Legacy latch poller — intentionally quarantined.
set -u
cat >&2 <<'EOF'
tdc_ts_latch_run.sh is QUARANTINED; no device was touched.

The retired poll loop could hot-spin on read failure, did not status-bracket a
capture, omitted modular-counter and build/source provenance, and could write
an incomplete run as if it were valid. Historical files remain analyzable as
exploratory evidence; this is not a final measurement procedure.
EOF
exit 78
