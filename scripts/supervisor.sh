#!/usr/bin/env bash
# supervisor.sh — retired first-generation station supervisor.
#
# This entry point is intentionally inert. The original implementation could
# reset the Pro after a partial producer outage, killed every live_radio and
# hackrf_transfer on the host, and did not honor maintenance.lock. On a bench
# that also runs a HackRF One, those are unsafe recovery semantics.
#
# supervisor_v2.sh is the maintained observe-only implementation. Automatic
# hardware recovery remains disabled until all Pro clients share one atomic
# ownership lock.

set -u

cat >&2 <<'EOF'
supervisor.sh (v1) is disabled.

It must not control this dual-radio station. For health monitoring, start the
maintained supervisor explicitly:

    scripts/supervisor_v2.sh

That command is observe-only. Read its header; there is no automatic hardware
recovery mode.
EOF

exit 64
