#!/usr/bin/env bash
# tdc_sweep_window.sh — quarantined hardware procedure.
#
# The first implementation is retained in git history (1e81b5c..f764361),
# but must not operate the bench. Static review found that it could label the
# post-reset TDC clock 40 MHz when firmware initializes it at nominal 10 MHz,
# could accept torn six-byte thermometer reads, and treated deterministic
# 1 PPS/TCXO phase rotation as proof of uniform code-density excitation.
# Those defects can scale a result by 4x or conflate source visitation with
# TDC DNL. Codes 0 and 48 also have censored/composite trigger semantics.

set -u

cat >&2 <<'EOF'
TDC sweep hardware procedure is QUARANTINED; no device was touched.

Do not restore the old script as an operator shortcut. A replacement needs:

  1. exact build-ID and artifact attestation without image switching;
  2. measured TDC/adclk source and frequency, not sample-rate inference;
  3. a long-lived reader that detects a fresh valid-toggle, reads status
     before and after the six-byte frozen word, and rejects a changed status;
  4. atomic maintenance ownership and verified fail-closed restoration;
  5. an independently swept/randomized or phase-tagged stimulus whose
     uniformity evidence is bound to the capture SHA-256;
  6. explicit handling of physical interior codes 1..47 and composite code 48.

Offline occupancy files may be inspected with scripts/tdc_density_cal.py.
That analyzer will not emit absolute widths, DNL/INL, or a LUT without a
matching independent phase-uniformity evidence record.
EOF

exit 78
