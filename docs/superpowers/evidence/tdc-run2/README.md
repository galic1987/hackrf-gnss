# TDC run2 evidence bundle (rescued 2026-08-28)

Raw artifacts of the 2026-08-27 TDC "calibration" runs, rescued from
`/private/tmp` (one cleanup away from unreproducible — the 2026-08-28 audit).
Canonical durable copy lives HERE in the repo; `gnss/observations/
tdc_rescue_20260827/` is the data-lake mirror.

- `tdc_cal_run2.jsonl` / `tdc_cal_run2_clean.jsonl` — the on-die
  ring-oscillator self-test runs behind the retracted "0.72 ns/bin
  calibrated" figure. NOT external-edge measurements: popcount is a
  wave-occupancy statistic aliased at the RO period; absolute scale nominal,
  never measured. See commit f2784eb (retraction).
- `tdc_pps_300.jsonl`, `tdc_pps_armed*.jsonl` — captured with `--tdc-read`,
  which forces the ring-oscillator self-test (0x30=0x03): also NOT PPS data.
- `tdc_pps_conn*.jsonl`, `tdc_pps_fixed.jsonl`, `tdc_pps_trigpace.jsonl` —
  external-trigger polling attempts: zero events (the open external-gate
  evidence).
- `tdc_pps_poll.py` — the polling harness used.
- `run2_artifact_stub_32000000.json` — the run2 summary stub formerly sitting
  untracked at the crate root as `32000000` (its sole on-disk metadata).

"Calibrated" stays off every TDC site until an external swept edge measures it.
