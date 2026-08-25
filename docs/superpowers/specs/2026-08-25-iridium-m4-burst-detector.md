# Iridium burst detector on the Cortex-M4 — spec

Date: 2026-08-25. Status: specification only (user decision 2026-08-25:
"both, offline first" — the offline half already exists).

## Existing assets (do not rebuild)

- `src/iridium/` in this crate: complete host pipeline (~4.1k lines) — burst
  finding, demod, BCH FEC, frame parse, geolocation
  (`examples/iridium_decode.rs` runs capture → bursts → RWA lines, validated
  against the reference demod3; `examples/iridium_ppm.rs` does TLE-Doppler
  ppm cross-checks).
- `gnss/iridium-toolkit/` reference Python.

## Goal

The HackRF Pro's Cortex-M4 flags Iridium burst windows in real time so the
host decodes without streaming the full band continuously. Per the systems
review: the M4 owns services (Viterbi/framing/policy/SPI); the M0 stays
SGPIO dataplane only; the FPGA owns correlation/timestamps.

## Constraints (from the 2026-08-25 systems review)

- The M4's RX path is a continuous service loop — a detector hook must live
  INSIDE the RX loop; a bare timer flag does not guarantee service.
- Detector must not starve the GNSS tracker: the Pro is owned 24/7 by
  `live_radio` at L1. Iridium (1626.25 MHz) is out-of-window for the current
  1575.42 MHz front-end config — so on-chip detection implies either
  (a) scheduled band visits (retune, dwell, return — each visit costs the
  tracker a relock; see the clock-write continuity findings for how
  expensive stream disruptions are), or (b) piggybacking a future
  second-tuner arrangement. THIS IS THE GATING DESIGN QUESTION.
- M4 compute budget is ample for an energy detector at decimated rate
  (the GNSS work reserves the M4 for exactly such services).

## Detector design (when the band-visit question is answered)

- Window: 1626.0–1626.5 MHz (matches `demod3::run_capture` bounds).
- Algorithm: band-energy detector on decimated IQ (e.g. 4 Msps → 1 MHz
  decimation → 1 ms energy buckets), adaptive threshold over a rolling
  noise floor (CFAR-style), burst = energy > floor × k for ≥ 2 consecutive
  buckets. Iridium bursts are ~8.28 ms TDMA slots — a 1 ms grid resolves
  them with margin.
- Output: burst events {timestamp (FPGA tick), center freq estimate,
  peak SNR} to the host via an SPI-readable register block or USB log —
  with a full timestamp anchor, sequential indices, and sticky overflow
  (the review's FIFO contract requirements apply verbatim).
- Host side: `examples/iridium_decode.rs` gains a mode consuming flagged
  windows instead of full-capture scanning; decoded RWA lines unchanged.

## Open questions before implementation

1. Band-visit policy: how often, for how long, and who pays the tracker
   relock cost? (Alternative: only detect while the tracker is already
   dark, e.g. night-time SBAS gaps.)
2. Register/FIFO contract (per the review's §3 requirements: epoch tags,
   generation counters, 256+ entries, sticky overflow).
3. Whether energy detection suffices or the FPGA should pre-correlate the
   Iridium sync word (better sensitivity, more gateware).

## Honest status line for the panel

"Specification only" — no firmware written; host decode exists and is
validated.
