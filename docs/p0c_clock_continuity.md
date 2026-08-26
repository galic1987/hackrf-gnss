> **SUPERSEDED (2026-08-26)** — the 2026-08-25 archive retro found that every observable correction write (122/122, down to ±0.01 ppm) was followed by a tracker-wide lock collapse and ~1 min relock, contradicting this note's "phase-continuous / no capture-boundary requirement" verdict; the discipline loop now runs SHADOW-only (`HACKRF_GNSS_ACTUATE=1` to actuate). Authoritative state: `docs/superpowers/specs/2026-08-25-clock-write-continuity-experiment.md`. Retained for history; do not cite the verdict below.

# P0c gate: does a live clock-correction write break received phase continuity?

Date: 2026-08-25. Read-only analysis; no radio touched, no process signalled.

**Question.** The Praline firmware path for a mid-stream clock-correction
change (`firmware/common/clock_gen.c`, `IS_PRALINE` branch) disables the SGPIO
stream, reprograms Si5351C multisynths MS0 (AFE_CLK) / MS1 (FPGA_CLK), and
calls `si5351c_reset_plls(SI5351C_PLL_MASK_A)`, which disables all PLL-A-sourced
outputs for ≥2 ms (`si5351c.c:216-224`). CLKOUT (the 10 MHz feeding the HackRF
One's CLKIN) is PLL-A-sourced (`si5351c.c:472`). So every correction write
*physically* drops the One's reference for ~2 ms and gaps the Pro's own sample
stream. The gate: does the received carrier phase survive that, or must
corrections be capture-boundary operations?

**Verdict: phase-continuous — YES, for every correction step observed
(|Δ| ≤ 0.1 ppm).** 47 live correction writes were logged while the ATSC ch35
carrier-phase track (HackRF One, CLKIN-slaved to the Pro's CLKOUT) was
recording; none produced a detectable gap, phase jump, lock loss, or relock
transient beyond background noise. No event-class threshold was found within
the tested range — even the largest exercised steps (±0.1 ppm) were
transparent, including one −0.1 ppm step with a completely clean window.
Steps > 0.1 ppm are untested (see coverage caveats).

## Data analyzed

| stream | span (UTC) | contents |
|---|---|---|
| `observations/phase_history.jsonl` | 08-23 14:18 → 08-25 04:21 (38.1 h) | 131,276 rows @ ~1 Hz: `t, disp_mm, freq_off_hz, sigma_mm, lock` |
| discipline records (`archive/2026-08-2{4,5}/discipline.parquet`, from telemetry) | 08-24 21:40 → 08-25 04:16 (6.6 h) | 403 records @ 60 s, in-process loop in `examples/live_radio.rs` |
| `clock_loop_log.jsonl` / `fused_loop_log.jsonl` | 08-21, 08-22 | 3 old correction changes — **outside** phase coverage |
| `/tmp/tracker_producer.log` (read-only) | 08-24 17:47 → 08-25 00:16 | tracker restarts / lock brownouts |

Overlap of correction record and phase record: the 6.6 h discipline window,
fully covered by the phase trace.

## Events

47 correction-change epochs (every discipline record whose `correction_ppm`
differs from the previous; the loop writes hardware only when it applies a
step — `live_radio.rs:367-377` does `set_clock_corr_ppm` + same-freq retune in
the same iteration the "applied" note is logged).

- step sizes: 41 × |Δ| = 0.010–0.029 ppm, 3 × 0.030 ppm, 3 × −0.100 ppm
  (plus one more +0.030 following each −0.1; 4 writes total at |Δ| ≥ 0.03
  excluding follow-ups — all four −0.1 steps and their +0.03 follow-ups
  analyzed individually).
- On a 32 MHz AFE rate the smallest step (0.010 ppm ≈ 0.3 Hz) still changes
  the achieved Si5351 divider, so per the firmware path **all 47 writes
  physically reprogrammed the clock and reset PLL A** — this is not a
  "small steps never reach hardware" artifact.

## Results

Per event, ±10 s window on the 1 Hz phase record vs 44 matched control
windows (no write within ±120 s), plus a boundary test on the 1 s sample
straddling the write epoch:

| metric | 44 event windows | 44 control windows |
|---|---|---|
| samples present / expected | 18–21 / ~21 (max gap 4 s) | 20–21 / ~21 (max 2 s) |
| detrended phase jump, median \|·\| / max | 629 / 6027 mm | 497 / 3075 mm |
| in-window peak deviation, median | 728 mm | 642 mm |
| freq_off step, median \|·\| | 0.36 Hz | 0.35 Hz |
| lock=false rows | 0 | 0 |
| boundary \|d(disp)/dt\|, med / max | 375 / 1806 mm/s | 252 / 1777 (p99) mm/s |
| boundary \|Δfreq\|, med / max | 0.42 / 2.5 Hz | 0.42 / 1.9 (p95) Hz |

Boundary velocities of event epochs sit at the 1st–99th percentile of the
control distribution (median percentile 0.65); 3/45 events exceed control p95
vs ~2.2 expected by chance. Lock was never lost: **zero `lock=false` rows in
the entire 38.1 h phase history**, spanning all 47 writes. (λ ≈ 0.52 m at the
ch35 pilot, so 500 mm ≈ 1 carrier cycle.)

### The three −0.1 ppm steps — confounded, not write-caused

Each −0.1 ppm step (22:57:20, 00:25:45, 01:25:20) coincides with a
phase-producer restart row (`disp=0, sigma=null`). Causality runs the other
way: in all three cases the phase producer's outage **began 68–79 s before
the write**, and the tracker itself was sick at the same moment — the −0.1
step was the loop reacting to a bogus −0.37 ppm residual measured on the
dying tracker (the tracker producer restarted 17 s–4 min later; the midnight
restart cluster 23:55–00:09 matches the manual radio-work dance in
AGENTS.md). The fourth large step (00:08:51, −0.1 ppm) has a completely clean
phase window. Phase-producer restarts are common (20 in 38 h); only these 3
land near writes, and all three predate their write.

### Tracker-side brownouts

`/tmp/tracker_producer.log` channel-count collapses ("0 channels, 0 locked")
occur only at producer restarts, never at routine correction writes. The
Pro's own stream does take the SGPIO disable hit per write (firmware
guarantees it), but the in-process loop rides through it via
`note_clock_step` (`live_radio.rs:377`) — no relock brownout is logged at any
of the 47 write epochs.

## Why the One survives (interpretation)

The ~2 ms CLKOUT dropout is short against the One's Si5351C loop bandwidth:
the VCO effectively holds over, and relock lands within a fraction of a
carrier cycle — small enough that the phase tracker's 30 Hz ENBW loop absorbs
it inside one 1 s aggregate. The Pro's own RX path is the one that provably
loses samples (SGPIO gap); the shared-clock consumer does not lose phase.

## Implication for the discipline loop

- Live bounded-slew at the current step sizes (≤0.03 ppm fine, 0.1 ppm
  coarse, 60 s cadence) is **compatible with continuous phase tracking** on
  the CLKIN-slaved receiver. No capture-boundary requirement is indicated by
  the data at these step sizes.
- Keep the 0.1 ppm coarse-step cap for live writes. Larger corrections are
  untested (the only historical >0.1 ppm changes — Aug 21: +0.84 then
  +1.77 ppm; Aug 22 fused: one write — predate phase recording). Until a
  >0.1 ppm step is observed clean, treat correction changes >0.1 ppm as
  capture-boundary operations by policy, not by evidence of harm.
- The −0.1 ppm events also expose a loop-hygiene bug worth fixing on its own
  merits: the loop stepped on a −0.37 ppm residual measured from a dying
  tracker (WAAS lock collapsing → host incident), then walked back over
  ~8 min. A plausibility gate on |residual| before stepping would have
  suppressed all three bad writes.

## Caveats / coverage gaps

- Sensitivity floor: 1 Hz aggregates of a 30 Hz loop; a sub-cycle phase slip
  (≲500 mm ≲ 1 cycle) inside one sample would hide in the ±280 mm/s median
  background velocity. Slips ≥ ~1–2 cycles are excluded by the jump test;
  16.7 ms epochs are not recorded in `phase_history.jsonl`.
- Correction writes are only logged where discipline records exist
  (08-24 21:40 → 08-25 04:16). The tracker ran 17:47–21:40 on 08-24 with no
  telemetry collection — any writes in that window are unlogged and untested.
- All tested steps were ≤0.1 ppm, corrections confined to [−0.48, −0.34] ppm.
  Nothing here speaks to first-lock corrections from 0, or to |Δ| > 0.1 ppm.
- Analysis is correlational; the write epoch is the discipline record's wall
  time (±~1 s vs the actual USB transaction).

## Reproduction

Analysis scripts were run as heredoc python (read-only) against:
`observations/phase_history.jsonl`,
`observations/archive/2026-08-2{4,5}/discipline.parquet`,
`clock_loop_log.jsonl`, `fused_loop_log.jsonl`, `/tmp/tracker_producer.log`.
Firmware references: `mac-archive/hackrf` branch `upstream-pr-submit`:
`firmware/common/clock_gen.c:331-417` (stream disable, MS0/MS1, PLL-A reset),
`firmware/common/si5351c.c:216-224,472` (PLL-A output disable incl. CLKOUT),
`firmware/common/radio.c:266-352,515,672-675` (correction → new_afe_rate →
apply), `firmware/hackrf_usb/usb_api_transceiver.c:646` (mid-stream
`radio_update`).
