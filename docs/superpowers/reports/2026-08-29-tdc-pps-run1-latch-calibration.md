# 2026-08-29 — TDC PPS Run 1 + Run 3 (Latch Calibration SUCCESS)

Window: 21:59–23:30 EDT (Run 1), 08:25–08:45 EDT next day (Run 3). Pro #2 (645061de).
Topology: Bodnar LBE-1421 GPSDO — OUT2 10 MHz split to both radios' CLKIN (matched cables), OUT1 1PPS split to both radios' P28 pin 16 TRIGGER.IN (matched cables). P28 pinout correction of this date stands: pin 16 = TRIGGER.IN on Pro and One alike.

Evidence: `gnss/observations/tdc_pps_run1.jsonl` (3,658 events, TCXO), `gnss/observations/ts_latch_run3.jsonl` (1200 s, CLKIN), analyzers `scripts/tdc_pps_analyze.py` and `scripts/tdc_ts_analyze.py`.

*Amended (2026-08-30): The previous version of this report incorrectly voided Run 1 and falsely claimed the Bodnar 1PPS was biased or steered on a 16-tap comb. This amendment corrects the record using the definitive Run 1 + Run 3 differential measurement.*

## 1. Run 1 & Run 3 — The Differential Measurement

Run 1 was a valid external single-edge TDC dataset captured on the default boot AFE clock of 40 MHz (the Pro's free-running TCXO, because no RX stream had yet requested a clock switch).
- Run 1 measured: TCXO vs PPS = −0.66719 ± 0.00003 ppm.

Run 3 was a valid latch capture run after forcing an RX stream at 16 Msps, which locked the Si5351 to the Bodnar 10 MHz CLKIN.
- Run 3 measured: 32,000,000.000834 ticks per PPS interval (VCO vs PPS ≈ 0.000 ppm) over 20 minutes with zero trend and 1-tick (32 ns) quantisation variance.

**Conclusion:** Subtracting the two gives the exact offset of the HackRF's internal TCXO against the Bodnar VCO: the HackRF TCXO sits exactly 0.667 ppm below the Bodnar. The Bodnar's 10 MHz and 1PPS are perfectly coherent with each other. The hypothesis that the Bodnar 1PPS was biased or broken is decisively refuted.

## 2. FPGA Fabric DNL vs. Bodnar Comb

The prior claim that the Bodnar 1PPS steers on a ~16-tap comb (quantising its edges) is **false**. 
The comb pattern aligns perfectly with the FPGA fabric hop boundaries. It is a TDC Differential Non-Linearity (DNL) artifact, a known property of routing delays in the FPGA logic fabric, creating "fast" and "slow" taps.

This means code-density DNL calibration is absolutely necessary before making any sub-ns physical time conversions (e.g., "400 ps comb", "1.3 ns window", "499 ps RMS"). The raw tap size cannot be linearly inferred from the in-window fraction without DNL mapping.

## 3. Retraction of Amendment 893526c

The conclusions drawn in commit 893526c regarding the Bodnar's output-1 synthesized behavior are fully retracted.
- The -0.667 ppm bias was the HackRF TCXO, not the Bodnar.
- The 16-tap comb was FPGA DNL, not the Bodnar.
- Run 1 was NOT an aliasing failure; the host polling jitter (12.0 ms) would have produced a ~480,000-tick spread if the register were free-running. The 4-tick spread proves it is a genuine hardware latch.

## 4. Coherence verdict

- Pro #2 sample clock: perfectly locked to the Bodnar 10 MHz CLKIN (Run 3 proves zero drift vs PPS over 20 min).
- HackRF One: still dark. No cross-radio coherence pairs possible until it locks. Physical RF feed check remains owner-owed.

## 5. Next Steps

1. Code-density DNL/INL calibration for the TDC to map the true tap delays, bypassing the routing-boundary artifacts.
2. HackRF One RF path physical check → splitter test → cross-radio coherence.

## 6. Midday addendum (2026-08-30) — Leg 1 hour-gate retry verdict

(Numbering restarts at 6: the da5c02d restructure dropped the f706d62 overnight addendum, which survives in git history. Verdict below supersedes it with fresh numbers.)

**Verdict: INSUFFICIENT DATA. No qualifying continuous hour has ever existed on this station.**

Collection timeline on the window-package build (nh_sync newest-window + emit gate 5→4, deployed 06:56):

- 06:56–08:20: gen `v3-1788087599` (3,498 rows). ~08:20 full-cohort wedge — tracker log and clock_bias both froze. The new supervisor (launchd `com.hormuz.tracker`, da5c02d) restarted the cohort at 09:35:09. 75-min hole.
- 09:38–12:25: gen `v3-1788096912-tb1788086830`, 4,683 rows / 2.77 h. Inside it, a **65-min clock_bias emission hole 10:56:39–12:01:32** while the tracker held 7–12 locked throughout and `clock_bias_shadow` — reading the same `state.tracker.json` — produced 1,369 rows continuously. Same pid, same gen, self-recovered: a producer-side stall, root cause open (candidate: per-sat ephemeris/freshness gate collapse below the emit floor; the `parse_rinex_gps … rejected (unit Radians)` spam in clock_bias.log is untimestamped, unproven). Second distinct stall signature of the day.

Analyzer on the current gen (12:25):

- n_sat distribution: 4 → 2,539 rows (54%), 5 → 1,901, 6 → 243. n_bds=1 on 2,309 rows.
- Quality gate (n_sat≥5, slips=0) keeps 2,144 rows → **12.9 quality rows/min vs the 56.7/min** the 3,400-row/3,600-s gate requires. Total emission duty 28.2/min — half of required even counting n_sat=4 rows.
- Segmentation (split at >1.5× median dt 1.03 s): **longest clean segment 142 s / 138 rows** vs the 3,600 s / 3,400-row gate.
- Structurally impossible today regardless: the gen began 09:38, so a qualifying hour could not exist before ~10:38 even under perfect continuity; the 65-min hole removed it.

Comparison vs overnight (f706d62: 12,838 rows/6.9 h, best segment 312 s):

- The window package did **not** move the hour gate (142 s vs 312 s; both >20× below the gate — different sky, not evidence of regression).
- **New structural finding — emit/analyzer mismatch:** the emit gate was lowered to 4 but the analyzer quality floor stayed at 5, so 54% of emitted rows are discarded before TDEV. The change raised emission volume, not analyzable density. Either the analyzer floor moves to 4 with a documented ISB caveat, or the emit gate returns to 5; as-is they work against each other.
- Binding constraints, ranked: (1) producer emission duty (~half of required); (2) single-epoch 2-s holes shattering segments under the 1.5× median-dt split; (3) producer-side stalls (two signatures in one day — full-cohort freeze at 08:20, silent consumer hole 10:57–12:01).
