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
