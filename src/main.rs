use anyhow::{Context, Result};
use clap::{Parser, ValueEnum};
use num_complex::Complex;
use rayon::prelude::*;
use rs_hackrf::HackRf;
use rustfft::FftPlanner;
use std::collections::HashMap;
use std::fs::File;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Instant;

use hackrf_gnss::decoder;
use hackrf_gnss::dsp_calib;
use hackrf_gnss::gps;
use hackrf_gnss::rf_calib;
use hackrf_gnss::site;


#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Debug, Hash)]
enum Mode {
    Chop,
    WidebandL1,
    WidebandL2,
    Analyze,
    Sweep,
    Decode,
    Serve,
    /// Parse Iridium frames from a frames.txt and print decoded ring alerts
    Iridium,
    /// Acquire GPS L1 from an IQ file and solve a coarse-time snapshot position
    /// fix from broadcast ephemeris (acquire -> snapshot -> PVT).
    Fix,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Debug, Hash)]
enum GnssBand {
    L1,
    L2,
    L5,
    E5b,
    E6,
    G1,
}

impl GnssBand {
    fn frequency_hz(&self) -> u64 {
        match self {
            GnssBand::L1 => 1_575_420_000,
            GnssBand::L2 => 1_227_600_000,
            GnssBand::L5 => 1_176_450_000,
            GnssBand::E5b => 1_207_140_000,
            GnssBand::E6 => 1_278_750_000,
            GnssBand::G1 => 1_602_000_000,
        }
    }
}

#[derive(Parser, Debug)]
#[command(author, version, about = "HackRF GNSS Multi-Constellation Optimizer & Live Signal Decoder", long_about = None)]
struct Args {
    #[arg(short, long, value_enum, default_value_t = Mode::WidebandL1)]
    mode: Mode,

    #[arg(long, default_value_t = 1_090_000_000)]
    freq_hz: u64,

    #[arg(short, long, default_value = "wideband_l1_b1.iq")]
    file: String,

    #[arg(short, long, value_enum, num_args = 1..)]
    bands: Vec<GnssBand>,

    #[arg(long, default_value_t = 100)]
    dwell_ms: u64,

    #[arg(long, default_value_t = 10)]
    settle_ms: u64,

    #[arg(short, long, default_value_t = 5)]
    duration_secs: u64,


    #[arg(long, default_value_t = 40)]
    lna_gain: u32,

    #[arg(long, default_value_t = 30)]
    vga_gain: u32,

    #[arg(long, default_value_t = true)]
    bias_tee: bool,

    /// Front-end RX amplifier (+14 dB). Must be set explicitly on every run:
    /// the HackRF keeps this setting in hardware between programs, so leaving
    /// it untouched makes captures depend on whatever ran last.
    #[arg(long, default_value_t = false)]
    amp: bool,

    /// Serial number of the HackRF to open. Required when more than one
    /// HackRF is attached; otherwise the first enumerated device is used,
    /// which may not be the one the antenna is connected to.
    #[arg(long)]
    serial: Option<String>,

    #[arg(long, default_value_t = 8080)]
    port: u16,

    /// Seconds into the file at which to start analysing (Analyze mode).
    #[arg(long, default_value_t = 0.0)]
    offset_secs: f64,

    /// Number of 1 ms blocks to accumulate non-coherently (Analyze mode).
    #[arg(long, default_value_t = 20)]
    blocks: usize,

    /// Iridium mode: accept BCH blocks that can be REPAIRED rather than only
    /// ones that arrive clean, and allow a few symbol errors in the unique
    /// word. Recovers far more frames -- 5283 ring alerts against 2860 on this
    /// station's corpus -- at a measured 2.5% false alarm rate on pure noise,
    /// so it is only sensible where something downstream validates the result.
    #[arg(long)]
    harder: bool,

    /// Fix mode: RINEX-3 broadcast navigation file (GPS ephemeris) covering the
    /// capture day, e.g. a BRDC..._MN.rnx.
    #[arg(long, default_value = "")]
    nav: String,

    /// Fix mode: approximate receiver latitude/longitude (deg). The snapshot
    /// solver only needs this to ~150 km to resolve the millisecond
    /// ambiguity. When omitted, the canonical anchor observations/site.json
    /// is used (no hardcoded coordinates).
    #[arg(long)]
    approx_lat: Option<f64>,
    #[arg(long)]
    approx_lon: Option<f64>,

    /// Fix mode: sample rate of the IQ file (Hz) and the IF at which L1 sits in
    /// it (Hz; 0 = the file is already at baseband, e.g. SatCatch GPS captures).
    #[arg(long, default_value_t = 8_000_000.0)]
    fs_hz: f64,
    #[arg(long, default_value_t = 0.0)]
    if_hz: f64,

    /// Extended-precision capture: samples are little-endian int16 I/Q
    /// (12-bit data from the ext_precision_rx gateware, load it first with
    /// `hackrf_debug -P 2`). Requires --fs-hz <= 2.5 MHz.
    #[arg(long, default_value_t = false)]
    ext16: bool,

    /// Fix mode: GPS time-of-week (s) at the START of the capture. Negative =
    /// derive it from the system clock (fine for a live/just-taken capture).
    #[arg(long, default_value_t = -1.0)]
    tow: f64,
}

/// Acquisition decision threshold on the peak-to-second-peak ratio.
/// 1.0 is the noise floor; a real satellite at open-sky C/N0 scores far higher.
const ACQ_THRESHOLD: f32 = 2.50;

#[derive(Debug, Clone, serde::Serialize)]
pub struct SatResult {
    pub constellation: &'static str,
    pub prn: usize,
    pub metric_pk_2nd: f32,
    pub doppler_hz: f64,
    pub code_phase_chips: f64,
}

fn analyze_iq_file(filepath: &str, offset_s: f64, blocks: usize) -> Result<Vec<SatResult>> {
    println!("Analyzing Wideband Multi-Constellation Dataset: {}", filepath);
    let mut file = File::open(filepath).context("Failed to open I/Q file for analysis.")?;

    let sample_rate = 20_000_000f64;
    let ms_samples = 20_000;
    let num_blocks = blocks;

    // Analysing only the first few milliseconds meant always looking at the
    // post-start_rx transient, and at a fixed 0.1% of the file. Let the caller
    // seek so a long capture can be examined anywhere.
    if offset_s > 0.0 {
        let byte_off = (offset_s * sample_rate).round() as u64 * 2;
        use std::io::Seek;
        file.seek(std::io::SeekFrom::Start(byte_off))
            .context("Failed to seek to the requested offset.")?;
    }
    println!("Reading {} ms starting {:.3} s into the file.", num_blocks, offset_s);

    let mut raw_bytes = vec![0u8; ms_samples * 2 * num_blocks];
    if file.read_exact(&mut raw_bytes).is_err() {
        anyhow::bail!("Insufficient file size for 20 ms analysis.");
    }

    let mut signal: Vec<Complex<f32>> = Vec::with_capacity(ms_samples * num_blocks);
    for chunk in raw_bytes.chunks_exact(2) {
        let i = chunk[0] as i8 as f32;
        let q = chunk[1] as i8 as f32;
        signal.push(Complex::new(i, q));
    }

    // ADC level check before anything else: a capture that is clipping or
    // sitting on the quantisation floor cannot be rescued downstream, and the
    // recommendation is cheap. rf_calib is fully unit-tested but was never
    // actually called by the binary until now.
    let clip = signal.iter().filter(|c| c.re.abs() >= 127.0 || c.im.abs() >= 127.0).count();
    let sigma = {
        let n = signal.len() as f64;
        let m = signal.iter().fold(0.0f64, |a, c| a + c.re as f64) / n;
        (signal.iter().fold(0.0f64, |a, c| a + (c.re as f64 - m).powi(2)) / n).sqrt()
    };
    rf_calib::remove_dc_offset(&mut signal);
    let (rec_lna, rec_vga) = rf_calib::calibrate_gain_levels(&signal, 40, 30);
    println!(
        "ADC level: sigma {:.1} counts, clipping {:.3}%  ->  suggested LNA {} dB, VGA {} dB",
        sigma, 100.0 * clip as f64 / signal.len() as f64, rec_lna, rec_vga
    );
    if sigma < 4.0 {
        println!("  (low: quantisation noise starts to matter below about sigma 4)");
    } else if clip * 1000 > signal.len() {
        println!("  (clipping above 0.1%: reduce gain)");
    }

    println!("Loaded {} ms of RF data ({} samples).", num_blocks, signal.len());
    println!("Running parallel code phase search across CPU cores (threshold {:.2})...", ACQ_THRESHOLD);

    // Wide Doppler search: -60 kHz to +60 kHz in steps of 500 Hz (handles TCXO offset)
    let dopplers: Vec<f64> = (-60000..=60000).step_by(500).map(|d| d as f64).collect();

    // Plan once, not once per PRN. FftPlanner::new() inside the Rayon closure
    // recomputed twiddle factors for N=20,000 on all 69 parallel tasks.
    let (shared_fwd, shared_inv) = {
        let mut planner = FftPlanner::new();
        (planner.plan_fft_forward(ms_samples), planner.plan_fft_inverse(ms_samples))
    };

    let start_search = Instant::now();

    // Parallel search GPS L1 (IF = +7.161 MHz)
    let gps_results: Vec<SatResult> = (1..=32)
        .into_par_iter()
        .map(|prn| {
            let ca_code = dsp_calib::generate_gps_ca_code(prn);
            let fft_forward = shared_fwd.clone();
            let fft_inverse = shared_inv.clone();

            acquire_prn(
                "GPS",
                prn,
                &ca_code,
                1_023_000.0,
                7_161_000.0,
                &signal,
                sample_rate,
                ms_samples,
                num_blocks,
                &dopplers,
                &fft_forward,
                &fft_inverse,
            )
        })
        .collect();

    // Parallel search BeiDou B1I (IF = -7.161 MHz)
    let bds_results: Vec<SatResult> = (1..=37)
        .into_par_iter()
        .map(|prn| {
            let b1_code = dsp_calib::generate_beidou_b1_code(prn);
            let fft_forward = shared_fwd.clone();
            let fft_inverse = shared_inv.clone();

            acquire_prn(
                "BeiDou",
                prn,
                &b1_code,
                2_046_000.0,
                -7_161_000.0,
                &signal,
                sample_rate,
                ms_samples,
                num_blocks,
                &dopplers,
                &fft_forward,
                &fft_inverse,
            )
        })
        .collect();

    let mut all_results = Vec::new();
    all_results.extend(gps_results);
    all_results.extend(bds_results);
    let acquired_sats: Vec<SatResult> = all_results
        .iter()
        .filter(|s| s.metric_pk_2nd > ACQ_THRESHOLD)
        .cloned()
        .collect();

    println!("Completed acquisition scan in {:.3} seconds.", start_search.elapsed().as_secs_f64());
    println!("\n==================================================================================================");
    println!("                       SATELLITE ACQUISITION REPORT                                             ");
    println!("==================================================================================================");
    println!("Acquisition Criterion: Peak-to-Second-Peak Ratio > {:.2} across 20 ms Non-Coherent Accumulation", ACQ_THRESHOLD);
    println!("Total Satellites Acquired: {}\n", acquired_sats.len());

    if acquired_sats.is_empty() {
        // Report what was measured, not an assumed range.
        let lo = all_results.iter().map(|s| s.metric_pk_2nd).fold(f32::INFINITY, f32::min);
        let hi = all_results.iter().map(|s| s.metric_pk_2nd).fold(f32::NEG_INFINITY, f32::max);
        let best = all_results
            .iter()
            .max_by(|a, b| a.metric_pk_2nd.total_cmp(&b.metric_pk_2nd))
            .unwrap();
        println!(
            "--> 0 ACQUIRED. Measured peak/2nd across {} PRNs: {:.2} - {:.2} (threshold {:.2}).",
            all_results.len(), lo, hi, ACQ_THRESHOLD
        );
        println!(
            "    Strongest: {} PRN {} at {:.2}, Doppler {:+.0} Hz.",
            best.constellation, best.prn, best.metric_pk_2nd, best.doppler_hz
        );
        println!("    A value near 1.0 is the noise floor; this run shows no signal above threshold.");
        println!("    Not diagnosed here: whether the cause is sky view, antenna, or RF path.");
    } else {
        println!("{:<14} {:<8} {:<15} {:<15} {:<15}", "Constellation", "PRN", "Peak/2nd Metric", "Doppler (Hz)", "Code Phase (chips)");
        println!("--------------------------------------------------------------------------------------------------");
        for sat in &acquired_sats {
            println!(
                "{:<14} PRN {:<4} {:<15.2} {:<15.0} {:<15.1}",
                sat.constellation, sat.prn, sat.metric_pk_2nd, sat.doppler_hz, sat.code_phase_chips
            );
        }
    }

    println!("==================================================================================================");
    Ok(acquired_sats)
}

fn acquire_prn(
    constellation: &'static str,
    prn: usize,
    code: &[f32],
    chiprate: f64,
    if_freq: f64,
    signal: &[Complex<f32>],
    sample_rate: f64,
    ms_samples: usize,
    num_blocks: usize,
    dopplers: &[f64],
    fft_forward: &Arc<dyn rustfft::Fft<f32>>,
    fft_inverse: &Arc<dyn rustfft::Fft<f32>>,
) -> SatResult {
    if code.is_empty() {
        // An empty replica cannot correlate: report the noise-floor value.
        return SatResult {
            constellation,
            prn,
            metric_pk_2nd: 1.0,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
        };
    }

    let code_len = code.len();
    let mut local_code: Vec<Complex<f32>> = Vec::with_capacity(ms_samples);
    for i in 0..ms_samples {
        let chip_idx = (i as f64 * chiprate / sample_rate) as usize % code_len;
        local_code.push(Complex::new(code[chip_idx], 0.0));
    }

    // RustFFT's `process` allocates a scratch buffer on EVERY call. The inner
    // loop below runs num_dopplers * num_blocks times per PRN (241 * 20 here),
    // so that is hundreds of thousands of allocations per satellite searched.
    // Allocate once and reuse.
    let mut scratch = vec![
        Complex::new(0.0f32, 0.0f32);
        fft_forward
            .get_inplace_scratch_len()
            .max(fft_inverse.get_inplace_scratch_len())
    ];

    let mut local_code_fft = local_code.clone();
    fft_forward.process_with_scratch(&mut local_code_fft, &mut scratch);
    for x in local_code_fft.iter_mut() {
        *x = x.conj();
    }

    let num_dopplers = dopplers.len();
    // One contiguous buffer rather than num_dopplers separate Vecs, which cuts
    // 16,629 heap allocations per scan (241 per PRN, 69 PRNs) down to 69.
    //
    // MEASURED: this produced NO detectable speedup. Paired, interleaved runs
    // against the previous build gave a median -2.4% at 20 blocks and +3.2% at
    // 5 blocks, faster in 7 of 10 pairs, sign test p = 0.34. The audit that
    // proposed it called this an allocation storm and a bottleneck; it is the
    // first of those and not the second. The runtime is dominated by the FFTs.
    // Kept because it is bit-identical in output and strictly less allocation
    // churn, NOT because it made anything faster.
    let mut acc = vec![0.0f32; num_dopplers * ms_samples];
    let mut mixed = vec![Complex::new(0.0f32, 0.0f32); ms_samples];

    for b in 0..num_blocks {
        let blk_start = b * ms_samples;
        if blk_start + ms_samples > signal.len() {
            break;
        }
        let blk = &signal[blk_start..blk_start + ms_samples];

        for (d_idx, &doppler) in dopplers.iter().enumerate() {
            let total_freq = if_freq + doppler;
            let init_phase = -2.0 * std::f64::consts::PI * total_freq * (blk_start as f64 / sample_rate);
            // Compounding `cur *= step` in f32 tapers the amplitude by about
            // 2e-4 across a 20,000-sample block. Harmless here because the phase
            // is re-seeded every block, but computing it directly costs nothing
            // and removes the drift entirely.
            let dphi = -2.0 * std::f64::consts::PI * total_freq / sample_rate;
            // Recursive rotation is far cheaper than a transcendental per sample,
            // but compounding in f32 tapers the amplitude. Re-seed periodically:
            // exact phase every RESEED samples, cheap multiply in between.
            const RESEED: usize = 512;
            let step = Complex::new((dphi as f32).cos(), (dphi as f32).sin());
            // Chunked instead of testing `idx % RESEED` inside the per-sample
            // loop. Same arithmetic, same re-seed points, and it removes a
            // branch from the hottest loop in the program -- which, measured,
            // also changed nothing. See the note on `acc` above.
            let mut base = 0usize;
            while base < ms_samples {
                let end = (base + RESEED).min(ms_samples);
                let ph = init_phase + dphi * (base as f64);
                let mut cur = Complex::new(ph.cos() as f32, ph.sin() as f32);
                for idx in base..end {
                    mixed[idx] = blk[idx] * cur;
                    cur = cur * step;
                }
                base = end;
            }

            fft_forward.process_with_scratch(&mut mixed, &mut scratch);
            for (m, l) in mixed.iter_mut().zip(local_code_fft.iter()) {
                *m = *m * l;
            }
            fft_inverse.process_with_scratch(&mut mixed, &mut scratch);

            let row = &mut acc[d_idx * ms_samples..(d_idx + 1) * ms_samples];
            for idx in 0..ms_samples {
                row[idx] += mixed[idx].norm_sqr();
            }
        }
    }


    // Find peak across 2D map (doppler, code_phase)
    let mut max_p = 0.0f32;
    let mut max_d = 0;
    let mut max_c = 0;

    for d in 0..num_dopplers {
        let row = &acc[d * ms_samples..(d + 1) * ms_samples];
        for (c, &v) in row.iter().enumerate() {
            if v > max_p {
                max_p = v;
                max_d = d;
                max_c = c;
            }
        }
    }

    // Exclusion window of +/- 1 chip around the peak
    let excl = (sample_rate / chiprate).ceil() as usize;
    let row = &acc[max_d * ms_samples..(max_d + 1) * ms_samples];
    let mut second_peak = 0.0f32;

    for (c, &val) in row.iter().enumerate() {
        let dist = if c >= max_c { c - max_c } else { max_c - c };
        let wrap_dist = ms_samples - dist;
        let min_dist = dist.min(wrap_dist);

        if min_dist > excl && val > second_peak {
            second_peak = val;
        }
    }

    let pk_2nd = if second_peak > 0.0 {
        max_p / second_peak
    } else {
        1.0
    };
    let acquired_doppler = dopplers[max_d];
    let code_phase_chips = (max_c as f64 * chiprate) / sample_rate;

    // Always return the measurement. The acquisition decision is made by the
    // caller, so the metric survives for PRNs that do not pass and the report
    // can state what was actually observed instead of asserting a range.
    SatResult {
        constellation,
        prn,
        metric_pk_2nd: pk_2nd,
        doppler_hz: acquired_doppler,
        code_phase_chips,
    }
}

fn run_wideband_spectrum_sweep(device: &mut HackRf) -> Result<()> {
    println!("\n==================================================================================================");
    println!("              FULL L-BAND SPECTRUM SWEEP (900 MHz - 1800 MHz)                                     ");
    println!("==================================================================================================");
    println!("Sweeping spectrum using active HackRF receiver...\n");

    let start_freq_hz = 900_000_000u64;
    let end_freq_hz = 1_800_000_000u64;
    let step_hz = 15_000_000u64;
    let sample_rate = 20_000_000u32;
    let fft_size = 1024;

    device.set_sample_rate(sample_rate)?;
    device.start_rx()?;

    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(fft_size);
    let mut raw_buf = vec![0u8; 262144];

    let mut spectrum_bins: Vec<(f64, f32)> = Vec::new();

    let mut current_freq = start_freq_hz;
    while current_freq <= end_freq_hz {
        device.set_freq(current_freq)?;

        let _ = device.read_sync(&mut raw_buf)?;

        // Average several windowed snapshots. A single un-windowed FFT let the
        // DC residual and any strong carrier leak across the whole span, and one
        // snapshot of 1024 samples is 0.4% of a single USB read.
        const AVG: usize = 16;
        let mut psd = vec![0.0f32; fft_size];
        let mut taken = 0usize;
        for _ in 0..AVG {
            let n = device.read_sync(&mut raw_buf)?;
            if n < fft_size * 2 {
                continue;
            }
            let mut complex_buf: Vec<Complex<f32>> = raw_buf[..fft_size * 2]
                .chunks_exact(2)
                .map(|c| Complex::new(c[0] as i8 as f32, c[1] as i8 as f32))
                .collect();
            rf_calib::remove_dc_offset(&mut complex_buf);
            // Hann window: without it a strong carrier smears into every bin
            for (i, v) in complex_buf.iter_mut().enumerate() {
                let w = 0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64
                                     / fft_size as f64).cos();
                *v = *v * (w as f32);
            }
            fft.process(&mut complex_buf);
            for (acc, c) in psd.iter_mut().zip(complex_buf.iter()) {
                *acc += c.norm_sqr();
            }
            taken += 1;
        }
        let n = if taken > 0 { fft_size * 2 } else { 0 };

        if n >= fft_size * 2 {
            let complex_buf: Vec<Complex<f32>> = psd
                .iter()
                .map(|&p| Complex::new((p / taken as f32).sqrt(), 0.0))
                .collect();

            let bin_bw = sample_rate as f64 / fft_size as f64;
            let start_bin = (fft_size as f64 * (2.5 / 20.0)) as usize;
            let end_bin = (fft_size as f64 * (17.5 / 20.0)) as usize;

            for b in start_bin..end_bin {
                let bin_idx = (b + fft_size / 2) % fft_size;
                let power = complex_buf[bin_idx].norm_sqr();
                let power_db = 10.0 * (power + 1e-6).log10();

                let rel_freq_hz = (b as f64 - fft_size as f64 / 2.0) * bin_bw;
                let abs_freq_mhz = (current_freq as f64 + rel_freq_hz) / 1_000_000.0;

                // Filter out HackRF 40 MHz internal clock harmonic at 1560.00 MHz
                if (abs_freq_mhz - 1560.0).abs() > 0.5 {
                    spectrum_bins.push((abs_freq_mhz, power_db));
                }
            }
        }

        current_freq += step_hz;
    }

    device.stop_rx()?;

    // total_cmp: a NaN bin from an empty FFT would panic partial_cmp().unwrap()
    spectrum_bins.sort_by(|a, b| a.0.total_cmp(&b.0));

    println!("--------------------------------------------------------------------------------------------------");
    println!("Levels are UNCALIBRATED ADC units, not dBm. 'excess' is relative to the median bucket.");
    println!("The allocation column says what is LICENSED at that frequency; it is not a detection.");
    println!("Only the STATUS column reflects a measurement.");
    println!("--------------------------------------------------------------------------------------------------");
    println!("Freq (MHz)  level    excess   spectrum                              status   allocation at this frequency");
    println!("--------------------------------------------------------------------------------------------------");

    let min_db = spectrum_bins.iter().map(|b| b.1).fold(f32::INFINITY, f32::min);
    let max_db = spectrum_bins.iter().map(|b| b.1).fold(f32::NEG_INFINITY, f32::max);

    let mut bucket_map: HashMap<u32, (f32, usize)> = HashMap::new();
    for (freq_mhz, power_db) in &spectrum_bins {
        let bucket = (freq_mhz / 10.0).round() as u32 * 10;
        let entry = bucket_map.entry(bucket).or_insert((0.0, 0));
        entry.0 += power_db;
        entry.1 += 1;
    }

    let mut sorted_buckets: Vec<u32> = bucket_map.keys().cloned().collect();
    sorted_buckets.sort();

    // Median bucket level: the reference against which "excess" is judged.
    let mut levels: Vec<f32> = sorted_buckets
        .iter()
        .map(|b| { let (s, c) = bucket_map[b]; s / c as f32 })
        .collect();
    levels.sort_by(|a, b| a.total_cmp(b));
    let median_db = levels[levels.len() / 2];

    for b in sorted_buckets {
        let (sum_db, count) = bucket_map[&b];
        let avg_db = sum_db / count as f32;
        let excess = avg_db - median_db;
        let norm = (((avg_db - min_db) / (max_db - min_db + 1e-5)) * 35.0) as usize;
        let bar = "█".repeat(norm);

        // The receiver generates strong artefacts of its own: a harmonic at
        // exactly 1560 MHz (39 x the 40 MHz reference) and combs on the 20 MHz
        // tuning grid. Reporting those as signals is how a sweep lies.
        let is_spur = b % 20 == 0 || b == 1560;
        let status = if is_spur {
            "SPUR   "
        } else if excess > 10.0 {
            "SIGNAL "
        } else if excess > 6.0 {
            "weak   "
        } else {
            "-      "
        };

        let label = match b {
            960..=1080 => "DME / TACAN nav beacons",
            1090 => "ADS-B aircraft transponders (1090 MHz)",
            1170..=1180 => "GPS L5 / Galileo E5a (1176.45 MHz)",
            1200..=1210 => "Galileo E5b / BeiDou B2 (1207.14 MHz)",
            1220..=1230 => "GPS L2 / GLONASS G2 (1227.60 MHz)",
            1270..=1280 => "Galileo E6 / BeiDou B3 (1278.75 MHz)",
            1520..=1550 => "Inmarsat L-band aero & maritime satcom",
            1560..=1580 => "GPS L1 / Galileo E1 / BeiDou B1 (1575.42 MHz)",
            1600..=1610 => "GLONASS G1 (1602.00 MHz)",
            1616..=1626 => "Iridium constellation (1616-1626.5 MHz)",
            1690..=1710 => "NOAA / METEOSAT weather satellites",
            _ => "",
        };

        println!("{:<4} MHz  {:<7.1} {:>+6.1}   {:<36} {}  {}",
                 b, avg_db, excess, bar, status, label);
    }

    println!("==================================================================================================");
    Ok(())
}

fn run_live_decoder(
    device: &mut HackRf,
    freq_hz: u64,
    duration_secs: u64,
    lna: u32,
    vga: u32,
    running: Arc<AtomicBool>,
) -> Result<()> {
    // The Iridium simplex band uses a different, validated chain (find_bursts2 +
    // demod3) that needs seconds of capture at a time, not per-buffer PPM.
    if (1_615_000_000..=1_627_000_000).contains(&freq_hz) {
        return run_iridium_live(device, freq_hz, duration_secs, lna, vga, running);
    }
    println!("\n==================================================================================================");
    println!("             LIVE HARDWARE SIGNAL STREAMING DECODER (HACKRF ONE)                                  ");
    println!("==================================================================================================");
    println!("Target Frequency: {} MHz", freq_hz as f64 / 1_000_000.0);
    println!("Gain Settings: LNA = {} dB, VGA = {} dB", lna, vga);
    println!("Streaming Mode: Continuous ring-buffer PPM demodulator with CRC-24 verification\n");

    let sample_rate = 8_000_000u32; // 8 Msps for Mode-S ADS-B PPM
    device.set_sample_rate(sample_rate)?;
    device.set_freq(freq_hz)?;
    device.set_lna_gain(lna)?;
    device.set_vga_gain(vga)?;

    let mut buf = vec![0u8; 262144];
    let mut overlap = Vec::<Complex<f32>>::new();
    // A frame lying inside the retained overlap is decoded again in the next
    // buffer, so the running total counted it twice. Remember what was already
    // reported and skip repeats.
    let mut seen: std::collections::HashSet<String> = std::collections::HashSet::new();
    let overlap_samples = (sample_rate as usize / 1_000_000) * 240; // 240 us overlap window

    device.start_rx()?;
    let start = Instant::now();
    let mut total_frames = 0;

    println!("{:<6} {:<10} {:<12} {:<20} {}", "DF", "ICAO Hex", "Type Code", "Callsign / Altitude", "Status");
    println!("--------------------------------------------------------------------------------------------------");

    while running.load(Ordering::SeqCst) {
        if duration_secs > 0 && start.elapsed().as_secs() >= duration_secs {
            break;
        }

        if let Ok(n) = device.read_sync(&mut buf) {
            let slice = &buf[..n];
            let mut current_iq = Vec::with_capacity(overlap.len() + slice.len() / 2);
            current_iq.extend_from_slice(&overlap);

            for chunk in slice.chunks_exact(2) {
                current_iq.push(Complex::new(chunk[0] as i8 as f32, chunk[1] as i8 as f32));
            }

            rf_calib::remove_dc_offset(&mut current_iq);

            if freq_hz == 1_090_000_000 {
                let frames = decoder::decode_adsb_ppm(&current_iq, sample_rate as f64);
                for f in &frames {
                    // same 14-byte payload from the same aircraft == same frame
                    let key = format!("{}{}", f.icao, f.payload_hex);
                    if !seen.insert(key) {
                        continue;
                    }
                    if seen.len() > 4096 {
                        seen.clear();
                    }
                    total_frames += 1;
                    let extra = if let Some(ref cs) = f.callsign {
                        format!("Callsign: {}", cs)
                    } else if let Some(alt) = f.altitude_ft {
                        format!("Alt: {} ft", alt)
                    } else {
                        f.payload_hex.clone()
                    };

                    println!("{:<6} {:<10} {:<12} {:<20} {}", f.df, f.icao, f.msg_type, extra, f.detail);
                }
            }

            // Keep tail for overlap to not drop frames spanning buffer cuts
            let keep_len = overlap_samples.min(current_iq.len());
            overlap = current_iq[current_iq.len() - keep_len..].to_vec();
        }
    }

    device.stop_rx()?;
    println!("\n==================================================================================================");
    println!("Streaming stopped. Total verified CRC frames received: {}", total_frames);
    println!("==================================================================================================");
    Ok(())
}

/// Live Iridium decoder: accumulate a few seconds at 4 Msps, run the validated
/// find_bursts2 + demod3 chain, parse each frame, and print IRA/IBC content.
fn run_iridium_live(
    device: &mut HackRf,
    freq_hz: u64,
    duration_secs: u64,
    lna: u32,
    vga: u32,
    running: Arc<AtomicBool>,
) -> Result<()> {
    use hackrf_gnss::iridium::demod3;
    use hackrf_gnss::iridium::message::{classify_effort, Effort};
    use hackrf_gnss::iridium::{parse_line, Class};

    let fs = 4_000_000u32;
    let fc = freq_hz as f64;
    let accum_secs = 4.0f64;
    let need_i8 = (accum_secs * fs as f64) as usize * 2;

    println!("\n==================================================================================================");
    println!("             LIVE IRIDIUM DECODER (find_bursts2 + demod3, HACKRF ONE)                             ");
    println!("==================================================================================================");
    println!("Target: {:.4} MHz, {} Msps, {:.0} s accumulation windows", fc / 1e6, fs / 1_000_000, accum_secs);
    println!("Gain: LNA {} dB, VGA {} dB\n", lna, vga);

    device.set_sample_rate(fs)?;
    device.set_freq(freq_hz)?;
    device.set_lna_gain(lna)?;
    device.set_vga_gain(vga)?;

    let mut buf = vec![0u8; 262_144];
    let mut acc: Vec<i8> = Vec::with_capacity(need_i8);
    let mut total_frames = 0usize;
    let mut classes: HashMap<&'static str, usize> = HashMap::new();

    device.start_rx()?;
    let start = Instant::now();
    while running.load(Ordering::SeqCst) {
        if duration_secs > 0 && start.elapsed().as_secs() >= duration_secs {
            break;
        }
        if let Ok(n) = device.read_sync(&mut buf) {
            acc.extend(buf[..n].iter().map(|&b| b as i8));
        }
        if acc.len() < need_i8 {
            continue;
        }
        let dur = acc.len() as f64 / 2.0 / fs as f64;
        let lines = demod3::run_capture(&acc, fc, fs as f64, dur, fc - 0.25e6, fc + 0.25e6, 4.0);
        let mut ira = 0usize;
        for line in &lines {
            let Some(fr) = parse_line(line) else { continue };
            let c = classify_effort(&fr.bits, None, Effort::Harder);
            *classes.entry(c.label()).or_insert(0) += 1;
            total_frames += 1;
            if let Class::RingAlert(ref r) = c {
                ira += 1;
                println!(
                    "IRA sat:{:03} beam:{:02} pos=({:+.2}/{:+.2}) alt={:.0} km  [{} corrected bits]",
                    r.sat, r.beam, r.lat_deg, r.lon_deg, r.alt_km - 6355.0, r.corrected
                );
            }
        }
        println!(
            "-- window {:.1}s: {} bursts-decoded frames, {} Ring Alerts  (total {})",
            dur, lines.len(), ira, total_frames
        );
        acc.clear();
    }
    device.stop_rx()?;

    let mut summary: Vec<_> = classes.into_iter().collect();
    summary.sort_by_key(|&(_, n)| std::cmp::Reverse(n));
    println!("\n==================================================================================================");
    println!("Iridium decoding stopped. Total frames: {}", total_frames);
    for (k, n) in summary {
        println!("  {:<10} {}", k, n);
    }
    println!("==================================================================================================");
    Ok(())
}

/// Locate the recorder's observation log. The recorder owns the radio and
/// writes here; the server only reads, so the two never fight over the device.
fn observations_path() -> Option<std::path::PathBuf> {
    for p in ["../observations/observations.jsonl", "observations/observations.jsonl",
              "../../observations/observations.jsonl"] {
        let pb = std::path::PathBuf::from(p);
        if pb.exists() { return Some(pb); }
    }
    None
}

/// Default staleness budget for a per-producer state file, seconds.
/// A producer can tighten it by writing "ttl_s" into its own file.
const STATE_TTL_S: f64 = 1200.0; // 20 min

/// Build the /api/sync payload by merging the per-producer state files.
///
/// The panel state used to be ONE shared observations/sync_state.json that
/// four producers read-modify-wrote; a lost-update race was observed. Now
/// each producer writes ONLY its own observations/state.<name>.json and the
/// merge happens here, at read time. band_producer.py still writes the
/// legacy sync_state.json, so that file stays in the merge as the base.
///
/// Precedence order (later wins): 1) legacy sync_state.json, then
/// 2) state.*.json in filename order (state.phase.json, state.series.json,
/// state.tick.json) — so a per-producer file overrides the legacy file for
/// every key it carries. Per-key rules:
///   - "sources": merged by "band" — a row from a later file replaces the
///     same band's row in place; rows present in only one file pass through.
///   - "clock": shallow merge, later file wins per key.
///   - "alerts": union, first-seen order, duplicates dropped.
///   - "epoch": max across all contributing files.
///   - everything else: whole top-level key, later-file-wins.
/// Staleness + ownership: a state.*.json whose mtime is older than its
/// "ttl_s" (default STATE_TTL_S) stops VOTING (its values are dropped), but
/// it still VETOES the legacy file's relic copies of everything it owns:
/// the legacy sync_state.json is a read-modify-write survivor of the old
/// shared-file regime, so it carries frozen copies of band_series, phase,
/// the "PC clock"/"ATSC ch35" rows, tick clock keys, etc. Without the veto,
/// a dead producer's data would fall back to those relics instead of
/// disappearing. Concretely: keys / source bands / clock sub-keys carried
/// by ANY state.*.json (fresh or stale) are stripped from the legacy base
/// before merging; then only fresh files contribute values. The legacy file
/// has no guard of its own — band_producer owns it and the panel dims old
/// rows itself. "ttl_s" is a control field and never reaches the panel.
fn merge_sync_state(dir: &std::path::Path) -> Option<String> {
    use serde_json::Value;

    let legacy_path = dir.join("sync_state.json");
    let legacy: Option<Value> = std::fs::read_to_string(&legacy_path)
        .ok()
        .and_then(|s| serde_json::from_str::<Value>(&s).ok())
        .filter(|v| {
            // The legacy writer expires like every other producer (the
            // tombstone class): a dead band_producer's values must
            // DISAPPEAR, not outrank live ones. Same rule as state.*.json:
            // mtime age vs the file's own ttl_s (default STATE_TTL_S).
            let ttl = v.get("ttl_s").and_then(|t| t.as_f64()).unwrap_or(STATE_TTL_S);
            std::fs::metadata(&legacy_path)
                .and_then(|m| m.modified()).ok()
                .and_then(|t| t.elapsed().ok())
                .map_or(false, |a| a.as_secs_f64() <= ttl)
        });
    let mut per: Vec<std::path::PathBuf> = std::fs::read_dir(dir).ok()?
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.file_name().and_then(|n| n.to_str())
            .map(|n| n.starts_with("state.") && n.ends_with(".json"))
            .unwrap_or(false))
        .collect();
    per.sort();

    let mut fresh: Vec<Value> = Vec::new();         // fresh, voting
    // Ownership claimed by STALE files only: their values are dropped, but
    // the legacy file's frozen relics of the same keys/rows must drop too
    // (the relic would otherwise outlive its dead owner). Fresh owners are
    // not listed here — they simply override in the merge below, in place.
    let mut dead_keys: Vec<String> = Vec::new();
    let mut dead_bands: Vec<String> = Vec::new();
    let mut dead_clock: Vec<String> = Vec::new();
    for p in per {
        let age = std::fs::metadata(&p).and_then(|m| m.modified()).ok()
            .and_then(|t| t.elapsed().ok())
            .map(|d| d.as_secs_f64());
        let Ok(s) = std::fs::read_to_string(&p) else { continue };
        let Ok(mut v) = serde_json::from_str::<Value>(&s) else { continue };
        let ttl = v.get("ttl_s").and_then(|t| t.as_f64()).unwrap_or(STATE_TTL_S);
        let stale = age.map_or(false, |a| a > ttl);
        if let Some(obj) = v.as_object_mut() {
            obj.remove("ttl_s");
        }
        if stale {
            if let Some(obj) = v.as_object() {
                dead_keys.extend(obj.keys().cloned());
                if let Some(rows) = obj.get("sources").and_then(|s| s.as_array()) {
                    for row in rows {
                        if let Some(b) = row.get("band").or_else(|| row.get("name"))
                            .and_then(|b| b.as_str()) {
                            dead_bands.push(b.to_string());
                        }
                    }
                }
                if let Some(c) = obj.get("clock").and_then(|c| c.as_object()) {
                    dead_clock.extend(c.keys().cloned());
                }
            }
            continue; // dead producer: vetoes its relics but casts no vote
        }
        fresh.push(v);
    }

    let mut docs: Vec<Value> = Vec::new();
    if let Some(l) = legacy {
        docs.push(l);
    }
    docs.extend(fresh.iter().cloned());

    if docs.is_empty() {
        return None;
    }

    let mut out = serde_json::Map::new();
    let mut epoch: Option<f64> = None;
    let mut sources: Vec<Value> = Vec::new();
    let mut alerts: Vec<Value> = Vec::new();
    let mut clock = serde_json::Map::new();
    let (mut sources_seen, mut alerts_seen, mut clock_seen) = (false, false, false);

    for d in &docs {
        let Some(obj) = d.as_object() else { continue };
        if let Some(e) = obj.get("epoch").and_then(|e| e.as_f64()) {
            epoch = Some(epoch.map_or(e, |m| m.max(e)));
        }
        if let Some(rows) = obj.get("sources").and_then(|s| s.as_array()) {
            sources_seen = true;
            for row in rows {
                let key = row.get("band").or_else(|| row.get("name"))
                    .and_then(|b| b.as_str());
                match key.and_then(|k| sources.iter().position(|s| {
                    s.get("band").or_else(|| s.get("name")).and_then(|b| b.as_str()) == Some(k)
                })) {
                    Some(i) => sources[i] = row.clone(),
                    None => sources.push(row.clone()),
                }
            }
        }
        if let Some(list) = obj.get("alerts").and_then(|a| a.as_array()) {
            alerts_seen = true;
            for a in list {
                if !alerts.contains(a) {
                    alerts.push(a.clone());
                }
            }
        }
        if let Some(c) = obj.get("clock").and_then(|c| c.as_object()) {
            clock_seen = true;
            for (k, v) in c {
                clock.insert(k.clone(), v.clone());
            }
        }
        for (k, v) in obj {
            if matches!(k.as_str(), "epoch" | "sources" | "alerts" | "clock") {
                continue;
            }
            out.insert(k.clone(), v.clone());
        }
    }

    // Stale-owner veto: drop the legacy file's frozen relics of everything a
    // dead producer owns, so a killed producer's data DISAPPEARS instead of
    // falling back to the relic. Anything a fresh file still votes for was
    // replaced in place during the merge above and is guarded here.
    let fresh_keys: Vec<&String> = fresh.iter().filter_map(|d| d.as_object())
        .flat_map(|o| o.keys()).collect();
    let fresh_bands: Vec<&str> = fresh.iter()
        .filter_map(|d| d.get("sources")).filter_map(|s| s.as_array())
        .flatten()
        .filter_map(|r| r.get("band").or_else(|| r.get("name")))
        .filter_map(|b| b.as_str()).collect();
    let fresh_clock: Vec<&String> = fresh.iter()
        .filter_map(|d| d.get("clock")).filter_map(|c| c.as_object())
        .flat_map(|c| c.keys()).collect();
    for k in &dead_keys {
        if !matches!(k.as_str(), "epoch" | "sources" | "alerts" | "clock")
            && !fresh_keys.iter().any(|f| f == &k) {
            out.remove(k);
        }
    }
    if !dead_bands.is_empty() {
        sources.retain(|row| {
            let b = row.get("band").or_else(|| row.get("name")).and_then(|b| b.as_str());
            match b {
                Some(b) => !dead_bands.iter().any(|d| d == b)
                    || fresh_bands.iter().any(|f| *f == b),
                None => true,
            }
        });
    }
    for k in &dead_clock {
        if !fresh_clock.iter().any(|f| f == &k) {
            clock.remove(k);
        }
    }

    // Preserve the legacy shape: a key the writers always emit (sources,
    // alerts, clock) stays present even when its merged value is empty —
    // the panel renders "no alerts" from [], not from a missing key.
    if sources_seen {
        out.insert("sources".to_string(), Value::Array(sources));
    }
    if alerts_seen {
        out.insert("alerts".to_string(), Value::Array(alerts));
    }
    if clock_seen {
        out.insert("clock".to_string(), Value::Object(clock));
    }
    if let Some(e) = epoch {
        out.insert("epoch".to_string(), serde_json::json!(e));
    }
    Some(Value::Object(out).to_string())
}

/// Build the live dashboard payload from whatever the recorder has written so far.
/// Running totals over the observation log.
///
/// The log is APPEND-ONLY, one JSON record per recorder cycle, and everything
/// the dashboard shows is either a sum, a count, a last-seen value, or a short
/// ring of the newest bursts. All of those fold record by record, so the status
/// can be maintained incrementally instead of rebuilt from scratch.
///
/// It used to be rebuilt from scratch: read_to_string on the whole file and a
/// serde parse of every line, on EVERY request. The dashboard polls every 5 s
/// and the log had reached 20 MB, so that was ~4 MB/s of re-reading and
/// re-parsing to serve data that changes once a minute, and it got worse every
/// hour the station ran. Now only the bytes appended since the last poll are
/// parsed.
#[derive(Default, Clone)]
struct StatusAcc {
    cycles: usize,
    bursts: u64,
    locked: u64,
    sync: u64,
    frames: u64,
    chan: std::collections::BTreeMap<String, u64>,
    recent: std::collections::VecDeque<serde_json::Value>,
    gnss: serde_json::Value,
    sweep: serde_json::Value,
    first_utc: serde_json::Value,
    last: serde_json::Value,
}

const RECENT_CAP: usize = 60;

impl StatusAcc {
    /// Fold one record in. Must produce the same result whether records arrive
    /// one at a time or all at once -- that equivalence is what makes the
    /// incremental path safe, and it is asserted in the tests.
    fn absorb(&mut self, r: &serde_json::Value) {
        self.cycles += 1;
        if self.first_utc.is_null() {
            self.first_utc = r["utc"].clone();
        }
        self.last = r.clone();

        let ir = &r["iridium"];
        self.bursts += ir["n_bursts"].as_u64().unwrap_or(0);
        self.locked += ir["n_locked"].as_u64().unwrap_or(0);
        // n_frames comes from the current demodulator; n_locked is the older
        // lock metric it bypasses, so on its own it understates the yield.
        self.frames += ir["n_frames"].as_i64().unwrap_or(0).max(0) as u64;
        self.sync += ir["n_sync"].as_u64().unwrap_or(0);

        if let Some(list) = ir["bursts"].as_array() {
            for b in list {
                if let Some(f) = b["freq_mhz"].as_f64() {
                    *self.chan.entry(format!("{:.2}", f)).or_insert(0) += 1;
                }
            }
            // newest record first, bursts within a record in their original
            // order: pushing the group onto the front in reverse achieves both
            for b in list.iter().rev() {
                let mut o = b.clone();
                o["utc"] = r["utc"].clone();
                o["cycle"] = r["cycle"].clone();
                self.recent.push_front(o);
            }
            while self.recent.len() > RECENT_CAP {
                self.recent.pop_back();
            }
        }

        if !r["gnss"].is_null() {
            self.gnss = r["gnss"].clone();
        }
        if !r["sweep"].is_null() {
            self.sweep = r["sweep"].clone();
        }
    }
}

struct StatusCache {
    offset: u64,
    acc: StatusAcc,
}

fn status_cache() -> &'static std::sync::Mutex<HashMap<std::path::PathBuf, StatusCache>> {
    static C: std::sync::OnceLock<std::sync::Mutex<HashMap<std::path::PathBuf, StatusCache>>> =
        std::sync::OnceLock::new();
    C.get_or_init(|| std::sync::Mutex::new(HashMap::new()))
}

/// Read whatever has been appended since last time and fold it in.
///
/// Two things it must not get wrong. A partially written final line is left
/// unconsumed, so the next poll picks it up whole rather than dropping a cycle.
/// And if the file has SHRUNK -- truncated, rotated, or replaced when the
/// recorder restarts -- the accumulator is thrown away and rebuilt, because
/// continuing to add to it would double-count.
fn refresh_status(path: &std::path::Path) -> StatusAcc {
    use std::io::{Seek, SeekFrom};
    let mut guard = status_cache().lock().unwrap();
    let entry = guard.entry(path.to_path_buf()).or_insert(StatusCache {
        offset: 0,
        acc: StatusAcc::default(),
    });

    let len = std::fs::metadata(path).map(|m| m.len()).unwrap_or(0);
    if len < entry.offset {
        entry.offset = 0;
        entry.acc = StatusAcc::default();
    }
    if len > entry.offset {
        if let Ok(mut f) = File::open(path) {
            if f.seek(SeekFrom::Start(entry.offset)).is_ok() {
                let mut buf = String::new();
                if f.read_to_string(&mut buf).is_ok() {
                    let consumed = match buf.rfind('\n') {
                        Some(i) => i + 1,
                        None => 0, // no complete line yet; wait for the rest
                    };
                    for line in buf[..consumed].lines() {
                        if line.trim().is_empty() {
                            continue;
                        }
                        if let Ok(v) = serde_json::from_str::<serde_json::Value>(line) {
                            entry.acc.absorb(&v);
                        }
                    }
                    entry.offset += consumed as u64;
                }
            }
        }
    }
    entry.acc.clone()
}

/// Count snippets, but not on every poll: a readdir over thousands of entries
/// is the other cost that grew with the run.
fn snippet_count(dir: &std::path::Path) -> usize {
    static C: std::sync::OnceLock<std::sync::Mutex<Option<(std::time::SystemTime, usize)>>> =
        std::sync::OnceLock::new();
    let cell = C.get_or_init(|| std::sync::Mutex::new(None));
    let mtime = std::fs::metadata(dir).and_then(|m| m.modified()).ok();
    let mut g = cell.lock().unwrap();
    if let (Some(mt), Some((cached_mt, n))) = (mtime, g.as_ref()) {
        if *cached_mt == mt {
            return *n;
        }
    }
    let n = std::fs::read_dir(dir)
        .map(|d| d.filter_map(|e| e.ok()).count())
        .unwrap_or(0);
    if let Some(mt) = mtime {
        *g = Some((mt, n));
    }
    n
}

fn status_json(acc: &StatusAcc, base: &std::path::Path) -> serde_json::Value {
    // The per-check gnss record only carries that check's threshold crossings.
    // Whether a satellite was actually SEEN is an aggregate judgement across
    // checks (does its Doppler sweep like a real pass?), computed by report.py.
    let verdict = std::fs::read_to_string(base.join("report_data.json"))
        .ok()
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        .map(|v| v["gnss"].clone())
        .unwrap_or(serde_json::Value::Null);

    serde_json::json!({
        "recorder": "running",
        "receiver": "HackRF One r9",
        "band": "Iridium simplex 1626.0-1626.5 MHz, 4 Msps, amp+bias ON",
        "cycles": acc.cycles,
        "first_utc": acc.first_utc,
        "last_utc": acc.last["utc"].clone(),
        "totals": { "bursts": acc.bursts, "locked": acc.locked,
                    "frame_sync": acc.sync, "frames": acc.frames },
        "last_cycle": acc.last,
        "recent_bursts": acc.recent.iter().cloned().collect::<Vec<_>>(),
        "channels": acc.chan,
        "gnss": acc.gnss,
        "gnss_verdict": verdict,
        "sweep": acc.sweep,
        "snippets": snippet_count(&base.join("snippets"))
    })
}

fn build_status() -> serde_json::Value {
    let path = match observations_path() {
        Some(p) => p,
        None => return serde_json::json!({
            "recorder": "not running",
            "hint": "start it with:  python3 validation/recorder.py",
            "cycles": 0
        }),
    };
    let acc = refresh_status(&path);
    let base = path.parent().unwrap().to_path_buf();
    status_json(&acc, &base)
}

fn run_web_server(port: u16) -> Result<()> {
    let addr = format!("0.0.0.0:{}", port);
    let server = tiny_http::Server::http(&addr).map_err(|e| anyhow::anyhow!("Failed to bind server on {}: {}", addr, e))?;
    println!("\n==================================================================================================");
    println!("              HACKRF REAL-TIME TELEMETRY WEB SERVER ACTIVE                                        ");
    println!("==================================================================================================");
    println!("--> Dashboard running at http://localhost:{}", port);
    println!("--> Live API endpoint: http://localhost:{}/api/status", port);
    println!("Press Ctrl-C to terminate.\n");

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\nStopping web server...");
        r.store(false, Ordering::SeqCst);
    })?;

    for request in server.incoming_requests() {
        if !running.load(Ordering::SeqCst) {
            break;
        }

        let url = request.url().to_string();
        let path = url.split('?').next().unwrap_or("/");

        match path {
            "/" | "/index.html" => {
                let html = std::fs::read_to_string("web/index.html").unwrap_or_else(|_| "<h1>Index not found</h1>".to_string());
                let response = tiny_http::Response::from_string(html).with_header(
                    tiny_http::Header::from_bytes(&b"Content-Type"[..], &b"text/html; charset=utf-8"[..]).unwrap(),
                );
                let _ = request.respond(response);
            }
            "/style.css" => {
                let css = std::fs::read_to_string("web/style.css").unwrap_or_default();
                let response = tiny_http::Response::from_string(css).with_header(
                    tiny_http::Header::from_bytes(&b"Content-Type"[..], &b"text/css; charset=utf-8"[..]).unwrap(),
                );
                let _ = request.respond(response);
            }
            "/app.js" => {
                let js = std::fs::read_to_string("web/app.js").unwrap_or_default();
                let response = tiny_http::Response::from_string(js).with_header(
                    tiny_http::Header::from_bytes(&b"Content-Type"[..], &b"application/javascript; charset=utf-8"[..]).unwrap(),
                );
                let _ = request.respond(response);
            }
            "/catalogue" | "/catalogue.html" | "/globe" | "/globe.html" => {
                // Generated artefacts live beside the recorder's log.
                let file = if path.starts_with("/globe") { "globe.html" } else { "catalogue.html" };
                let body = observations_path()
                    .and_then(|p| std::fs::read_to_string(p.parent().unwrap().join(file)).ok())
                    .unwrap_or_else(|| format!("<h1>{} not built yet</h1>\
                        <p>Run validation/catalogue_build.py (and globe_build.py).</p>", file));
                let response = tiny_http::Response::from_string(body).with_header(
                    tiny_http::Header::from_bytes(&b"Content-Type"[..], &b"text/html; charset=utf-8"[..]).unwrap(),
                );
                let _ = request.respond(response);
            }
            "/report" | "/report.html" => {
                // The morning report, regenerated periodically by the refresher.
                let body = observations_path()
                    .and_then(|p| std::fs::read_to_string(p.parent().unwrap().join("report.html")).ok())
                    .unwrap_or_else(|| "<h1>No report yet</h1><p>Run validation/report.py then validation/build_report.py.</p>".to_string());
                let response = tiny_http::Response::from_string(body).with_header(
                    tiny_http::Header::from_bytes(&b"Content-Type"[..], &b"text/html; charset=utf-8"[..]).unwrap(),
                );
                let _ = request.respond(response);
            }
            "/sync" | "/sync.html" => {
                // The fused-sync panel; the writer side is the fusion engine.
                let body = observations_path()
                    .and_then(|p| std::fs::read_to_string(p.parent().unwrap().join("sync.html")).ok())
                    .unwrap_or_else(|| "<h1>sync panel not built yet</h1>\
                        <p>sync.html lives beside the recorder's generated artefacts.</p>".to_string());
                let response = tiny_http::Response::from_string(body).with_header(
                    tiny_http::Header::from_bytes(&b"Content-Type"[..], &b"text/html; charset=utf-8"[..]).unwrap(),
                );
                let _ = request.respond(response);
            }
            "/api/sync" => {
                // Merged view of the per-producer state files + the legacy
                // sync_state.json (band_producer still writes it). All files
                // missing is a normal pre-writer state, reported as 200 +
                // error JSON to keep the page simple.
                let body = observations_path()
                    .and_then(|p| merge_sync_state(p.parent().unwrap()))
                    .unwrap_or_else(|| "{\"error\":\"no sync state yet\"}".to_string());
                let response = tiny_http::Response::from_string(body).with_header(
                    tiny_http::Header::from_bytes(&b"Content-Type"[..], &b"application/json"[..]).unwrap(),
                );
                let _ = request.respond(response);
            }
            "/api/status" => {
                let json = build_status();
                let response = tiny_http::Response::from_string(json.to_string()).with_header(
                    tiny_http::Header::from_bytes(&b"Content-Type"[..], &b"application/json"[..]).unwrap(),
                );
                let _ = request.respond(response);
            }
            _ => {
                let response = tiny_http::Response::from_string("404 Not Found").with_status_code(404);
                let _ = request.respond(response);
            }
        }
    }

    Ok(())
}

/// Parse a frames.txt and print every ring alert we can decode.
///
/// Output mirrors the fields iridium-toolkit prints, so the two can be compared
/// line for line -- which is the only way to know this port is right. `alt` is
/// quoted the way the reference quotes it: radius minus a 6355 km sphere.
fn run_iridium_parse(path: &str, harder: bool) -> Result<()> {
    use hackrf_gnss::iridium::message::{classify_effort, Effort};
    use hackrf_gnss::iridium::{parse_line, Class};
    use std::io::BufRead;

    let f = File::open(path).with_context(|| format!("opening {}", path))?;
    let mut total = 0usize;
    let mut classes: HashMap<&'static str, usize> = HashMap::new();
    for line in std::io::BufReader::new(f).lines() {
        let line = line?;
        let Some(fr) = parse_line(&line) else { continue };
        total += 1;
        let c = classify_effort(&fr.bits, None,
                                if harder { Effort::Harder } else { Effort::Strict });
        let name = c.label();
        *classes.entry(name).or_insert(0) += 1;
        // Output in iridium-toolkit's `-o line` format so this is a drop-in
        // for the three places that shell out to it (ira_pipeline,
        // catalogue_build, frames_report). Those parse the line with a regex,
        // so the shape matters as much as the values.
        // The name token is not decoration: catalogue_build maps each frame
        // back to its capture time through it. iridium-toolkit emits
        // "p-<unix seconds>" when it understood the name and
        // "u-<name, dashes as dots>" when it did not, and callers key off both
        // forms. Synthesising "p-0" for an unrecognised name silently detaches
        // every frame from its timestamp -- the frames still decode, they just
        // stop belonging to any capture.
        let tag = match fr.start_epoch {
            Some(e) => format!("p-{}", e as i64),
            None => format!("u-{}", fr.name.replace('-', ".")),
        };
        if let Class::Broadcast(ref ibc) = c {
            println!(
                "IBC: {}-e{:03} {:014.4} {:.0} {:3}% -06.02|-100.00|00.00 {} DL \
                 bc:{} sat:{:03} cell:{:02} 0 slot:{} sv_blkn:{} aq_sb:{:02} aq_ch:{}",
                tag, ibc.corrected, fr.offset_ms, fr.freq_hz, fr.confidence, fr.symbols,
                ibc.bc_type, ibc.sv_id, ibc.beam_id, ibc.slot, ibc.sv_blocking,
                ibc.acqu_subband, ibc.acqu_channels
            );
        }
        if let Class::RingAlert(ref ira) = c {
            println!(
                "IRA: {}-e{:03} {:014.4} {:.0} {:3}% -06.02|-100.00|00.00 {} DL \
                 sat:{:03} beam:{:02} xyz=({:+05},{:+05},{:+05}) pos=({:+06.2}/{:+07.2}) \
                 alt={:.0} RAI:{:02} ?{:02} bc_sb:{:02}",
                tag, ira.corrected, fr.offset_ms, fr.freq_hz, fr.confidence, fr.symbols,
                ira.sat, ira.beam, ira.pos_x, ira.pos_y, ira.pos_z,
                ira.lat_deg, ira.lon_deg, ira.alt_km - 6355.0,
                ira.interval, ira.eip, ira.bc_sb
            );
        }
    }
    let mut summary: Vec<_> = classes.into_iter().collect();
    summary.sort_by_key(|&(_, n)| std::cmp::Reverse(n));
    eprintln!("{} frames read", total);
    for (k, n) in summary {
        eprintln!("  {:<10} {}", k, n);
    }
    Ok(())
}

/// Acquire GPS L1 from an IQ file and solve a coarse-time snapshot position fix
/// from broadcast ephemeris. This is the validated `gps::acquire ->
/// gps::snapshot_fix` chain (see tests/gps_e2e.rs) driven from a real capture.
fn run_fix(args: &Args) -> Result<()> {
    println!("GPS coarse-time snapshot fix");
    println!(
        "  file {}   fs {:.3} Msps   IF {:.3} MHz",
        args.file, args.fs_hz / 1e6, args.if_hz / 1e6
    );

    if args.ext16 {
        println!("Extended-precision: parsing little-endian int16 I/Q (12-bit data)");
    }

    let ms_samples = (args.fs_hz / 1000.0) as usize;
    let nblocks = args.blocks.max(10);
    let bytes_per_sample: u64 = if args.ext16 { 4 } else { 2 };
    let mut file = File::open(&args.file).context("open IQ file for Fix")?;
    if args.offset_secs > 0.0 {
        use std::io::Seek;
        let byte_off = (args.offset_secs * args.fs_hz).round() as u64 * bytes_per_sample;
        file.seek(std::io::SeekFrom::Start(byte_off)).context("seek")?;
    }
    let mut raw = vec![0u8; ms_samples * nblocks * bytes_per_sample as usize];
    file.read_exact(&mut raw)
        .context("capture too short for the requested window")?;
    let mut sig: Vec<Complex<f32>> = if args.ext16 {
        raw.chunks_exact(4)
            .map(|c| {
                // ext-precision lane: 12-bit sample right-justified; the top
                // nibble is a timestamp nibble, not sign extension — mask
                // and re-extend from bit 11.
                let i = i16::from_le_bytes([c[0], c[1]]);
                let q = i16::from_le_bytes([c[2], c[3]]);
                Complex::new(((i << 4) >> 4) as f32, ((q << 4) >> 4) as f32)
            })
            .collect()
    } else {
        raw.chunks_exact(2)
            .map(|c| Complex::new(c[0] as i8 as f32, c[1] as i8 as f32))
            .collect()
    };

    // remove the IF so L1 sits at baseband (SatCatch GPS captures are IF=0)
    if args.if_hz != 0.0 {
        let w = -2.0 * std::f64::consts::PI * args.if_hz / args.fs_hz;
        for (n, s) in sig.iter_mut().enumerate() {
            let (sn, cs) = (w * n as f64).sin_cos();
            *s = *s * Complex::new(cs as f32, sn as f32);
        }
    }
    println!("  loaded {} samples ({} ms)", sig.len(), nblocks);

    // wide Doppler window absorbs TCXO offset; the compensated acquirer is used
    let dopplers: Vec<f64> = (-10000..=10000).step_by(250).map(|d| d as f64).collect();
    let prns: Vec<usize> = (1..=32).collect();
    let mut acq: Vec<_> = gps::acquire(&sig, args.fs_hz, &prns, &dopplers, nblocks, ACQ_THRESHOLD)
        .into_iter()
        .filter(|r| r.metric >= ACQ_THRESHOLD)
        .collect();
    acq.sort_by(|a, b| b.metric.partial_cmp(&a.metric).unwrap());
    println!("  acquired {} satellites:", acq.len());
    for r in &acq {
        println!(
            "    PRN {:2}  metric {:.2}  Doppler {:+6.0} Hz  code phase {:7.1} chips",
            r.prn, r.metric, r.doppler, r.code_phase
        );
    }
    if acq.len() < 4 {
        println!("  need >=4 satellites for a fix (open sky helps). No fix.");
        return Ok(());
    }

    if args.nav.is_empty() {
        anyhow::bail!("Fix mode needs --nav <RINEX broadcast nav file> (e.g. a BRDC..._MN.rnx)");
    }
    let nav_txt = std::fs::read_to_string(&args.nav).context("read --nav RINEX file")?;
    let ephs = gps::parse_rinex_gps(&nav_txt);
    println!("  ephemeris: {} GPS satellites from {}", ephs.len(), args.nav);

    let tow = if args.tow >= 0.0 {
        args.tow
    } else {
        use std::time::{SystemTime, UNIX_EPOCH};
        let unix = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs_f64();
        (unix - 315_964_800.0 + 18.0).rem_euclid(604_800.0) // GPS epoch + 18 s leap
    };

    let obs: Vec<gps::Obs> = acq
        .iter()
        .map(|r| gps::Obs { prn: r.prn as u8, code_phase: r.code_phase, doppler: r.doppler })
        .collect();
    let approx = match (args.approx_lat, args.approx_lon) {
        (Some(la), Some(lo)) => [la, lo, 0.0],
        (None, None) => site::load_site(std::path::Path::new(
            "/Volumes/Radiator 8TB/gnss/observations/site.json",
        ))
        .map(|s| [s[0], s[1], 0.0])
        .unwrap_or_else(|| {
            eprintln!(
                "no approximate position: pass --approx-lat/--approx-lon or provide observations/site.json"
            );
            std::process::exit(2);
        }),
        _ => {
            eprintln!("--approx-lat and --approx-lon must be given together");
            std::process::exit(2);
        }
    };
    match gps::snapshot_fix(&obs, &ephs, approx, tow) {
        Some(fix) => {
            println!(
                "\n  FIX: {:.6} N, {:.6} E   alt {:.0} m",
                fix.lat, fix.lon, fix.alt_km * 1000.0
            );
            println!(
                "       {} sats · residual rms {:.1} m · GDOP {:.2}",
                fix.n_sat, fix.residual_rms_m, fix.gdop
            );
        }
        None => println!("  snapshot solve failed (geometry / ambiguity)."),
    }
    Ok(())
}

fn main() -> Result<()> {
    let args = Args::parse();

    if args.mode == Mode::Fix {
        return run_fix(&args);
    }

    if args.mode == Mode::Analyze {
        analyze_iq_file(&args.file, args.offset_secs, args.blocks)?;
        return Ok(());
    }

    if args.mode == Mode::Serve {
        return run_web_server(args.port);
    }

    if args.mode == Mode::Iridium {
        return run_iridium_parse(&args.file, args.harder);
    }

    println!("Starting GNSS HackRF Optimizer...");
    println!("Mode: {:?}", args.mode);
    println!(
        "Duration: {} seconds",
        if args.duration_secs == 0 {
            "Unlimited".to_string()
        } else {
            args.duration_secs.to_string()
        }
    );
    println!("Bias-Tee (Antenna Power): {}", if args.bias_tee { "ON" } else { "OFF" });

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\nReceived Ctrl-C, stopping capture...");
        r.store(false, Ordering::SeqCst);
    })?;

    let mut device = match &args.serial {
        Some(wanted) => {
            let serials = HackRf::list_devices()
                .context("Failed to enumerate HackRF devices")?;
            let index = serials
                .iter()
                .position(|s| s.trim_start_matches('0') == wanted.trim_start_matches('0'))
                .with_context(|| {
                    format!(
                        "No HackRF with serial {} found. Attached: {:?}",
                        wanted, serials
                    )
                })?;
            HackRf::open_by_index(index)
                .with_context(|| format!("Failed to open HackRF serial {}", wanted))?
        }
        None => HackRf::open_first().context("Failed to open HackRF device. Ensure device is plugged in and udev/permissions are set. If multiple HackRFs are attached, use --serial to select one.")?,
    };
    device.set_lna_gain(args.lna_gain)?;
    device.set_vga_gain(args.vga_gain)?;
    device.set_antenna_enable(args.bias_tee)?;
    device.set_amp_enable(args.amp)?;
    println!("RX Amp (+14 dB): {}", if args.amp { "ON" } else { "OFF" });

    if args.ext16 {
        // Live capture in extended precision: the gateware decimates 4x-32x
        // (the halfband tail was removed for the 8 Msps boost), so the MCU
        // sample rate tops out at 10 Msps; 8 Msps (32 MB/s int16) is the
        // validated USB2 envelope, so cap there. The capture itself just
        // moves raw bytes; int16 parsing happens at analysis time.
        if args.fs_hz > 8_000_000.0 {
            anyhow::bail!("--ext16 live capture requires --fs-hz <= 8 MHz (extended-precision gateware decimates 4x-32x; 8 Msps int16 = 32 MB/s is the validated USB limit)");
        }
        println!("Extended-precision: expecting int16 I/Q from the device (load gateware with `hackrf_debug -P 2` first)");
    }

    if args.mode == Mode::Sweep {
        return run_wideband_spectrum_sweep(&mut device);
    }

    if args.mode == Mode::Decode {
        return run_live_decoder(&mut device, args.freq_hz, args.duration_secs, args.lna_gain, args.vga_gain, running);
    }

    let mut buf = vec![0u8; 262144];
    let start_time = Instant::now();

    let is_done = |start: Instant, duration: u64, run_flag: &AtomicBool| -> bool {
        if !run_flag.load(Ordering::SeqCst) {
            return true;
        }
        if duration > 0 && start.elapsed().as_secs() >= duration {
            return true;
        }
        false
    };

    match args.mode {
        Mode::WidebandL1 | Mode::WidebandL2 => {
            device.set_sample_rate(20_000_000)?;
            // set_sample_rate auto-programs the baseband filter to 75% of the rate,
            // i.e. 15 MHz (+/-7.5 MHz). This capture deliberately places GPS L1 at
            // +7.161 MHz and BeiDou B1I at -7.161 MHz, which lands both carriers
            // 340 kHz inside that corner and rolls off the outer half of each
            // +/-1.023 MHz main lobe. Widen the filter so the signals the capture
            // exists to find are actually in the passband.
            device.set_baseband_filter_bandwidth(20_000_000)?;

            let (freq_hz, filename) = if args.mode == Mode::WidebandL1 {
                (1_568_259_000, "wideband_l1_b1.iq")
            } else {
                (1_236_800_000, "wideband_l2_g2.iq")
            };

            println!("Setting frequency to {} Hz and Sample Rate to 20 Msps", freq_hz);
            println!("Simultaneous capture: GPS L1, Galileo E1, BeiDou B1");
            device.set_freq(freq_hz)?;

            let mut file = File::create(filename)?;
            println!("Saving continuous I/Q stream to {} ...", filename);

            device.start_rx()?;
            let mut total_bytes = 0;
            while !is_done(start_time, args.duration_secs, &running) {
                let n = device.read_sync(&mut buf)?;
                file.write_all(&buf[..n])?;
                total_bytes += n;
            }
            device.stop_rx()?;
            println!(
                "Captured {} bytes ({:.2} MB) in {:.2} seconds.",
                total_bytes,
                total_bytes as f64 / 1_048_576.0,
                start_time.elapsed().as_secs_f64()
            );
        }
        Mode::Chop => {
            if args.bands.is_empty() {
                anyhow::bail!("Must specify at least one band when using Chop mode.");
            }
            device.set_sample_rate(10_000_000)?;

            let mut files = HashMap::new();
            for band in &args.bands {
                let filename = format!("{:?}_chopped.iq", band).to_lowercase();
                files.insert(*band, File::create(&filename)?);
            }

            let bytes_per_ms = (10_000_000 * 2) / 1000;
            let settle_bytes = (args.settle_ms * (bytes_per_ms as u64)) as usize;
            let dwell_bytes = (args.dwell_ms * (bytes_per_ms as u64)) as usize;

            device.start_rx()?;
            while !is_done(start_time, args.duration_secs, &running) {
                for band in &args.bands {
                    if is_done(start_time, args.duration_secs, &running) {
                        break;
                    }
                    device.set_freq(band.frequency_hz())?;

                    let mut discarded = 0;
                    while discarded < settle_bytes && !is_done(start_time, args.duration_secs, &running) {
                        discarded += device.read_sync(&mut buf)?;
                    }

                    let mut captured = 0;
                    let file = files.get_mut(band).unwrap();
                    while captured < dwell_bytes && !is_done(start_time, args.duration_secs, &running) {
                        let n = device.read_sync(&mut buf)?;
                        let to_write = n.min(dwell_bytes - captured);
                        file.write_all(&buf[..to_write])?;
                        captured += to_write;
                    }
                }
            }
            device.stop_rx()?;
        }
        Mode::Analyze | Mode::Sweep | Mode::Decode | Mode::Serve | Mode::Iridium | Mode::Fix => {
            unreachable!()
        }
    }

    println!("Capture finished successfully.");
    Ok(())
}

#[cfg(test)]
mod status_tests {
    use super::*;
    use std::io::Write as _;

    fn rec(cycle: u64, utc: &str, bursts: &[f64], gnss: bool, sweep: bool) -> String {
        let b: Vec<String> = bursts
            .iter()
            .map(|f| format!(r#"{{"freq_mhz":{:.4},"snr":9.5}}"#, f))
            .collect();
        format!(
            r#"{{"cycle":{},"utc":"{}","iridium":{{"n_bursts":{},"n_locked":2,"n_sync":1,"n_frames":3,"bursts":[{}]}}{}{}}}"#,
            cycle, utc, bursts.len(), b.join(","),
            if gnss { r#","gnss":{"max_metric":1.2}"# } else { "" },
            if sweep { r#","sweep":{"bins":{"1000.5":-45.0}}"# } else { "" },
        )
    }

    fn fold(lines: &[String]) -> StatusAcc {
        let mut a = StatusAcc::default();
        for l in lines {
            a.absorb(&serde_json::from_str(l).unwrap());
        }
        a
    }

    fn same(a: &StatusAcc, b: &StatusAcc) -> bool {
        a.cycles == b.cycles && a.bursts == b.bursts && a.locked == b.locked
            && a.sync == b.sync && a.frames == b.frames && a.chan == b.chan
            && a.gnss == b.gnss && a.sweep == b.sweep && a.first_utc == b.first_utc
            && a.last == b.last
            && a.recent.iter().cloned().collect::<Vec<_>>()
                == b.recent.iter().cloned().collect::<Vec<_>>()
    }

    fn tmp(name: &str) -> std::path::PathBuf {
        let p = std::env::temp_dir().join(format!("gnss_status_{}_{}", name, std::process::id()));
        let _ = std::fs::remove_file(&p);
        p
    }

    #[test]
    fn a_stale_legacy_state_file_expires_instead_of_ruling_from_the_grave() {
        // The per-producer merge expires state.*.json by ttl, but the legacy
        // sync_state.json was merged UNCONDITIONALLY — a dead band_producer's
        // rows would outrank live ones indefinitely (the discipline-tombstone
        // class: a 15-h-old +1.9 ppm beating a live -0.416). A legacy file
        // older than its ttl must contribute nothing; a fresh one must merge.
        let dir = std::env::temp_dir().join(format!("gnss_merge_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        // stale: negative ttl forces staleness regardless of mtime (the
        // existing merge tests' convention)
        std::fs::write(dir.join("sync_state.json"), serde_json::json!({
            "ttl_s": -1, "epoch": 10,
            "sources": [{"band": "Dead Band", "kind": "ClockDriftPpm",
                         "value": 9.9, "epoch": 10}],
            "clock": {"residual_ppm": 9.9},
        }).to_string()).unwrap();
        // a fresh per-producer file keeps the merge non-empty
        std::fs::write(dir.join("state.tick.json"),
                       r#"{"epoch":20,"clock":{"live_tick_hz":32e6}}"#).unwrap();
        let merged = merge_sync_state(&dir).expect("fresh file keeps the doc alive");
        assert!(!merged.contains("Dead Band"),
                "stale legacy row survived the merge: {merged}");
        assert!(!merged.contains("9.9"),
                "stale legacy clock value survived the merge: {merged}");
        assert!(merged.contains("live_tick_hz"), "fresh file lost: {merged}");
        // fresh: merges normally
        std::fs::write(dir.join("sync_state.json"), serde_json::json!({
            "epoch": 30,
            "sources": [{"band": "Dead Band", "value": 9.9, "epoch": 30}],
        }).to_string()).unwrap();
        let merged = merge_sync_state(&dir).unwrap();
        assert!(merged.contains("Dead Band"),
                "fresh legacy row dropped: {merged}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    fn append(path: &std::path::Path, lines: &[String]) {
        let mut f = std::fs::OpenOptions::new()
            .create(true).append(true).open(path).unwrap();
        for l in lines {
            writeln!(f, "{}", l).unwrap();
        }
    }

    #[test]
    fn incremental_reads_match_a_single_full_read() {
        // THE invariant. Serving the dashboard from an accumulator is only safe
        // if folding the log in pieces gives exactly what folding it all at once
        // would. Anything else and the dashboard slowly drifts from the truth
        // while looking perfectly plausible.
        let path = tmp("incr");
        let all: Vec<String> = (0..25)
            .map(|i| rec(i, &format!("2026-08-16T00:{:02}:00Z", i),
                         &[1626.05 + 0.01*(i as f64 % 5.0), 1626.31],
                         i % 4 == 0, i % 7 == 0))
            .collect();

        append(&path, &all[..10]);
        let _ = refresh_status(&path);
        append(&path, &all[10..17]);
        let _ = refresh_status(&path);
        append(&path, &all[17..]);
        let incremental = refresh_status(&path);

        assert!(same(&incremental, &fold(&all)),
                "incremental accumulation diverged from a full re-parse");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn a_half_written_line_is_not_consumed_until_it_is_complete() {
        // The recorder appends while the dashboard polls, so a poll can land
        // mid-write. Parsing the fragment would drop that cycle permanently.
        let path = tmp("partial");
        let full = rec(1, "2026-08-16T00:00:00Z", &[1626.05], false, false);
        append(&path, &[full.clone()]);
        let mut f = std::fs::OpenOptions::new().append(true).open(&path).unwrap();
        write!(f, "{}", &full[..full.len()/2]).unwrap();   // torn line, no newline
        drop(f);

        let a = refresh_status(&path);
        assert_eq!(a.cycles, 1, "a partial line was parsed as a cycle");

        // finish the line; the next poll must pick it up whole
        let mut f = std::fs::OpenOptions::new().append(true).open(&path).unwrap();
        writeln!(f, "{}", &full[full.len()/2..]).unwrap();
        drop(f);
        let b = refresh_status(&path);
        assert_eq!(b.cycles, 2, "the completed line was never picked up");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn a_shrinking_file_resets_instead_of_double_counting() {
        // The recorder restarts and can start a fresh log. Continuing to add to
        // the old accumulator would report totals that never happened.
        let path = tmp("rotate");
        let a: Vec<String> = (0..5)
            .map(|i| rec(i, "2026-08-16T00:00:00Z", &[1626.05], false, false))
            .collect();
        append(&path, &a);
        assert_eq!(refresh_status(&path).cycles, 5);

        std::fs::write(&path, "").unwrap();
        let b = vec![rec(0, "2026-08-16T01:00:00Z", &[1626.09], false, false)];
        append(&path, &b);
        let acc = refresh_status(&path);
        assert_eq!(acc.cycles, 1, "totals carried over a log rotation");
        assert!(same(&acc, &fold(&b)));
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn recent_bursts_are_newest_first_and_capped() {
        let lines: Vec<String> = (0..40)
            .map(|i| rec(i, &format!("2026-08-16T00:{:02}:00Z", i), &[1626.05, 1626.31], false, false))
            .collect();
        let a = fold(&lines);
        assert_eq!(a.recent.len(), RECENT_CAP, "the ring is not capped");
        // the newest record is cycle 39, and within a record order is preserved
        assert_eq!(a.recent[0]["cycle"], 39);
        assert_eq!(a.recent[0]["freq_mhz"].as_f64().unwrap(), 1626.05);
        assert_eq!(a.recent[1]["freq_mhz"].as_f64().unwrap(), 1626.31);
        assert!(a.recent[0]["cycle"].as_u64() > a.recent[59]["cycle"].as_u64());
    }

    #[test]
    fn last_seen_gnss_and_sweep_survive_records_that_lack_them() {
        // Only some cycles carry a gnss check or a sweep. The dashboard shows
        // the most recent one, which must not be blanked by the cycles between.
        let lines = vec![
            rec(0, "t0", &[], true, true),
            rec(1, "t1", &[], false, false),
            rec(2, "t2", &[], false, false),
        ];
        let a = fold(&lines);
        assert!(!a.gnss.is_null(), "the last gnss check was lost");
        assert!(!a.sweep.is_null(), "the last sweep was lost");
        assert_eq!(a.last["cycle"], 2);
        assert_eq!(a.first_utc, "t0");
    }

    #[test]
    fn totals_sum_across_every_record() {
        let lines: Vec<String> = (0..12)
            .map(|i| rec(i, "t", &[1626.05, 1626.09, 1626.31], false, false))
            .collect();
        let a = fold(&lines);
        assert_eq!(a.cycles, 12);
        assert_eq!(a.bursts, 36);
        assert_eq!(a.frames, 36);
        assert_eq!(a.chan.values().sum::<u64>(), 36);
        assert_eq!(a.chan.len(), 3, "channels are tallied at 10 kHz resolution");
    }
}


#[cfg(test)]
mod sync_merge_tests {
    use super::*;

    fn dir(name: &str) -> std::path::PathBuf {
        let p = std::env::temp_dir()
            .join(format!("gnss_syncmerge_{}_{}", name, std::process::id()));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).unwrap();
        p
    }

    fn put(dir: &std::path::Path, name: &str, body: &str) {
        std::fs::write(dir.join(name), body).unwrap();
    }

    #[test]
    fn per_producer_files_override_legacy_for_the_keys_they_carry() {
        let d = dir("precedence");
        put(&d, "sync_state.json",
            r#"{"epoch":10,"clock":{"live_tick_hz":1.0,"residual_ppm":9},"consensus_ppm":0.5}"#);
        put(&d, "state.tick.json",
            r#"{"epoch":20,"seq":7,"clock":{"live_tick_hz":32e6}}"#);
        let m: serde_json::Value =
            serde_json::from_str(&merge_sync_state(&d).unwrap()).unwrap();
        // tick's clock key wins, legacy-only clock key passes through
        assert_eq!(m["clock"]["live_tick_hz"], 32e6);
        assert_eq!(m["clock"]["residual_ppm"], 9.0);
        // legacy-only top-level key passes through; tick-only key appears
        assert_eq!(m["consensus_ppm"], 0.5);
        assert_eq!(m["seq"], 7);
        // epoch is the max across files
        assert_eq!(m["epoch"], 20.0);
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn sources_merge_by_band_and_unknown_bands_pass_through() {
        let d = dir("sources");
        put(&d, "sync_state.json",
            r#"{"sources":[{"band":"L1 / WAAS","value":1},{"band":"GPS L5","value":2}]}"#);
        put(&d, "state.phase.json",
            r#"{"sources":[{"band":"ATSC ch35","value":3}]}"#);
        put(&d, "state.series.json",
            r#"{"sources":[{"band":"L1 / WAAS","value":99}]}"#);
        let m: serde_json::Value =
            serde_json::from_str(&merge_sync_state(&d).unwrap()).unwrap();
        let rows = m["sources"].as_array().unwrap();
        assert_eq!(rows.len(), 3, "bands must merge, not duplicate");
        assert_eq!(rows[0]["band"], "L1 / WAAS");
        assert_eq!(rows[0]["value"], 99, "fresher file's row must win per band");
        assert_eq!(rows[1]["band"], "GPS L5");
        assert_eq!(rows[2]["band"], "ATSC ch35");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn alerts_are_a_deduped_union() {
        let d = dir("alerts");
        put(&d, "sync_state.json", r#"{"alerts":["a","b"]}"#);
        put(&d, "state.phase.json", r#"{"alerts":["b","c"]}"#);
        let m: serde_json::Value =
            serde_json::from_str(&merge_sync_state(&d).unwrap()).unwrap();
        let a: Vec<&str> = m["alerts"].as_array().unwrap()
            .iter().map(|v| v.as_str().unwrap()).collect();
        assert_eq!(a, ["a", "b", "c"]);
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_state_file_older_than_its_ttl_stops_voting() {
        let d = dir("stale");
        put(&d, "sync_state.json", r#"{"epoch":10}"#);
        // ttl_s negative => older than its own ttl no matter the mtime
        put(&d, "state.series.json",
            r#"{"ttl_s":-1,"band_series":{"x":[[1,2]]},"epoch":99}"#);
        let m: serde_json::Value =
            serde_json::from_str(&merge_sync_state(&d).unwrap()).unwrap();
        assert!(m.get("band_series").is_none(),
                "a stale producer's keys must disappear");
        assert_eq!(m["epoch"], 10.0);
        assert!(m.get("ttl_s").is_none(), "ttl_s is a control field");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn no_files_at_all_is_the_pre_writer_state() {
        let d = dir("empty");
        assert!(merge_sync_state(&d).is_none());
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn keys_the_writers_always_emit_stay_present_even_when_empty() {
        // band_producer always writes "alerts" (often []); the panel must
        // keep seeing the key after the merge, exactly as before.
        let d = dir("shape");
        put(&d, "sync_state.json", r#"{"alerts":[],"sources":[],"clock":{}}"#);
        let m: serde_json::Value =
            serde_json::from_str(&merge_sync_state(&d).unwrap()).unwrap();
        assert!(m.get("alerts").is_some());
        assert!(m.get("sources").is_some());
        assert!(m.get("clock").is_some());
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_stale_file_vetoes_the_legacy_relics_it_owns() {
        // The legacy file is a survivor of the shared-file regime: it froze
        // copies of band_series, the "PC clock" row and tick clock keys.
        // When the owner dies (stale), those must DISAPPEAR, not fall back
        // to the frozen relics.
        let d = dir("veto");
        put(&d, "sync_state.json",
            r#"{"epoch":10,"band_series":{"x":[[1,2]]},"clock":{"live_tick_hz":32e6,"residual_ppm":9},"sources":[{"band":"L1 / WAAS","value":1},{"band":"PC clock","value":2}],"alerts":[]}"#);
        // stale series producer: owns band_series + the PC clock row
        put(&d, "state.series.json",
            r#"{"ttl_s":-1,"epoch":99,"band_series":{"x":[[1,2]]},"sources":[{"band":"PC clock","value":2}]}"#);
        // fresh tick producer: owns live_tick_hz, votes a new value
        put(&d, "state.tick.json",
            r#"{"epoch":11,"clock":{"live_tick_hz":31e6}}"#);
        let m: serde_json::Value =
            serde_json::from_str(&merge_sync_state(&d).unwrap()).unwrap();
        assert!(m.get("band_series").is_none(),
                "stale owner's key must vanish, not fall back to the relic");
        let bands: Vec<&str> = m["sources"].as_array().unwrap()
            .iter().map(|r| r["band"].as_str().unwrap()).collect();
        assert_eq!(bands, ["L1 / WAAS"], "the PC clock relic must be vetoed");
        assert_eq!(m["clock"]["live_tick_hz"], 31e6,
                   "fresh owner still overrides the legacy relic");
        assert_eq!(m["clock"]["residual_ppm"], 9.0,
                   "unowned legacy clock keys pass through");
        assert_eq!(m["epoch"], 11.0, "a stale file's epoch does not vote");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn tmp_siblings_and_unrelated_json_are_not_merged() {
        let d = dir("glob");
        put(&d, "sync_state.json", r#"{"epoch":1}"#);
        put(&d, "state.tick.json.tmp", r#"{"poison":true}"#);
        put(&d, "statement.json", r#"{"poison":true}"#);
        let m: serde_json::Value =
            serde_json::from_str(&merge_sync_state(&d).unwrap()).unwrap();
        assert!(m.get("poison").is_none());
        let _ = std::fs::remove_dir_all(&d);
    }
}
