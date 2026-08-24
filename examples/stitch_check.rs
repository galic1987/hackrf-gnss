//! Live stitch test for the FPGA timestamp counter: takes two captures of a
//! stable narrowband on-air tone (same tuning, taken ~60 s apart) plus their
//! timestamp sidecars, and checks that the tone's phase advance between the
//! two stream starts equals what the counter says elapsed:
//!
//!     phi2 - phi1  =?=  2*pi*f_tone * (start2 - start1) / tick_hz   (mod 2*pi)
//!
//! The residual is printed in samples (at fs). Sub-sample agreement means the
//! counter timestamps and the RF stream advance in lockstep.
//!
//! Tone frequency/phase per capture: coarse FFT peak in a search window, then
//! a weighted least-squares fit of the demodulated chunk phases (frequency =
//! slope, phase = intercept at sample 0). The prediction uses the mean of the
//! two per-capture frequency fits to absorb slow tone wander.
//!
//! usage: stitch_check <cap1.iq> <sidecar1.json> <cap2.iq> <sidecar2.json> [f_min] [f_max]
//!   captures are interleaved i8 I/Q; f_min/f_max bound the tone search in
//!   baseband Hz (default 300 kHz .. 3 MHz, clear of the DC spur).
use hackrf_gnss::ts_sidecar::TsSidecar;
use num_complex::Complex;
use rustfft::FftPlanner;
use std::f64::consts::PI;

fn read_i8_iq(path: &str) -> Vec<Complex<f32>> {
    let bytes = std::fs::read(path).expect("read capture");
    bytes
        .chunks_exact(2)
        .map(|c| Complex::new(c[0] as i8 as f32, c[1] as i8 as f32))
        .collect()
}

/// (frequency Hz, phase at sample 0 rad, mean chunk SNR-ish amplitude)
fn fit_tone(x: &[Complex<f32>], fs: f64, f_min: f64, f_max: f64) -> (f64, f64, f64) {
    // coarse: FFT of the first ~0.5 s, peak in [f_min, f_max]
    let nfft = (x.len()).min(1 << 22);
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(nfft);
    let mut buf: Vec<Complex<f32>> = x[..nfft].to_vec();
    fft.process(&mut buf);
    let k_lo = (f_min * nfft as f64 / fs).ceil() as usize;
    let k_hi = (f_max * nfft as f64 / fs).floor() as usize;
    let k_peak = (k_lo..=k_hi)
        .max_by(|&a, &b| buf[a].norm_sqr().total_cmp(&buf[b].norm_sqr()))
        .expect("empty search window");
    let f_coarse = k_peak as f64 * fs / nfft as f64;
    let peak = buf[k_peak].norm() as f64 / nfft as f64;
    eprintln!(
        "  coarse peak at {:.1} Hz (amplitude {:.4})",
        f_coarse, peak
    );

    // refine: demodulate at f_coarse, block-average into ~1 ms chunks, fit a
    // line to the unwrapped chunk phases (amplitude-weighted)
    let chunk = (fs / 1000.0) as usize; // 1 ms
    let n_chunks = x.len() / chunk;
    let mut phase = Vec::with_capacity(n_chunks);
    let mut tvec = Vec::with_capacity(n_chunks);
    let mut wgt = Vec::with_capacity(n_chunks);
    for (k, c) in x.chunks(chunk).take(n_chunks).enumerate() {
        let mut acc = Complex::<f64>::new(0.0, 0.0);
        for (i, &s) in c.iter().enumerate() {
            let n = k * chunk + i;
            let (sn, cs) = (2.0 * PI * f_coarse * n as f64 / fs).sin_cos();
            acc += Complex::new(s.re as f64, s.im as f64) * Complex::new(cs, -sn);
        }
        phase.push(acc.im.atan2(acc.re));
        tvec.push((k as f64 + 0.5) * chunk as f64 / fs);
        wgt.push(acc.norm_sqr());
    }
    // unwrap
    for k in 1..phase.len() {
        let mut d = phase[k] - phase[k - 1];
        while d > PI {
            phase[k] -= 2.0 * PI;
            d -= 2.0 * PI;
        }
        while d < -PI {
            phase[k] += 2.0 * PI;
            d += 2.0 * PI;
        }
    }
    // weighted least squares phase = a + b*t
    let (mut sw, mut st, mut sp, mut stt, mut stp) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for k in 0..phase.len() {
        let (w, t, p) = (wgt[k], tvec[k], phase[k]);
        sw += w;
        st += w * t;
        sp += w * p;
        stt += w * t * t;
        stp += w * t * p;
    }
    let det = sw * stt - st * st;
    let a = (sp * stt - st * stp) / det;
    let b = (sw * stp - st * sp) / det;
    let f = f_coarse + b / (2.0 * PI);
    // intercept `a` is the phase at t=0 modulo the wrapping history; the fit
    // ran on unwrapped phases anchored at chunk 0, so a is a valid phase at
    // sample 0 only mod 2*pi — which is all the comparison needs.
    (f, a, (wgt.iter().sum::<f64>() / phase.len() as f64).sqrt() / chunk as f64)
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 5 {
        eprintln!(
            "usage: stitch_check <cap1.iq> <sidecar1.json> <cap2.iq> <sidecar2.json> [f_min] [f_max] [tps]"
        );
        std::process::exit(2);
    }
    let f_min: f64 = a.get(5).and_then(|s| s.parse().ok()).unwrap_or(300e3);
    let f_max: f64 = a.get(6).and_then(|s| s.parse().ok()).unwrap_or(3e6);
    // ticks per sample: exact by construction (the counter and the decimation
    // chain share the adclk domain). 4 at 8 Msps std, 16 at 2.5 Msps ext.
    let tps: f64 = a.get(7).and_then(|s| s.parse().ok()).unwrap_or(4.0);
    let sc1: TsSidecar =
        serde_json::from_str(&std::fs::read_to_string(&a[2]).unwrap()).unwrap();
    let sc2: TsSidecar =
        serde_json::from_str(&std::fs::read_to_string(&a[4]).unwrap()).unwrap();
    assert!((sc1.fs - sc2.fs).abs() < 1.0, "sidecar fs mismatch");
    let fs = sc1.fs;

    let x1 = read_i8_iq(&a[1]);
    eprintln!("cap1: {} samples, fitting tone...", x1.len());
    let (f1, p1, a1) = fit_tone(&x1, fs, f_min, f_max);
    let x2 = read_i8_iq(&a[3]);
    eprintln!("cap2: {} samples, fitting tone...", x2.len());
    let (f2, p2, a2) = fit_tone(&x2, fs, f_min, f_max);

    let f_mean = 0.5 * (f1 + f2);
    let dticks = (sc2.stream_start_ticks - sc1.stream_start_ticks) as f64;
    // Self-consistent prediction in sample-period units: f/fs is cycles per
    // sample period from the fit, dticks/tps is elapsed sample periods from
    // the counter. Any common clock error (ppm) cancels between the two —
    // the calibrated tick_hz in the sidecars is NOT used for the prediction
    // (its bracket calibration carries transfer-overhead bias; printed below
    // for reference only).
    let dt_samples = dticks / tps;
    let dphi_pred = 2.0 * PI * (f_mean / fs) * dt_samples;
    let dphi_meas = (p2 - p1).rem_euclid(2.0 * PI);
    let mut err = dphi_meas - dphi_pred.rem_euclid(2.0 * PI);
    if err > PI {
        err -= 2.0 * PI;
    }
    if err < -PI {
        err += 2.0 * PI;
    }
    let err_samples = err / (2.0 * PI * f_mean) * fs;

    let tick_hz = 0.5 * (sc1.tick_hz + sc2.tick_hz);
    println!("tone1: {:.3} Hz phase {:+.6} rad (amp {:.4})", f1, p1, a1);
    println!("tone2: {:.3} Hz phase {:+.6} rad (amp {:.4})", f2, p2, a2);
    println!("tone wander between captures: {:+.3} Hz", f2 - f1);
    println!(
        "counter: dticks = {} -> {:.3} sample periods ({:.6} s at fs; sidecar tick_hz {:.2})",
        dticks,
        dt_samples,
        dt_samples / fs,
        tick_hz
    );
    println!(
        "phase: measured {:+.6} rad, predicted {:+.6} rad (mod 2pi)",
        dphi_meas,
        dphi_pred.rem_euclid(2.0 * PI)
    );
    println!("STITCH RESULT: phase error = {:+.4} samples ({:+.6} rad)", err_samples, err);
}
