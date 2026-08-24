//! Refine carrier Doppler before tracking: 1 ms code-stripped prompt sequence
//! (tolerant to ~100 Hz handoff error) -> FFT -> residual carrier to ~0.25 Hz.
//! usage: dop_refine <bb.f32> <fs> <prn> <dopp_hz> <code_phase_chips> <seconds>
use hackrf_gnss::gps::ca_code::{gps_ca, CHIP_RATE};
use hackrf_gnss::gps::F_L1;
use num_complex::Complex;
use rustfft::FftPlanner;
use std::f64::consts::PI;
use std::fs;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let bytes = fs::read(&a[1]).unwrap();
    let fs: f64 = a[2].parse().unwrap();
    let prn: usize = a[3].parse().unwrap();
    let dopp: f64 = a[4].parse().unwrap();
    let cp0: f64 = a[5].parse().unwrap();
    let secs: usize = a[6].parse().unwrap();
    let sig: Vec<Complex<f32>> = bytes
        .chunks_exact(8)
        .map(|c| {
            Complex::new(
                f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
            )
        })
        .collect();
    let code = gps_ca(prn);
    let ns = (fs / 1000.0).round() as usize;
    let n_ms = (secs * 1000).min(sig.len() / ns);
    let code_rate = CHIP_RATE * (1.0 + dopp / F_L1);
    let code_step = code_rate / fs;
    let dphi = 2.0 * PI * dopp / fs;
    let mut prompts: Vec<Complex<f32>> = Vec::with_capacity(n_ms);
    let mut code_phase = cp0;
    let mut carrier_phase = 0.0f64;
    for e in 0..n_ms {
        let (mut ip, mut qp) = (0.0f64, 0.0f64);
        for k in 0..ns {
            let s = sig[e * ns + k];
            let ph = carrier_phase + dphi * k as f64;
            let bi = s.re as f64 * ph.cos() + s.im as f64 * ph.sin();
            let bq = -s.re as f64 * ph.sin() + s.im as f64 * ph.cos();
            let c = code_phase + code_step * k as f64;
            let idx = c.rem_euclid(1023.0) as usize % 1023;
            ip += bi * code[idx] as f64;
            qp += bq * code[idx] as f64;
        }
        code_phase = (code_phase + code_step * ns as f64).rem_euclid(1023.0);
        carrier_phase = (carrier_phase + dphi * ns as f64).rem_euclid(2.0 * PI);
        prompts.push(Complex::new(ip as f32, qp as f32));
    }
    let nfft = 1 << (usize::BITS - (n_ms - 1).leading_zeros()); // next pow2 >= n_ms
    let mut buf = prompts.clone();
    buf.resize(nfft, Complex::new(0.0, 0.0));
    let mut planner = FftPlanner::new();
    planner.plan_fft_forward(nfft).process(&mut buf);
    let mut best = (0usize, 0.0f32);
    for (i, b) in buf.iter().enumerate().take(nfft / 2).skip(1) {
        let p = b.re * b.re + b.im * b.im;
        if p > best.1 { best = (i, p); }
    }
    // prompt stream is at 1 kHz; positive residual = true dopp above handoff.
    // data-bit sign flips can split energy; take the strongest peak either way.
    let residual = best.0 as f64 * 1000.0 / nfft as f64;
    let refined = dopp + residual;
    let noise: f32 = buf.iter().map(|b| b.re * b.re + b.im * b.im).sum::<f32>() / nfft as f32;
    println!(
        "handoff {:+.0} Hz, residual {:+.2} Hz -> refined {:+.2} Hz (peak/noise {:.1})",
        dopp, residual, refined, best.1 / noise.max(1e-9)
    );
}
