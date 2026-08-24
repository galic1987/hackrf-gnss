//! Diagnostic: with the tracker's own math, sweep code-phase offset around the
//! acquisition handoff and print prompt power — is the peak where acq says?
//! usage: track_diag <bb.f32> <fs> <prn> <dopp_hz> <code_phase_chips>
use hackrf_gnss::gps::ca_code::{gps_ca, CHIP_RATE};
use hackrf_gnss::gps::F_L1;
use num_complex::Complex;
use std::f64::consts::PI;
use std::fs;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let bytes = fs::read(&a[1]).unwrap();
    let fs: f64 = a[2].parse().unwrap();
    let prn: usize = a[3].parse().unwrap();
    let dopp: f64 = a[4].parse().unwrap();
    let cp0: f64 = a[5].parse().unwrap();
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
    let epochs = 20.min(sig.len() / ns); // 20 ms
    println!("code-phase offset scan around {cp0} chips (dopp {dopp}):");
    let mut k = -12;
    while k <= 12 {
        let off = k as f64 * 0.25;
        let (mut ip, mut qp) = (0.0f64, 0.0f64);
        let mut code_phase = cp0 + off;
        let mut carrier_phase = 0.0f64;
        for e in 0..epochs {
            let code_rate = CHIP_RATE * (1.0 + dopp / F_L1);
            let code_step = code_rate / fs;
            let dphi = 2.0 * PI * dopp / fs;
            for kk in 0..ns {
                let s = sig[e * ns + kk];
                let ph = carrier_phase + dphi * kk as f64;
                let bi = s.re as f64 * ph.cos() + s.im as f64 * ph.sin();
                let bq = -s.re as f64 * ph.sin() + s.im as f64 * ph.cos();
                let c = code_phase + code_step * kk as f64;
                let idx = c.rem_euclid(1023.0) as usize % 1023;
                let chip = code[idx] as f64;
                ip += bi * chip;
                qp += bq * chip;
            }
            code_phase = (code_phase + code_step * ns as f64).rem_euclid(1023.0);
            carrier_phase = (carrier_phase + dphi * ns as f64).rem_euclid(2.0 * PI);
        }
        let p = (ip * ip + qp * qp).sqrt();
        println!("  off {:+.2} chips  |P| {:.1}", off, p);
        k += 1;
    }
}
