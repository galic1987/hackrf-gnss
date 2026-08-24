//! GPS L2C CM acquisition on a real capture at 1227.60 MHz. The CM code is
//! 10230 chips at 0.5115 Mcps (20 ms period) so the 1 ms-block acquire_codes
//! cannot host it; this uses upper_l::acquire_l2c_cm (20 ms coherent blocks,
//! non-coherent over the capture). Only Block IIR-M/IIF/III/IIIA broadcast
//! L2C. The full-slot replica sees ~half the CM amplitude (the other half of
//! each slot carries CL) - acceptable for presence detection.
//! usage: l2c_acq <iq> <fs> <fc> [secs]
use hackrf_gnss::upper_l::{acquire_l2c_cm, F_L2};
use num_complex::Complex;
use std::f64::consts::PI;
use std::fs::File;
use std::io::Read;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let fs: f64 = a[2].parse().unwrap();
    let fc: f64 = a[3].parse().unwrap();
    let secs: f64 = a.get(4).and_then(|s| s.parse().ok()).unwrap_or(6.0);
    let mut f = File::open(&a[1]).expect("open");
    let mut raw = vec![0u8; (2.0 * secs * fs) as usize * 2];
    let n = f.read(&mut raw).unwrap();
    raw.truncate(n);
    let mut sig: Vec<Complex<f32>> = raw
        .chunks_exact(2)
        .map(|c| Complex::new(c[0] as i8 as f32, c[1] as i8 as f32))
        .collect();
    let mean: Complex<f32> = sig.iter().copied().sum::<Complex<f32>>() / sig.len() as f32;
    let ifhz = F_L2 - fc;
    for (k, v) in sig.iter_mut().enumerate() {
        *v -= mean;
        let ph = -2.0 * PI * ifhz * k as f64 / fs;
        *v *= Complex::new(ph.cos() as f32, ph.sin() as f32);
    }
    let dop: Vec<f64> = (-6000..=6000).step_by(250).map(|x| x as f64).collect();
    let nperiods = (secs / 0.02) as usize;
    let prns: Vec<usize> = (1..=32).collect();
    let mut res = acquire_l2c_cm(&sig, fs, &prns, &dop, nperiods, 2.5);
    res.sort_by(|x, y| y.metric.partial_cmp(&x.metric).unwrap());
    println!("GPS L2C CM acquisition at {F_L2} Hz ({secs} s, 20 ms blocks):");
    for r in res.iter().take(10) {
        println!(
            "  PRN {:2}  metric {:5.2}  dopp {:+7.0}  cp {:8.1}  {}",
            r.prn,
            r.metric,
            r.doppler,
            r.code_phase,
            if r.metric > 2.5 { "<== ACQUIRED" } else { "" }
        );
    }
    println!("=> {} acquired", res.iter().filter(|r| r.metric > 2.5).count());
}
