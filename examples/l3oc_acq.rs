//! GLONASS L3OC (CDMA, 1202.025 MHz) acquisition: searches the L3OCd (data)
//! and L3OCp (pilot) 10230-chip truncated Kasami codes at 10.23 Mcps. Only
//! GLONASS-K satellites transmit L3OC. At 8 Msps this is partial-band
//! correlation (~7 dB loss), like anyband_acq.
//! usage: l3oc_acq <iq> <fs> <fc> [secs]
use hackrf_gnss::gps::acquire_codes;
use hackrf_gnss::upper_l::l3oc::{l3ocd_code, l3ocp_code, CHIP_RATE, F_L3};
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
    let ifhz = F_L3 - fc;
    for (k, v) in sig.iter_mut().enumerate() {
        *v -= mean;
        let ph = -2.0 * PI * ifhz * k as f64 / fs;
        *v *= Complex::new(ph.cos() as f32, ph.sin() as f32);
    }
    let dop: Vec<f64> = (-10000..=10000).step_by(400).map(|x| x as f64).collect();
    let nms = (secs * 1000.0) as usize;
    // GLONASS orbital slots 1..=24 (+ a few spares); both components searched.
    for (name, genfn) in [("L3OCd", l3ocd_code as fn(usize) -> Vec<f32>), ("L3OCp", l3ocp_code)] {
        let codes: Vec<(usize, Vec<f32>)> = (1..=26).map(|p| (p, genfn(p))).collect();
        let mut res = acquire_codes(&sig, fs, &codes, CHIP_RATE, &dop, nms, 2.5, F_L3, true);
        res.sort_by(|x, y| y.metric.partial_cmp(&x.metric).unwrap());
        println!("GLONASS {name} acquisition at {F_L3} Hz ({secs} s, partial-band):");
        for r in res.iter().take(8) {
            println!(
                "  SV {:2}  metric {:5.2}  dopp {:+7.0}  cp {:8.1}  {}",
                r.prn,
                r.metric,
                r.doppler,
                r.code_phase,
                if r.metric > 2.5 { "<== ACQUIRED" } else { "" }
            );
        }
        println!("=> {} acquired", res.iter().filter(|r| r.metric > 2.5).count());
    }
}
