//! Acquire GPS from a baseband IQ file and print per-PRN results as JSON.
//!
//! Reads interleaved little-endian f32 (I,Q,I,Q,...) already mixed to baseband
//! at `fs`. Used to validate the Rust acquisition against the Python oracle on
//! identical real samples — the Python front end dumps the baseband, both sides
//! acquire it, and the results are compared.
//!
//! usage: acquire_file <baseband.f32> <fs> <dopp_lo> <dopp_hi> <dopp_step> <nblocks>

use num_complex::Complex;
use std::fs;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() != 7 {
        eprintln!("usage: acquire_file <baseband.f32> <fs> <dopp_lo> <dopp_hi> <dopp_step> <nblocks>");
        std::process::exit(2);
    }
    let path = &a[1];
    let fs: f64 = a[2].parse().unwrap();
    let dlo: f64 = a[3].parse().unwrap();
    let dhi: f64 = a[4].parse().unwrap();
    let dstep: f64 = a[5].parse().unwrap();
    let nblocks: usize = a[6].parse().unwrap();

    let bytes = fs::read(path).expect("read baseband");
    let sig: Vec<Complex<f32>> = bytes
        .chunks_exact(8)
        .map(|c| {
            let i = f32::from_le_bytes([c[0], c[1], c[2], c[3]]);
            let q = f32::from_le_bytes([c[4], c[5], c[6], c[7]]);
            Complex::new(i, q)
        })
        .collect();

    let mut dopplers = Vec::new();
    let mut d = dlo;
    while d <= dhi + 1e-9 {
        dopplers.push(d);
        d += dstep;
    }

    let prns: Vec<usize> = (1..=32).collect();
    let mut res = hackrf_gnss::gps::acquire(&sig, fs, &prns, &dopplers, nblocks, 2.5);
    // also sweep SBAS (WAAS/EGNOS) — GEO, so a narrow Doppler window is enough
    let sbas_dopp: Vec<f64> = (-3000..=3000).step_by(250).map(|x| x as f64).collect();
    let sbas_prns: Vec<usize> = (120..=158).collect();
    let sres = hackrf_gnss::gps::acquire_sbas(&sig, fs, &sbas_prns, &sbas_dopp, nblocks, 2.5);
    res.extend(sres);
    res.sort_by(|a, b| b.metric.partial_cmp(&a.metric).unwrap());
    println!("{}", serde_json::to_string(&res).unwrap());
}
