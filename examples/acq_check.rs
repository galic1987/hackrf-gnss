//! Verify code-Doppler compensation on real captured baseband: acquire GPS PRN 4
//! and SBAS PRN 131 with compensation off vs on, at 2 s and 4 s integration.
//! usage: acq_check <baseband.f32> <fs>
use hackrf_gnss::gps::{acquire_codes, ca_code::gps_ca, sbas_code, F_L1, ca_code::CHIP_RATE};
use num_complex::Complex;
use std::fs;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let bytes = fs::read(&a[1]).unwrap();
    let fs: f64 = a[2].parse().unwrap();
    let sig: Vec<Complex<f32>> = bytes.chunks_exact(8).map(|c| Complex::new(
        f32::from_le_bytes([c[0],c[1],c[2],c[3]]), f32::from_le_bytes([c[4],c[5],c[6],c[7]]))).collect();
    let dopp: Vec<f64> = (-2000..=2000).step_by(100).map(|x| x as f64).collect();
    let targets: [(&str, usize, Vec<f32>); 2] =
        [("GPS PRN 4", 4, gps_ca(4)), ("SBAS PRN 131", 131, sbas_code(131))];
    println!("{:<14} {:>8} {:>12} {:>12}", "target", "n_ms", "metric(off)", "metric(on)");
    for (name, prn, code) in &targets {
        for &nms in &[2000usize, 4000] {
            let codes = vec![(*prn, code.clone())];
            let off = acquire_codes(&sig, fs, &codes, CHIP_RATE, &dopp, nms, 2.5, F_L1, false);
            let on  = acquire_codes(&sig, fs, &codes, CHIP_RATE, &dopp, nms, 2.5, F_L1, true);
            println!("{:<14} {:>8} {:>12.2} {:>12.2}", name, nms, off[0].metric, on[0].metric);
        }
    }
}
