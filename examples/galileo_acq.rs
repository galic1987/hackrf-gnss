//! Acquire Galileo E1B on an L1 baseband capture (E1 shares 1575.42 MHz with
//! GPS L1). Correlates against a BOC(1,1)-shaped replica of the ICD memory
//! codes over 4 ms coherent blocks with non-coherent sums.
//! usage: galileo_acq <bb.f32> <fs> <nms>
use hackrf_gnss::galileo::acquire_e1b;
use hackrf_gnss::gps::AcqResult;
use num_complex::Complex;
use std::fs;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let bytes = fs::read(&a[1]).unwrap();
    let fs: f64 = a[2].parse().unwrap();
    let nms: usize = a[3].parse().unwrap();
    let sig: Vec<Complex<f32>> = bytes
        .chunks_exact(8)
        .map(|c| {
            Complex::new(
                f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
            )
        })
        .collect();
    // full constellation in view of the ICD table, L1 Doppler window
    let prns: Vec<usize> = (1..=36).collect();
    let dopp: Vec<f64> = (-3000..=3000).step_by(250).map(|x| x as f64).collect();
    // nms is in ms; the coherent block is the 4 ms E1 code period
    let mut res: Vec<AcqResult> = acquire_e1b(&sig, fs, &prns, &dopp, nms / 4, 2.5);
    res.sort_by(|a, b| b.metric.partial_cmp(&a.metric).unwrap());
    println!("Galileo E1B acquisition ({} ms):", nms);
    for r in res.iter().take(8) {
        println!(
            "  PRN {:3}  metric {:5.2}  dopp {:+6.0}  cp {:6.1}  {}",
            r.prn,
            r.metric,
            r.doppler,
            r.code_phase,
            if r.metric > 2.5 { "<== ACQUIRED" } else { "" }
        );
    }
}
