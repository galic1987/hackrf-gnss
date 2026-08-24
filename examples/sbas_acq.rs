//! Acquire SBAS (WAAS) PRNs on an L1 baseband capture. GEO => near-zero Doppler,
//! so a hit doubles as a stationary carrier reference for clock-drift work.
//! usage: sbas_acq <bb.f32> <fs> <nms>
use hackrf_gnss::gps::{acquire_sbas, AcqResult};
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
    // WAAS set visible over CONUS + a couple of neighbours, narrow Doppler window
    let prns: Vec<usize> = vec![121, 122, 123, 131, 133, 135, 136, 138, 139];
    let dopp: Vec<f64> = (-3000..=3000).step_by(100).map(|x| x as f64).collect();
    let mut res: Vec<AcqResult> = acquire_sbas(&sig, fs, &prns, &dopp, nms, 2.5);
    res.sort_by(|a, b| b.metric.partial_cmp(&a.metric).unwrap());
    println!("SBAS acquisition ({} ms):", nms);
    for r in res.iter().take(9) {
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
