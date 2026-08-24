//! throwaway diagnostic: per-second prompt/noise power breakdown for one sat
use hackrf_gnss::gps::acquire;
use hackrf_gnss::live::Channel;
use hackrf_gnss::live::Sys;
use num_complex::Complex;
use std::io::Read;

fn main() {
    let path = format!("{}/scripts/l1_30s_bb.f32", env!("CARGO_MANIFEST_DIR"));
    let fs = 4.0e6f64;
    let mut f = std::fs::File::open(&path).unwrap();
    let mut bytes = vec![0u8; (fs as usize) * 8 * 8];
    let n = f.read(&mut bytes).unwrap();
    bytes.truncate(n);
    let sig: Vec<Complex<f32>> = bytes
        .chunks_exact(8)
        .map(|c| {
            Complex::new(
                f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
            )
        })
        .collect();
    let seed = &sig[..(fs as usize) * 2];
    let dopp: Vec<f64> = (-5000..=5000).step_by(500).map(|x| x as f64).collect();
    let prn: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(26);
    let r = acquire(seed, fs, &[prn], &dopp, 2000, 2.5)[0].clone();
    println!("acq PRN {}: metric {:.2} dopp {:+.0} cp {:.1}", prn, r.metric, r.doppler, r.code_phase);
    let mut ch = Channel::new(Sys::Gps, prn, fs, r.doppler, r.code_phase);
    ch.debug_refine(seed);
    let ns = 4000usize;
    for s in 0..6 {
        let mut p_acc = 0.0;
        let mut q_acc = 0.0;
        let mut n_acc = 0.0;
        for e in 0..1000 {
            let (p, q, nz) = ch.debug_epoch(&sig[(2 + s) * fs as usize + e * ns..]);
            p_acc += p;
            q_acc += q;
            n_acc += nz;
        }
        println!(
            "sec {}: prompt {:.3e} quad {:.3e} offcode {:.3e}  ratio_p/q {:.1} ratio_p/n {:.3} dopp {:+.1}",
            s,
            p_acc / 1000.0,
            q_acc / 1000.0,
            n_acc / 1000.0,
            (p_acc / 1000.0) / (q_acc / 1000.0),
            (p_acc / 1000.0) / (n_acc / 1000.0),
            ch.debug_dopp(),
        );
    }
}
