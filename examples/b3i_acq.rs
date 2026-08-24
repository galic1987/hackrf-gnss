//! BeiDou B3I acquisition on a real capture at 1268.52 MHz. 10230 chips at
//! 10.23 Mcps; at 8 Msps the capture cannot hold the full code bandwidth, so
//! this is partial-band correlation (~7 dB loss) like anyband_acq's b3 mode.
//! usage: b3i_acq <iq> <fs> <fc> [secs]
use hackrf_gnss::beidou::b3i_code;
use hackrf_gnss::gps::acquire_codes;
use num_complex::Complex;
use std::f64::consts::PI;
use std::fs::File;
use std::io::Read;

const B3I: f64 = 1268.52e6;

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
    let ifhz = B3I - fc;
    for (k, v) in sig.iter_mut().enumerate() {
        *v -= mean;
        let ph = -2.0 * PI * ifhz * k as f64 / fs;
        *v *= Complex::new(ph.cos() as f32, ph.sin() as f32);
    }
    let codes: Vec<(usize, Vec<f32>)> = (1..=37).map(|p| (p, b3i_code(p))).collect();
    let dop: Vec<f64> = (-10000..=10000).step_by(200).map(|x| x as f64).collect();
    let nms = (secs * 1000.0) as usize;
    let mut res = acquire_codes(&sig, fs, &codes, 10.23e6, &dop, nms, 2.5, B3I, true);
    res.sort_by(|x, y| y.metric.partial_cmp(&x.metric).unwrap());
    println!("BeiDou B3I acquisition at {B3I} Hz ({secs} s, partial-band):");
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
