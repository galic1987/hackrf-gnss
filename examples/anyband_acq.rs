//! Generic wideband GNSS acquisition for L5 (GPS/QZSS) and B3I (BeiDou):
//! mixes the capture to baseband, decimates lightly, and correlates the
//! 10230-chip codes. Partial-band at 4 Msps — expect ~7 dB correlation loss.
//! usage: anyband_acq <iq> <fs> <fc> <secs> <l5|b3>
use hackrf_gnss::beidou::b3i_code;
use hackrf_gnss::gps::acquire_codes;
use hackrf_gnss::gps::l5_code::l5_code;
use hackrf_gnss::iridium::demod3::decimate;
use num_complex::Complex;
use std::f64::consts::PI;
use std::fs;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let raw: Vec<i8> = fs::read(&a[1]).unwrap().into_iter().map(|b| b as i8).collect();
    let fs: f64 = a[2].parse().unwrap();
    let fc: f64 = a[3].parse().unwrap();
    let secs: f64 = a[4].parse().unwrap();
    let band = a[5].as_str();
    let n = (fs * secs) as usize;
    let mut sig: Vec<Complex<f32>> = Vec::with_capacity(n);
    for k in 0..n.min(raw.len() / 2) {
        let ph = -2.0 * PI * 0.0 * k as f64 / fs; // already at baseband (fc = tune)
        let (i, q) = (raw[2 * k] as f32 / 128.0, raw[2 * k + 1] as f32 / 128.0);
        sig.push(Complex::new(i * ph.cos() as f32 - q * ph.sin() as f32,
                              i * ph.sin() as f32 + q * ph.cos() as f32));
    }
    let _ = decimate; // samples already at 4 Msps; use as-is
    let (chiprate, codes): (f64, Vec<(usize, Vec<f32>)>) = match band {
        "l5" => (10.23e6, (1..=32).map(|p| (p, l5_code(p, true))).collect()),
        "b3" => (10.23e6, (1..=37).map(|p| (p, b3i_code(p))).collect()),
        _ => panic!("band must be l5 or b3"),
    };
    let dopp: Vec<f64> = (-10000..=10000).step_by(200).map(|x| x as f64).collect();
    let nms = (secs * 1000.0) as usize;
    let mut res = acquire_codes(&sig, fs, &codes, chiprate, &dopp, nms, 2.5, fc, true);
    res.sort_by(|x, y| y.metric.partial_cmp(&x.metric).unwrap());
    println!("{band} acquisition at {fc} Hz ({secs} s, partial-band):");
    for r in res.iter().take(10) {
        println!(
            "  PRN {:2}  metric {:5.2}  dopp {:+7.0}  cp {:7.1}  {}",
            r.prn, r.metric, r.doppler, r.code_phase,
            if r.metric > 2.5 { "<== ACQUIRED" } else { "" }
        );
    }
    println!("=> {} acquired", res.iter().filter(|r| r.metric > 2.5).count());
}
