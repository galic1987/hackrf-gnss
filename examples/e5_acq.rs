//! Acquisition for the 1176.45 MHz band (E5/L5/B2a): GPS L5 (Q5 pilot),
//! Galileo E5a-I and BeiDou B2a (data or pilot). All are BPSK(10)-family
//! signals with 10230-chip codes at 10.23 Mcps, so at 8 Msps capture rate the
//! full code cannot be represented — the replica is resampled to the capture
//! rate (partial-band, ~7 dB correlation loss, fine for presence detection).
//! Same approach as anyband_acq.rs.
//!
//! Codes: GPS L5 from gps::l5_code (IS-GPS-705 golden-tested), Galileo E5a-I
//! from e5::e5a_code, BeiDou B2a from e5::b2a_code (both golden-tested against
//! their ICDs — see those modules for provenance).
//!
//! usage: e5_acq <iq> <fs> <fc> <secs> <l5|e5a|b2a|b2ap>
use hackrf_gnss::e5::b2a_code::b2a_code;
use hackrf_gnss::e5::e5a_code::e5ai_code;
use hackrf_gnss::gps::acquire_codes;
use hackrf_gnss::gps::l5_code::l5_code;
use num_complex::Complex;
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
        // capture is already at baseband (tuned to fc)
        sig.push(Complex::new(raw[2 * k] as f32 / 128.0, raw[2 * k + 1] as f32 / 128.0));
    }
    let (chiprate, codes): (f64, Vec<(usize, Vec<f32>)>) = match band {
        "l5" => (10.23e6, (1..=32).map(|p| (p, l5_code(p, true))).collect()), // Q5 pilot
        "e5a" => (10.23e6, (1..=50).map(|p| (p, e5ai_code(p))).collect()),
        "b2a" => (10.23e6, (1..=63).map(|p| (p, b2a_code(p, false))).collect()), // data
        "b2ap" => (10.23e6, (1..=63).map(|p| (p, b2a_code(p, true))).collect()), // pilot
        _ => panic!("band must be l5|e5a|b2a|b2ap"),
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
