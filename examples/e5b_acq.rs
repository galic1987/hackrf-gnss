//! Galileo E5b-I / BeiDou B2b-I acquisition on the 1207.14 MHz band.
//! Both are BPSK(10) at 10.23 Mcps with 10230-chip codes, so at 8 Msps capture
//! rate the code is not representable: the capture is decimated to ~4 Msps and
//! correlated partial-band (same approach as anyband_acq.rs for L5/B3I) —
//! expect ~7 dB correlation loss, fine for presence detection.
//! usage: e5b_acq <iq> <fs> <fc> <secs> <e5b|b2b> [target_rate=4e6]
use hackrf_gnss::e5b::{b2b_code, e5b_i_code, CHIP_RATE, F_1207};
use hackrf_gnss::gps::acquire_codes;
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
    let ifhz = F_1207 - fc; // residual IF if the capture was not tuned on-centre
    for k in 0..n.min(raw.len() / 2) {
        let ph = -2.0 * PI * ifhz * k as f64 / fs;
        let (i, q) = (raw[2 * k] as f32 / 128.0, raw[2 * k + 1] as f32 / 128.0);
        sig.push(Complex::new(
            i * ph.cos() as f32 - q * ph.sin() as f32,
            i * ph.sin() as f32 + q * ph.cos() as f32,
        ));
    }
    // decimate to ~4 Msps by default: the FIR inside `decimate` keeps only the
    // central part of the 10.23 Mcps main lobe (partial-band correlation).
    // A wider target rate (e.g. 8e6 = no decimation at 8 Msps) keeps +-4 MHz of
    // the main lobe (~+3 dB) at 2x the compute.
    let target: f64 = a.get(6).and_then(|s| s.parse().ok()).unwrap_or(4.0e6);
    let q = (fs / target).round() as usize;
    let (sig, fsd) = if q > 1 {
        (decimate(&sig, q), fs / q as f64)
    } else {
        (sig, fs)
    };
    let codes: Vec<(usize, Vec<f32>)> = match band {
        "e5b" => (1..=50).map(|p| (p, e5b_i_code(p))).collect(),
        "b2b" => (6..=58).map(|p| (p, b2b_code(p))).collect(),
        _ => panic!("band must be e5b or b2b"),
    };
    let dopp: Vec<f64> = (-10000..=10000).step_by(200).map(|x| x as f64).collect();
    let nms = (secs * 1000.0) as usize;
    let mut res = acquire_codes(&sig, fsd, &codes, CHIP_RATE, &dopp, nms, 2.5, F_1207, true);
    res.sort_by(|x, y| y.metric.partial_cmp(&x.metric).unwrap());
    println!(
        "{band} acquisition at {fc} Hz ({secs} s, {:.1} Msps decim {q}x, partial-band):",
        fsd / 1e6
    );
    for r in res.iter().take(10) {
        println!(
            "  PRN {:2}  metric {:5.2}  dopp {:+7.0}  cp {:7.1}  {}",
            r.prn,
            r.metric,
            r.doppler,
            r.code_phase,
            if r.metric > 2.5 { "<== ACQUIRED" } else { "" }
        );
    }
    println!(
        "=> {} acquired",
        res.iter().filter(|r| r.metric > 2.5).count()
    );
}
