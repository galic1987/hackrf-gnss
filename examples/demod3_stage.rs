//! Run the Rust demod3 chain on a real snippet and print stage scalars + RWA as
//! JSON, for validation against the Python oracle (scripts/demod3_oracle.py).
//!
//! usage: demod3_stage <snippet.iq> <fcen_hz> [z2_out.f32]

use hackrf_gnss::iridium::demod3;
use num_complex::Complex;
use std::fs;

fn dump_c(path: &str, z: &[Complex<f32>]) {
    let mut out = Vec::with_capacity(z.len() * 8);
    for c in z {
        out.extend_from_slice(&c.re.to_le_bytes());
        out.extend_from_slice(&c.im.to_le_bytes());
    }
    fs::write(path, out).unwrap();
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 3 {
        eprintln!("usage: demod3_stage <snippet.iq> <fcen_hz> [z2_out.f32]");
        std::process::exit(2);
    }
    let raw_u8 = fs::read(&a[1]).expect("read snippet");
    let raw_i8: Vec<i8> = raw_u8.iter().map(|&b| b as i8).collect();
    let fcen: f64 = a[2].parse().unwrap();
    let (fc, fs) = (1626.25e6, 4.0e6);

    let d = demod3::demod_snippet_debug(&raw_i8, fcen, fc, fs).expect("demod");
    if a.len() >= 4 {
        dump_c(&a[3], &d.z2);
    }
    let ds: String = d.ds.iter().map(|s| char::from(b'0' + s)).collect();
    let rwa_bits = d
        .rwa
        .as_ref()
        .and_then(|l| l.split_whitespace().last())
        .unwrap_or("");
    println!(
        "{{\"i0\":{},\"i1\":{},\"df\":{:.3},\"frac\":{:.4},\"hint\":{},\"conf\":{},\
         \"uw_pos\":{},\"n_sym\":{},\"n_z2\":{},\"ds\":\"{}\",\"rwa_bits\":\"{}\"}}",
        d.i0, d.i1, d.df, d.frac, d.hint, d.conf,
        d.uw_pos.map(|x| x as i64).unwrap_or(-1),
        d.ds.len(), d.z2.len(), ds, rwa_bits
    );
}
