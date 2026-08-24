//! Detect Iridium bursts in a capture and print them as JSON, for validation
//! against the Python oracle (validation/demod3.py:find_bursts2).
//!
//! usage: find_bursts <capture.iq> <dur_s>   (fc=1626.25 MHz, fs=4 Msps, band 1626.0-1626.5)

use hackrf_gnss::iridium::demod3;
use std::fs::File;
use std::io::Read;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() != 3 {
        eprintln!("usage: find_bursts <capture.iq> <dur_s>");
        std::process::exit(2);
    }
    let dur: f64 = a[2].parse().unwrap();
    let (fc, fs) = (1626.25e6, 4.0e6);
    // read only what the requested duration needs (plus a chunk of slack)
    let want = ((dur + 4.2) * fs) as usize * 2;
    let mut f = File::open(&a[1]).expect("open capture");
    let mut raw_u8 = vec![0u8; want];
    let got = f.read(&mut raw_u8).unwrap_or(0);
    raw_u8.truncate(got);
    let raw_i8: Vec<i8> = raw_u8.iter().map(|&b| b as i8).collect();

    let bursts = demod3::find_bursts2(&raw_i8, fc, fs, dur, 1626.0e6, 1626.5e6, 4.0);
    println!("{}", serde_json::to_string(&bursts).unwrap());
    eprintln!("{} bursts", bursts.len());
}
