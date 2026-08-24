//! End-to-end live Iridium decode: find bursts in a capture, demodulate each,
//! print RWA lines (one per frame). Validated against demod3.run_file.
//! usage: iridium_decode <capture.iq> <dur_s> [fs_hz] [bits]
use hackrf_gnss::iridium::demod3;
use std::fs::File;
use std::io::Read;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let dur: f64 = a[2].parse().unwrap();
    let (fc, fs) = (1626.25e6, if a.len() > 3 { a[3].parse().unwrap() } else { 4.0e6 });
    let mut f = File::open(&a[1]).unwrap();
    let bits: usize = if a.len() > 4 { a[4].parse().unwrap() } else { 8 };
    let want = ((dur + 4.2) * fs) as usize * 2 * (bits / 8);
    let mut raw_u8 = vec![0u8; want];
    let got = f.read(&mut raw_u8).unwrap_or(0);
    raw_u8.truncate(got);
    let lines = if bits == 16 {
        let raw_i16: Vec<i16> = raw_u8.chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]])).collect();
        demod3::run_capture(&raw_i16, fc, fs, dur, 1626.0e6, 1626.5e6, 4.0)
    } else {
        let raw_i8: Vec<i8> = raw_u8.iter().map(|&b| b as i8).collect();
        demod3::run_capture(&raw_i8, fc, fs, dur, 1626.0e6, 1626.5e6, 4.0)
    };
    for l in &lines { println!("{}", l); }
    eprintln!("{} frames", lines.len());
}
