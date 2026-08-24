//! Does gps::track_full slice valid nav bits off the fixture for PRN 26?
use hackrf_gnss::gps::acquire;
use hackrf_gnss::gps::lnav::find_subframes;
use hackrf_gnss::gps::track::track_full;
use num_complex::Complex;
const FS: f64 = 4.0e6;
fn main() {
    use std::io::Read;
    let mut f = std::fs::File::open(
        "/Volumes/Radiator 8TB/gnss/hackrf_gnss/scripts/l1_30s_bb.f32").unwrap();
    let mut bytes = vec![0u8; (FS as usize) * 28 * 8];
    let n = f.read(&mut bytes).unwrap();
    bytes.truncate(n);
    let sig: Vec<Complex<f32>> = bytes.chunks_exact(8).map(|c| Complex::new(
        f32::from_le_bytes([c[0],c[1],c[2],c[3]]), f32::from_le_bytes([c[4],c[5],c[6],c[7]]))).collect();
    // acquire PRN 26 to get dopp/code phase
    let dopp: Vec<f64> = (-5000..=5000).step_by(250).map(|x| x as f64).collect();
    let res = acquire(&sig[..(4.0*FS) as usize], FS, &[26usize], &dopp, 4000, 2.5);
    let r = res.into_iter().find(|r| r.acquired).expect("PRN 26 should acquire");
    println!("acquired: dopp {:.1} cp {:.1}", r.doppler, r.code_phase);
    for dd in (-6..=6).step_by(2) {
        let d = r.doppler + dd as f64 * 5.0;
        let (tr, bits) = track_full(&sig, FS, 26, d, (1023.0 - r.code_phase) % 1023.0, 26000);
        let subs = find_subframes(&bits);
        println!("dopp {:>7.1}: sync={} score={:.2} bits={} subs={}", d, tr.bit_sync, tr.bit_sync_score, bits.len(), subs.len());
    }
}
