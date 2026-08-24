use hackrf_gnss::gps::acquire;
use hackrf_gnss::live::{Band, Channel, Sys};
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
    let dopp: Vec<f64> = (-5000..=5000).step_by(250).map(|x| x as f64).collect();
    let res = acquire(&sig[..(4.0*FS) as usize], FS, &[26usize], &dopp, 4000, 2.5);
    let r = res.into_iter().find(|r| r.acquired).unwrap();
    let mut ch = Channel::new(Sys::Gps, 26, FS, r.doppler, r.code_phase);
    // refine like the seed does now
    let r1 = ch.debug_refine(&sig[..(2.0*FS) as usize]);
    println!("debug_refine -> {:.2}", r1);
    println!("refined dopp: {:.2}", ch.debug_dopp());
    let ns = (FS/1000.0) as usize;
    for sec in 0..24 {
        let start = (2.0 + sec as f64) * FS as f64;
        let seg = &sig[start as usize..(start as usize) + 1000*ns];
        ch.debug_epoch(seg);
        println!("sec {:>2}: carrier {:.2} Hz", sec, ch.debug_dopp());
    }
}
