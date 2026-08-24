//! Acquire one GPS PRN with code-Doppler compensation off vs on at 2s & 4s.
//! usage: acq_one <baseband.f32> <fs> <prn>
use hackrf_gnss::gps::{acquire_codes, ca_code::gps_ca, ca_code::CHIP_RATE, F_L1};
use num_complex::Complex;
use std::fs;
fn main(){
    let a:Vec<String>=std::env::args().collect();
    let bytes=fs::read(&a[1]).unwrap(); let fs:f64=a[2].parse().unwrap(); let prn:usize=a[3].parse().unwrap();
    let sig:Vec<Complex<f32>>=bytes.chunks_exact(8).map(|c|Complex::new(f32::from_le_bytes([c[0],c[1],c[2],c[3]]),f32::from_le_bytes([c[4],c[5],c[6],c[7]]))).collect();
    let dopp:Vec<f64>=(-3000..=3000).step_by(100).map(|x|x as f64).collect();
    let codes=vec![(prn,gps_ca(prn))];
    for nms in [2000usize,4000]{
        let off=acquire_codes(&sig,fs,&codes,CHIP_RATE,&dopp,nms,2.5,F_L1,false);
        let on =acquire_codes(&sig,fs,&codes,CHIP_RATE,&dopp,nms,2.5,F_L1,true);
        println!("PRN {} @ {} ms: comp OFF {:.2}, comp ON {:.2} (dopp {:+.0}) {}",prn,nms,off[0].metric,on[0].metric,on[0].doppler, if on[0].metric>2.5{"<== ACQUIRED"}else{""});
    }
}
