//! GLONASS L1OF FDMA acquisition: search each frequency channel with the single
//! 511-chip code. usage: glonass_acq <iq> <fs> <fc> [secs]
use hackrf_gnss::glonass::{glonass_code, l1_freq, CHIP_RATE};
use hackrf_gnss::gps::acquire_codes;
use hackrf_gnss::iridium::demod3::decimate;
use num_complex::Complex;
use std::f64::consts::PI;
use std::fs::File; use std::io::Read;
fn main(){
    let a:Vec<String>=std::env::args().collect();
    let fs:f64=a[2].parse().unwrap(); let fc:f64=a[3].parse().unwrap();
    let secs:f64=a.get(4).and_then(|s|s.parse().ok()).unwrap_or(2.0);
    let mut f=File::open(&a[1]).unwrap(); let mut raw=vec![0u8;(2.0*secs*fs) as usize*2];
    let n=f.read(&mut raw).unwrap(); raw.truncate(n);
    let base:Vec<Complex<f32>>=raw.chunks_exact(2).map(|c|Complex::new(c[0] as i8 as f32,c[1] as i8 as f32)).collect();
    let mean:Complex<f32>=base.iter().copied().sum::<Complex<f32>>()/base.len() as f32;
    let q=(fs/2.0e6).round() as usize;                 // decimate to ~2 Msps
    let code=glonass_code();
    let dop:Vec<f64>=(-10000..=10000).step_by(250).map(|x|x as f64).collect();
    let nb=(secs*1000.0) as usize;
    println!("GLONASS L1OF FDMA search, {:.1}s:",secs);
    let mut hits=0;
    for k in -7..=6 {
        let ifhz=l1_freq(k)-fc;
        let mut sig:Vec<Complex<f32>>=base.iter().enumerate().map(|(i,&v)|{
            let ph=-2.0*PI*ifhz*i as f64/fs;(v-mean)*Complex::new(ph.cos() as f32,ph.sin() as f32)}).collect();
        sig=decimate(&sig,q);
        let fsd=fs/q as f64;
        let r=acquire_codes(&sig,fsd,&[(0usize,code.clone())],CHIP_RATE,&dop,nb,2.5,l1_freq(k),true);
        let m=r[0].metric;
        if m>2.5 {hits+=1;}
        println!("  chan {:+2} ({:.4} MHz):  metric {:5.2}  dopp {:+6.0}  {}",k,l1_freq(k)/1e6,m,r[0].doppler,if m>2.5{"<== SATELLITE"}else{""});
    }
    println!("=> {} GLONASS channels with a satellite",hits);
}
