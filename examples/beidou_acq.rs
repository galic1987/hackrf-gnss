//! BeiDou B1I acquisition on a real capture. usage: beidou_acq <iq> <fs> <fc> [secs]
use hackrf_gnss::gps::acquire_codes;
use hackrf_gnss::dsp_calib::generate_beidou_b1_code;
use hackrf_gnss::iridium::demod3::decimate;
use num_complex::Complex;
use std::f64::consts::PI;
use std::fs::File; use std::io::Read;
const B1I:f64=1561.098e6;
fn main(){
    let a:Vec<String>=std::env::args().collect();
    let fs:f64=a[2].parse().unwrap(); let fc:f64=a[3].parse().unwrap();
    let secs:f64=a.get(4).and_then(|s|s.parse().ok()).unwrap_or(6.0);
    let mut f=File::open(&a[1]).expect("open"); let mut raw=vec![0u8;(2.0*secs*fs) as usize*2];
    let n=f.read(&mut raw).unwrap(); raw.truncate(n);
    let mut sig:Vec<Complex<f32>>=raw.chunks_exact(2).map(|c|Complex::new(c[0] as i8 as f32,c[1] as i8 as f32)).collect();
    let mean:Complex<f32>=sig.iter().copied().sum::<Complex<f32>>()/sig.len() as f32;
    let ifhz=B1I-fc;
    for (k,v) in sig.iter_mut().enumerate(){*v-=mean;let ph=-2.0*PI*ifhz*k as f64/fs;*v*=Complex::new(ph.cos() as f32,ph.sin() as f32);}
    // decimate to ~5 Msps to keep the +-2.046 MHz lobe while speeding up
    let q=(fs/5.0e6).round() as usize;
    let (sig,fsd)= if q>1 {(decimate(&sig,q), fs/q as f64)} else {(sig,fs)};
    let codes:Vec<(usize,Vec<f32>)>=(1..=37).map(|p|(p,generate_beidou_b1_code(p))).collect();
    let dop:Vec<f64>=(-6000..=6000).step_by(400).map(|x|x as f64).collect();
    let nb=(secs*1000.0) as usize;
    let mut res=acquire_codes(&sig,fsd,&codes,2.046e6,&dop,nb,2.5,B1I,true);
    res.sort_by(|a,b|b.metric.partial_cmp(&a.metric).unwrap());
    println!("BeiDou B1I: {:.1}s, {:.1} Msps (decim {}x), {} blocks:",secs,fsd/1e6,q,nb);
    for r in res.iter().take(8){println!("  PRN {:2}  metric {:5.2}  dopp {:+6.0}  {}",r.prn,r.metric,r.doppler,if r.metric>2.5{"<== ACQUIRED"}else{""});}
    println!("{} acquired",res.iter().filter(|r|r.metric>2.5).count());
}
