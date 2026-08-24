//! Acquire all 32 GPS PRNs with code-Doppler compensation. usage: acq_all <bb.f32> <fs> <nms>
use hackrf_gnss::gps::{acquire, AcqResult};
use num_complex::Complex;
use std::fs;
fn main(){
    let a:Vec<String>=std::env::args().collect();
    let bytes=fs::read(&a[1]).unwrap(); let fs:f64=a[2].parse().unwrap(); let nms:usize=a[3].parse().unwrap();
    let sig:Vec<Complex<f32>>=bytes.chunks_exact(8).map(|c|Complex::new(f32::from_le_bytes([c[0],c[1],c[2],c[3]]),f32::from_le_bytes([c[4],c[5],c[6],c[7]]))).collect();
    let dopp:Vec<f64>=(-18000..=18000).step_by(200).map(|x|x as f64).collect();
    let prns:Vec<usize>=(1..=32).collect();
    let mut res:Vec<AcqResult>=acquire(&sig,fs,&prns,&dopp,nms,2.5); // comp ON (acquire uses it)
    res.sort_by(|a,b|b.metric.partial_cmp(&a.metric).unwrap());
    println!("GPS acquisition (code-Doppler compensated, {} ms):",nms);
    for r in res.iter().take(12){println!("  PRN {:2}  metric {:5.2}  dopp {:+7.0}  cp {:6.1}  {}",r.prn,r.metric,r.doppler,r.code_phase,if r.metric>2.5{"<== ACQUIRED"}else{""});}
    println!("=> {} satellites acquired (>2.5)",res.iter().filter(|r|r.metric>2.5).count());
}
