use hackrf_gnss::gps::pvt::{solve, Meas};
fn norm(a:[f64;3])->f64{(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]).sqrt()}
fn main(){
    let station=[1351.991,-4653.584,4133.012];
    let sats=[[5869.975,-16762.018,19215.021],[-3107.678,-19845.218,17310.754],[13935.468,-7948.651,20949.401],[12385.868,-23003.053,2879.614],[21723.378,-9762.873,11751.846],[-11644.487,-10338.459,21397.985]];
    let m:Vec<Meas>=sats.iter().map(|&s|{let g=norm([station[0]-s[0],station[1]-s[1],station[2]-s[2]]);Meas{sat:s,pseudorange:g+50.0,clock_free:false}}).collect();
    let f=solve(&m,[0.0,0.0,0.0]).unwrap();
    let err=norm([f.ecef[0]-station[0],f.ecef[1]-station[1],f.ecef[2]-station[2]])*1000.0;
    println!("solved lat {:.5} lon {:.5} alt {:.1} m",f.lat,f.lon,f.alt_km*1000.0);
    println!("position error {:.4} m, clock {:.3} km, GDOP {:.2} PDOP {:.2} TDOP {:.2}, {} sats, {} iters",
        err,f.clock_km,f.gdop,f.pdop,f.tdop,f.n_sat,f.iterations);
}
