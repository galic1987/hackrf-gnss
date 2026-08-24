//! Identify which TLE sat matches a measured Doppler at an arbitrary frequency.
//! usage: sat_ident <tle> <lat> <lon> <epoch_unix> <freq_hz> <meas_dopp_hz>
use hackrf_gnss::gps::{geodetic_to_ecef, load_tle_named, predict_doppler_el_f};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let tle = std::fs::read_to_string(&a[1]).unwrap();
    let lat: f64 = a[2].parse().unwrap();
    let lon: f64 = a[3].parse().unwrap();
    let t: f64 = a[4].parse().unwrap();
    let f: f64 = a[5].parse().unwrap();
    let meas: f64 = a[6].parse().unwrap();
    let sats = load_tle_named(&tle);
    let rx = geodetic_to_ecef(lat, lon, 0.0);
    let mut rows: Vec<(String, f64, f64, f64)> = Vec::new();
    for s in &sats {
        if let Some((dopp, el)) = predict_doppler_el_f(s, rx, t, f) {
            if el > 5.0 {
                rows.push((s.name.clone(), dopp, el, (dopp - meas).abs()));
            }
        }
    }
    rows.sort_by(|x, y| x.3.partial_cmp(&y.3).unwrap());
    println!("{:.3} MHz @ t={:.0}, measured {meas:+.0} Hz — candidates:", f / 1e6, t);
    for (name, dopp, el, err) in rows.iter().take(4) {
        println!("  {name:26} pred {dopp:+8.0} Hz  el {el:5.1}°  |err| {err:6.0} Hz");
    }
}
