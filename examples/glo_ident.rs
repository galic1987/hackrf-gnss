//! Identify which GLONASS sat occupies an FDMA channel: rank TLE sats by
//! |predicted Doppler - measured| at the channel frequency.
//! usage: glo_ident <tle> <lat> <lon> <epoch_unix> <chan_k> <meas_dopp_hz>
use hackrf_gnss::glonass::l1_freq;
use hackrf_gnss::gps::{geodetic_to_ecef, load_tle_named, predict_doppler_el_f};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let tle = std::fs::read_to_string(&a[1]).unwrap();
    let lat: f64 = a[2].parse().unwrap();
    let lon: f64 = a[3].parse().unwrap();
    let t: f64 = a[4].parse().unwrap();
    let k: i32 = a[5].parse().unwrap();
    let meas: f64 = a[6].parse().unwrap();
    let sats = load_tle_named(&tle);
    let rx = geodetic_to_ecef(lat, lon, 0.0);
    let f = l1_freq(k);
    let mut rows: Vec<(String, f64, f64, f64)> = Vec::new();
    for s in &sats {
        if let Some((dopp, el)) = predict_doppler_el_f(s, rx, t, f) {
            if el > 5.0 {
                rows.push((s.name.clone(), dopp, el, (dopp - meas).abs()));
            }
        }
    }
    rows.sort_by(|x, y| x.3.partial_cmp(&y.3).unwrap());
    println!("chan {k} ({:.4} MHz), measured dopp {meas:+.0} Hz — candidates:", f / 1e6);
    for (name, dopp, el, err) in rows.iter().take(5) {
        println!("  {name:24} pred {dopp:+8.0} Hz  el {el:5.1}°  |err| {err:6.0} Hz");
    }
}
