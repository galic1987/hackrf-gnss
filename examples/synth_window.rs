//! Emulate tonight's capture window synthetically with the REAL predicted
//! geometry: propagate the actual TLE for the 8 satellites of pass window #1
//! (Aug 23 00:14:52 EDT, unix 1787458479, 20 min), synthesize bursts at the
//! observed indoor cadence with the noise/bias/drift levels measured on real
//! captures, and run the production solver over the knob sweep that decides
//! tonight's capture length.
//!
//! usage: synth_window [tle_path]   (default ../observations/iridium_today.tle)
use hackrf_gnss::gps::{geodetic_to_ecef, load_tle_named, predict_doppler_el_f, GpsSat};
use hackrf_gnss::iridium::geo::{solve_fix, Obs};

const RX_LAT: f64 = 39.001;
const RX_LON: f64 = -77.60732;
const T0: f64 = 1787458479.0; // window #1 start (unix)
const WINDOW_S: f64 = 20.0 * 60.0;
const F_NOM: f64 = 1626.270833e6;
const E_TRUE: f64 = -0.9e-6; // current disciplined clock state
const SATS: [&str; 8] = ["IRIDIUM 117", "IRIDIUM 168", "IRIDIUM 170", "IRIDIUM 174",
                         "IRIDIUM 176", "IRIDIUM 180", "IRIDIUM 29", "IRIDIUM 57"];
// strongest three arcs by predicted max elevation (from pass_windows)
const STRONG3: [&str; 3] = ["IRIDIUM 29", "IRIDIUM 57", "IRIDIUM 170"];

/// Deterministic per-satellite pseudo-random in [-1, 1] (name-seeded).
fn sat_rand(name: &str, salt: u64) -> f64 {
    let mut x = salt ^ 0x9E3779B97F4A7C15;
    for b in name.bytes() {
        x = (x ^ b as u64).wrapping_mul(0x100000001B3);
    }
    x ^= x >> 33;
    (x % (1u64 << 31)) as f64 / (1u64 << 30) as f64 - 1.0
}

struct Noise(u64);
impl Noise {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 33) as f64 / (1u64 << 31) as f64) - 1.0
    }
}

/// Synthesize one capture's observations: every 20 s per satellite while
/// el > 25 deg, true station, with obs noise, per-sat constant bias and drift
/// drawn at the levels measured on real captures (+-1.5 kHz, +-80 Hz/s).
fn synth<'a>(sats: &[&'a GpsSat], dur_s: f64, noise_hz: f64, rx: [f64; 3]) -> Vec<Obs<'a>> {
    let mut out = Vec::new();
    let mut nz = Noise(0xC0FFEE);
    for s in sats {
        let bias = sat_rand(&s.name, 1) * 1500.0;
        let drift = sat_rand(&s.name, 2) * 80.0;
        // collect this sat's obs times first for its drift reference
        let mut ts = Vec::new();
        let mut t = T0;
        while t < T0 + dur_s {
            if let Some((_, el)) = predict_doppler_el_f(s, rx, t, F_NOM) {
                if el > 25.0 {
                    ts.push(t);
                }
            }
            t += 20.0;
        }
        let t_ref = ts.iter().sum::<f64>() / ts.len().max(1) as f64;
        for t in ts {
            let (fd, _) = predict_doppler_el_f(s, rx, t, F_NOM).unwrap();
            out.push(Obs {
                t,
                f_meas: (F_NOM + fd) * (1.0 + E_TRUE) + bias + drift * (t - t_ref) + noise_hz * nz.next(),
                f_nom: F_NOM,
                sat: s,
                conf: 90,
                w_scale: 0.5, // the realistic class: Doppler-attributed decoded
                cap: 0,
            });
        }
    }
    out
}

fn err_km(lat: f64, lon: f64) -> f64 {
    let a = geodetic_to_ecef(lat, lon, 0.0);
    let b = geodetic_to_ecef(RX_LAT, RX_LON, 0.0);
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn run(tag: &str, sats: &[&GpsSat], dur_s: f64, noise_hz: f64, guess: (f64, f64), rx: [f64; 3]) {
    let obs = synth(sats, dur_s, noise_hz, rx);
    match solve_fix(&obs, guess.0, guess.1, &[0.0], Some(2.0e3), Some(100.0)) {
        Ok(fix) => {
            println!(
                "{:<28} n={:<4} err {:7.2} km  sigma {:6.2} km  rms {:5.1} Hz  clock {:+.2} ppm",
                tag,
                fix.n_used,
                err_km(fix.lat_deg, fix.lon_deg),
                fix.sigma_km,
                fix.rms_hz,
                fix.clock_ppm[0]
            );
        }
        Err(m) => println!("{:<28} n={:<4} REFUSED: {}", tag, obs.len(), m),
    }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let tle = a.get(1).map(|s| s.as_str()).unwrap_or("../observations/iridium_today.tle");
    let all = load_tle_named(&std::fs::read_to_string(tle).expect("read TLE"));
    let find = |n: &str| all.iter().find(|s| s.name == n).expect(n);
    let win: Vec<&GpsSat> = SATS.iter().map(|n| find(n)).collect();
    let strong: Vec<&GpsSat> = STRONG3.iter().map(|n| find(n)).collect();
    let rx = geodetic_to_ecef(RX_LAT, RX_LON, 0.0);

    println!("synthetic window #1: {} sats, t0 {} (Aug 23 00:14:52 EDT), true station ({}, {})",
             win.len(), T0 as u64, RX_LAT, RX_LON);
    println!("noise 20 Hz, per-sat bias +-1.5 kHz, drift +-80 Hz/s, clock {:+.1} ppm\n", E_TRUE * 1e6);
    println!("-- capture length sweep (all 8 sats)");
    for dur in [WINDOW_S, 8.0 * 60.0, 4.0 * 60.0] {
        run(&format!("full {:.0} min", dur / 60.0), &win, dur, 20.0, (RX_LAT, RX_LON), rx);
    }
    println!("-- yield / noise variants (full 20 min)");
    run("3 strongest arcs only", &strong, WINDOW_S, 20.0, (RX_LAT, RX_LON), rx);
    run("all 8, noise x2 (40 Hz)", &win, WINDOW_S, 40.0, (RX_LAT, RX_LON), rx);
    println!("-- mirror check (guess 500 km off)");
    run("all 8, guess +4.5 deg lat", &win, WINDOW_S, 20.0, (RX_LAT + 4.5, RX_LON), rx);
    run("all 8, guess -4.5 deg lat", &win, WINDOW_S, 20.0, (RX_LAT - 4.5, RX_LON), rx);

    // detail for the headline config
    let obs = synth(&win, WINDOW_S, 20.0, rx);
    let fix = solve_fix(&obs, RX_LAT, RX_LON, &[0.0], Some(2.0e3), Some(100.0)).unwrap();
    println!("\nheadline detail (full window, 20 Hz):");
    for (name, n, rms, bias, rate) in &fix.per_sat {
        println!("  {:<14} n={:<3} resid {:5.1} Hz  bias {:+6.0} (true {:+6.0})  drift {:+6.1} (true {:+5.1})",
                 name, n, rms, bias, sat_rand(name, 1) * 1500.0, rate, sat_rand(name, 2) * 80.0);
    }
}
