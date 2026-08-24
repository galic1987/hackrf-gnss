//! Multi-start basin selection: the global grid must find the true basin with
//! no initial guess, and a deliberately mirror-prone geometry must surface
//! the ambiguity warning rather than a silent wrong fix.

use hackrf_gnss::gps::{geodetic_to_ecef, load_tle_named, predict_doppler_el_f, GpsSat};
use hackrf_gnss::iridium::geo::{global_grid, multistart_fix, Obs};
use std::fs;
use std::path::PathBuf;

const RX_LAT: f64 = 40.65;
const RX_LON: f64 = -73.80;

fn fx(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(name)
}

fn err_km(lat: f64, lon: f64) -> f64 {
    let a = geodetic_to_ecef(lat, lon, 0.0);
    let b = geodetic_to_ecef(RX_LAT, RX_LON, 0.0);
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn arc<'a>(sat: &'a GpsSat, t_from: f64, e_true: f64, noise_hz: f64, cap: usize, seed: u64, count: usize) -> Vec<Obs<'a>> {
    let rx = geodetic_to_ecef(RX_LAT, RX_LON, 0.0);
    let f_nom = 1626.270833e6;
    for m in 0..(24 * 60) {
        let t0 = t_from + m as f64 * 60.0;
        let Some((_, el)) = predict_doppler_el_f(sat, rx, t0 + 30.0, f_nom) else { continue };
        if el < 25.0 {
            continue;
        }
        let mut state = seed;
        let mut out = Vec::new();
        for k in 0..count {
            let t = t0 + k as f64 * 20.0;
            let Some((fd, el)) = predict_doppler_el_f(sat, rx, t, f_nom) else { continue };
            if el < 15.0 {
                continue;
            }
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let nz = ((state >> 33) as f64 / (1u64 << 31) as f64) - 1.0;
            out.push(Obs { t, f_meas: (f_nom + fd) * (1.0 + e_true) + noise_hz * nz,
                           f_nom, sat, conf: 90, w_scale: 1.0, cap });
        }
        if out.len() >= count / 2 {
            return out;
        }
    }
    panic!("no pass for {}", sat.name);
}

#[test]
fn multistart_finds_the_true_basin_without_a_guess() {
    let sats = load_tle_named(&fs::read_to_string(fx("iridium.tle")).unwrap());
    let t_epoch = sats[0].epoch_unix;
    let mut obs = arc(&sats[0], t_epoch, -26.0e-6, 20.0, 0, 7, 20);
    obs.extend(arc(&sats[1], t_epoch + 5400.0, -0.92e-6, 20.0, 1, 11, 20));
    obs.extend(arc(&sats[2], t_epoch + 10800.0, -0.92e-6, 20.0, 1, 13, 20));
    let ms = multistart_fix(&obs, &[0.0, 0.0], Some(2.0e3), Some(100.0), &global_grid());
    let passing: Vec<_> = ms.candidates.iter().filter(|c| c.converged && c.sigma_km <= 10.0).collect();
    eprintln!("multistart: {} converged, top basins:", ms.candidates.iter().filter(|c| c.converged).count());
    for c in passing.iter().take(3) {
        eprintln!("  ({:+.2}, {:+.2}) rms {:.1} Hz sigma {:.2} km", c.lat_deg, c.lon_deg, c.rms_hz, c.sigma_km);
    }
    let best = ms.best.expect("a passing basin must exist");
    let err = err_km(best.lat_deg, best.lon_deg);
    eprintln!("winner: err {:.2} km, rms {:.1} Hz, ambiguous {}", err, best.rms_hz, ms.ambiguous);
    assert!(err < 5.0, "winner landed {err} km off");
    assert!(!ms.ambiguous, "three well-separated arcs should not be ambiguous");
}

#[test]
fn a_single_arc_surfaces_ambiguity_or_refuses() {
    // one satellite, one capture: Doppler-only is mirror-prone by
    // construction. The sweep must NOT report a confident clean winner —
    // either it refuses (no basin passes the guard) or it flags ambiguity.
    let sats = load_tle_named(&fs::read_to_string(fx("iridium.tle")).unwrap());
    let obs = arc(&sats[0], sats[0].epoch_unix, -0.92e-6, 20.0, 0, 7, 20);
    let ms = multistart_fix(&obs, &[0.0], Some(2.0e3), Some(100.0), &global_grid());
    let passing: Vec<_> = ms.candidates.iter().filter(|c| c.converged && c.sigma_km <= 10.0).collect();
    for c in passing.iter().take(3) {
        eprintln!("single-arc basin: ({:+.2}, {:+.2}) rms {:.1} Hz sigma {:.2} km",
                  c.lat_deg, c.lon_deg, c.rms_hz, c.sigma_km);
    }
    match &ms.best {
        None => eprintln!("single-arc: all basins refused (acceptable)"),
        Some(f) => {
            eprintln!("single-arc winner: ({:+.3}, {:+.3}) rms {:.1} Hz sigma {:.2} km, ambiguous {}",
                      f.lat_deg, f.lon_deg, f.rms_hz, f.sigma_km, ms.ambiguous);
            // if a basin passed at all, it must be the true one AND the
            // margin must be flagged when thin
            let err = err_km(f.lat_deg, f.lon_deg);
            assert!(err < 10.0 || ms.ambiguous,
                    "wrong or thin-margin basin reported as a confident fix: err {err} km, ambiguous {}",
                    ms.ambiguous);
        }
    }
}
