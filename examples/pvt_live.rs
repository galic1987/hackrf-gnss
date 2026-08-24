//! Snapshot PVT from today's own decodes: ephemeris sidecars (gps_tow --merge
//! JSON) + acquisition code phases -> receiver position. No prior needed.
//! usage: pvt_live <tow_s> <lat0> <lon0> <eph1.json> <eph2.json> ...
//!   code phases / dopplers are edited below per acquisition run (spike tool).
use hackrf_gnss::gps::lnav::{parse_ephemeris, Subframe};
use hackrf_gnss::gps::snapshot::{snapshot_fix, Obs};
use hackrf_gnss::gps::broadcast::BrdcEph;
use std::collections::HashMap;

#[derive(serde::Deserialize)]
struct SfJson {
    sfid: u8,
    tow_next: u32,
    #[serde(default)]
    bit_index: usize,
    words: Vec<[u8; 24]>,
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let tow: f64 = a[1].parse().unwrap();
    let approx: [f64; 3] = [a[2].parse().unwrap(), a[3].parse().unwrap(), 0.1];
    let mut ephs: HashMap<u8, BrdcEph> = HashMap::new();
    for path in &a[4..] {
        let sfs: Vec<SfJson> =
            serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let subs: Vec<Subframe> = sfs
            .into_iter()
            .map(|s| Subframe { sfid: s.sfid, tow_next: s.tow_next, bit_index: s.bit_index, words: s.words })
            .collect();
        // PRN from filename: eph_prnNN.json
        let prn: u8 = path
            .split("eph_prn").nth(1).unwrap()
            .trim_end_matches(".json").parse().unwrap();
        match parse_ephemeris(&subs) {
            Some(mut e) => { e.prn = prn; ephs.insert(prn, e); }
            None => println!("PRN {prn}: no ephemeris (need sfid 1,2,3)"),
        }
    }
    println!("ephemerides decoded for PRNs: {:?}", ephs.keys().collect::<Vec<_>>());

    // Slice C (capture+24 s) acquisition, code phases MIRRORED (1023 - acq),
    // dopplers from dop_refine. tow = GPS TOW at slice start.
    let obs = vec![
        Obs { prn: 10, code_phase: 1023.0 - 418.7, doppler: -4585.84 },
        Obs { prn: 26, code_phase: 1023.0 - 602.0, doppler: 2057.37 },
        Obs { prn: 32, code_phase: 1023.0 - 182.6, doppler: -2288.8 },
        Obs { prn: 1,  code_phase: 1023.0 - 300.3, doppler: -2784.38 },
        Obs { prn: 28, code_phase: 1023.0 - 398.5, doppler: -1905.27 },
    ];
    match snapshot_fix(&obs, &ephs, approx, tow) {
        Some(f) => println!(
            "FIX: lat {:.5} lon {:.5} alt {:.0} m | sats {} resid {:.1} m gdop {:.1} clock {:.3} km iters {}",
            f.lat, f.lon, f.alt_km * 1000.0, f.n_sat, f.residual_rms_m, f.gdop, f.clock_km, f.iterations
        ),
        None => println!("no fix"),
    }
}
