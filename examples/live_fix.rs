//! Live GPS position fix from the 1 Hz tracker state + fresh BRDC ephemeris.
//!
//! Reads observations/state.tracker.json (the live tracker's per-PRN code
//! phases), pairs them with a RINEX nav file (BRDC00WRD_R from BKG, fetched
//! daily by position_producer.py), and runs the coarse-time snapshot solver.
//! Writes observations/state.position.json (the /sync server merges it).
//!
//! usage: live_fix [rinex_path]   — one solve per invocation.

use hackrf_gnss::beidou_d1::{parse_rinex_bds, sat_at_txtime_bds};
use hackrf_gnss::gps::broadcast::parse_rinex_gps;
use hackrf_gnss::gps::snapshot::{snapshot_fix, Obs};

const TRACKER_STATE: &str = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json";
const OUT: &str = "/Volumes/Radiator 8TB/gnss/observations/state.position.json";
const RINEX: &str = "/Volumes/Radiator 8TB/gnss/observations/brdc_latest.rnx";
// surveyed site (NYC), matches --approx-lat/lon defaults in main.rs
const APPROX_LLA: [f64; 3] = [40.65, -73.80, 20.0];
const GPS_UNIX_EPOCH: f64 = 315_964_800.0;
const LEAP_S: f64 = 18.0;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let rinex_path = a.get(1).map(|s| s.as_str()).unwrap_or(RINEX);

    let mut ephs = match std::fs::read_to_string(rinex_path) {
        Ok(t) => parse_rinex_gps(&t),
        Err(_) => Default::default(), // self-decoded may still cover the sky
    };
    // BeiDou records of the same mixed RINEX file (BDS ICD constants; toe/toc
    // stored as GPST-equivalent SOW — see beidou_d1.rs)
    let mut bds_ephs = match std::fs::read_to_string(rinex_path) {
        Ok(t) => parse_rinex_bds(&t),
        Err(_) => Default::default(),
    };
    // self-decoded ephemerides from the live tracker take precedence (they
    // are fresher than any daily download and need no network). sys: 0 = GPS,
    // 1 = BeiDou (absent in older files -> GPS).
    let mut n_self = 0;
    if let Ok(t) = std::fs::read_to_string(
        "/Volumes/Radiator 8TB/gnss/observations/tracker_eph.json",
    ) {
        if let Ok(v) = serde_json::from_str::<serde_json::Value>(&t) {
            for e in v["ephemeris"].as_array().into_iter().flatten() {
                if let Ok(eph) = serde_json::from_value::<
                    hackrf_gnss::gps::broadcast::BrdcEph,
                >(e.clone())
                {
                    if eph.sys == 1 {
                        bds_ephs.insert(eph.prn, eph);
                    } else {
                        ephs.insert(eph.prn, eph);
                    }
                    n_self += 1;
                }
            }
        }
    }
    if n_self > 0 {
        eprintln!("live_fix: {n_self} self-decoded ephemerides in use");
    }

    let text = match std::fs::read_to_string(TRACKER_STATE) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("live_fix: cannot read tracker state: {e}");
            std::process::exit(2);
        }
    };
    let v: serde_json::Value = serde_json::from_str(&text).unwrap();
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();

    let mut obs = Vec::new();
    for s in v["tracker"]["sats"].as_array().into_iter().flatten() {
        if s["sys"].as_str() != Some("gps") {
            continue;
        }
        // skip stale channels
        if now - s["epoch"].as_f64().unwrap_or(0.0) > 10.0 {
            continue;
        }
        // only mature locks: a mid-pull-in channel's code phase can sit
        // tens of chips off (observed: 45 km RMS live vs 3 km on settled
        // replay data), and with 4-6 sats one outlier wrecks the fix
        if s["cn0_proxy"].as_f64().unwrap_or(0.0) < 30.0
            || s["lock_s"].as_f64().unwrap_or(0.0) < 10.0
        {
            continue;
        }
        let (Some(prn), Some(cp), Some(dopp)) = (
            s["prn"].as_u64().map(|p| p as u8),
            s["code_phase"].as_f64(),
            s["doppler_hz"].as_f64(),
        ) else {
            continue;
        };
        obs.push(Obs { prn, code_phase: cp, doppler: dopp });
    }
    let gps_t = now - GPS_UNIX_EPOCH + LEAP_S;
    let tow = gps_t % 604_800.0;

    // Preferred path: true pseudoranges from TOW-anchored channels (LNAV
    // subframes decoded live) and SOW-anchored BeiDou channels (D1 decoded
    // live; t_tx already in GPST). Falls back to the coarse snapshot below
    // when fewer than 4 channels are anchored.
    let mut gps_meas = Vec::new();
    let mut bds_meas = Vec::new();
    for s in v["tracker"]["sats"].as_array().into_iter().flatten() {
        if now - s["epoch"].as_f64().unwrap_or(0.0) > 10.0 {
            continue;
        }
        let (Some(prn), Some(rho_m), Some(t_tx)) = (
            s["prn"].as_u64().map(|p| p as u8),
            s["rho_m"].as_f64(),
            s["t_tx"].as_f64(),
        ) else {
            continue;
        };
        match s["sys"].as_str() {
            Some("gps") => {
                let Some(eph) = ephs.get(&prn) else { continue };
                let (sat_m, dt_sv, _) =
                    hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, t_tx, [0.0, 0.0, 0.0]);
                gps_meas.push(hackrf_gnss::gps::pvt::Meas {
                    sat: [sat_m[0] / 1000.0, sat_m[1] / 1000.0, sat_m[2] / 1000.0],
                    pseudorange: rho_m / 1000.0 + dt_sv * 299_792.458, // remove sat clock
                    clock_free: false,
                });
            }
            Some("beidou") => {
                let Some(eph) = bds_ephs.get(&prn) else { continue };
                let (sat_m, dt_sv, _) = sat_at_txtime_bds(eph, t_tx, [0.0, 0.0, 0.0]);
                bds_meas.push(hackrf_gnss::gps::pvt::Meas {
                    sat: [sat_m[0] / 1000.0, sat_m[1] / 1000.0, sat_m[2] / 1000.0],
                    pseudorange: rho_m / 1000.0 + dt_sv * 299_792.458,
                    clock_free: false,
                });
            }
            _ => {}
        }
    }

    // Mixed-constellation two-clock solve: >=3 GPS + >=2 BDS anchored rows
    // solve x,y,z,dt_gps,dt_bds. The inter-system clock offset (isx_km)
    // doubles as a spoof-detection observable.
    let g = hackrf_gnss::gps::ephemeris::geodetic_to_ecef(
        APPROX_LLA[0], APPROX_LLA[1], APPROX_LLA[2] / 1000.0,
    );
    if gps_meas.len() >= 3 && bds_meas.len() >= 2 {
        let mut rows: Vec<hackrf_gnss::gps::pvt::MeasSys> = gps_meas
            .iter()
            .map(|m| hackrf_gnss::gps::pvt::MeasSys {
                sat: m.sat,
                pseudorange: m.pseudorange,
                system: 0,
            })
            .chain(bds_meas.iter().map(|m| hackrf_gnss::gps::pvt::MeasSys {
                sat: m.sat,
                pseudorange: m.pseudorange,
                system: 1,
            }))
            .collect();
        // normalize: pseudoranges carry the huge stream-time-vs-GPS offset
        // (~1e10 km) which wrecks the solver's conditioning; the two clock
        // terms absorb any per-system common shift
        let mut sorted: Vec<f64> = rows.iter().map(|m| m.pseudorange).collect();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let med = sorted[sorted.len() / 2];
        for m in rows.iter_mut() {
            m.pseudorange -= med;
        }
        if let Some(f) = hackrf_gnss::gps::pvt::solve_mixed(&rows, g) {
            if f.residual_rms_m > 2000.0 {
                eprintln!(
                    "live_fix: mixed fix rms {:.0} m — too coarse to publish ({} gps + {} bds)",
                    f.residual_rms_m, f.n_gps, f.n_bds
                );
                std::process::exit(5);
            }
            println!(
                "PVT(anchored,3D(mixed GPS+BDS)): {:.6} {:.6} h {:.0} m | {} gps + {} bds, rms {:.1} m, gdop {:.1}, isx {:.2} km",
                f.lat, f.lon, f.alt_km * 1000.0, f.n_gps, f.n_bds, f.residual_rms_m, f.gdop, f.isx_km
            );
            let doc = serde_json::json!({
                "epoch": now,
                "ttl_s": 900,
                "position": {
                    "lat": f.lat, "lon": f.lon, "alt_km": f.alt_km,
                    "clock_km": f.clock_gps_km, "isx_km": f.isx_km,
                    "residual_rms_m": f.residual_rms_m,
                    "gdop": f.gdop, "n_sat": f.n_sat, "mode": "3D(mixed GPS+BDS)",
                    "source": "live TOW/SOW-anchored pseudoranges + self-decoded/BRDC ephemeris",
                }
            });
            let tmp = format!("{OUT}.tmp");
            std::fs::write(&tmp, doc.to_string()).unwrap();
            std::fs::rename(&tmp, OUT).unwrap();
            return;
        }
    }

    let meas = gps_meas;
    if meas.len() >= 3 {
        // normalize: pseudoranges carry the huge stream-time-vs-GPS offset
        // (~1e10 km) which wrecks the solver's conditioning; the clock term
        // absorbs any common shift
        let mut sorted: Vec<f64> = meas.iter().map(|m| m.pseudorange).collect();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let med = sorted[sorted.len() / 2];
        let mut meas: Vec<_> = meas
            .into_iter()
            .map(|mut m| {
                m.pseudorange -= med;
                m
            })
            .collect();
        let g = hackrf_gnss::gps::ephemeris::geodetic_to_ecef(
            APPROX_LLA[0], APPROX_LLA[1], APPROX_LLA[2] / 1000.0,
        );
        let mut mode = "3D";
        if meas.len() == 3 {
            // 3 sats: altitude-hold pseudo-measurement. Range to Earth's
            // centre = the EXACT geocentric radius of the surveyed site
            // (the 6371 km mean sphere would bake in ~1.3 km of bias at
            // this latitude). The row carries no receiver clock — solved
            // with a zero clock coefficient (clock_free), otherwise clock
            // and altitude trade freely along this row.
            let r0 = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
            meas.push(hackrf_gnss::gps::pvt::Meas {
                sat: [0.0, 0.0, 0.0],
                pseudorange: r0,
                clock_free: true,
            });
            mode = "2D(alt-hold)";
        }
        if let Some(f) = hackrf_gnss::gps::pvt::solve(&meas, g) {
            if f.residual_rms_m > 2000.0 {
                // same publish gate as the snapshot path: a fix this loose
                // is meaningless — keep showing the previous good fix
                eprintln!(
                    "live_fix: anchored fix rms {:.0} m — too coarse to publish ({} sats, {mode})",
                    f.residual_rms_m, f.n_sat
                );
                std::process::exit(5);
            }
            println!(
                "PVT(anchored,{mode}): {:.6} {:.6} h {:.0} m | {} sats, rms {:.1} m, gdop {:.1}",
                f.lat, f.lon, f.alt_km * 1000.0, f.n_sat, f.residual_rms_m, f.gdop
            );
            let doc = serde_json::json!({
                "epoch": now,
                "ttl_s": 900,
                "position": {
                    "lat": f.lat, "lon": f.lon, "alt_km": f.alt_km,
                    "clock_km": f.clock_km, "residual_rms_m": f.residual_rms_m,
                    "gdop": f.gdop, "n_sat": f.n_sat, "mode": mode,
                    "source": "live TOW-anchored pseudoranges + self-decoded/BRDC ephemeris",
                }
            });
            let tmp = format!("{OUT}.tmp");
            std::fs::write(&tmp, doc.to_string()).unwrap();
            std::fs::rename(&tmp, OUT).unwrap();
            return;
        }
    }
    if obs.len() < 4 {
        eprintln!("live_fix: only {} fresh GPS channels — need 4", obs.len());
        std::process::exit(3);
    }
    match snapshot_fix(&obs, &ephs, APPROX_LLA, tow) {
        Some(f) => {
            if f.residual_rms_m > 2000.0 {
                // a coarse-snapshot fix this loose is meaningless — don't
                // publish (the panel keeps showing the previous good fix)
                eprintln!(
                    "live_fix: fix rms {:.0} m — too coarse to publish ({} sats)",
                    f.residual_rms_m, f.n_sat
                );
                std::process::exit(5);
            }
            let doc = serde_json::json!({
                "epoch": now,
                "ttl_s": 900,
                "position": {
                    "lat": f.lat,
                    "lon": f.lon,
                    "alt_km": f.alt_km,
                    "clock_km": f.clock_km,
                    "residual_rms_m": f.residual_rms_m,
                    "gdop": f.gdop,
                    "n_sat": f.n_sat,
                    "source": "live tracker code phases + BRDC ephemeris",
                }
            });
            let tmp = format!("{OUT}.tmp");
            std::fs::write(&tmp, doc.to_string()).unwrap();
            std::fs::rename(&tmp, OUT).unwrap();
            println!(
                "fix: {:.5} {:.5} h {:.0} m | {} sats, rms {:.1} m, gdop {:.1}",
                f.lat, f.lon, f.alt_km * 1000.0, f.n_sat, f.residual_rms_m, f.gdop
            );
        }
        None => {
            eprintln!("live_fix: no converged fix");
            std::process::exit(4);
        }
    }
}
