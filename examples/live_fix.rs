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
const APPROX_LLA: [f64; 3] = [39.0032, -77.6058, 20.0];
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
    //
    // Satellite positions must come from a RECEPTION-anchored light-time
    // solve at the approx site: sat_at_txtime's tow is a GPS reception
    // time, so calling it with the SV-clock t_tx (and a geocenter rx) used
    // to evaluate each satellite ~70-90 ms away from its true emission
    // point — a per-satellite range-rate * 80 ms artifact of up to ~80 m
    // that wandered the anchored fix around by hundreds of metres. The
    // approx-site anchor is fine: a 1 km site error changes tau by ~3 us,
    // i.e. millimetres of evaluated position.
    // Mobile station: the reference/guess is the latest GATED fix when one
    // is fresh, not a constant — the station is going in a car, and a stale
    // constant would bias every light-time anchor after a move. Falls back
    // to APPROX_LLA only on cold start.
    let dyn_lla: [f64; 3] = std::fs::read_to_string(OUT)
        .ok()
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        .and_then(|p| {
            let pos = &p["position"];
            let fresh = now - p["epoch"].as_f64().unwrap_or(0.0) < 900.0;
            let gated = pos["gate"].as_str().unwrap_or("") == "redundant";
            if fresh && gated {
                Some([
                    pos["lat"].as_f64()?,
                    pos["lon"].as_f64()?,
                    pos["alt_km"].as_f64()? * 1000.0,
                ])
            } else {
                None
            }
        })
        .unwrap_or(APPROX_LLA);
    let site_m = {
        let g = hackrf_gnss::gps::ephemeris::geodetic_to_ecef(
            dyn_lla[0], dyn_lla[1], dyn_lla[2] / 1000.0,
        );
        g.map(|x| x * 1000.0)
    };
    let mut gps_meas = Vec::new();
    let mut gps_prns: Vec<u8> = Vec::new();
    let mut bds_meas = Vec::new();
    // SBAS fast corrections (WAAS MT2-5), harvested from any streak-locked
    // SBAS channel's published fast_corr: GPS PRN -> PRC metres. DO-229
    // convention: the PRC is ADDED to the measured pseudorange.
    let mut sbas_prc: std::collections::HashMap<u8, f64> = std::collections::HashMap::new();
    let mut sbas_lt: std::collections::HashMap<u8, (f64, f64, f64, f64)> =
        std::collections::HashMap::new();
    for s in v["tracker"]["sats"].as_array().into_iter().flatten() {
        if s["sys"].as_str() != Some("sbas") {
            continue;
        }
        if !s["sbas_msgs"]["locked"].as_bool().unwrap_or(false) {
            continue;
        }
        for row in s["sbas_msgs"]["fast_corr"].as_array().into_iter().flatten() {
            if let (Some(prn), Some(prc)) = (row[0].as_u64(), row[1].as_f64()) {
                sbas_prc.insert(prn as u8, prc);
            }
        }
        // SBAS long-term corrections (MT24/25): GPS PRN -> (dx, dy, dz m,
        // daf0 s). Corrected sat position = broadcast + delta, corrected
        // sat clock offset = broadcast + daf0 (DO-229).
        for row in s["sbas_msgs"]["lt_corr"].as_array().into_iter().flatten() {
            if let (Some(prn), Some(dx), Some(dy), Some(dz), Some(daf0)) = (
                row[0].as_u64(),
                row[1].as_f64(),
                row[2].as_f64(),
                row[3].as_f64(),
                row[4].as_f64(),
            ) {
                sbas_lt.insert(prn as u8, (dx, dy, dz, daf0));
            }
        }
    }
    let mut n_sbas_corr = 0usize;
    let mut n_lt_corr = 0usize;
    // WAAS iono (MT18 masks + MT26 delays): (IGP lat, lon) deg -> vertical
    // delay m. An MT26 block is used only when its IODI matches the band's
    // current mask IODI (DO-229 consistency rule); 511 counts = not
    // monitored; GIVEI >= 15 = don't use.
    let mut igp_delay: std::collections::HashMap<(i16, i16), f64> =
        std::collections::HashMap::new();
    for s in v["tracker"]["sats"].as_array().into_iter().flatten() {
        if s["sys"].as_str() != Some("sbas") {
            continue;
        }
        if !s["sbas_msgs"]["locked"].as_bool().unwrap_or(false) {
            continue;
        }
        for mask in s["sbas_msgs"]["igp_mask"].as_array().into_iter().flatten() {
            let (Some(band), Some(miodi)) = (mask[0].as_u64(), mask[1].as_u64()) else {
                continue;
            };
            let Some(igps) = mask[2].as_array() else { continue };
            for dl in s["sbas_msgs"]["iono_delay"].as_array().into_iter().flatten() {
                let (Some(dband), Some(block), Some(diodi)) =
                    (dl[0].as_u64(), dl[1].as_u64(), dl[2].as_u64())
                else {
                    continue;
                };
                if dband != band || diodi != miodi {
                    continue;
                }
                let Some(rows) = dl[3].as_array() else { continue };
                for (i, row) in rows.iter().enumerate() {
                    let (Some(counts), Some(givei)) = (row[0].as_u64(), row[1].as_u64()) else {
                        continue;
                    };
                    if counts == 511 || givei >= 15 {
                        continue;
                    }
                    let j = block as usize * 15 + i;
                    if let Some(igp_num) = igps.get(j).and_then(|v| v.as_u64()) {
                        if let Some(coord) =
                            hackrf_gnss::sbas_iono::igp_latlon(band as u8, igp_num as u16)
                        {
                            igp_delay.insert(coord, counts as f64 * 0.125);
                        }
                    }
                }
            }
        }
    }
    let mut n_iono_corr = 0usize;
    for s in v["tracker"]["sats"].as_array().into_iter().flatten() {
        if now - s["epoch"].as_f64().unwrap_or(0.0) > 10.0 {
            continue;
        }
        // lock-quality gate, same bars as the snapshot path below: a
        // marginal channel's carrier loop can sit 60-280 Hz off its own
        // code comb while staying "locked" (measured live), and although
        // the anchor advance no longer uses that estimate, a channel that
        // marginal has no business in the fix
        if s["cn0_proxy"].as_f64().unwrap_or(0.0) < 30.0
            || s["lock_s"].as_f64().unwrap_or(0.0) < 10.0
        {
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
                let (_, dt0, _) =
                    hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, t_tx, site_m);
                let mut a = t_tx - dt0 + 0.075;
                let (mut sat_m, mut dt_sv) = ([0.0; 3], dt0);
                for _ in 0..2 {
                    let (s, d, r) =
                        hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, a, site_m);
                    sat_m = s;
                    dt_sv = d;
                    a = t_tx - d + r / 299_792_458.0;
                }
                let prc = sbas_prc.get(&prn).copied().unwrap_or(0.0);
                if prc != 0.0 {
                    n_sbas_corr += 1;
                }
                let (dx, dy, dz, daf0) = sbas_lt.get(&prn).copied().unwrap_or((0.0, 0.0, 0.0, 0.0));
                if dx != 0.0 || dy != 0.0 || dz != 0.0 || daf0 != 0.0 {
                    n_lt_corr += 1;
                }
                let sat_m = [sat_m[0] + dx, sat_m[1] + dy, sat_m[2] + dz];
                // WAAS iono slant delay: pierce point from the site to the
                // (LT-corrected) satellite, bilinear/triangle over the
                // live IGP grid, obliquity-scaled. The iono delays the
                // signal, so the correction is SUBTRACTED.
                let rel = [
                    sat_m[0] - site_m[0],
                    sat_m[1] - site_m[1],
                    sat_m[2] - site_m[2],
                ];
                let (az, el) = hackrf_gnss::sbas_iono::azel(
                    dyn_lla[0].to_radians(),
                    dyn_lla[1].to_radians(),
                    rel,
                );
                let mut iono_m = 0.0;
                if el > 0.0 && !igp_delay.is_empty() {
                    let ((plat, plon), fp) = hackrf_gnss::sbas_iono::ion_pierce_point(
                        (dyn_lla[0].to_radians(), dyn_lla[1].to_radians()),
                        az,
                        el,
                    );
                    if let Some(d) = hackrf_gnss::sbas_iono::iono_slant_delay(
                        plat.to_degrees(),
                        plon.to_degrees(),
                        fp,
                        &igp_delay,
                    ) {
                        iono_m = d;
                        n_iono_corr += 1;
                    }
                }
                gps_meas.push(hackrf_gnss::gps::pvt::Meas {
                    sat: [sat_m[0] / 1000.0, sat_m[1] / 1000.0, sat_m[2] / 1000.0],
                    pseudorange: (rho_m + prc - iono_m) / 1000.0 + (dt_sv + daf0) * 299_792.458, // SBAS PRC added, iono slant subtracted, sat clock + LT daf0 removed
                    clock_free: false,
                });
                gps_prns.push(prn);
            }
            Some("beidou") => {
                let Some(eph) = bds_ephs.get(&prn) else { continue };
                let (_, dt0, _) = sat_at_txtime_bds(eph, t_tx, site_m);
                let mut a = t_tx - dt0 + 0.075;
                let (mut sat_m, mut dt_sv) = ([0.0; 3], dt0);
                for _ in 0..2 {
                    let (s, d, r) = sat_at_txtime_bds(eph, a, site_m);
                    sat_m = s;
                    dt_sv = d;
                    a = t_tx - d + r / 299_792_458.0;
                }
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
        dyn_lla[0], dyn_lla[1], dyn_lla[2] / 1000.0,
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
        // pre-solve sanity: after median normalization, honest channels sit
        // within a few thousand km of the median; a garbage-frame anchor
        // (observed live: a 2.1e10 km residual class) poisons every residual
        // and exhausts the drop budget. Drop those rows BEFORE solving.
        rows.retain(|m| m.pseudorange.abs() < 5000.0);
        if let Some((f, dropped)) = hackrf_gnss::gps::pvt::solve_mixed_with_rejection(&rows, g, 3) {
            if !dropped.is_empty() {
                // rows = GPS first, then BDS; a dropped row is a >=1 km
                // outlier (a full 1 ms tooth slip is ~300 km)
                let names: Vec<String> = dropped
                    .iter()
                    .map(|&i| {
                        if i < gps_prns.len() {
                            format!("G{}", gps_prns[i])
                        } else {
                            format!("BDS-row{}", i - gps_prns.len())
                        }
                    })
                    .collect();
                eprintln!("live_fix: dropped outlier channels {:?} (>1 km residual)", names);
            }
            if f.residual_rms_m > 2000.0 {
                eprintln!(
                    "live_fix: mixed fix rms {:.0} m — too coarse to publish ({} gps + {} bds)",
                    f.residual_rms_m, f.n_gps, f.n_bds
                );
                std::process::exit(5);
            }
            // honesty gate: the mixed solve has 5 unknowns, so n_sat <= 5
            // is an EXACT solve — rms is zero by construction and the fix
            // can be arbitrarily wrong. Publish, but say so.
            let gate = if f.n_sat >= 6 {
                "redundant"
            } else {
                "ungated — exact solve, unverifiable"
            };
            println!(
                "PVT(anchored,3D(mixed GPS+BDS)): {:.6} {:.6} h {:.0} m | {} gps + {} bds, rms {:.1} m, gdop {:.1}, isx {:.2} km, sbas-corr {} lt-corr {} iono {} [{}]",
                f.lat, f.lon, f.alt_km * 1000.0, f.n_gps, f.n_bds, f.residual_rms_m, f.gdop, f.isx_km, n_sbas_corr, n_lt_corr, n_iono_corr, gate
            );
            let doc = serde_json::json!({
                "epoch": now,
                "ttl_s": 900,
                "position": {
                    "lat": f.lat, "lon": f.lon, "alt_km": f.alt_km,
                    "clock_km": f.clock_gps_km, "isx_km": f.isx_km,
                    "residual_rms_m": f.residual_rms_m,
                    "gdop": f.gdop, "n_sat": f.n_sat, "mode": "3D(mixed GPS+BDS)",
                    "gate": gate,
                    "n_sbas_corr": n_sbas_corr, "n_lt_corr": n_lt_corr, "n_iono_corr": n_iono_corr,
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
            // pre-solve sanity, same as the mixed path: a garbage-frame
            // anchor sits ~1e10 km out and must never reach the solver
            .filter(|m| m.pseudorange.abs() < 5000.0)
            .collect();
        let g = hackrf_gnss::gps::ephemeris::geodetic_to_ecef(
            dyn_lla[0], dyn_lla[1], dyn_lla[2] / 1000.0,
        );
        let r0 = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
        let alt_hold = || hackrf_gnss::gps::pvt::Meas {
            sat: [0.0, 0.0, 0.0],
            pseudorange: r0,
            clock_free: true,
        };
        // EXACT solve (4 sats, 4 unknowns): rms is identically zero and
        // cannot flag a bad anchor. Leave-one-out: solve all four
        // 3-sat + alt-hold subsets; if dropping one sat moves the fix by
        // more than LOO_KM, that sat was dragging the solve — publish the
        // reduced fix instead, annotated.
        const LOO_KM: f64 = 20.0;
        let mut loo_note: Option<String> = None;
        if meas.len() == 4 {
            if let Some(full) = hackrf_gnss::gps::pvt::solve(&meas, g) {
                let mut worst: Option<(u8, f64, hackrf_gnss::gps::pvt::Fix)> = None;
                for i in 0..meas.len() {
                    let mut sub = meas.clone();
                    sub.remove(i);
                    sub.push(alt_hold());
                    if let Some(fi) = hackrf_gnss::gps::pvt::solve(&sub, g) {
                        let d = ((fi.ecef[0] - full.ecef[0]).powi(2)
                            + (fi.ecef[1] - full.ecef[1]).powi(2)
                            + (fi.ecef[2] - full.ecef[2]).powi(2))
                        .sqrt();
                        if worst.as_ref().map_or(true, |w| d > w.1) {
                            worst = Some((gps_prns[i], d, fi));
                        }
                    }
                }
                if let Some((prn, d, fi)) = worst {
                    if d > LOO_KM {
                        eprintln!(
                            "live_fix: LOO — dropping PRN {prn} moves the fix {d:.0} km; publishing the reduced 3-sat fix"
                        );
                        loo_note = Some(format!(
                            "LOO excluded PRN {prn} (moved fix {d:.0} km)"
                        ));
                        let gate = "ungated — exact solve, unverifiable";
                        println!(
                            "PVT(anchored,2D(alt-hold),LOO): {:.6} {:.6} h {:.0} m | 3 sats, rms {:.1} m, gdop {:.1} [{}; {}]",
                            fi.lat, fi.lon, fi.alt_km * 1000.0, fi.residual_rms_m, fi.gdop, gate,
                            loo_note.as_deref().unwrap_or("")
                        );
                        let doc = serde_json::json!({
                            "epoch": now,
                            "ttl_s": 900,
                            "position": {
                                "lat": fi.lat, "lon": fi.lon, "alt_km": fi.alt_km,
                                "clock_km": fi.clock_km, "residual_rms_m": fi.residual_rms_m,
                                "gdop": fi.gdop, "n_sat": fi.n_sat,
                                "mode": "2D(alt-hold)",
                                "gate": gate,
                                "n_sbas_corr": n_sbas_corr, "n_lt_corr": n_lt_corr, "n_iono_corr": n_iono_corr,
                                "loo": loo_note,
                                "source": "live TOW-anchored pseudoranges + self-decoded/BRDC ephemeris",
                            }
                        });
                        let tmp = format!("{OUT}.tmp");
                        std::fs::write(&tmp, doc.to_string()).unwrap();
                        std::fs::rename(&tmp, OUT).unwrap();
                        return;
                    }
                }
            }
        }
        let mut mode = "3D";
        if meas.len() == 3 {
            // 3 sats: altitude-hold pseudo-measurement. Range to Earth's
            // centre = the EXACT geocentric radius of the surveyed site
            // (the 6371 km mean sphere would bake in ~1.3 km of bias at
            // this latitude). The row carries no receiver clock — solved
            // with a zero clock coefficient (clock_free), otherwise clock
            // and altitude trade freely along this row.
            meas.push(alt_hold());
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
            // honesty gate: 4 sats / 4 unknowns is an EXACT solve — rms is
            // zero by construction and says nothing about correctness
            let gate = if f.n_sat >= 5 {
                "redundant"
            } else {
                "ungated — exact solve, unverifiable"
            };
            println!(
                "PVT(anchored,{mode}): {:.6} {:.6} h {:.0} m | {} sats, rms {:.1} m, gdop {:.1}, sbas-corr {} lt-corr {} iono {} [{}]",
                f.lat, f.lon, f.alt_km * 1000.0, f.n_sat, f.residual_rms_m, f.gdop, n_sbas_corr, n_lt_corr, n_iono_corr, gate
            );
            let doc = serde_json::json!({
                "epoch": now,
                "ttl_s": 900,
                "position": {
                    "lat": f.lat, "lon": f.lon, "alt_km": f.alt_km,
                    "clock_km": f.clock_km, "residual_rms_m": f.residual_rms_m,
                    "gdop": f.gdop, "n_sat": f.n_sat, "mode": mode,
                    "gate": gate,
                    "n_sbas_corr": n_sbas_corr, "n_lt_corr": n_lt_corr, "n_iono_corr": n_iono_corr,
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
            let gate = if f.n_sat >= 5 {
                "redundant"
            } else {
                "ungated — exact solve, unverifiable"
            };
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
                    "gate": gate,
                    "corr_note": "code-phase snapshot path — WAAS corrections not applicable to this measurement model",
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
