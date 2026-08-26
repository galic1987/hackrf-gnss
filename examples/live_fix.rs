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
const SITE_JSON: &str = "/Volumes/Radiator 8TB/gnss/observations/site.json";
const GPS_UNIX_EPOCH: f64 = 315_964_800.0;
const LEAP_S: f64 = 18.0;

/// Plausibility band on altitude (km): outside this a fix is physically
/// impossible for this station (roof/road) and must not publish — such
/// solves historically reached position_history ungated (review round 4).
const ALT_SANE_KM: (f64, f64) = (-1.0, 30.0);

fn alt_sane(alt_km: f64) -> bool {
    alt_km.is_finite() && alt_km >= ALT_SANE_KM.0 && alt_km <= ALT_SANE_KM.1
}

/// Trust split (review round 6): equation REDUNDANCY is not VALIDITY.
/// `gate` (existing) speaks to geometry only. `integrity_valid` adds
/// plausibility bounds a solve can fail while being technically redundant
/// (observed live: a 6-sat fix with 105 m rms; a mixed fix with a 10,369 km
/// intersystem "bias" — one BDS row absorbed entirely by its clock term).
/// `trusted_for_history` = redundant AND integrity_valid. Exact solves are
/// diagnostic, never trusted.
const RMS_INTEGRITY_M: f64 = 50.0; // beyond this the solve measures outliers
const ISX_SANE_KM: f64 = 50.0; // GPS-BDS clock offset is ~10 km class

fn trust_fields(n_sat: usize, redundant_at: usize, rms_m: f64, isx_km: Option<f64>, alt_km: f64) -> (bool, bool, bool) {
    let geometry_redundant = n_sat >= redundant_at;
    let integrity_valid = rms_m.is_finite()
        && rms_m < RMS_INTEGRITY_M
        && isx_km.map_or(true, |x| x.is_finite() && x.abs() < ISX_SANE_KM)
        && alt_sane(alt_km);
    (geometry_redundant, integrity_valid, geometry_redundant && integrity_valid)
}

/// Multi-GEO merge of one fast-correction row into the map: several locked
/// SBAS channels (different GEOs) can publish a row for the same GPS PRN.
/// Keep the row with the freshest insert age; a material disagreement
/// (|Δprc| > 2 m) is recorded once per PRN (largest Δ kept) for the log
/// line.
fn merge_fast_corr(
    map: &mut std::collections::HashMap<u8, (f64, f64)>,
    conflicts: &mut std::collections::BTreeMap<u8, f64>,
    prn: u8,
    prc: f64,
    age: f64,
) {
    if let Some(&(prev, prev_age)) = map.get(&prn) {
        let d = (prev - prc).abs();
        if d > 2.0 {
            let e = conflicts.entry(prn).or_insert(0.0);
            *e = e.max(d);
        }
        if age >= prev_age {
            return; // the held row is fresher (or tied): keep it
        }
    }
    map.insert(prn, (prc, age));
}

/// Same freshest-age preference for long-term corrections (no materiality
/// note — the |Δprc| > 2 m rule is a fast-correction metric). The row is
/// the harvested sbas::LtCorr: vc=1 rows carry the rates and t_lt needed
/// for solve-time propagation (DO-229D A.4.4.7 eq. A-18/A-19).
fn merge_lt_corr(
    map: &mut std::collections::HashMap<u8, (hackrf_gnss::sbas::LtCorr, f64)>,
    prn: u8,
    row: hackrf_gnss::sbas::LtCorr,
    age: f64,
) {
    if let Some(&(_, prev_age)) = map.get(&prn) {
        if age >= prev_age {
            return;
        }
    }
    map.insert(prn, (row, age));
}

/// Canonical site anchor (observations/site.json) — NO hardcoded
/// coordinates. Only a coarse (~150 km) guess is needed for the light-time
/// anchor / snapshot solver, but even that must come from the one file.
fn site_lla() -> [f64; 3] {
    hackrf_gnss::site::load_site(std::path::Path::new(SITE_JSON)).unwrap_or_else(|| {
        eprintln!("live_fix: no site anchor — provide {SITE_JSON}");
        std::process::exit(2);
    })
}

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
    // to the canonical site.json anchor only on cold start.
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
        .unwrap_or_else(site_lla);
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
    // SBAS channel's published fast_corr: GPS PRN -> (PRC metres, insert
    // age s). DO-229 convention: the PRC is ADDED to the measured
    // pseudorange. Several locked GEO channels can publish a row for the
    // same PRN — merge_fast_corr keeps the freshest and records material
    // disagreement (a silent last-writer-wins overwrite was the review
    // round-4 finding).
    let mut sbas_prc: std::collections::HashMap<u8, (f64, f64)> =
        std::collections::HashMap::new();
    let mut sbas_lt: std::collections::HashMap<u8, (hackrf_gnss::sbas::LtCorr, f64)> =
        std::collections::HashMap::new();
    let mut sbas_conf: std::collections::BTreeMap<u8, f64> = std::collections::BTreeMap::new();
    for s in v["tracker"]["sats"].as_array().into_iter().flatten() {
        if s["sys"].as_str() != Some("sbas") {
            continue;
        }
        if !s["sbas_msgs"]["locked"].as_bool().unwrap_or(false) {
            continue;
        }
        for row in s["sbas_msgs"]["fast_corr"].as_array().into_iter().flatten() {
            if let (Some(prn), Some(prc)) = (row[0].as_u64(), row[1].as_f64()) {
                // rows from a pre-age tracker build lack element [3] —
                // an unknown age sorts oldest
                let age = row[3].as_f64().unwrap_or(f64::MAX);
                merge_fast_corr(&mut sbas_prc, &mut sbas_conf, prn as u8, prc, age);
            }
        }
        // SBAS long-term corrections (MT24/25), one JSON object per row:
        // the flattened sbas::LtCorr (prn, dx/dy/dz m and daf0 s at t_lt,
        // vc=1 rates ddx/ddy/ddz m/s and daf1 s/s, t_lt_s time-of-day
        // applicability, iod) plus age_s. Corrected sat position =
        // broadcast + δ(t), corrected sat clock offset = broadcast +
        // δΔtSV(t); the base + rate·(t − t_lt) propagation of DO-229D
        // A.4.4.7 eq. (A-18)/(A-19) happens at application time below
        // (LtCorr::propagate). The iod is the GPS IODE of the ephemeris
        // the correction was generated against (DO-229D Table A-10
        // Note 3); it is gated below against the ephemeris in use when
        // that ephemeris carries an IODE.
        for row in s["sbas_msgs"]["lt_corr"].as_array().into_iter().flatten() {
            if let Ok(corr) = serde_json::from_value::<hackrf_gnss::sbas::LtCorr>(row.clone()) {
                // rows from a pre-age tracker build lack age_s — an
                // unknown age sorts oldest
                let age = row["age_s"].as_f64().unwrap_or(f64::MAX);
                merge_lt_corr(&mut sbas_lt, corr.prn, corr, age);
            }
        }
    }
    if !sbas_conf.is_empty() {
        let list: Vec<String> = sbas_conf
            .iter()
            .map(|(p, d)| format!("G{p} Δ{d:.1} m"))
            .collect();
        eprintln!(
            "live_fix: SBAS GEOs disagree on PRC (>2 m): {} — freshest row used",
            list.join(", ")
        );
    }
    let mut n_sbas_corr = 0usize;
    let mut n_lt_corr = 0usize;
    // LT corrections present but not applied cleanly: rejected = IOD
    // mismatch vs the self-decoded ephemeris in use; ungated = BRDC
    // ephemeris (no IODE — unverifiable, applied anyway)
    let mut n_lt_rejected = 0usize;
    let mut n_lt_ungated = 0usize;
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
                let prc = sbas_prc.get(&prn).map(|&(p, _)| p).unwrap_or(0.0);
                if prc != 0.0 {
                    n_sbas_corr += 1;
                }
                let lt = sbas_lt.get(&prn);
                // a correction is present when any base OR rate term is
                // nonzero (a vc=1 row can be pure rate at t_lt)
                let lt_nonzero = lt
                    .map(|(c, _)| {
                        [c.dx, c.dy, c.dz, c.daf0].iter().any(|&v| v != 0.0)
                            || [c.ddx, c.ddy, c.ddz, c.daf1]
                                .iter()
                                .any(|v| v.map_or(false, |r| r != 0.0))
                    })
                    .unwrap_or(false);
                // IOD gate (DO-229D Table A-10 Note 3): the LT correction is
                // valid only against the ephemeris issue it names. Self-
                // decoded ephemerides carry IODE and can be checked; BRDC
                // ones can't (RINEX has no IODE) and apply unverified —
                // counted separately so the panel can tell.
                let lt_ok = lt_nonzero
                    && match lt {
                        Some((c, _)) => match eph.iode {
                            Some(iode) if iode != c.iod => {
                                n_lt_rejected += 1;
                                false
                            }
                            Some(_) => true,
                            None => {
                                n_lt_ungated += 1;
                                true
                            }
                        },
                        None => false,
                    };
                // propagate to the current epoch (tow is GPST
                // seconds-of-week; LtCorr::propagate folds it to
                // time-of-day and corrects the day rollover per DO-229D
                // A.4.4.7). Counted only when a correction is applied.
                let (dx, dy, dz, daf0) = if lt_ok {
                    n_lt_corr += 1;
                    lt.unwrap().0.propagate(tow)
                } else {
                    (0.0, 0.0, 0.0, 0.0)
                };
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
            if !alt_sane(f.alt_km) {
                eprintln!(
                    "live_fix: impossible altitude {:.1} km — not publishing (mixed solve)",
                    f.alt_km
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
            let (geo_red, integ, trusted) =
                trust_fields(f.n_sat, 6, f.residual_rms_m, Some(f.isx_km), f.alt_km);
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
                    "geometry_redundant": geo_red, "integrity_valid": integ,
                    "trusted_for_history": trusted,
                    "n_sbas_corr": n_sbas_corr, "n_lt_corr": n_lt_corr, "n_iono_corr": n_iono_corr,
                    "n_lt_rejected": n_lt_rejected, "n_lt_ungated": n_lt_ungated,
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
                        if !alt_sane(fi.alt_km) {
                            continue; // impossible LOO subset solves don't count
                        }
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
                                // a LOO-excluded subset solve is diagnostic,
                                // never trusted for history (round 6)
                                "geometry_redundant": false,
                                "integrity_valid": fi.residual_rms_m < RMS_INTEGRITY_M && alt_sane(fi.alt_km),
                                "trusted_for_history": false,
                                "n_sbas_corr": n_sbas_corr, "n_lt_corr": n_lt_corr, "n_iono_corr": n_iono_corr,
                    "n_lt_rejected": n_lt_rejected, "n_lt_ungated": n_lt_ungated,
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
        // RAIM: with >= 5 sats the solve is redundant — police outliers
        // (review round 4: this path produces most published fixes and had
        // zero production callers of solve_with_rejection). Exact solves
        // (4 rows) keep the LOO procedure above; nothing independent to
        // test against there.
        let solved = if meas.len() >= 5 {
            hackrf_gnss::gps::pvt::solve_with_rejection(&meas, g, 2).map(|(f, dropped)| {
                if !dropped.is_empty() {
                    let names: Vec<String> = dropped
                        .iter()
                        .map(|&i| format!("G{}", gps_prns.get(i).copied().unwrap_or(0)))
                        .collect();
                    eprintln!("live_fix: RAIM dropped outlier channels {names:?} (>1 km residual)");
                }
                f
            })
        } else {
            hackrf_gnss::gps::pvt::solve(&meas, g)
        };
        if let Some(f) = solved {
            if f.residual_rms_m > 2000.0 {
                // same publish gate as the snapshot path: a fix this loose
                // is meaningless — keep showing the previous good fix
                eprintln!(
                    "live_fix: anchored fix rms {:.0} m — too coarse to publish ({} sats, {mode})",
                    f.residual_rms_m, f.n_sat
                );
                std::process::exit(5);
            }
            if !alt_sane(f.alt_km) {
                eprintln!(
                    "live_fix: impossible altitude {:.1} km — not publishing ({} sats, {mode})",
                    f.alt_km, f.n_sat
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
            let (geo_red, integ, trusted) =
                trust_fields(f.n_sat, 5, f.residual_rms_m, None, f.alt_km);
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
                    "geometry_redundant": geo_red, "integrity_valid": integ,
                    "trusted_for_history": trusted,
                    "n_sbas_corr": n_sbas_corr, "n_lt_corr": n_lt_corr, "n_iono_corr": n_iono_corr,
                    "n_lt_rejected": n_lt_rejected, "n_lt_ungated": n_lt_ungated,
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
    match snapshot_fix(&obs, &ephs, site_lla(), tow) {
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
            if !alt_sane(f.alt_km) {
                eprintln!(
                    "live_fix: impossible altitude {:.1} km — not publishing (snapshot)",
                    f.alt_km
                );
                std::process::exit(5);
            }
            let gate = if f.n_sat >= 5 {
                "redundant"
            } else {
                "ungated — exact solve, unverifiable"
            };
            let (geo_red, integ, trusted) =
                trust_fields(f.n_sat, 5, f.residual_rms_m, None, f.alt_km);
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
                    "geometry_redundant": geo_red, "integrity_valid": integ,
                    "trusted_for_history": trusted,
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


#[cfg(test)]
mod tests {
    use super::*;

    /// Trust split (review round 6): redundancy is not validity — a
    /// redundant solve with 105 m rms or a 10,369 km isx is NOT
    /// integrity-valid; an exact solve is never trusted for history.
    #[test]
    fn trust_fields_separates_geometry_from_validity() {
        // clean redundant solve: trusted
        assert_eq!(trust_fields(6, 5, 3.0, None, 0.02), (true, true, true));
        // redundant geometry but 105 m rms (observed live): not valid
        assert_eq!(trust_fields(6, 5, 105.3, None, 0.02), (true, false, false));
        // absurd intersystem bias (10,369 km observed): not valid
        assert_eq!(trust_fields(6, 6, 3.0, Some(10369.0), 0.02), (true, false, false));
        // exact solve: never trusted, even when clean
        assert_eq!(trust_fields(4, 5, 0.0, None, 0.02), (false, true, false));
        // impossible altitude: not valid
        assert_eq!(trust_fields(6, 5, 3.0, None, 100.0), (true, false, false));
    }

    /// Multi-GEO merge (review round 4): the freshest row wins; material
    /// disagreement (|Δprc| > 2 m) is noted once per PRN with the largest
    /// Δ kept; sub-threshold disagreement is silent and an older row never
    /// displaces a fresher one.
    #[test]
    fn multi_geo_merge_prefers_freshest_and_notes_conflict() {
        let mut map = std::collections::HashMap::new();
        let mut conf = std::collections::BTreeMap::new();
        merge_fast_corr(&mut map, &mut conf, 5, 1.0, 3.0); // GEO A, 3 s old
        merge_fast_corr(&mut map, &mut conf, 5, 4.5, 1.0); // GEO B, fresher
        assert_eq!(map[&5], (4.5, 1.0), "the freshest row must win");
        assert!((conf[&5] - 3.5).abs() < 1e-12, "Δ3.5 m must be noted");
        merge_fast_corr(&mut map, &mut conf, 5, 4.7, 2.0); // older, small Δ
        assert_eq!(map[&5], (4.5, 1.0), "an older row never displaces a fresher one");
        assert_eq!(conf.len(), 1, "sub-2 m disagreement stays silent");
        merge_fast_corr(&mut map, &mut conf, 5, 9.0, 0.5); // freshest, big Δ
        assert_eq!(map[&5], (9.0, 0.5));
        assert!((conf[&5] - 4.5).abs() < 1e-12, "the largest Δ is kept");
        // a second PRN is tracked independently
        merge_fast_corr(&mut map, &mut conf, 12, 1.0, 1.0);
        merge_fast_corr(&mut map, &mut conf, 12, 1.5, 0.0);
        assert_eq!(map[&12], (1.5, 0.0));
        assert_eq!(conf.len(), 1, "Δ0.5 m on G12 must not be noted");
        // LT rows: same freshest-age preference (no materiality note)
        let mut lt = std::collections::HashMap::new();
        let lt_row = |dx: f64| hackrf_gnss::sbas::LtCorr {
            prn: 7,
            dx,
            dy: 0.0,
            dz: 0.0,
            daf0: 0.0,
            ddx: None,
            ddy: None,
            ddz: None,
            daf1: None,
            t_lt_s: None,
            iod: 42,
        };
        merge_lt_corr(&mut lt, 7, lt_row(1.0), 5.0);
        merge_lt_corr(&mut lt, 7, lt_row(2.0), 2.0);
        assert_eq!(lt[&7].0.dx, 2.0, "the freshest LT row must win");
        merge_lt_corr(&mut lt, 7, lt_row(3.0), 4.0);
        assert_eq!(lt[&7].0.dx, 2.0, "an older LT row must not displace a fresher one");
    }

    /// The lt_corr JSON rows the tracker publishes (LtCorrReport: the
    /// flattened sbas::LtCorr + age_s) parse into sbas::LtCorr exactly the
    /// way the main-path loop does, and vc=1 rows propagate per DO-229D
    /// A.4.4.7 eq. (A-18)/(A-19): correction(t) = base + rate·(t − t_lt).
    #[test]
    fn lt_corr_row_json_parse_and_propagate() {
        let j = serde_json::json!({
            "prn": 7, "dx": 10.0, "dy": 0.0, "dz": 0.0, "daf0": 1e-6,
            "ddx": 0.01, "ddy": null, "ddz": null, "daf1": null,
            "t_lt_s": 3600, "iod": 42, "age_s": 12.5,
        });
        let c: hackrf_gnss::sbas::LtCorr = serde_json::from_value(j).unwrap();
        assert_eq!(c.prn, 7);
        assert_eq!(c.t_lt_s, Some(3600));
        assert_eq!(c.iod, 42);
        // dt = +120 s: dx grows by ddx·120, daf0 unchanged (daf1 null)
        let (x, _, _, a) = c.propagate(3720.0);
        assert!((x - 11.2).abs() < 1e-12, "{x}");
        assert_eq!(a, 1e-6);
        // a vc=0 row (null rates, null t_lt) parses and is constant
        let j0 = serde_json::json!({
            "prn": 3, "dx": 1.0, "dy": 0.0, "dz": 0.0, "daf0": 0.0,
            "ddx": null, "ddy": null, "ddz": null, "daf1": null,
            "t_lt_s": null, "iod": 7, "age_s": 3.0,
        });
        let c0: hackrf_gnss::sbas::LtCorr = serde_json::from_value(j0).unwrap();
        assert_eq!(c0.propagate(123_456.0), (1.0, 0.0, 0.0, 0.0));
    }
}
