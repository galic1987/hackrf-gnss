//! Compare per-PRN anchor residuals using SELF-DECODED vs BRDC ephemeris.
//! usage: eph_cmp
use hackrf_gnss::gps::broadcast::BrdcEph;

const C_KM_S: f64 = 299_792.458;

fn sat(eph: &BrdcEph, t_tx: f64) -> ([f64; 3], f64) {
    let (sat_m, dt_sv, _) = hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, t_tx, [0.0; 3]);
    ([sat_m[0] / 1000.0, sat_m[1] / 1000.0, sat_m[2] / 1000.0], dt_sv)
}

fn main() {
    let site = hackrf_gnss::gps::ephemeris::geodetic_to_ecef(39.0032, -77.6058, 0.020);
    let text = std::fs::read_to_string(
        "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json",
    )
    .unwrap();
    let v: serde_json::Value = serde_json::from_str(&text).unwrap();
    let self_eph: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string("/Volumes/Radiator 8TB/gnss/observations/tracker_eph.json")
            .unwrap(),
    )
    .unwrap();
    let brdc = std::fs::read_to_string(
        "/Volumes/Radiator 8TB/gnss/observations/brdc_latest.rnx",
    )
    .unwrap();
    let brdc_ephs = hackrf_gnss::gps::broadcast::parse_rinex_gps(&brdc);

    for s in v["tracker"]["sats"].as_array().into_iter().flatten() {
        if s["sys"].as_str() != Some("gps") || s["rho_m"].is_null() {
            continue;
        }
        let prn = s["prn"].as_u64().unwrap() as u8;
        let rho_km = s["rho_m"].as_f64().unwrap() / 1000.0;
        let t_tx = s["t_tx"].as_f64().unwrap();
        println!("PRN {prn} t_tx {t_tx}");
        for e in self_eph["ephemeris"].as_array().into_iter().flatten() {
            let eph: BrdcEph = serde_json::from_value(e.clone()).unwrap();
            if eph.prn == prn && eph.sys == 0 {
                let (pos, dt) = sat(&eph, t_tx);
                let g = ((site[0] - pos[0]).powi(2)
                    + (site[1] - pos[1]).powi(2)
                    + (site[2] - pos[2]).powi(2))
                .sqrt();
                println!(
                    "  self-decoded: rho-geom {:+12.3} km  (dt_sv {:+.1} us)",
                    rho_km + dt * C_KM_S - g,
                    dt * 1e6
                );
            }
        }
        for (p, eph) in &brdc_ephs {
            if *p == prn {
                let (pos, dt) = sat(eph, t_tx);
                let g = ((site[0] - pos[0]).powi(2)
                    + (site[1] - pos[1]).powi(2)
                    + (site[2] - pos[2]).powi(2))
                .sqrt();
                println!(
                    "  brdc(toe {:.0}): rho-geom {:+12.3} km  (dt_sv {:+.1} us)",
                    eph.toe,
                    rho_km + dt * C_KM_S - g,
                    dt * 1e6
                );
            }
        }
    }
}
