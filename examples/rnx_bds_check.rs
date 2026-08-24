//! Sanity check for parse_rinex_bds on a real mixed BRDC file: record count,
//! orbit-radius distribution (MEO ~27900 km, IGSO/GEO ~42164 km), and toe
//! spread. usage: rnx_bds_check [rinex]
use hackrf_gnss::beidou_d1::{parse_rinex_bds, sat_pos_ecef_bds};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let path = a
        .get(1)
        .map(|s| s.as_str())
        .unwrap_or("/Volumes/Radiator 8TB/gnss/observations/brdc_latest.rnx");
    let text = std::fs::read_to_string(path).expect("read rinex");
    let ephs = parse_rinex_bds(&text);
    println!("{} BDS ephemerides", ephs.len());
    let mut rows: Vec<_> = ephs.values().collect();
    rows.sort_by_key(|e| e.prn);
    for e in rows {
        let p = sat_pos_ecef_bds(e, e.toe);
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() / 1000.0;
        println!(
            "  C{:02} sqrtA {:8.1} e {:.4} i0 {:6.1} deg  toe {:9.0}  radius {:9.0} km",
            e.prn,
            e.sqrt_a,
            e.e,
            e.i0.to_degrees(),
            e.toe,
            r
        );
    }
}
