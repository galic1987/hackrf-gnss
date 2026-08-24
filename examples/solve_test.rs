use hackrf_gnss::gps::broadcast::parse_rinex_gps;
use hackrf_gnss::gps::snapshot::sat_at_txtime_pub;
fn main() {
    let rinex = parse_rinex_gps(&std::fs::read_to_string(
        "/Volumes/Radiator 8TB/gnss/observations/brdc_latest.rnx").unwrap());
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(
        "/Volumes/Radiator 8TB/gnss/observations/tracker_eph.json").unwrap()).unwrap();
    for e in v["ephemeris"].as_array().unwrap() {
        let se: hackrf_gnss::gps::broadcast::BrdcEph = serde_json::from_value(e.clone()).unwrap();
        if se.sys != 0 {
            continue; // BeiDou: compared against the Cxx records elsewhere
        }
        let Some(r) = rinex.get(&se.prn) else { continue };
        // compare satellite positions at the self-decoded toe
        let (a, _, _) = sat_at_txtime_pub(&se, se.toe, [0.0,0.0,0.0]);
        let (b, _, _) = sat_at_txtime_pub(r, se.toe, [0.0,0.0,0.0]);
        let d = ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt();
        println!("PRN {:>2}: |self-decoded - RINEX| at toe = {:.1} m", se.prn, d);
    }
}
