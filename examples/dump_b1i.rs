// Dump BeiDou B1I codes so the Python port can be checked against this one
// bit for bit, rather than against its own expectations.
fn main() {
    for prn in [1usize, 2, 6, 19, 37] {
        let c = hackrf_gnss::dsp_calib::generate_beidou_b1_code(prn);
        let bits: String = c.iter().map(|v| if *v < 0.0 { '1' } else { '0' }).collect();
        println!("{} {}", prn, bits);
    }
}
