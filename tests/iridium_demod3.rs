//! Real-data validation of the Rust demod3 port: a genuine captured Iridium
//! burst (tests/fixtures/iridium_burst.iq) is demodulated and must reproduce the
//! Python oracle's differential symbols and decoded frame bits exactly.
//!
//! The wider real-data sweep (scripts/validate_demod3_realdata.py) shows ~92% of
//! real bursts decode byte-identically to Python, the rest differing by a single
//! symbol at timing boundaries on marginal bursts. This test pins one clean
//! burst where the agreement is exact, so a regression in any stage is caught.

use hackrf_gnss::iridium::demod3;
use std::fs;
use std::path::PathBuf;

fn fx(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(name)
}

#[test]
fn decodes_a_real_burst_identically_to_python() {
    let meta: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(fx("iridium_burst.json")).unwrap()).unwrap();
    let raw_u8 = fs::read(fx("iridium_burst.iq")).unwrap();
    let raw_i8: Vec<i8> = raw_u8.iter().map(|&b| b as i8).collect();

    let fcen = meta["fcen"].as_f64().unwrap();
    let fc = meta["fc"].as_f64().unwrap();
    let fs = meta["fs"].as_f64().unwrap();

    let d = demod3::demod_snippet_debug(&raw_i8, fcen, fc, fs).expect("demod produced a burst");

    // differential symbols identical to Python
    let ds: String = d.ds.iter().map(|s| char::from(b'0' + s)).collect();
    assert_eq!(ds, meta["ds"].as_str().unwrap(), "differential symbols differ");

    // unique word at the same position
    assert_eq!(
        d.uw_pos.map(|x| x as i64).unwrap_or(-1),
        meta["uw_pos"].as_i64().unwrap(),
        "unique-word position"
    );

    // decoded frame bits identical to Python
    let rwa = d.rwa.expect("a frame was decoded");
    let bits = rwa.split_whitespace().last().unwrap();
    assert_eq!(bits, meta["rwa_bits"].as_str().unwrap(), "decoded frame bits differ");
    assert_eq!(d.conf, meta["conf"].as_i64().unwrap() as i32, "confidence");
}

/// The same golden burst, stored as 16-bit samples (i8 << 4 — full-scale
/// 12-bit right-justified, the ext-precision format), must decode
/// identically through the extended-precision (12-bit in int16) input path.
/// The top nibble of an ext-precision lane is a timestamp nibble, not data,
/// so <<8 data is outside the format's domain; <<4 is its full-scale form.
#[test]
fn decodes_a_real_burst_identically_from_i16() {
    let meta: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(fx("iridium_burst.json")).unwrap()).unwrap();
    let raw_u8 = fs::read(fx("iridium_burst.iq")).unwrap();
    let raw_i16: Vec<i16> = raw_u8.iter().map(|&b| (b as i8 as i16) << 4).collect();

    let fcen = meta["fcen"].as_f64().unwrap();
    let fc = meta["fc"].as_f64().unwrap();
    let fs = meta["fs"].as_f64().unwrap();

    let d = demod3::demod_snippet_debug(&raw_i16, fcen, fc, fs).expect("demod produced a burst");

    let ds: String = d.ds.iter().map(|s| char::from(b'0' + s)).collect();
    assert_eq!(ds, meta["ds"].as_str().unwrap(), "differential symbols differ");
    assert_eq!(
        d.uw_pos.map(|x| x as i64).unwrap_or(-1),
        meta["uw_pos"].as_i64().unwrap(),
        "unique-word position"
    );
    let rwa = d.rwa.expect("a frame was decoded");
    let bits = rwa.split_whitespace().last().unwrap();
    assert_eq!(bits, meta["rwa_bits"].as_str().unwrap(), "decoded frame bits differ");
    assert_eq!(d.conf, meta["conf"].as_i64().unwrap() as i32, "confidence");
}
