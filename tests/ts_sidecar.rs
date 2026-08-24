//! Tests for the timestamp sidecar: tick-rate calibration math and the
//! sidecar JSON schema roundtrip.

use hackrf_gnss::ts_sidecar::{calibrate_tick_hz, TicksSource, TsSidecar};

#[test]
fn tick_hz_calibration_is_exact() {
    // 8 s at 8 Msps = 64M samples; counter advanced 320_000_042 ticks.
    // (320_001_042 - 1_000) * 8e6 / 64e6 = 40_000_005.25 exactly — the task
    // brief's expected constant (40_000_006.56) doesn't match its own
    // formula; the formula is the contract.
    let hz = calibrate_tick_hz(1_000, 320_001_042, 64_000_000, 8e6);
    assert!((hz - 40_000_005.25).abs() < 0.5);
    // exact mean-rate hardware truth from the ext image: 16 ticks/sample at
    // 2.5 Msps -> exactly 40 MHz
    let hz = calibrate_tick_hz(0, 208 * 1000, 13 * 1000, 2.5e6);
    assert!((hz - 40e6).abs() < 1e-3);
}

#[test]
fn sidecar_roundtrip() {
    let sc = TsSidecar {
        stream_start_ticks: 10_930_291_099,
        tick_hz: 32_000_000.42,
        fs: 8e6,
        image: 0,
        ticks_source: TicksSource::Spi,
        utc_known: false,
    };
    let s = serde_json::to_string(&sc).unwrap();
    let back: TsSidecar = serde_json::from_str(&s).unwrap();
    assert_eq!(back, sc);
    // the exact keys the schema promises
    let v: serde_json::Value = serde_json::from_str(&s).unwrap();
    for k in [
        "stream_start_ticks",
        "tick_hz",
        "fs",
        "image",
        "ticks_source",
        "utc_known",
    ] {
        assert!(v.get(k).is_some(), "missing key {k} in {s}");
    }
    assert_eq!(v["ticks_source"].as_str().unwrap(), "spi");
    assert_eq!(v["utc_known"].as_bool().unwrap(), false);
}

#[test]
fn sidecar_roundtrip_nibble_source() {
    let sc = TsSidecar {
        stream_start_ticks: 0x00AB_CDEF_01,
        tick_hz: 40e6,
        fs: 2.5e6,
        image: 2,
        ticks_source: TicksSource::NibbleStream,
        utc_known: false,
    };
    let s = serde_json::to_string(&sc).unwrap();
    let v: serde_json::Value = serde_json::from_str(&s).unwrap();
    assert_eq!(v["ticks_source"].as_str().unwrap(), "nibble-stream");
    let back: TsSidecar = serde_json::from_str(&s).unwrap();
    assert_eq!(back, sc);
}
