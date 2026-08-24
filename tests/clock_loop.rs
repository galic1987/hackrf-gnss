//! Tests for the clock-discipline scaffolding: correction accumulation
//! (including its sign convention and the no-estimate case) and the
//! clock_loop_log.jsonl schema roundtrip.

use hackrf_gnss::discipline::{Correction, CycleLog};

#[test]
fn correction_accumulates_the_negated_measurement() {
    // hardware-verified sequence: correction +26.00 zeroed a -26.00 ppm
    // measurement; a later -0.92 ppm residual needs delta +0.92
    let mut c = Correction::new(26.0);
    let d = c.update(Some(-0.92)).unwrap();
    assert!((d - 0.92).abs() < 1e-12, "delta {d}");
    assert!((c.ppm - 26.92).abs() < 1e-12, "correction {}", c.ppm);
    // a clock now measured fast (positive ppm) must LOWER the correction
    let d = c.update(Some(0.5)).unwrap();
    assert!((d + 0.5).abs() < 1e-12);
    assert!((c.ppm - 26.42).abs() < 1e-12);
}

#[test]
fn a_cycle_without_an_estimate_leaves_the_state_untouched() {
    let mut c = Correction::new(26.0);
    assert_eq!(c.update(None), None);
    assert!((c.ppm - 26.0).abs() < 1e-12);
    // and the next real measurement still applies relative to 26.0
    c.update(Some(-1.0));
    assert!((c.ppm - 27.0).abs() < 1e-12);
}

#[test]
fn cycle_log_roundtrips_with_the_published_schema() {
    let line = CycleLog {
        t: 1787340000.5,
        measured_ppm: Some(-0.92),
        delta_ppm: Some(0.92),
        correction_ppm: 26.92,
        n_detected: 7,
        n_attributed: 4,
        sats: vec!["IRIDIUM 104".to_string()],
    };
    let s = serde_json::to_string(&line).unwrap();
    // the exact keys the schema promises, in a null-free Some cycle
    let v: serde_json::Value = serde_json::from_str(&s).unwrap();
    for k in [
        "t",
        "measured_ppm",
        "delta_ppm",
        "correction_ppm",
        "n_detected",
        "n_attributed",
        "sats",
    ] {
        assert!(v.get(k).is_some(), "missing key {k} in {s}");
    }
    assert_eq!(v["measured_ppm"].as_f64().unwrap(), -0.92);
    let back: CycleLog = serde_json::from_str(&s).unwrap();
    assert_eq!(back, line);
}

#[test]
fn a_null_cycle_serializes_as_null_not_missing() {
    let line = CycleLog {
        t: 1787340050.0,
        measured_ppm: None,
        delta_ppm: None,
        correction_ppm: 26.0,
        n_detected: 0,
        n_attributed: 0,
        sats: vec![],
    };
    let v: serde_json::Value =
        serde_json::from_str(&serde_json::to_string(&line).unwrap()).unwrap();
    assert!(v["measured_ppm"].is_null(), "{}", v);
    assert!(v["delta_ppm"].is_null(), "{}", v);
    let back: CycleLog = serde_json::from_str(&serde_json::to_string(&v).unwrap()).unwrap();
    assert_eq!(back, line);
}
