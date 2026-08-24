//! Tests for the `RateReference` trait contract: the default logging methods
//! every source inherits, and a scripted reference driven through full
//! discipline cycles exactly the way `examples/clock_loop.rs` drives them.

use hackrf_gnss::discipline::{Correction, CycleLog, RateReference};

/// A minimal reference that replays a scripted sequence of ppm estimates and
/// relies on the trait DEFAULTS for the logging methods (the point of the
/// test: the defaults are the logging contract for sources that do not track
/// their own observation counts).
struct Scripted {
    seq: Vec<Option<f64>>,
    i: usize,
}

impl RateReference for Scripted {
    fn name(&self) -> &str {
        "scripted"
    }
    fn estimate_ppm(&mut self) -> Option<f64> {
        let v = self.seq.get(self.i).copied().flatten();
        self.i += 1;
        v
    }
}

#[test]
fn trait_defaults_report_no_observations() {
    let r = Scripted { seq: vec![], i: 0 };
    assert_eq!(r.name(), "scripted");
    assert_eq!(r.n_obs(), 0);
    assert_eq!(r.n_detected(), 0);
    assert!(r.sources().is_empty());
}

#[test]
fn scripted_cycles_discipline_like_the_live_loop() {
    // The live loop's sequence: estimate -> fold the negation into the
    // correction -> log the cycle. A None estimate must neither move the
    // correction nor log a delta.
    let mut r = Scripted { seq: vec![Some(-26.0), None, Some(0.4)], i: 0 };
    let mut corr = Correction::new(0.0);

    // cycle 1: the station's real first lock (measured -26 ppm on a virgin
    // radio) applies +26.00 of correction
    let m = r.estimate_ppm();
    let d = corr.update(m);
    let log = CycleLog {
        t: 1.0,
        measured_ppm: m,
        delta_ppm: d,
        correction_ppm: corr.ppm,
        n_detected: r.n_detected(),
        n_attributed: r.n_obs(),
        sats: r.sources(),
    };
    assert!((corr.ppm - 26.0).abs() < 1e-12, "correction {}", corr.ppm);
    let s = serde_json::to_string(&log).unwrap();
    assert!(s.contains("\"measured_ppm\":-26.0"), "{s}");
    assert!(s.contains("\"delta_ppm\":26.0"), "{s}");

    // cycle 2: no usable observation -- state and log must show it
    let m = r.estimate_ppm();
    assert_eq!(m, None);
    let d = corr.update(m);
    assert_eq!(d, None);
    assert!((corr.ppm - 26.0).abs() < 1e-12, "None must not move the correction");
    let log = CycleLog {
        t: 2.0,
        measured_ppm: m,
        delta_ppm: d,
        correction_ppm: corr.ppm,
        n_detected: r.n_detected(),
        n_attributed: r.n_obs(),
        sats: r.sources(),
    };
    let s = serde_json::to_string(&log).unwrap();
    assert!(s.contains("\"measured_ppm\":null"), "{s}");
    assert!(s.contains("\"delta_ppm\":null"), "{s}");
    // and a null cycle still roundtrips
    let back: CycleLog = serde_json::from_str(&s).unwrap();
    assert_eq!(back, log);

    // cycle 3: a small positive residual (clock now fast) LOWERS the correction
    let m = r.estimate_ppm();
    corr.update(m);
    assert!((corr.ppm - 25.6).abs() < 1e-12, "correction {}", corr.ppm);

    // past the end of the script: still None, never a fabricated zero
    assert_eq!(r.estimate_ppm(), None);
}
