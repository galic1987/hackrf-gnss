//! Numerical parity of the Rust GPS acquisition against the Python oracle
//! (`validation/acquire.py`). The fixture is a synthetic baseband capture with
//! two PRNs injected at known Doppler/code-phase plus noise; `expected.json`
//! holds the Python `acquire()` result for every PRN over the same input. The
//! Rust acquisition must agree: same winning Doppler bin, code phase within a
//! chip, a metric close to Python's, and no false acquisition on noise PRNs.
//!
//! Regenerate the fixture with the generator in the commit that added this test.

use num_complex::Complex;
use std::fs;
use std::path::PathBuf;

fn fixtures() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

fn load_baseband() -> Vec<Complex<f32>> {
    let bytes = fs::read(fixtures().join("baseband.f32")).expect("baseband.f32");
    assert!(bytes.len() % 8 == 0, "interleaved f32 IQ");
    bytes
        .chunks_exact(8)
        .map(|c| {
            let i = f32::from_le_bytes([c[0], c[1], c[2], c[3]]);
            let q = f32::from_le_bytes([c[4], c[5], c[6], c[7]]);
            Complex::new(i, q)
        })
        .collect()
}

#[test]
fn rust_acquisition_matches_python_oracle() {
    let meta: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(fixtures().join("expected.json")).unwrap())
            .unwrap();
    let fs = meta["fs"].as_f64().unwrap();
    let nblocks = meta["nblocks"].as_u64().unwrap() as usize;
    let lo = meta["dopp_lo"].as_i64().unwrap();
    let hi = meta["dopp_hi"].as_i64().unwrap();
    let step = meta["dopp_step"].as_i64().unwrap();
    let dopplers: Vec<f64> = (lo..=hi).step_by(step as usize).map(|x| x as f64).collect();

    let sig = load_baseband();
    let prns: Vec<usize> = (1..=32).collect();
    let got = hackrf_gnss::gps::acquire(&sig, fs, &prns, &dopplers, nblocks, 2.5);

    let expected = meta["expected"].as_array().unwrap();
    let mut checked_injected = 0;
    for e in expected {
        let prn = e["prn"].as_u64().unwrap() as usize;
        let py_metric = e["metric"].as_f64().unwrap();
        let py_dopp = e["doppler"].as_f64().unwrap();
        let py_cp = e["code_phase"].as_f64().unwrap();
        let r = got.iter().find(|r| r.prn == prn).unwrap();

        if py_metric > 10.0 {
            // an injected PRN: demand tight agreement
            checked_injected += 1;
            assert_eq!(r.doppler, py_dopp, "PRN {prn} doppler bin");
            let dphi = ((r.code_phase - py_cp + 1023.0) % 1023.0)
                .min((py_cp - r.code_phase + 1023.0) % 1023.0);
            assert!(dphi <= 1.0, "PRN {prn} code phase {} vs {}", r.code_phase, py_cp);
            let rel = ((r.metric as f64) - py_metric).abs() / py_metric;
            assert!(
                rel < 0.20,
                "PRN {prn} metric {} vs python {} (rel {:.2})",
                r.metric,
                py_metric,
                rel
            );
            assert!(r.acquired);
        } else {
            // noise PRN: Rust must also see nothing (no false acquisition)
            assert!(
                r.metric < 2.5,
                "PRN {prn} falsely acquired: rust {} python {}",
                r.metric,
                py_metric
            );
        }
    }
    assert_eq!(checked_injected, 2, "both injected PRNs must be checked");
}
