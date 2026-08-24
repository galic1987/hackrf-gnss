//! Integration test for the live 1 Hz multi-constellation tracker
//! (`src/live.rs`).
//!
//! Fixture choice: `scripts/l1_30s_bb.f32` — 30 s of REAL L1 baseband
//! (interleaved f32 I/Q at 4 Msps, IF already removed). The wideband
//! `wideband_l1_b1.iq` capture (20 Msps int8, 1568.259 MHz centre) also exists
//! but is only ~8.9 s long — too short to prove a >= 10 s lock after seeding,
//! and it is the live front end's (Engine) territory; the tracker core under
//! test here is the per-band layer. The 30 s L1 fixture carries GPS C/A,
//! WAAS SBAS and Galileo E1B on the same carrier (verified by acquisition:
//! GPS PRNs 26/1/10/28/32/16/31, SBAS 131/133/135, Galileo 12/27/21/29/7).
//!
//! Asserts: >= 4 PRNs across >= 2 systems reach lock_s >= 10 s, with
//! plausible Dopplers (|d| <= 5 kHz). If the fixture is absent (CI), falls
//! back to a synthetic GPS + SBAS composite and asserts the same.

use hackrf_gnss::gps::ca_code::{gps_ca, CHIP_RATE};
use hackrf_gnss::gps::sbas_code;
use hackrf_gnss::live::Band;
use num_complex::Complex;
use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;

const FS: f64 = 4.0e6;
const REAL_S: usize = 18; // seconds of fixture to track
const SYNTH_S: usize = 16;

fn fixture_path() -> String {
    format!("{}/scripts/l1_30s_bb.f32", env!("CARGO_MANIFEST_DIR"))
}

fn load_fixture(path: &str, secs: usize) -> Option<Vec<Complex<f32>>> {
    use std::io::Read;
    let mut f = std::fs::File::open(path).ok()?;
    let want = (FS as usize) * secs * 8;
    let mut bytes = vec![0u8; want];
    let n = f.read(&mut bytes).ok()?;
    bytes.truncate(n);
    Some(
        bytes
            .chunks_exact(8)
            .map(|c| {
                Complex::new(
                    f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                    f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
                )
            })
            .collect(),
    )
}

/// Run a band over `sig`, return per-(sys,prn) best lock_s + last Doppler.
fn run_band(
    band: &mut Band,
    sig: &[Complex<f32>],
) -> (HashMap<(String, usize), f64>, HashMap<(String, usize), (f64, f64)>) {
    let mut best_lock: HashMap<(String, usize), f64> = HashMap::new();
    let mut last: HashMap<(String, usize), (f64, f64)> = HashMap::new();
    let sec = FS as usize;
    for chunk in sig.chunks(sec) {
        for r in band.push(chunk) {
            let k = (r.sys.to_string(), r.prn);
            let e = best_lock.entry(k.clone()).or_insert(0.0);
            if r.lock_s > *e {
                *e = r.lock_s;
            }
            last.insert(k, (r.doppler_hz, r.cn0_proxy));
        }
        // acquisition runs on a worker thread (production: consumer never
        // stalls); with a finite fixture we must wait the worker out, then
        // continue feeding
        while band.worker_active() {
            band.poll();
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        band.poll(); // pick up a result delivered just after the last push
    }
    (best_lock, last)
}

fn assert_constellations(
    best_lock: &HashMap<(String, usize), f64>,
    last: &HashMap<(String, usize), (f64, f64)>,
    min_lock: f64,
) {
    let locked: Vec<&(String, usize)> = best_lock
        .iter()
        .filter(|(_, l)| **l >= min_lock)
        .map(|(k, _)| k)
        .collect();
    let systems: HashSet<&str> = locked.iter().map(|(s, _)| s.as_str()).collect();
    for (k, l) in best_lock {
        let (dopp, cn0) = last[k];
        eprintln!(
            "  {} PRN {}: max lock {:.0} s, last dopp {:+.1} Hz, cn0 {:.1}",
            k.0, k.1, l, dopp, cn0
        );
    }
    assert!(
        locked.len() >= 4,
        "need >= 4 PRNs locked >= {min_lock} s, got {}",
        locked.len()
    );
    assert!(
        systems.len() >= 2,
        "need >= 2 systems locked, got {:?}",
        systems
    );
    for k in locked {
        let d = last[k].0.abs();
        assert!(
            d <= 5000.0,
            "{} PRN {} doppler {:+.0} Hz outside ±5 kHz",
            k.0,
            k.1,
            last[k].0
        );
    }
}

#[test]
fn live_tracker_locks_real_constellations() {
    let sig = match load_fixture(&fixture_path(), REAL_S) {
        Some(s) if s.len() >= FS as usize * REAL_S / 2 => s,
        _ => {
            eprintln!("fixture missing — synthetic fallback");
            let sig = synthetic_gps_sbas(SYNTH_S);
            let mut band = Band::new_l1(FS, 0.0);
            let (best_lock, last_dopp) = run_band(&mut band, &sig);
            assert_constellations(&best_lock, &last_dopp, 10.0);
            return;
        }
    };
    eprintln!("tracking {:.1} s of real L1 baseband", sig.len() as f64 / FS);
    let mut band = Band::new_l1(FS, 0.0);
    let (best_lock, last_dopp) = run_band(&mut band, &sig);
    assert_constellations(&best_lock, &last_dopp, 10.0);
}

/// GPS PRN 11 (+800 Hz, 50 bps nav) + WAAS SBAS PRN 131 (-300 Hz, 500 sps
/// data) composite at 4 Msps with noise — the "no fixture" path.
fn synthetic_gps_sbas(secs: usize) -> Vec<Complex<f32>> {
    let n = FS as usize * secs;
    let ns = (FS / 1000.0) as usize;
    let mut sig = vec![Complex::new(0.0f32, 0.0); n];
    let mut st = 0x9e3779b97f4a7c15u64;
    let mut nxt = || {
        st ^= st << 13;
        st ^= st >> 7;
        st ^= st << 17;
        ((st >> 40) as f32 / 8_388_608.0) - 1.0
    };
    // (code, chip_rate, dopp, data period in ms, amplitude)
    let sats: Vec<(Vec<f32>, f64, f64, usize, f32)> = vec![
        (gps_ca(11), CHIP_RATE, 800.0, 20, 1.0),
        (sbas_code(131), CHIP_RATE, -300.0, 2, 0.8),
    ];
    for (code, chip_rate, dopp, data_ms, amp) in &sats {
        let code_len = code.len();
        let mut data = vec![1.0f32; n / ns / data_ms + 2];
        for d in data.iter_mut() {
            *d = if nxt() > 0.0 { 1.0 } else { -1.0 };
        }
        for k in 0..n {
            let t = k as f64 / FS;
            let ci = (k as f64 * chip_rate * (1.0 + dopp / 1575.42e6) / FS) as usize;
            let c = code[ci % code_len];
            let ph = 2.0 * PI * dopp * t;
            let bit = data[(k / ns) / data_ms];
            let v = c * bit * amp;
            sig[k].re += v * ph.cos() as f32;
            sig[k].im += v * ph.sin() as f32;
        }
    }
    // noise floor: per-quadrature std chosen for a ~45 dB-Hz proxy
    for s in sig.iter_mut() {
        s.re += 8.0 * nxt();
        s.im += 8.0 * nxt();
    }
    sig
}
