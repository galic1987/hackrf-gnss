//! Decode SBAS/WAAS message payloads from an int8 I/Q capture: acquire one
//! SBAS PRN, track it (epoch-aligned DLL + Costas, port of
//! validation/sbas_decode.py), recover 500 sym/s soft symbols, and run the
//! sbas back end (Viterbi -> frame sync -> CRC-24Q -> message parsing).
//!
//! This touches NO radio — it is the offline/capture path. The live path
//! needs live.rs to export the SBAS channels' 1 ms prompt-I stream; the hook
//! is documented at the top of src/sbas.rs.
//!
//! usage: sbas_decode <capture.iq> <fs> <fc_hz> <prn> [seconds]
//!   e.g. sbas_decode ../observations/sbas_work/multi.iq 8000000 1575420000 131 60
use hackrf_gnss::gps::{acquire_sbas, sbas_code, F_L1};
use hackrf_gnss::sbas;
use num_complex::Complex;
use std::io::Read;

const CHIP_RATE: f64 = 1.023e6;
const CODE_LEN: usize = 1023;

/// Carrier-aided DLL + 2nd-order Costas, integrated over code-epoch-aligned
/// blocks exactly one code period long (port of sbas_decode.py:track). An
/// SBAS symbol is exactly two code periods, so epoch-aligned blocks never
/// straddle a data transition. Returns one prompt value per epoch (~1 ms).
fn track(
    sig: &[Complex<f32>],
    fs: f64,
    prn: usize,
    if_hz: f64,
    dopp0: f64,
    cp0: f64,
    nms: usize,
) -> Vec<f64> {
    let code = sbas_code(prn);
    let (pll_bw, dll_bw, zeta, spacing) = (5.0f64, 1.0f64, 0.707f64, 0.5f64);
    let half = spacing / 2.0;
    let wn = pll_bw / 0.53;
    let kp = 2.0 * zeta * wn / (2.0 * std::f64::consts::PI);
    let ki = wn * wn / (2.0 * std::f64::consts::PI);
    let k_dll = 4.0 * dll_bw;

    let mut fcar = dopp0;
    let mut fint = dopp0;
    let mut chip_rate = CHIP_RATE * (1.0 + fcar / F_L1);
    // sample index of the next code epoch: local phase cp0 at sample 0
    let mut pos = (CODE_LEN as f64 - cp0) % CODE_LEN as f64 * fs / chip_rate;
    let mut phi = 0.0f64;
    let mut prompts = Vec::with_capacity(nms);
    for _ in 0..nms {
        chip_rate = CHIP_RATE * (1.0 + fcar / F_L1); // carrier aiding
        let span = CODE_LEN as f64 * fs / chip_rate;
        let n0 = pos.floor() as usize;
        let frac = pos - n0 as f64;
        let nlen = (pos + span).floor() as usize - n0;
        if n0 + nlen > sig.len() || nlen == 0 {
            break;
        }
        let f = if_hz + fcar;
        let (mut p, mut e, mut l) = (Complex::new(0.0f32, 0.0), 0.0f32, 0.0f32);
        for (j, &x) in sig[n0..n0 + nlen].iter().enumerate() {
            let t = j as f64 / fs;
            let ph = (-frac * chip_rate / fs + chip_rate * t) as f32; // chips since epoch
            let b = x * Complex::from_polar(1.0, -(2.0 * std::f64::consts::PI * f * t + phi) as f32);
            let idx = |ph: f32| {
                let i = ph as i64;
                code[i.rem_euclid(CODE_LEN as i64) as usize]
            };
            p += b * idx(ph);
            e += (b * idx(ph + half as f32)).norm();
            l += (b * idx(ph - half as f32)).norm();
        }
        let s = e + l;
        let disc = (if s > 0.0 { (e - l) / s } else { 0.0 }) as f64;
        // ideal triangular ACF: delta = local minus true
        let delta = -0.5 * (2.0 - spacing) * disc;
        let t_blk = nlen as f64 / fs;
        let pos_next = pos + span + k_dll * delta * t_blk * fs / chip_rate;

        let (i, q) = (p.re as f64, p.im as f64);
        let perr = if i != 0.0 {
            (q * i.signum()).atan2(i.abs())
        } else {
            q.atan2(1e-30)
        };
        fint += ki * perr * t_blk;
        let fcar_next = fint + kp * perr;

        prompts.push(i);
        phi = (phi + 2.0 * std::f64::consts::PI * f * t_blk)
            .rem_euclid(2.0 * std::f64::consts::PI);
        pos = pos_next;
        fcar = fcar_next;
    }
    prompts
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 5 {
        eprintln!("usage: sbas_decode <capture.iq> <fs> <fc_hz> <prn> [seconds]");
        std::process::exit(2);
    }
    let path = &a[1];
    let fs: f64 = a[2].parse().unwrap();
    let fc: f64 = a[3].parse().unwrap();
    let prn: usize = a[4].parse().unwrap();
    let seconds: f64 = a.get(5).and_then(|s| s.parse().ok()).unwrap_or(60.0);
    let if_hz = F_L1 - fc;

    // read at most `seconds` of int8 interleaved I/Q
    let nbytes = (2.0 * seconds * fs) as usize;
    let mut raw = Vec::with_capacity(nbytes.min(1 << 30));
    std::fs::File::open(path)
        .unwrap()
        .take(nbytes as u64)
        .read_to_end(&mut raw)
        .unwrap();
    let sig: Vec<Complex<f32>> = raw
        .chunks_exact(2)
        .map(|c| Complex::new(c[0] as i8 as f32, c[1] as i8 as f32))
        .collect();
    let dur = sig.len() as f64 / fs;
    eprintln!(
        "{path}: {:.1} Msps, fc {:.4} MHz (IF {:+.3} MHz), {:.1} s, PRN {prn}",
        fs / 1e6,
        fc / 1e6,
        if_hz / 1e6,
        dur
    );

    // mix IF down to baseband (single-threaded; plenty fast at capture rates)
    let n: Vec<f32> = (0..sig.len()).map(|i| i as f32).collect();
    let bb: Vec<Complex<f32>> = sig
        .iter()
        .zip(n.iter())
        .map(|(&x, &i)| {
            x * Complex::from_polar(1.0, (-2.0 * std::f64::consts::PI * if_hz * i as f64 / fs) as f32)
        })
        .collect();

    // GEO Doppler ~ receiver clock offset only; search a narrow window.
    let dopp: Vec<f64> = (-6000..=6000).step_by(200).map(|x| x as f64).collect();
    let acq_ms = (600.0f64).min(dur * 1000.0) as usize;
    let res = acquire_sbas(&bb, fs, &[prn], &dopp, acq_ms, 2.5);
    let Some(r) = res.first() else {
        eprintln!("no acquisition result");
        std::process::exit(1);
    };
    eprintln!(
        "acquisition   peak/2nd {:.2}   Doppler {:+.1} Hz   code phase {:.2} chips",
        r.metric, r.doppler, r.code_phase
    );
    if !r.acquired {
        eprintln!("VERDICT: NO SIGNAL (metric below threshold)");
        std::process::exit(1);
    }

    let nms = (dur * 1000.0) as usize - 1;
    // Convention bridge: acquire_codes reports the FFT bin of the correlation
    // peak, which is the NEGATIVE of the "chip phase of the signal at sample
    // 0" convention the tracker below (a port of sbas_decode.py:track) uses.
    // Verified on a synthetic capture: the two sum to 1023 chips.
    let cp0 = (CODE_LEN as f64 - r.code_phase) % CODE_LEN as f64;
    let prompts = track(&bb, fs, prn, 0.0, r.doppler, cp0, nms);
    eprintln!("tracking      {} ms of prompt symbols", prompts.len());

    let (soft, par) = sbas::symbols_from_prompt(&prompts);
    eprintln!(
        "symbols       {} soft symbols at 500 sps (parity {})",
        soft.len(),
        par
    );

    let rep = sbas::decode_symbols(&soft, 3);
    eprintln!(
        "framing       offset {:?}: {}/{} blocks pass CRC-24Q  {}   (inv_g2 {}, sym_off {})",
        rep.sync.offset,
        rep.sync.npass,
        rep.sync.nblocks,
        if rep.sync.locked { "LOCKED" } else { "NO LOCK" },
        rep.invert_g2,
        rep.sym_offset
    );
    eprintln!(
        "preamble      {}/{} valid blocks carry the expected 0x53/0x9A/0xC6 cycle (phase {})",
        rep.preamble.1, rep.preamble.2, rep.preamble.0
    );
    let mut counts = std::collections::BTreeMap::new();
    for dm in &rep.messages {
        *counts.entry(dm.message.mt()).or_insert(0usize) += 1;
    }
    let summary: Vec<String> = counts.iter().map(|(k, v)| format!("MT{k} x{v}")).collect();
    println!("messages      {} decoded: {}", rep.messages.len(), summary.join(", "));
    for dm in &rep.messages {
        println!("  [{:3}] {:?}", dm.block_index, dm.message);
    }
    println!(
        "VERDICT: {}   CRC pass rate {:.1}%",
        if rep.sync.locked {
            "DECODED"
        } else {
            "NO SIGNAL / NO LOCK"
        },
        100.0 * rep.sync.npass as f64 / rep.sync.nblocks.max(1) as f64
    );
}
