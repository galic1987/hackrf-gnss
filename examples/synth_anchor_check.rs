//! Synthetic-truth anchor bench: feed the 30 s gpssim capture (s45.iq, 8 Msps
//! complex int8, zero IF, 7 GPS sats at 45 dB-Hz) through a real live `Band`
//! and score every anchor against the simulation's own truth:
//!     predicted t_bit = (A - t0_gps) * (1 + clk_drift)
//! with A = t_tx - dt_sv + tau_geo from a reception-anchored light-time solve
//! (the sim's exact model, validation/gps_sim.py: the stream runs on the rx
//! clock, GPS advances at 1/(1+d0) per rx second). Any per-channel offset
//! beyond ~1 us is an anchor-chain bug, measured against truth with no
//! ephemeris or site error in the loop.
//!
//! Truth-mapping pitfalls (both manufactured a stable per-channel
//! "fractional code-phase" differential of 0.1-0.4 us on PERFECT anchors —
//! the 2026-08-25 round-7 false alarm):
//!   * anchoring the light-time solve at t_tx (SV clock) instead of the GPS
//!     reception time: the satellite is evaluated ~tau ~ 70 ms from emission,
//!     leaving range_rate * 70 ms (up to ~50 m) per channel;
//!   * mis-modelling the rx clock (drift applied against GPS TOW instead of
//!     rx-clock seconds): a ~173 ms common offset that hid the class.
//!
//! knobs:
//!   SYNTH_BIAS="PRN:HZ_PER_S"  slew one channel's carrier estimate (live
//!                               marginal-lock drift class; anchors must stay
//!                               sub-us via the measured comb)
//!   SYNTH_DELAY_SAMP=D         delay the whole capture by D samples
//!                               (fractional, linear interpolation): every
//!                               anchor error must shift by exactly D/fs
//!                               — end-to-end sub-chip reference probe
//!   SYNTH_DEC=2                boxcar-decimate to 4 Msps (the live band rate)
//!
//! usage: synth_anchor_check [s45.iq]

use hackrf_gnss::live::{Band, Sys};
use num_complex::Complex;
use std::io::Read;

const C: f64 = 299_792_458.0;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let path = a
        .get(1)
        .map(|s| s.as_str())
        .unwrap_or("/Volumes/Radiator 8TB/gnss/gpssim/s45.iq");
    let truth: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string("/Volumes/Radiator 8TB/gnss/gpssim/s45.truth.json").unwrap(),
    )
    .unwrap();
    let t0 = truth["t0_gps_s"].as_f64().unwrap();
    let rx = truth["rx_ecef"].as_array().unwrap();
    let rx = [
        rx[0].as_f64().unwrap(),
        rx[1].as_f64().unwrap(),
        rx[2].as_f64().unwrap(),
    ];
    let clk_drift = truth["clk_drift"].as_f64().unwrap();
    // per-sat truth ephemeris (sim's own), keyed by prn
    let mut ephs: std::collections::HashMap<u8, hackrf_gnss::gps::broadcast::BrdcEph> =
        Default::default();
    for s in truth["sats"].as_array().unwrap() {
        let prn = s["prn"].as_u64().unwrap() as u8;
        let e = &s["eph"];
        ephs.insert(
            prn,
            hackrf_gnss::gps::broadcast::BrdcEph {
                prn,
                week: e["week"].as_f64().unwrap(),
                toe: e["toe"].as_f64().unwrap(),
                toc: e["toc"].as_f64().unwrap(),
                sqrt_a: e["sqrtA"].as_f64().unwrap(),
                e: e["e"].as_f64().unwrap(),
                m0: e["M0"].as_f64().unwrap(),
                delta_n: e["deltan"].as_f64().unwrap(),
                omega0: e["Omega0"].as_f64().unwrap(),
                omega: e["omega"].as_f64().unwrap(),
                omega_dot: e["OmegaDot"].as_f64().unwrap(),
                i0: e["i0"].as_f64().unwrap(),
                idot: e["IDOT"].as_f64().unwrap(),
                cuc: e["Cuc"].as_f64().unwrap(),
                cus: e["Cus"].as_f64().unwrap(),
                crs: e["Crs"].as_f64().unwrap(),
                crc: e["Crc"].as_f64().unwrap(),
                cis: e["Cis"].as_f64().unwrap(),
                cic: e["Cic"].as_f64().unwrap(),
                af0: e["af0"].as_f64().unwrap(),
                af1: e["af1"].as_f64().unwrap(),
                af2: e["af2"].as_f64().unwrap(),
                tgd: e["TGD"].as_f64().unwrap(),
                sys: 0,
                // sim truth carries no IODE — None, as for BRDC-loaded eph
                iode: None,
            },
        );
    }

    let fs = 8.0e6;
    // optional rate reduction to the live band rate: SYNTH_DEC=2 boxcar-
    // decimates the 8 Msps capture to 4 Msps (what the live Engine's
    // downconvert feeds its Bands). Stream TIME is unchanged (seconds are
    // rate-independent), so the truth mapping below is untouched.
    let dec: usize = std::env::var("SYNTH_DEC")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);
    let fs_eff = fs / dec as f64;
    let epoch0 = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();
    let mut band = Band::new_l1(fs_eff, epoch0);
    let mut f = std::fs::File::open(path).expect("open capture");
    let mut raw = vec![0u8; 2 * 1024 * 1024];
    let mut fed_s = 0.0f64;
    // optional loop-bias injection: SYNCH_BIAS="PRN:SLEW_HZ_PER_S" — after
    // the anchor forms, force the channel's carrier estimate off by a
    // linearly growing amount each second (models a marginal-lock loop
    // sitting off-peak; the live drift class). Truth mapping below includes
    // the SV clock, so per-channel errors should stay sub-us regardless.
    let bias: Option<(usize, f64)> = std::env::var("SYNTH_BIAS").ok().and_then(|s| {
        let mut it = s.split(':');
        Some((it.next()?.parse().ok()?, it.next()?.parse().ok()?))
    });
    let mut last_report_s = 16usize;
    let mut last_ckpt = 0usize;
    // optional sub-chip injection: SYNTH_DELAY_SAMP=D delays the whole
    // capture by D samples (fractional part by linear interpolation) before
    // the Band sees it. Every satellite's true arrival shifts by D/fs, so
    // every anchor error must shift by exactly +D/fs*1e6 us — an end-to-end
    // probe of the sub-chip reference (acquisition -> DLL -> anchor). The
    // engine's code-phase zero point is exact if the shift shows up in
    // full; a fractional-chip bias would eat part of it.
    let delay: f64 = std::env::var("SYNTH_DELAY_SAMP")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.0);
    let d_int = delay.floor().max(0.0) as usize;
    let d_frac = delay - d_int as f64;
    if delay != 0.0 {
        eprintln!(
            "SYNTH_DELAY_SAMP={delay}: expect a uniform {:+.3} us anchor-err shift",
            delay / fs * 1e6
        );
    }
    // delayed-sample history (x[n-1] .. x[n-d_int-1] across chunk edges)
    let mut hist: std::collections::VecDeque<Complex<f32>> =
        std::collections::VecDeque::from(vec![Complex::new(0.0, 0.0); d_int + 1]);
    while fed_s < 30.0 {
        let n = f.read(&mut raw).unwrap_or(0);
        if n == 0 {
            break;
        }
        let ns = n / 2;
        let mut sig: Vec<Complex<f32>> = raw[..ns * 2]
            .chunks_exact(2)
            .map(|c| Complex::new(c[0] as i8 as f32, c[1] as i8 as f32))
            .collect();
        if delay != 0.0 {
            for s in sig.iter_mut() {
                let cur = *s;
                hist.push_front(cur);
                let older = hist.pop_back().unwrap(); // x[n - d_int - 1]
                let recent = hist.back().unwrap(); // x[n - d_int]
                *s = *recent * (1.0 - d_frac as f32) + older * d_frac as f32;
            }
        }
        if dec > 1 {
            sig = sig
                .chunks_exact(dec)
                .map(|c| c.iter().sum::<Complex<f32>>() / dec as f32)
                .collect();
        }
        band.push(&sig);
        fed_s += sig.len() as f64 / fs_eff;
        while band.worker_active() {
            band.poll();
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        band.poll();
        let whole = fed_s as usize;
        if whole > last_report_s {
            for sec in last_report_s..whole {
                if let Some((prn, slew)) = bias {
                    if sec >= 16 {
                        if let Some(ch) =
                            band.channels.iter_mut().find(|c| c.prn == prn && c.sys == Sys::Gps)
                        {
                            let cur = ch.debug_dopp();
                            ch.debug_set_dopp(cur + slew);
                        }
                    }
                }
            }
            last_report_s = whole;
        }
        if whole >= 16 && whole % 5 == 0 && whole > last_ckpt {
            last_ckpt = whole;
            report(&band, &ephs, t0, rx, clk_drift, whole);
        }
    }
    eprintln!("fed {fed_s:.1} s; final:");
    report(&band, &ephs, t0, rx, clk_drift, last_ckpt);
}

#[allow(clippy::too_many_arguments)]
fn report(
    band: &Band,
    ephs: &std::collections::HashMap<u8, hackrf_gnss::gps::broadcast::BrdcEph>,
    t0: f64,
    rx: [f64; 3],
    clk_drift: f64,
    at_s: usize,
) {
    for ch in &band.channels {
        if ch.sys != Sys::Gps {
            continue;
        }
        let Some((t_bit, t_tx)) = ch.anchor else {
            eprintln!("  PRN {:2}: no anchor (bits {})", ch.prn, ch.nav_bits.len());
            continue;
        };
        let Some(eph) = ephs.get(&(ch.prn as u8)) else {
            eprintln!("  PRN {:2}: no truth eph", ch.prn);
            continue;
        };
        let (_, dt_sv, _) = hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, t_tx, rx);
        // sim model (validation/gps_sim.py): the stream runs on the rx clock;
        // at stream time u the rx clock reads t0 + b0 + u while true GPS is
        // t0 + u/(1+d0). The SV transmits the boundary when its clock reads
        // t_tx, i.e. at GPS emission E = t_tx - dt_sv(E); it arrives at GPS
        // A = E + tau_geo, i.e. at stream time u_A = (A - t0)*(1 + d0).
        // Iterate the reception-anchored light-time solve: sat_at_txtime's
        // tow is a GPS RECEPTION time (returns the satellite at tow - tau
        // and dt_sv there); two passes pin A to ps class. (Anchoring the
        // solve at t_tx instead — the pre-fix code — evaluates the satellite
        // ~tau ~ 70 ms from emission and leaves a per-channel range-rate *
        // 70 ms artifact of up to ~50 m in this truth table.)
        let mut a = t_tx - dt_sv + 0.075;
        for _ in 0..2 {
            let (_, d, rng) = hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, a, rx);
            a = t_tx - d + rng / C;
        }
        let t_bit_true = (a - t0) * (1.0 + clk_drift);
        let err_us = (t_bit - t_bit_true) * 1e6;
        eprintln!(
            "  [t={at_s:2}s] PRN {:2}: t_tx {:.0} anchor err {:+10.3} us",
            ch.prn, t_tx, err_us
        );
    }
}
