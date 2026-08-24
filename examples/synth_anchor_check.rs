//! Synthetic-truth anchor bench: feed the 30 s gpssim capture (s45.iq, 8 Msps
//! complex int8, zero IF, 7 GPS sats at 45 dB-Hz) through a real live `Band`
//! and score every anchor against the simulation's own truth:
//!     predicted t_bit = (t_tx + |rx - sat(t_tx)|/c + clk(t)) - t0_gps
//! Any per-channel offset beyond ~1 us is an anchor-chain bug, measured
//! against truth with no ephemeris or site error in the loop.
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
    let clk_bias = truth["clk_bias_s"].as_f64().unwrap();
    let clk_drift = truth["clk_drift"].as_f64().unwrap();
    let t_ref = truth["t_ref_rxclock_s"].as_f64().unwrap();
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
            },
        );
    }

    let fs = 8.0e6;
    let epoch0 = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();
    let mut band = Band::new_l1(fs, epoch0);
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
    while fed_s < 30.0 {
        let n = f.read(&mut raw).unwrap_or(0);
        if n == 0 {
            break;
        }
        let ns = n / 2;
        let sig: Vec<Complex<f32>> = raw[..ns * 2]
            .chunks_exact(2)
            .map(|c| Complex::new(c[0] as i8 as f32, c[1] as i8 as f32))
            .collect();
        band.push(&sig);
        fed_s += ns as f64 / fs;
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
            report(&band, &ephs, t0, rx, clk_bias, clk_drift, t_ref, whole);
        }
    }
    eprintln!("fed {fed_s:.1} s; final:");
    report(&band, &ephs, t0, rx, clk_bias, clk_drift, t_ref, last_ckpt);
}

#[allow(clippy::too_many_arguments)]
fn report(
    band: &Band,
    ephs: &std::collections::HashMap<u8, hackrf_gnss::gps::broadcast::BrdcEph>,
    t0: f64,
    rx: [f64; 3],
    clk_bias: f64,
    clk_drift: f64,
    t_ref: f64,
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
        let (sat_m, dt_sv, _) = hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, t_tx, [0.0; 3]);
        let geom = ((rx[0] - sat_m[0]).powi(2)
            + (rx[1] - sat_m[1]).powi(2)
            + (rx[2] - sat_m[2]).powi(2))
        .sqrt();
        // sim: receiver clock reads GPS + bias + drift*(t-t_ref); the sample
        // stream runs on the receiver clock. The SV transmits the boundary
        // when its clock reads t_tx, i.e. at GPS time t_tx - dt_sv, so the
        // boundary arrives at stream time (t_tx - dt_sv + geom/c + clk) - t0
        let t_rx_gps = t_tx + geom / C;
        let clk = clk_bias + clk_drift * (t_rx_gps - t_ref);
        let t_bit_true = t_tx - dt_sv + geom / C + clk - t0;
        let err_us = (t_bit - t_bit_true) * 1e6;
        eprintln!(
            "  [t={at_s:2}s] PRN {:2}: t_tx {:.0} anchor err {:+10.3} us",
            ch.prn, t_tx, err_us
        );
    }
}
