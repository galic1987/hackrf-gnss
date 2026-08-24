//! Fused multi-satellite sync: common Observation currency, tick/anchor math.
//! Spec: docs/superpowers/specs/2026-08-22-fused-sync-engine-design.md

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Constellation { Iridium, Gps, Glonass, Beidou, Inmarsat, Sbas, Ext1pps }

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ObsKind { DopplerHz, TimeFix, ClockDriftPpm, PhaseNs }

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Observation {
    pub source: Constellation,
    pub sat_id: String,
    /// Session-relative seconds (FPGA tick deltas, not PC wall clock).
    pub t_rel_s: f64,
    pub kind: ObsKind,
    pub value: f64,
    pub aux: f64,
    pub sigma: f64,
    pub capture_id: usize,
}

/// 40-bit FPGA timestamp counter modulus (regs 0x17-0x1B).
pub const TICKS_MOD: u64 = 1 << 40;

/// Counter value for absolute time `utc` given the anchor `tick0_utc`
/// (UTC at counter 0) and the image-dependent `tick_hz`. Wraps mod 2^40.
pub fn ticks_for_utc(utc: f64, tick0_utc: f64, tick_hz: f64) -> u64 {
    let dt = utc - tick0_utc;
    let ticks = (dt * tick_hz).round() as i128;
    ticks.rem_euclid(TICKS_MOD as i128) as u64
}

/// Parse `ts.<src> = N ticks` from hackrf_pro stdout. hackrf_pro exits 0
/// even on failure, so a missing line is the ONLY failure signal.
pub fn parse_ts_read(stdout: &str) -> Option<u64> {
    for line in stdout.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("ts.") {
            let mut parts = rest.split_whitespace(); // "<src>" "=" N "ticks"
            let _src = parts.next()?;
            if parts.next() != Some("=") { continue; }
            if let Some(n) = parts.next().and_then(|p| p.parse::<u64>().ok()) {
                return Some(n);
            }
        }
    }
    None
}

// ---------------------------------------------------------------- adapters

use crate::gps::GpsSat;
use crate::iridium::demod3::IqSample;
use crate::iridium::ppm::{self, PpmEstimate};

/// Decode one Iridium capture into fused observations. `t_rel_start` is the
/// capture's start on the session timeline (from sidecar tick deltas, or 0.0
/// for the first/only capture); `pc_epoch` is the PC-clock start (prior only).
pub fn iridium_observations<S: IqSample>(
    raw: &[S], fc: f64, fs: f64, dur: f64,
    t_rel_start: f64, pc_epoch: f64,
    sats: &[GpsSat], rx: [f64; 3], capture_id: usize,
) -> Vec<Observation> {
    let est = ppm::estimate_ppm(raw, fc, fs, dur, pc_epoch, sats, rx);
    observations_from_estimate(&est, t_rel_start, pc_epoch, dur, capture_id)
}

/// Map an Iridium ppm estimate onto fused observations: one DopplerHz per
/// attributed burst, at most one TimeFix per capture from the median nonzero
/// epoch shift, and one ClockDriftPpm when any burst was attributed.
pub fn observations_from_estimate(
    est: &PpmEstimate, t_rel_start: f64, pc_epoch: f64, dur: f64, capture_id: usize,
) -> Vec<Observation> {
    let mut out = Vec::new();
    for b in &est.per_burst {
        let conf_scale = (b.confidence as f64 / 100.0).max(0.2);
        out.push(Observation {
            source: Constellation::Iridium,
            sat_id: b.sat.clone(),
            t_rel_s: t_rel_start + (b.t_epoch - pc_epoch),
            kind: ObsKind::DopplerHz,
            value: b.f_meas_hz,
            aux: b.f_nom_hz,
            sigma: 50.0 / conf_scale,
            capture_id,
        });
    }
    let shifts: Vec<f64> = est.per_burst.iter()
        .map(|b| b.epoch_shift_s).filter(|&s| s != 0.0).collect();
    if !shifts.is_empty() {
        let mut s = shifts.clone();
        s.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let med = s[s.len() / 2];
        out.push(Observation {
            source: Constellation::Iridium,
            sat_id: "iridium-epoch".into(),
            t_rel_s: t_rel_start,
            kind: ObsKind::TimeFix,
            value: pc_epoch + med,
            aux: 0.0,
            sigma: 5.0,
            capture_id,
        });
    }
    if let Some(p) = est.ppm() {
        out.push(Observation {
            source: Constellation::Iridium,
            sat_id: "iridium-clock".into(),
            t_rel_s: t_rel_start + dur / 2.0,
            kind: ObsKind::ClockDriftPpm,
            value: p,   // positive = clock fast (crate convention)
            aux: 0.0,
            sigma: 0.1,
            capture_id,
        });
    }
    out
}

// ---------------------------------------------------------------- GPS time

use crate::gps::lnav::Subframe;

/// GPS time: unix seconds of GPS epoch 1980-01-06 and the current leap count.
pub const GPS_UNIX_EPOCH: f64 = 315_964_800.0;
pub const GPS_UTC_LEAP_S: f64 = 18.0; // valid 2017-..; revisit if leap seconds change

pub fn tow_to_unix(week: u32, tow_s: f64) -> f64 {
    GPS_UNIX_EPOCH + week as f64 * 604_800.0 + tow_s - GPS_UTC_LEAP_S
}

pub fn current_gps_week(unix_now: f64) -> u32 {
    ((unix_now + GPS_UTC_LEAP_S - GPS_UNIX_EPOCH) / 604_800.0).floor() as u32
}

/// TimeFix from one decoded subframe. `t_rel_mid` is the session time of the
/// middle of the tracked segment (bit-level timing is not recovered, so sigma
/// covers the segment half-length; caller widens sigma accordingly).
pub fn gps_time_observation_from(
    sf: &Subframe, t_rel_mid: f64, pc_now: f64, capture_id: usize,
) -> Option<Observation> {
    if sf.tow_next == 0 || sf.tow_next > 100_799 { return None; }
    let tow_s = (sf.tow_next - 1) as f64 * 6.0;
    let value = tow_to_unix(current_gps_week(pc_now), tow_s);
    // sanity: must be within a week of the PC clock (guards week rollover)
    if (value - t_rel_mid - pc_now).abs() > 604_800.0 { return None; }
    Some(Observation {
        source: Constellation::Gps,
        sat_id: format!("G{:02}", sf.sfid), // sfid, not PRN — identity resolved upstream
        t_rel_s: t_rel_mid,
        kind: ObsKind::TimeFix,
        value, aux: 0.0, sigma: 3.0, capture_id,
    })
}

pub fn gps_time_observation(bits: &[u8], t_rel_mid: f64, pc_now: f64, capture_id: usize) -> Option<Observation> {
    crate::gps::lnav::find_subframes(bits).first()
        .and_then(|sf| gps_time_observation_from(sf, t_rel_mid, pc_now, capture_id))
}

// ------------------------------------------------------ carrier (any band)

/// Measure a stable carrier's apparent frequency error as clock ppm.
/// `samples` are baseband centred on `f_carrier_rf`; the carrier's true
/// nominal RF is `f_nominal`. Peak search is within ±`search_hz` of the
/// expected baseband position. Returns None when the peak is indistinguishable
/// from the loudest noise bin.
///
/// The rejection gate is NOT a fixed "10× median": over a window of K bins the
/// loudest WHITE-NOISE bin is Gumbel with E[max]/median = (ln K + γ)/ln 2,
/// which passes 10× median for any K ≳ a few hundred — a fixed gate accepts
/// pure noise essentially always at useful window widths (measured: the 20 kHz
/// window of the original plan false-accepts deterministic hash noise). The
/// gate is therefore 2× the expected noise max for the actual window, which
/// rejects white noise with P(false accept) ~ 1e-5 while any real carrier a
/// few dB above the noise floor still passes.
///
/// SIGN CONVENTION: value = (f_meas - f_nominal)/f_nominal, the same formula
/// as `iridium::ppm::ppm_from_burst` with Doppler removed, and f_meas is
/// built the same way (tuned centre + measured baseband offset) — so the
/// sign agrees with the Iridium path BY CONSTRUCTION (positive = measured
/// carrier above nominal = "clock fast" in the crate's convention). Whether
/// that matches a given radio's physical LO error is settled on hardware in
/// Task 8, not here.
pub fn carrier_observation(
    samples: &[num_complex::Complex<f32>], fs: f64,
    f_carrier_rf: f64, f_nominal: f64, search_hz: f64,
    t_rel_s: f64, source: Constellation, sat_id: &str, capture_id: usize,
) -> Option<Observation> {
    use rustfft::{FftPlanner, num_complex::Complex32};
    let n = samples.len().min(1 << 20);
    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(n);
    let mut buf: Vec<Complex32> = samples[..n].iter()
        .map(|s| Complex32::new(s.re, s.im)).collect();
    fft.process(&mut buf);
    let bin_hz = fs / n as f64;
    let expected = f_nominal - f_carrier_rf; // baseband Hz (signed)
    let centre_bin = ((expected / bin_hz).round() as i64).rem_euclid(n as i64) as usize;
    let half = (search_hz / bin_hz) as usize;
    let pwr = |b: &Complex32| (b.re * b.re + b.im * b.im) as f64;
    let mut best = (0usize, 0.0f64);
    for k in 0..=2 * half {
        let b = (centre_bin + n - half + k) % n;
        let p = pwr(&buf[b]);
        if p > best.1 { best = (b, p); }
    }
    let mut all: Vec<f64> = buf.iter().map(pwr).collect();
    all.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = all[all.len() / 2].max(1e-20);
    // expected loudest noise bin over the K-bin window (Gumbel); see doc above
    let k_bins = (2 * half + 1) as f64;
    let noise_max = median / std::f64::consts::LN_2 * (k_bins.ln() + 0.5772);
    if best.1 < 2.0 * noise_max { return None; }
    let f_bb_meas = if best.0 as i64 > n as i64 / 2 {
        best.0 as f64 - n as f64
    } else { best.0 as f64 } * bin_hz;
    let f_meas = f_carrier_rf + f_bb_meas;
    Some(Observation {
        source, sat_id: sat_id.into(), t_rel_s,
        kind: ObsKind::ClockDriftPpm,
        value: (f_meas - f_nominal) / f_nominal * 1e6,
        aux: f_meas, sigma: 0.2, capture_id,
    })
}

// ------------------------------------------------------------------- solver

use crate::iridium::geo as geo;

pub struct SyncEstimate {
    pub fix: Option<geo::Fix>,
    /// UTC = t_rel + time_offset_s (None = epoch not anchored).
    pub time_offset_s: Option<f64>,
    /// Positive = clock fast.
    pub drift_ppm: Option<f64>,
    /// UTC at session t_rel = 0 (the FPGA anchor target).
    pub tick0_utc: Option<f64>,
    pub n_obs: usize,
}

pub fn solve(
    obs: &[Observation], sats: &[GpsSat], t0_utc_prior: f64,
    guess: Option<(f64, f64)>,
) -> Result<SyncEstimate, String> {
    if obs.is_empty() { return Err("no observations".into()); }
    let n_caps = obs.iter().map(|o| o.capture_id).max().unwrap_or(0) + 1;

    // position half: DopplerHz -> geo solver (bias-only; rate states are
    // forbidden on short arcs — Global Constraints)
    let geobs: Vec<geo::Obs> = obs.iter()
        .filter(|o| o.kind == ObsKind::DopplerHz)
        .filter_map(|o| {
            let sat = sats.iter().find(|s| s.name == o.sat_id)?;
            Some(geo::Obs {
                t: t0_utc_prior + o.t_rel_s,
                f_meas: o.value, f_nom: o.aux, sat,
                conf: 100, w_scale: 1.0, cap: o.capture_id,
            })
        })
        .collect();
    let fix = if geobs.len() >= 20 {
        let e0 = vec![0.0; n_caps];
        match guess {
            Some((la, lo)) => geo::solve_fix(&geobs, la, lo, &e0, Some(2.0e3), None).ok(),
            None => {
                let ms = geo::multistart_fix(&geobs, &e0, Some(2.0e3), None, &geo::global_grid());
                match ms.best {
                    Some(f) if !ms.ambiguous && f.sigma_km <= 10.0 => Some(f),
                    _ => None,
                }
            }
        }
    } else { None };

    // time half: weighted mean of (utc - t_rel) over TimeFix
    let (mut sw, mut swx) = (0.0f64, 0.0f64);
    for o in obs.iter().filter(|o| o.kind == ObsKind::TimeFix) {
        let w = 1.0 / (o.sigma * o.sigma);
        sw += w; swx += w * (o.value - o.t_rel_s);
    }
    let time_offset_s = (sw > 0.0).then(|| swx / sw);

    // drift: weighted mean of ClockDriftPpm; fallback to geo per-cap clocks
    let (mut dw, mut dwx) = (0.0f64, 0.0f64);
    for o in obs.iter().filter(|o| o.kind == ObsKind::ClockDriftPpm) {
        let w = 1.0 / (o.sigma * o.sigma);
        dw += w; dwx += w * o.value;
    }
    let mut drift_ppm = (dw > 0.0).then(|| dwx / dw);
    if drift_ppm.is_none() {
        if let Some(f) = &fix {
            // geo::Fix.clock_ppm is ALREADY in ppm (geo.rs doc + the
            // doppler_fix tests compare it against ppm-scale truth) — the
            // plan's `* 1e6` here would square the unit
            let mut c: Vec<f64> = f.clock_ppm.iter().copied()
                .filter(|c| !c.is_nan()).collect();
            if !c.is_empty() {
                c.sort_by(|a, b| a.partial_cmp(b).unwrap());
                drift_ppm = Some(c[c.len() / 2]);
            }
        }
    }

    Ok(SyncEstimate {
        fix, time_offset_s, drift_ppm,
        tick0_utc: time_offset_s,
        n_obs: obs.len(),
    })
}

// ------------------------------------------------------- hardware apply
// CLI spelling verified against host/hackrf-tools/src/hackrf_pro.c:
// `--ts-set TICKS` is strtoull base-0 (decimal fine), `--ts-read now` prints
// `ts.<src> = N ticks` on stdout, `--clock-corr PPM` is strtod. The tool
// exits 0 even on failure, so only a missing ts. line signals failure.

pub const HACKRF_PRO: &str =
    "/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_pro";

pub fn ts_set_args(serial: &str, ticks: u64) -> Vec<String> {
    vec!["-d".into(), serial.into(), "--ts-set".into(), ticks.to_string()]
}

pub fn clock_corr_args(serial: &str, ppm: f64) -> Vec<String> {
    vec!["-d".into(), serial.into(), "--clock-corr".into(), ppm.to_string()]
}

pub fn run_pro(args: &[String]) -> Result<String, String> {
    let out = std::process::Command::new(HACKRF_PRO)
        .args(args).output().map_err(|e| format!("spawn hackrf_pro: {e}"))?;
    Ok(String::from_utf8_lossy(&out.stdout).into_owned())
}

/// Inject absolute time. Confirms via --ts-read now; a missing ts. line is
/// failure even though hackrf_pro exits 0.
pub fn apply_ts_set(serial: &str, ticks: u64) -> Result<u64, String> {
    run_pro(&ts_set_args(serial, ticks))?;
    let back = run_pro(&["-d".into(), serial.into(), "--ts-read".into(), "now".into()])?;
    parse_ts_read(&back).ok_or_else(|| format!("ts-read gave no ts. line: {back:?}"))
}

/// Steer the clock correction. Issued twice: the readback is stale by one call.
pub fn apply_clock_corr(serial: &str, ppm: f64) -> Result<(), String> {
    run_pro(&clock_corr_args(serial, ppm))?;
    run_pro(&clock_corr_args(serial, ppm))?;
    Ok(())
}

// ----------------------------------------------------------------- GLONASS

use crate::glonass::{glonass_code, l1_freq, CHIP_RATE as GLO_CHIP_RATE};
use crate::gps::acquire_codes;

/// GLONASS L1OF Doppler adapter. Every satellite transmits the same 511-chip
/// code; FDMA separates them, so each channel k in -7..=6 is mixed to
/// baseband, decimated to ~2 Msps and searched once (thresh 2.5, the survey
/// recipe in examples/glonass_acq.rs). `sig` is baseband centred on `fc`
/// (the whole G1 band fits one capture tuned to 1602 MHz); `t_abs` is the
/// capture's absolute start (unix s), `t_rel_start` its session time.
///
/// A detection is attributed to the TLE satellite whose predicted Doppler at
/// the channel frequency is nearest the measurement, only when that match is
/// within 2.5 kHz AND at least 2x closer than the runner-up (the margin rule
/// mirrors geo::doppler_attribute). No unambiguous attribution -> no
/// observation: never guess.
pub fn glonass_observations(
    sig: &[num_complex::Complex<f32>], fs: f64, fc: f64, t_abs: f64, t_rel_start: f64,
    sats: &[GpsSat], rx: [f64; 3], capture_id: usize,
) -> Vec<Observation> {
    use num_complex::Complex;
    use std::f64::consts::PI;
    if sig.is_empty() { return Vec::new(); }
    let dur = sig.len() as f64 / fs;
    let mean: Complex<f32> = sig.iter().copied().sum::<Complex<f32>>() / sig.len() as f32;
    let q = (fs / 2.0e6).round().max(1.0) as usize; // decimate to ~2 Msps
    let code = glonass_code();
    let dop: Vec<f64> = (-10000..=10000).step_by(250).map(|x| x as f64).collect();
    let nb = (dur * 1000.0) as usize;
    let mut out = Vec::new();
    for k in -7..=6 {
        let f_nom = l1_freq(k);
        let ifhz = f_nom - fc;
        // channel plus the Doppler search grid must fit the Nyquist band
        if ifhz.abs() + 10_000.0 >= fs / 2.0 { continue; }
        let mixed: Vec<Complex<f32>> = sig.iter().enumerate().map(|(i, &v)| {
            let ph = -2.0 * PI * ifhz * i as f64 / fs;
            (v - mean) * Complex::new(ph.cos() as f32, ph.sin() as f32)
        }).collect();
        let sigd = crate::iridium::demod3::decimate(&mixed, q);
        let fsd = fs / q as f64;
        let r = acquire_codes(&sigd, fsd, &[(0usize, code.clone())],
                              GLO_CHIP_RATE, &dop, nb, 2.5, f_nom, true);
        if r.is_empty() || !r[0].acquired { continue; }
        let d_meas = r[0].doppler;
        // attribute: rank visible TLE sats by |predicted - measured|
        let mut cand: Vec<(f64, &str)> = sats.iter().filter_map(|s| {
            let (d, el) = crate::gps::predict_doppler_el_f(s, rx, t_abs, f_nom)?;
            (el > 5.0).then_some(((d - d_meas).abs(), s.name.as_str()))
        }).collect();
        cand.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let Some(&(best_err, name)) = cand.first() else { continue };
        let second = cand.get(1).map(|c| c.0).unwrap_or(f64::INFINITY);
        if best_err > 2500.0 || best_err * 2.0 > second { continue; }
        out.push(Observation {
            source: Constellation::Glonass,
            sat_id: name.into(),
            t_rel_s: t_rel_start + dur / 2.0,
            kind: ObsKind::DopplerHz,
            value: f_nom + d_meas,
            aux: f_nom,
            sigma: 500.0,
            capture_id,
        });
    }
    out
}
