//! Receiver clock-error estimation from Iridium burst carriers.
//!
//! Every decoded burst gives one equation. The satellite transmits on a known
//! channel centre `f_nom`; its motion adds a Doppler shift `D` we predict from
//! a TLE at the burst's time. The receiver's LO is off by a fractional error
//! `e`, so the measured carrier is
//!
//! ```text
//! f_meas = (f_nom + D) * (1 + e)
//! ```
//!
//! SIGN CONVENTION: positive ppm means the receiver clock runs FAST -- the LO
//! frequency is high, so measured carriers come out HIGHER than the TLE
//! predicts. `hackrf_set_clock_correction` takes the correction to apply,
//! which is the negative of this estimate: a clock measured +25 ppm fast is
//! corrected with -25.
//!
//! The per-burst estimate is noisy (channel quantisation of the transmitter,
//! SGP4 error, ~100 Hz measurement noise), so bursts are combined with a plain
//! median -- a handful of misattributed or mistimed bursts must not move the
//! answer.
//!
//! THE CHANNEL-FOLD TRAP: |D + e*f| regularly exceeds half a channel
//! (20.8 kHz), and then snapping f_meas to the nearest centre picks the WRONG
//! channel and the ppm answer jumps by a whole channel (41666.67 Hz = 25.6
//! ppm at L-band). Observed live: a receding satellite on a disciplined
//! receiver (D = -35 kHz, e ~ 0) read as +25.19 ppm. The fold is STRUCTURALLY
//! ambiguous: shifting every burst's channel by k and the clock by k*25.6 ppm
//! leaves every residual invariant, so no amount of cross-burst statistics
//! resolves it. `estimate_ppm` snaps with the predicted Doppler removed and
//! picks, per burst, the channel implying the smallest |clock error| (the
//! disciplined-device prior); when that changes the answer versus the naive
//! raw snap, `PpmEstimate::ambiguous_ppm` carries the raw-snap alternative.
//! On a VIRGIN radio the true error may be the warned-about value (this
//! station's real -26 ppm was exactly one channel of fold).

/// Iridium channel plan (iridium-toolkit util.py): channels of 1e7/240 Hz
/// above a 1616 MHz base, centres on the half-channel.
pub const BASE_FREQ: f64 = 1616.0e6;
pub const CHANNEL_WIDTH: f64 = 1e7 / (30.0 * 8.0); // 41666.667 Hz

/// Snap a measured carrier to the nearest Iridium channel centre.
pub fn channel_center(f_hz: f64) -> f64 {
    let n = ((f_hz - BASE_FREQ) / CHANNEL_WIDTH).floor();
    BASE_FREQ + (n + 0.5) * CHANNEL_WIDTH
}

/// Fractional clock error solved from one burst, in ppm.
/// `f_meas` is the measured carrier (Hz), `f_nom` the channel centre, `d_hz`
/// the TLE-predicted Doppler at the burst's time (approaching satellite > 0).
pub fn ppm_from_burst(f_meas: f64, f_nom: f64, d_hz: f64) -> f64 {
    (f_meas / (f_nom + d_hz) - 1.0) * 1e6
}

/// Median of a slice, robust by construction. NaN-free input assumed; empty
/// input yields None rather than a made-up zero.
pub fn median(v: &[f64]) -> Option<f64> {
    if v.is_empty() {
        return None;
    }
    let mut s = v.to_vec();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    Some(s[s.len() / 2])
}

/// Iridium spacecraft id from a TLE title like `IRIDIUM 106`. The ring-alert
/// `sat` field and the number in the TLE name are the same spacecraft number
/// for the operational constellation (spares and debris have no such match
/// and simply yield None).
pub fn sat_id_from_name(name: &str) -> Option<u32> {
    let idx = name.find("IRIDIUM")?;
    let digits: String = name[idx + 7..]
        .chars()
        .skip_while(|c| !c.is_ascii_digit())
        .take_while(|c| c.is_ascii_digit())
        .collect();
    digits.parse().ok()
}

// ---------------------------------------------------------------- estimator

use crate::gps::{predict_doppler_el_f, sat_ecef, GpsSat};
use super::demod3::{self, IqSample};
use super::message::{classify_effort, Class, Effort};
use super::parse_line;

/// One burst's contribution to the estimate.
#[derive(Debug, Clone)]
pub struct BurstEst {
    pub sat: String,
    pub ppm: f64,
    pub doppler_hz: f64,
    pub f_meas_hz: f64,
    /// snapped channel centre the burst transmitted on
    pub f_nom_hz: f64,
    /// absolute burst time (unix seconds), epoch-refinement corrected
    pub t_epoch: f64,
    /// demodulator confidence, percent
    pub confidence: u32,
    /// TLE position-match distance, km (-1 when attributed by name number)
    pub match_km: f64,
    /// epoch correction applied to make the position match (0 normally; a
    /// large value means the capture's stated start time was wrong)
    pub epoch_shift_s: f64,
}

/// A decoded burst that could NOT be tied to a satellite by its payload,
/// retained for Doppler-based attribution (see `geo::doppler_attribute`).
#[derive(Debug, Clone)]
pub struct DecodedBurst {
    /// absolute time (unix s), capture start + offset, no epoch refinement
    pub t_epoch: f64,
    pub f_meas_hz: f64,
    /// snapped channel centre
    pub f_nom_hz: f64,
    pub confidence: u32,
}

/// The result of running the estimator over a capture.
#[derive(Debug, Default)]
pub struct PpmEstimate {
    pub detected: usize,
    pub decoded: usize,
    pub unattributed: usize,
    pub per_burst: Vec<BurstEst>,
    /// decoded but unattributed bursts (payload gave no usable satellite)
    pub decoded_bursts: Vec<DecodedBurst>,
    /// detected bursts that never decoded but still carry a fine carrier:
    /// the preamble FFT (the demodulator's own early stages) yields the
    /// frequency to tens of Hz without a clean frame. Confidence is 0 — the
    /// burst is unverified as Iridium until Doppler attribution accepts it.
    pub detected_fine: Vec<DecodedBurst>,
    /// median raw channel offset over decoded bursts; the fallback diagnostic
    /// when nothing could be attributed (LO error PLUS mean Doppler)
    pub fallback_median_offset_hz: Option<f64>,
    /// if the channel-fold resolution tied, the runner-up clock hypothesis
    /// (ppm). The chosen estimate and this differ by ~one channel (25.6 ppm);
    /// only an external fact (a prior correction, a second channel in use)
    /// can separate them.
    pub ambiguous_ppm: Option<f64>,
}

/// Fine carrier of a burst that never decoded: baseband mix + envelope edges
/// + the unmodulated-preamble FFT (the demodulator's own early stages).
/// Returns (f_meas_hz, preamble SNR dB). None without a clean preamble tone.
fn fine_carrier<S: IqSample>(raw: &[S], b: &demod3::Burst, fc: f64, fs: f64) -> Option<(f64, f64)> {
    let (y, fsy) = demod3::load_bb(raw, b.t, b.dur, b.fcen, fc, fs, 0.0015)?;
    let (i0, _) = demod3::burst_edges(&y, fsy)?;
    let (df, psnr) = demod3::preamble_carrier(&y, fsy, i0, 16)?;
    (psnr >= 10.0).then_some((b.fcen + df, psnr))
}

impl PpmEstimate {
    /// Robust clock error over all attributed bursts, ppm. None if none.
    pub fn ppm(&self) -> Option<f64> {
        median(&self.per_burst.iter().map(|b| b.ppm).collect::<Vec<_>>())
    }
    /// The delta to ADD to the radio's current clock correction to zero the
    /// measured residual: the negative of the measured error.
    pub fn correction_delta(&self) -> Option<f64> {
        self.ppm().map(|p| -p)
    }
    /// Distinct satellites that contributed, sorted.
    pub fn sats(&self) -> Vec<String> {
        let mut v: Vec<String> = self.per_burst.iter().map(|b| b.sat.clone()).collect();
        v.sort();
        v.dedup();
        v
    }
}

fn dist_km(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// Identify a ring alert's satellite. Ring alerts alternate between the
/// spacecraft's own position and a spot-beam surface point; only the former
/// (orbital radius, alt_km > 7100) identifies the satellite, and IRA sat
/// numbers are NOT the TLE name numbers for spares and replacements, so the
/// position is matched against every propagated TLE -- the same thing
/// iridium-toolkit's satmap does. Surface-point ring alerts and broadcast
/// frames fall back to the name number and the elevation gate.
///
/// Returns (satellite, match distance km, epoch shift s). The shift is 0 when
/// the match succeeds at the supplied epoch; see `match_position` for when it
/// is not.
fn attribute<'a>(cls: &Class, sats: &'a [GpsSat], t: f64) -> Option<(&'a GpsSat, f64, f64)> {
    let by_name = |id: u32| sats.iter().find(|s| sat_id_from_name(&s.name) == Some(id));
    match cls {
        Class::RingAlert(ira) if ira.alt_km > 7100.0 => {
            // alt_km is the geocentric RADIUS (see message::Ira), not a height:
            // build the query point geocentrically. Passing it to
            // geodetic_to_ecef as an altitude puts the point one Earth radius
            // above the orbital shell and every match misses by ~6370 km.
            let (la, lo) = (ira.lat_deg.to_radians(), ira.lon_deg.to_radians());
            let p = [
                ira.alt_km * la.cos() * lo.cos(),
                ira.alt_km * la.cos() * lo.sin(),
                ira.alt_km * la.sin(),
            ];
            match_position(sats, &p, t)
        }
        Class::RingAlert(ira) => by_name(ira.sat).map(|s| (s, -1.0, 0.0)),
        Class::Broadcast(ibc) => by_name(ibc.sv_id).map(|s| (s, -1.0, 0.0)),
        _ => None,
    }
}

/// Nearest TLE satellite to `p` (ECEF km) at time `t`, with an epoch-refinement
/// fallback. A capture whose stated start is wrong by tens of seconds (a
/// command timestamp logged instead of the transfer start, an mtime guess on
/// a slowly-written file) misses the position gate at ~7.5 km/s along-track
/// per second of error while being geometrically perfect. The burst's
/// self-reported position doubles as a clock: if the exact-epoch match fails
/// the 200 km gate, search +-150 s in 10 s steps and accept the best only
/// below a tighter 100 km gate. Returns (sat, distance km, epoch shift s).
pub fn match_position<'a>(sats: &'a [GpsSat], p: &[f64; 3], t: f64) -> Option<(&'a GpsSat, f64, f64)> {
    let best_at = |shift: f64| -> Option<(f64, &'a GpsSat)> {
        sats.iter()
            .filter_map(|s| sat_ecef(s, t + shift).map(|e| (dist_km(p, &e), s)))
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
    };
    if let Some((d, s)) = best_at(0.0) {
        if d < 200.0 {
            return Some((s, d, 0.0));
        }
    }
    let mut best: Option<(f64, &GpsSat, f64)> = None;
    let mut k = -15i32;
    while k <= 15 {
        if k != 0 {
            let shift = k as f64 * 10.0;
            if let Some((d, s)) = best_at(shift) {
                if best.as_ref().is_none_or(|b| d < b.0) {
                    best = Some((d, s, shift));
                }
            }
        }
        k += 1;
    }
    best.filter(|b| b.0 < 100.0).map(|b| (b.1, b.0, b.2))
}

/// Estimate the receiver clock error from an Iridium capture: find bursts,
/// demodulate, attribute to satellites, solve per-burst ppm, median.
/// `start_epoch` is the capture's absolute start (unix seconds) and must be
/// true to a few seconds -- the position match and the elevation gate both
/// depend on it. `rx` is the station ECEF (km).
pub fn estimate_ppm<S: IqSample>(
    raw: &[S],
    fc: f64,
    fs: f64,
    dur: f64,
    start_epoch: f64,
    sats: &[GpsSat],
    rx: [f64; 3],
) -> PpmEstimate {
    let bursts = demod3::find_bursts2(raw, fc, fs, dur, 1626.0e6, 1626.5e6, 4.0);
    let mut est = PpmEstimate { detected: bursts.len(), ..PpmEstimate::default() };
    let mut raw_off: Vec<f64> = Vec::new();
    // pass 1: attribute and propagate; keep the three channel candidates
    // (predicted Doppler removed) per burst for the fold resolution below
    struct Row {
        sat: String,
        dop: f64,
        f_meas: f64,
        t: f64,
        conf: u32,
        sep: f64,
        shift: f64,
        cand: [(f64, f64); 3], // (f_nom, ppm) for channel offsets -1, 0, +1
    }
    let mut rows: Vec<Row> = Vec::new();
    for b in &bursts {
        let Some(d) = demod3::demod_burst_debug(raw, b.t, b.dur, b.fcen, fc, fs) else {
            // never demodulated at all: the preamble tone still gives a carrier
            if let Some((f, _psnr)) = fine_carrier(raw, b, fc, fs) {
                est.detected_fine.push(DecodedBurst {
                    t_epoch: start_epoch + b.t,
                    f_meas_hz: f,
                    f_nom_hz: channel_center(f),
                    confidence: 0,
                });
            }
            continue;
        };
        let Some(rwa) = &d.rwa else {
            // demodulated but no frame survived: df is still the fine carrier
            let f = b.fcen + d.df;
            est.detected_fine.push(DecodedBurst {
                t_epoch: start_epoch + b.t,
                f_meas_hz: f,
                f_nom_hz: channel_center(f),
                confidence: 0,
            });
            continue;
        };
        let Some(fr) = parse_line(rwa) else { continue };
        est.decoded += 1;
        let f_meas = b.fcen + d.df;
        raw_off.push(f_meas - channel_center(f_meas));
        // Harder effort: misattributed bursts are outliers here, and the median
        // plus the below-horizon rejection are the downstream TLE cross-check
        // that makes that mode safe (see message::Effort).
        let cls = classify_effort(&fr.bits, Some(f_meas), Effort::Harder);
        let t = start_epoch + b.t;
        let db = DecodedBurst {
            t_epoch: t,
            f_meas_hz: f_meas,
            f_nom_hz: channel_center(f_meas),
            confidence: fr.confidence,
        };
        let Some((sat, sep, shift)) = attribute(&cls, sats, t) else {
            est.unattributed += 1;
            est.decoded_bursts.push(db);
            continue;
        };
        // a nonzero shift means the capture's stated epoch was off; the
        // Doppler and the elevation gate both run at the corrected time
        let t = t + shift;
        let f0 = channel_center(f_meas);
        let Some((dop, el)) = predict_doppler_el_f(sat, rx, t, f0) else { continue };
        if el < 0.0 {
            eprintln!("t+{:7.3} {} below horizon (el {:.0}) -- skipped", b.t, sat.name, el);
            est.unattributed += 1;
            est.decoded_bursts.push(db);
            continue;
        }
        let fdc = channel_center(f_meas - dop);
        let cand: [(f64, f64); 3] = [-1.0, 0.0, 1.0].map(|k| {
            let f_nom = fdc + k * CHANNEL_WIDTH;
            (f_nom, ppm_from_burst(f_meas, f_nom, dop))
        });
        rows.push(Row { sat: sat.name.clone(), dop, f_meas, t, conf: fr.confidence, sep, shift, cand });
    }
    // pass 2: the channel fold is STRUCTURALLY ambiguous -- shifting every
    // burst's channel by k and the clock by k*25.6 ppm leaves every residual
    // invariant, so no clustering can resolve it from the data. Resolve per
    // burst toward the smallest |clock error| (the disciplined-device prior)
    // and, when that changes the answer versus the naive raw snap, report the
    // raw-snap alternative: on a VIRGIN radio the true error may be the
    // warned-about one (this device's -26 ppm was one channel of fold).
    let mut raw_ppms: Vec<f64> = Vec::new();
    for r in rows {
        let &(f_nom, ppm) = r
            .cand
            .iter()
            .min_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .unwrap();
        raw_ppms.push(ppm_from_burst(r.f_meas, channel_center(r.f_meas), r.dop));
        est.per_burst.push(BurstEst {
            sat: r.sat,
            ppm,
            doppler_hz: r.dop,
            f_meas_hz: r.f_meas,
            f_nom_hz: f_nom,
            t_epoch: r.t,
            confidence: r.conf,
            match_km: r.sep,
            epoch_shift_s: r.shift,
        });
    }
    if let (Some(chosen), Some(raw)) = (est.ppm(), median(&raw_ppms)) {
        if (chosen - raw).abs() > 3.0 {
            est.ambiguous_ppm = Some(raw);
        }
    }
    est.fallback_median_offset_hz = median(&raw_off);
    est
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channel_centres_snap_to_the_grid() {
        // ring-alert band: 1626.020833 MHz is the first simplex centre
        let f0 = channel_center(1626.020833e6 + 900.0);
        assert!((f0 - 1626.020833e6).abs() < 1.0, "{f0}");
        // just below the next channel boundary still snaps down
        let f1 = channel_center(1626.020833e6 + 20000.0);
        assert!((f1 - 1626.020833e6).abs() < 1.0);
        // and above it snaps up by one channel width
        let f2 = channel_center(1626.020833e6 + 21000.0);
        assert!((f2 - (1626.020833e6 + CHANNEL_WIDTH)).abs() < 1.0);
    }

    #[test]
    fn solver_recovers_a_known_clock_error() {
        let f_nom = channel_center(1626.25e6);
        // approaching satellite, 2.5 km/s range rate -> Doppler up
        let d = f_nom * 2500.0 / 299_792_458.0;
        // receiver clock 25 ppm fast: measured = true * (1 + 25e-6)
        let f_meas = (f_nom + d) * (1.0 + 25e-6);
        let ppm = ppm_from_burst(f_meas, f_nom, d);
        assert!((ppm - 25.0).abs() < 1e-3, "ppm {ppm}");
    }

    #[test]
    fn positive_ppm_means_clock_fast_and_frequency_high() {
        let f_nom = 1626.25e6;
        // clock fast by +10 ppm -> measured carrier ABOVE prediction
        let ppm = ppm_from_burst(f_nom * (1.0 + 10e-6), f_nom, 0.0);
        assert!(ppm > 0.0 && (ppm - 10.0).abs() < 1e-6, "ppm {ppm}");
        // clock slow -> measured below prediction -> negative ppm
        let ppm = ppm_from_burst(f_nom * (1.0 - 10e-6), f_nom, 0.0);
        assert!(ppm < 0.0 && (ppm + 10.0).abs() < 1e-6, "ppm {ppm}");
        // unaccounted Doppler masquerades as clock error, with its sign
        let d = -20e3; // receding satellite
        let f_meas = f_nom + d;
        assert!(ppm_from_burst(f_meas, f_nom, 0.0) < 0.0);
    }

    #[test]
    fn median_ignores_outliers() {
        let v = [10.0, 11.0, 12.0, 13.0, 500.0, -400.0];
        let m = median(&v).unwrap();
        assert!((m - 12.0).abs() < 1e-9 || (m - 13.0).abs() < 1e-9, "median {m}");
        // one wildly wrong burst cannot drag the estimate off the pack
        let mut v: Vec<f64> = vec![20.0; 21];
        v.push(9_999.0);
        assert!((median(&v).unwrap() - 20.0).abs() < 1e-9);
        assert!(median(&[]).is_none());
    }

    #[test]
    fn tle_titles_give_spacecraft_ids() {
        assert_eq!(sat_id_from_name("IRIDIUM 106"), Some(106));
        assert_eq!(sat_id_from_name("IRIDIUM 33"), Some(33));
        assert_eq!(sat_id_from_name("IRIDIUM 911 DEB"), Some(911));
        assert_eq!(sat_id_from_name("GPS BIIR-5  (PRN 22)"), None);
        assert_eq!(sat_id_from_name("IRIDIUM"), None);
    }

    #[test]
    fn a_receding_satellite_folds_the_naive_snap_one_channel() {
        // the live failure: D = -35 kHz on a disciplined receiver read as
        // +25.19 ppm through the raw snap -- one full channel of fold
        let f_nom = channel_center(1626.25e6);
        let d = -35e3;
        let e = -0.4e-6;
        let f_meas = (f_nom + d) * (1.0 + e);
        let naive = channel_center(f_meas);
        assert!((naive - (f_nom - CHANNEL_WIDTH)).abs() < 1.0, "naive snap folded low");
        let naive_ppm = ppm_from_burst(f_meas, naive, d);
        let chan_ppm = 1e6 * CHANNEL_WIDTH / f_nom;
        assert!((naive_ppm - (e * 1e6 + chan_ppm)).abs() < 0.5, "{naive_ppm}");
        // snapping with the Doppler removed lands on the true channel
        let corrected = channel_center(f_meas - d);
        assert!((corrected - f_nom).abs() < 1.0);
        assert!((ppm_from_burst(f_meas, corrected, d) - e * 1e6).abs() < 1e-3);
    }
}
