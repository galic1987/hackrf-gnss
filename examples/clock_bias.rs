//! 1 Hz receiver clock-bias series for the sub-ns precision claim (Leg 1, v2).
//!
//! Reads state.tracker.json once per second (the tracker owns the radio; we
//! never touch it), Hatch-smooths each GPS satellite's pseudorange against its
//! published carrier (reset on slip, lock_s regression, or >500 m innovation),
//! solves CLOCK-ONLY with the position fixed at the surveyed site anchor
//! (weighted AND unweighted — the paired A/B), and appends one row to
//! observations/clock_bias.jsonl. Rows:
//! {epoch, clock_ns, clock_ns_uw, residual_rms_m, residual_rms_m_uw, n_sat,
//!  n_fresh, n_pred, slips, gen, source}
//!
//! v2 (2026-08-28 amendment; root causes verified live that morning):
//!  - carrier_cycles integrates the replica NCO with the OPPOSITE sign to
//!    range (all 4 live GPS channels: drho vs λ·Δcarr opposite sign,
//!    magnitudes within ~20%). EVERY carrier use is therefore negated: the
//!    Hatch update gets -carr, and the prediction paths use
//!    λ·(carr_base − carr_now) — the right-signed carrier integral.
//!  - rho_m is a ~6 s STAIRCASE (tracker refreshes at ~1/6 Hz, per-channel
//!    phase; t_tx freezes with it): a bit-frozen rho means "no new code
//!    measurement", NOT pathology — skip the update (never evict; v1's
//!    3-freeze eviction destroyed healthy smoothers) and let the smoother
//!    carrier-predict the range between code updates (the interpolation the
//!    Hatch filter exists for). A sat contributes while its last code update
//!    is < 12 s old and it has had ≥1 fresh update since the last reset.
//!  - the free-position solve leaks m-class × TDOP (7–20 observed live)
//!    noise into clock_ns — the 1 ns gate is unreachable that way. The
//!    position is FIXED at the site anchor (pvt::solve_clock_only, one
//!    unknown, aeae8ec-pattern studentized rejection at the flat 1000 m
//!    gate). No median normalization, as v1.

use hackrf_gnss::gps::broadcast::{parse_rinex_gps, wrap_tk, BrdcEph};
use hackrf_gnss::gps::hatch::Hatch;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::{fs, thread, time::Duration};

const STATE: &str = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json";
const OUT: &str = "/Volumes/Radiator 8TB/gnss/observations/clock_bias.jsonl";
const STATE_CB: &str = "/Volumes/Radiator 8TB/gnss/observations/state.clock_bias.json";
const RINEX: &str = "/Volumes/Radiator 8TB/gnss/observations/brdc_latest.rnx";
const TRACKER_EPH: &str = "/Volumes/Radiator 8TB/gnss/observations/tracker_eph.json";
const SITE_JSON: &str = "/Volumes/Radiator 8TB/gnss/observations/site.json";
const LAM_L1: f64 = 299_792_458.0 / 1_575_420_000.0;
const WINDOW_S: f64 = 100.0;
/// BRDC refresh cadence (position_producer refetches the file hourly).
const EPH_REFRESH_S: f64 = 900.0;
/// A sat's carrier prediction is trusted for two staircase periods.
const PRED_WINDOW_S: f64 = 12.0;

/// Per-PRN smoother output + prediction bookkeeping. The prediction base is
/// the smoother's value and the carrier reading AT THE LAST FRESH CODE
/// UPDATE; between updates the sat's range is the base plus the
/// (right-signed) carrier integral since then.
struct PrevSat {
    /// last seen lock_s (relock detector — the per-sat slip flag does not
    /// fire across relocks, while carrier_cycles restarts its origin)
    lock_s: f64,
    /// last seen rho (freeze detector — bit compare)
    rho_m: f64,
    /// prediction base: smoother value (m) + carrier (cycles) at the last
    /// fresh code update
    base_smoothed: f64,
    base_carr: f64,
    /// file epoch of the last fresh code update (12 s contribution window)
    last_code_epoch: f64,
    /// file epoch at which this PRN was last seen (innovation contiguity
    /// guard — a returning sat's stale chain must not inject a false
    /// innovation)
    file_epoch: f64,
    /// has had ≥1 fresh update since the last reset/clear
    contrib_valid: bool,
}

fn unix_now() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64()
}

/// Canonical ephemeris path, copied from live_fix.rs:237-240 + :272-313 (GPS
/// branch): BRDC from the RINEX file position_producer refreshes, with live
/// self-decoded LNAV ephemerides taking precedence ONLY when they are the
/// newer valid issue (health belt + wrap-aware toe compare — the tracker
/// keeps its first decode per channel and re-stamps the envelope every
/// write, so envelope freshness says nothing about issue freshness).
fn load_ephs() -> HashMap<u8, BrdcEph> {
    let mut ephs = match fs::read_to_string(RINEX) {
        Ok(t) => parse_rinex_gps(&t),
        Err(_) => Default::default(),
    };
    if let Ok(t) = fs::read_to_string(TRACKER_EPH) {
        if let Ok(v) = serde_json::from_str::<Value>(&t) {
            for e in v["ephemeris"].as_array().into_iter().flatten() {
                if let Ok(eph) = serde_json::from_value::<BrdcEph>(e.clone()) {
                    if eph.sys != 0 {
                        continue; // GPS only in this example
                    }
                    // health belt: never merge a KNOWN-unhealthy self-decode
                    if let Some(h) = eph.health.filter(|&h| h != 0) {
                        eprintln!(
                            "clock_bias: self-decoded ephemeris PRN {} rejected — SV health {}",
                            eph.prn, h
                        );
                        continue;
                    }
                    if ephs.get(&eph.prn).is_some_and(|c| wrap_tk(eph.toe - c.toe) <= 0.0) {
                        continue; // the held record is the newer (or same) issue
                    }
                    ephs.insert(eph.prn, eph);
                }
            }
        }
    }
    ephs
}

/// Canonical site anchor (observations/site.json) — NO hardcoded
/// coordinates, copied from live_fix.rs:218-223.
fn site_lla() -> [f64; 3] {
    hackrf_gnss::site::load_site(std::path::Path::new(SITE_JSON)).unwrap_or_else(|| {
        eprintln!("clock_bias: no site anchor — provide {SITE_JSON}");
        std::process::exit(2);
    })
}

/// live_fix's solve guess: site LLA -> ECEF (km). In v2 this is not a guess
/// but THE solve input: the position is fixed at this anchor.
fn site_guess() -> [f64; 3] {
    let lla = site_lla();
    hackrf_gnss::gps::ephemeris::geodetic_to_ecef(lla[0], lla[1], lla[2] / 1000.0)
}

fn build_meas(
    prn: u8,
    rho_m: f64,
    t_tx: f64,
    ephs: &HashMap<u8, BrdcEph>,
    site_m: [f64; 3],
) -> Option<hackrf_gnss::gps::pvt::Meas> {
    // live_fix.rs:574-585 + :678-682 (GPS branch), verbatim except the SBAS
    // terms: this example harvests no SBAS corrections, so prc / iono_m /
    // daf0 are 0 — the (dt_sv + daf0) SV-clock structure is kept.
    let eph = ephs.get(&prn)?;
    let (_, dt0, _) = hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, t_tx, site_m);
    let mut a = t_tx - dt0 + 0.075;
    let (mut sat_m, mut dt_sv) = ([0.0; 3], dt0);
    for _ in 0..2 {
        let (s, d, r) = hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, a, site_m);
        sat_m = s;
        dt_sv = d;
        a = t_tx - d + r / 299_792_458.0;
    }
    let (prc, iono_m, daf0) = (0.0, 0.0, 0.0); // no SBAS corrections here
    Some(hackrf_gnss::gps::pvt::Meas {
        sat: [sat_m[0] / 1000.0, sat_m[1] / 1000.0, sat_m[2] / 1000.0],
        pseudorange: (rho_m + prc - iono_m) / 1000.0 + (dt_sv + daf0) * 299_792.458, // sat clock removed
        clock_free: false,
    })
}

fn main() {
    // ephemeris: same loader live_fix uses (BRDC); refresh every 15 min
    let mut ephs = load_ephs();
    let mut eph_loaded = unix_now();
    // the surveyed site.json anchor: site_m (metres) is the light-time
    // anchor for sat_at_txtime_pub (live_fix's site_m), anchor_km is the
    // FIXED position of the clock-only solve. Computed once — a site.json
    // change is a config change and must restart the producer (new gen).
    let site_m = site_guess().map(|x| x * 1000.0);
    let anchor_km = site_guess();
    // session id (v2 amendment): producer start epoch — restarts/config
    // changes can't silently mix into one series
    let gen_id = format!("v2-{}", unix_now() as u64);
    let mut smoothers: HashMap<u8, Hatch> = HashMap::new();
    let mut prev: HashMap<u8, PrevSat> = HashMap::new();
    let mut last_epoch = 0.0_f64;
    loop {
        thread::sleep(Duration::from_millis(500));
        let txt = match fs::read_to_string(STATE) { Ok(t) => t, Err(_) => continue };
        let st: Value = match serde_json::from_str(&txt) { Ok(v) => v, Err(_) => continue };
        let epoch = st["epoch"].as_f64().unwrap_or(0.0);
        if epoch <= last_epoch { continue; }
        let prev_file_epoch = last_epoch; // last processed file epoch (0.0 before the first)
        last_epoch = epoch;
        let sats = match st["tracker"]["sats"].as_array() { Some(s) => s, None => continue };

        if ephs.is_empty() || unix_now() - eph_loaded > EPH_REFRESH_S {
            ephs = load_ephs();
            eph_loaded = unix_now();
        }

        let mut meas = Vec::new();
        let mut slips = 0u32;
        let mut n_fresh = 0u32;
        let mut n_pred = 0u32;
        for s in sats {
            if s["sys"].as_str() != Some("gps") { continue; }
            if s["cn0_proxy"].as_f64().unwrap_or(0.0) < 30.0 { continue; }
            let lock_s = s["lock_s"].as_f64().unwrap_or(0.0);
            if lock_s < 20.0 { continue; }
            let (rho, t_tx) = match (s["rho_m"].as_f64(), s["t_tx"].as_f64()) {
                (Some(a), Some(b)) => (a, b), _ => continue };
            let prn = s["prn"].as_u64().unwrap_or(0) as u8;
            let carr = s["carrier_cycles"].as_f64().unwrap_or(0.0);
            let slip = s["slip"].as_bool().unwrap_or(false);
            let lock_regressed = prev.get(&prn).is_some_and(|p| lock_s < p.lock_s);

            if prev.get(&prn).is_some_and(|p| rho.to_bits() == p.rho_m.to_bits()) {
                // Frozen rho — the ~6 s staircase: no new code measurement
                // (bit-identical rho is NORMAL, per-channel refresh phase).
                // Skip the update, never evict; carry the sat on the carrier.
                if slip || lock_regressed {
                    // the carrier's integration origin broke while no new
                    // code was available — the prediction chain is void.
                    // Clear the contribution until the next FRESH update
                    // (which re-inits the smoother from code); a reset epoch
                    // counts as a slip so the analyzer keeps it out.
                    smoothers.remove(&prn);
                    if let Some(p) = prev.get_mut(&prn) {
                        p.lock_s = lock_s;
                        p.file_epoch = epoch;
                        p.contrib_valid = false;
                    }
                    slips += 1;
                    continue;
                }
                let p = prev.get_mut(&prn).unwrap();
                if p.contrib_valid && epoch - p.last_code_epoch < PRED_WINDOW_S {
                    // right-signed carrier integral since the last code
                    // update: Δrho = −λ·Δcarr (sign verified live
                    // 2026-08-28, all channels)
                    let rho_used = p.base_smoothed + LAM_L1 * (p.base_carr - carr);
                    // t_tx froze WITH rho (verified live), so project it by
                    // the same interval — the predicted range and the
                    // ephemeris evaluation must refer to the same epoch
                    let t_tx_used = t_tx + (epoch - p.last_code_epoch);
                    meas.push(build_meas(prn, rho_used, t_tx_used, &ephs, site_m));
                    n_pred += 1;
                }
                p.lock_s = lock_s;
                p.file_epoch = epoch;
                continue;
            }

            // Fresh code update.
            // Innovation gate (only when the previous epoch was contiguous):
            // the prediction is the same base the solve carries — code noise
            // is ~10–30 m, so 500 m fires only on a real discontinuity.
            let innov_breach = prev.get(&prn).is_some_and(|p| {
                p.contrib_valid
                    && p.file_epoch == prev_file_epoch
                    && (rho - (p.base_smoothed + LAM_L1 * (p.base_carr - carr))).abs() > 500.0
            });
            let reset = slip || lock_regressed || innov_breach;
            if reset { slips += 1; }
            let h = smoothers.entry(prn).or_insert_with(|| Hatch::new(WINDOW_S));
            // negated carrier — 2026-08-28 live verification (all channels)
            let rho_s = h.update(rho, -carr, LAM_L1, reset);
            n_fresh += 1;
            // the prediction base moves to this update: on fresh epochs the
            // carried range equals the new smoothed value by construction
            prev.insert(prn, PrevSat {
                lock_s,
                rho_m: rho,
                base_smoothed: rho_s,
                base_carr: carr,
                last_code_epoch: epoch,
                file_epoch: epoch,
                contrib_valid: true,
            });
            meas.push(build_meas(prn, rho_s, t_tx, &ephs, site_m));
        }
        let meas: Vec<_> = meas.into_iter().flatten().collect();
        if meas.len() < 5 { continue; }   // redundancy required; exact 4-sat solves are unverifiable
        let fw = hackrf_gnss::gps::pvt::solve_clock_only(&meas, anchor_km, true);
        let fu = hackrf_gnss::gps::pvt::solve_clock_only(&meas, anchor_km, false);
        if let (Some(a), Some(b)) = (fw, fu) {
            let row = json!({
                "epoch": epoch,
                "clock_ns": a.clock_km * 1e9 / 299_792.458,
                "clock_ns_uw": b.clock_km * 1e9 / 299_792.458,
                "residual_rms_m": a.residual_rms_m,
                "residual_rms_m_uw": b.residual_rms_m,
                "n_sat": a.n_sat,
                "n_fresh": n_fresh,
                "n_pred": n_pred,
                "slips": slips,
                "gen": gen_id,
                "source": "clock_bias",
            });
            use std::io::Write;
            let mut f = fs::OpenOptions::new().create(true).append(true).open(OUT).unwrap();
            writeln!(f, "{}", row).unwrap();
            // Atomic per-second state for /api/sync (tmp + rename, like
            // every other producer): the panel reads the live clock bias
            // without parsing the archive. Fail-closed by TTL: when no
            // clean solve exists the file simply expires.
            let st = json!({
                "epoch": epoch,
                "ttl_s": 10,
                "clock_bias": {
                    "clock_ns": row["clock_ns"],
                    "clock_ns_uw": row["clock_ns_uw"],
                    "residual_rms_m": row["residual_rms_m"],
                    "n_sat": row["n_sat"],
                    "n_fresh": row["n_fresh"],
                    "n_pred": row["n_pred"],
                    "slips": row["slips"],
                    "gen": gen_id,
                },
            });
            let tmp = format!("{}.tmp", STATE_CB);
            fs::write(&tmp, st.to_string()).unwrap();
            fs::rename(&tmp, STATE_CB).unwrap();
        }
    }
}
