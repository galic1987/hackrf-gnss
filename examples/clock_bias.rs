//! 1 Hz receiver clock-bias series for the sub-ns precision claim (Leg 1, v4).
//!
//! Reads state.tracker.json once per second (the tracker owns the radio; we
//! never touch it), Hatch-smooths each locked GPS+BDS satellite's pseudorange
//! against its published carrier (reset on slip, lock_s regression, or >500 m
//! innovation), solves CLOCK-ONLY with the position fixed at the surveyed site
//! anchor (weighted AND unweighted — the paired A/B), and appends one row to
//! observations/clock_bias.jsonl. Rows:
//! {schema, epoch, clock_ns, clock_ns_uw, residual_rms_m,
//!  residual_rms_m_uw, n_sat, n_gps, n_bds, n_sbas, n_bds_pre_reject,
//!  ab_membership_match, n_sat_weighted, n_sat_unweighted, n_fresh,
//!  n_pred, slips, gen, source} plus optional geo_ranging/dropped_slip
//! (see the v4 additive amendment below). n_gps/n_bds/n_sbas describe the
//! weighted solver's accepted post-rejection set.
//!
//! v4 additive amendment (2026-09-02, availability work order): schema
//! string stays clock_bias-v4, new fields are ADDITIVE ONLY.
//!  - SBAS GEO ranging (lever 2): sys=sbas rows for PRN 131/135 join the
//!    solve. The tracker anchors their pseudorange to the 1 s SBAS block
//!    boundary (src/live.rs Sys::Sbas nav arm); satellite state/clock come
//!    from the tracker-published MT9 (sbas_geonav) via sbas::geo_at_txtime
//!    (DO-229D A.4.5.1 2nd-order Taylor + agf0/agf1, the audited python
//!    GeoCorrector math). Accepted GEO PRNs are listed additively as
//!    geo_ranging: [prns] and counted in n_sbas so leg-1 weighting can
//!    distinguish the ~10 m-class GEO ranging bias later; the residual
//!    rejection loop is NOT relaxed for them.
//!  - Slip-tolerant gate (lever 3b): when a satellite CONTRIBUTING to this
//!    epoch's solve had a smoother reset (tracker slip, lock regression,
//!    or innovation breach — exactly the events the `slips` counter
//!    counts) and the pre-drop measurement count is >= 6, that satellite
//!    is excluded and the epoch solved on the remainder (floor 4); the
//!    row records dropped_slip: ["G..",..] and its `slips` count excludes
//!    the dropped resets. Below 6 (or if dropping would go under 4) the
//!    previous behavior is unchanged (publish with slips counted).
//!  - A/B flag-not-suppress (lever 6): an accepted_indices mismatch
//!    between the weighted and unweighted solves now PUBLISHES the epoch
//!    with ab_membership_match:false plus n_sat_weighted/n_sat_unweighted
//!    instead of silently skipping it; the analyzer keeps such rows out
//!    of claims (quality filter) — visible in data, excluded from claims.
//!    On mismatch rows the n_sat/n_gps/n_bds/n_sbas identity fields
//!    describe the WEIGHTED solve's accepted set.
//!  - Galileo E1B I/NAV ranging (lever 1, 2026-09-02 spec): sys=galileo
//!    rows join the solve. Ephemeris/clock from the same BRDC RINEX via
//!    gps::broadcast::parse_rinex_gal (I/NAV-clocked records only — Data
//!    Sources bit 9 + bit 0|2; the F/NAV twin's (E1,E5a) clock is wrong
//!    for E1 — plus fail-closed SISA and E1-B health gates), evaluated by
//!    sat_at_txtime_gal (GAL mu/F constants; BGD(E1,E5b) subtracted inside
//!    the clock per ICD Eq. 17, exactly like -tgd). E1 is 1575.42 MHz ==
//!    L1: lambda_L1 carrier, SBAS IGP iono UNSCALED (a GPS-L1-certified
//!    product applied to an uncertified constellation at the same
//!    frequency — physically sound, uncertified) + the same tropo; no
//!    SBAS PRC/LT/DNU (GPS-PRN products). GGTO per spec §5.4 Option B:
//!    the tracker publishes the broadcast word-10 offset as an OPTIONAL
//!    additive ggto_ns per sat row; when present and finite the anchor
//!    converts t_tx_gpst = t_tx_gst - ggto, else zero is applied
//!    fail-closed and the raw GGTO (tens of ns) folds into the same
//!    unmodeled ISB the BDS rows ride. New additive row fields: n_gal,
//!    n_gal_pre_reject, ggto_applied; n_gps is now COUNTED from the
//!    accepted set (it was derived n_sat - n_bds - n_sbas, which would
//!    silently misattribute GAL rows to GPS). Carrier sign inherited
//!    (same tracker NCO machinery; live GAL sign verification pending,
//!    like BDS).
//!
//! v4 (2026-08-30): post-rejection constellation identity and a strict
//! paired-A/B membership gate. v3 (2026-08-29) added the GPS+BDS vector.
//! The motivation is
//! AVAILABILITY: the GPS-only build rode the >=5-measurement gate at exactly
//! n = 5 for 63% of rows — one bird dropping killed every clean run
//! (2026-08-29 diagnosis: 75.9% outage time, best clean run 764 s = 21% of
//! the 3600 s claim gate). The tracker locks BDS B1I alongside GPS (C32/C37
//! at this writing) with published rho_m/t_tx, and the hourly BRDC already
//! carries the C records — adding BDS raises the typical n to ~8.
//!  - frame handling: the tracker's t_tx is GPST for BOTH constellations
//!    (converted at the anchor: BDT + 14 s, src/live.rs sow_bdt_to_gpst),
//!    and beidou_d1::parse_rinex_bds stores toe/toc GPST-equivalent (+14 s),
//!    so the orbit/clock evaluation is uniform — sat_at_txtime_bds takes the
//!    same GPST t_tx as the GPS path.
//!  - BDS rows get no SBAS prc/iono terms (those are GPS-L1 products); the
//!    BDS sat clock already carries TGD1 (the B1I group delay,
//!    beidou_d1::sat_clock_bds).
//!  - CAVEAT: ONE clock state for what is now FOUR constellations (GPS,
//!    BDS, SBAS GEO, GAL — this bullet said "two" when only BDS had
//!    joined) — the inter-system channel biases are UNMODELED in this 1-state solve (the
//!    solver is a weighted mean with studentized rejection; ISB surgery is
//!    out of scope). Rows carry accepted n_gps/n_bds so the analyzer can
//!    quantify mix-dependence; a structurally-biased BDS row that trips the 1000 m
//!    studentized gate is legitimately DROPPED — fail-closed, not silent.
//!  - BDS carrier prediction uses the B1I wavelength (1561.098 MHz) under
//!    the same staircase rules; the negated-carrier sign is inherited from
//!    the 2026-08-28 GPS verification (same tracker NCO machinery — live
//!    BDS sign verification pending, the shadow validator is GPS-only).
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

use hackrf_gnss::beidou_d1::{parse_rinex_bds, sat_at_txtime_bds};
use hackrf_gnss::gps::broadcast::{parse_rinex_gal, parse_rinex_gps, sat_at_txtime_gal, wrap_tk, BrdcEph};
use hackrf_gnss::gps::hatch::Hatch;
use hackrf_gnss::gps::pvt::ClockFix;
use hackrf_gnss::sbas::{geo_at_txtime, GeoEph};
use serde_json::{json, Value};
use std::collections::HashMap;
use std::{fs, thread, time::Duration};

const STATE: &str = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json";
const OUT: &str = "/Volumes/Radiator 8TB/gnss/observations/clock_bias.jsonl";
const STATE_CB: &str = "/Volumes/Radiator 8TB/gnss/observations/state.clock_bias.json";
const TRACKER_BIN: &str = "/Volumes/Radiator 8TB/gnss/hackrf_gnss/target/release/examples/live_radio";
const RINEX: &str = "/Volumes/Radiator 8TB/gnss/observations/brdc_latest.rnx";
const TRACKER_EPH: &str = "/Volumes/Radiator 8TB/gnss/observations/tracker_eph.json";
const SITE_JSON: &str = "/Volumes/Radiator 8TB/gnss/observations/site.json";
const LAM_L1: f64 = 299_792_458.0 / 1_575_420_000.0;
/// BDS B1I carrier wavelength (1561.098 MHz, BDS-SIS-ICD-B1I §2.2.1) — the
/// BDS channels integrate carrier in B1I cycles, so their prediction
/// integral uses λ_B1I, not λ_L1 (~0.9% scale difference).
const LAM_B1I: f64 = 299_792_458.0 / 1_561_098_000.0;
const WINDOW_S: f64 = 20.0;
/// BRDC refresh cadence (position_producer refetches the file hourly).
const EPH_REFRESH_S: f64 = 900.0;
/// A sat's carrier prediction is trusted for two staircase periods.
const PRED_WINDOW_S: f64 = 12.0;
/// The WAAS GEOs this station ranges on (lever 2). The gate is a
/// whitelist, not a Sys filter: only birds whose MT9 stream this station
/// has actually observed and whose LOS physics were sanity-checked are
/// admitted.
const GEO_RANGING_PRNS: [u8; 2] = [131, 135];
/// Slip-tolerant gate floor (lever 3b): only with this many built
/// measurements may a slipped contributor be dropped and the epoch
/// re-solved; below it, behavior is unchanged (publish with slips
/// counted). 6 pre-drop leaves >= 5 after a single drop — above both the
/// solver's 4-floor and the analyzer's n_sat >= 5 quality gate.
const SLIP_DROP_MIN_PRE: usize = 6;
/// DO-229D Table 2-1 MT9 timeout (en-route/terminal): a GEO vector whose
/// |t − t0| exceeds this is stale — refuse to range on it (fail-closed).
const GEO_MAX_DT_S: f64 = 360.0;
/// Galileo ephemeris age gate (fail-closed): |t_tx - toe| beyond this
/// refuses to range. 4 h is the I/NAV nominal data validity (the batch
/// refreshes every ~10-180 min; RINEX E records carry no fit interval, so
/// the gate is explicit here rather than in selection). Wrap-aware.
const GAL_EPH_MAX_AGE_S: f64 = 4.0 * 3600.0;

/// Constellation of one built measurement (the per-row identity the
/// accepted-set counters and the dropped_slip labels are derived from).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Cls {
    Gps,
    Bds,
    Sbas,
    Gal,
}

impl Cls {
    /// RINEX-style satellite label ("G12" / "C32" / "S131" / "E05") —
    /// dropped_slip rows carry these instead of bare PRNs because PRNs
    /// collide across constellations (the live sky holds GPS 32 and BDS 32
    /// simultaneously; GAL E1B PRNs 1-36 overlap GPS 1-32 the same way).
    fn label(self, prn: u8) -> String {
        match self {
            Cls::Gps => format!("G{prn:02}"),
            Cls::Bds => format!("C{prn:02}"),
            Cls::Sbas => format!("S{prn}"),
            Cls::Gal => format!("E{prn:02}"),
        }
    }
}

/// GGTO conversion at the anchor (spec §5.4 Option B): the tracker
/// publishes the broadcast word-10 offset dt_systems = t_Galileo - t_GPS
/// as an OPTIONAL ggto_ns SatReport field (null when word 10 is absent or
/// carries the ICD §5.1.8 all-ones invalid sentinel — never 0.0-as-unknown).
/// When present and finite: t_tx_gpst = t_tx_gst - dt_systems; otherwise
/// apply ZERO fail-closed and let the raw GGTO fold into the unmodeled ISB
/// the analyzer quantifies via n_gal. Returns (t_tx_gpst, applied).
fn ggto_convert(t_tx_gst: f64, ggto_ns: Option<f64>) -> (f64, bool) {
    match ggto_ns {
        Some(g) if g.is_finite() => (t_tx_gst - g * 1e-9, true),
        _ => (t_tx_gst, false),
    }
}

/// Lever 3b decision, pure for the window tests: indices of measurements
/// to drop before the solve. Non-empty only when the pre-drop count is
/// >= SLIP_DROP_MIN_PRE, at least one contributor slipped, and dropping
/// every slipped contributor still leaves the solver's 4-sat floor.
fn plan_slip_drop(slipped: &[bool]) -> Vec<usize> {
    let n_slip = slipped.iter().filter(|&&s| s).count();
    if slipped.len() >= SLIP_DROP_MIN_PRE && n_slip > 0 && slipped.len() - n_slip >= 4 {
        (0..slipped.len()).filter(|&i| slipped[i]).collect()
    } else {
        Vec::new()
    }
}

/// Assemble the jsonl epoch row (pure for the window tests). `cls`/`prn`/
/// `ggto` describe the measurement slice BOTH solves consumed (post
/// slip-drop; `ggto[i]` = measurement i's anchor used a broadcast GGTO);
/// identity counters (n_sat/n_gps/n_bds/n_sbas/n_gal), geo_ranging and
/// ggto_applied come from the WEIGHTED solve's accepted set — on an A/B
/// membership mismatch the row still publishes (lever 6) with
/// ab_membership_match:false and both n_sat_weighted/n_sat_unweighted, and
/// the analyzer keeps it out of claims. `slips` must already exclude the
/// dropped resets. n_gps is COUNTED (never derived by subtraction — the
/// pre-GAL derivation would have misattributed GAL rows to GPS), so
/// n_gps + n_bds + n_sbas + n_gal == n_sat holds by construction — the
/// analyzer's generalized identity gate.
#[allow(clippy::too_many_arguments)]
fn epoch_row(
    a: &ClockFix,
    b: &ClockFix,
    cls: &[Cls],
    prn: &[u8],
    ggto: &[bool],
    epoch: f64,
    n_bds_pre_reject: u32,
    n_gal_pre_reject: u32,
    n_fresh: u32,
    n_pred: u32,
    slips: u32,
    dropped_slip: &[String],
    gen_id: &str,
) -> Value {
    let ab_match = a.accepted_indices == b.accepted_indices;
    let count = |c: Cls| a.accepted_indices.iter().filter(|&&i| cls[i] == c).count();
    let n_gps = count(Cls::Gps);
    let n_bds = count(Cls::Bds);
    let n_sbas = count(Cls::Sbas);
    let n_gal = count(Cls::Gal);
    let ggto_applied = a
        .accepted_indices
        .iter()
        .any(|&i| cls[i] == Cls::Gal && ggto[i]);
    let geo_ranging: Vec<u8> = a
        .accepted_indices
        .iter()
        .filter(|&&i| cls[i] == Cls::Sbas)
        .map(|&i| prn[i])
        .collect();
    let mut row = json!({
        "schema": "clock_bias-v4",
        "epoch": epoch,
        "clock_ns": a.clock_km * 1e9 / 299_792.458,
        "clock_ns_uw": b.clock_km * 1e9 / 299_792.458,
        "residual_rms_m": a.residual_rms_m,
        "residual_rms_m_uw": b.residual_rms_m,
        "n_sat": a.n_sat,
        "n_bds": n_bds,
        "n_gps": n_gps,
        "n_sbas": n_sbas,
        "n_gal": n_gal,
        "n_bds_pre_reject": n_bds_pre_reject,
        "n_gal_pre_reject": n_gal_pre_reject,
        "ggto_applied": ggto_applied,
        "ab_membership_match": ab_match,
        "n_sat_weighted": a.n_sat,
        "n_sat_unweighted": b.n_sat,
        "n_fresh": n_fresh,
        "n_pred": n_pred,
        "slips": slips,
        "gen": gen_id,
        "source": "clock_bias",
    });
    if !geo_ranging.is_empty() {
        row["geo_ranging"] = json!(geo_ranging);
    }
    if !dropped_slip.is_empty() {
        row["dropped_slip"] = json!(dropped_slip);
    }
    row
}

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
                        continue; // the tracker_eph self-decode merge stays GPS-only (BDS loads from RINEX)
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

/// BDS ephemerides from the same RINEX file position_producer refreshes
/// hourly (the 2026-08-29 file carries 256 C-record blocks), parsed by the
/// strict RINEX-3 BDS path — beidou_d1::parse_rinex_bds applies the same
/// laws as the GPS parser: SatH1 != 0 hard-excluded at selection (health
/// gate), radians/semicircles unit votes, newest-issue (week, toe). Toe/toc
/// come back GPST-equivalent (+14 s), matching the tracker's BDS t_tx
/// frame. Refreshed on the GPS cadence in main. Unlike the GPS map, no
/// tracker_eph.json self-decode (D1) merge — out of scope here.
fn load_bds_ephs() -> HashMap<u8, BrdcEph> {
    match fs::read_to_string(RINEX) {
        Ok(t) => parse_rinex_bds(&t),
        Err(_) => Default::default(),
    }
}

/// Galileo ephemerides from the same hourly RINEX file, parsed by the
/// strict I/NAV-only path — gps::broadcast::parse_rinex_gal selects ONLY
/// records whose Data Sources field flags the I/NAV (E5b,E1) clock pair
/// (bit 9 + bit 0|2; the F/NAV twin every SV also carries has the WRONG
/// clock/BGD for the E1 user and is skipped by design), hard-excludes
/// nonzero E1-B health bits and out-of-band SISA (NAPA -1) at selection,
/// and stores BGD(E1,E5b) in the tgd slot so sat_clock_gal applies ICD
/// Eq. 17. toe/toc come back as GST SOW, GPST-equivalent unshifted (GST is
/// GPST-aligned), matching the tracker's GAL t_tx frame. Refreshed on the
/// GPS cadence in main. RINEX-only, like the BDS precedent — no
/// tracker_eph.json self-decode merge yet (out of scope here).
fn load_gal_ephs() -> HashMap<u8, BrdcEph> {
    match fs::read_to_string(RINEX) {
        Ok(t) => parse_rinex_gal(&t),
        Err(_) => Default::default(),
    }
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

fn tropo_delay_m(site_alt_m: f64, el_rad: f64) -> f64 {
    if el_rad <= 0.0 {
        return 0.0;
    }
    let ztd = 2.47 * (-site_alt_m / 7000.0).exp();
    let sin_el = el_rad.sin();
    let tan_el = el_rad.tan();
    let map = 1.0 / (sin_el + 0.00143 / (tan_el + 0.0445));
    (ztd * map).clamp(0.0, 35.0)
}

fn build_meas(
    prn: u8,
    rho_m: f64,
    t_tx: f64,
    ephs: &HashMap<u8, BrdcEph>,
    site_m: [f64; 3],
    site_lla: [f64; 3],
    igp_delay: &HashMap<(i16, i16), f64>,
    sbas_prc: &HashMap<u8, (f64, f64)>,
    sbas_lt: &HashMap<u8, (hackrf_gnss::sbas::LtCorr, f64)>,
    sbas_dnu: &HashMap<u8, f64>,
) -> Option<hackrf_gnss::gps::pvt::Meas> {
    if sbas_dnu.contains_key(&prn) {
        return None; // UDREI >= 14 exclusion
    }
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
    
    // SBAS Fast Corrections (PRC)
    let prc = sbas_prc.get(&prn).map(|&(p, _)| p).unwrap_or(0.0);
    
    // SBAS Long-Term Corrections (dx, dy, dz, daf0)
    let mut daf0 = 0.0;
    if let Some(&(ref lt, _)) = sbas_lt.get(&prn) {
        let lt_valid = match eph.iode {
            Some(iode) => iode == lt.iod,
            None => true,
        };
        if lt_valid {
            let (dx, dy, dz, d_daf0) = lt.propagate(t_tx);
            sat_m = [sat_m[0] + dx, sat_m[1] + dy, sat_m[2] + dz];
            daf0 = d_daf0;
        }
    }
    
    let rel = [sat_m[0] - site_m[0], sat_m[1] - site_m[1], sat_m[2] - site_m[2]];
    let (az, el) = hackrf_gnss::sbas_iono::azel(
        site_lla[0].to_radians(),
        site_lla[1].to_radians(),
        rel,
    );
    
    // SBAS Ionospheric Slant Delay
    let mut iono_m = 0.0;
    if el > 0.0 && !igp_delay.is_empty() {
        let ((plat, plon), fp) = hackrf_gnss::sbas_iono::ion_pierce_point(
            (site_lla[0].to_radians(), site_lla[1].to_radians()),
            az,
            el,
        );
        if let Some(d) = hackrf_gnss::sbas_iono::iono_slant_delay(
            plat.to_degrees(),
            plon.to_degrees(),
            fp,
            igp_delay,
        ) {
            iono_m = d;
        }
    }
    
    // Tropospheric Slant Delay
    let tropo_m = tropo_delay_m(site_lla[2], el);

    Some(hackrf_gnss::gps::pvt::Meas {
        sat: [sat_m[0] / 1000.0, sat_m[1] / 1000.0, sat_m[2] / 1000.0],
        pseudorange: (rho_m + prc - iono_m - tropo_m) / 1000.0 + (dt_sv + daf0) * 299_792.458,
        clock_free: false,
    })
}

/// BDS twin of build_meas, mirroring live_fix.rs:763-780 (the "beidou"
/// branch): the same transmit-time iteration through sat_at_txtime_bds
/// (t_tx is GPST for both constellations — the tracker converts BDS at the
/// anchor and the BDS ephemeris toe/toc are stored GPST-equivalent).
fn build_meas_bds(
    prn: u8,
    rho_m: f64,
    t_tx: f64,
    ephs: &HashMap<u8, BrdcEph>,
    site_m: [f64; 3],
    site_lla: [f64; 3],
) -> Option<hackrf_gnss::gps::pvt::Meas> {
    let eph = ephs.get(&prn)?;
    let (_, dt0, _) = sat_at_txtime_bds(eph, t_tx, site_m);
    let mut a = t_tx - dt0 + 0.075;
    let (mut sat_m, mut dt_sv) = ([0.0; 3], dt0);
    for _ in 0..2 {
        let (s, d, r) = sat_at_txtime_bds(eph, a, site_m);
        sat_m = s;
        dt_sv = d;
        a = t_tx - d + r / 299_792_458.0;
    }
    let rel = [sat_m[0] - site_m[0], sat_m[1] - site_m[1], sat_m[2] - site_m[2]];
    let (_, el) = hackrf_gnss::sbas_iono::azel(
        site_lla[0].to_radians(),
        site_lla[1].to_radians(),
        rel,
    );
    let tropo_m = tropo_delay_m(site_lla[2], el);

    Some(hackrf_gnss::gps::pvt::Meas {
        sat: [sat_m[0] / 1000.0, sat_m[1] / 1000.0, sat_m[2] / 1000.0],
        pseudorange: (rho_m - tropo_m) / 1000.0 + dt_sv * 299_792.458, // sat clock (incl. TGD1) removed
        clock_free: false,
    })
}

/// GEO twin of build_meas (lever 2): satellite position/velocity/clock
/// from the tracker-published MT9 vector (sbas_geonav -> sbas::GeoEph),
/// propagated by sbas::geo_at_txtime — the DO-229D A.4.5.1 2nd-order
/// Taylor + Sagnac rotation, the same transmit-time iteration convention
/// as the GPS/BDS paths — with the agf0 + agf1·dt clock applied exactly
/// like a GPS SV clock. Iono comes from the SBAS grid (the GEO is an L1
/// signal whose pierce point sits inside the WAAS grid) and tropo from the
/// same map as every other row. No PRC/LT corrections: WAAS addresses
/// MT2-5/24/25 to mask slots 1..=37 (GPS), never to the GEO itself; the
/// GEO integrity gates are MT9's own URA (15 = do-not-use) and the 360 s
/// MT9 timeout. Expected quality is ~10 m-class ranging bias (GEO
/// ephemeris + uncorrected code biases) — well inside the 1000 m
/// studentized gate, which is deliberately NOT relaxed for these rows;
/// the epoch row flags them (geo_ranging/n_sbas) for later weighting.
fn build_meas_sbas(
    prn: u8,
    rho_m: f64,
    t_tx: f64,
    geos: &HashMap<u8, GeoEph>,
    site_m: [f64; 3],
    site_lla: [f64; 3],
    igp_delay: &HashMap<(i16, i16), f64>,
) -> Option<hackrf_gnss::gps::pvt::Meas> {
    let geo = geos.get(&prn)?;
    if geo.ura >= 15 {
        return None; // MT9 URA "do not use"
    }
    if geo.dt_from(t_tx).abs() > GEO_MAX_DT_S {
        return None; // stale vector — fail-closed rather than extrapolate
    }
    let (_, dt0, _) = geo_at_txtime(geo, t_tx, site_m);
    let mut a = t_tx - dt0 + 0.125;
    let (mut sat_m, mut dt_sv) = ([0.0; 3], dt0);
    for _ in 0..2 {
        let (s, d, r) = geo_at_txtime(geo, a, site_m);
        sat_m = s;
        dt_sv = d;
        a = t_tx - d + r / 299_792_458.0;
    }
    let rel = [sat_m[0] - site_m[0], sat_m[1] - site_m[1], sat_m[2] - site_m[2]];
    let (az, el) = hackrf_gnss::sbas_iono::azel(
        site_lla[0].to_radians(),
        site_lla[1].to_radians(),
        rel,
    );
    let mut iono_m = 0.0;
    if el > 0.0 && !igp_delay.is_empty() {
        let ((plat, plon), fp) = hackrf_gnss::sbas_iono::ion_pierce_point(
            (site_lla[0].to_radians(), site_lla[1].to_radians()),
            az,
            el,
        );
        if let Some(d) = hackrf_gnss::sbas_iono::iono_slant_delay(
            plat.to_degrees(),
            plon.to_degrees(),
            fp,
            igp_delay,
        ) {
            iono_m = d;
        }
    }
    let tropo_m = tropo_delay_m(site_lla[2], el);

    Some(hackrf_gnss::gps::pvt::Meas {
        sat: [sat_m[0] / 1000.0, sat_m[1] / 1000.0, sat_m[2] / 1000.0],
        pseudorange: (rho_m - iono_m - tropo_m) / 1000.0 + dt_sv * 299_792.458,
        clock_free: false,
    })
}

/// Galileo twin of build_meas (lever 1): the same transmit-time iteration
/// through sat_at_txtime_gal (t_tx is the GGTO-converted GPST anchor — the
/// caller applies ggto_convert BEFORE this builder so the frozen-staircase
/// projection and the fresh path convert identically). BGD(E1,E5b) is
/// inside sat_clock_gal (ICD Eq. 17, like TGD1 in the BDS clock), so no
/// extra term appears here. Iono: E1 is the SAME 1575.42 MHz as L1, so the
/// SBAS IGP slant delay applies UNSCALED (unlike the BDS lever-7 proposal)
/// — a GPS-L1-certified product on an uncertified constellation at the
/// same frequency: physically sound, uncertified. NO SBAS PRC/LT/DNU
/// (GPS-PRN products, meaningless for GAL). Tropo from the same map.
/// Fail-closed gates: no ephemeris -> None (SISA/E1-B health/F/NAV already
/// gated at parse_rinex_gal selection); ephemeris age |t_tx - toe| >
/// GAL_EPH_MAX_AGE_S (wrap-aware) -> None.
fn build_meas_gal(
    prn: u8,
    rho_m: f64,
    t_tx: f64,
    ephs: &HashMap<u8, BrdcEph>,
    site_m: [f64; 3],
    site_lla: [f64; 3],
    igp_delay: &HashMap<(i16, i16), f64>,
) -> Option<hackrf_gnss::gps::pvt::Meas> {
    let eph = ephs.get(&prn)?;
    if wrap_tk(t_tx - eph.toe).abs() > GAL_EPH_MAX_AGE_S {
        return None; // stale batch — refuse to range (fail-closed)
    }
    let (_, dt0, _) = sat_at_txtime_gal(eph, t_tx, site_m);
    let mut a = t_tx - dt0 + 0.075;
    let (mut sat_m, mut dt_sv) = ([0.0; 3], dt0);
    for _ in 0..2 {
        let (s, d, r) = sat_at_txtime_gal(eph, a, site_m);
        sat_m = s;
        dt_sv = d;
        a = t_tx - d + r / 299_792_458.0;
    }
    let rel = [sat_m[0] - site_m[0], sat_m[1] - site_m[1], sat_m[2] - site_m[2]];
    let (az, el) = hackrf_gnss::sbas_iono::azel(
        site_lla[0].to_radians(),
        site_lla[1].to_radians(),
        rel,
    );
    let mut iono_m = 0.0;
    if el > 0.0 && !igp_delay.is_empty() {
        let ((plat, plon), fp) = hackrf_gnss::sbas_iono::ion_pierce_point(
            (site_lla[0].to_radians(), site_lla[1].to_radians()),
            az,
            el,
        );
        if let Some(d) = hackrf_gnss::sbas_iono::iono_slant_delay(
            plat.to_degrees(),
            plon.to_degrees(),
            fp,
            igp_delay,
        ) {
            iono_m = d;
        }
    }
    let tropo_m = tropo_delay_m(site_lla[2], el);

    Some(hackrf_gnss::gps::pvt::Meas {
        sat: [sat_m[0] / 1000.0, sat_m[1] / 1000.0, sat_m[2] / 1000.0],
        pseudorange: (rho_m - iono_m - tropo_m) / 1000.0 + dt_sv * 299_792.458, // sat clock (incl. BGD(E1,E5b)) removed
        clock_free: false,
    })
}

fn main() {
    // ephemeris: same loader live_fix uses (BRDC); refresh every 15 min
    let mut ephs = load_ephs();
    let mut bds_ephs = load_bds_ephs(); // same RINEX file, same cadence
    let mut gal_ephs = load_gal_ephs(); // same RINEX file, same cadence
    let mut eph_loaded = unix_now();
    // the surveyed site.json anchor: site_m (metres) is the light-time
    // anchor for sat_at_txtime_pub (live_fix's site_m), anchor_km is the
    // FIXED position of the clock-only solve. Computed once — a site.json
    // change is a config change and must restart the producer (new gen).
    let site_m = site_guess().map(|x| x * 1000.0);
    let anchor_km = site_guess();
    // session id (v2 amendment): producer start epoch — restarts/config
    // changes can't silently mix into one series. Round-14 provenance:
    // include the tracker's BUILD identity (live_radio binary mtime) — the
    // process-start-only gen let rows mix across tracker builds/configs
    // (the 22:19 filter-change restart kept gen v2-1787924784 while the
    // measurement chain changed). A tracker rebuild now shows in the gen at
    // the next producer start. Restart-TIME gen rolls need a tracker-
    // published start marker (queued with the tracker_producer window work).
    let trk_build = fs::metadata(TRACKER_BIN)
        .and_then(|m| m.modified())
        .map(|t| {
            t.duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs()
        })
        .unwrap_or(0);
    let gen_id = format!("v4-{}-tb{}", unix_now() as u64, trk_build);
    let mut smoothers: HashMap<u8, Hatch> = HashMap::new();
    let mut prev: HashMap<u8, PrevSat> = HashMap::new();
    // BDS channels keep SEPARATE smoother/prediction chains: the maps are
    // keyed by PRN and PRNs collide across constellations (the live state
    // carries GPS 32 AND BDS 32 locked simultaneously; GPS 23 and BDS 23
    // would collide the same way). The GPS-only build was protected by its
    // sys filter running BEFORE any map access; the constellation split in
    // the loop keeps that protection for both.
    let mut smoothers_bds: HashMap<u8, Hatch> = HashMap::new();
    let mut prev_bds: HashMap<u8, PrevSat> = HashMap::new();
    // SBAS GEO chains (lever 2): same PRN-collision reasoning — a third
    // map pair. GEO carrier is L1 (λ_L1), same NCO machinery, and the
    // frame anchor refreshes every second so the fresh-code path runs
    // every epoch (no ~6 s staircase for these channels).
    let mut smoothers_sbas: HashMap<u8, Hatch> = HashMap::new();
    let mut prev_sbas: HashMap<u8, PrevSat> = HashMap::new();
    // Galileo chains (lever 1): fourth map pair, same PRN-collision
    // reasoning (GAL E1B PRNs 1-36 overlap GPS 1-32). E1 carrier is the
    // L1 frequency (λ_L1); same staircase machinery as GPS/BDS.
    let mut smoothers_gal: HashMap<u8, Hatch> = HashMap::new();
    let mut prev_gal: HashMap<u8, PrevSat> = HashMap::new();
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
            bds_ephs = load_bds_ephs(); // same file, same cadence
            gal_ephs = load_gal_ephs(); // same file, same cadence
            eph_loaded = unix_now();
        }

        let mut sbas_prc: HashMap<u8, (f64, f64)> = HashMap::new();
        let mut sbas_lt: HashMap<u8, (hackrf_gnss::sbas::LtCorr, f64)> = HashMap::new();
        let mut sbas_dnu: HashMap<u8, f64> = HashMap::new();
        let mut igp_delay: HashMap<(i16, i16), f64> = HashMap::new();
        // per-GEO MT9 state vectors (lever 2): the published sbas_geonav
        // deserializes straight into sbas::GeoEph (same field names/units)
        let mut sbas_geo: HashMap<u8, GeoEph> = HashMap::new();

        for s in sats {
            if s["sys"].as_str() != Some("sbas") {
                continue;
            }
            // the MT9 vector is a CRC-validated cached record: harvest it
            // whether or not THIS second's decode window is locked (rho_m
            // freshness is gated by the tracker's own anchor machinery)
            if let (Some(prn), Ok(geo)) = (
                s["prn"].as_u64(),
                serde_json::from_value::<GeoEph>(s["sbas_geonav"].clone()),
            ) {
                sbas_geo.insert(prn as u8, geo);
            }
            if !s["sbas_msgs"]["locked"].as_bool().unwrap_or(false) {
                continue;
            }
            for row in s["sbas_msgs"]["fast_corr"].as_array().into_iter().flatten() {
                if let (Some(prn), Some(prc)) = (row[0].as_u64(), row[1].as_f64()) {
                    let age = row.get(3).and_then(|v| v.as_f64()).unwrap_or(f64::MAX);
                    if let Some(&(_, prev_age)) = sbas_prc.get(&(prn as u8)) {
                        if age >= prev_age { continue; }
                    }
                    sbas_prc.insert(prn as u8, (prc, age));
                }
            }
            for row in s["sbas_msgs"]["lt_corr"].as_array().into_iter().flatten() {
                if let Ok(corr) = serde_json::from_value::<hackrf_gnss::sbas::LtCorr>(row.clone()) {
                    let age = row["age_s"].as_f64().unwrap_or(f64::MAX);
                    if let Some(&(_, prev_age)) = sbas_lt.get(&corr.prn) {
                        if age >= prev_age { continue; }
                    }
                    sbas_lt.insert(corr.prn, (corr, age));
                }
            }
            for row in s["sbas_msgs"]["dont_use"].as_array().into_iter().flatten() {
                if let Some(prn) = row[0].as_u64() {
                    let age = row[1].as_f64().unwrap_or(f64::MAX);
                    if let Some(&prev_age) = sbas_dnu.get(&(prn as u8)) {
                        if age >= prev_age { continue; }
                    }
                    sbas_dnu.insert(prn as u8, age);
                }
            }
            for mask in s["sbas_msgs"]["igp_mask"].as_array().into_iter().flatten() {
                let (Some(band), Some(miodi)) = (mask[0].as_u64(), mask[1].as_u64()) else { continue };
                let Some(igps) = mask[2].as_array() else { continue };
                for dl in s["sbas_msgs"]["iono_delay"].as_array().into_iter().flatten() {
                    let (Some(dband), Some(block), Some(diodi)) = (dl[0].as_u64(), dl[1].as_u64(), dl[2].as_u64()) else { continue };
                    if dband != band || diodi != miodi { continue; }
                    let Some(rows) = dl[3].as_array() else { continue };
                    for (i, row) in rows.iter().enumerate() {
                        let (Some(counts), Some(givei)) = (row[0].as_u64(), row[1].as_u64()) else { continue };
                        if counts == 511 || givei >= 15 { continue; }
                        let j = block as usize * 15 + i;
                        if let Some(igp_num) = igps.get(j).and_then(|v| v.as_u64()) {
                            if let Some(coord) = hackrf_gnss::sbas_iono::igp_latlon(band as u8, igp_num as u16) {
                                igp_delay.insert(coord, counts as f64 * 0.125);
                            }
                        }
                    }
                }
            }
        }

        let site_lla = site_lla();
        // built measurements with their identity + slip + GGTO flags:
        // (Meas, Cls, prn, contributed-with-a-reset-this-epoch,
        //  anchor-used-broadcast-GGTO)
        let mut meas: Vec<(hackrf_gnss::gps::pvt::Meas, Cls, u8, bool, bool)> = Vec::new();
        let mut slips = 0u32;
        let mut n_fresh = 0u32;
        let mut n_pred = 0u32;
        let mut n_bds_pre_reject = 0u32;
        let mut n_gal_pre_reject = 0u32;
        for s in sats {
            // Constellation split FIRST — before any smoother-map access
            // (the chains are per-constellation; see the map decls above).
            let cls = match s["sys"].as_str() {
                Some("gps") => Cls::Gps,
                Some("beidou") => Cls::Bds,
                Some("sbas") => Cls::Sbas,
                Some("galileo") => Cls::Gal,
                _ => continue,
            };
            let prn = s["prn"].as_u64().unwrap_or(0) as u8;
            if cls == Cls::Sbas && !GEO_RANGING_PRNS.contains(&prn) {
                continue; // GEO ranging is whitelisted (see the const)
            }
            if s["cn0_proxy"].as_f64().unwrap_or(0.0) < 30.0 { continue; }
            let lock_s = s["lock_s"].as_f64().unwrap_or(0.0);
            if lock_s < 20.0 { continue; }
            let (rho, t_tx) = match (s["rho_m"].as_f64(), s["t_tx"].as_f64()) {
                (Some(a), Some(b)) => (a, b), _ => continue };
            let carr = s["carrier_cycles"].as_f64().unwrap_or(0.0);
            let slip = s["slip"].as_bool().unwrap_or(false);
            let s_epoch = s["epoch"].as_f64().unwrap_or(epoch);
            // GGTO (GAL only, spec §5.4 Option B): the tracker's cached
            // word-10 offset, OPTIONAL and additive on the sat row — null/
            // absent applies zero fail-closed inside ggto_convert.
            let ggto_ns = if cls == Cls::Gal { s["ggto_ns"].as_f64() } else { None };
            // the same staircase/carrier machinery serves all four
            // constellations — only the chain maps and the carrier
            // wavelength differ (λ_B1I for BDS; the WAAS GEO broadcasts
            // on L1 proper and GAL E1 IS 1575.42 MHz, so both share λ_L1
            // with GPS)
            let (smoothers, prev, lam) = match cls {
                Cls::Bds => (&mut smoothers_bds, &mut prev_bds, LAM_B1I),
                Cls::Gps => (&mut smoothers, &mut prev, LAM_L1),
                Cls::Sbas => (&mut smoothers_sbas, &mut prev_sbas, LAM_L1),
                Cls::Gal => (&mut smoothers_gal, &mut prev_gal, LAM_L1),
            };
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
                        p.file_epoch = s_epoch;
                        p.contrib_valid = false;
                    }
                    slips += 1;
                    continue;
                }
                let p = prev.get_mut(&prn).unwrap();
                if p.contrib_valid && s_epoch - p.last_code_epoch < PRED_WINDOW_S {
                    // right-signed carrier integral since the last code
                    // update: Δrho = −λ·Δcarr (sign verified live
                    // 2026-08-28 on the GPS channels; inherited for BDS —
                    // same tracker NCO machinery, live BDS check pending)
                    let rho_used = p.base_smoothed + lam * (p.base_carr - carr);
                    // t_tx froze WITH rho (verified live), so project it by
                    // the same interval — the predicted range and the
                    // ephemeris evaluation must refer to the same epoch
                    let t_tx_used = t_tx + (s_epoch - p.last_code_epoch);
                    // GAL: convert the projected GST anchor to GPST (the
                    // GGTO drifts ~fs over the 12 s window — projecting
                    // then converting is exact at this precision)
                    let (t_tx_gal, ggto_used) = ggto_convert(t_tx_used, ggto_ns);
                    let m = match cls {
                        Cls::Bds => {
                            build_meas_bds(prn, rho_used, t_tx_used, &bds_ephs, site_m, site_lla)
                        }
                        Cls::Sbas => build_meas_sbas(
                            prn, rho_used, t_tx_used, &sbas_geo, site_m, site_lla, &igp_delay,
                        ),
                        Cls::Gal => build_meas_gal(
                            prn, rho_used, t_tx_gal, &gal_ephs, site_m, site_lla, &igp_delay,
                        ),
                        Cls::Gps => build_meas(
                            prn,
                            rho_used,
                            t_tx_used,
                            &ephs,
                            site_m,
                            site_lla,
                            &igp_delay,
                            &sbas_prc,
                            &sbas_lt,
                            &sbas_dnu,
                        ),
                    };
                    if let Some(m) = m {
                        match cls {
                            Cls::Bds => n_bds_pre_reject += 1,
                            Cls::Gal => n_gal_pre_reject += 1,
                            _ => {}
                        }
                        // predicted rows carry no reset by construction
                        // (a reset on the frozen path voids the chain above)
                        meas.push((m, cls, prn, false, cls == Cls::Gal && ggto_used));
                    }
                    n_pred += 1;
                }
                p.lock_s = lock_s;
                p.file_epoch = s_epoch;
                continue;
            }

            // Fresh code update.
            // Innovation gate (only when the previous epoch was contiguous):
            // the prediction is the same base the solve carries — code noise
            // is ~10–30 m, so 500 m fires only on a real discontinuity.
            let innov_breach = prev.get(&prn).is_some_and(|p| {
                p.contrib_valid
                    && p.file_epoch == prev_file_epoch
                    && (rho - (p.base_smoothed + lam * (p.base_carr - carr))).abs() > 500.0
            });
            let reset = slip || lock_regressed || innov_breach;
            if reset { slips += 1; }
            let h = smoothers.entry(prn).or_insert_with(|| Hatch::new(WINDOW_S));
            // negated carrier — 2026-08-28 live verification (GPS channels;
            // inherited for BDS, same NCO machinery)
            let rho_s = h.update(rho, -carr, lam, reset);
            n_fresh += 1;
            // the prediction base moves to this update: on fresh epochs the
            // carried range equals the new smoothed value by construction
            prev.insert(prn, PrevSat {
                lock_s,
                rho_m: rho,
                base_smoothed: rho_s,
                base_carr: carr,
                last_code_epoch: s_epoch,
                file_epoch: s_epoch,
                contrib_valid: true,
            });
            let (t_tx_gal, ggto_used) = ggto_convert(t_tx, ggto_ns);
            let m = match cls {
                Cls::Bds => build_meas_bds(prn, rho_s, t_tx, &bds_ephs, site_m, site_lla),
                Cls::Sbas => {
                    build_meas_sbas(prn, rho_s, t_tx, &sbas_geo, site_m, site_lla, &igp_delay)
                }
                Cls::Gal => {
                    build_meas_gal(prn, rho_s, t_tx_gal, &gal_ephs, site_m, site_lla, &igp_delay)
                }
                Cls::Gps => build_meas(
                    prn,
                    rho_s,
                    t_tx,
                    &ephs,
                    site_m,
                    site_lla,
                    &igp_delay,
                    &sbas_prc,
                    &sbas_lt,
                    &sbas_dnu,
                ),
            };
            if let Some(m) = m {
                match cls {
                    Cls::Bds => n_bds_pre_reject += 1,
                    Cls::Gal => n_gal_pre_reject += 1,
                    _ => {}
                }
                // `reset` is exactly the event the `slips` counter counted
                // for this sat — the lever-3b drop plan keys off it
                meas.push((m, cls, prn, reset, cls == Cls::Gal && ggto_used));
            }
        }
        if meas.len() < 4 { continue; }   // emit gate 5->4 (window pkg, UNBUILT): the n>=5 floor was the hour-gate killer (2026-08-29 bake-off: duty 24.7% -> 38.3%). ISB tension: a 2-state (clock+ISB) solve needs n>=5 with BDS OR GAL present (GAL adds the GGTO-residual + E1B-vs-C/A receiver ISB on top of the BDS one) — when ISB surgery lands, re-raise the gate for mixed solves or constrain ISB from the recent estimate at n==4.
        // Lever 3b: with >= SLIP_DROP_MIN_PRE built measurements, drop the
        // contributors whose smoother reset this epoch and solve the
        // remainder — one slipping bird must not poison an otherwise-deep
        // epoch (the analyzer's slips==0 quality gate would exclude it).
        // The dropped resets leave the published `slips` count; the row
        // records them as dropped_slip labels instead.
        let slipped: Vec<bool> = meas.iter().map(|&(_, _, _, s, _)| s).collect();
        let drop = plan_slip_drop(&slipped);
        let mut dropped_slip: Vec<String> = Vec::new();
        if !drop.is_empty() {
            dropped_slip = drop.iter().map(|&i| meas[i].1.label(meas[i].2)).collect();
            slips = slips.saturating_sub(drop.len() as u32);
            let mut keep = Vec::with_capacity(meas.len() - drop.len());
            for (i, r) in meas.into_iter().enumerate() {
                if !drop.contains(&i) {
                    keep.push(r);
                }
            }
            meas = keep;
        }
        let cls_of: Vec<Cls> = meas.iter().map(|&(_, c, _, _, _)| c).collect();
        let prn_of: Vec<u8> = meas.iter().map(|&(_, _, p, _, _)| p).collect();
        let ggto_of: Vec<bool> = meas.iter().map(|&(_, _, _, _, g)| g).collect();
        let meas: Vec<hackrf_gnss::gps::pvt::Meas> =
            meas.into_iter().map(|(m, _, _, _, _)| m).collect();
        // CAVEAT: ONE clock state for FOUR constellations — the GPS/BDS/GAL
        // inter-system channel biases are unmodeled in this 1-state solve (a
        // weighted mean with studentized rejection; no ISB state — surgery
        // out of scope), the GEO rows add their own ~10 m-class ranging
        // bias, and the GAL rows carry the receiver's irreducible E1B-vs-C/A
        // hardware/correlation bias plus whatever GGTO residual survives the
        // broadcast word-10 conversion (zero applied fail-closed when word
        // 10 is absent/invalid — raw GGTO is tens of ns, still far under the
        // gate). Rows carry n_bds/n_sbas/n_gal/geo_ranging/ggto_applied so
        // the analyzer and leg-1 weighting can quantify mix-dependence; a
        // structurally-biased row that trips the 1000 m studentized gate is
        // legitimately DROPPED by the rejection — fail-closed, not silent.
        // The solver returns accepted input indices so the row reports
        // post-rejection constellation counts.
        let fw = hackrf_gnss::gps::pvt::solve_clock_only(&meas, anchor_km, true);
        let fu = hackrf_gnss::gps::pvt::solve_clock_only(&meas, anchor_km, false);
        if let (Some(a), Some(b)) = (fw, fu) {
            // Lever 6 — A/B means weighting only, and independent rejection
            // sets would mix weighting with satellite composition. Such an
            // epoch used to be silently skipped; it now PUBLISHES with
            // ab_membership_match:false and both accepted counts, and the
            // analyzer keeps it out of claims (visible in data, excluded
            // from claims). epoch_row derives the flag and the counts.
            let row = epoch_row(
                &a,
                &b,
                &cls_of,
                &prn_of,
                &ggto_of,
                epoch,
                n_bds_pre_reject,
                n_gal_pre_reject,
                n_fresh,
                n_pred,
                slips,
                &dropped_slip,
                &gen_id,
            );
            use std::io::Write;
            if let Ok(mut f) = fs::OpenOptions::new().create(true).append(true).open(OUT) {
                let _ = writeln!(f, "{}", row);
            }
            // Atomic per-second state for /api/sync (tmp + rename, like
            // every other producer): the panel reads the live clock bias
            // without parsing the archive. Fail-closed by TTL: when no
            // clean solve exists the file simply expires.
            let st = json!({
                "schema": "clock_bias-v4",
                "epoch": epoch,
                "ttl_s": 10,
                "clock_bias": {
                    "clock_ns": row["clock_ns"],
                    "clock_ns_uw": row["clock_ns_uw"],
                    "residual_rms_m": row["residual_rms_m"],
                    "n_sat": row["n_sat"],
                    "n_bds": row["n_bds"],
                    "n_gps": row["n_gps"],
                    "n_sbas": row["n_sbas"],
                    "n_gal": row["n_gal"],
                    "n_bds_pre_reject": row["n_bds_pre_reject"],
                    "n_gal_pre_reject": row["n_gal_pre_reject"],
                    "ggto_applied": row["ggto_applied"],
                    "ab_membership_match": row["ab_membership_match"],
                    "n_fresh": row["n_fresh"],
                    "n_pred": row["n_pred"],
                    "slips": row["slips"],
                    "gen": gen_id,
                },
            });
            let tmp = format!("{STATE_CB}.tmp");
            if fs::write(&tmp, serde_json::to_string(&st).unwrap_or_default()).is_ok() {
                let _ = fs::rename(&tmp, STATE_CB);
            }
        }
    }
}

// Window-build tests (Cargo.toml marks this example `test = true`; nothing
// here has run yet — the ABSOLUTE no-cargo law holds until the maintenance
// window).
#[cfg(test)]
mod tests {
    use super::*;
    use hackrf_gnss::gps::pvt::{solve_clock_only, Meas};

    /// Real six-sat GPS geometry from the pvt solver tests (km): the site
    /// anchor and satellite ECEFs; pseudorange = geometric + clock.
    const STATION: [f64; 3] = [1351.991, -4653.584, 4133.012];
    const SATS: [[f64; 3]; 6] = [
        [5869.975, -16762.018, 19215.021],
        [-3107.678, -19845.218, 17310.754],
        [13935.468, -7948.651, 20949.401],
        [12385.868, -23003.053, 2879.614],
        [21723.378, -9762.873, 11751.846],
        [-11644.487, -10338.459, 21397.985],
    ];

    fn norm3(a: [f64; 3]) -> f64 {
        (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
    }

    fn ranges(clock_km: f64) -> Vec<Meas> {
        SATS.iter()
            .map(|&s| {
                let g = norm3([STATION[0] - s[0], STATION[1] - s[1], STATION[2] - s[2]]);
                Meas { sat: s, pseudorange: g + clock_km, clock_free: false }
            })
            .collect()
    }

    fn fix(n: usize, accepted: Vec<usize>) -> ClockFix {
        ClockFix {
            clock_km: 50.0,
            residual_rms_m: 1.0,
            n_sat: n,
            accepted_indices: accepted,
            max_leverage: 0.3,
        }
    }

    #[test]
    fn plan_slip_drop_honors_the_floors() {
        // below SLIP_DROP_MIN_PRE: never drop, even with a slip present
        assert!(plan_slip_drop(&[true, false, false, false, false]).is_empty());
        // at the floor with one slip: drop exactly it
        assert_eq!(plan_slip_drop(&[false, true, false, false, false, false]), vec![1]);
        // no slips: nothing to do
        assert!(plan_slip_drop(&[false; 6]).is_empty());
        // dropping all three slips would leave 3 < 4: refuse (unchanged
        // behavior — publish with slips counted)
        assert!(plan_slip_drop(&[true, true, true, false, false, false]).is_empty());
        // 7 with two slips leaves 5: drop both
        assert_eq!(
            plan_slip_drop(&[false, true, false, true, false, false, false]),
            vec![1, 3]
        );
    }

    /// Lever 3b end-to-end on real geometry: a slipped contributor whose
    /// pseudorange broke by a whole code tooth is dropped by the plan, and
    /// the re-solve on the remainder recovers the exact clock with the
    /// dropped label recorded.
    #[test]
    fn slip_drop_resolve_recovers_the_clock() {
        let clock = 50.0;
        let mut m = ranges(clock);
        m[2].pseudorange += 299.792_458; // 1 ms tooth slip on the slipped sat
        let cls = [Cls::Gps, Cls::Gps, Cls::Gps, Cls::Bds, Cls::Bds, Cls::Sbas];
        let prn = [16u8, 4, 26, 27, 31, 131];
        let slipped = [false, false, true, false, false, false];
        let drop = plan_slip_drop(&slipped);
        assert_eq!(drop, vec![2]);
        let dropped_slip: Vec<String> =
            drop.iter().map(|&i| cls[i].label(prn[i])).collect();
        assert_eq!(dropped_slip, vec!["G26".to_string()]);
        let kept: Vec<Meas> = (0..m.len()).filter(|i| !drop.contains(i)).map(|i| m[i]).collect();
        let kept_cls: Vec<Cls> =
            (0..cls.len()).filter(|i| !drop.contains(i)).map(|i| cls[i]).collect();
        let kept_prn: Vec<u8> =
            (0..prn.len()).filter(|i| !drop.contains(i)).map(|i| prn[i]).collect();
        let a = solve_clock_only(&kept, STATION, true).expect("weighted solve");
        let b = solve_clock_only(&kept, STATION, false).expect("unweighted solve");
        assert!((a.clock_km - clock).abs() < 1e-6, "clock {}", a.clock_km);
        assert_eq!(a.n_sat, 5);
        let row = epoch_row(
            &a, &b, &kept_cls, &kept_prn, &[false; 5], 1.0, 0, 0, 5, 0, 0, &dropped_slip, "t",
        );
        assert_eq!(row["slips"], 0);
        assert_eq!(row["dropped_slip"], json!(["G26"]));
        assert_eq!(row["ab_membership_match"], json!(true));
        // the GEO stayed in the solve and is flagged for leg-1 weighting
        assert_eq!(row["geo_ranging"], json!([131]));
        assert_eq!(row["n_sbas"], 1);
        assert_eq!(row["n_bds"], 2);
        assert_eq!(row["n_gps"], 2);
        assert_eq!(row["n_gal"], 0);
        assert_eq!(row["ggto_applied"], json!(false));
    }

    /// Lever 6: a membership mismatch publishes (flagged), never suppresses.
    #[test]
    fn ab_mismatch_publishes_flagged_with_both_counts() {
        let cls = [Cls::Gps, Cls::Gps, Cls::Bds, Cls::Bds, Cls::Sbas];
        let prn = [16u8, 4, 27, 31, 135];
        // weighted kept all 5; unweighted rejected the GEO
        let a = fix(5, vec![0, 1, 2, 3, 4]);
        let b = fix(4, vec![0, 1, 2, 3]);
        let row = epoch_row(&a, &b, &cls, &prn, &[false; 5], 2.0, 0, 0, 5, 0, 0, &[], "t");
        assert_eq!(row["ab_membership_match"], json!(false));
        assert_eq!(row["n_sat_weighted"], 5);
        assert_eq!(row["n_sat_unweighted"], 4);
        // identity fields describe the WEIGHTED accepted set (documented)
        assert_eq!(row["n_sat"], 5);
        assert_eq!(row["n_gps"], 2);
        assert_eq!(row["n_bds"], 2);
        assert_eq!(row["n_sbas"], 1);
        assert_eq!(row["n_gal"], 0);
        assert_eq!(row["geo_ranging"], json!([135]));
        // and a matching pair still reads true with both counts equal
        let b2 = fix(5, vec![0, 1, 2, 3, 4]);
        let row2 = epoch_row(&a, &b2, &cls, &prn, &[false; 5], 3.0, 0, 0, 5, 0, 0, &[], "t");
        assert_eq!(row2["ab_membership_match"], json!(true));
        assert_eq!(row2["n_sat_weighted"], row2["n_sat_unweighted"]);
        // optional fields stay absent when empty (additive-only schema)
        assert!(row2.get("dropped_slip").is_none());
    }

    /// Lever 1: GAL rows join the identity counters — n_gps is COUNTED,
    /// never derived by subtraction (the pre-GAL derivation would have
    /// silently misattributed every GAL row to GPS) — the four-way
    /// identity n_gps+n_bds+n_sbas+n_gal == n_sat holds by construction,
    /// ggto_applied reflects the ACCEPTED GAL set only, and dropped_slip
    /// labels are E-prefixed.
    #[test]
    fn gal_rows_join_the_identity_counters_and_labels() {
        let cls = [Cls::Gps, Cls::Gps, Cls::Bds, Cls::Gal, Cls::Gal, Cls::Sbas];
        let prn = [16u8, 4, 27, 5, 33, 131];
        let ggto = [false, false, false, true, false, false];
        let a = fix(6, vec![0, 1, 2, 3, 4, 5]);
        let b = fix(6, vec![0, 1, 2, 3, 4, 5]);
        let row = epoch_row(&a, &b, &cls, &prn, &ggto, 4.0, 1, 2, 6, 0, 0, &[], "t");
        assert_eq!(row["n_sat"], 6);
        assert_eq!(row["n_gps"], 2);
        assert_eq!(row["n_bds"], 1);
        assert_eq!(row["n_gal"], 2);
        assert_eq!(row["n_sbas"], 1);
        assert_eq!(row["n_gal_pre_reject"], 2);
        // one accepted GAL anchor used the broadcast GGTO -> flagged
        assert_eq!(row["ggto_applied"], json!(true));
        // identity by construction: counted, not derived
        let sum = ["n_gps", "n_bds", "n_sbas", "n_gal"]
            .iter()
            .map(|k| row[*k].as_u64().unwrap())
            .sum::<u64>();
        assert_eq!(sum, row["n_sat"].as_u64().unwrap());
        // solver rejected BOTH GAL rows: n_gal drops to 0 and ggto_applied
        // reads false — the flag describes the accepted set, not the input
        let a2 = fix(4, vec![0, 1, 2, 5]);
        let b2 = fix(4, vec![0, 1, 2, 5]);
        let row2 = epoch_row(&a2, &b2, &cls, &prn, &ggto, 5.0, 1, 2, 6, 0, 0, &[], "t");
        assert_eq!(row2["n_gal"], 0);
        assert_eq!(row2["n_gps"], 2);
        assert_eq!(row2["ggto_applied"], json!(false));
        // E-prefixed dropped_slip label (PRN collision namespace)
        assert_eq!(Cls::Gal.label(5), "E05");
    }

    /// Spec §5.4 Option B, pinned against tests/fixtures/inav/
    /// ggto_bgd_vectors.json (python reference, ICD §5.1.8 Eq. 23): the
    /// tracker publishes dt_systems as ggto_ns; the anchor converts
    /// t_tx_gpst = t_tx_gst - dt_systems, and absent/invalid word 10
    /// (null ggto_ns) applies ZERO fail-closed — never 0.0-as-unknown
    /// arithmetic on a sentinel.
    #[test]
    fn ggto_convert_pins_the_fixture_cases() {
        // nominal: a0g=-232, a1g=5, t0g=68, wn0g=10 at (wn 10, tow 245425)
        let (t, used) = ggto_convert(245_425.0, Some(-6.750700887181438));
        assert!(used);
        assert!((t - 245_425.00000000675).abs() < 1e-9, "t {t}");
        // negative a0g across the week seam (wn0g 63 -> wn 0)
        let (t, used) = ggto_convert(3_625.0, Some(-5.944678971303574));
        assert!(used);
        assert!((t - 3_625.0000000059445).abs() < 1e-9, "t {t}");
        // invalid sentinel -> tracker publishes null -> identity, unflagged
        let (t, used) = ggto_convert(245_425.0, None);
        assert!(!used);
        assert_eq!(t, 245_425.0);
        // NaN smuggled through JSON is refused the same way (fail-closed)
        let (t, used) = ggto_convert(245_425.0, Some(f64::NAN));
        assert!(!used);
        assert_eq!(t, 245_425.0);
    }

    /// build_meas_gal against the pinned RINEX fixture record (tests/
    /// fixtures/inav/rinex_gal_eval.json E02, live brdc_latest.rnx
    /// 2026-09-02): the parse -> Kepler -> clock(-BGD) chain produces a
    /// sane Meas, and the fail-closed gates refuse unknown PRNs and stale
    /// batches. (The bit-exact orbital/clock pins live in
    /// gps::broadcast's sat_at_txtime_gal tests against the same fixture.)
    #[test]
    fn build_meas_gal_ranges_the_fixture_and_fails_closed() {
        const RNX: &str = "\
     3.05           NAVIGATION DATA     MIXED               RINEX VERSION / TYPE
                                                            END OF HEADER
E02 2026 09 01 20 00 00 6.600067717955e-05 2.685851541173e-12 0.000000000000e+00
     2.700000000000e+01 1.567812500000e+02 3.000124967247e-09 2.036137310846e+00
     7.089227437973e-06 3.099278546870e-04 1.248344779015e-05 5.440630514145e+03
     2.448000000000e+05-3.911554813385e-08-2.069042130333e+00-2.793967723846e-08
     9.611374355521e-01 7.159375000000e+01 4.523971024409e-02-5.452012812503e-09
    -2.857261873569e-11 5.160000000000e+02 2.434000000000e+03 0.000000000000e+00
     3.120000000000e+00 0.000000000000e+00-3.026798367500e-09-3.958120942116e-09
     2.454640000000e+05                                                         ";
        let ephs = parse_rinex_gal(RNX);
        assert_eq!(ephs.len(), 1, "the I/NAV fixture record parses");
        // the fixture evaluations' receiver (ECEF m) and its approx LLA
        let site_m = [4_278_600.0, 636_800.0, 4_672_300.0];
        let site_lla = [47.4, 8.5, 500.0];
        let igp = HashMap::new();
        let t_tx = 244_800.0;
        let rho = 25_344_562.0;
        let m = build_meas_gal(2, rho, t_tx, &ephs, site_m, site_lla, &igp)
            .expect("healthy fresh batch ranges");
        assert!(!m.clock_free);
        // Galileo orbital radius ~29600 km
        let r = (m.sat[0] * m.sat[0] + m.sat[1] * m.sat[1] + m.sat[2] * m.sat[2]).sqrt();
        assert!((r - 29_600.0).abs() < 200.0, "radius {r} km");
        // pseudorange = (rho - tropo)/1000 + dt_sv*c with dt_sv within
        // ~1e-12 s of the pinned txtime clock (evaluation-time jitter);
        // tropo is bounded (0, 35] m so the difference sits in that band
        let base = rho / 1000.0 + 6.600396567344267e-05 * 299_792.458;
        let d_km = m.pseudorange - base;
        assert!(d_km < 0.0 && d_km > -0.036, "tropo-shaped residual, got {d_km} km");
        // unknown PRN -> None
        assert!(build_meas_gal(3, rho, t_tx, &ephs, site_m, site_lla, &igp).is_none());
        // ephemeris age gate (wrap-aware): 4 h past toe refuses
        let stale = 244_800.0 + GAL_EPH_MAX_AGE_S + 1.0;
        assert!(build_meas_gal(2, rho, stale, &ephs, site_m, site_lla, &igp).is_none());
        let early = 244_800.0 - GAL_EPH_MAX_AGE_S - 1.0;
        assert!(build_meas_gal(2, rho, early, &ephs, site_m, site_lla, &igp).is_none());
        // just inside the gate still ranges
        let ok = 244_800.0 + GAL_EPH_MAX_AGE_S - 1.0;
        assert!(build_meas_gal(2, rho, ok, &ephs, site_m, site_lla, &igp).is_some());
    }

    /// The GEO measurement builder fails closed on the MT9 gates.
    #[test]
    fn geo_meas_gates_fail_closed() {
        let geo = GeoEph {
            t0_s: 43_200.0,
            ura: 2,
            pos_m: [-17_600_000.0, -36_500_000.0, 80_000.0],
            vel_mps: [0.375, -0.125, 1.2],
            acc_mps2: [1.25e-5, -2.5e-5, 6.25e-5],
            agf0_s: 10.0 * 2.0f64.powi(-31),
            agf1_sps: 3.0 * 2.0f64.powi(-40),
        };
        let site_m = [1_334_751.79, -4_654_832.67, 4_137_255.32];
        let site_lla = [40.7, -74.0, 50.0];
        let t_tx = 3.0 * 86_400.0 + 43_296.0; // dt = 96 s
        let rho = 37_270_622.0;
        let mut geos = HashMap::new();
        geos.insert(131u8, geo.clone());
        let igp = HashMap::new();
        let m = build_meas_sbas(131, rho, t_tx, &geos, site_m, site_lla, &igp)
            .expect("healthy vector ranges");
        // pseudorange (km) ~ rho/1000 + clock*c; the GEO clock is ~4.9 ns
        assert!((m.sat[0] * 1000.0 - -17_600_294.9).abs() < 5.0, "sagnac x {}", m.sat[0]);
        assert!(!m.clock_free);
        // unknown PRN -> None
        assert!(build_meas_sbas(135, rho, t_tx, &geos, site_m, site_lla, &igp).is_none());
        // URA 15 (do-not-use) -> None
        let mut bad = geo.clone();
        bad.ura = 15;
        geos.insert(131, bad);
        assert!(build_meas_sbas(131, rho, t_tx, &geos, site_m, site_lla, &igp).is_none());
        // stale vector (|dt| > 360 s) -> None
        let mut stale = geo.clone();
        stale.t0_s = 43_200.0 - 1_000.0;
        geos.insert(131, stale);
        assert!(build_meas_sbas(131, rho, t_tx, &geos, site_m, site_lla, &igp).is_none());
    }
}
