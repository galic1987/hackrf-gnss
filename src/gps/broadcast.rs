//! GPS broadcast ephemeris: parse a RINEX-3 navigation file and compute
//! satellite ECEF position + clock correction (IS-GPS-200 Table 20-IV). This is
//! the missing input for `pvt::solve` — it turns real broadcast nav into the
//! satellite geometry the WLS solver needs. Ported from the Python reference
//! (`validation/rinex_nav.py`, `gps_engine.py`) and cross-checked against it.
//!
//! Angular RINEX fields are semicircles per the spec (and semicircles/s), but
//! BKG/IGS mixed files carry RADIANS — the unit is decided per constellation
//! on parse (see [`detect_ang_unit`]). Positions are returned in **metres**
//! (ICD units); the snapshot/PVT layer converts to km.
//!
//! Parsing is STRICT (round-11 review): a blank or malformed consumed field
//! rejects the whole record (it is skipped and counted in
//! [`RinexParse::rejected`]) — the pre-round-11 parser zero-filled them,
//! fabricating plausible-looking ephemerides out of corrupt lines.

use std::collections::HashMap;

const MU_E: f64 = 3.986005e14; // WGS-84 gravitational parameter, m^3/s^2
const OMEGA_E: f64 = 7.2921151467e-5; // Earth rotation rate, rad/s (ICD)
const F_REL: f64 = -4.442807633e-10; // relativistic clock constant
const WEEK_S: f64 = 604800.0;
const PI: f64 = std::f64::consts::PI;

/// One satellite's broadcast ephemeris (SI units, angles in radians).
/// `sys`: 0 = GPS, 1 = BeiDou (BDS ephemeris times are stored as
/// GPST-equivalent seconds-of-week — BDT SOW + 14 s — so one t_tx timescale
/// serves every constellation; see beidou_d1.rs).
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct BrdcEph {
    #[serde(default)]
    pub sys: u8,
    pub prn: u8,
    pub toe: f64,
    pub toc: f64,
    pub week: f64,
    pub sqrt_a: f64,
    pub e: f64,
    pub m0: f64,
    pub delta_n: f64,
    pub omega0: f64,
    pub omega: f64,
    pub i0: f64,
    pub idot: f64,
    pub omega_dot: f64,
    pub cuc: f64,
    pub cus: f64,
    pub crc: f64,
    pub crs: f64,
    pub cic: f64,
    pub cis: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub tgd: f64,
    /// Issue of data, ephemeris. Some for self-decoded LNAV AND for BRDC
    /// GPS records (RINEX-3 line 2 field 1 = IODE, the 8 LSB of IODC);
    /// None only where truly unverifiable (BDS BRDC records carry AODE,
    /// a different quantity, and stay None). WAAS LT corrections are valid
    /// only when their IOD matches this (DO-229D Table A-10 Note 3).
    #[serde(default)]
    pub iode: Option<u8>,
    /// SV health. GPS: RINEX-3.05 nav line 7 field 2 (= LNAV subframe 1
    /// word 3 bits 17-22, IS-GPS-200 20.3.3.3.1.4 — 0 = healthy). BDS:
    /// line 7 field 2 = SatH1 (D1 subframe 1 word 2 bit 13). None where
    /// the source record doesn't carry it. ENFORCED at RINEX selection:
    /// a record with known health != 0 is hard-excluded (parse_gps_record
    /// and parse_bds_record alike), so an unhealthy SV's ephemeris can
    /// never win — None stays eligible. The live self-decode refresh
    /// applies the same law two-sided (src/live.rs): an unhealthy decode
    /// is rejected, and a strictly newer unhealthy issue drops the
    /// stale-healthy incumbent.
    #[serde(default)]
    pub health: Option<u8>,
    /// Fit interval in hours (RINEX-3.05 GPS nav line 8 field 2; the spec's
    /// 0 = "not known" maps to None). LNAV carries only the 1-bit flag
    /// (subframe 2 word 10 bit 17, IS-GPS-200 20.3.4.4): 0 -> Some(4.0),
    /// 1 -> None ("> 4 h"; the actual span needs IODC + Table 20-XII).
    /// BDS records carry AODC in that slot — a different quantity — and
    /// stay None. Selection prefers records whose fit window (toe ..
    /// toe + fit_h) covers the constellation's newest issue epoch.
    #[serde(default)]
    pub fit_h: Option<f64>,
    /// Issue of data, clock. Present in RINEX-3.05 GPS nav (line 7 field 4,
    /// carried since RINEX 2.10 — the round-10/11 review draft claiming
    /// "IODC appears only in RINEX 4" was wrong) and in LNAV subframe 1
    /// (word 3 bits 23-24 MSB + word 8 bits 1-8). None for BDS (its records
    /// carry AODC/AODE instead).
    #[serde(default)]
    pub iodc: Option<u16>,
    /// When WE obtained this record (unix epoch s): decode time for
    /// self-decoded LNAV/D1; the BRDC file mtime when the caller attaches
    /// it (the text-only RINEX parsers leave it None). Lifecycle honesty:
    /// the tracker_eph.json cache envelope is re-stamped with a fresh wall
    /// clock on every write and must never be read as issue freshness
    /// (round-11 review).
    #[serde(default)]
    pub rx_epoch: Option<f64>,
}

pub(crate) fn fld(line: &str, a: usize, b: usize) -> &str {
    let n = line.len();
    if a >= n {
        ""
    } else {
        &line[a..b.min(n)]
    }
}

/// Strict RINEX-3 D/E-exponent float field (e.g. "-1.234567890123D-04"):
/// None on a blank OR malformed field — never a silent 0.0 (round-11
/// review: zero-filling fabricated plausible ephemerides out of corrupt
/// lines). Non-finite spellings ("NaN"/"inf") parse fine in Rust but are
/// not RINEX content, so they are rejected too.
pub(crate) fn df_strict(s: &str) -> Option<f64> {
    let t = s.trim();
    if t.is_empty() {
        return None;
    }
    t.replace('D', "E")
        .replace('d', "E")
        .parse::<f64>()
        .ok()
        .filter(|v| v.is_finite())
}

/// Optional-metadata variant: blank/absent -> Ok(None) (RINEX-3.05 §3 allows
/// trimming trailing blanks), malformed -> Err(()) — the record must reject.
pub(crate) fn df_opt(s: &str) -> Result<Option<f64>, ()> {
    if s.trim().is_empty() {
        Ok(None)
    } else {
        df_strict(s).map(Some).ok_or(())
    }
}

/// Strict integer field (record epoch / PRN): None on blank/malformed.
pub(crate) fn iparse_strict(s: &str) -> Option<i64> {
    s.trim().parse().ok()
}

/// Inclination evidence bands for the radians-vs-semicircles unit decision
/// (round-11 review of the old "first record's |i0| > 0.6" heuristic).
/// RINEX-3.05 MANDATES radians (Table A6 fn. ***: the generator converts
/// semi-circle broadcasts to radians); a non-conforming generator can still
/// write semicircles, which the vote catches. Live MEO
/// GNSS inclinations span 53-59 deg and drifting BDS IGSOs exceed 60 deg
/// (live BRDC 2026-08-26: C09 at i0 = 1.0523 rad = 60.3 deg): 0.93-1.06 rad
/// or 0.294-0.336 semicircles. A record's raw |i0| is rad-like in
/// [0.85, 1.10], sc-like in [0.25, 0.36], neutral below 0.25 (BDS GEOs
/// ~0.02-0.12 carry no unit evidence), and physically impossible under
/// either unit anywhere else — such a record earns no vote and is rejected
/// in the main pass.
pub(crate) const I0_RAD_LIKE: (f64, f64) = (0.85, 1.10);
pub(crate) const I0_SC_LIKE: (f64, f64) = (0.25, 0.36);
/// The grey band between the two hypotheses: impossible under either unit,
/// so a record whose raw i0 sits inside (0.36, 0.85) is rejected regardless
/// of the constellation's chosen unit.
pub(crate) const I0_GREY: (f64, f64) = (0.36, 0.85);

/// Angle-unit verdict for one constellation of a RINEX-3 nav file.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AngUnit {
    /// A non-conforming generator that skipped the mandated conversion
    /// (RINEX-3.05 Table A6 footnote ***: semi-circle angles "have to be
    /// converted to radians by the RINEX generator"); multiplied by PI.
    Semicircles,
    /// RINEX-3.05 as specified: nav-file angles are already radians.
    /// The zero-evidence default too (Table A6 fn. ***; the observed
    /// BKG/IGS 3.05 population conforms).
    #[default]
    Radians,
    /// Contradictory content with no supermajority (rad-like AND sc-like
    /// records present, neither holding >= 2/3 of the decided votes): the
    /// file is internally inconsistent — fail closed; every record of the
    /// constellation is rejected rather than parsed under a guessed unit.
    Ambiguous,
}

impl AngUnit {
    /// The multiplier applied to raw angle fields, or None when the unit is
    /// ambiguous (fail closed).
    pub(crate) fn factor(self) -> Option<f64> {
        match self {
            AngUnit::Semicircles => Some(PI),
            AngUnit::Radians => Some(1.0),
            AngUnit::Ambiguous => None,
        }
    }
}

/// Per-constellation unit decision by per-record i0 votes over the WHOLE
/// constellation (no early-exit cap: BKG files sort records by PRN, and a
/// 64-record early scan of a BDS constellation can see nothing but
/// unit-neutral GEOs). A record with an unparseable or physically
/// impossible i0 earns no vote — one bad record cannot flip the file.
/// Decision law: a unit voted by ALL decided votes wins outright; when
/// BOTH units drew votes the winner must hold a strict majority that is
/// also a >= 2/3 supermajority of the decided votes, else the verdict is
/// Ambiguous and fails closed (a lone dissenting or corrupt-but-plausible
/// record can no longer deadlock the whole constellation, but a genuinely
/// contested file is still rejected rather than guessed). No decided
/// votes -> Radians — the spec default: RINEX-3.05 Table A6 footnote ***
/// mandates the radians conversion by the generator (verified against
/// rinex305.pdf 2026-08-26; the observed BKG/IGS population conforms). A
/// GEO-only BDS constellation lands here — GEOs carry no evidence and
/// their Kepler math is unused by this station anyway.
pub(crate) fn detect_ang_unit(lines: &[&str], hdr_end: usize, sys: char) -> AngUnit {
    let (mut rad, mut sc) = (0usize, 0usize);
    let mut j = hdr_end;
    while j + 4 < lines.len() {
        let ln = lines[j];
        if ln.starts_with(sys) && ln.len() > 4 {
            if let Some(i0) = df_strict(fld(lines[j + 4], 4, 23)) {
                let m = i0.abs();
                if (I0_RAD_LIKE.0..=I0_RAD_LIKE.1).contains(&m) {
                    rad += 1;
                } else if (I0_SC_LIKE.0..=I0_SC_LIKE.1).contains(&m) {
                    sc += 1;
                }
            }
            j += 8;
            continue;
        }
        j += 1;
    }
    let decided = rad + sc;
    if decided == 0 {
        return AngUnit::Radians; // spec default (Table A6 fn. ***)
    }
    if sc == 0 {
        return AngUnit::Radians;
    }
    if rad == 0 {
        return AngUnit::Semicircles;
    }
    // both units drew votes: the winner needs >= 2/3 of them (integer
    // form, no float rounding)
    let (win, unit) = if rad > sc { (rad, AngUnit::Radians) } else { (sc, AngUnit::Semicircles) };
    if win * 3 >= decided * 2 {
        unit
    } else {
        AngUnit::Ambiguous
    }
}

/// Per-record i0 sanity against the chosen unit (round-11): the record's raw
/// i0 must fit the unit's range — [0, 1] semicircles, [0, PI] radians — and
/// must not sit in the grey band that is physically impossible under either
/// unit. (The unit's garbage protection is the grey band plus the vote
/// detection; the sanity range deliberately does NOT gate inclination —
/// drifting BDS IGSOs legitimately exceed 60 deg, live BRDC 2026-08-26 C09.)
/// Round-14 minority-unit quarantine: a record whose raw i0 sits in the
/// LOSING unit's evidence band is rejected — admitted, it would get every
/// angular field scaled by the winner's factor (silently garbage orbit).
/// Its i0 vote in detect_ang_unit still counted (that is how the winner was
/// decided); the record just never parses or enters the per-PRN selection.
pub(crate) fn i0_sane(i0_raw: f64, unit: AngUnit) -> bool {
    if i0_raw < 0.0 || (I0_GREY.0..I0_GREY.1).contains(&i0_raw) {
        return false;
    }
    let loser_band = match unit {
        AngUnit::Semicircles => I0_RAD_LIKE,
        AngUnit::Radians => I0_SC_LIKE,
        AngUnit::Ambiguous => return false,
    };
    if (loser_band.0..=loser_band.1).contains(&i0_raw) {
        return false;
    }
    match unit {
        AngUnit::Semicircles => i0_raw <= 1.0,
        AngUnit::Radians => i0_raw <= PI,
        AngUnit::Ambiguous => false,
    }
}

/// Parse outcome for one constellation's RINEX-3 nav records (round-11
/// review): the accepted ephemerides plus an honest rejection ledger.
#[derive(Debug, Default)]
pub struct RinexParse {
    /// newest valid issue per PRN — see parse_rinex_gps
    pub ephs: HashMap<u8, BrdcEph>,
    /// records skipped: any malformed/blank consumed field, an i0 failing
    /// the chosen unit's sanity band, a known-unhealthy SV (health hard-
    /// excluded), or wholesale under a fail-closed Ambiguous unit verdict
    pub rejected: usize,
    /// the angle-unit verdict applied to every angular field
    pub unit: AngUnit,
}

/// Days from the proleptic Gregorian calendar to a Julian Day Number.
fn jdn(y: i64, m: i64, d: i64) -> i64 {
    let a = (14 - m) / 12;
    let yy = y + 4800 - a;
    let mm = m + 12 * a - 3;
    d + (153 * mm + 2) / 5 + 365 * yy + yy / 4 - yy / 100 + yy / 400 - 32045
}

/// GPS seconds-of-week for a GPS RINEX nav-record epoch. RINEX-3 G-record
/// epochs are already GPS time (RINEX 3.05 time-system code G = GPST;
/// GLONASS is the system whose records ride UTC) — adding the 18 s leap was
/// double-counting, proven empirically on the live BRDC: across 126 G
/// records, toe - toc == 0 exactly when the epoch is read as GPST, and -18 s
/// with the old leap-adding read. The 18 s toc displacement made sat_clock's
/// af1*(t-toc) wrong by up to ~11 cm of range on high-drift clocks.
fn gps_sow(y: i64, mo: i64, d: i64, h: i64, mi: i64, s: i64) -> f64 {
    let gps_epoch = jdn(1980, 1, 6);
    let days = jdn(y, mo, d) - gps_epoch;
    let secs = days * 86400 + h * 3600 + mi * 60 + s;
    (secs as f64).rem_euclid(WEEK_S)
}

/// Parse the GPS records of a RINEX-3 MIXED navigation file, keeping the
/// newest VALID issue per PRN. Returns {prn: BrdcEph} (the rejection ledger
/// is logged; use [`parse_rinex_gps_nav`] to inspect it).
///
/// Angle units: RINEX-3.05 mandates radians (Table A6 fn. ***), and BKG's
/// mixed files conform (i0 = 0.957 rad, not 0.306 sc) — but a generator that
/// skips the conversion writes semicircles, so the unit is decided per
/// constellation by
/// per-record i0 votes ([`detect_ang_unit`]), applied to every angle field —
/// the previous unconditional xPI produced near-equatorial orbits
/// 15,000-44,000 km off (found by cross-checking against SGP4/TLE
/// positions), and the first-record-only heuristic let one odd record pick
/// the unit.
///
/// Selection: RINEX weeks are continuous (3.05 §4.1.1 "the GPS week reported
/// in the RINEX navigation message files is a continuous number without
/// roll-over"), so the (week, toe) tuple compare is rollover-exact across
/// the week boundary — unlike the old bare `toe > toe`, which kept last
/// week's record over a fresh one with a small toe. A record with KNOWN
/// nonzero SV health never enters the map (hard-excluded in
/// parse_gps_record), and among a PRN's candidates a record whose fit
/// window provably covers the constellation's newest issue epoch beats
/// one whose window has expired (an out-of-fit record stays usable when
/// no in-fit alternative exists).
pub fn parse_rinex_gps(text: &str) -> HashMap<u8, BrdcEph> {
    let r = parse_rinex_gps_nav(text);
    if r.rejected > 0 {
        eprintln!(
            "parse_rinex_gps: {} GPS record(s) rejected (unit {:?})",
            r.rejected, r.unit
        );
    }
    r.ephs
}

/// Full GPS-record parse with the rejection ledger (round-11 review).
pub fn parse_rinex_gps_nav(text: &str) -> RinexParse {
    let lines: Vec<&str> = text.lines().collect();
    let mut hdr = 0usize;
    while hdr < lines.len() && !lines[hdr].contains("END OF HEADER") {
        hdr += 1;
    }
    hdr += 1;
    let unit = detect_ang_unit(&lines, hdr, 'G');
    let mut recs: Vec<BrdcEph> = Vec::new();
    let mut rejected = 0usize;
    let mut i = hdr;
    while i < lines.len() {
        let ln = lines[i];
        if ln.is_empty() || !ln.starts_with('G') || i + 7 >= lines.len() {
            i += 1;
            continue;
        }
        let b: Vec<&str> = (0..7).map(|k| lines[i + 1 + k]).collect();
        match parse_gps_record(ln, &b, unit) {
            Some(e) => recs.push(e),
            None => rejected += 1,
        }
        i += 8;
    }
    // Selection per PRN, newest valid issue — with a fit-window
    // preference. The reference epoch is the constellation's newest
    // (week, toe): the text-only parse's only notion of "now" is the file
    // itself. A record whose fit window provably covers the reference
    // (toe .. toe + fit_h hours) beats one whose window has provably
    // expired; an out-of-fit record is still usable when no in-fit
    // alternative exists. fit_h unknown -> not provably in-fit, and
    // recency decides between equals. RINEX weeks are continuous (3.05
    // §4.1.1), so the (week, toe) tuple compare is rollover-exact.
    let t_ref = recs
        .iter()
        .map(|e| (e.week, e.toe))
        .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let in_fit = |e: &BrdcEph| match (t_ref, e.fit_h) {
        (Some((w, t)), Some(f)) => {
            let age = (w - e.week) * WEEK_S + (t - e.toe);
            (0.0..=f * 3600.0).contains(&age)
        }
        _ => false,
    };
    let mut out: HashMap<u8, BrdcEph> = HashMap::new();
    for e in recs {
        out.entry(e.prn)
            .and_modify(|cur| {
                let replace = if in_fit(&e) != in_fit(cur) {
                    in_fit(&e)
                } else {
                    (e.week, e.toe) > (cur.week, cur.toe)
                };
                if replace {
                    *cur = e.clone();
                }
            })
            .or_insert(e);
    }
    RinexParse { ephs: out, rejected, unit }
}

/// One 8-line GPS nav record -> BrdcEph, STRICT (round-11 review): every
/// consumed field must parse — a blank/malformed core field rejects the
/// record (None) where the old df() silently zero-filled. Optional metadata
/// (SV health, IODC, fit interval) is blank-tolerant (RINEX-3.05 §3 allows
/// trimmed trailing blanks) but rejects when present-and-malformed. A
/// KNOWN-unhealthy record (health Some, != 0) is rejected outright — an
/// unhealthy SV's ephemeris must never win selection. The raw i0 must
/// pass [`i0_sane`] for the file's unit verdict.
///
/// RINEX-3.05 GPS nav record layout (Appendix A6, "GNSS Navigation Message
/// File — GPS Data Record Description"; the 8-line record is unchanged since
/// RINEX 2.10):
///   L1 epoch (toc, already GPST — see gps_sow) + af0/af1/af2
///   L2 IODE, Crs, Delta n, M0
///   L3 Cuc, e, Cus, sqrtA
///   L4 toe, Cic, OMEGA0, Cis
///   L5 i0, Crc, omega, OMEGA DOT
///   L6 IDOT, codes on L2, GPS week (continuous, §4.1.1), L2 P flag
///   L7 SV accuracy, SV health, TGD, IODC
///   L8 transmission time of message, fit interval (h; 0 = unknown),
///      spare, spare
fn parse_gps_record(ln: &str, b: &[&str], unit: AngUnit) -> Option<BrdcEph> {
    let ang = unit.factor()?; // Ambiguous fails closed: no guessed unit
    let prn = u8::try_from(iparse_strict(fld(ln, 1, 3))?).ok()?;
    let (y, mo, d) = (
        iparse_strict(fld(ln, 4, 8))?,
        iparse_strict(fld(ln, 9, 11))?,
        iparse_strict(fld(ln, 12, 14))?,
    );
    let (h, mi, s) = (
        iparse_strict(fld(ln, 15, 17))?,
        iparse_strict(fld(ln, 18, 20))?,
        iparse_strict(fld(ln, 21, 23))?,
    );
    // orbit field j on line `l`: 3-space indent, 19-char columns
    let f = |l: usize, j: usize| df_strict(fld(b[l], 4 + j * 19, 4 + (j + 1) * 19));
    let i0_raw = f(3, 0)?;
    if !i0_sane(i0_raw, unit) {
        return None;
    }
    // optional metadata: blank -> None, malformed or out of range -> reject
    let health = match df_opt(fld(b[5], 23, 42)).ok()? {
        Some(v) if (0.0..=63.0).contains(&v) => Some(v as u8), // 6-bit field
        Some(_) => return None,
        None => None,
    };
    // A known-unhealthy SV is hard-excluded at the source (IS-GPS-200
    // 20.3.3.3.1.4: 0 = healthy): its ephemeris must never win selection,
    // even when it is the only record on file for the PRN. None (no
    // health field) stays eligible.
    if matches!(health, Some(h) if h != 0) {
        return None;
    }
    let iodc = match df_opt(fld(b[5], 61, 80)).ok()? {
        Some(v) if (0.0..=1023.0).contains(&v) => Some(v as u16), // 10-bit
        Some(_) => return None,
        None => None,
    };
    // fit interval in hours; RINEX writes 0 for "not known" -> None
    let fit_h = match df_opt(fld(b[6], 23, 42)).ok()? {
        Some(v) if v > 0.0 => Some(v),
        Some(_) => None,
        None => None,
    };
    let iode_v = f(0, 0)?;
    if !(0.0..=255.0).contains(&iode_v) {
        return None;
    }
    Some(BrdcEph {
        sys: 0,
        prn,
        // RINEX-3 GPS nav line 2 field 1 IS the IODE (8 LSB of IODC,
        // DO-229D's link between SBAS long-term corrections and the GPS
        // broadcast ephemeris). The old comment claiming RINEX carries
        // no IODE was wrong (round-10 review).
        iode: Some(iode_v as u8),
        af0: df_strict(fld(ln, 23, 42))?,
        af1: df_strict(fld(ln, 42, 61))?,
        af2: df_strict(fld(ln, 61, 80))?,
        crs: f(0, 1)?,
        delta_n: f(0, 2)? * ang,
        m0: f(0, 3)? * ang,
        cuc: f(1, 0)?,
        e: f(1, 1)?,
        cus: f(1, 2)?,
        sqrt_a: f(1, 3)?,
        toe: f(2, 0)?,
        cic: f(2, 1)?,
        omega0: f(2, 2)? * ang,
        cis: f(2, 3)?,
        i0: i0_raw * ang,
        crc: f(3, 1)?,
        omega: f(3, 2)? * ang,
        omega_dot: f(3, 3)? * ang,
        idot: f(4, 0)? * ang,
        week: f(4, 2)?,
        health,
        fit_h,
        iodc,
        tgd: f(5, 2)?,
        toc: gps_sow(y, mo, d, h, mi, s),
        rx_epoch: None, // text-only parser: the caller attaches the file mtime
    })
}

fn kepler_e(m: f64, ecc: f64) -> f64 {
    let mut ek = m;
    for _ in 0..12 {
        ek -= (ek - ecc * ek.sin() - m) / (1.0 - ecc * ek.cos());
    }
    ek
}

/// Week-rollover-aware time difference: folds dt into (-half-week,
/// +half-week]. Used both for propagation (t - toe in the Kepler model) and
/// for ephemeris-issue SELECTION (the newer of two issues sits a positive
/// wrap-aware difference ahead) — the rollover-safe compare for records
/// whose week fields are not comparable (e.g. LNAV's 10-bit broadcast week
/// vs RINEX's continuous week).
pub fn wrap_tk(mut tk: f64) -> f64 {
    if tk > WEEK_S / 2.0 {
        tk -= WEEK_S;
    } else if tk < -WEEK_S / 2.0 {
        tk += WEEK_S;
    }
    tk
}

/// Satellite ECEF position (metres) at GPS time-of-week `t` (IS-GPS-200 20-IV).
pub fn sat_pos_ecef(e: &BrdcEph, t: f64) -> [f64; 3] {
    let a = e.sqrt_a * e.sqrt_a;
    let n0 = (MU_E / (a * a * a)).sqrt();
    let tk = wrap_tk(t - e.toe);
    let mk = e.m0 + (n0 + e.delta_n) * tk;
    let ek = kepler_e(mk, e.e);
    let (se, ce) = (ek.sin(), ek.cos());
    let vk = ((1.0 - e.e * e.e).sqrt() * se).atan2(ce - e.e);
    let phik = vk + e.omega;
    let (s2, c2) = ((2.0 * phik).sin(), (2.0 * phik).cos());
    let uk = phik + e.cus * s2 + e.cuc * c2;
    let rk = a * (1.0 - e.e * ce) + e.crs * s2 + e.crc * c2;
    let ik = e.i0 + e.cis * s2 + e.cic * c2 + e.idot * tk;
    let (xp, yp) = (rk * uk.cos(), rk * uk.sin());
    let om = e.omega0 + (e.omega_dot - OMEGA_E) * tk - OMEGA_E * e.toe;
    let (co, so, ci, si) = (om.cos(), om.sin(), ik.cos(), ik.sin());
    [xp * co - yp * ci * so, xp * so + yp * ci * co, yp * si]
}

/// Satellite clock correction dt_sv (seconds), incl. relativity and L1 group delay.
pub fn sat_clock(e: &BrdcEph, t: f64) -> f64 {
    let dt = wrap_tk(t - e.toc);
    let poly = e.af0 + e.af1 * dt + e.af2 * dt * dt;
    let a = e.sqrt_a * e.sqrt_a;
    let n0 = (MU_E / (a * a * a)).sqrt();
    let tk = wrap_tk(t - e.toe);
    let mk = e.m0 + (n0 + e.delta_n) * tk;
    let ek = kepler_e(mk, e.e);
    poly + F_REL * e.e * e.sqrt_a * ek.sin() - e.tgd
}

pub const C_LIGHT: f64 = 299_792_458.0;
pub const EARTH_RATE: f64 = OMEGA_E;

#[cfg(test)]
mod tests {
    use super::*;

    fn circular_eph() -> BrdcEph {
        // a nearly-ideal GPS orbit: sqrt(A)=5153.6 -> A ~ 26560 km, e=0
        BrdcEph { sqrt_a: 5153.6, e: 0.0, toe: 0.0, toc: 0.0, ..Default::default() }
    }

    #[test]
    fn orbit_radius_is_gps_nominal() {
        let e = circular_eph();
        let p = sat_pos_ecef(&e, 0.0);
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        let a = (e.sqrt_a * e.sqrt_a) as f64; // metres
        assert!((r - a).abs() < 1.0, "radius {r} vs A {a}");
        assert!((r - 26_560_000.0).abs() < 30_000.0, "radius {r} not ~26560 km");
    }

    #[test]
    fn clock_reduces_to_af0_at_toc() {
        let mut e = circular_eph();
        e.af0 = 1.5e-4;
        e.af1 = 0.0;
        // at toc, with e=0 the relativistic term vanishes -> dt == af0 - tgd
        let dt = sat_clock(&e, e.toc);
        assert!((dt - 1.5e-4).abs() < 1e-12, "dt {dt}");
    }

    #[test]
    fn position_advances_over_time() {
        // a real-ish orbit moves ~3.9 km/s; 60 s apart must be kilometres apart
        let e = circular_eph();
        let p0 = sat_pos_ecef(&e, 0.0);
        let p1 = sat_pos_ecef(&e, 60.0);
        let d = ((p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2) + (p1[2] - p0[2]).powi(2)).sqrt();
        assert!(d > 100_000.0 && d < 300_000.0, "60 s displacement {d} m");
    }

    #[test]
    fn parses_a_minimal_rinex_record() {
        // one synthetic GPS record in RINEX-3 layout (values need not be physical)
        let txt = "\
     3.05           NAVIGATION DATA     MIXED               RINEX VERSION / TYPE
                                                            END OF HEADER
G01 2026 08 20 00 00 00-1.000000000000D-04 0.000000000000D+00 0.000000000000D+00
     1.000000000000D+02-5.000000000000D+00 4.000000000000D-09 3.000000000000D-01
     1.000000000000D-06 4.000000000000D-03 5.000000000000D-06 5.153600000000D+03
     3.456000000000D+05 1.000000000000D-08-2.500000000000D+00 2.000000000000D-08
     3.000000000000D-01 2.000000000000D+02-5.000000000000D-01-8.000000000000D-09
    -2.600000000000D-10 0.000000000000D+00 2.350000000000D+03 0.000000000000D+00
     0.000000000000D+00 0.000000000000D+00-1.000000000000D-08 1.000000000000D+02
     3.456000000000D+05 0.000000000000D+00 0.000000000000D+00 0.000000000000D+00";
        let ephs = parse_rinex_gps(txt);
        assert_eq!(ephs.len(), 1);
        let e = &ephs[&1];
        assert_eq!(e.prn, 1);
        assert!((e.sqrt_a - 5153.6).abs() < 1e-6);
        assert!((e.toe - 345600.0).abs() < 1e-6);
        // toc regression (round-10 review): RINEX-3 G epochs are GPST, so a
        // record whose calendar epoch equals toe's GPST second must parse
        // toc == toe exactly; the old code added the 18 s GPS-UTC leap on
        // top (toe-toc == -18 s on every one of 126 live BRDC records).
        assert!((e.toc - 345600.0).abs() < 1e-6, "toc {} must equal toe (GPST epochs)", e.toc);
        assert!((e.af0 - (-1.0e-4)).abs() < 1e-12);
        // line 2 field 1 is the IODE (100 in this fixture)
        assert_eq!(e.iode, Some(100));
        // line 7: SV accuracy 0, SV health 0, TGD -1e-8, IODC 100;
        // line 8 fit interval 0.0 = "not known" -> None (RINEX-3.05 A6)
        assert_eq!(e.health, Some(0));
        assert_eq!(e.iodc, Some(100));
        assert_eq!(e.fit_h, None);
        // a text-only parse cannot know when WE received the record
        assert_eq!(e.rx_epoch, None);
        // angular field converted semicircles -> radians (M0 = 0.3 * pi)
        assert!((e.m0 - 0.3 * PI).abs() < 1e-9);
        // and the orbit it describes is a sane GPS radius
        let p = sat_pos_ecef(e, e.toe);
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        assert!(r > 26_000_000.0 && r < 27_100_000.0, "radius {r}");
    }

    /// round-11 review: the fixture record layout as a reusable builder —
    /// `i0` is line 5 field 1 (the unit-detection field); every other field
    /// fixed and well-formed. Every field is exactly 19 columns with the
    /// first field at col 4 (writers left-pad positive numbers with a
    /// space, so a positive first field reads as 5 leading blanks).
    fn rinex_record(prn: u8, week: f64, toe: f64, i0: &str) -> String {
        rinex_record_hf(prn, week, toe, i0, "0.000000000000D+00", "4.000000000000D+00")
    }

    /// rinex_record with L7 field 2 (SV health) and L8 field 2 (fit
    /// interval hours) parameterised, for the health/fit selection tests.
    fn rinex_record_hf(prn: u8, week: f64, toe: f64, i0: &str, health: &str, fit: &str) -> String {
        format!(
            "G{prn:02} 2026 08 20 00 00 00-1.000000000000D-04 0.000000000000D+00 0.000000000000D+00\n\
             \x20    1.000000000000D+02-5.000000000000D+00 4.000000000000D-09 3.000000000000D-01\n\
             \x20    1.000000000000D-06 4.000000000000D-03 5.000000000000D-06 5.153600000000D+03\n\
             \x20   {toe:19.12E} 1.000000000000D-08-2.500000000000D+00 2.000000000000D-08\n\
             \x20   {i0:>19} 2.000000000000D+02-5.000000000000D-01-8.000000000000D-09\n\
             \x20   -2.600000000000D-10 0.000000000000D+00{week:19.12E} 0.000000000000D+00\n\
             \x20    0.000000000000D+00{health:>19}-1.000000000000D-08 1.000000000000D+02\n\
             \x20   {toe:19.12E}{fit:>19} 0.000000000000D+00 0.000000000000D+00"
        )
    }

    const RNX_HDR: &str = "\
     3.05           NAVIGATION DATA     MIXED               RINEX VERSION / TYPE
                                                            END OF HEADER
";

    #[test]
    fn malformed_core_field_rejects_the_record() {
        // sqrtA line-3 field 4 blanked out: the old parser zero-filled it
        // (a 0 m orbit!); strict parsing must reject, never zero-fill.
        // (replacement strings are exactly the 18 chars of the number, so
        // the line keeps its 80-col record alignment)
        let bad = rinex_record(5, 2350.0, 345600.0, "3.000000000000D-01")
            .replace("5.153600000000D+03", &" ".repeat(18));
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{bad}"));
        assert_eq!(r.rejected, 1);
        assert!(r.ephs.is_empty(), "malformed record must not enter the map");
        // non-finite spellings parse in Rust but are not RINEX content
        let nan = rinex_record(5, 2350.0, 345600.0, "3.000000000000D-01")
            .replace("5.153600000000D+03", &format!("{:>18}", "NaN"));
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{nan}"));
        assert_eq!(r.rejected, 1);
        assert!(r.ephs.is_empty());
    }

    #[test]
    fn blank_optional_metadata_is_tolerated_malformed_rejects() {
        // fit interval blank (RINEX allows trimmed trailing blanks): accepted
        let blank_fit = rinex_record(5, 2350.0, 345600.0, "3.000000000000D-01")
            .replace("4.000000000000D+00", &" ".repeat(18));
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{blank_fit}"));
        assert_eq!(r.rejected, 0);
        assert_eq!(r.ephs[&5].fit_h, None);
        // ...but a MALFORMED fit field rejects the record
        let bad_fit = rinex_record(5, 2350.0, 345600.0, "3.000000000000D-01")
            .replace("4.000000000000D+00", &format!("{:>18}", "xyzzy"));
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{bad_fit}"));
        assert_eq!(r.rejected, 1);
        assert!(r.ephs.is_empty());
    }

    #[test]
    fn grey_band_i0_rejects_and_one_bad_record_cannot_flip_the_unit() {
        // a semicircles file (two sc-like records) plus one corrupt record
        // whose i0 is physically impossible under either unit: the corrupt
        // record earns no vote, the file stays semicircles, and only the
        // corrupt record is rejected.
        let good1 = rinex_record(5, 2350.0, 345600.0, "3.000000000000D-01");
        let good2 = rinex_record(8, 2350.0, 345600.0, "3.100000000000D-01");
        let bad = rinex_record(9, 2350.0, 345600.0, "5.000000000000D-01"); // 0.5: grey under both units
        let txt = format!("{RNX_HDR}{good1}\n{good2}\n{bad}");
        let r = parse_rinex_gps_nav(&txt);
        assert_eq!(r.unit, AngUnit::Semicircles);
        assert_eq!(r.rejected, 1, "only the grey-band record is rejected");
        assert!((r.ephs[&5].m0 - 0.3 * PI).abs() < 1e-9, "unit not flipped");
        assert!(r.ephs.contains_key(&8) && !r.ephs.contains_key(&9));
    }

    #[test]
    fn contradictory_unit_content_fails_closed() {
        // one sc-like AND one rad-like record: the file is internally
        // inconsistent — reject BOTH, never guess a unit
        let sc = rinex_record(5, 2350.0, 345600.0, "3.000000000000D-01");
        let rad = rinex_record(8, 2350.0, 345600.0, "9.600000000000D-01");
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{sc}\n{rad}"));
        assert_eq!(r.unit, AngUnit::Ambiguous);
        assert_eq!(r.rejected, 2);
        assert!(r.ephs.is_empty());
    }

    #[test]
    fn radians_file_detects_and_converts_nothing() {
        let rad = rinex_record(5, 2350.0, 345600.0, "9.599885583587D-01"); // ~55 deg in rad
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{rad}"));
        assert_eq!(r.unit, AngUnit::Radians);
        assert_eq!(r.rejected, 0);
        let e = &r.ephs[&5];
        assert!((e.m0 - 0.3).abs() < 1e-12, "radians pass through unscaled");
        assert!((e.i0 - 0.9599885583587).abs() < 1e-9);
    }

    #[test]
    fn newest_issue_selection_is_week_rollover_exact() {
        // same PRN across the GPS week boundary: the fresh issue (week+1,
        // small toe) must displace the old one (week, toe near 604800) — the
        // old bare `toe > toe` compare kept the STALE record here. RINEX
        // weeks are continuous (3.05 §4.1.1), so (week, toe) is exact.
        let old = rinex_record(5, 2350.0, 604_000.0, "3.000000000000D-01");
        let new = rinex_record(5, 2351.0, 200.0, "3.000000000000D-01");
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{old}\n{new}"));
        assert_eq!(r.rejected, 0);
        let e = &r.ephs[&5];
        assert_eq!(e.week, 2351.0, "the next-week issue must win");
        assert_eq!(e.toe, 200.0);
        // order-independent: the same outcome when the file lists new first
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{new}\n{old}"));
        assert_eq!(r.ephs[&5].week, 2351.0);
    }

    #[test]
    fn unit_vote_supermajority_resolves_a_lone_dissenter() {
        // 134 sc-like records vs 1 rad-like dissenter: the supermajority
        // rules. The old any-mix law failed the WHOLE constellation closed
        // over one corrupt-but-plausible record — and round-14 quarantines
        // the dissenter itself: it voted for the losing unit, so admitting
        // it would scale every angular field by the winner's factor (a
        // silently garbage orbit).
        let mut txt = String::from(RNX_HDR);
        for k in 0..134u32 {
            let prn = (k % 32 + 1) as u8;
            let rec = rinex_record(prn, 2350.0, 345_600.0 + k as f64 * 1800.0, "3.000000000000D-01");
            txt.push_str(&rec);
            txt.push('\n');
        }
        txt.push_str(&rinex_record(33, 2350.0, 345_600.0, "9.600000000000D-01"));
        let r = parse_rinex_gps_nav(&txt);
        assert_eq!(r.unit, AngUnit::Semicircles, "134 vs 1: the supermajority wins");
        assert_eq!(r.rejected, 1, "the losing-unit dissenter is quarantined, not scaled");
        assert_eq!(r.ephs.len(), 32);
        assert!(!r.ephs.contains_key(&33), "the minority-unit record never enters the map");
        assert!((r.ephs[&1].m0 - 0.3 * PI).abs() < 1e-9, "the majority unit is applied");
    }

    #[test]
    fn minority_unit_records_are_quarantined() {
        // round-14: a record whose raw i0 voted for the LOSING unit is
        // rejected (counted in the ledger) — never scaled by the winner's
        // factor. Semicircles file + a rad-band record:
        let txt = format!(
            "{RNX_HDR}{}\n{}\n{}\n{}",
            rinex_record(5, 2350.0, 345_600.0, "3.000000000000D-01"),
            rinex_record(6, 2350.0, 345_600.0, "3.100000000000D-01"),
            rinex_record(7, 2350.0, 345_600.0, "3.200000000000D-01"),
            rinex_record(8, 2350.0, 345_600.0, "9.600000000000D-01"),
        );
        let r = parse_rinex_gps_nav(&txt);
        assert_eq!(r.unit, AngUnit::Semicircles);
        assert_eq!(r.rejected, 1, "the rad-band record lost the vote");
        assert!(!r.ephs.contains_key(&8) && r.ephs.len() == 3);
        // radians file + an sc-band record (the pre-round-14 leak: 0.30 sc
        // passed the [0, PI] sanity and stayed UNSCALED — a garbage orbit):
        let txt = format!(
            "{RNX_HDR}{}\n{}\n{}\n{}",
            rinex_record(5, 2350.0, 345_600.0, "9.500000000000D-01"),
            rinex_record(6, 2350.0, 345_600.0, "9.600000000000D-01"),
            rinex_record(7, 2350.0, 345_600.0, "9.700000000000D-01"),
            rinex_record(8, 2350.0, 345_600.0, "3.000000000000D-01"),
        );
        let r = parse_rinex_gps_nav(&txt);
        assert_eq!(r.unit, AngUnit::Radians);
        assert_eq!(r.rejected, 1, "the sc-band record lost the vote");
        assert!(!r.ephs.contains_key(&8) && r.ephs.len() == 3);
        assert!((r.ephs[&5].m0 - 0.3).abs() < 1e-12, "radians pass through unscaled");
        // unit-neutral records (i0 < 0.25: no evidence either way) still
        // pass under either winner
        let txt = format!(
            "{RNX_HDR}{}\n{}",
            rinex_record(5, 2350.0, 345_600.0, "9.500000000000D-01"),
            rinex_record(9, 2350.0, 345_600.0, "1.000000000000D-01"),
        );
        let r = parse_rinex_gps_nav(&txt);
        assert_eq!(r.unit, AngUnit::Radians);
        assert_eq!(r.rejected, 0);
        assert!(r.ephs.contains_key(&9));
        let txt = format!(
            "{RNX_HDR}{}\n{}",
            rinex_record(5, 2350.0, 345_600.0, "3.000000000000D-01"),
            rinex_record(9, 2350.0, 345_600.0, "1.000000000000D-01"),
        );
        let r = parse_rinex_gps_nav(&txt);
        assert_eq!(r.unit, AngUnit::Semicircles);
        assert_eq!(r.rejected, 0);
        assert!(r.ephs.contains_key(&9));
    }

    #[test]
    fn unit_vote_near_even_split_still_fails_closed() {
        // 3 sc-like vs 2 rad-like: 3/5 < 2/3 — genuinely contested content
        // still fails closed; the supermajority law is not a coin flip.
        let mut txt = String::from(RNX_HDR);
        for (k, i0) in ["3.000000000000D-01", "3.100000000000D-01", "3.200000000000D-01",
            "9.500000000000D-01", "9.600000000000D-01"]
            .iter()
            .enumerate()
        {
            txt.push_str(&rinex_record(k as u8 + 5, 2350.0, 345_600.0, i0));
            txt.push('\n');
        }
        let r = parse_rinex_gps_nav(&txt);
        assert_eq!(r.unit, AngUnit::Ambiguous, "a 3:2 split has no supermajority");
        assert_eq!(r.rejected, 5);
        assert!(r.ephs.is_empty());
    }

    #[test]
    fn unhealthy_records_never_win_selection() {
        let healthy_old =
            rinex_record_hf(5, 2350.0, 345_600.0, "3.000000000000D-01", "0.000000000000D+00", "4.000000000000D+00");
        let unhealthy_new =
            rinex_record_hf(5, 2350.0, 349_200.0, "3.000000000000D-01", "1.000000000000D+00", "4.000000000000D+00");
        // the unhealthy NEWER record loses to the healthy older one
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{healthy_old}\n{unhealthy_new}"));
        assert_eq!(r.rejected, 1, "the unhealthy record is hard-excluded");
        assert_eq!(r.ephs[&5].toe, 345_600.0, "the healthy older record wins");
        // ...and selects nothing even when it is the ONLY record on file
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{unhealthy_new}"));
        assert_eq!(r.rejected, 1);
        assert!(r.ephs.get(&5).is_none(), "an unhealthy-only PRN selects nothing");
        // health unknown (blank field -> None) stays eligible
        let blank_health =
            rinex_record_hf(7, 2350.0, 345_600.0, "3.000000000000D-01", "", "4.000000000000D+00");
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{blank_health}"));
        assert_eq!(r.rejected, 0);
        assert_eq!(r.ephs[&7].health, None);
    }

    #[test]
    fn fit_window_preference_picks_the_in_fit_record() {
        // Reference epoch = the constellation's newest (week, toe): PRN 6's
        // record at toe 363601 puts PRN 5's NEWER record (toe 349200, fit
        // 4 h) 1 s PAST its fit window while the OLDER record (toe 345600,
        // fit 6 h) still covers it — the in-fit record beats the newer
        // out-of-fit one, in either file order.
        let old_long =
            rinex_record_hf(5, 2350.0, 345_600.0, "3.000000000000D-01", "0.000000000000D+00", "6.000000000000D+00");
        let new_short =
            rinex_record_hf(5, 2350.0, 349_200.0, "3.000000000000D-01", "0.000000000000D+00", "4.000000000000D+00");
        let clock_ref = rinex_record(6, 2350.0, 363_601.0, "3.100000000000D-01");
        for order in [&format!("{new_short}\n{old_long}"), &format!("{old_long}\n{new_short}")] {
            let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{order}\n{clock_ref}"));
            assert_eq!(r.rejected, 0);
            assert_eq!(r.ephs[&5].toe, 345_600.0, "in-fit beats newer out-of-fit");
        }
        // an out-of-fit record is still usable when it is the only candidate
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{new_short}\n{clock_ref}"));
        assert_eq!(r.ephs[&5].toe, 349_200.0, "out-of-fit sole candidate stays selected");
    }
}
