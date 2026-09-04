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

/// WGS-84 gravitational parameter, m^3/s^2 (IS-GPS-200 Table 20-IV).
/// Public since 2026-09-04 so the relativity-off experiment
/// (examples/clock_bias.rs) can name the GPS pair explicitly.
pub const MU_E: f64 = 3.986005e14;
const OMEGA_E: f64 = 7.2921151467e-5; // Earth rotation rate, rad/s (ICD)
/// GPS relativistic clock constant F = -2 sqrt(mu)/c^2, s/sqrt(m)
/// (IS-GPS-200 20.3.3.3.3.1). Public for the same reason as [`MU_E`].
pub const F_REL: f64 = -4.442807633e-10;
/// Galileo gravitational parameter (OS SIS ICD 2.1 Table 66) — equals the
/// BDS value, NOT the GPS MU_E; omega_E is the GPS value (Table 66).
pub const MU_GAL: f64 = 3.986004418e14;
/// Galileo relativistic clock constant F = -2 sqrt(mu)/c^2 (ICD Eq. 15;
/// GPS: -4.442807633e-10).
pub const F_REL_GAL: f64 = -4.442807309e-10;
const WEEK_S: f64 = 604800.0;
const PI: f64 = std::f64::consts::PI;

/// One satellite's broadcast ephemeris (SI units, angles in radians).
/// `sys`: 0 = GPS, 1 = BeiDou (BDS ephemeris times are stored as
/// GPST-equivalent seconds-of-week — BDT SOW + 14 s — so one t_tx timescale
/// serves every constellation; see beidou_d1.rs), 2 = Galileo (GST is
/// GPST-aligned — GST TOW == GPST TOW, GST WN 0 == GPS week 1024 — so
/// toe/toc are stored unshifted and `week` is the continuous GPS-aligned
/// week the RINEX E record carries; see parse_rinex_gal).
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
    sat_pos_ecef_impl(e, t, MU_E)
}

/// The shared Keplerian pipeline, mu-parameterized (spec 2026-09-02 §5.1:
/// the Galileo user algorithm is the IS-GPS-200 Table 20-IV form verbatim —
/// ICD Table 66 — so the constants are parameters, not a fork).
fn sat_pos_ecef_impl(e: &BrdcEph, t: f64, mu: f64) -> [f64; 3] {
    let a = e.sqrt_a * e.sqrt_a;
    let n0 = (mu / (a * a * a)).sqrt();
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
    sat_clock_impl(e, t, MU_E, F_REL)
}

/// Shared clock polynomial + relativity - group delay, constant-parameterized
/// (the `- e.tgd` slot is TGD for GPS and BGD(E1,E5b) for Galileo — the ICD
/// Eq. 17 single-frequency E1 correction has the same sign and shape).
fn sat_clock_impl(e: &BrdcEph, t: f64, mu: f64, f_rel: f64) -> f64 {
    let dt = wrap_tk(t - e.toc);
    let poly = e.af0 + e.af1 * dt + e.af2 * dt * dt;
    // 2026-09-04: the periodic relativistic term is factored out (see
    // relativistic_periodic_s) with the expression order preserved —
    // `poly + (((f_rel * e) * sqrt_a) * sin(Ek)) - tgd` before and after,
    // so every GPS/BDS/GAL clock stays bit-identical.
    poly + relativistic_periodic_s(e, t, mu, f_rel) - e.tgd
}

/// The periodic relativistic clock term dt_r = F·e·√A·sin(Ek) (seconds,
/// IS-GPS-200 20.3.3.3.3.1 / Galileo ICD Eq. 15) evaluated with EXACTLY
/// the eccentric anomaly the clock uses — `sat_clock_impl` calls this, so
/// the value returned here is, bit for bit, what the constellation's
/// dt_sv contains at the same `t`. Constant-parameterized like the clock
/// (GPS: [`MU_E`]/[`F_REL`]; Galileo: [`MU_GAL`]/[`F_REL_GAL`]).
///
/// Purpose (2026-09-04, relativity-off experiment): the clock_bias emitter
/// removes this term from one satellite's modeled clock so the station's
/// own residual log can measure general relativity's periodic term
/// (amplitude F·e·√A·c ≈ 14 m for e ≈ 0.02) instead of assuming it.
pub fn relativistic_periodic_s(e: &BrdcEph, t: f64, mu: f64, f_rel: f64) -> f64 {
    let a = e.sqrt_a * e.sqrt_a;
    let n0 = (mu / (a * a * a)).sqrt();
    let tk = wrap_tk(t - e.toe);
    let mk = e.m0 + (n0 + e.delta_n) * tk;
    let ek = kepler_e(mk, e.e);
    f_rel * e.e * e.sqrt_a * ek.sin()
}

/// [`relativistic_periodic_s`] with the GPS constants — the term
/// [`sat_clock`] contains at `t`.
pub fn relativistic_periodic_gps_s(e: &BrdcEph, t: f64) -> f64 {
    relativistic_periodic_s(e, t, MU_E, F_REL)
}

// ------------------------------------------------------------- Galileo I/NAV
//
// Ephemeris evaluation + RINEX-3 `E` record parser for the E1-B ranging
// lever (spec docs/superpowers/specs/2026-09-02-galileo-inav-ranging-spec.md
// §5). Lives beside the GPS parser (round-11 helper/parity reuse: fld /
// df_strict / detect_ang_unit / i0_sane / RinexParse are shared verbatim)
// rather than in the live-decoder module — the solve emitter needs exactly
// this and nothing of the symbol-domain machinery. Every numeric behavior
// below is pinned against the python reference (scripts/inav_reference.py)
// via tests/fixtures/inav/rinex_gal_eval.json + ggto_bgd_vectors.json.

/// RINEX 3.04 Galileo Data Sources bits (gLAB reference; verified on live
/// brdc_latest.rnx 2026-09-02: 258 = F/NAV (bit1|bit8), 513/516/517 = I/NAV).
pub const DS_INAV_E1B: u16 = 1 << 0; // I/NAV E1-B
pub const DS_INAV_E5B: u16 = 1 << 2; // I/NAV E5b-I (same message)
pub const DS_CLOCK_E5B_E1: u16 = 1 << 9; // af0-2 are the (E5b,E1) pair -> I/NAV

/// SISA acceptance band (metres). RINEX writes the SISA(E1,E5b) index as
/// metres; ICD Table 76 maps index 0..=125 to 0..6 m and 255 = NAPA ("no
/// accuracy prediction available"), which RINEX conventionally writes as
/// -1. The URA-equivalent gate is fail-closed: anything outside [0, 6] m
/// (NAPA, spare indices, garbage) rejects the record at selection.
pub const SISA_MAX_M: f64 = 6.0;

/// Satellite ECEF position (metres) at GST time-of-week `t` — the Keplerian
/// pipeline with Galileo constants (ICD Table 66: mu == the BDS value,
/// omega_E == the GPS value).
pub fn sat_pos_ecef_gal(e: &BrdcEph, t: f64) -> [f64; 3] {
    sat_pos_ecef_impl(e, t, MU_GAL)
}

/// Galileo dt_sv (seconds) for the single-frequency E1 user: the (E1,E5b)
/// broadcast clock (ICD Eq. 13-14) + relativity, MINUS BGD(E1,E5b) per ICD
/// §5.1.5 Eq. 17 (f1 = E1) — the exact analogue of the GPS `- e.tgd` term.
/// parse_rinex_gal stores BGD(E1,E5b) in `e.tgd`, so the shared impl's
/// subtraction IS the Eq. 17 correction.
pub fn sat_clock_gal(e: &BrdcEph, t: f64) -> f64 {
    sat_clock_impl(e, t, MU_GAL, F_REL_GAL)
}

/// Satellite ECEF (metres) at transmit time, Sagnac-rotated into the
/// reception frame, plus clock correction (s) and geometric range (m) —
/// the Galileo twin of `beidou_d1::sat_at_txtime_bds` (tau = 0.075 start,
/// two iterations, clock at t_tx - tau) with the GPS omega_E (Table 66).
/// `t_tx` is the GST time of week, GPST-equivalent up to the GGTO the
/// emitter applies upstream (spec §5.4 Option B).
pub fn sat_at_txtime_gal(e: &BrdcEph, t_tx: f64, rx_m: [f64; 3]) -> ([f64; 3], f64, f64) {
    let mut tau = 0.075;
    let mut s = [0.0f64; 3];
    for _ in 0..2 {
        let s0 = sat_pos_ecef_gal(e, t_tx - tau);
        let th = OMEGA_E * tau;
        let (ct, st) = (th.cos(), th.sin());
        s = [s0[0] * ct + s0[1] * st, -s0[0] * st + s0[1] * ct, s0[2]];
        let d = [s[0] - rx_m[0], s[1] - rx_m[1], s[2] - rx_m[2]];
        tau = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() / C_LIGHT;
    }
    let dt = sat_clock_gal(e, t_tx - tau);
    let d = [s[0] - rx_m[0], s[1] - rx_m[1], s[2] - rx_m[2]];
    let rng = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    (s, dt, rng)
}

/// Parse the Galileo records of a RINEX-3 MIXED navigation file, keeping the
/// newest VALID I/NAV issue per PRN. F/NAV-clocked twins are skipped BY
/// DESIGN (not counted as rejections — every SV legitimately appears twice;
/// see [`parse_rinex_gal_nav`]); malformed/unhealthy/NAPA records are
/// rejected and logged.
pub fn parse_rinex_gal(text: &str) -> HashMap<u8, BrdcEph> {
    let r = parse_rinex_gal_nav(text);
    if r.rejected > 0 {
        eprintln!(
            "parse_rinex_gal: {} GAL record(s) rejected (unit {:?})",
            r.rejected, r.unit
        );
    }
    r.ephs
}

/// Full Galileo-record parse with the rejection ledger.
///
/// CRITICAL selection gate (spec §5.2): GAL records appear TWICE per SV —
/// I/NAV and F/NAV issues with DIFFERENT clock parameters (F/NAV af0-2 are
/// the (E1,E5a) pair, I/NAV the (E5b,E1) pair, ICD Table 69) and different
/// BGD slots. Only records whose Data Sources field has bit 9 (E5b,E1
/// clock pair) AND bit 0|2 (I/NAV broadcast) are selected; an F/NAV record
/// (live file: 258) is skipped silently — it is the expected twin stream,
/// not a defect — while a malformed or out-of-range Data Sources field
/// rejects. Health (RINEX Galileo SV-health bitfield: bit 0 = E1-B DVS,
/// bits 1-2 = E1-B HS) is hard-excluded on nonzero E1-B bits, and the
/// SISA gate ([`SISA_MAX_M`]) rejects NAPA/-1 and out-of-band values —
/// both fail-closed at selection, mirroring the GPS/BDS health law.
/// Angle units ride the same per-constellation vote as GPS/BDS (RINEX-3
/// mandates radians; live E records conform).
///
/// Field mapping into [`BrdcEph`]: toe/toc are GST SOW stored UNSHIFTED
/// (GST is GPST-aligned); `week` is the record's continuous GPS-aligned
/// GAL week; `tgd` carries BGD(E1,E5b) (orbit-6 field 4) so
/// [`sat_clock_gal`] applies Eq. 17 through the shared `- tgd` slot;
/// `iodc` carries the 10-bit IODnav (it tags the whole batch, clock
/// included — the 8-bit `iode` slot cannot hold it and stays None);
/// `health` stores the E1-B bits (always 0 for an accepted record).
pub fn parse_rinex_gal_nav(text: &str) -> RinexParse {
    let lines: Vec<&str> = text.lines().collect();
    let mut hdr = 0usize;
    while hdr < lines.len() && !lines[hdr].contains("END OF HEADER") {
        hdr += 1;
    }
    hdr += 1;
    let unit = detect_ang_unit(&lines, hdr, 'E');
    let mut out: HashMap<u8, BrdcEph> = HashMap::new();
    let mut rejected = 0usize;
    let mut i = hdr;
    while i < lines.len() {
        let ln = lines[i];
        if ln.is_empty() || !ln.starts_with('E') || i + 7 >= lines.len() {
            i += 1;
            continue;
        }
        let b: Vec<&str> = (0..7).map(|k| lines[i + 1 + k]).collect();
        // Data Sources (orbit line 6 = b[4], field 2) decides I/NAV vs
        // F/NAV BEFORE the strict parse: the F/NAV twin is a different
        // message stream skipped by design, not a malformed record.
        match df_strict(fld(b[4], 23, 42)) {
            Some(ds) if (0.0..=1023.0).contains(&ds) && ds.fract() == 0.0 => {
                let dsi = ds as u16;
                if dsi & DS_CLOCK_E5B_E1 != 0 && dsi & (DS_INAV_E1B | DS_INAV_E5B) != 0 {
                    match parse_gal_record(ln, &b, unit) {
                        Some(e) => {
                            out.entry(e.prn)
                                .and_modify(|cur| {
                                    // RINEX GAL weeks are continuous
                                    // (GPS-aligned), so (week, toe) is
                                    // rollover-exact, as GPS/BDS.
                                    if (e.week, e.toe) > (cur.week, cur.toe) {
                                        *cur = e.clone();
                                    }
                                })
                                .or_insert(e);
                        }
                        None => rejected += 1,
                    }
                }
                // else: F/NAV-clocked twin — expected, silently skipped
            }
            _ => rejected += 1, // malformed/absent Data Sources: fail closed
        }
        i += 8;
    }
    RinexParse { ephs: out, rejected, unit }
}

/// One 8-line Galileo nav record -> BrdcEph, STRICT (same round-11 law as
/// the GPS/BDS parsers: any malformed consumed field rejects; the caller
/// has already applied the I/NAV Data Sources gate). RINEX 3.04 E-record
/// layout (spec §5.2): L1 epoch (toc, GAL time == GPST frame) + af0-2;
/// orbit 1 IODnav, Crs, Delta n, M0; orbit 2 Cuc, e, Cus, sqrtA; orbit 3
/// toe, Cic, OMEGA0, Cis; orbit 4 i0, Crc, omega, OMEGAdot; orbit 5 IDOT,
/// Data Sources, GAL week (continuous, GPS-aligned); orbit 6 SISA, SV
/// health, BGD(E1,E5a), BGD(E1,E5b); orbit 7 transmission time.
fn parse_gal_record(ln: &str, b: &[&str], unit: AngUnit) -> Option<BrdcEph> {
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
    let f = |l: usize, j: usize| df_strict(fld(b[l], 4 + j * 19, 4 + (j + 1) * 19));
    let i0_raw = f(3, 0)?;
    if !i0_sane(i0_raw, unit) {
        return None;
    }
    // IODnav: 10-bit (ICD Table 40) — out of range rejects
    let iodnav = f(0, 0)?;
    if !(0.0..=1023.0).contains(&iodnav) || iodnav.fract() != 0.0 {
        return None;
    }
    // SISA(E1,E5b): the URA-equivalent gate, FAIL-CLOSED — NAPA (-1),
    // spare-index and garbage values never range (spec work order).
    let sisa = f(5, 0)?;
    if !(0.0..=SISA_MAX_M).contains(&sisa) {
        return None;
    }
    // SV health bitfield (orbit 6 field 2): bit 0 = E1-B DVS, bits 1-2 =
    // E1-B HS (RINEX 3.04 Table A8). Nonzero E1-B bits are hard-excluded
    // (two-sided health law, as GPS health / BDS SatH1). Bits for E5a/E5b
    // do not gate the E1 user. Blank is NOT tolerated here: the same
    // orbit-6 line already parsed strictly for SISA/BGD, and an E record
    // without health cannot prove E1-B validity — fail closed.
    let health_raw = f(5, 1)?;
    if !(0.0..=511.0).contains(&health_raw) || health_raw.fract() != 0.0 {
        return None;
    }
    let e1b_health = (health_raw as u16 & 0x7) as u8;
    if e1b_health != 0 {
        return None;
    }
    Some(BrdcEph {
        sys: 2,
        prn,
        // IODnav tags the whole batch (ephemeris AND clock, ICD §5.1.9.2):
        // the 16-bit iodc slot holds the 10-bit value; the 8-bit iode slot
        // cannot and stays None. fit_h: E records carry no fit interval.
        iode: None,
        iodc: Some(iodnav as u16),
        fit_h: None,
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
        toe: f(2, 0)?, // GST SOW, GPST-equivalent unshifted
        cic: f(2, 1)?,
        omega0: f(2, 2)? * ang,
        cis: f(2, 3)?,
        i0: i0_raw * ang,
        crc: f(3, 1)?,
        omega: f(3, 2)? * ang,
        omega_dot: f(3, 3)? * ang,
        idot: f(4, 0)? * ang,
        week: f(4, 2)?, // continuous GPS-aligned GAL week (RINEX)
        health: Some(e1b_health), // always 0 here (gated above)
        tgd: f(5, 3)?, // BGD(E1,E5b): the Eq. 17 E1 correction slot
        toc: gps_sow(y, mo, d, h, mi, s), // GAL epochs are GPST-frame
        rx_epoch: None,
    })
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

    // ------------------------------------------------- Galileo (window-run)
    //
    // Every number below is PINNED from tests/fixtures/inav/
    // rinex_gal_eval.json (generated and proven by the running python
    // reference scripts/inav_reference.py + its 61-test suite, 2026-09-02).
    // The RINEX lines are the live brdc_latest.rnx E02 record verbatim.

    const GAL_E02: &str = "\
E02 2026 09 01 20 00 00 6.600067717955e-05 2.685851541173e-12 0.000000000000e+00
     2.700000000000e+01 1.567812500000e+02 3.000124967247e-09 2.036137310846e+00
     7.089227437973e-06 3.099278546870e-04 1.248344779015e-05 5.440630514145e+03
     2.448000000000e+05-3.911554813385e-08-2.069042130333e+00-2.793967723846e-08
     9.611374355521e-01 7.159375000000e+01 4.523971024409e-02-5.452012812503e-09
    -2.857261873569e-11 5.160000000000e+02 2.434000000000e+03 0.000000000000e+00
     3.120000000000e+00 0.000000000000e+00-3.026798367500e-09-3.958120942116e-09
     2.454640000000e+05                                                         ";

    /// Fixture receiver ECEF (m) the python evaluations used.
    const GAL_RX_M: [f64; 3] = [4_278_600.0, 636_800.0, 4_672_300.0];

    #[test]
    fn parse_rinex_gal_pins_the_fixture_record() {
        let r = parse_rinex_gal_nav(&format!("{RNX_HDR}{GAL_E02}"));
        assert_eq!(r.rejected, 0, "live E02 record must parse clean");
        assert_eq!(r.unit, AngUnit::Radians, "live E records are radians");
        let e = &r.ephs[&2];
        assert_eq!(e.sys, 2);
        assert_eq!(e.prn, 2);
        // every value below is the python reference's parse, JSON-pinned
        assert_eq!(e.toc, 244_800.0, "GAL epochs are GPST-frame SOW");
        assert_eq!(e.toe, 244_800.0);
        assert_eq!(e.week, 2434.0, "continuous GPS-aligned GAL week");
        assert_eq!(e.af0, 6.600067717955e-05);
        assert_eq!(e.af1, 2.685851541173e-12);
        assert_eq!(e.af2, 0.0);
        assert_eq!(e.iodc, Some(27), "IODnav rides the 16-bit iodc slot");
        assert_eq!(e.iode, None);
        assert_eq!(e.crs, 156.78125);
        assert_eq!(e.delta_n, 3.000124967247e-09);
        assert_eq!(e.m0, 2.036137310846);
        assert_eq!(e.cuc, 7.089227437973e-06);
        assert_eq!(e.e, 3.099278546870e-04);
        assert_eq!(e.cus, 1.248344779015e-05);
        assert_eq!(e.sqrt_a, 5440.630514145);
        assert_eq!(e.cic, -3.911554813385e-08);
        assert_eq!(e.omega0, -2.069042130333);
        assert_eq!(e.cis, -2.793967723846e-08);
        assert_eq!(e.i0, 0.9611374355521);
        assert_eq!(e.crc, 71.59375);
        assert_eq!(e.omega, 4.523971024409e-02);
        assert_eq!(e.omega_dot, -5.452012812503e-09);
        assert_eq!(e.idot, -2.857261873569e-11);
        assert_eq!(e.health, Some(0));
        assert_eq!(e.tgd, -3.958120942116e-09, "tgd slot = BGD(E1,E5b)");
        assert_eq!(e.fit_h, None);
        assert_eq!(e.rx_epoch, None);
    }

    #[test]
    fn sat_at_txtime_gal_matches_the_python_reference() {
        // (t_sow, txtime sat ECEF, txtime clock, txtime range) pinned from
        // rinex_gal_eval.json record E02 (python sat_at_txtime_gal). The
        // pipelines are op-identical f64, so agreement is sub-mm / sub-fs;
        // the tolerances leave room only for libm last-ulp differences.
        let cases: [(f64, [f64; 3], f64, f64); 3] = [
            (
                244_800.0,
                [6_028_192.559060952, 19_797_645.48543666, 21_169_201.834917065],
                6.600396567344267e-05,
                25_344_562.414655928,
            ),
            (
                245_700.0,
                [5_221_120.744260678, 21_458_410.291688666, 19_716_744.25471923],
                6.600642454210922e-05,
                25_705_312.747089297,
            ),
            (
                246_600.0,
                [4_584_601.6703967005, 23_039_374.239954162, 18_019_196.92649376],
                6.600889121425358e-05,
                26_078_892.36845369,
            ),
        ];
        let ephs = parse_rinex_gal(&format!("{RNX_HDR}{GAL_E02}"));
        let e = &ephs[&2];
        for (t, pos, clk, rng) in cases {
            let (s, dt, r) = sat_at_txtime_gal(e, t, GAL_RX_M);
            for k in 0..3 {
                assert!(
                    (s[k] - pos[k]).abs() < 1e-3,
                    "t {t} axis {k}: {} vs pinned {}",
                    s[k],
                    pos[k]
                );
            }
            assert!((dt - clk).abs() < 1e-15, "t {t} clock {dt} vs {clk}");
            assert!((r - rng).abs() < 1e-3, "t {t} range {r} vs {rng}");
        }
    }

    #[test]
    fn gal_clock_subtracts_bgd_e1e5b() {
        // ggto_bgd_vectors.json bgd_case: at toe, clock_e1 = clock_e1e5b -
        // BGD(E1,E5b) (ICD Eq. 17, f1 = E1). Both sides pinned.
        let ephs = parse_rinex_gal(&format!("{RNX_HDR}{GAL_E02}"));
        let e = &ephs[&2];
        let dt = sat_clock_gal(e, 244_800.0);
        assert!((dt - 6.600396590403022e-05).abs() < 1e-15, "clock_e1 {dt}");
        // and removing the BGD recovers the broadcast (E1,E5b) clock
        let mut no_bgd = e.clone();
        no_bgd.tgd = 0.0;
        let dt_pair = sat_clock_gal(&no_bgd, 244_800.0);
        assert!((dt_pair - 6.60000077830881e-05).abs() < 1e-15, "clock_e1e5b {dt_pair}");
        assert!(dt > dt_pair, "negative BGD must ADD when subtracted");
    }

    #[test]
    fn gal_constants_differ_from_gps_where_the_icd_says() {
        assert_eq!(MU_GAL, 3.986004418e14);
        assert_ne!(MU_GAL, MU_E, "GAL mu is the BDS value, not GPS");
        assert_eq!(F_REL_GAL, -4.442807309e-10);
        assert_ne!(F_REL_GAL, F_REL);
        // omega_E is shared with GPS (ICD Table 66) — sat_at_txtime_gal
        // deliberately reuses OMEGA_E, unlike BDS's 7.2921150e-5.
        assert_eq!(OMEGA_E, 7.2921151467e-5);
    }

    // -------------------------------------- relativity-off (window-run)
    //
    // 2026-09-04 experiment support: relativistic_periodic_s must be the
    // exact term the clock contains, on GPS and Galileo constants alike,
    // and the live G07 record is pinned against the python cross-check
    // tests/fixtures/relativity/gps_rel_eval.json (scripts/
    // relativity_reference.py, hand-parsed RINEX, 12-step Newton Kepler).

    /// A synthetic eccentric GPS orbit: e = 0.02 makes the term ~1e-8 s,
    /// far above the 1e-15 tolerance, and the toc != toe / af1 / af2 /
    /// tgd fields make sure the identity is not trivially 0 == 0.
    fn eccentric_eph() -> BrdcEph {
        BrdcEph {
            sqrt_a: 5153.7,
            e: 0.02,
            m0: 0.3,
            delta_n: 4.5e-9,
            toe: 453_600.0,
            toc: 453_600.0 + 600.0,
            af0: -2.2e-4,
            af1: -3.3e-12,
            af2: 1.0e-20,
            tgd: -1.07e-8,
            ..Default::default()
        }
    }

    #[test]
    fn relativistic_term_is_exactly_the_clocks_periodic_part() {
        let e = eccentric_eph();
        for &t in &[453_600.0, 453_600.0 + 900.0, 453_600.0 + 2700.0, 453_600.0 + 21_000.0, 3_000.0] {
            let dt = wrap_tk(t - e.toc);
            let poly = e.af0 + e.af1 * dt + e.af2 * dt * dt;
            let rel = relativistic_periodic_gps_s(&e, t);
            assert!(rel.abs() > 1e-10, "t {t}: term {rel} should be ~1e-8 s on e=0.02");
            assert!(
                (sat_clock(&e, t) - (poly - e.tgd) - rel).abs() < 1e-15,
                "t {t}: sat_clock - (poly - tgd) != rel: {} vs {rel}",
                sat_clock(&e, t) - (poly - e.tgd)
            );
            // the factoring preserved the op order: the clock is bit-identical
            // to the pre-factoring expression
            let a = e.sqrt_a * e.sqrt_a;
            let n0 = (MU_E / (a * a * a)).sqrt();
            let tk = wrap_tk(t - e.toe);
            let ek = kepler_e(e.m0 + (n0 + e.delta_n) * tk, e.e);
            let legacy = poly + F_REL * e.e * e.sqrt_a * ek.sin() - e.tgd;
            assert_eq!(legacy.to_bits(), sat_clock(&e, t).to_bits(), "t {t}: op order changed");
            assert_eq!(rel, relativistic_periodic_s(&e, t, MU_E, F_REL));
        }
        // circular orbit: the term vanishes identically
        let c = circular_eph();
        assert_eq!(relativistic_periodic_gps_s(&c, 1234.5), 0.0);
    }

    #[test]
    fn relativistic_term_matches_the_galileo_clock_too() {
        // the same factoring serves sat_clock_gal (MU_GAL / F_REL_GAL):
        // on the pinned E02 record the identity holds and the pinned
        // clock_e1 value is untouched by the refactor
        let ephs = parse_rinex_gal(&format!("{RNX_HDR}{GAL_E02}"));
        let e = &ephs[&2];
        let t = 244_800.0;
        let rel = relativistic_periodic_s(e, t, MU_GAL, F_REL_GAL);
        let poly = e.af0; // dt == 0 at toc
        assert!((sat_clock_gal(e, t) - (poly - e.tgd) - rel).abs() < 1e-15);
        assert!((sat_clock_gal(e, t) - 6.600396590403022e-05).abs() < 1e-15);
        // GPS constants on a GAL record give a DIFFERENT term (the
        // function is honest about its parameters, never a hidden default)
        assert_ne!(rel, relativistic_periodic_gps_s(e, t));
    }

    /// Live brdc_latest.rnx G07 record (2026-09-04 06:00, toe 453600,
    /// e = 0.0209, √A = 5153.76) verbatim — the python cross-check's input.
    const GPS_G07: &str = "\
G07 2026 09 04 06 00 00-2.215462736785e-04-3.296918293927e-12 0.000000000000e+00
     7.700000000000e+01-1.853125000000e+01 4.966278293997e-09-1.184373590828e-01
    -8.661299943924e-07 2.093016507570e-02 7.657334208488e-06 5.153760953903e+03
     4.536000000000e+05-1.993030309677e-07 2.943443499676e+00 3.203749656677e-07
     9.516107696539e-01 2.325937500000e+02-1.950459143607e+00-8.424993791951e-09
     2.207234797332e-10 1.000000000000e+00 2.434000000000e+03 0.000000000000e+00
     2.000000000000e+00 0.000000000000e+00-1.071020960808e-08 7.700000000000e+01
     4.464180000000e+05 4.000000000000e+00                                      ";

    #[test]
    fn relativistic_term_pins_the_python_cross_check_on_live_g07() {
        // (t_sow, Ek rad, rel_s, sat_clock s) from tests/fixtures/
        // relativity/gps_rel_eval.json. The pipelines are op-identical
        // f64 (Newton seeded at Mk, 12 steps), so agreement is libm-ulp
        // class: rel_s to 1e-18 (3e-10 m), Ek to 1e-12, clock to 1e-15.
        let cases: [(f64, f64, f64, f64); 3] = [
            (453_600.0, -0.12096296423660788, 5.7829206782970885e-09, -0.00022152978054821364),
            (454_500.0, 0.013103234446386914, -6.279434253455477e-10, -0.0002215391586387818),
            (456_300.0, 0.2811693738649913, -1.3297959694025663e-08, -0.00022155776310797955),
        ];
        let r = parse_rinex_gps_nav(&format!("{RNX_HDR}{GPS_G07}"));
        assert_eq!(r.rejected, 0, "live G07 record must parse clean");
        assert_eq!(r.unit, AngUnit::Radians, "live G records are radians");
        let e = &r.ephs[&7];
        assert_eq!(e.toe, 453_600.0);
        assert_eq!(e.toc, 453_600.0);
        assert_eq!(e.e, 2.093016507570e-02);
        assert_eq!(e.sqrt_a, 5153.760953903);
        assert_eq!(e.tgd, -1.071020960808e-08);
        // amplitude F·e·√A: -4.79e-8 s = -14.37 m
        let amp = F_REL * e.e * e.sqrt_a;
        assert!((amp - -4.7924151656860264e-08).abs() < 1e-20, "amplitude {amp}");
        for (t, ek, rel, clk) in cases {
            let a = e.sqrt_a * e.sqrt_a;
            let n0 = (MU_E / (a * a * a)).sqrt();
            let ek_rs = kepler_e(e.m0 + (n0 + e.delta_n) * wrap_tk(t - e.toe), e.e);
            assert!((ek_rs - ek).abs() < 1e-12, "t {t}: Ek {ek_rs} vs pinned {ek}");
            let rel_rs = relativistic_periodic_gps_s(e, t);
            assert!((rel_rs - rel).abs() < 1e-18, "t {t}: rel {rel_rs} vs pinned {rel}");
            let clk_rs = sat_clock(e, t);
            assert!((clk_rs - clk).abs() < 1e-15, "t {t}: clock {clk_rs} vs pinned {clk}");
            // and the experiment's "clock without the term" is the pinned
            // clock_minus_rel_s: poly - tgd
            let dt = wrap_tk(t - e.toc);
            let poly = e.af0 + e.af1 * dt + e.af2 * dt * dt;
            assert!((clk_rs - rel_rs - (poly - e.tgd)).abs() < 1e-15);
        }
    }

    #[test]
    fn fnav_twin_records_are_skipped_not_rejected() {
        // Data Sources 258 (bit1|bit8) is the F/NAV twin carrying the
        // (E1,E5a) clock pair — wrong for the E1 user equation. It must
        // neither enter the map nor count as a rejection (every SV appears
        // twice by design; live file verified 2026-09-02).
        let fnav = GAL_E02.replace("5.160000000000e+02", "2.580000000000e+02");
        let r = parse_rinex_gal_nav(&format!("{RNX_HDR}{fnav}"));
        assert!(r.ephs.is_empty(), "F/NAV clock must never range E1");
        assert_eq!(r.rejected, 0, "the twin stream is expected, not a defect");
        // I/NAV twin alongside: only the I/NAV record selects, even when
        // the F/NAV twin is listed first
        let both = format!("{RNX_HDR}{fnav}\n{GAL_E02}");
        let r = parse_rinex_gal_nav(&both);
        assert_eq!(r.ephs.len(), 1);
        assert_eq!(r.ephs[&2].tgd, -3.958120942116e-09, "I/NAV BGD slot");
        // 513 (bit0|bit9) and 517 (bit0|bit2|bit9) also pass the gate
        for ds in ["5.130000000000e+02", "5.170000000000e+02"] {
            let rec = GAL_E02.replace("5.160000000000e+02", ds);
            let r = parse_rinex_gal_nav(&format!("{RNX_HDR}{rec}"));
            assert_eq!(r.ephs.len(), 1, "ds {ds} is I/NAV");
        }
        // a malformed Data Sources field fails closed as a rejection
        let bad = GAL_E02.replace("5.160000000000e+02", &" ".repeat(19));
        let r = parse_rinex_gal_nav(&format!("{RNX_HDR}{bad}"));
        assert!(r.ephs.is_empty());
        assert_eq!(r.rejected, 1);
    }

    #[test]
    fn gal_health_and_sisa_gates_fail_closed() {
        // E1-B DVS bit set (health 1): hard-excluded at selection
        let sick = GAL_E02
            .replace("3.120000000000e+00 0.000000000000e+00", "3.120000000000e+00 1.000000000000e+00");
        let r = parse_rinex_gal_nav(&format!("{RNX_HDR}{sick}"));
        assert!(r.ephs.is_empty(), "nonzero E1-B health bits must never range");
        assert_eq!(r.rejected, 1);
        // E1-B HS = 2 (Extended Operations Mode, bits 1-2) likewise
        let eom = GAL_E02
            .replace("3.120000000000e+00 0.000000000000e+00", "3.120000000000e+00 4.000000000000e+00");
        let r = parse_rinex_gal_nav(&format!("{RNX_HDR}{eom}"));
        assert!(r.ephs.is_empty());
        // E5a-only health bits (bit 3 = 8) do NOT gate the E1 user
        let e5a = GAL_E02
            .replace("3.120000000000e+00 0.000000000000e+00", "3.120000000000e+00 8.000000000000e+00");
        let r = parse_rinex_gal_nav(&format!("{RNX_HDR}{e5a}"));
        assert_eq!(r.ephs.len(), 1, "E5a health must not gate E1-B");
        assert_eq!(r.ephs[&2].health, Some(0), "stored health is the E1-B bits");
        // SISA NAPA (-1): the URA-equivalent gate rejects fail-closed
        let napa = GAL_E02.replace(" 3.120000000000e+00", "-1.000000000000e+00");
        let r = parse_rinex_gal_nav(&format!("{RNX_HDR}{napa}"));
        assert!(r.ephs.is_empty(), "NAPA must never range");
        assert_eq!(r.rejected, 1);
        // SISA above the 6 m ICD band likewise
        let big = GAL_E02.replace(" 3.120000000000e+00", " 7.000000000000e+00");
        let r = parse_rinex_gal_nav(&format!("{RNX_HDR}{big}"));
        assert!(r.ephs.is_empty());
    }

    #[test]
    fn gal_newest_issue_selection_is_week_rollover_exact() {
        // same PRN across the week boundary, mirroring the GPS test: the
        // fresh issue (week+1, small toe) displaces the old one.
        let old = GAL_E02
            .replace("2.448000000000e+05-3.911554813385e-08", "6.040000000000e+05-3.911554813385e-08");
        let new = GAL_E02
            .replace("2.448000000000e+05-3.911554813385e-08", "2.000000000000e+02-3.911554813385e-08")
            .replace("2.434000000000e+03", "2.435000000000e+03");
        for order in [format!("{old}\n{new}"), format!("{new}\n{old}")] {
            let r = parse_rinex_gal_nav(&format!("{RNX_HDR}{order}"));
            assert_eq!(r.rejected, 0);
            let e = &r.ephs[&2];
            assert_eq!(e.week, 2435.0, "the next-week issue must win");
            assert_eq!(e.toe, 200.0);
        }
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
