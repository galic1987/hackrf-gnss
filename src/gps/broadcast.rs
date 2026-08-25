//! GPS broadcast ephemeris: parse a RINEX-3 navigation file and compute
//! satellite ECEF position + clock correction (IS-GPS-200 Table 20-IV). This is
//! the missing input for `pvt::solve` — it turns real broadcast nav into the
//! satellite geometry the WLS solver needs. Ported from the Python reference
//! (`validation/rinex_nav.py`, `gps_engine.py`) and cross-checked against it.
//!
//! Angular RINEX fields are semicircles (and semicircles/s); they are converted
//! to radians on parse. Positions are returned in **metres** (ICD units); the
//! snapshot/PVT layer converts to km.

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
    /// Issue of data, ephemeris — Some only for self-decoded LNAV (RINEX
    /// has no IODE field; BRDC-derived ephemerides stay None =
    /// unverifiable). WAAS LT corrections are valid only when their IOD
    /// matches this (DO-229D Table A-10 Note 3).
    #[serde(default)]
    pub iode: Option<u8>,
}

fn fld(line: &str, a: usize, b: usize) -> &str {
    let n = line.len();
    if a >= n {
        ""
    } else {
        &line[a..b.min(n)]
    }
}

/// Parse a RINEX-3 D-exponent float field (e.g. "-1.234567890123D-04").
fn df(s: &str) -> f64 {
    let t = s.trim();
    if t.is_empty() {
        return 0.0;
    }
    t.replace('D', "E").replace('d', "E").parse().unwrap_or(0.0)
}

fn iparse(s: &str) -> i64 {
    s.trim().parse().unwrap_or(0)
}

/// Days from the proleptic Gregorian calendar to a Julian Day Number.
fn jdn(y: i64, m: i64, d: i64) -> i64 {
    let a = (14 - m) / 12;
    let yy = y + 4800 - a;
    let mm = m + 12 * a - 3;
    d + (153 * mm + 2) / 5 + 365 * yy + yy / 4 - yy / 100 + yy / 400 - 32045
}

/// GPS seconds-of-week for a UTC calendar epoch (adds the 18 s GPS-UTC leap so
/// it is consistent with a time-of-week derived the same way).
fn gps_sow(y: i64, mo: i64, d: i64, h: i64, mi: i64, s: i64) -> f64 {
    let gps_epoch = jdn(1980, 1, 6);
    let days = jdn(y, mo, d) - gps_epoch;
    let secs = days * 86400 + h * 3600 + mi * 60 + s + 18;
    (secs as f64).rem_euclid(WEEK_S)
}

/// Parse the GPS records of a RINEX-3 MIXED navigation file, keeping the latest
/// ephemeris (highest `toe`) per PRN. Returns {prn: BrdcEph}.
///
/// Angle units: the RINEX spec says semicircles, but BKG's mixed files carry
/// RADIANS (i0 = 0.957, not 0.306). Detected per file from the first
/// record's i0 magnitude and applied to every angle field — the previous
/// unconditional ×PI produced near-equatorial orbits 15,000-44,000 km off
/// (found by cross-checking against SGP4/TLE positions).
pub fn parse_rinex_gps(text: &str) -> HashMap<u8, BrdcEph> {
    let lines: Vec<&str> = text.lines().collect();
    // unit detection: first GPS record's i0 (line 4, field 0)
    let mut ang = PI; // spec default: semicircles
    let mut j = 0usize;
    while j < lines.len() && !lines[j].contains("END OF HEADER") {
        j += 1;
    }
    j += 1;
    while j + 4 < lines.len() {
        let ln = lines[j];
        if ln.starts_with('G') && ln.len() > 4 {
            let i0_raw = df(fld(lines[j + 4], 4, 23));
            if i0_raw.abs() > 0.6 {
                ang = 1.0; // radians (BKG)
            }
            break;
        }
        j += 1;
    }
    let mut i = 0usize;
    while i < lines.len() && !lines[i].contains("END OF HEADER") {
        i += 1;
    }
    i += 1;
    let mut out: HashMap<u8, BrdcEph> = HashMap::new();
    while i < lines.len() {
        let ln = lines[i];
        if ln.is_empty() || !ln.starts_with('G') || i + 7 >= lines.len() {
            i += 1;
            continue;
        }
        let prn = iparse(fld(ln, 1, 3)) as u8;
        let (y, mo, d) = (iparse(fld(ln, 4, 8)), iparse(fld(ln, 9, 11)), iparse(fld(ln, 12, 14)));
        let (h, mi, s) = (iparse(fld(ln, 15, 17)), iparse(fld(ln, 18, 20)), iparse(fld(ln, 21, 23)));
        let b: Vec<&str> = (0..7).map(|k| lines[i + 1 + k]).collect();
        // orbit field j on line `l`: 3-space indent, 19-char columns
        let f = |l: usize, j: usize| df(fld(b[l], 4 + j * 19, 4 + (j + 1) * 19));
        let e = BrdcEph {
            sys: 0,
            prn,
            iode: None, // RINEX nav records carry no IODE — unverifiable
            af0: df(fld(ln, 23, 42)),
            af1: df(fld(ln, 42, 61)),
            af2: df(fld(ln, 61, 80)),
            crs: f(0, 1),
            delta_n: f(0, 2) * ang,
            m0: f(0, 3) * ang,
            cuc: f(1, 0),
            e: f(1, 1),
            cus: f(1, 2),
            sqrt_a: f(1, 3),
            toe: f(2, 0),
            cic: f(2, 1),
            omega0: f(2, 2) * ang,
            cis: f(2, 3),
            i0: f(3, 0) * ang,
            crc: f(3, 1),
            omega: f(3, 2) * ang,
            omega_dot: f(3, 3) * ang,
            idot: f(4, 0) * ang,
            week: f(4, 2),
            tgd: f(5, 2),
            toc: gps_sow(y, mo, d, h, mi, s),
        };
        out.entry(prn)
            .and_modify(|cur| {
                if e.toe > cur.toe {
                    *cur = e.clone();
                }
            })
            .or_insert(e);
        i += 8;
    }
    out
}

fn kepler_e(m: f64, ecc: f64) -> f64 {
    let mut ek = m;
    for _ in 0..12 {
        ek -= (ek - ecc * ek.sin() - m) / (1.0 - ecc * ek.cos());
    }
    ek
}

fn wrap_tk(mut tk: f64) -> f64 {
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
        assert!((e.af0 - (-1.0e-4)).abs() < 1e-12);
        // angular field converted semicircles -> radians (M0 = 0.3 * pi)
        assert!((e.m0 - 0.3 * PI).abs() < 1e-9);
        // and the orbit it describes is a sane GPS radius
        let p = sat_pos_ecef(e, e.toe);
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        assert!(r > 26_000_000.0 && r < 27_100_000.0, "radius {r}");
    }
}
