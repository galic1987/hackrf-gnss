//! WAAS ionospheric grid model (DO-229): IGP coordinate tables, thin-shell
//! pierce point, IGP search + interpolation, obliquity factor. Ported from
//! RTKLIB sbas.c (igpband tables, searchigp, sbsioncorr weights) and
//! rtkcmn.c (ionppp) — the reference implementation of the DO-229 grid.

use std::collections::HashMap;

const X1: [i16; 28] = [
    -75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35,
    40, 45, 50, 55, 65, 75, 85,
];
const X2: [i16; 23] = [
    -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45,
    50, 55,
];
const X3: [i16; 27] = [
    -75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35,
    40, 45, 50, 55, 65, 75,
];
const X4: [i16; 28] = [
    -85, -75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30,
    35, 40, 45, 50, 55, 65, 75,
];
const X5: [i16; 72] = [
    -180, -175, -170, -165, -160, -155, -150, -145, -140, -135, -130, -125, -120, -115, -110, -105,
    -100, -95, -90, -85, -80, -75, -70, -65, -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10,
    -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105,
    110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175,
];
const X6: [i16; 36] = [
    -180, -170, -160, -150, -140, -130, -120, -110, -100, -90, -80, -70, -60, -50, -40, -30, -20,
    -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170,
];
const X7: [i16; 12] = [-180, -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150];
const X8: [i16; 12] = [-170, -140, -110, -80, -50, -20, 10, 40, 70, 100, 130, 160];

struct BandCol {
    /// Longitude (bands 0-8) or latitude (bands 9-10) of the column, deg.
    x: i16,
    /// Latitude (bands 0-8) or longitude (bands 9-10) list for the column.
    ys: &'static [i16],
    /// First 201-bit-mask index covered (1-based).
    bits: u16,
    /// Last mask index covered.
    bite: u16,
}

const fn c(x: i16, ys: &'static [i16], bits: u16, bite: u16) -> BandCol {
    BandCol { x, ys, bits, bite }
}

const IGPBAND1: [[BandCol; 8]; 9] = [
    [
        c(-180, &X1, 1, 28),
        c(-175, &X2, 29, 51),
        c(-170, &X3, 52, 78),
        c(-165, &X2, 79, 101),
        c(-160, &X3, 102, 128),
        c(-155, &X2, 129, 151),
        c(-150, &X3, 152, 178),
        c(-145, &X2, 179, 201),
    ],
    [
        c(-140, &X4, 1, 28),
        c(-135, &X2, 29, 51),
        c(-130, &X3, 52, 78),
        c(-125, &X2, 79, 101),
        c(-120, &X3, 102, 128),
        c(-115, &X2, 129, 151),
        c(-110, &X3, 152, 178),
        c(-105, &X2, 179, 201),
    ],
    [
        c(-100, &X3, 1, 27),
        c(-95, &X2, 28, 50),
        c(-90, &X1, 51, 78),
        c(-85, &X2, 79, 101),
        c(-80, &X3, 102, 128),
        c(-75, &X2, 129, 151),
        c(-70, &X3, 152, 178),
        c(-65, &X2, 179, 201),
    ],
    [
        c(-60, &X3, 1, 27),
        c(-55, &X2, 28, 50),
        c(-50, &X4, 51, 78),
        c(-45, &X2, 79, 101),
        c(-40, &X3, 102, 128),
        c(-35, &X2, 129, 151),
        c(-30, &X3, 152, 178),
        c(-25, &X2, 179, 201),
    ],
    [
        c(-20, &X3, 1, 27),
        c(-15, &X2, 28, 50),
        c(-10, &X3, 51, 77),
        c(-5, &X2, 78, 100),
        c(0, &X1, 101, 128),
        c(5, &X2, 129, 151),
        c(10, &X3, 152, 178),
        c(15, &X2, 179, 201),
    ],
    [
        c(20, &X3, 1, 27),
        c(25, &X2, 28, 50),
        c(30, &X3, 51, 77),
        c(35, &X2, 78, 100),
        c(40, &X4, 101, 128),
        c(45, &X2, 129, 151),
        c(50, &X3, 152, 178),
        c(55, &X2, 179, 201),
    ],
    [
        c(60, &X3, 1, 27),
        c(65, &X2, 28, 50),
        c(70, &X3, 51, 77),
        c(75, &X2, 78, 100),
        c(80, &X3, 101, 127),
        c(85, &X2, 128, 150),
        c(90, &X1, 151, 178),
        c(95, &X2, 179, 201),
    ],
    [
        c(100, &X3, 1, 27),
        c(105, &X2, 28, 50),
        c(110, &X3, 51, 77),
        c(115, &X2, 78, 100),
        c(120, &X3, 101, 127),
        c(125, &X2, 128, 150),
        c(130, &X4, 151, 178),
        c(135, &X2, 179, 201),
    ],
    [
        c(140, &X3, 1, 27),
        c(145, &X2, 28, 50),
        c(150, &X3, 51, 77),
        c(155, &X2, 78, 100),
        c(160, &X3, 101, 127),
        c(165, &X2, 128, 150),
        c(170, &X3, 151, 177),
        c(175, &X2, 178, 200),
    ],
];

const IGPBAND2: [[BandCol; 5]; 2] = [
    [
        c(60, &X5, 1, 72),
        c(65, &X6, 73, 108),
        c(70, &X6, 109, 144),
        c(75, &X6, 145, 180),
        c(85, &X7, 181, 192),
    ],
    [
        c(-60, &X5, 1, 72),
        c(-65, &X6, 73, 108),
        c(-70, &X6, 109, 144),
        c(-75, &X6, 145, 180),
        c(-85, &X8, 181, 192),
    ],
];

/// (lat, lon) degrees of mask bit `igp_num` (1-based) in `band`, per the
/// DO-229 IGP band layout. None for an out-of-range band/number.
pub fn igp_latlon(band: u8, igp_num: u16) -> Option<(i16, i16)> {
    if band <= 8 {
        for col in &IGPBAND1[band as usize] {
            if igp_num >= col.bits && igp_num <= col.bite {
                let lat = col.ys[(igp_num - col.bits) as usize];
                return Some((lat, col.x));
            }
        }
        None
    } else if band <= 10 {
        for col in &IGPBAND2[(band - 9) as usize] {
            if igp_num >= col.bits && igp_num <= col.bite {
                let lon = col.ys[(igp_num - col.bits) as usize];
                return Some((col.x, lon));
            }
        }
        None
    } else {
        None
    }
}

/// Azimuth/elevation (rad) of `rel` = sat − site (ECEF, any common unit)
/// seen from a site at geodetic (lat, lon) rad.
pub fn azel(site_lat: f64, site_lon: f64, rel: [f64; 3]) -> (f64, f64) {
    let (slat, clat) = site_lat.sin_cos();
    let (slon, clon) = site_lon.sin_cos();
    let e = -slon * rel[0] + clon * rel[1];
    let n = -slat * clon * rel[0] - slat * slon * rel[1] + clat * rel[2];
    let u = clat * clon * rel[0] + clat * slon * rel[1] + slat * rel[2];
    (e.atan2(n), u.atan2((e * e + n * n).sqrt()))
}

/// Thin-shell ionospheric pierce point. `pos` = site (lat, lon) rad,
/// `az`/`el` rad. Returns ((ipp lat, ipp lon) rad, obliquity factor).
/// Port of RTKLIB ionppp (re = 6378.1363 km, hion = 350 km).
pub fn ion_pierce_point(pos: (f64, f64), az: f64, el: f64) -> ((f64, f64), f64) {
    const RE: f64 = 6378.1363;
    const HION: f64 = 350.0;
    let rp = RE / (RE + HION) * el.cos();
    let ap = std::f64::consts::FRAC_PI_2 - el - rp.asin();
    let (sinap, _) = ap.sin_cos();
    let tanap = ap.tan();
    let cosaz = az.cos();
    let lat = (pos.0.sin() * ap.cos() + pos.0.cos() * sinap * cosaz).asin();
    let lon = if (pos.0 > 70.0f64.to_radians() && tanap * cosaz > (std::f64::consts::FRAC_PI_2 - pos.0).tan())
        || (pos.0 < -70.0f64.to_radians()
            && -tanap * cosaz > (std::f64::consts::FRAC_PI_2 + pos.0).tan())
    {
        pos.1 + std::f64::consts::PI - (sinap * az.sin() / lat.cos()).asin()
    } else {
        pos.1 + (sinap * az.sin() / lat.cos()).asin()
    };
    ((lat, lon), 1.0 / (1.0 - rp * rp).sqrt())
}

/// Slant ionospheric delay (m) at L1 for a pierce point at
/// (`ipp_lat_deg`, `ipp_lon_deg`) with obliquity `fp`, given the live IGP
/// vertical delays ((lat, lon) deg → metres). 4-corner bilinear when all
/// four surrounding IGPs are present, the DO-229 3-corner triangles
/// otherwise; None when no valid combination exists. Port of RTKLIB
/// searchigp + the sbsioncorr weighting.
pub fn iono_slant_delay(
    ipp_lat_deg: f64,
    ipp_lon_deg: f64,
    fp: f64,
    igps: &HashMap<(i16, i16), f64>,
) -> Option<f64> {
    let lat = ipp_lat_deg;
    let mut lon = ipp_lon_deg;
    if lon >= 180.0 {
        lon -= 360.0;
    }
    let latp: [i32; 2];
    let mut lonp: [i32; 4];
    let x: f64;
    let y: f64;
    if (-55.0..55.0).contains(&lat) {
        latp = [(lat / 5.0).floor() as i32 * 5, (lat / 5.0).floor() as i32 * 5 + 5];
        let l0 = (lon / 5.0).floor() as i32 * 5;
        lonp = [l0, l0, l0 + 5, l0 + 5];
        x = (lon - l0 as f64) / 5.0;
        y = (lat - latp[0] as f64) / 5.0;
    } else {
        latp = [
            ((lat - 5.0) / 10.0).floor() as i32 * 10 + 5,
            ((lat - 5.0) / 10.0).floor() as i32 * 10 + 15,
        ];
        let l0 = (lon / 10.0).floor() as i32 * 10;
        lonp = [l0, l0, l0 + 10, l0 + 10];
        x = (lon - l0 as f64) / 10.0;
        y = (lat - latp[0] as f64) / 10.0;
        if (75.0..85.0).contains(&lat) {
            lonp[1] = (lon / 90.0).floor() as i32 * 90;
            lonp[3] = lonp[1] + 90;
        } else if (-85.0..-75.0).contains(&lat) {
            lonp[0] = ((lon - 50.0) / 90.0).floor() as i32 * 90 + 40;
            lonp[2] = lonp[0] + 90;
        } else if lat >= 85.0 {
            let l90 = (lon / 90.0).floor() as i32 * 90;
            lonp = [l90, l90, l90, l90];
        } else if lat < -85.0 {
            let l90 = ((lon - 50.0) / 90.0).floor() as i32 * 90 + 40;
            lonp = [l90, l90, l90, l90];
        }
    }
    for l in lonp.iter_mut() {
        if *l == 180 {
            *l = -180;
        }
    }
    // corners: 0 = ws, 1 = wn, 2 = es, 3 = en
    let corner = [
        (latp[0] as i16, lonp[0] as i16),
        (latp[1] as i16, lonp[1] as i16),
        (latp[0] as i16, lonp[2] as i16),
        (latp[1] as i16, lonp[3] as i16),
    ];
    let d: [Option<f64>; 4] = [
        igps.get(&corner[0]).copied(),
        igps.get(&corner[1]).copied(),
        igps.get(&corner[2]).copied(),
        igps.get(&corner[3]).copied(),
    ];
    let w: [f64; 4];
    if d.iter().all(|v| v.is_some()) {
        w = [(1.0 - x) * (1.0 - y), (1.0 - x) * y, x * (1.0 - y), x * y];
    } else if d[0].is_some() && d[1].is_some() && d[2].is_some() {
        let w0 = 1.0 - y - x;
        if w0 < 0.0 {
            return None;
        }
        w = [w0, y, x, 0.0];
    } else if d[0].is_some() && d[2].is_some() && d[3].is_some() {
        let w0 = 1.0 - x;
        let w3 = y;
        let w2c = 1.0 - w0 - w3;
        if w2c < 0.0 {
            return None;
        }
        w = [w0, 0.0, w2c, w3];
    } else if d[0].is_some() && d[1].is_some() && d[3].is_some() {
        let w0 = 1.0 - y;
        let w3 = x;
        let w1 = 1.0 - w0 - w3;
        if w1 < 0.0 {
            return None;
        }
        w = [w0, w1, 0.0, w3];
    } else if d[1].is_some() && d[2].is_some() && d[3].is_some() {
        let w1 = 1.0 - x;
        let w2 = 1.0 - y;
        let w3 = 1.0 - w1 - w2;
        if w3 < 0.0 {
            return None;
        }
        w = [0.0, w1, w2, w3];
    } else {
        return None;
    }
    let mut vert = 0.0;
    for i in 0..4 {
        if let Some(di) = d[i] {
            vert += w[i] * di;
        }
    }
    Some(fp * vert)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn igp_latlon_spot_checks() {
        // Band 0, mask bit 1: first column, first latitude entry.
        assert_eq!(igp_latlon(0, 1), Some((-75, -180)));
        // Band 0, bit 29: second column (lon -175), first X2 entry.
        assert_eq!(igp_latlon(0, 29), Some((-55, -175)));
        // Band 4 (lon 0 column at bits 101-128), bit 101: X1[0] = -75.
        assert_eq!(igp_latlon(4, 101), Some((-75, 0)));
        // Band 9 (polar north): bit 1 -> lat 60, lon -180.
        assert_eq!(igp_latlon(9, 1), Some((60, -180)));
        // Out of range.
        assert_eq!(igp_latlon(0, 202), None);
        assert_eq!(igp_latlon(11, 1), None);
    }

    #[test]
    fn pierce_point_zenith() {
        // Zenith: pierce point at the site, obliquity 1.
        let (lat, lon) = (39.0f64.to_radians(), -77.0f64.to_radians());
        let ((plat, plon), fp) = ion_pierce_point((lat, lon), 0.0, std::f64::consts::FRAC_PI_2);
        assert!((plat - lat).abs() < 1e-9);
        assert!((plon - lon).abs() < 1e-9);
        assert!((fp - 1.0).abs() < 1e-12);
    }

    #[test]
    fn slant_delay_four_corners() {
        // Pierce point at (37.5, -77.5): mid-grid corners (35,-80),(40,-80),
        // (35,-75),(40,-75) with delays 1,2,3,4 m; x=y=0.5 -> mean 2.5.
        let mut igps = HashMap::new();
        igps.insert((35i16, -80i16), 1.0);
        igps.insert((40i16, -80i16), 2.0);
        igps.insert((35i16, -75i16), 3.0);
        igps.insert((40i16, -75i16), 4.0);
        let d = iono_slant_delay(37.5, -77.5, 1.0, &igps).unwrap();
        assert!((d - 2.5).abs() < 1e-12);
        // Obliquity scales the vertical delay.
        let d2 = iono_slant_delay(37.5, -77.5, 2.0, &igps).unwrap();
        assert!((d2 - 5.0).abs() < 1e-12);
        // A missing corner falls back to the 3-corner triangle.
        igps.remove(&(40, -75));
        let d3 = iono_slant_delay(37.4, -77.6, 1.0, &igps).unwrap();
        // corners present: ws(35,-80)=1, wn(40,-80)=2, es(35,-75)=3
        // x=(−77.6+80)/5=0.48, y=(37.4−35)/5=0.48; w=[1−x−y, y, x]
        let expect = (1.0 - 0.48 - 0.48) * 1.0 + 0.48 * 2.0 + 0.48 * 3.0;
        assert!((d3 - expect).abs() < 1e-12);
        // No corners at all -> None.
        assert!(iono_slant_delay(37.5, -77.5, 1.0, &HashMap::new()).is_none());
    }
}
