//! GPS L1 C/A acquisition and the Doppler-physics detection verdict, ported
//! from the Python reference in `validation/` (acquire.py, report.py). The
//! acquisition metric and the verdict's pass-fit match the oracle so results can
//! be cross-checked against it.

pub mod acquire;
pub mod broadcast;
pub mod ca_code;
pub mod ephemeris;
pub mod hatch;
pub mod l2c_code;
pub mod l5_code;
pub mod lnav;
pub mod pvt;
pub mod sbas_code;
pub mod snapshot;
pub mod track;
pub mod verdict;

pub use acquire::{acquire, acquire_codes, acquire_sbas, AcqResult, F_L1};
pub use broadcast::{parse_rinex_gps, sat_clock, sat_pos_ecef, BrdcEph};
pub use sbas_code::sbas_code;
pub use snapshot::{snapshot_fix, Obs};
pub use ephemeris::{
    ephemeris_match, geodetic_to_ecef, load_tle, load_tle_named, predict_doppler_el,
    predict_doppler_el_f, sat_ecef, sat_ecef_pv, EphemMatch, GpsSat,
};
pub use l5_code::l5_code;
pub use verdict::{fit_passes, gnss_verdict, gnss_verdict_eph, Crossing, Pass, Verdict};
