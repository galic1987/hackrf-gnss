//! Iridium frame parsing in Rust: a replacement for the parts of
//! iridium-toolkit this station depends on.
//!
//! Why port it at all: the Python toolkit lives outside this repository (it was
//! sitting in a temporary directory), and the station's whole Iridium result --
//! 37,000 frames, 74 spacecraft, the Doppler geolocation -- rests on it. A
//! dependency that can vanish between reboots is a poor foundation for a
//! measurement.
//!
//! Correctness is established against that toolkit rather than against this
//! code's own expectations: `verify_against_reference` replays real frames
//! through both and compares field by field. That is the same discipline the
//! rest of this project uses, and it is there because a decoder validated only
//! against itself will happily agree with its own mistakes -- which is exactly
//! how 39 SBAS ranging codes stayed time-mirrored here for weeks.

pub mod bch;
pub mod demod3;
pub mod frame;
pub mod geo;
pub mod locate;
pub mod message;
pub mod ppm;

pub use frame::{parse_line, Frame};
pub use locate::{locate, score_location, LocateResult, Track};
pub use message::{classify, Class, Ira};
