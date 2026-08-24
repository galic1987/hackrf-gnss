//! Timestamp sidecar: per-capture calibration of the FPGA timestamp counter
//! and the JSON record (`l1_ts_<TS>.json`) that anchors a capture to it.
//!
//! The counter's tick rate depends on the FPGA image and sample rate (AFE
//! clock / fs: measured 32 MHz at 8 Msps std and at 8 Msps ext since the
//! boost, 40 MHz at legacy 2.5 Msps ext) and is
//! NEVER hardcoded — it is calibrated per capture by reading the counter
//! before and after a capture of a known number of samples.
//!
//! Where the stream start tick comes from depends on the image: on images 0/1
//! the counter is read over SPI (`hackrf_pro --ts-read start`); on image 2
//! (ext) SPI reads 0 and the counter travels in-stream in the top nibble of
//! each 12-bit lane — see `ts_nibble`.

use serde::{Deserialize, Serialize};

/// Calibrated counter tick rate from a capture of `n_samples` at `fs` Hz,
/// bracketed by counter reads `start_ticks` / `end_ticks`.
pub fn calibrate_tick_hz(start_ticks: u64, end_ticks: u64, n_samples: u64, fs: f64) -> f64 {
    (end_ticks - start_ticks) as f64 * fs / n_samples as f64
}

/// Where `stream_start_ticks` was obtained from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TicksSource {
    /// `hackrf_pro --ts-read start` over SPI (images 0 and 1).
    #[serde(rename = "spi")]
    Spi,
    /// First validated counter anchor from the in-stream nibble channel
    /// (image 2, ext) — see `ts_nibble::extract_with_tps`.
    #[serde(rename = "nibble-stream")]
    NibbleStream,
}

/// The sidecar JSON written next to every timestamped capture.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TsSidecar {
    /// Counter value at the first sample of the capture (SPI `start` latch on
    /// images 0/1, first nibble anchor on image 2).
    pub stream_start_ticks: u64,
    /// Calibrated counter tick rate for this capture — never assumed.
    pub tick_hz: f64,
    /// Sample rate the capture was taken at.
    pub fs: f64,
    /// FPGA image index: 0 = std, 1 = half-prec, 2 = ext.
    pub image: u8,
    pub ticks_source: TicksSource,
    /// False until the counter is disciplined against GPS nav time; the
    /// counter epoch is arbitrary (it starts at FPGA boot).
    pub utc_known: bool,
}
