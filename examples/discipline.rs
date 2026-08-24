//! Timestamp-counter discipline: calibrates the FPGA counter's tick rate for
//! one capture, recovers the stream-start tick, and writes the timestamp
//! sidecar (`l1_ts_<TS>.json`). When a GPS nav-time fix is available this is
//! also where the counter would be anchored to absolute time (via
//! `hackrf_pro --ts-set`); the gps module parses LNAV subframes at the bit
//! level but has no capture -> time-fix pipeline yet, so the example stops at
//! calibration + sidecar and says so. That is the complete deliverable for
//! now, not a stub.
//!
//! usage:
//!   discipline <capture> <fs> --image <0|1|2> [--sidecar out.json]
//!              [--ts-begin T] [--ts-end T] [--ts-start T]
//!
//!   image 2 (ext, i16 .rawiq): tick rate and stream start come from the
//!       in-stream nibble channel (ts_nibble); no --ts-* args needed.
//!   image 0/1 (i8 .iq): tick rate is calibrated from counter reads that
//!       bracketed the capture: --ts-begin/--ts-end are `--ts-read now`
//!       before/after, --ts-start is `--ts-read start` after the capture
//!       (falls back to --ts-begin, which predates the stream start).
use hackrf_gnss::ts_nibble;
use hackrf_gnss::ts_sidecar::{calibrate_tick_hz, TicksSource, TsSidecar};

fn take_flag(a: &[String], name: &str) -> Option<String> {
    a.iter()
        .position(|s| s == name)
        .and_then(|i| a.get(i + 1).cloned())
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 4 {
        eprintln!(
            "usage: discipline <capture> <fs> --image <0|1|2> [--sidecar out.json]\n\
             \x20        [--ts-begin T] [--ts-end T] [--ts-start T]"
        );
        std::process::exit(2);
    }
    let path = &a[1];
    let fs: f64 = a[2].parse().expect("fs");
    let image: u8 = take_flag(&a, "--image")
        .and_then(|s| s.parse().ok())
        .expect("--image 0|1|2");
    let sidecar_path = take_flag(&a, "--sidecar");
    let bytes = std::fs::read(path).expect("read capture");

    let sidecar = if image == 2 {
        // ext: 12-bit samples right-justified in little-endian i16 lanes,
        // timestamp counter in the top nibble of each lane
        let raw: Vec<i16> = bytes
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect();
        let n_samples = (raw.len() / 2) as u64;
        let tps = ts_nibble::estimate_ticks_per_sample(&raw)
            .expect("no tps estimate — too few nibble rotations in the stream");
        let ts = ts_nibble::extract_with_tps(&raw, tps);
        let &(idx, c0) = ts
            .first()
            .expect("no validated timestamp anchor in the stream");
        let tick_hz = tps as f64 * fs;
        eprintln!(
            "nibble: tps={} tick_hz={:.2} first anchor at sample {} ({} anchors, {} samples)",
            tps,
            tick_hz,
            idx,
            ts.len(),
            n_samples
        );
        TsSidecar {
            stream_start_ticks: c0,
            tick_hz,
            fs,
            image,
            ticks_source: TicksSource::NibbleStream,
            utc_known: false,
        }
    } else {
        // std/half-prec: interleaved i8, calibration from bracketing SPI reads
        let n_samples = (bytes.len() / 2) as u64;
        let begin: u64 = take_flag(&a, "--ts-begin")
            .and_then(|s| s.parse().ok())
            .expect("image 0/1 needs --ts-begin (ts-read now before the capture)");
        let end: u64 = take_flag(&a, "--ts-end")
            .and_then(|s| s.parse().ok())
            .expect("image 0/1 needs --ts-end (ts-read now after the capture)");
        let start: u64 = take_flag(&a, "--ts-start")
            .and_then(|s| s.parse().ok())
            .unwrap_or_else(|| {
                eprintln!("no --ts-start (ts-read start); using --ts-begin, which predates the stream start");
                begin
            });
        let tick_hz = calibrate_tick_hz(begin, end, n_samples, fs);
        eprintln!(
            "spi: dticks={} over {} samples -> tick_hz={:.2}",
            end - begin,
            n_samples,
            tick_hz
        );
        TsSidecar {
            stream_start_ticks: start,
            tick_hz,
            fs,
            image,
            ticks_source: TicksSource::Spi,
            utc_known: false,
        }
    };

    println!(
        "stream_start_ticks = {} ticks @ {:.2} Hz ({:?}, image {})",
        sidecar.stream_start_ticks, sidecar.tick_hz, sidecar.ticks_source, sidecar.image
    );
    if let Some(p) = sidecar_path {
        std::fs::write(&p, serde_json::to_string_pretty(&sidecar).unwrap()).expect("write sidecar");
        println!("sidecar written: {p}");
    }
    // Absolute-time anchoring: the gps module decodes LNAV subframes
    // (lnav::find_subframes yields TOW from the HOW) but nothing in the crate
    // yet turns a capture into an absolute time fix, so there is no epoch to
    // set the counter to. When that lands: ticks_at_fix = (utc - epoch) *
    // tick_hz, then `hackrf_pro -d <serial> --ts-set <ticks>` (--apply).
    println!("no time fix — epoch not anchored");
}
