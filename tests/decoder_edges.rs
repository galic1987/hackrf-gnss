//! Edge and rejection tests for the ADS-B decoder: malformed frames must be
//! rejected (not panicked on, not half-decoded), degenerate inputs must yield
//! nothing, and the 56-bit short-frame path must decode a CRC-valid DF11.

use hackrf_gnss::decoder::{crc24, decode_adsb_ppm};
use num_complex::Complex;

fn hexb(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

/// Build a 1090 MHz PPM magnitude waveform for `bytes` (the same synthesis the
/// in-module happy-path test uses): preamble pulses at 0/1/3.5/4.5 us, then
/// bit=1 as energy in the first half-chip, bit=0 in the second.
fn ppm_waveform(bytes: &[u8], sr: f64) -> Vec<Complex<f32>> {
    let spb = (sr / 1e6) as usize;
    let n = 300 * spb; // >= the 240 us the demodulator requires
    let mut mag = vec![0.6f32; n]; // noise floor
    let mut pulse = |mag: &mut [f32], at_us: f64| {
        let s = (at_us * spb as f64) as usize;
        for x in mag.iter_mut().take(s + spb / 2).skip(s) {
            *x = 9.0;
        }
    };
    for &t in &[0.0, 1.0, 3.5, 4.5] {
        pulse(&mut mag, t);
    }
    for b in 0..bytes.len() * 8 {
        let bit = (bytes[b / 8] >> (7 - (b % 8))) & 1;
        let s = (8 + b) * spb;
        let (lo, hi) = if bit == 1 { (s, s + spb / 2) } else { (s + spb / 2, s + spb) };
        for x in mag.iter_mut().take(hi).skip(lo) {
            *x = 9.0;
        }
    }
    mag.iter().map(|&m| Complex::new(m, 0.0)).collect()
}

#[test]
fn a_frame_with_a_corrupted_crc_is_rejected() {
    // the known-good DF17 from the in-module test, with one parity bit flipped
    let mut bytes = hexb("8D4062D558C382D690C8ACFB0295");
    bytes[13] ^= 0x01;
    let frames = decode_adsb_ppm(&ppm_waveform(&bytes, 8.0e6), 8.0e6);
    assert!(frames.is_empty(), "a corrupt CRC must not decode: {frames:?}");
}

#[test]
fn a_frame_with_a_bad_df_is_rejected_before_the_crc() {
    // DF=7 is not in the decoder's accepted set for 112-bit frames; the same
    // bytes with a freshly computed valid CRC must still be rejected
    let mut bytes = hexb("8D4062D558C382D690C8ACFB0295");
    bytes[0] = (7 << 3) | (bytes[0] & 0x07);
    let rem = crc24(&bytes[..11]);
    bytes[11] = (rem >> 16) as u8;
    bytes[12] = (rem >> 8) as u8;
    bytes[13] = rem as u8;
    let frames = decode_adsb_ppm(&ppm_waveform(&bytes, 8.0e6), 8.0e6);
    assert!(frames.is_empty(), "DF7 is not decodable: {frames:?}");
}

#[test]
fn pure_noise_yields_no_frames() {
    let sr = 8.0e6;
    let n = 300 * 8;
    let mut s = 0x9E3779B97F4A7C15u64;
    let iq: Vec<Complex<f32>> = (0..n)
        .map(|_| {
            // xorshift64* noise tightly around the 0.6 floor -- never 2.5x it
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            let u = (s >> 11) as f32 / (1u64 << 53) as f32;
            Complex::new(0.5 + 0.2 * u, 0.0)
        })
        .collect();
    assert!(decode_adsb_ppm(&iq, sr).is_empty(), "noise decoded as a frame");
}

#[test]
fn degenerate_inputs_yield_no_frames_and_do_not_panic() {
    // shorter than one 240 us frame window
    let short = vec![Complex::new(0.0f32, 0.0); 1000];
    assert!(decode_adsb_ppm(&short, 8.0e6).is_empty());
    // a sample rate below 1 MHz leaves zero samples per microsecond
    let bytes = hexb("8D4062D558C382D690C8ACFB0295");
    let w = ppm_waveform(&bytes, 8.0e6);
    assert!(decode_adsb_ppm(&w, 0.5e6).is_empty());
    // empty input
    assert!(decode_adsb_ppm(&[], 8.0e6).is_empty());
}

#[test]
fn a_df11_all_call_reply_decodes_through_the_56_bit_path() {
    // 56-bit short frame: DF=11, CA=5, ICAO 0xABCDEF, CRC over the first 4
    // bytes appended as the 3-byte parity
    let mut bytes = vec![(11u8 << 3) | 5, 0xAB, 0xCD, 0xEF];
    let rem = crc24(&bytes);
    bytes.extend_from_slice(&[(rem >> 16) as u8, (rem >> 8) as u8, rem as u8]);
    let frames = decode_adsb_ppm(&ppm_waveform(&bytes, 8.0e6), 8.0e6);
    assert_eq!(frames.len(), 1, "the DF11 frame must decode exactly once: {frames:?}");
    assert_eq!(frames[0].df, 11);
    assert_eq!(frames[0].icao, "ABCDEF");
    // a 56-bit frame has no ME field: no callsign/altitude/velocity may appear
    assert!(frames[0].callsign.is_none());
    assert!(frames[0].altitude_ft.is_none());
    assert!(frames[0].speed_kts.is_none());
}
