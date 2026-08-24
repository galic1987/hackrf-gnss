use num_complex::Complex;
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct AdsbFrame {
    pub df: u8,
    pub icao: String,
    pub msg_type: u8,
    pub payload_hex: String,
    pub callsign: Option<String>,
    pub altitude_ft: Option<i32>,
    pub speed_kts: Option<u16>,
    pub heading_deg: Option<u16>,
    pub detail: String,
}

/// Mode-S CRC-24 calculation (Polynomial: 0xFFF409)
pub fn crc24(msg_bytes: &[u8]) -> u32 {
    let poly = 0xFFF409u32;
    let mut reg = 0u32;
    for &b in msg_bytes {
        reg ^= (b as u32) << 16;
        for _ in 0..8 {
            if (reg & 0x800000) != 0 {
                reg = ((reg << 1) ^ poly) & 0xFFFFFF;
            } else {
                reg = (reg << 1) & 0xFFFFFF;
            }
        }
    }
    reg
}

/// Decodes callsign from Type Code 1..=4
fn decode_callsign(payload: &[u8]) -> Option<String> {
    if payload.len() < 7 {
        return None;
    }
    let chars = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######";
    let chars_bytes = chars.as_bytes();

    // 8 6-bit characters packed into bits 8..56 of payload (bytes 1..7)
    let b = &payload[1..7];
    let mut indices = [0usize; 8];
    indices[0] = (b[0] >> 2) as usize;
    indices[1] = (((b[0] & 0x03) << 4) | (b[1] >> 4)) as usize;
    indices[2] = (((b[1] & 0x0F) << 2) | (b[2] >> 6)) as usize;
    indices[3] = (b[2] & 0x3F) as usize;
    indices[4] = (b[3] >> 2) as usize;
    indices[5] = (((b[3] & 0x03) << 4) | (b[4] >> 4)) as usize;
    indices[6] = (((b[4] & 0x0F) << 2) | (b[5] >> 6)) as usize;
    indices[7] = (b[5] & 0x3F) as usize;

    let mut callsign = String::new();
    for &idx in &indices {
        if idx < chars_bytes.len() {
            let ch = chars_bytes[idx] as char;
            if ch != '#' && ch != '_' {
                callsign.push(ch);
            }
        }
    }

    let trimmed = callsign.trim().to_string();
    if trimmed.is_empty() {
        None
    } else {
        Some(trimmed)
    }
}

/// Decodes altitude from 12-bit Mode-S altitude field (Q-bit format)
fn decode_altitude(payload: &[u8]) -> Option<i32> {
    if payload.len() < 3 {
        return None;
    }
    // Altitude field is 12 bits: bits 8..20 of payload (byte 1 and upper 4 bits of byte 2)
    let raw_alt = (((payload[1] as u16) << 4) | ((payload[2] >> 4) as u16)) & 0x0FFF;
    let q_bit = (raw_alt & 0x0010) != 0;

    if q_bit {
        // 25 ft increments
        let n = ((raw_alt & 0x0FE0) >> 1) | (raw_alt & 0x000F);
        let alt = (n as i32) * 25 - 1000;
        Some(alt)
    } else {
        // Gray code / 100 ft increments
        None
    }
}

/// Decodes ground speed and heading from Type Code 19 (Airborne Velocity)
fn decode_velocity(payload: &[u8]) -> (Option<u16>, Option<u16>) {
    if payload.len() < 7 {
        return (None, None);
    }
    let sub_type = payload[0] & 0x07;
    if sub_type == 1 || sub_type == 2 {
        let dew = (payload[1] >> 2) & 0x01;
        let v_ew = (((payload[1] as u16 & 0x03) << 8) | (payload[2] as u16)) as f32 - 1.0;
        let dns = (payload[3] >> 7) & 0x01;
        let v_ns = (((payload[3] as u16 & 0x7F) << 3) | ((payload[4] >> 5) as u16)) as f32 - 1.0;

        if v_ew >= 0.0 && v_ns >= 0.0 {
            let vel_x = if dew == 1 { -v_ew } else { v_ew };
            let vel_y = if dns == 1 { -v_ns } else { v_ns };
            let speed = (vel_x * vel_x + vel_y * vel_y).sqrt().round() as u16;
            let mut heading = (vel_x.atan2(vel_y).to_degrees()).round() as i32;
            if heading < 0 {
                heading += 360;
            }
            return (Some(speed), Some(heading as u16));
        }
    }
    (None, None)
}

/// PPM Demodulator for ADS-B Mode-S (1090 MHz) with Mandatory 24-bit CRC Check
pub fn decode_adsb_ppm(iq_samples: &[Complex<f32>], sample_rate: f64) -> Vec<AdsbFrame> {
    let mut frames = Vec::new();
    let spb = (sample_rate / 1_000_000.0) as usize; // samples per us
    if spb == 0 {
        return frames;
    }

    let mag: Vec<f32> = iq_samples.iter().map(|c| c.norm()).collect();
    let n = mag.len();
    if n < 240 * spb {
        return frames;
    }

    let hi_off = [0.0, 1.0, 3.5, 4.5];
    let lo_off = [0.5, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 5.5];
    let hi: Vec<usize> = hi_off.iter().map(|o| (o * spb as f64).round() as usize).collect();
    let lo: Vec<usize> = lo_off.iter().map(|o| (o * spb as f64).round() as usize).collect();

    // Noise floor threshold calculation
    let mut sample_for_median = Vec::with_capacity(1000.min(n));
    let step = (n / 1000).max(1);
    for i in (0..n).step_by(step) {
        sample_for_median.push(mag[i]);
    }
    sample_for_median.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median_mag = sample_for_median[sample_for_median.len() / 2];
    let thresh_noise = median_mag * 2.5;

    let mut i = 0;
    while i < n - 240 * spb {
        let h_min = hi.iter().map(|&k| mag[i + k]).fold(f32::INFINITY, f32::min);
        if h_min < thresh_noise {
            i += 1;
            continue;
        }

        let l_max = lo.iter().map(|&k| mag[i + k]).fold(f32::NEG_INFINITY, f32::max);
        if h_min <= l_max * 1.3 {
            i += 1;
            continue;
        }

        // Preamble detected! Check 112-bit (DF17/18/19/20/21) and 56-bit frames
        for &(nbits, nbytes) in &[(112, 14), (56, 7)] {
            let start = i + 8 * spb;
            if start + nbits * spb + spb > n {
                continue;
            }

            let mut bits = vec![0u8; nbits];
            for b in 0..nbits {
                let idx = start + b * spb;
                let half = spb / 2;
                let first_half: f32 = mag[idx..idx + half].iter().sum();
                let second_half: f32 = mag[idx + half..idx + spb].iter().sum();
                bits[b] = if first_half > second_half { 1 } else { 0 };
            }

            let mut bytes = vec![0u8; nbytes];
            for b in 0..nbits {
                if bits[b] == 1 {
                    bytes[b / 8] |= 1 << (7 - (b % 8));
                }
            }

            let df = (bytes[0] >> 3) & 0x1F;
            if nbits == 112 && !matches!(df, 16 | 17 | 18 | 19 | 20 | 21 | 24) {
                continue;
            }
            if nbits == 56 && !matches!(df, 0 | 4 | 5 | 11) {
                continue;
            }

            let rem = crc24(&bytes[..nbytes - 3]);
            let par = ((bytes[nbytes - 3] as u32) << 16) | ((bytes[nbytes - 2] as u32) << 8) | (bytes[nbytes - 1] as u32);
            let syn = rem ^ par;

            if syn == 0 {
                // Verified 100% Clean CRC Pass
                let icao = format!("{:02X}{:02X}{:02X}", bytes[1], bytes[2], bytes[3]);
                let type_code = (bytes[4] >> 3) & 0x1F;
                // 56-bit frames (DF11 etc) have no ME field: nbytes is 7, so the
                // payload slice must end at nbytes-3, not a fixed 11.
                let payload = &bytes[4..nbytes - 3];
                let payload_hex = payload.iter().map(|b| format!("{:02X}", b)).collect::<String>();

                let mut callsign = None;
                let mut altitude_ft = None;
                let mut speed_kts = None;
                let mut heading_deg = None;
                let mut detail = format!("DF{} Mode-S Squitter [CRC-OK]", df);

                if df == 17 || df == 18 {
                    if (1..=4).contains(&type_code) {
                        callsign = decode_callsign(payload);
                        if let Some(ref cs) = callsign {
                            detail = format!("DF{} Aircraft ID / Callsign: {}", df, cs);
                        }
                    } else if (9..=18).contains(&type_code) || (20..=22).contains(&type_code) {
                        altitude_ft = decode_altitude(payload);
                        if let Some(alt) = altitude_ft {
                            detail = format!("DF{} Airborne Position (Alt: {} ft)", df, alt);
                        }
                    } else if type_code == 19 {
                        let (spd, hdg) = decode_velocity(payload);
                        speed_kts = spd;
                        heading_deg = hdg;
                        if let (Some(s), Some(h)) = (spd, hdg) {
                            detail = format!("DF{} Velocity: {} kts @ {}°", df, s, h);
                        }
                    }
                }

                frames.push(AdsbFrame {
                    df,
                    icao,
                    msg_type: type_code,
                    payload_hex,
                    callsign,
                    altitude_ft,
                    speed_kts,
                    heading_deg,
                    detail,
                });
                // Advance past THIS frame only: 8 us preamble + nbits us.
                // A fixed 240 us skipped a further whole frame period, dropping
                // back-to-back squitters. (The Python decoder was fixed for the
                // same bug; this one was missed.)
                i += (8 + nbits) * spb;
                break;
            }
        }
        i += 1;
    }

    frames
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hexb(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    #[test]
    fn test_crc24_zero_syndrome() {
        // Standard DF17 test message with verified CRC: 8D4062D558C382D690C8ACFB0295
        let bytes = hexb("8D4062D558C382D690C8ACFB0295");
        assert_eq!(bytes.len(), 14);
        let rem = crc24(&bytes[..11]);
        let par = ((bytes[11] as u32) << 16) | ((bytes[12] as u32) << 8) | (bytes[13] as u32);
        assert_eq!(rem ^ par, 0);
    }

    #[test]
    fn decodes_a_known_callsign() {
        // documented DF17 aircraft-ID frame -> callsign "KLMH" style; the ME field
        // is bytes[4..11]. This exercises decode_callsign end to end.
        let bytes = hexb("8D4840D6202CC371C32CE0576098");
        let cs = decode_callsign(&bytes[4..11]).expect("callsign");
        assert!(cs.chars().all(|c| c.is_ascii_alphanumeric()), "clean callsign: {cs}");
        assert!(!cs.is_empty() && cs.len() <= 8);
    }

    #[test]
    fn decodes_altitude_and_velocity() {
        // airborne position frame (TC 11) carries a Q-bit altitude
        let pos = hexb("8D40621D58C382D690C8AC2863A7");
        let alt = decode_altitude(&pos[4..11]);
        assert!(alt.is_some(), "airborne position should yield an altitude");
        assert!((0..60000).contains(&alt.unwrap()));
        // airborne velocity frame (TC 19)
        let vel = hexb("8D485020994409940838175B284F");
        let (spd, hdg) = decode_velocity(&vel[4..11]);
        assert!(spd.is_some() && hdg.is_some(), "velocity subtype 1 decodes");
        assert!(spd.unwrap() < 1000 && hdg.unwrap() < 360);
    }

    #[test]
    fn decodes_a_synthetic_ppm_frame_end_to_end() {
        // Build a clean 1090 MHz PPM waveform for a CRC-valid DF17 frame and
        // confirm the demodulator recovers it with the right ICAO.
        let bytes = hexb("8D4062D558C382D690C8ACFB0295");
        let mut bits = [0u8; 112];
        for (b, slot) in bits.iter_mut().enumerate() {
            *slot = (bytes[b / 8] >> (7 - (b % 8))) & 1;
        }
        let sr = 8.0e6;
        let spb = (sr / 1e6) as usize; // 8 samples/us
        let n = 300 * spb; // >= the 240 us the demodulator requires
        let mut mag = vec![0.6f32; n]; // noise floor
        let pulse = |mag: &mut [f32], at_us: f64| {
            let s = (at_us * spb as f64) as usize;
            for x in mag.iter_mut().take(s + spb / 2).skip(s) {
                *x = 9.0;
            }
        };
        // Mode-S preamble pulses at 0, 1, 3.5, 4.5 us
        for &t in &[0.0, 1.0, 3.5, 4.5] {
            pulse(&mut mag, t);
        }
        // data: bit=1 -> energy in first half-chip, bit=0 -> second half
        for (b, &bit) in bits.iter().enumerate() {
            let s = (8 + b) * spb;
            let (lo, hi) = if bit == 1 { (s, s + spb / 2) } else { (s + spb / 2, s + spb) };
            for x in mag.iter_mut().take(hi).skip(lo) {
                *x = 9.0;
            }
        }
        let iq: Vec<Complex<f32>> = mag.iter().map(|&m| Complex::new(m, 0.0)).collect();
        let frames = decode_adsb_ppm(&iq, sr);
        assert!(!frames.is_empty(), "should decode the synthetic frame");
        assert_eq!(frames[0].icao, "4062D5");
        assert_eq!(frames[0].df, 17);
    }
}

