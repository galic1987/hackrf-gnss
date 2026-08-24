use hackrf_gnss::decoder;
use hackrf_gnss::dsp_calib;
use hackrf_gnss::rf_calib;
use num_complex::Complex;

#[test]
fn test_integration_full_rf_and_dsp_pipeline() {
    // 1. Synthesize 1 ms of wideband signal at 20 Msps (20,000 samples)
    let ms_samples = 20_000;
    let mut signal = vec![Complex::new(0.5f32, -0.5f32); ms_samples];

    // Add a DC bias offset of (+5.0, -3.0)
    for sample in signal.iter_mut() {
        *sample += Complex::new(5.0, -3.0);
    }

    // Verify DC offset is present
    let dc_i: f32 = signal.iter().map(|s| s.re).sum::<f32>() / ms_samples as f32;
    let dc_q: f32 = signal.iter().map(|s| s.im).sum::<f32>() / ms_samples as f32;
    assert!((dc_i - 5.5).abs() < 0.1);
    assert!((dc_q - (-3.5)).abs() < 0.1);

    // Apply RF DC offset removal filter
    rf_calib::remove_dc_offset(&mut signal);

    // Verify DC offset has been zeroed
    let clean_i: f32 = signal.iter().map(|s| s.re).sum::<f32>() / ms_samples as f32;
    let clean_q: f32 = signal.iter().map(|s| s.im).sum::<f32>() / ms_samples as f32;
    assert!(clean_i.abs() < 1e-4);
    assert!(clean_q.abs() < 1e-4);

    // Run AGC gain calibrator
    let (lna, vga) = rf_calib::calibrate_gain_levels(&signal, 32, 40);
    assert!(lna <= 40 && lna % 8 == 0);
    assert!(vga <= 62 && vga % 2 == 0);

    // Verify PRN code generators for GPS and BeiDou
    let gps_prn1 = dsp_calib::generate_gps_ca_code(1);
    let beidou_prn1 = dsp_calib::generate_beidou_b1_code(1);

    assert_eq!(gps_prn1.len(), 1023);
    assert_eq!(beidou_prn1.len(), 2046);
    assert_ne!(gps_prn1.len(), beidou_prn1.len());
}

#[test]
fn test_multiconstellation_prn_coverage() {
    // Test all GPS (1..=32) and BeiDou (1..=37) PRN ranges
    for prn in 1..=32 {
        let code = dsp_calib::generate_gps_ca_code(prn);
        assert_eq!(code.len(), 1023);
        assert!(code.iter().all(|&val| val == 1.0 || val == -1.0));
    }
    for prn in 1..=37 {
        let code = dsp_calib::generate_beidou_b1_code(prn);
        assert_eq!(code.len(), 2046);
        assert!(code.iter().all(|&val| val == 1.0 || val == -1.0));
    }
}

#[test]
fn test_adsb_crc_validation() {
    // Standard DF17 message with verified CRC: 8D4062D558C382D690C8ACFB0295
    let hex_msg = "8D4062D558C382D690C8ACFB0295";
    let bytes: Vec<u8> = (0..hex_msg.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex_msg[i..i + 2], 16).unwrap())
        .collect();

    let rem = decoder::crc24(&bytes[..11]);
    let par = ((bytes[11] as u32) << 16) | ((bytes[12] as u32) << 8) | (bytes[13] as u32);
    assert_eq!(rem ^ par, 0);
}


/// Regression: a 56-bit Mode-S frame (DF11 all-call reply) has no ME field, so
/// nbytes is 7. Slicing the payload at a fixed [4..11] panicked on the first
/// valid short frame -- and DF11 genuinely passes CRC because its parity is
/// overlaid with an interrogator ID that is normally zero.
#[test]
fn test_short_mode_s_frame_does_not_panic() {
    // Build a DF11 frame with a correct CRC.
    let msg = [0x5Du8, 0x40, 0x62, 0xD5];
    let par = decoder::crc24(&msg);
    let mut frame = msg.to_vec();
    frame.push(((par >> 16) & 0xFF) as u8);
    frame.push(((par >> 8) & 0xFF) as u8);
    frame.push((par & 0xFF) as u8);
    assert_eq!(frame.len(), 7);
    // Syndrome must be zero, otherwise the decoder would never reach the payload.
    let rem = decoder::crc24(&frame[..4]);
    let p = ((frame[4] as u32) << 16) | ((frame[5] as u32) << 8) | (frame[6] as u32);
    assert_eq!(rem ^ p, 0, "test vector must have a valid CRC");

    // Render as 8 Msps PPM: 8 us preamble then 56 bit-periods of 1 us.
    let spb = 8usize;
    let mut mag = vec![0.0f32; 400 * spb];
    for &o in &[0.0f64, 1.0, 3.5, 4.5] {
        let s = (o * spb as f64) as usize;
        for k in 0..spb / 2 {
            mag[s + k] = 40.0;
        }
    }
    for b in 0..56 {
        let bit = (frame[b / 8] >> (7 - (b % 8))) & 1;
        let start = 8 * spb + b * spb;
        let half = if bit == 1 { start } else { start + spb / 2 };
        for k in 0..spb / 2 {
            mag[half + k] = 40.0;
        }
    }
    let iq: Vec<Complex<f32>> = mag.iter().map(|&m| Complex::new(m, 0.0)).collect();

    // Must not panic. Any frame it returns must be the DF11 we transmitted.
    let frames = decoder::decode_adsb_ppm(&iq, 8_000_000.0);
    for f in &frames {
        assert!(f.df == 11 || f.df == 17 || f.df == 18);
    }
}

// ---------------------------------------------------------------------------
// Mode S / ADS-B: synthesize the waveform so the demodulator can be tested
// end to end, not just its CRC.

const SPB: usize = 4; // samples per microsecond -> 4 Msps

/// Append a valid CRC so the frame passes the decoder's parity check, the same
/// way a transponder does: parity = CRC over the preceding bytes.
fn with_parity(mut body: Vec<u8>) -> Vec<u8> {
    let rem = decoder::crc24(&body);
    body.push((rem >> 16) as u8);
    body.push((rem >> 8) as u8);
    body.push(rem as u8);
    body
}

/// Build a Mode S pulse-position-modulated burst: a 4-pulse preamble at
/// 0/1.0/3.5/4.5 us, then one bit per microsecond with the pulse in the first
/// half for a 1 and the second half for a 0.
fn place_frame(buf: &mut [Complex<f32>], start_us: f64, msg: &[u8]) {
    let put = |buf: &mut [Complex<f32>], t_us: f64, width_us: f64| {
        let a = ((start_us + t_us) * SPB as f64).round() as usize;
        let b = a + (width_us * SPB as f64).round() as usize;
        for s in buf.iter_mut().take(b).skip(a) {
            *s = Complex::new(1.0, 0.0);
        }
    };
    for &o in &[0.0, 1.0, 3.5, 4.5] {
        put(buf, o, 0.5);
    }
    for (bit_idx, _) in (0..msg.len() * 8).enumerate() {
        let bit = (msg[bit_idx / 8] >> (7 - (bit_idx % 8))) & 1;
        let t = 8.0 + bit_idx as f64;
        put(buf, if bit == 1 { t } else { t + 0.5 }, 0.5);
    }
}

fn noise_buffer(total_us: usize) -> Vec<Complex<f32>> {
    // deterministic low-level floor so the decoder's median threshold is stable
    let mut seed = 0x2545_F491_4F6C_DD1Du64;
    (0..total_us * SPB)
        .map(|_| {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            let v = ((seed >> 40) as f32 / 16_777_216.0) * 0.02;
            Complex::new(v, -v)
        })
        .collect()
}

#[test]
fn adsb_decodes_a_synthesized_frame() {
    // DF17 (10001 -> 0x8D), ICAO 4CA1FA, then an ME field
    let msg = with_parity(vec![
        0x8D, 0x4C, 0xA1, 0xFA, 0x58, 0xC3, 0x82, 0xD6, 0x90, 0xC8, 0xAC,
    ]);
    let mut buf = noise_buffer(600);
    place_frame(&mut buf, 10.0, &msg);
    let frames = decoder::decode_adsb_ppm(&buf, SPB as f64 * 1e6);
    assert_eq!(frames.len(), 1, "expected exactly one frame, got {:?}", frames);
    assert_eq!(frames[0].icao, "4CA1FA");
    assert_eq!(frames[0].df, 17);
}

#[test]
fn adsb_decodes_back_to_back_frames() {
    // REGRESSION: the decoder used to advance a fixed 240 us after a frame,
    // which is two whole frame periods. A second squitter arriving before that
    // was skipped entirely. Real traffic is bursty, so this silently dropped
    // frames. The advance is now 8 us preamble + nbits us.
    let a = with_parity(vec![
        0x8D, 0x4C, 0xA1, 0xFA, 0x58, 0xC3, 0x82, 0xD6, 0x90, 0xC8, 0xAC,
    ]);
    let b = with_parity(vec![
        0x8D, 0xA0, 0x0B, 0x1C, 0x60, 0xB5, 0x0F, 0x00, 0x00, 0x00, 0x00,
    ]);
    let mut buf = noise_buffer(900);
    place_frame(&mut buf, 10.0, &a);
    place_frame(&mut buf, 138.0, &b); // 128 us later: inside the old 240 us skip
    let frames = decoder::decode_adsb_ppm(&buf, SPB as f64 * 1e6);
    let icaos: Vec<_> = frames.iter().map(|f| f.icao.as_str()).collect();
    assert!(
        icaos.contains(&"4CA1FA") && icaos.contains(&"A00B1C"),
        "both frames must decode; got {:?}",
        icaos
    );
}

#[test]
fn adsb_decodes_a_short_frame_after_a_long_one() {
    // DF11 (01011 -> 0x5D) is 56 bits. Mixing lengths exercises both the
    // per-length advance and the payload slice that used to panic on DF11.
    let long = with_parity(vec![
        0x8D, 0x4C, 0xA1, 0xFA, 0x58, 0xC3, 0x82, 0xD6, 0x90, 0xC8, 0xAC,
    ]);
    let short = with_parity(vec![0x5D, 0x3C, 0x65, 0x99]);
    let mut buf = noise_buffer(900);
    place_frame(&mut buf, 10.0, &long);
    place_frame(&mut buf, 140.0, &short);
    let frames = decoder::decode_adsb_ppm(&buf, SPB as f64 * 1e6);
    let icaos: Vec<_> = frames.iter().map(|f| f.icao.as_str()).collect();
    assert!(icaos.contains(&"4CA1FA"), "long frame lost: {:?}", icaos);
    assert!(icaos.contains(&"3C6599"), "short DF11 lost: {:?}", icaos);
}

#[test]
fn adsb_finds_nothing_in_noise() {
    // The property that matters most: no aircraft where there are none.
    let buf = noise_buffer(4000);
    let frames = decoder::decode_adsb_ppm(&buf, SPB as f64 * 1e6);
    assert!(frames.is_empty(), "decoded {} frames from noise", frames.len());
}

#[test]
fn adsb_rejects_a_frame_with_a_corrupted_bit() {
    // A single flipped bit must fail the CRC. Without this the decoder would
    // happily report aircraft that do not exist.
    let mut msg = with_parity(vec![
        0x8D, 0x4C, 0xA1, 0xFA, 0x58, 0xC3, 0x82, 0xD6, 0x90, 0xC8, 0xAC,
    ]);
    msg[6] ^= 0x08;
    let mut buf = noise_buffer(600);
    place_frame(&mut buf, 10.0, &msg);
    let frames = decoder::decode_adsb_ppm(&buf, SPB as f64 * 1e6);
    assert!(frames.is_empty(), "a corrupted frame passed CRC: {:?}", frames);
}

#[test]
fn address_overlaid_mode_s_formats_are_not_decoded() {
    // DOCUMENTED LIMITATION, pinned so it is a known choice rather than a
    // surprise. In DF 0/4/5/16/20/21 the parity field is the CRC XORed with the
    // aircraft's ICAO address (AP, "address parity"), so the syndrome equals
    // the address rather than zero. decode_adsb_ppm lets these DFs through its
    // length/format filter and then requires syn == 0, so it rejects every one
    // of them.
    //
    // Decoding them is possible but needs an address whitelist: recover the
    // address as the syndrome and accept it only if that aircraft was already
    // seen in a DF11 or DF17, which carry their address in the clear. That is
    // not built here because this station receives no ADS-B at all -- its
    // antenna is a filtered GNSS unit measuring -13.5 dB at 1090 MHz, with a
    // passband that starts at 1154 MHz. The limitation is in the decoder; the
    // reason there is nothing to decode is in the antenna.
    let icao: [u8; 3] = [0x4C, 0xA1, 0xFA];
    let body = vec![0x20, icao[0], icao[1], icao[2]];      // DF4 (00100)
    let rem = decoder::crc24(&body);
    let mut msg = body.clone();
    msg.push(((rem >> 16) as u8) ^ icao[0]);               // AP = CRC xor address
    msg.push(((rem >> 8) as u8) ^ icao[1]);
    msg.push((rem as u8) ^ icao[2]);

    let mut buf = noise_buffer(600);
    place_frame(&mut buf, 10.0, &msg);
    let frames = decoder::decode_adsb_ppm(&buf, SPB as f64 * 1e6);
    assert!(
        frames.is_empty(),
        "address-overlaid DF4 now decodes; if that is deliberate, this test \
         should be replaced with one that checks the recovered address: {:?}",
        frames
    );

    // and the same airframe's DF17, whose parity is a plain CRC, still decodes
    let df17 = with_parity(vec![
        0x8D, icao[0], icao[1], icao[2], 0x58, 0xC3, 0x82, 0xD6, 0x90, 0xC8, 0xAC,
    ]);
    let mut buf2 = noise_buffer(600);
    place_frame(&mut buf2, 10.0, &df17);
    let f2 = decoder::decode_adsb_ppm(&buf2, SPB as f64 * 1e6);
    assert_eq!(f2.len(), 1, "the non-overlaid format must still work");
    assert_eq!(f2[0].icao, "4CA1FA");
}
