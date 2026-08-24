//! Engine-level smoke test: the Downconvert + decimation path from raw int8
//! to the bands. The tracker integration test drives `Band` directly, so
//! this is the only coverage of `Engine::push_i8` — added after the live
//! run showed `seeded=false` forever on a healthy stream.

use hackrf_gnss::live::Engine;

/// Deterministic pseudo-noise (xorshift), interleaved int8 I/Q.
fn noise_bytes(n: usize, mut state: u64) -> Vec<u8> {
    (0..n)
        .map(|_| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 32) as u8
        })
        .collect()
}

#[test]
fn engine_decimation_reaches_bands() {
    let fs = 16.0e6;
    let mut eng = Engine::new(fs, 1_568_250_000.0, 0.0);
    // 4 s of stream in 512 KiB chunks, like the live_track reader thread
    let chunk = 512 * 1024;
    let total = chunk * ((4.0 * fs * 2.0) as usize / chunk);
    let mut off = 0;
    while off < total {
        let n = chunk.min(total - off);
        let buf = noise_bytes(n, off as u64 + 1);
        let _ = eng.push_i8(&buf);
        off += n;
    }
    // Seeding must have been ATTEMPTED (2 s of decimated data reached the
    // seeding stage). Pure noise acquires nothing, and a failed seed now
    // retries instead of latching `seeded`, so the flag itself stays false.
    assert!(
        eng.l1_band.seed_attempts > 0,
        "l1 band: decimated data never reached seeding"
    );
    assert!(
        eng.b1i_band.seed_attempts > 0,
        "b1i band: decimated data never reached seeding"
    );
}
