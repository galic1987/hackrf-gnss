//! Front-end throughput probe: feeds N seconds of int8 noise through
//! Engine::push_i8 with seeding effectively disabled (noise never acquires,
//! but seed attempts still burn CPU — so we measure the STEADY-STATE rate
//! from the tail: bytes processed after the last seed attempt started).
//! Not a pass/fail test; prints MB/s so `cargo test --release --test
//! frontend_rate -- --nocapture` shows whether the consumer beats 32 MB/s.

use hackrf_gnss::live::Engine;
use std::time::Instant;

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
fn frontend_steady_state_rate() {
    let fs = 16.0e6_f64;
    let mut eng = Engine::new(fs, 1_568_250_000.0, 0.0);
    let chunk = 512 * 1024usize;
    // 1 s of stream per iteration; 3 s total: sec 0 is seed-free (buffer
    // filling), the first seed burst lands inside sec 1, sec 2 refills.
    let mut best_mbps = 0.0f64;
    for sec in 0..3u64 {
        let bytes = (fs * 2.0) as usize;
        let buf = noise_bytes(bytes, sec * 7919 + 1);
        let t0 = Instant::now();
        let mut off = 0;
        while off < buf.len() {
            let n = chunk.min(buf.len() - off);
            let _ = eng.push_i8(&buf[off..off + n]);
            off += n;
        }
        let dt = t0.elapsed().as_secs_f64();
        let attempts = eng.l1_band.seed_attempts + eng.b1i_band.seed_attempts;
        eprintln!(
            "sec {}: {:.2} s wall ({:.1} MB/s), seed_attempts {}",
            sec,
            dt,
            bytes as f64 / dt / 1e6,
            attempts
        );
        if dt < 0.9 {
            // seed-free second (a seed burst takes far longer than 0.9 s)
            best_mbps = best_mbps.max(bytes as f64 / dt / 1e6);
        }
    }
    eprintln!("steady-state front-end rate: {:.1} MB/s (need > 32)", best_mbps);
    assert!(best_mbps > 0.0);
}
