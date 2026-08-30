//! Live 1 Hz multi-constellation tracker front end.
//!
//! Reads interleaved int8 I/Q from stdin (the python wrapper pipes the
//! hackrf_transfer FIFO straight in), tracks every acquired GPS / SBAS /
//! Galileo E1B / BeiDou B1I satellite, and prints one JSON line per tracked
//! PRN per second to stdout:
//!   {"prn":26,"sys":"gps","doppler_hz":...,"cn0_proxy":...,"code_phase":...,
//!    "lock_s":...,"epoch":...}
//! Logs go to stderr. usage: live_track [fs] [fc_hz]
//! (defaults: 16 Msps @ 1568.25 MHz — spans BeiDou B1I .. GPS L1).
//!
//! The reader thread does NOTHING but drain stdin into an unbounded channel:
//! hackrf_transfer exits if its writer stalls for >1 s (the phase_producer.py
//! lesson), so acquisition and tracking run here, on the main thread, and the
//! reader never waits on them.

use hackrf_gnss::live::Engine;
use std::io::{self, BufWriter, Read, Write};
use std::time::{SystemTime, UNIX_EPOCH};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let fs: f64 = a.get(1).and_then(|s| s.parse().ok()).unwrap_or(16.0e6);
    let fc: f64 = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(1_568_250_000.0);
    let epoch0 = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0);
    eprintln!("live_track: fs {:.3} Msps, fc {:.3} MHz, epoch0 {:.0}", fs / 1e6, fc / 1e6, epoch0);

    // reader thread: drain stdin, never block on the worker. BOUNDED channel:
    // the worker falls behind during the seeding acquisition (minutes of CPU),
    // and an unbounded queue turned that into an 11.5 GB / 7-min-stale memory
    // bomb on the first live run. When full, drop the NEWEST chunk — the
    // tracker must ride the real-time edge, not work through a backlog.
    let (tx, rx) = crossbeam_channel::bounded::<Vec<u8>>(128); // ~2 s of stream
    // Dropped BYTES, not chunks: the engine converts this to stream time so
    // code-phase extrapolation stays correct across gaps (a samples-processed
    // clock would under-count by exactly the dropped time and misalign every
    // subsequently installed channel).
    let drops = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
    let drops_r = drops.clone();
    std::thread::spawn(move || {
        let mut stdin = io::stdin();
        let mut buf = vec![0u8; 64 * 1024];
        // Batch tiny FIFO reads into 512 KiB chunks before handing off:
        // macOS FIFOs deliver ~16 KiB per read, and each chunk costs the
        // consumer a rayon join + several allocations — 2000 such events/s
        // put the consumer at only ~0.9x line rate with channels tracking
        // (queue overflowed every ~90 s, killing locks). Batching cuts the
        // per-chunk overhead 30x; at 32 MB/s a batch fills in ~16 ms.
        let mut acc: Vec<u8> = Vec::with_capacity(512 * 1024);
        loop {
            match stdin.read(&mut buf) {
                Ok(0) => break, // EOF
                Ok(n) => {
                    acc.extend_from_slice(&buf[..n]);
                    if acc.len() >= 512 * 1024 {
                        let full =
                            std::mem::replace(&mut acc, Vec::with_capacity(512 * 1024));
                        let l = full.len() as u64;
                        if tx.try_send(full).is_err() {
                            drops_r.fetch_add(l, std::sync::atomic::Ordering::Relaxed);
                        }
                    }
                }
                Err(_) => break,
            }
        }
        if !acc.is_empty() {
            let l = acc.len() as u64;
            if tx.try_send(acc).is_err() {
                drops_r.fetch_add(l, std::sync::atomic::Ordering::Relaxed);
            }
        }
    });

    let mut eng = Engine::new(fs, fc, epoch0);
    // Seed cache: skip the minutes-long blind all-sky seed after a restart.
    // Candidates fresher than 10 min go straight to the fast align phase
    // (the align grid ±150 Hz absorbs the Doppler drift over that window).
    const CACHE: &str = "/Volumes/Radiator 8TB/gnss/observations/tracker_seed_cache.json";
    if let Ok(text) = std::fs::read_to_string(CACHE) {
        if let Ok(v) = serde_json::from_str::<serde_json::Value>(&text) {
            let fresh = v["epoch"].as_f64().map(|e| epoch0 - e < 600.0).unwrap_or(false);
            if fresh {
                let mut l1 = Vec::new();
                let mut b1i = Vec::new();
                for s in v["sats"].as_array().into_iter().flatten() {
                    let (Some(sys), Some(prn), Some(dopp)) = (
                        s["sys"].as_str().and_then(hackrf_gnss::live::Sys::from_name),
                        s["prn"].as_u64().map(|p| p as usize),
                        s["dopp"].as_f64(),
                    ) else { continue };
                    match sys {
                        hackrf_gnss::live::Sys::Beidou => b1i.push((sys, prn, dopp)),
                        _ => l1.push((sys, prn, dopp)),
                    }
                }
                eprintln!("live_track: seed cache — {} L1 + {} B1I candidates, skipping blind seed", l1.len(), b1i.len());
                eng.l1_band.prime_candidates(l1);
                eng.b1i_band.prime_candidates(b1i);
            }
        }
    }
    let mut last_cache_write = 0.0f64;
    let stdout = io::stdout();
    let mut out = BufWriter::new(stdout.lock());
    // Discard the first second of stream: transfer startup (PLL settle, gain
    // ramp, DC transient) poisoned the one-shot seeding acquisition on the
    // first live run — it seeded on junk, found nothing, and never retried.
    let mut skip_bytes = (fs * 2.0) as u64; // 1 s of int8 I/Q
    let mut bytes_in = 0u64;
    let mut last_log = 0u64;
    let mut last_drops = 0u64;
    // Report epochs are stamped at EMISSION, not from the engine's t_s: the
    // bounded queue drops input whenever the engine falls behind (seeding
    // bursts), so processed-sample time diverges arbitrarily from wall time —
    // and the producer was discarding every report as "silent > 10 s".
    let emit = |reports: &mut Vec<hackrf_gnss::live::SatReport>,
                out: &mut BufWriter<io::StdoutLock>| {
        if reports.is_empty() {
            return;
        }
        for r in reports.iter() {
            if let Ok(line) = serde_json::to_string(r) {
                let _ = writeln!(out, "{}", line);
            }
        }
        let _ = out.flush();
    };
    while let Ok(chunk) = rx.recv() {
        if skip_bytes > 0 {
            let n = skip_bytes.min(chunk.len() as u64) as usize;
            skip_bytes -= n as u64;
            if n == chunk.len() {
                continue;
            }
            bytes_in += (chunk.len() - n) as u64;
            let mut reports = eng.push_i8(&chunk[n..]);
            emit(&mut reports, &mut out);
            continue;
        }
        bytes_in += chunk.len() as u64;
        let t0 = std::time::Instant::now();
        let mut reports = eng.push_i8(&chunk);
        let slow = t0.elapsed();
        if slow > std::time::Duration::from_millis(200) {
            eprintln!(
                "live_track: SLOW push_i8 {:.0} ms (queue {})",
                slow.as_secs_f64() * 1e3,
                rx.len()
            );
        }
        emit(&mut reports, &mut out);
        // Gap detector: if the reader dropped chunks (queue overflowed while
        // we were busy), everything queued is stale — jump to the live edge,
        // account the skipped bytes as STREAM time (phase extrapolation),
        // and re-align channels on fresh data (their phases died at the gap).
        let d = drops.load(std::sync::atomic::Ordering::Relaxed);
        if d != last_drops {
            let mut gap_bytes = d - last_drops;
            last_drops = d;
            while let Ok(stale) = rx.try_recv() {
                gap_bytes += stale.len() as u64; // drained backlog: also a gap
            }
            eng.note_gap_bytes(gap_bytes);
            if eng.l1_band.seeded() || eng.b1i_band.seeded() {
                eng.request_reseed();
                eprintln!("live_track: input gap — realigning channels at the edge");
            }
        }
        // persist the seed cache every 30 s while anything is tracked
        let now_wall = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0);
        if now_wall - last_cache_write > 30.0
            && (!eng.l1_band.channels.is_empty() || !eng.b1i_band.channels.is_empty())
        {
            last_cache_write = now_wall;
            let mut sats = Vec::new();
            for (sys, prn, dopp) in eng
                .l1_band
                .candidates_snapshot()
                .into_iter()
                .chain(eng.b1i_band.candidates_snapshot())
            {
                sats.push(serde_json::json!({"sys": sys.name(), "prn": prn, "dopp": dopp}));
            }
            let doc = serde_json::json!({"epoch": now_wall, "sats": sats});
            let tmp = format!("{CACHE}.tmp");
            if std::fs::write(&tmp, doc.to_string()).is_ok() {
                let _ = std::fs::rename(&tmp, CACHE);
            }
        }
        // ~2 s of stream between status lines; depth = backpressure.
        // (Byte-based: FIFO/pipe reads are partial, so counting CHUNKS
        // inflated this label ~25x on the first live run.)
        if bytes_in / (2 * fs as u64 * 2) != last_log {
            last_log = bytes_in / (2 * fs as u64 * 2);
            eprintln!(
                "live_track: {:.1} s in, l1 {} chans [{}], b1i {} chans [{}], queue {}",
                bytes_in as f64 / 2.0 / fs,
                eng.l1_band.channels.len(),
                eng.l1_band.status(),
                eng.b1i_band.channels.len(),
                eng.b1i_band.status(),
                rx.len()
            );
        }
    }
    let _ = out.flush();
    eprintln!("live_track: stdin EOF — exiting");
}
