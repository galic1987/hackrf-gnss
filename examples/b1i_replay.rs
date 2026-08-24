//! Offline B1I D1 validation: replay a wideband capture through the live
//! Engine and report BeiDou nav decoding (NH sync, D1 frames, SOW anchors)
//! plus GPS LNAV anchors for cross-checking (BDS SOW + 14 s == GPS TOW at
//! the same instant). Feeds the engine SYNCHRONOUSLY and pauses while an
//! acquisition worker runs — a file replay must not drop input.
//!
//! usage: b1i_replay <iq> <fs> <fc>   e.g.
//!   b1i_replay wideband_l1_b1.iq 20e6 1568.259e6

use hackrf_gnss::live::Engine;
use std::io::Read;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let path = a.get(1).map(|s| s.as_str()).unwrap_or("wideband_l1_b1.iq");
    let fs: f64 = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(20.0e6);
    let fc: f64 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(1_568_259_000.0);
    let epoch0 = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();
    let mut eng = Engine::new(fs, fc, epoch0);
    eprintln!("b1i_replay: {path} fs {fs:.3e} fc {fc:.6e}");

    let mut f = std::fs::File::open(path).expect("open capture");
    let mut buf = vec![0u8; 4 * 1024 * 1024];
    let mut fed = 0u64;
    loop {
        let n = f.read(&mut buf).unwrap_or(0);
        if n == 0 {
            break;
        }
        let reports = eng.push_i8(&buf[..n]);
        fed += n as u64;
        for r in &reports {
            if r.sys == "beidou" || r.rho_m.is_some() {
                println!(
                    "{:6.1}s {:7} {:3} cn0 {:5.1} lock {:4.0} bits {:6} subs {:2} rho {} t_tx {}",
                    fed as f64 / 2.0 / fs,
                    r.sys,
                    r.prn,
                    r.cn0_proxy,
                    r.lock_s,
                    r.nav_bits,
                    r.nav_subs,
                    r.rho_m.map(|v| format!("{v:.0}")).unwrap_or_else(|| "-".into()),
                    r.t_tx.map(|v| format!("{v:.0}")).unwrap_or_else(|| "-".into()),
                );
            }
        }
        // a file replay must wait out acquisition workers, not drop input
        while eng.l1_band.worker_active() || eng.b1i_band.worker_active() {
            eng.l1_band.poll();
            eng.b1i_band.poll();
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        eng.l1_band.poll();
        eng.b1i_band.poll();
    }
    eprintln!("b1i_replay: EOF after {:.1} s of stream", fed as f64 / 2.0 / fs);
    // final channel summary; for BeiDou, dump any D1 frame candidates
    // (unpaired ones too — a short capture can't form a consistent pair)
    for band in [&eng.l1_band, &eng.b1i_band] {
        for ch in &band.channels {
            eprintln!(
                "  {} {:7} PRN {:3}: locked={} bits={} anchor={:?} eph={}",
                band.name,
                ch.sys.name(),
                ch.prn,
                ch.locked,
                ch.nav_bits.len(),
                ch.anchor,
                ch.eph.is_some()
            );
            if ch.sys == hackrf_gnss::live::Sys::Beidou && ch.nav_bits.len() >= 300 {
                for sf in hackrf_gnss::beidou_d1::find_candidates(&ch.nav_bits) {
                    eprintln!(
                        "    D1 candidate: bit {} frid {} sow_bdt {} (= GPST {:.0}) | {}",
                        sf.bit_index,
                        sf.frid,
                        sf.sow_bdt,
                        hackrf_gnss::beidou_d1::sow_bdt_to_gpst(sf.sow_bdt as f64),
                        sf.debug_fields()
                    );
                }
            }
        }
    }
}
