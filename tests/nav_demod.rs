//! Nav demod validation: track the real L1 fixture (30 s of live sky with
//! GPS + WAAS + Galileo), slice 50 bps LNAV bits off the locked GPS
//! channels, and require at least one parity-valid LNAV subframe.

use hackrf_gnss::gps::lnav::find_subframes;
use hackrf_gnss::live::Band;
use num_complex::Complex;

const FS: f64 = 4.0e6;

fn load_fixture(secs: usize) -> Option<Vec<Complex<f32>>> {
    use std::io::Read;
    let path = format!("{}/scripts/l1_30s_bb.f32", env!("CARGO_MANIFEST_DIR"));
    let mut f = std::fs::File::open(path).ok()?;
    let want = (FS as usize) * secs * 8;
    let mut bytes = vec![0u8; want];
    let n = f.read(&mut bytes).ok()?;
    bytes.truncate(n);
    Some(
        bytes
            .chunks_exact(8)
            .map(|c| {
                Complex::new(
                    f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                    f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
                )
            })
            .collect(),
    )
}

#[test]
fn nav_bits_decode_subframes_from_real_sky() {
    let Some(sig) = load_fixture(28) else {
        eprintln!("fixture missing — skipping");
        return;
    };
    let mut band = Band::new_l1(FS, 0.0);
    let sec = FS as usize;
    for chunk in sig.chunks(sec) {
        let _ = band.push(chunk);
        while band.worker_active() {
            band.poll();
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        band.poll();
    }
    let mut total_bits = 0;
    let mut total_subs = 0;
    for ch in &band.channels {
        if ch.sys.name() != "gps" {
            continue;
        }
        let (nt, hist) = ch.debug_nav_hist();
        eprintln!("  PRN {} hist: ntrans={} {:?}", ch.prn, nt, hist);
        let subs = find_subframes(&ch.nav_bits);
        eprintln!(
            "PRN {}: {} nav bits, {} valid subframes (lock {:.0} s)",
            ch.prn,
            ch.nav_bits.len(),
            subs.len(),
            ch.lock_s
        );
        if ch.nav_bits.len() > 320 && total_subs == 0 {
            let bits: String = ch.nav_bits[..1200.min(ch.nav_bits.len())].iter().map(|b| (b'0' + b) as char).collect();
            eprintln!("  PRN {} bits[0..320]: {}", ch.prn, bits);
        }
        total_bits += ch.nav_bits.len();
        total_subs += subs.len();
    }
    assert!(total_bits > 600, "only {total_bits} nav bits decoded");
    assert!(total_subs >= 1, "no parity-valid LNAV subframes decoded");
}
