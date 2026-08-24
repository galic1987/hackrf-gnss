//! GLONASS L1OF nav-string decoder: acquire the strongest FDMA channel, demod
//! the Manchester nav data block-wise, decode strings live.
//!
//! usage: glonass_nav <iq> <fs> <fc> <secs>
//!
//! Chain: FDMA acquisition (as in glonass_acq) -> fine 25 Hz re-acquisition ->
//! block-wise demod (code-wipe + 10 ms integrate + squared-spectrum carrier
//! track; chosen over a closed DLL/PLL because at the indoor C/N0 ~ 30 dB-Hz
//! seen from this station the 10 Hz Costas loop would not converge from a
//! 250 Hz acquisition grid, while the block-wise path needs no pull-in) ->
//! 10 ms chip sync via the 30-chip time mark -> Manchester + relative decode
//! -> Hamming verification -> strings 1-4 ephemeris assembly.
use hackrf_gnss::glonass::{glonass_code, l1_freq, CHIP_RATE};
use hackrf_gnss::glonass_nav::{
    blockwise_chips, decode_strings, vote_strings, EphCollector, Hamming, StringData,
};
use hackrf_gnss::gps::acquire_codes;
use hackrf_gnss::iridium::demod3::decimate;
use num_complex::Complex;
use std::f64::consts::PI;
use std::fs::File;
use std::io::Read;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let fs: f64 = a[2].parse().unwrap();
    let fc: f64 = a[3].parse().unwrap();
    let secs: f64 = a.get(4).and_then(|s| s.parse().ok()).unwrap_or(60.0);
    let mut f = File::open(&a[1]).unwrap();
    let mut raw = vec![0u8; (2.0 * secs * fs) as usize * 2];
    let n = f.read(&mut raw).unwrap();
    raw.truncate(n);
    let base: Vec<Complex<f32>> = raw
        .chunks_exact(2)
        .map(|c| Complex::new(c[0] as i8 as f32, c[1] as i8 as f32))
        .collect();
    drop(raw);
    let mean: Complex<f32> = base.iter().copied().sum::<Complex<f32>>() / base.len() as f32;
    let q = (fs / 2.0e6).round() as usize; // decimate to ~2 Msps
    let fsd = fs / q as f64;
    let code = glonass_code();
    let dop: Vec<f64> = (-10000..=10000).step_by(250).map(|x| x as f64).collect();

    // ---- FDMA acquisition on the first 2 s (validated pattern from glonass_acq)
    let acq_samps = (2.0 * secs.min(2.0) * fs) as usize;
    let acq_base = &base[..acq_samps.min(base.len())];
    let mut best: Option<(i32, f32, f64, f64)> = None; // (k, metric, doppler, code_phase)
    println!("GLONASS L1OF FDMA search (first 2 s):");
    for k in -7..=6 {
        let ifhz = l1_freq(k) - fc;
        let mut sig: Vec<Complex<f32>> = acq_base
            .iter()
            .enumerate()
            .map(|(i, &v)| {
                let ph = -2.0 * PI * ifhz * i as f64 / fs;
                (v - mean) * Complex::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        sig = decimate(&sig, q);
        let r = acquire_codes(&sig, fsd, &[(0usize, code.clone())], CHIP_RATE, &dop, 2000, 2.5, l1_freq(k), true);
        let m = r[0].metric;
        println!(
            "  chan {:+2} ({:.4} MHz):  metric {:5.2}  dopp {:+6.0}  {}",
            k, l1_freq(k) / 1e6, m, r[0].doppler,
            if m > 2.5 { "<== SATELLITE" } else { "" }
        );
        if m > 2.5 && best.as_ref().map_or(true, |b| m > b.1) {
            best = Some((k, m, r[0].doppler, r[0].code_phase));
        }
    }
    let Some((k, _, dopp0, _code_phase0)) = best else {
        println!("=> no GLONASS satellite acquired; nothing to track");
        return;
    };
    println!("=> tracking channel {k:+} ({:.4} MHz), dopp {dopp0:+.0} Hz", l1_freq(k) / 1e6);

    // ---- mix the chosen channel to baseband over the whole capture
    let ifhz = l1_freq(k) - fc;
    let mut sig: Vec<Complex<f32>> = base
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            let ph = -2.0 * PI * ifhz * i as f64 / fs;
            (v - mean) * Complex::new(ph.cos() as f32, ph.sin() as f32)
        })
        .collect();
    drop(base);
    sig = decimate(&sig, q);

    // ---- fine Doppler: the 250 Hz acquisition grid is far too coarse for the
    // squared-spectrum carrier track (it aliases above ~25 Hz residual), so
    // re-acquire on a 25 Hz grid over the first 4 s.
    let fine: Vec<f64> = (-5..=5).map(|i| dopp0 + i as f64 * 25.0).collect();
    let r = acquire_codes(
        &sig[..(4.0 * fsd) as usize],
        fsd,
        &[(0usize, code.clone())],
        CHIP_RATE,
        &fine,
        4000,
        2.0,
        l1_freq(k),
        true,
    );
    let dopp_fine = r[0].doppler;
    let cp = r[0].code_phase;
    println!("fine acq: metric {:.2}  dopp {dopp_fine:+.0}  code_phase {cp:.1}", r[0].metric);

    // ---- block-wise demod: code-wipe + 10 ms integrate + squared-spectrum
    // carrier track + time-mark pilot phase refinement (see glonass_nav
    // module docs for why not a closed loop).
    let Some((chips, sync)) = blockwise_chips(&sig, fsd, l1_freq(k), dopp_fine, cp) else {
        println!("=> block-wise demod failed (no carrier track / no time-mark sync)");
        return;
    };
    drop(sig);
    let strs = decode_strings(&chips, &sync);
    let nblocks = (chips.len() - sync.offset) / 200;
    println!(
        "== time-mark sync: offset {} chips, score {:.3}, polarity {:+}; {} of {} strings Hamming-valid ==",
        sync.offset, sync.score, sync.polarity, strs.len(), nblocks
    );
    // cross-frame soft vote: strings repeat every 15 blocks (30 s frame);
    // voting two frames doubles the effective chip SNR. String 1 is excluded
    // here (its tk field advances each frame) and decoded per frame below.
    let voted = vote_strings(&chips, &sync, 15);
    println!("== cross-frame vote: {} of 15 string classes Hamming-valid ==", voted.len());

    // ---- print decoded strings: per-frame first, then the cross-frame vote
    let print_str = |blk: usize, s: &hackrf_gnss::glonass_nav::GloString| {
        let ham = match s.hamming {
            Hamming::Ok => "ok".to_string(),
            Hamming::OkCheckBit => "ok(check-bit err)".to_string(),
            Hamming::Corrected(p) => format!("ok(corrected bit {p})"),
            Hamming::Fail => unreachable!(),
        };
        let t = blk * 2; // seconds into the capture
        match &s.data {
            StringData::S1 { p1, tk_s, x_m, vx_ms, ax_ms2 } => {
                println!(
                    "[{t:3}s] str {:2}  {ham}  tk={:02}:{:02}:{:02} P1={p1}  x={x_m:+.1} m vx={vx_ms:+.3} m/s ax={ax_ms2:+.3e} m/s2",
                    s.str_num,
                    (*tk_s / 3600.0) as u32,
                    ((*tk_s % 3600.0) / 60.0) as u32,
                    *tk_s % 60.0
                );
            }
            StringData::S2 { bn, tb_min, y_m, vy_ms, ay_ms2, .. } => {
                println!(
                    "[{t:3}s] str {:2}  {ham}  tb={tb_min} min ({:02}:{:02} UTC(SU)+3h) Bn={bn}  y={y_m:+.1} m vy={vy_ms:+.3} m/s ay={ay_ms2:+.3e} m/s2",
                    s.str_num, tb_min / 60, tb_min % 60
                );
            }
            StringData::S3 { gamma_n, ln, z_m, vz_ms, az_ms2, .. } => {
                println!(
                    "[{t:3}s] str {:2}  {ham}  ln={ln} gamma_n={gamma_n:+.3e}  z={z_m:+.1} m vz={vz_ms:+.3} m/s az={az_ms2:+.3e} m/s2",
                    s.str_num
                );
            }
            StringData::S4 { tau_n_s, en, ft, nt, slot, m_type, .. } => {
                println!(
                    "[{t:3}s] str {:2}  {ham}  slot n={slot} NT={nt} M={m_type} En={en} FT={ft} tau_n={tau_n_s:+.3e} s",
                    s.str_num
                );
            }
            StringData::S5 { na, tau_c_s, n4, .. } => {
                println!("[{t:3}s] str {:2}  {ham}  NA={na} N4={n4} tau_c={tau_c_s:+.6e} s", s.str_num);
            }
            StringData::Almanac => {
                println!("[{t:3}s] str {:2}  {ham}  (almanac)", s.str_num);
            }
        }
    };
    println!("-- per-frame strings:");
    for (blk, s) in &strs {
        print_str(*blk, s);
    }
    println!("-- cross-frame voted strings:");
    for (res, s) in &voted {
        print_str(*res, s);
    }

    // ---- ephemeris: strings 2-4 from the vote (content constant within a tb
    // interval), string 1 from per-frame decodes (tk advances 30 s per frame;
    // accept the frame whose tk is consistent with the previous frame's).
    let mut col = EphCollector::default();
    for (_, s) in &voted {
        col.push(s);
    }
    // string-1 candidates, in capture order
    let s1s: Vec<&hackrf_gnss::glonass_nav::GloString> =
        strs.iter().filter(|(_, s)| s.str_num == 1).map(|(_, s)| s).collect();
    for (i, s) in s1s.iter().enumerate() {
        if let StringData::S1 { tk_s, x_m, vx_ms, ax_ms2, .. } = s.data {
            // consistency: tk steps +30 s per frame; x/vx/ax must repeat
            let ok = if i == 0 {
                true
            } else if let StringData::S1 { tk_s: pt, x_m: px, vx_ms: pv, ax_ms2: pa, .. } = s1s[i - 1].data {
                (tk_s - pt - 30.0).abs() < 1.0
                    && (x_m - px).abs() < 1.0
                    && (vx_ms - pv).abs() < 1e-2
                    && (ax_ms2 - pa).abs() < 1e-4
            } else {
                false
            };
            if ok {
                col.push(s);
            } else {
                println!("-- rejecting string 1 at frame {} (tk/pos inconsistent with previous frame)", i + 1);
            }
        }
    }
    match col.ephemeris() {
        Some(e) => {
            println!("== EPHEMERIS (PZ-90.02, strings 1-4) ==");
            println!("  slot {}  NT {}  tb {} min ({:02}:{:02} UTC(SU)+3h)  tk {:.0} s", e.slot, e.nt, e.tb_min, e.tb_min / 60, e.tb_min % 60, e.tk_s);
            println!("  pos  [{:+.3}, {:+.3}, {:+.3}] km   |r| = {:.1} km",
                e.pos_m[0] / 1e3, e.pos_m[1] / 1e3, e.pos_m[2] / 1e3, e.radius_m() / 1e3);
            println!("  vel  [{:+.4}, {:+.4}, {:+.4}] km/s |v| = {:.4} km/s",
                e.vel_ms[0] / 1e3, e.vel_ms[1] / 1e3, e.vel_ms[2] / 1e3, e.speed_ms() / 1e3);
            println!("  acc  [{:+.3e}, {:+.3e}, {:+.3e}] m/s2 (Sun+Moon)", e.acc_ms2[0], e.acc_ms2[1], e.acc_ms2[2]);
            println!("  gamma_n {:+.4e}  tau_n {:+.4e} s  En {}  FT {}", e.gamma_n, e.tau_n_s, e.en, e.ft);
        }
        None => println!("=> no complete ephemeris (need Hamming-valid strings 1-4 of one frame)"),
    }
}
