//! Doppler geolocation: solve for the receiver position and per-capture clock
//! errors from attributed Iridium burst carriers, given satellite TLEs and
//! absolute time. The inverse of iridium_ppm. Multiple captures share ONE
//! receiver position but each gets its own clock unknown e_k — arcs from
//! different satellites at different times fix the geometry a single short
//! arc cannot.
//!
//! usage:
//!   doppler_fix <tle> --cap file,dur_s,fs,bits,epoch_s [--cap ...] [--guess lat lon] [--no-doppler-attr] [--rate] [--include-undecoded] [--no-bias]
//!   Per-satellite frequency-bias states (ridge prior, sigma 2 kHz) are ON by
//!   default; --no-bias restores the plain solve.
//!   doppler_fix <tle> <capture.iq> <dur_s> <fs> [bits] <start_epoch_s> [lat0 lon0]   (legacy single-capture)
//!   epochs must be true transfer starts (a wrong epoch moves the fix, not
//!   just the match gate). Default guess 39.001 -77.60732.
use hackrf_gnss::gps::{geodetic_to_ecef, load_tle_named, predict_doppler_el_f};
use hackrf_gnss::iridium::geo::{doppler_attribute, solve_fix, Obs};
use hackrf_gnss::iridium::ppm::{estimate_ppm, median, ppm_from_burst, DecodedBurst, PpmEstimate};
use std::fs::File;
use std::io::Read;

struct Cap {
    path: String,
    dur: f64,
    fs: f64,
    bits: usize,
    epoch: f64,
}

fn usage() -> ! {
    eprintln!("usage: doppler_fix <tle> --cap file,dur_s,fs,bits,epoch_s [--cap ...] [--guess lat lon] [--no-doppler-attr] [--rate] [--include-undecoded] [--no-bias]");
    eprintln!("   or: doppler_fix <tle> <capture.iq> <dur_s> <fs> [bits] <start_epoch_s> [lat0 lon0]");
    std::process::exit(2);
}

fn parse_args(a: &[String]) -> (String, Vec<Cap>, Option<(f64, f64)>, bool, bool, bool, bool) {
    if a.len() < 3 {
        usage();
    }
    let tle = a[1].clone();
    let mut caps = Vec::new();
    let mut guess: Option<(f64, f64)> = None;
    let mut no_da = false;
    let mut no_bias = false;
    let mut rate_on = false;
    let mut incl_undec = false;
    if a[2] == "--cap" {
        let mut i = 2;
        while i < a.len() {
            match a[i].as_str() {
                "--cap" => {
                    let f: Vec<&str> = a[i + 1].split(',').collect();
                    if f.len() != 5 {
                        usage();
                    }
                    caps.push(Cap {
                        path: f[0].to_string(),
                        dur: f[1].parse().unwrap(),
                        fs: f[2].parse().unwrap(),
                        bits: f[3].parse().unwrap(),
                        epoch: f[4].parse().unwrap(),
                    });
                    i += 2;
                }
                "--guess" => {
                    guess = Some((a[i + 1].parse().unwrap(), a[i + 2].parse().unwrap()));
                    i += 3;
                }
                "--no-doppler-attr" => {
                    no_da = true;
                    i += 1;
                }
                "--no-bias" => {
                    no_bias = true;
                    i += 1;
                }
                "--rate" => {
                    rate_on = true;
                    i += 1;
                }
                "--include-undecoded" => {
                    incl_undec = true;
                    i += 1;
                }
                _ => usage(),
            }
        }
    } else {
        // legacy single-capture positional form
        let rest: Vec<&String> = a[2..].iter()
            .filter(|s| !s.starts_with("--"))
            .collect();
        no_da = a.iter().any(|s| s == "--no-doppler-attr");
        no_bias = a.iter().any(|s| s == "--no-bias");
        rate_on = a.iter().any(|s| s == "--rate");
        incl_undec = a.iter().any(|s| s == "--include-undecoded");
        if rest.len() < 4 {
            usage();
        }
        let mut idx = 4; // past file, dur, fs
        let mut bits = 8usize;
        if matches!(rest.get(idx).and_then(|s| s.parse::<usize>().ok()), Some(8 | 12 | 16)) {
            bits = rest[idx].parse().unwrap();
            idx += 1;
        }
        if idx >= rest.len() {
            eprintln!("start_epoch_s is required: a position fix needs true absolute time");
            std::process::exit(2);
        }
        caps.push(Cap {
            path: rest[0].clone(),
            dur: rest[1].parse().unwrap(),
            fs: rest[2].parse().unwrap(),
            bits,
            epoch: rest[idx].parse().unwrap(),
        });
        if rest.len() >= idx + 3 {
            guess = Some((rest[idx + 1].parse().unwrap(), rest[idx + 2].parse().unwrap()));
        }
    }
    if caps.is_empty() {
        usage();
    }
    (tle, caps, guess, no_da, no_bias, rate_on, incl_undec)
}

/// Front end for one capture: decode, attribute by payload position, then by
/// carrier offset. Returns the attributed observations and the bootstrap
/// clock estimate (ppm).
fn process_cap(
    cap: &Cap,
    ci: usize,
    sats: &[hackrf_gnss::gps::GpsSat],
    rx0: [f64; 3],
    no_da: bool,
    incl_undec: bool,
) -> (Vec<(String, f64, f64, f64, u32, f64)>, f64) {
    let fc = 1626.25e6;
    let mut f = File::open(&cap.path).unwrap();
    let bps = if cap.bits >= 12 { 2 } else { 1 };
    let want = ((cap.dur + 4.2) * cap.fs) as usize * 2 * bps;
    let mut raw_u8 = vec![0u8; want];
    // read() is not guaranteed to fill the buffer (short reads on multi-GB
    // files) — loop to EOF so long captures are fully processed.
    let mut got = 0usize;
    while got < want {
        match f.read(&mut raw_u8[got..]) {
            Ok(0) => break,
            Ok(n) => got += n,
            Err(_) => break,
        }
    }
    raw_u8.truncate(got);
    let est: PpmEstimate = if bps == 2 {
        let raw: Vec<i16> = raw_u8.chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]])).collect();
        estimate_ppm(&raw, fc, cap.fs, cap.dur, cap.epoch, sats, rx0)
    } else {
        let raw: Vec<i8> = raw_u8.iter().map(|&b| b as i8).collect();
        estimate_ppm(&raw, fc, cap.fs, cap.dur, cap.epoch, sats, rx0)
    };
    println!(
        "cap {} {}: detected {} decoded {} position-attributed {}",
        ci, cap.path, est.detected, est.decoded, est.per_burst.len()
    );
    if let Some(alt) = est.ambiguous_ppm {
        println!("  WARNING: channel-fold ambiguity: clock error also fits at {:+.2} ppm", alt);
    }
    if est.per_burst.iter().any(|b| b.epoch_shift_s != 0.0) {
        println!("  WARNING: epoch refinement shifted this capture's start; a position fix wants true time");
    }

    let shift = median(&est.per_burst.iter().map(|b| b.epoch_shift_s).collect::<Vec<_>>()).unwrap_or(0.0);
    // the anonymous pool: decoded-unattributed, plus detected-only on request
    let mut anon: Vec<DecodedBurst> = est.decoded_bursts.iter()
        .map(|b| DecodedBurst { t_epoch: b.t_epoch + shift, ..b.clone() })
        .collect();
    if incl_undec {
        let n0 = anon.len();
        anon.extend(est.detected_fine.iter()
            .map(|b| DecodedBurst { t_epoch: b.t_epoch + shift, ..b.clone() }));
        println!("  including {} detected-only bursts (fine carrier, no decode)", anon.len() - n0);
    }
    let (e0, e_tol, clock_known) = match est.ppm() {
        Some(p) => (p * 1e-6, 0.5e-6, true),
        None => {
            // No position anchor: the clock is unknown. +-30 ppm is 49 kHz at
            // L-band -- wider than the whole Doppler spread -- so a blanket
            // wide tolerance would "attribute" every burst to the nearest
            // prediction (observed: it silently poisoned the joint solve).
            // Instead scan e over +-30 ppm in 2 ppm steps at the NORMAL
            // tolerance: the true clock clusters attributions, wrong clocks
            // scatter them.
            let mut best = (0usize, 0.0f64);
            let mut eg = -30e-6;
            while eg <= 30.5e-6 {
                let n = doppler_attribute(&anon, sats, rx0, eg, 0.5e-6, 2.5e3).len();
                if n > best.0 {
                    best = (n, eg);
                }
                eg += 2e-6;
            }
            if best.0 < 3 {
                println!("  NOTE: no position anchor and no clock-scan clustering; capture left unattributed");
                (0.0, 0.5e-6, false)
            } else {
                println!("  NOTE: no position anchor; clock scan selected e = {:+.1} ppm ({} bursts cluster)",
                         best.1 * 1e6, best.0);
                (best.1, 0.5e-6, true)
            }
        }
    };
    // (sat name, t, f_meas, f_nom, conf, w_scale)
    let mut out: Vec<(String, f64, f64, f64, u32, f64)> = est.per_burst.iter()
        .map(|b| (b.sat.clone(), b.t_epoch, b.f_meas_hz, b.f_nom_hz, b.confidence, 1.0))
        .collect();
    let mut e_use = e0;
    if !no_da && !anon.is_empty() && clock_known {
        let mut attrs = doppler_attribute(&anon, sats, rx0, e0, e_tol, 2.5e3);
        if !attrs.is_empty() {
            // second pass: re-estimate e on the enlarged set, re-attribute once
            let mut es: Vec<f64> = est.per_burst.iter().map(|b| b.ppm).collect();
            for m in &attrs {
                if let Some(sat) = sats.iter().find(|s| s.name == m.sat) {
                    if let Some((fd, _)) = predict_doppler_el_f(sat, rx0, m.burst.t_epoch, m.f_nom_hz) {
                        es.push(ppm_from_burst(m.burst.f_meas_hz, m.f_nom_hz, fd));
                    }
                }
            }
            let e1 = median(&es).unwrap() * 1e-6;
            if (e1 - e0).abs() > 0.2e-6 {
                e_use = e1;
                attrs = doppler_attribute(&anon, sats, rx0, e1, 0.5e-6, 2.5e3);
            }
        }
        println!("  doppler-attributed {} of {} anonymous decoded bursts ({} ambiguous)",
                 attrs.len(), anon.len(), anon.len() - attrs.len());
        for m in &attrs {
            out.push((m.sat.clone(), m.burst.t_epoch, m.burst.f_meas_hz, m.f_nom_hz,
                      m.burst.confidence, m.w_scale));
        }
    }
    (out, e_use * 1e6)
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let (tle_path, caps, guess, no_da, no_bias, rate_on, incl_undec) = parse_args(&a);
    let text = match std::fs::read_to_string(&tle_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("cannot read TLE {}: {}", tle_path, e);
            std::process::exit(2);
        }
    };
    let sats = load_tle_named(&text);
    if sats.is_empty() {
        eprintln!("no satellites parsed from {} -- is it a TLE file?", tle_path);
        std::process::exit(2);
    }
    // the front end (elevation gate, Doppler attribution tolerance) needs A
    // reference position; the default station serves. The solver carries the
    // prior: with --guess it starts there, without it a global multi-start
    // grid runs and the front-end reference does not constrain the answer.
    let (g_lat, g_lon) = guess.unwrap_or((39.001, -77.60732));
    let rx0 = geodetic_to_ecef(g_lat, g_lon, 0.0);

    let mut obs: Vec<Obs> = Vec::new();
    let mut e0: Vec<f64> = Vec::new();
    for (ci, cap) in caps.iter().enumerate() {
        let (rows, e_ppm) = process_cap(cap, ci, &sats, rx0, no_da, incl_undec);
        e0.push(e_ppm * 1e-6);
        for (sat_name, t, f_meas, f_nom, conf, w) in rows {
            if let Some(sat) = sats.iter().find(|s| s.name == sat_name) {
                obs.push(Obs { t, f_meas, f_nom, sat, conf, w_scale: w, cap: ci });
            }
        }
    }
    println!("joint solve: {} observations over {} captures", obs.len(), caps.len());
    let bias = if no_bias { None } else { Some(2.0e3) };
    let rate = if rate_on { Some(100.0) } else { None };
    let fix = match guess {
        Some((gla, glo)) => solve_fix(&obs, gla, glo, &e0, bias, rate),
        None => {
            // no prior: sweep a global start grid, rank basins by RMS
            let ms = hackrf_gnss::iridium::geo::multistart_fix(
                &obs, &e0, bias, rate, &hackrf_gnss::iridium::geo::global_grid(),
            );
            let passing: Vec<_> = ms.candidates.iter().filter(|c| c.converged && c.sigma_km <= 10.0).collect();
            println!("multi-start: {} starts, {} converged, {} pass the sigma guard",
                     ms.candidates.len(),
                     ms.candidates.iter().filter(|c| c.converged).count(),
                     passing.len());
            for (i, c) in passing.iter().take(3).enumerate() {
                println!("  candidate {}: lat {:+.4} lon {:+.4}  rms {:.1} Hz  sigma {:.2} km",
                         i + 1, c.lat_deg, c.lon_deg, c.rms_hz, c.sigma_km);
            }
            if ms.ambiguous {
                println!("WARNING: ambiguity — runner-up basin within 2x of the winner's RMS;");
                println!("         the basins are not decisively separated, distrust the fix");
            }
            ms.best.ok_or_else(|| "no basin passed the sigma guard".to_string())
        }
    };
    match fix {
        Ok(fix) => {
            println!();
            println!("fix: lat {:+.5}  lon {:+.5}  alt 0 (constrained)", fix.lat_deg, fix.lon_deg);
            for (k, e) in fix.clock_ppm.iter().enumerate() {
                println!("clock error cap {}: {:+.2} ppm  (positive = clock FAST)", k, e);
            }
            println!("residual RMS {:.1} Hz over {} bursts, {} LM iterations", fix.rms_hz, fix.n_used, fix.iters);
            println!("formal 1-sigma horizontal: {:.2} km  (DOP {:.1} km per kHz residual)", fix.sigma_km, fix.dop);
            println!("per-satellite:");
            for (name, n, rms, bias, rate) in &fix.per_sat {
                println!("  {:<18} n={:<3} resid RMS {:6.1} Hz  bias {:+7.0} Hz  drift {:+6.1} Hz/s", name, n, rms, bias, rate);
            }
        }
        Err(m) => {
            println!("no fix: {m}");
            std::process::exit(1);
        }
    }
}
