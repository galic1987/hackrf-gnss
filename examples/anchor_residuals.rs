//! Per-satellite anchor residual diagnostic — the truth table for the
//! anchored-PVT pipeline, computed WITHOUT the solver.
//!
//! For every TOW-anchored channel in the live tracker state:
//!     resid_i = rho_i - |r_surveyed - sat_i(E_i)| - clock
//! where E_i = t_tx_i - dt_sv is the GPS emission time (reception-anchored
//! light-time solve) and clock = median over channels of (rho_i - geometric
//! range) — the common-mode stream-time offset the solver's clock unknown
//! would absorb.
//! What remains is per-channel anchor error, and its magnitude decodes the
//! bug class instantly:
//!     ~5995 km  -> 20 ms nav-bit slip (one C/A code period of 20 ms is a bit)
//!     ~299.8 km -> 1 ms code-period ambiguity
//!     ~tens of km -> ~100 us fractional code-phase misreference
//!     ~meters   -> a healthy anchor
//!
//! Gate for the mixed GPS+BDS work: GPS residuals at meter class against
//! the survey BEFORE a second constellation joins — otherwise mixed solves
//! bisect twice as hard.
//!
//! usage: anchor_residuals   (reads live state; no radio access)

use hackrf_gnss::beidou_d1::{parse_rinex_bds, sat_at_txtime_bds};
use hackrf_gnss::gps::broadcast::BrdcEph;

const TRACKER_STATE: &str = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json";
const EPH: &str = "/Volumes/Radiator 8TB/gnss/observations/tracker_eph.json";
const APPROX_LLA: [f64; 3] = [39.0032, -77.6058, 20.0];
const C_KM_S: f64 = 299_792.458;

fn main() {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();

    let mut ephs: std::collections::HashMap<u8, BrdcEph> = Default::default();
    let mut bds_ephs: std::collections::HashMap<u8, BrdcEph> = Default::default();
    if let Ok(t) = std::fs::read_to_string(EPH) {
        if let Ok(v) = serde_json::from_str::<serde_json::Value>(&t) {
            for e in v["ephemeris"].as_array().into_iter().flatten() {
                if let Ok(eph) = serde_json::from_value::<BrdcEph>(e.clone()) {
                    if eph.sys == 1 {
                        bds_ephs.insert(eph.prn, eph);
                    } else if eph.sys == 0 {
                        ephs.insert(eph.prn, eph);
                    }
                }
            }
        }
    }
    // BRDC fills whatever self-decoded doesn't cover
    if let Ok(t) = std::fs::read_to_string(
        "/Volumes/Radiator 8TB/gnss/observations/brdc_latest.rnx",
    ) {
        for (prn, eph) in hackrf_gnss::gps::broadcast::parse_rinex_gps(&t) {
            ephs.entry(prn).or_insert(eph);
        }
        for (prn, eph) in parse_rinex_bds(&t) {
            bds_ephs.entry(prn).or_insert(eph);
        }
    }

    let text = std::fs::read_to_string(TRACKER_STATE).expect("tracker state");
    let v: serde_json::Value = serde_json::from_str(&text).unwrap();
    // Site reference priority: SITE_LL env > latest GATED published fix
    // (mobile station: the reference IS the current fix, not a constant —
    // the old hardcoded site made every residual a lie after relocation,
    // and the station is going in a car) > APPROX_LLA cold-start fallback.
    let fix_ref: Option<[f64; 3]> = std::fs::read_to_string(
        "/Volumes/Radiator 8TB/gnss/observations/state.position.json",
    )
    .ok()
    .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
    .and_then(|p| {
        let pos = &p["position"];
        let fresh = now - p["epoch"].as_f64().unwrap_or(0.0) < 900.0;
        let gated = pos["gate"].as_str().unwrap_or("") == "redundant";
        if fresh && gated {
            Some([
                pos["lat"].as_f64()?,
                pos["lon"].as_f64()?,
                pos["alt_km"].as_f64()? * 1000.0,
            ])
        } else {
            None
        }
    });
    let lla: [f64; 3] = std::env::var("SITE_LL")
        .ok()
        .and_then(|s| {
            let p: Vec<f64> = s.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            if p.len() >= 2 {
                Some([p[0], p[1], *p.get(2).unwrap_or(&20.0)])
            } else {
                None
            }
        })
        .or(fix_ref)
        .unwrap_or(APPROX_LLA);
    let site = hackrf_gnss::gps::ephemeris::geodetic_to_ecef(
        lla[0], lla[1], lla[2] / 1000.0,
    ); // km
    println!(
        "site: {:.5}, {:.5} ({})",
        lla[0],
        lla[1],
        if std::env::var("SITE_LL").is_ok() {
            "env"
        } else if fix_ref.is_some() {
            "latest gated fix"
        } else {
            "fallback constant"
        }
    );

    // (prn, raw residual in km before clock removal)
    let mut raw: Vec<(u8, f64)> = Vec::new();
    for s in v["tracker"]["sats"].as_array().into_iter().flatten() {
        if s["sys"].as_str() != Some("gps") {
            continue;
        }
        if now - s["epoch"].as_f64().unwrap_or(0.0) > 10.0 {
            continue;
        }
        let (Some(prn), Some(rho_m), Some(t_tx)) = (
            s["prn"].as_u64().map(|p| p as u8),
            s["rho_m"].as_f64(),
            s["t_tx"].as_f64(),
        ) else {
            continue;
        };
        let Some(eph) = ephs.get(&prn) else {
            println!("PRN {prn:2}: anchored but NO ephemeris — skipped");
            continue;
        };
        // Reception-anchored light-time solve. sat_at_txtime's tow is a GPS
        // RECEPTION time; calling it with the SV-clock transmit time (the
        // pre-2026-08-25 code) evaluates the satellite ~tau ~ 70 ms away
        // from emission, leaving a per-channel range-rate * 70 ms artifact
        // of up to ~60 m in this table (the "fractional code-phase suspect"
        // class has a truth-side component). Iterate: A = t_tx - dt_sv +
        // tau converges in two passes to ps class.
        let site_m = site.map(|x| x * 1000.0);
        let (_, dt0, _) =
            hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, t_tx, site_m);
        let mut a = t_tx - dt0 + 0.075;
        let mut dt_sv = dt0;
        let mut rng_m = 0.0;
        for _ in 0..2 {
            let (_, d, r) = hackrf_gnss::gps::snapshot::sat_at_txtime_pub(eph, a, site_m);
            dt_sv = d;
            rng_m = r;
            a = t_tx - d + r / 299_792_458.0;
        }
        let geom_km = rng_m / 1000.0;
        let rho_km = rho_m / 1000.0 + dt_sv * C_KM_S; // sat clock removed, as in live_fix
        raw.push((prn, rho_km - geom_km));
    }
    // (prn, raw BDS residual in km before clock removal). t_tx for BDS is
    // stored as GPST-equivalent SOW (see beidou_d1.rs), so the BDS median is
    // directly comparable to the GPS median: bds_clock - gps_clock is the
    // inter-system time-base bias the mixed solver reports as isx.
    let mut raw_bds: Vec<(u8, f64)> = Vec::new();
    for s in v["tracker"]["sats"].as_array().into_iter().flatten() {
        if s["sys"].as_str() != Some("beidou") {
            continue;
        }
        if now - s["epoch"].as_f64().unwrap_or(0.0) > 10.0 {
            continue;
        }
        let (Some(prn), Some(rho_m), Some(t_tx)) = (
            s["prn"].as_u64().map(|p| p as u8),
            s["rho_m"].as_f64(),
            s["t_tx"].as_f64(),
        ) else {
            continue;
        };
        let Some(eph) = bds_ephs.get(&prn) else {
            println!("BDS PRN {prn:2}: anchored but NO ephemeris — skipped");
            continue;
        };
        let site_m = site.map(|x| x * 1000.0);
        let (_, dt0, _) = sat_at_txtime_bds(eph, t_tx, site_m);
        let mut a = t_tx - dt0 + 0.075;
        let mut dt_sv = dt0;
        let mut rng_m = 0.0;
        for _ in 0..2 {
            let (_, d, r) = sat_at_txtime_bds(eph, a, site_m);
            dt_sv = d;
            rng_m = r;
            a = t_tx - d + r / 299_792_458.0;
        }
        let geom_km = rng_m / 1000.0;
        let rho_km = rho_m / 1000.0 + dt_sv * C_KM_S;
        raw_bds.push((prn, rho_km - geom_km));
    }
    if raw.is_empty() {
        println!("no anchored GPS channels in the tracker state right now");
        return;
    }

    let mut sorted: Vec<f64> = raw.iter().map(|&(_, r)| r).collect();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let clock = sorted[sorted.len() / 2];

    println!("surveyed site ECEF [{:.3}, {:.3}, {:.3}] km", site[0], site[1], site[2]);
    println!("common-mode clock (median): {clock:.3} km ({:.3} us)\n",
             clock / C_KM_S * 1e6);
    if raw.len() < 3 {
        println!("*** WARNING: only {} anchored channel(s) — the median pins one", raw.len());
        println!("*** channel to 0.000 BY CONSTRUCTION; residuals are unmeasurable");
        println!("*** below 3 channels. Use the pairwise drift rates below instead.");
        println!("*** A 0.000 'healthy' verdict right now is the artifact, not truth.\n");
    }
    // Pairwise divergence diagnostic (meaningful even at n=2 — this is what
    // caught the 2026-08-24 frozen-anchor divergence): compare each channel's
    // raw (rho - geom) against the previous run and report the drift rate.
    // A fresh anchor re-derives t_bit every refresh, so any sustained
    // per-channel rate beyond a few ns/s is an anchor gone stale (its rate
    // grows to the satellite's range rate, hundreds of m/s).
    let cache = "/tmp/anchor_residuals_prev.json";
    let prev: Option<serde_json::Value> = std::fs::read_to_string(cache)
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok());
    if let Some(p) = &prev {
        let dt = now - p["t"].as_f64().unwrap_or(now);
        if dt > 1.0 && dt < 900.0 {
            println!("drift vs run {:.0} s ago (stale-anchor detector):", dt);
            let rates: Vec<(u8, f64)> = raw
                .iter()
                .filter_map(|(prn, r)| {
                    p["raw"][prn.to_string()]
                        .as_f64()
                        .map(|pr| (*prn, (r - pr) / C_KM_S / dt * 1e9))
                })
                .collect();
            // Split into common-mode (mean — TCXO wander between discipline
            // updates, real clock physics, NOT an anchor bug) and per-channel
            // differential (rate_i - mean — the anchor-health signal). A
            // stale/biased anchor shows as differential drift; if all
            // channels move together, it's the clock.
            let mean = if rates.is_empty() {
                0.0
            } else {
                rates.iter().map(|(_, v)| v).sum::<f64>() / rates.len() as f64
            };
            if rates.len() >= 2 {
                println!("  common-mode (clock wander): {mean:+9.1} ns/s — not an anchor signal");
            }
            for (prn, rate_ns_s) in &rates {
                let diff = rate_ns_s - mean;
                let flag = if rates.len() >= 2 {
                    if diff.abs() > 5.0 {
                        format!("  <== DIFFERENTIAL DRIFT {diff:+.1} ns/s — anchor suspect")
                    } else {
                        String::new()
                    }
                } else if rate_ns_s.abs() > 5.0 {
                    "  <== drifting (single channel — cannot split common-mode)".to_string()
                } else {
                    String::new()
                };
                println!("  PRN {prn:2}: {rate_ns_s:+9.1} ns/s{flag}");
            }
            println!();
        }
    }
    let mut cur = serde_json::json!({"t": now, "raw": {}});
    for (prn, r) in &raw {
        cur["raw"][prn.to_string()] = serde_json::json!(r);
    }
    let _ = std::fs::write(cache, cur.to_string());
    println!("{:>6} {:>14} {:>12}  verdict", "PRN", "resid (km)", "resid (us)");
    let mut worst = 0.0f64;
    for (prn, r) in &raw {
        let resid = r - clock;
        let us = resid / C_KM_S * 1e6;
        worst = worst.max(resid.abs());
        let verdict = if resid.abs() < 0.05 {
            "healthy (meter class)"
        } else if resid.abs() < 1.0 {
            "tens-hundreds of m — fractional code-phase suspect"
        } else if (resid.abs() - 299.8).abs() < 50.0 {
            "~300 km — 1 ms CODE-PERIOD AMBIGUITY"
        } else if (resid.abs() - 5995.0).abs() < 500.0 {
            "~6000 km — 20 ms NAV-BIT SLIP"
        } else {
            "unclassified — inspect"
        };
        println!("{:>6} {:>14.3} {:>12.1}  {}", prn, resid, us, verdict);
    }
    println!(
        "\nworst |resid|: {:.3} km — {}",
        worst,
        if raw.len() < 3 {
            "UNMEASURABLE — need >= 3 anchored channels (median pinning artifact)"
        } else if worst < 0.05 {
            "anchors trustworthy for PVT"
        } else {
            "DO NOT trust an anchored solve"
        }
    );

    // ---- BeiDou split (round-8 phase 2) ----
    if !raw_bds.is_empty() {
        let mut bsorted: Vec<f64> = raw_bds.iter().map(|&(_, r)| r).collect();
        bsorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let bds_clock = bsorted[bsorted.len() / 2];
        let isx_km = bds_clock - clock;
        println!("\n--- BeiDou ({} anchored) ---", raw_bds.len());
        println!(
            "BDS common-mode: {bds_clock:.3} km | isx (BDS-GPS median split): {isx_km:+.3} km ({:+.2} us)",
            isx_km / C_KM_S * 1e6
        );
        if raw_bds.len() < 2 {
            println!("*** only one BDS channel — per-channel split unmeasurable;");
            println!("*** the isx line above conflates time-base bias with this channel's own error");
        }
        println!("{:>6} {:>14} {:>12}", "PRN", "resid (km)", "resid (us)");
        for (prn, r) in &raw_bds {
            let resid = r - bds_clock;
            println!(
                "{:>6} {:>14.3} {:>12.1}",
                prn,
                resid,
                resid / C_KM_S * 1e6
            );
        }
        if raw_bds.len() >= 2 {
            // All channels sharing the same offset => common BDS time-base
            // bias; one channel departing => per-channel measurement issue.
            let spread = bsorted[bsorted.len() - 1] - bsorted[0];
            println!(
                "BDS channel spread: {:.3} km — {}",
                spread,
                if spread < 0.05 {
                    "channels agree; offset is a COMMON time-base bias"
                } else {
                    "channels disagree; per-channel measurement issue in the mix"
                }
            );
        }
    }
}
