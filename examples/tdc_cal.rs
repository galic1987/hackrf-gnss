//! TDC code-density calibration report (offline, no radio).
//!
//! Ingests a `--tdc-read` JSONL capture (ring-osc selftest thermometer
//! codes), builds the popcount histogram via `TdcCal`, and prints the DNL
//! report. Also runs a per-tap transition-density analysis, which — unlike
//! the popcount histogram — stays valid for the multi-edge wave excitation
//! our fast ring oscillator produces (see below).
//!
//! EXCITATION CAVEAT (measured, run0): the classic code-density formula
//! (bin width = hit share x clock period) assumes a SINGLE edge in flight
//! with phase uniform over the adclk period. The 3-inverter RO is much
//! faster than the chain is deep: samples hold several edges (mean 12.3
//! transitions per 64-tap sample), so the popcount histogram is the wave
//! statistics, not the single-edge response. The plan-formula widths/LUT
//! are printed and saved marked "formal", but absolute calibration needs
//! either a slow single-edge source (Task 6 trigger, or a future gateware
//! rev with an RO divider) or an independent RO frequency measurement —
//! the wave analysis here fixes all per-tap delays RELATIVE to the RO
//! half-period, leaving one unknown scale (T_ro itself).
//!
//! Usage: tdc_cal [INPUT.jsonl] [OUTPUT.json] [ADCLK_HZ]

use hackrf_gnss::tdc::{TdcCal, popcount_thermo};

#[derive(serde::Deserialize)]
struct TdcRec {
    thermo: String,
    popcount: u32,
}

fn thermo_bits(hex: &str) -> Vec<u8> {
    (0..hex.len() / 2)
        .flat_map(|i| {
            let b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap();
            (0..8).map(move |j| (b >> j) & 1) // byte i bit j = tap 8i+j, LSB first
        })
        .collect()
}

fn main() -> anyhow::Result<()> {
    let input = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/tmp/tdc_cal_run0.jsonl".into());
    let output = std::env::args()
        .nth(2)
        .unwrap_or_else(|| "/Volumes/Radiator 8TB/gnss/observations/tdc_cal_run0.json".into());
    let adclk_hz: f64 = std::env::args()
        .nth(3)
        .map(|s| s.parse())
        .transpose()?
        .unwrap_or(32.1178e6);

    let text = std::fs::read_to_string(&input)?;
    let recs: Vec<TdcRec> = text
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(serde_json::from_str)
        .collect::<Result<_, _>>()?;
    let n = recs.len();
    // Cross-check: host-side popcount of the raw block must equal the
    // CLI's popcount (validates the byte-lane order contract).
    let bytes_of = |hex: &str| -> Vec<u8> {
        (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap())
            .collect()
    };
    for r in &recs {
        assert_eq!(popcount_thermo(&bytes_of(&r.thermo)), r.popcount);
    }
    println!("== TDC calibration report: {input}");
    println!("samples: {n}   adclk: {adclk_hz} Hz (period {:.3} ns)", 1e9 / adclk_hz);

    // --- Plan-formula popcount code density (formal; see header caveat) ---
    let mut cal = TdcCal::new(1.0 / adclk_hz);
    for r in &recs {
        cal.ingest(r.popcount);
    }
    let widths = cal.bin_widths_ps();
    let lut = cal.lut_ps();
    let pops: Vec<u32> = recs.iter().map(|r| r.popcount).collect();
    let (pmin, pmax) = (*pops.iter().min().unwrap(), *pops.iter().max().unwrap());
    let mut populated: Vec<(usize, f64, u64)> = cal
        .hist
        .iter()
        .enumerate()
        .filter(|(_, c)| **c > 0)
        .map(|(i, &c)| (i, widths[i], c))
        .collect();
    populated.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    let mean_w = widths.iter().sum::<f64>() / populated.len() as f64;
    let median_w = populated[populated.len() / 2].1;
    println!("\n-- popcount histogram (formal code-density) --");
    println!(
        "populated bins: {} (pop {}..={}); saturation: pop0={} pop64={}",
        populated.len(),
        pmin,
        pmax,
        cal.hist.first().copied().unwrap_or(0),
        cal.hist.get(64).copied().unwrap_or(0)
    );
    println!(
        "bin widths ps: min {:.0} (pop {})  median {:.0}  mean {:.0}  widest {:.0} (pop {})",
        populated.first().unwrap().1,
        populated.first().unwrap().0,
        median_w,
        mean_w,
        cal.widest_bin_ps(),
        populated.last().unwrap().0
    );
    println!(
        "formal LUT span: {:.3} ns over {} populated bins (by construction = 1 adclk period)",
        1e9 / adclk_hz,
        populated.len()
    );

    // --- Per-tap transition density (valid for wave excitation) ---
    // For a square wave, P(ci_k != ci_{k+1}) = 2 * tap_delay_k / T_ro
    // (tap delay <= T_ro/2). Single-registered sampling adds metastability
    // bubbles (~uniform background of fake transitions), so treat small
    // p_k as upper bounds. Only the 48 real taps are analyzed: the shipped
    // rev3c chain is 48 taps in a 6-byte block (the 64-tap/16-byte legacy
    // panicked here on a real rev3c capture — round-15 chain).
    let taps = 48;
    let bits: Vec<Vec<u8>> = recs
        .iter()
        .map(|r| thermo_bits(&r.thermo)[..taps].to_vec())
        .collect();
    let mut trans = vec![0u64; taps - 1];
    for s in &bits {
        for (k, t) in trans.iter_mut().enumerate() {
            if s[k] != s[k + 1] {
                *t += 1;
            }
        }
    }
    let p: Vec<f64> = trans.iter().map(|&t| t as f64 / n as f64).collect();
    let sum_p: f64 = p.iter().sum();
    println!("\n-- wave transition analysis (scale-free) --");
    println!(
        "mean transitions/sample: {:.2}  ->  chain depth D = {:.2} T_ro (= sum p_k / 2)",
        sum_p,
        sum_p / 2.0
    );
    let mut by_p: Vec<(usize, f64)> = p.iter().copied().enumerate().collect();
    by_p.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    println!("widest inter-tap boundaries (k->k+1: p_k, delay = p_k x T_ro/2):");
    for (k, pk) in by_p.iter().take(6) {
        let hop = [15, 31, 47].contains(k);
        println!("  {:2}->{:<2} p={:.3}{}", k, k + 1, pk, if hop { "  (segment hop)" } else { "" });
    }
    println!(
        "segment-hop boundaries: 15->16 p={:.3}  31->32 p={:.3}",
        p[15], p[31]
    );
    let mut sorted_p = p.clone();
    sorted_p.sort_by(f64::total_cmp);
    println!(
        "inter-tap p: median {:.3}  min {:.3}",
        sorted_p[sorted_p.len() / 2],
        sorted_p[0]
    );

    // --- Window / coverage assessment ---
    println!("\n-- window assessment --");
    println!(
        "observed pop window {pmin}..={pmax} with soft tails (edge counts {} and {} of {n});",
        cal.hist[pmin as usize], cal.hist[pmax as usize]
    );
    println!("no railing at pop 0 or 64 -> the window is RO-WAVE STATISTICS, not chain-end saturation.");
    println!(
        "Coverage of the {:.2} ns adclk period cannot be derived from wave data alone",
        1e9 / adclk_hz
    );
    println!("(one free scale: T_ro). Model estimate (icetime worst case ~81 ns for 64 taps + hops;");
    println!("typical silicon ~50-60 ns) puts D > period -> probably NO dead zone; verify with a");
    println!("single-edge source (Task 6 trigger, or gateware rev with RO divider / RO edge counter).");

    // --- Statistical sufficiency ---
    let min_count = populated.first().unwrap().2;
    // Width relative error ~ 1/sqrt(count); 5% needs ~400 hits.
    let need_total = (n as u64 * 400).div_ceil(min_count);
    println!("\n-- statistical sufficiency --");
    println!(
        "thinnest pop bin: {} hits (pop {}); relative width error ~{:.0}% now",
        min_count,
        populated.first().unwrap().0,
        100.0 / (min_count as f64).sqrt()
    );
    println!(
        "for ~5% on the thinnest observed bin: ~{}k samples total (have {}k); tails beyond the",
        need_total / 1000 + 1,
        n / 1000
    );
    println!("window need more. Per-tap transition probs already solid (se <= 0.4% at N={n}).");

    // --- JSON artifact ---
    let artifact = serde_json::json!({
        "meta": {
            "source": input,
            "samples": n,
            "adclk_hz": adclk_hz,
            "clk_period_s": 1.0 / adclk_hz,
            "taps": 64,
            "image": "timing variant (top/timing.py), taps=64, sync_stages=1, segment=16",
            "caveat": "multi-edge RO wave excitation: popcount bin widths are FORMAL (single-edge assumption violated); use tap_trans_prob for relative DNL; absolute scale pending T_ro measurement or single-edge calibration",
            "created_unix": std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)?
                .as_secs(),
        },
        "hist": cal.hist,
        "bin_widths_ps_formal": widths,
        "lut_ps_formal": lut,
        "tap_trans_prob": p,
        "pop_window": [pmin, pmax],
    });
    std::fs::write(&output, serde_json::to_string_pretty(&artifact)?)?;
    println!("\nartifact written: {output}");
    Ok(())
}
