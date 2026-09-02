# Sub-ns Leg 1: Clock-Bias Series + ADEV Analyzer — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a continuous 1 Hz receiver clock-bias series b(t) from the live
tracker, and measure its detrended RMS + Allan deviation to test the sub-ns
precision claim of the approved design.

**Architecture:** The tracker (`live_radio`) already publishes everything needed
at 1 Hz in `state.tracker.json`: `rho_m`, `t_tx`, `carrier_cycles`, `slip`,
`cn0_proxy`, `lock_s` per satellite. A new Rust example `clock_bias` reads that
file every second, Hatch-smooths each satellite's pseudorange with its carrier
(reset on slip), solves PVT with `pvt::solve_w` (both weighted and unweighted —
this doubles as the paired A/B for the elevation-weighting question), and appends
one JSON row per epoch to `observations/clock_bias.jsonl`. A Python analyzer
detrends the series and computes RMS + OADEV against the 1 ns claim gates. The
example only READS state files — it never touches the radio, so it can run
continuously beside the tracker.

**Tech Stack:** Rust (crate `hackrf_gnss`, example under `examples/`), Python 3
(stdlib + numpy), existing `src/gps/pvt.rs` solver, OADEV pattern reused from
`scripts/sync_stats.py`.

**Spec:** `docs/superpowers/specs/2026-08-27-sub-ns-precision-validation-design.md`

## Global Constraints

- **Build law (AGENTS.md):** NEVER run `cargo build`/`cargo test` while
  `tracker_producer.py` runs. A build window must gate the exact production
  serial with `pro_lease.py`, gracefully stop the tracker, acquire the lease,
  and reset the board immediately; only then may a full build/test run while
  the tracker remains down. Steps explicitly marked `[tracker-safe]` are
  limited to lightweight offline work.
- Tracker restart pattern (only): token-owned gate → graceful exact stop →
  token-owned lease acquisition → immediate serial-addressed board reset →
  staged work/restore → token-owned release → relaunch. Never `pkill -9` and
  never remove lock objects directly.
- SHADOW invariant: production `live_radio` has no actuator call and rejects
  the retired `HACKRF_GNSS_ACTUATE` variable before radio open; this feature is
  observe-only by construction (reads state files only).
- Antenna freeze: no antenna/cable moves during data collection; after any
  antenna event, FULL power-cycle of the radio (USB `-R` does not clear the
  bias-tee overcurrent latch — the 2026-08-27 13:21–18:44 outage).
- Local commits only; single-quote commit messages; never push.
- Claim gates (from spec): detrended RMS < 1 ns and OADEV < 1 ns for
  τ = 10–1000 s over ≥ 1 h of continuous data.

## File Structure

- `src/gps/hatch.rs` (NEW): the Hatch carrier-smoother — one responsibility,
  unit-testable. `pub struct Hatch { window: f64, ... }` with per-satellite
  instances keyed by PRN in the caller.
- `src/gps/mod.rs` (MODIFY: +1 line): `pub mod hatch;`
- `examples/clock_bias.rs` (NEW): the 1 Hz loop (state read → smooth → solve →
  append). Modeled on `examples/live_fix.rs` ephemeris/measurement code.
- `scripts/clock_bias_analyzer.py` (NEW): detrend + RMS + OADEV + verdict.
- `scripts/test_clock_bias_analyzer.py` (NEW): analyzer test on a synthetic
  series with known ADEV (mirrors repo's `scripts/test_*.py` pattern).

## Interfaces (cross-task contract)

- `hatch.rs` produces:
  ```rust
  pub struct Hatch { n: f64, window: f64, smoothed: Option<f64>, last_carrier: Option<f64> }
  impl Hatch {
      pub fn new(window_epochs: f64) -> Self;
      /// code_m: raw pseudorange (m); carrier_cycles: integrated carrier (cycles);
      /// lam: carrier wavelength (m/cycle); reset: slip/generation change.
      /// Returns smoothed pseudorange (m).
      pub fn update(&mut self, code_m: f64, carrier_cycles: f64, lam: f64, reset: bool) -> f64;
  }
  ```
- `examples/clock_bias.rs` consumes `Hatch` and `pvt::solve_w`, produces rows in
  `observations/clock_bias.jsonl`, keys verbatim:
  `epoch, clock_ns, clock_ns_uw, tdop, n_sat, gdop, residual_rms_m, residual_rms_m_uw, n_smoothed, slips, source`
  (`_uw` = unweighted solve of the same epoch — the paired A/B).
- Analyzer consumes those keys; prints `rms_ns`, `adev` table, verdict.

---

### Task 1: Hatch carrier-smoother (`src/gps/hatch.rs`)

**Files:**
- Create: `src/gps/hatch.rs`
- Modify: `src/gps/mod.rs` (add `pub mod hatch;` next to the other `pub mod` lines)

**Interfaces:**
- Consumes: nothing (std only).
- Produces: `Hatch::new`, `Hatch::update` as specified above.

- [ ] **Step 1: Write the failing tests** (in `src/gps/hatch.rs` `mod tests`)

All test data is CONSISTENT: the carrier series is derived from the same truth
as the code (`carrier = (truth - R0) / LAM`). Feeding the filter inconsistent
data (code static, carrier ramping) is unphysical and drives the recursion to
the wrong fixed point (`code + (n-1)·d`) — an earlier draft of these tests made
exactly that mistake.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    const LAM: f64 = 0.190293672798; // L1 wavelength m
    const R0: f64 = 20_000_000.0;

    #[test]
    fn exact_on_consistent_data() {
        // truth drifts 19.029 m/epoch = exactly 100 L1 cycles/epoch
        let mut h = Hatch::new(100.0);
        for k in 0..300u64 {
            let r = R0 + 19.029 * k as f64;
            let carr = (r - R0) / LAM;
            let out = h.update(r, carr, LAM, false);
            assert!((out - r).abs() < 1e-6, "k={k} out={out} r={r}");
        }
    }

    #[test]
    fn tracks_consistent_step_immediately() {
        // truth steps an extra +19.0294 m at k=200; carrier steps with it
        let mut h = Hatch::new(100.0);
        for k in 0..300u64 {
            let step = if k >= 200 { 19.0293672798 } else { 0.0 };
            let r = R0 + 19.029 * k as f64 + step;
            let carr = (r - R0) / LAM;
            let out = h.update(r, carr, LAM, false);
            assert!((out - r).abs() < 1e-6, "k={k} out={out} r={r}");
        }
    }

    #[test]
    fn reset_flushes_state() {
        let mut h = Hatch::new(100.0);
        for k in 0..200u64 {
            let r = R0 + 19.029 * k as f64;
            h.update(r, (r - R0) / LAM, LAM, false);
        }
        // slip: carrier jumps arbitrarily; reset must re-init from code
        let v = h.update(R0 + 500.0, 555_555.0, LAM, true);
        assert_eq!(v, R0 + 500.0);
    }

    #[test]
    fn noise_var_shrinks_vs_raw() {
        // deterministic pseudo-noise on code only; smoother must cut its spread
        let mut h = Hatch::new(100.0);
        let mut raw = Vec::new();
        let mut sm = Vec::new();
        for k in 0..300u64 {
            let noise = (k.wrapping_mul(2654435761) % 2000) as f64 / 100.0 - 10.0;
            let r = R0 + 19.029 * k as f64;
            let out = h.update(r + noise, (r - R0) / LAM, LAM, false);
            if k >= 200 { raw.push(noise); sm.push(out - r); }
        }
        let spread = |v: &[f64]| {
            let m = v.iter().sum::<f64>() / v.len() as f64;
            (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / v.len() as f64).sqrt()
        };
        assert!(spread(&sm) < spread(&raw) * 0.3,
                "raw {} sm {}", spread(&raw), spread(&sm));
    }
}
```

- [ ] **Step 2: Verify tests fail to compile** (tracker-down window, or
  `[tracker-safe]` `nice -n 19 cargo check` first):

Run: `cargo test --lib hatch 2>&1 | tail -5`
Expected: FAIL — `src/gps/hatch.rs` missing (add `pub mod hatch;` to
`src/gps/mod.rs` first to see the unresolved-import error).

- [ ] **Step 3: Implement `src/gps/hatch.rs`**

```rust
//! Hatch carrier-smoothing of pseudoranges (sub-ns Leg 1, spec 2026-08-27).
//!
//! One filter per satellite per epoch:
//!   S(k) = (1/n)·P(k) + ((n-1)/n)·[S(k-1) + λ·(Φ(k) - Φ(k-1))],  n → window
//! with S(0) = P(0). Carrier carries the dynamics; code pins the absolute.
//! Any slip/generation change in the carrier series must call update with
//! reset = true — a smoothed value across a discontinuity is worse than raw.

#[derive(Debug, Clone)]
pub struct Hatch {
    n: f64,
    window: f64,
    smoothed: Option<f64>,
    last_carrier: Option<f64>,
}

impl Hatch {
    pub fn new(window_epochs: f64) -> Self {
        Self { n: 0.0, window: window_epochs, smoothed: None, last_carrier: None }
    }

    pub fn update(&mut self, code_m: f64, carrier_cycles: f64, lam: f64, reset: bool) -> f64 {
        if reset || self.smoothed.is_none() || self.last_carrier.is_none() {
            self.n = 1.0;
            self.smoothed = Some(code_m);
            self.last_carrier = Some(carrier_cycles);
            return code_m;
        }
        let d_m = (carrier_cycles - self.last_carrier.unwrap()) * lam;
        self.last_carrier = Some(carrier_cycles);
        self.n = (self.n + 1.0).min(self.window);
        let s = self.smoothed.unwrap() + d_m;
        let out = code_m / self.n + s * (self.n - 1.0) / self.n;
        self.smoothed = Some(out);
        out
    }
}
```

- [ ] **Step 4: Run the tests** (tracker-down window)

Run: `cargo test --lib hatch -- --nocapture`
Expected: 4 passed, 0 failed.

- [ ] **Step 5: Commit**

```bash
git add src/gps/hatch.rs src/gps/mod.rs
git commit -m 'hatch: carrier-smoother for the sub-ns clock-bias series (Leg 1). Classic per-epoch Hatch recursion with n capped at the window; reset flag flushes state on slip/generation change (a smoothed value across a carrier discontinuity is worse than raw code). Tests: convergence, step tracking via carrier, reset, variance reduction vs raw (<30%).'
```

---

### Task 2: `examples/clock_bias.rs` — the 1 Hz series

**Files:**
- Create: `examples/clock_bias.rs`

**Interfaces:**
- Consumes: `hackrf_gnss::gps::hatch::Hatch` (Task 1);
  `hackrf_gnss::gps::pvt` solve path; ephemeris helpers mirrored from
  `examples/live_fix.rs` (`sat_at_txtime_pub` :575-585, measurement
  construction :680, gates :560-561); `state.tracker.json` sat keys
  (`prn, sys, rho_m, t_tx, carrier_cycles, slip, cn0_proxy, lock_s, epoch`).
- Produces: `observations/clock_bias.jsonl` rows with keys:
  `epoch, clock_ns, clock_ns_uw, tdop, n_sat, gdop, residual_rms_m,
   residual_rms_m_uw, n_smoothed, slips, source`.

- [ ] **Step 1: Read the reference code**

Read `examples/live_fix.rs:540-720` (state read, gates, ephemeris, Meas
construction) and `src/gps/pvt.rs:68-150` (`solve`, `Fix` fields: `clock_km`,
`tdop`, `residual_rms_m`, `gdop`, `n_sat`). The new example reuses the same
ephemeris-loading helper live_fix uses for BRDC (copy the exact loader call
from live_fix — it is the crate's canonical ephemeris path).

- [ ] **Step 2: Write the example** (complete skeleton; fill the two marked
  helpers by copying live_fix verbatim)

```rust
//! 1 Hz receiver clock-bias series for the sub-ns precision claim (Leg 1).
//!
//! Reads state.tracker.json once per second (the tracker owns the radio; we
//! never touch it), Hatch-smooths each GPS satellite's pseudorange against its
//! published carrier (reset on slip), solves PVT weighted AND unweighted
//! (paired elevation-weighting A/B), and appends one row to
//! observations/clock_bias.jsonl. Rows:
//! {epoch, clock_ns, clock_ns_uw, tdop, n_sat, gdop, residual_rms_m,
//!  residual_rms_m_uw, n_smoothed, slips, source}

use hackrf_gnss::gps::hatch::Hatch;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::{fs, thread, time::Duration};

const STATE: &str = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json";
const OUT: &str = "/Volumes/Radiator 8TB/gnss/observations/clock_bias.jsonl";
const LAM_L1: f64 = 299_792_458.0 / 1_575_420_000.0;
const WINDOW_S: f64 = 100.0;

fn main() {
    // ephemeris: same loader live_fix uses (BRDC); refresh every 15 min
    let mut smoothers: HashMap<u8, Hatch> = HashMap::new();
    let mut last_epoch = 0.0_f64;
    loop {
        thread::sleep(Duration::from_millis(500));
        let txt = match fs::read_to_string(STATE) { Ok(t) => t, Err(_) => continue };
        let st: Value = match serde_json::from_str(&txt) { Ok(v) => v, Err(_) => continue };
        let epoch = st["epoch"].as_f64().unwrap_or(0.0);
        if epoch <= last_epoch { continue; }
        last_epoch = epoch;
        let sats = match st["tracker"]["sats"].as_array() { Some(s) => s, None => continue };

        let mut meas = Vec::new();
        let mut slips = 0u32;
        let mut n_smoothed = 0u32;
        for s in sats {
            if s["sys"].as_str() != Some("gps") { continue; }
            if s["cn0_proxy"].as_f64().unwrap_or(0.0) < 30.0 { continue; }
            if s["lock_s"].as_f64().unwrap_or(0.0) < 20.0 { continue; }
            let (rho, t_tx) = match (s["rho_m"].as_f64(), s["t_tx"].as_f64()) {
                (Some(a), Some(b)) => (a, b), _ => continue };
            let prn = s["prn"].as_u64().unwrap_or(0) as u8;
            let slip = s["slip"].as_bool().unwrap_or(false);
            if slip { slips += 1; }
            let carr = s["carrier_cycles"].as_f64().unwrap_or(0.0);
            let h = smoothers.entry(prn).or_insert_with(|| Hatch::new(WINDOW_S));
            let rho_s = h.update(rho, carr, LAM_L1, slip);
            n_smoothed += 1;
            // sat position + SV clock correction: copy live_fix.rs:575-585 and
            // :680 verbatim (sat_at_txtime_pub, dt_sv + daf0 term)
            meas.push(build_meas(prn, rho_s, t_tx)); // -> Option<Meas> filter
        }
        let meas: Vec<_> = meas.into_iter().flatten().collect();
        if meas.len() < 5 { continue; }   // redundancy required; exact 4-sat solves are unverifiable
        let g = site_guess();             // copy live_fix site guess
        let fw = hackrf_gnss::gps::pvt::solve(&meas, g);        // weighted (default)
        let fu = hackrf_gnss::gps::pvt::solve_unweighted(&meas, g); // Task 2 Step 3
        if let (Some(a), Some(b)) = (fw, fu) {
            let row = json!({
                "epoch": epoch,
                "clock_ns": a.clock_km * 1e9 / 299_792.458,
                "clock_ns_uw": b.clock_km * 1e9 / 299_792.458,
                "tdop": a.tdop, "n_sat": a.n_sat, "gdop": a.gdop,
                "residual_rms_m": a.residual_rms_m,
                "residual_rms_m_uw": b.residual_rms_m,
                "n_smoothed": n_smoothed, "slips": slips,
                "source": "clock_bias",
            });
            use std::io::Write;
            let mut f = fs::OpenOptions::new().create(true).append(true).open(OUT).unwrap();
            writeln!(f, "{}", row).unwrap();
        }
    }
}

fn build_meas(_prn: u8, _rho_m: f64, _t_tx: f64) -> Option<hackrf_gnss::gps::pvt::Meas> {
    // copy live_fix.rs:560-585 + :680 (gates already applied above):
    // sat_at_txtime_pub for sat ECEF at transmit, dt_sv + daf0 SV clock term,
    // returns Meas { sat, pseudorange (km), clock_free: false }
    unimplemented!("copy from live_fix.rs")
}

fn site_guess() -> [f64; 3] {
    // copy live_fix's site ECEF guess (site_lla -> ecef)
    unimplemented!("copy from live_fix.rs")
}
```

- [ ] **Step 3: Expose the unweighted solve** (for the paired A/B)

`pvt::solve_w` is `pub(crate)`. In `src/gps/pvt.rs` add:

```rust
/// Unweighted companion to `solve` (paired elevation-weighting A/B, Leg 1).
pub fn solve_unweighted(meas: &[Meas], guess: [f64; 3]) -> Option<Fix> {
    solve_w(meas, guess, false)
}
```

- [ ] **Step 4: `nice -n 19 cargo check --example clock_bias`** `[tracker-safe]`
  then full `cargo build --release --example clock_bias` in the tracker-down
  window. Expected: compiles with zero warnings.

- [ ] **Step 5: Smoke-run against the live state file** `[tracker-safe]` (the
  example only reads state files; the tracker keeps running):

```bash
DYLD_LIBRARY_PATH=/Volumes/"Radiator 8TB"/mac-archive/hackrf/host/build/libhackrf/src \
  timeout 30 ./target/release/examples/clock_bias &
sleep 32; tail -3 /Volumes/"Radiator 8TB"/gnss/observations/clock_bias.jsonl
```

Expected: ≥ 25 rows appended; each has `n_sat >= 5`, `clock_ns` finite,
`residual_rms_m` materially below the unsmoothed history norm (~10 m) once the
smoother converges (~100 s).

- [ ] **Step 6: Commit**

```bash
git add examples/clock_bias.rs src/gps/pvt.rs
git commit -m 'clock_bias: 1 Hz receiver clock-bias series (Leg 1). Reads state.tracker.json (never the radio), Hatch-smooths GPS pseudoranges against the published carrier with slip reset, solves weighted + unweighted per epoch (paired elevation A/B), appends clock_bias.jsonl with tdop/gdop/rms provenance. Redundancy-gated at n>=5 — exact 4-sat solves stay unverifiable and out of the series.'
```

---

### Task 3: Analyzer with claim gates

**Files:**
- Create: `scripts/clock_bias_analyzer.py`
- Test: `scripts/test_clock_bias_analyzer.py`

**Interfaces:**
- Consumes: `observations/clock_bias.jsonl` keys from Task 2.
- Produces: stdout report + exit 0/1 on gates. Functions: `detrend(epochs,
  clock_ns) -> residuals_ns`, `oadev(residuals_ns, dt_s, taus) -> dict`,
  `verdict(rms_ns, adev_tbl) -> bool`.

- [ ] **Step 1: Write the failing test** `scripts/test_clock_bias_analyzer.py`

```python
import math
from clock_bias_analyzer import detrend, oadev, verdict

def test_recovers_known_white_noise_adev():
    # synthetic: 2 h at 1 Hz, white phase noise sigma = 0.4 ns
    import random
    random.seed(7)
    n = 7200
    ep = [1_787_000_000.0 + k for k in range(n)]
    clk = [1e-6 * k + random.gauss(0, 0.4) for k in range(n)]  # drift + noise
    res = detrend(ep, clk)
    rms = math.sqrt(sum(r * r for r in res) / len(res))
    assert 0.35 < rms < 0.45, rms
    tbl = oadev(res, 1.0, [10, 100, 1000])
    for t, v in tbl.items():
        assert 0.2 < v < 0.6, (t, v)      # TDEV of white phase noise ~= sigma, flat

def test_verdict_gates():
    assert verdict(0.5, {10: 0.3, 100: 0.1, 1000: 0.05}) is True
    assert verdict(1.5, {10: 0.3, 100: 0.1, 1000: 0.05}) is False
    assert verdict(0.5, {10: 1.2, 100: 0.1, 1000: 0.05}) is False
```

Run: `python3 scripts/test_clock_bias_analyzer.py`
Expected: FAIL (`ModuleNotFoundError: clock_bias_analyzer`).

- [ ] **Step 2: Implement `scripts/clock_bias_analyzer.py`**

```python
#!/usr/bin/env python3
"""Sub-ns claim analyzer (Leg 1): detrend clock_bias.jsonl, RMS + OADEV, gates.

Usage: python3 scripts/clock_bias_analyzer.py [path] [--min-rows 3600]
Gates (spec 2026-08-27): RMS < 1 ns and OADEV < 1 ns for tau in 10..1000 s.
Exit 0 = claim supported, 1 = not (or insufficient data)."""
import json, math, sys

GATES_TAU = [10, 100, 1000]

def detrend(epochs, clock_ns):
    """Least-squares linear detrend (Bodnar GPS-steering is common-mode)."""
    n = len(epochs)
    t0 = sum(epochs) / n
    b0 = sum(clock_ns) / n
    s_tt = sum((t - t0) ** 2 for t in epochs)
    s_tb = sum((t - t0) * (b - b0) for t, b in zip(epochs, clock_ns))
    slope = s_tb / s_tt if s_tt else 0.0
    return [b - (b0 + slope * (t - t0)) for t, b in zip(epochs, clock_ns)]

def oadev(residuals, dt_s, taus):
    """Time deviation TDEV(tau) = tau*ADEV/sqrt(3), in the input's units (ns).

    x_k is the time-error series; ADEV = sqrt(mean((x[k+2m]-2x[k+m]+x[k])^2)/2)/tau
    (dimensionless); TDEV puts it back in ns. White phase noise sigma -> TDEV = sigma
    at all tau (flat), which is what the synthetic test pins."""
    out = {}
    n = len(residuals)
    for tau in taus:
        m = max(1, int(round(tau / dt_s)))
        vals = []
        for k in range(n - 2 * m):
            v = (residuals[k + 2 * m] - 2 * residuals[k + m] + residuals[k])
            vals.append(v * v)
        if vals:
            adev = math.sqrt(sum(vals) / (2 * len(vals))) / tau
            out[tau] = adev * tau / math.sqrt(3.0)
    return out

def verdict(rms_ns, adev_tbl):
    return rms_ns < 1.0 and all(v < 1.0 for v in adev_tbl.values())

def main():
    path = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else \
        "/Volumes/Radiator 8TB/gnss/observations/clock_bias.jsonl"
    min_rows = 3600
    rows = [json.loads(l) for l in open(path) if l.strip()]
    rows = [r for r in rows if r.get("n_sat", 0) >= 5 and r.get("slips", 1) == 0]
    ep = [r["epoch"] for r in rows]
    ck = [r["clock_ns"] for r in rows]
    print(f"rows {len(rows)} (slip-free, n_sat>=5)")
    if len(rows) < min_rows:
        print(f"INSUFFICIENT DATA (<{min_rows} rows)"); sys.exit(1)
    res = detrend(ep, ck)
    rms = math.sqrt(sum(r * r for r in res) / len(res))
    dt = (ep[-1] - ep[0]) / max(1, len(ep) - 1)
    tbl = oadev(res, dt, GATES_TAU)
    print(f"detrended RMS: {rms:.3f} ns   (dt {dt:.2f} s)")
    for t in GATES_TAU:
        print(f"  OADEV(tau={t:>4}s): {tbl.get(t, float('nan')):.3f} ns")
    ok = verdict(rms, tbl)
    print("VERDICT:", "SUB-NS CLAIM SUPPORTED" if ok else "claim not supported")
    # paired elevation A/B summary (same epochs, both solves)
    d = [r["residual_rms_m"] - r["residual_rms_m_uw"] for r in rows
         if "residual_rms_m_uw" in r]
    if d:
        print(f"paired A/B: median (weighted-raw RMS) = {sorted(d)[len(d)//2]:+.2f} m "
              f"over {len(d)} epochs (negative = weighting helps)")
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
```

(Note: the estimator is TDEV — τ·ADEV/√3 in ns — the clock-world standard for a
time-error series. White phase noise σ gives TDEV ≈ σ at every τ, which is what
the synthetic test pins with loose ±50% bounds.)

- [ ] **Step 3: Run the test**

Run: `python3 scripts/test_clock_bias_analyzer.py`
Expected: both tests PASS (adjust the loose bounds only if the deterministic
seed lands outside; do not weaken the gate semantics).

- [ ] **Step 4: Commit**

```bash
git add scripts/clock_bias_analyzer.py scripts/test_clock_bias_analyzer.py
git commit -m 'clock_bias_analyzer: detrend + RMS + OADEV against the 1 ns claim gates (tau 10/100/1000 s), slip-free redundancy-gated rows only, exit 0/1 for automation; prints the paired elevation-weighting A/B delta from the same epochs. Test: synthetic white-noise series of known sigma must be recovered, plus verdict gate semantics.'
```

---

### Task 4: Maintenance window — deploy + first collection

**Files:** none new (operations only).

- [ ] **Step 1: Enter the atomic window.** Use `scripts/pro_lease.py gate`
  with the exact production serial and an operator token before any stop.
  Pause a pre-lease legacy
  `band_producer` only during first deployment, gracefully stop the exact
  tracker, then require token-owned `pro_lease.py acquire` success. Reset the
  board immediately—no build/test in the stop→reset gap.
- [ ] **Step 2: Build while gated and reset.** With the tracker down and the
  maintenance lease still held, run
  `cargo build --release --example clock_bias && cargo test --lib hatch`,
  then restore the verified production radio state. Release with the same
  token; never create/remove `maintenance.lock` directly.
- [ ] **Step 3: Restart tracker** after lease release. Confirm its JSON lease
  owner, ≥ 5 GPS channels and
  `rho_m` rows fresh in `state.tracker.json`.
- [ ] **Step 4: Launch the series producer** beside the tracker:

```bash
cd /Volumes/"Radiator 8TB"/gnss/hackrf_gnss && \
  nohup ./target/release/examples/clock_bias >> /tmp/clock_bias.log 2>&1 &
```

- [ ] **Step 5: SIGCONT band_producer.** Collect ≥ 1 h (3600 rows).
- [ ] **Step 6: Verdict run** `[tracker-safe]`:
  `python3 scripts/clock_bias_analyzer.py` — report RMS/OADEV and whether the
  sub-ns claim gates pass; archive output under
  `docs/superpowers/reports/2026-08-27-sub-ns-leg1-first-collection.md`.

---

## Self-Review notes (filled after writing)

- Spec coverage: Leg 1 components 1–4 → Tasks 1–4. GEO cross-check is covered
  by comparing `d(clock_ns)/dt` against `state.phase_drift.json` ppm in the
  Task 4 report step (phase_drift_producer already publishes the GEO series;
  no new code needed for v1). Leg 1b/Leg 2 are out of this plan's scope.
- The `oadev` helper computes TDEV (τ·ADEV/√3, ns) — the time-error stability
  measure the spec's "ADEV < 1 ns" gates mean in clock terms; test bounds are
  loose (±50%) to avoid over-fitting a unit test to estimator scatter.
- `solve_unweighted` addition is the only change to existing crate code;
  `solve` keeps its weighted default — no behavior change for live_fix.

---

## v2 amendment (2026-08-28, post-audit — supersedes v1 where they conflict)

The 2026-08-27/28 v1 collections produced 100% poison-class rows (25/25 rows
residual_rms_m 6.1–25 km; the earlier undefended run 1041/1041 at 25–520 km).
Root causes verified from live data 2026-08-28 morning:

1. **Carrier sign**: `carrier_cycles` integrates the replica NCO with the
   OPPOSITE sign convention to range — measured on all 4 live GPS channels
   (drho vs λ·Δcarr opposite sign, magnitudes within ~20%). The v1 Hatch
   propagated the smoothed pseudorange the wrong way. v2: pass the negated
   carrier to `Hatch::update` (and to the innovation prediction).
2. **rho_m is a ~6 s staircase**, not 1 Hz — the tracker refreshes rho at
   ~1/6 Hz per channel with per-channel phase. v1's frozen-rho gate therefore
   skips 5 of 6 epochs as designed, but the 3-freeze eviction fired on
   HEALTHY channels and destroyed smoothers. v2 semantics: a frozen rho is
   "no new code measurement" — SKIP the update (never evict on freeze
   count), and on skipped epochs use the filter's carrier prediction
   (right-signed) as the sat's range for the solve. The smoother becomes the
   1 Hz interpolator the staircase needs: code anchors on fresh epochs,
   carrier carries the epoch between. A sat contributes only while its last
   code update is < 12 s old (two staircase periods).
3. **The 1 ns gate is unreachable with a free-position solve** — per-epoch
   position noise (m-class × TDOP 7–20 observed) leaks into the clock
   unknown. v2 solves CLOCK-ONLY with the position FIXED at the surveyed
   site anchor (site.json): one unknown, residuals = clock + noise, outlier
   rejection via the studentized statistic (aeae8ec pattern generalized to
   the 1-unknown design). This is the claim enabler; live_fix keeps its
   free-position solve for the dashboard.
4. **Analyzer was mislabeled and under-gated**: the v1 "TDEV" print computed
   τ·ADEV/√3 (ordinary second-difference ADEV); proper TDEV uses MODIFIED
   Allan deviation (NIST SP 1065). v2: textbook MDEV-based TDEV with the
   correct white-PM τ behavior pinned by test; gap segmentation (spans with
   inter-row gaps > 5× median dt are excluded from τ evaluation, reported);
   `gen` session id on every row (producer start + config epoch) so
   restarts/config changes can't silently mix; "continuous hour" = explicit
   span ≥ 3600 s AND max-gap ≤ 5 s AND rows ≥ 3400, not row count alone;
   keep the residual_rms_m < 100 m poison-class exclusion; linear-detrend LF
   concealment documented (LF structure is the GEO cross-check's job, spec
   component 4).

### Task 5 (v2): producer `examples/clock_bias.rs` + `src/gps/pvt.rs` clock-only solve

- pvt.rs gains `solve_clock_only(meas: &[Meas], anchor_ecef_km: [f64;3],
  weighted: bool) -> Option<ClockFix>` (clock_km, residual_rms_m raw, n_sat,
  leverage figure) with studentized rejection mirroring aeae8ec; existing
  functions untouched.
- Producer: negated carrier everywhere; staircase-aware skip/predict (no
  freeze eviction); freshness < 12 s per sat; n≥5 gate on contributing sats;
  paired weighted/unweighted clock-only solves; rows:
  `{epoch, clock_ns, clock_ns_uw, residual_rms_m, residual_rms_m_uw, n_sat,
  n_fresh, n_pred, slips, gen, source}`.
- Keep from v1: gates (cn0≥30, lock_s≥20, rho/t_tx present), ephemeris path,
  raw solve (no median normalization), 500 ms rising-epoch poll, reset-on-
  (slip || lock regression || innovation>500 m) with slips counting resets.

### Task 6 (v2): analyzer `scripts/clock_bias_analyzer.py` + tests

- MDEV-TDEV, gap segmentation, gen-aware grouping (analyze latest gen by
  default, --all-gens to pool), continuity gates, poison-class exclusion,
  exit 0/1 unchanged in spirit. Test: synthetic white-PM series recovers the
  correct TDEV τ-slope AND σ; gapped series is segmented, not silently
  averaged.

### Task 7 (v2): window — full `cargo test --lib`, build clock_bias + live_fix
examples, `manifest_check.py` all four Pro#2 slots (tracker down), release-
ledger Pro#2 attestation, restart + re-arm for the evening GPS window.
