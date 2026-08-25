#!/usr/bin/env python3
"""Carrier-phase-derived clock-drift producer ("phase drift producer").

The station's highest-precision drift source. Reads ONLY
observations/state.tracker.json (the 1 Hz live tracker's per-satellite
carrier phase, commit dcfcfaa) and writes ONLY its own
observations/state.phase_drift.json (tmp + os.replace; the Rust server
merges all observations/state.*.json at /api/sync read time, so no
shared-file race is possible). Pure reader/writer — touches no radio,
signals no process.

Measurement: per locked SBAS/WAAS GEO satellite, the least-squares slope
of carrier_cycles over a sliding window (default 40 s, clamp 30-60) IS
the integrated Doppler in cycles/s = Hz — the same observable the code
row ("L1 / WAAS (live)") derives from the instantaneous PLL doppler_hz,
but at carrier-phase precision: live, the per-second NCO-advance noise
is ~1 cycle (the reported doppler_hz itself jitters +/-15 Hz — the
accumulator is the quiet observable), so the slope sigma over a 40 s
window is ~0.02 Hz = 1.27e-5 ppm class (0.02 / 1575.42e6 * 1e6) — ~3
orders of magnitude finer than the code Doppler's 0.01-0.05 ppm. TCXO
wander over the window shows up in the fit residuals, so the honest fit
sigma lands above the thermal floor — still well below any code-class
voter.

Convention (mirrors tracker_producer's WAAS ClockDriftPpm row exactly):
the measured slope is taken AFTER the hardware clock-correction register
(resid = raw - corr), so the published value adds the register back:
    ppm = slope_hz / L1_HZ * 1e6 + discipline.correction_ppm
making the row a RAW-TCXO measurement comparable to every other voter
(ATSC ch35 via CLKOUT, PC clock). Like the code row, GEO line-of-sight
motion Doppler is NOT subtracted (the tracker publishes no sat velocity;
the code row absorbs it in its sigma floor). The honest fit sigma is
published as-is; inter-source systematics (GEO motion ±0.025 ppm, CLKOUT
chain) are visible in the scatter.

NON-VOTING (systems review 2026-08-25, finding 6): this chain is an
implemented DIAGNOSTIC, not a discipline voter. It shares the radio and
the GEOs with the WAAS code row (one instrument must not vote twice),
GEO line-of-sight range rate is not yet removed, and the uncertainty is
white-noise OLS rather than correlated-residual. The consensus row is
therefore published as ClockDriftPpmComponent — visible in the merged
sources table, excluded from series_producer's voting consensus — until
geometry correction and real uncertainty land (P0b).

Continuity guards (a window is only as good as its phase chain):
  - slip=true on any report -> the chain broke that second (watchdog or
    reseed): the window is flushed and the slip sample itself is dropped.
  - reseed without a slip flag shows as carrier_cycles snapping back to
    ~0: |new| < 2% of |prev| with |prev| > 50 cycles is a collapse, not
    physics (a GEO Doppler cannot walk the accumulator back to zero in
    one report). The increment-vs-median check below catches smaller
    breaks.
  - increments are compared against the chain's OWN median increment,
    NOT against the reported doppler_hz: live, the FLL's doppler state
    jitters +/-15 Hz second-to-second while the integrated NCO advance
    wanders only ~1 cycle/s — the accumulator is the quiet observable,
    the Doppler report is not. Break threshold: 25 cycles in one step
    (nothing physical moves a GEO Doppler 25 Hz in 1 s; loop events
    that could come with slip / lock_s = 0 anyway).
  - a discipline STEP mid-window (correction_ppm changed between two
    reports) retunes the LO and shifts every channel's measured rate by
    up to step·L1 (0.03 ppm = 47 Hz); the register add-back convention
    is only exact for a constant register, so the window flushes on any
    correction change. Once the loop is in its deadband this is rare.
  - report gaps > 5 s or non-monotonic epochs (producer restart, file
    rotation) flush the window.
  - gates: newest lock_s >= window length (locked for the whole window)
    and every in-window cn0_proxy >= 30 dB (the tracker's UNLOCK_DB).

Missing fields (pre-dcfcfaa state files lack carrier_cycles/phase_frac/
slip): those sats are skipped; if no sat carries the fields the producer
just heartbeats its file with no rows — rows vanish rather than freeze
(the tombstone class).

Publishes:
  one consensus row (band "L1 / WAAS GEO (phase)") = sigma-weighted
  median across GEOs, sigma =
  max(formal 1/sqrt(Σw), weighted scatter) — the scatter term keeps the
  consensus honest when GEOs disagree (their line-of-sight rates differ
  by m/s class; live, the three visible WAAS GEOs spread ~0.002 ppm and
  the consensus sigma correctly reports that, not the 1e-4-ppm formal).
  ALL rows (per-satellite and consensus) are kind ClockDriftPpmComponent
  — visible in the merged sources table but NON-VOTING (see the
  NON-VOTING note above): one instrument must vote once in
  series_producer's cross-producer consensus, and this chain shares the
  radio with the WAAS code row.
  state["phase_drift"] — diagnostics: per-sat slope/fit sigma/n/gates.

Unit tests: scripts/test_phase_drift_producer.py (synthetic fixtures,
no live observations touched).
"""
import argparse
import json
import math
import os
import time

OBS = "/Volumes/Radiator 8TB/gnss/observations"
TRACKER = f"{OBS}/state.tracker.json"
STATE = f"{OBS}/state.phase_drift.json"
L1_HZ = 1575.42e6
C_MPS = 299792458.0
MY_BAND = "L1 / WAAS GEO (phase)"

WINDOW_S = 40.0           # sliding fit window; clamped to [30, 60]
MIN_SAMPLES_FRAC = 0.7    # of window length (1 Hz reports)
MIN_CN0 = 30.0            # dB — the tracker's UNLOCK_DB
GAP_S = 5.0               # report gap longer than this breaks the chain
BREAK_CYC = 25.0          # one-step increment deviation = phase break
COLLAPSE_CYC = 50.0       # |accumulator| above this can reseed-collapse
COLLAPSE_FRAC = 0.02      # ...to below this fraction of its magnitude


def log(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def fit_drift(samples):
    """Least-squares slope of (t, cycles) samples.

    Returns (slope_hz, sigma_hz, n) or None when the window is degenerate
    (<3 points or zero time span). sigma_hz is the honest 1-sigma slope
    uncertainty from the fit residuals: sqrt(Σr²/(n-2) / Σ(t-t̄)²)."""
    n = len(samples)
    if n < 3:
        return None
    t0 = samples[0][0]
    xs = [t - t0 for t, _ in samples]
    ys = [c for _, c in samples]
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0.0:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx
    s2 = sum((y - my - slope * (x - mx)) ** 2 for x, y in zip(xs, ys)) / (n - 2)
    return slope, math.sqrt(s2 / sxx), n


def weighted_median(values_sigmas):
    """Sigma-weighted median of (value, sigma) pairs.

    Returns (value, sigma); sigma = max(formal 1/sqrt(Σ 1/σᵢ²),
    weighted scatter about the median) — the scatter term keeps the
    consensus honest when the GEOs genuinely disagree."""
    pts = sorted((v, max(s, 1e-12)) for v, s in values_sigmas)
    w = [1.0 / (s * s) for _, s in pts]
    tot = sum(w)
    acc = 0.0
    med = pts[-1][0]
    for (v, _), wi in zip(pts, w):
        acc += wi
        if acc >= 0.5 * tot:
            med = v
            break
    formal = math.sqrt(1.0 / tot)
    scatter = math.sqrt(sum(wi * (v - med) ** 2 for (v, _), wi in zip(pts, w)) / tot)
    return med, max(formal, scatter)


class SatWindow:
    """Sliding window of continuity-verified (t, cycles, cn0) samples for
    one channel. Any phase break — slip flag, reseed collapse, increment
    jump vs the chain's own median, report gap, epoch going backwards, or
    a discipline-step retune — flushes the window; only samples on one
    unbroken chain may join a fit."""

    def __init__(self, window_s):
        self.window_s = float(window_s)
        self.samples = []          # (t, cycles, cn0) on the current chain
        self.incs = []             # per-step cycle increments on the chain
        self.last = None           # last seen (t, cycles, corr)
        self.lock_s = 0.0          # lock_s of the newest report

    def add(self, t, cycles, slip=False, cn0=None, lock_s=None, corr=None):
        """Ingest one 1 Hz report. Returns True if it joined the chain."""
        if lock_s is not None:
            self.lock_s = lock_s
        if self.last is not None:
            pt, pc, pcorr = self.last
            gap = t - pt
            broken = gap <= 0.0 or gap > GAP_S or slip or corr != pcorr
            if not broken:
                # reseed collapse: the accumulator snapped back to ~0.
                # A GEO Doppler cannot walk |pc| > 50 cycles back to zero
                # in one report — this is a chain break, flag or no flag.
                broken = (abs(pc) > COLLAPSE_CYC
                          and abs(cycles) < COLLAPSE_FRAC * abs(pc))
            if not broken and self.incs:
                # increment vs the chain's OWN median: the reported
                # doppler_hz (FLL state) jitters +/-15 Hz live while the
                # NCO advance wanders ~1 cycle/s, so only the chain's own
                # statistics are a quiet enough reference
                med = sorted(self.incs)[len(self.incs) // 2]
                broken = abs((cycles - pc) - med * gap) > BREAK_CYC
            if broken:
                self.samples = []
                self.incs = []
                self.last = None
                return False         # the breaking sample joins no chain
        elif slip:
            return False             # first sighting is a break — drop it
        if self.last is not None:
            self.incs.append((cycles - pc) / (t - pt))
        self.samples.append((t, cycles, cn0))
        self.last = (t, cycles, corr)
        cut = t - self.window_s - 2.0 * GAP_S
        while self.samples and self.samples[0][0] < cut:
            self.samples.pop(0)
        del self.incs[:-max(1, len(self.samples))]
        return True

    def evaluate(self, min_lock_s, min_cn0, min_samples):
        """Fit the current window. Returns dict(slope_hz, sigma_hz, n) or
        None (window too short, lock younger than the window, or a weak
        C/N0 second inside it)."""
        if not self.samples:
            return None
        t_new = self.samples[-1][0]
        win = [s for s in self.samples if t_new - s[0] <= self.window_s]
        if len(win) < min_samples:
            return None
        if self.lock_s < min_lock_s:
            return None
        if any(cn0 is None or cn0 < min_cn0 for _, _, cn0 in win):
            return None
        fit = fit_drift([(t, c) for t, c, _ in win])
        if fit is None:
            return None
        slope, sigma, n = fit
        return {"slope_hz": slope, "sigma_hz": sigma, "n": n}


def row(band, name, ppm, sigma, epoch, sats, extra=None,
        kind="ClockDriftPpm"):
    r = {
        "band": band,
        "name": name,
        "kind": kind,
        "value": round(ppm, 9), "sigma": round(sigma, 9),
        "ref_hz": L1_HZ, "epoch": round(epoch, 2),
        "sats": sats,
        "anchor": "Pro live track @ 1568.25 MHz",
        "ns_per_s": round(ppm * 1000.0, 6),
        "m_per_s": round(ppm * 1e-6 * C_MPS, 6),
    }
    if extra:
        r.update(extra)
    return r


def process_state(state, windows, window_s, min_lock_s, min_cn0, min_samples):
    """Ingest one tracker state file. Returns (rows, diag): sources rows
    (per-GEO + consensus) and per-sat diagnostics for the phase_drift
    key. `windows` persists across calls (keyed by (sys, prn))."""
    corr = ((state.get("discipline") or {}).get("correction_ppm")) or 0.0
    sats = (state.get("tracker") or {}).get("sats") or []
    file_epoch = state.get("epoch") or time.time()
    votes = []
    diag = {}
    for s in sats:
        sysname = s.get("sys")
        prn = s.get("prn")
        cycles = s.get("carrier_cycles")
        if sysname is None or prn is None or cycles is None:
            continue               # pre-dcfcfaa row: no carrier phase
        t = s.get("epoch") or file_epoch
        key = (sysname, prn)
        w = windows.setdefault(key, SatWindow(window_s))
        w.window_s = window_s
        w.add(t, cycles, slip=bool(s.get("slip")),
              cn0=s.get("cn0_proxy"), lock_s=s.get("lock_s"), corr=corr)
        if sysname != "sbas":
            continue               # MEO slope is orbit-dominated; GEOs only
        ev = w.evaluate(min_lock_s, min_cn0, min_samples)
        if ev is None:
            diag[f"{sysname} {prn}"] = {"ok": False, "lock_s": w.lock_s,
                                        "n": len(w.samples)}
            continue
        ppm = ev["slope_hz"] / L1_HZ * 1e6 + corr
        sig_ppm = ev["sigma_hz"] / L1_HZ * 1e6
        votes.append((ppm, sig_ppm, prn, t))
        diag[f"{sysname} {prn}"] = {
            "ok": True, "n": ev["n"],
            "slope_hz": round(ev["slope_hz"], 6),
            "fit_sigma_ppm": round(sig_ppm, 9),
            "ppm": round(ppm, 9),
        }
    rows = []
    for ppm, sig_ppm, prn, t in votes:
        # components are NOT "ClockDriftPpm": one instrument must vote
        # once in series_producer's cross-producer consensus — N near-
        # identical per-sat rows from the same phase chain would outvote
        # every independent path. The component kind keeps the rows
        # visible in the merged sources table without voting/charting.
        rows.append(row(f"L1 / WAAS {prn} (phase)",
                        f"WAAS PRN {prn} carrier-phase slope {window_s:.0f} s "
                        f"+ corr register · Pro+AA.250",
                        ppm, sig_ppm, t, [f"PRN {prn}"],
                        kind="ClockDriftPpmComponent"))
    if votes:
        med, sig = weighted_median([(v, s) for v, s, _, _ in votes])
        t = max(t for _, _, _, t in votes)
        # Consensus is a component too (systems review 2026-08-25 #6):
        # observe-only until GEO range-rate removal and correlated-
        # residual uncertainty land; it must not double-vote the radio
        # it shares with the WAAS code row.
        rows.append(row(MY_BAND,
                        f"WAAS GEO carrier-phase consensus ({len(votes)} sats, "
                        f"{window_s:.0f} s slope) + corr register · Pro+AA.250 "
                        f"· observe-only",
                        med, sig, t,
                        [f"PRN {prn}" for _, _, prn, _ in votes],
                        extra={"n_sats": len(votes)},
                        kind="ClockDriftPpmComponent"))
    return rows, diag


def publish(rows, diag, now):
    state = {
        "epoch": round(now, 2),
        "ttl_s": 30,
        "phase_drift": diag,
    }
    if rows:
        state["sources"] = rows
    tmp = STATE + ".phase_drift.tmp"
    json.dump(state, open(tmp, "w"), indent=1)
    os.replace(tmp, STATE)


def main():
    ap = argparse.ArgumentParser(description="carrier-phase clock-drift producer")
    ap.add_argument("--window-s", type=float, default=WINDOW_S,
                    help="fit window, seconds (clamped to [30, 60]; "
                         f"default {WINDOW_S:.0f})")
    args = ap.parse_args()
    window_s = min(60.0, max(30.0, args.window_s))
    min_lock_s = window_s          # locked for the whole window
    min_samples = max(10, int(MIN_SAMPLES_FRAC * window_s))
    log(f"phase drift producer starting — {window_s:.0f} s window, "
        f"min {min_samples} samples, cn0>={MIN_CN0:.0f} dB, GEOs only")
    windows = {}
    last_mtime = 0.0
    last_pub = 0.0
    while True:
        try:
            mtime = os.path.getmtime(TRACKER)
        except OSError:
            time.sleep(1.0)
            continue
        now = time.time()
        if mtime != last_mtime:
            last_mtime = mtime
            try:
                with open(TRACKER) as f:
                    state = json.load(f)
            except Exception:
                time.sleep(0.2)    # mid-replace read; retry next tick
                continue
            rows, diag = process_state(state, windows, window_s,
                                       min_lock_s, MIN_CN0, min_samples)
            try:
                publish(rows, diag, now)
                last_pub = now
                nok = sum(1 for d in diag.values() if d.get("ok"))
                if rows:
                    cons = [r for r in rows if r["band"] == MY_BAND]
                    if cons:
                        log(f"{nok} GEOs — consensus {cons[0]['value']:+.9f} ppm "
                            f"(sigma {cons[0]['sigma']:.2e})")
            except Exception as e:
                log(f"publish error: {e}")
        elif now - last_pub >= 5.0:
            # tracker stalled: heartbeat with NO rows so the merged panel
            # ages/drops this source instead of voting a frozen value
            try:
                publish([], {}, now)
                last_pub = now
            except Exception as e:
                log(f"heartbeat error: {e}")
        time.sleep(0.25)


if __name__ == "__main__":
    main()
