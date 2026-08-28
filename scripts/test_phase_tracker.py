#!/usr/bin/env python3
"""Offline validation of the 60 Hz ch35 carrier-phase tracker.

Drives scripts/phase_producer.py's Tracker class — the SAME code path the
live FIFO loop runs — without touching any radio or the live FIFO.

Default mode (no args): synthetic known-truth test. Generates the exact
signal the live producer sees (pilot line near -500 kHz baseband at
FS 6 Msps, C/N0 = 55 dB-Hz matching the real ch35 pilot, int8-clipped
I/Q), injects a KNOWN displacement profile (mm-scale sinusoids + drift
ramp), a live-like reference-frequency wander, and a 0.3 s deep fade,
then asserts:
  1. exactly 60 epochs per second of input,
  2. lock acquired within ~4 s (2 s warmup + 2 s stable),
  3. recovered displacement tracks the injected motion with ~1 mm-class
     per-epoch sigma (robust 2nd-difference statistic),
  4. the frequency steer converges to the injected residual offset,
  5. lock survives the fade and the track resumes afterwards,
  6. SNR floor (2026-08-28 audit): noise-only input never locks or
     publishes, a real-level pilot locks and publishes, and a pilot
     dropping below the floor mid-run unseeds publication within the
     lost-blocks horizon.

Capture mode: --capture PATH --fs-in HZ --line-hz HZ runs the tracker on
a recorded int8 I/Q file (any sample rate; resampled to 6 Msps, line
moved to the nominal -500 kHz slot first) and reports the real-data
per-epoch sigma at 60 Hz. Warmup/lock windows are shortened (1 s each)
because the surviving ch35-era captures are only ~5 s long; the DSP path
is untouched. Example:
  python3 scripts/test_phase_tracker.py --capture /tmp/one_tv_601500000.iq \
      --fs-in 8000000 --line-hz 809440
"""
import argparse
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import phase_producer as pp


def robust_sig_d2(x):
    """Robust per-epoch sigma from the 2nd difference (same statistic the
    producer publishes): MAD std / sqrt(6)."""
    d2 = np.diff(np.asarray(x, dtype=np.float64), 2)
    return float(1.4826 * np.median(np.abs(d2 - np.median(d2))) / np.sqrt(6))


# ---------------- synthetic known-truth mode ----------------

F_RES0 = -285.0          # injected residual offset vs -500 kHz (live-like)
WANDER_A = 0.5           # Hz, sinusoidal reference wander amplitude
WANDER_T = 120.0         # s, wander period (max slope ~0.026 Hz/s ~ live)
CN0_DBHZ = 55.0          # ch35 pilot class (58 dB in a 0.36 Hz FFT bin)
FADE_T = (15.0, 15.3)    # deep fade window (0.3 s < 5 s lock-loss)
DUR_S = 30.0


def motion_mm(t):
    """Injected displacement truth: 2.5 mm @ 0.5 Hz + 0.3 mm @ 2 Hz
    + 0.08 mm/s drift ramp."""
    return (2.5 * np.sin(2 * np.pi * 0.5 * t)
            + 0.3 * np.sin(2 * np.pi * 2.0 * t)
            + 0.08 * t)


def wander_cycles(t, wander_a=WANDER_A):
    """Integral of wander_a*sin(2 pi t / WANDER_T) Hz, in cycles."""
    return wander_a * WANDER_T / (2 * np.pi) * (1 - np.cos(2 * np.pi * t / WANDER_T))


def synth_blocks(dur_s=DUR_S, seed=35, wander_a=WANDER_A, fade=True,
                 pilot=True, dark_after=None):
    """Yield (iq complex64 block, t_epoch, truth_mm at epoch center).

    pilot=False emits pure noise (no pilot anywhere); dark_after=T kills
    the pilot from epoch T on (mid-run drop below the SNR floor)."""
    rate, fs = pp.EPOCH_HZ, pp.FS
    blk = int(fs / rate)
    amp0 = 10.0
    var = amp0 * amp0 * fs / 10 ** (CN0_DBHZ / 10)   # E|n|^2 for target C/N0
    rng = np.random.default_rng(seed)
    n0 = 0
    for _ in range(int(dur_s * rate)):
        n = n0 + np.arange(blk)
        t = n / fs
        phase = (2 * np.pi * (-500e3 + F_RES0) * t
                 + 2 * np.pi * wander_cycles(t, wander_a)
                 + 2 * np.pi * motion_mm(t) / pp.LAMBDA_MM)
        t_c = (n0 + blk / 2) / fs
        amp = amp0 * (0.1 if fade and FADE_T[0] <= t_c < FADE_T[1] else 1.0)
        if not pilot or (dark_after is not None and t_c >= dark_after):
            amp = 0.0
        sig = amp * np.exp(1j * phase)
        noise = (rng.standard_normal(blk) + 1j * rng.standard_normal(blk)) \
            * np.sqrt(var / 2)
        iq = sig + noise
        # int8 quantization like hackrf_transfer's output
        iq = (np.clip(iq.real, -128, 127) + 1j * np.clip(iq.imag, -128, 127))
        yield iq.astype(np.complex64), t_c, float(motion_mm(t_c))
        n0 += blk


def run_synthetic():
    fails = []

    def check(name, ok, detail):
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail}")
        if not ok:
            fails.append(name)

    # theoretical thermal floor: B=30 Hz ENBW, C/N0=55 dB-Hz
    sig_theory = (np.sqrt(30.0 / 10 ** (CN0_DBHZ / 10))
                  / (2 * np.pi) * pp.LAMBDA_MM)
    print(f"  info  thermal theory: {sig_theory:.3f} mm/epoch "
          f"(B=30 Hz ENBW, C/N0={CN0_DBHZ:.0f} dB-Hz)")

    # ---- scenario A: wander + motion + fade (dynamics) ----
    print("scenario A: reference wander + motion + 0.3 s fade")
    tr = pp.Tracker(-500e3 + F_RES0)      # exact initial estimate
    eps = [tr.process(iq, t) for iq, t, _ in synth_blocks(seed=35)]
    truth = np.array([m for _, _, m in synth_blocks(seed=35)])
    ts = np.array([e["t"] for e in eps])

    # 1. epoch rate
    dt = np.diff(ts)
    check("60 Hz epochs", len(eps) == int(DUR_S * pp.EPOCH_HZ)
          and np.max(np.abs(dt - 1 / pp.EPOCH_HZ)) < 1e-9,
          f"{len(eps)} epochs over {DUR_S:.0f} s, dt uniform "
          f"{1/pp.EPOCH_HZ*1e3:.3f} ms")

    # 2. lock acquisition
    lock_idx = next((i for i, e in enumerate(eps) if e["locked"]), None)
    t_lock = ts[lock_idx] if lock_idx is not None else None
    check("lock acquired", t_lock is not None and 3.0 < t_lock < 5.0,
          f"t_lock = {t_lock and round(t_lock, 3)} s "
          f"(2 s warmup + 2 s stable expected)")
    if t_lock is None:
        print("no lock — aborting further checks")
        return False

    # 3. per-epoch noise vs injected truth, pre-fade. The 1-Hz steer loop
    #    has ~2 s group delay, so the injected 0.5 Hz reference wander
    #    shows in the raw residual as a slow A*tau sinusoid (cm-class over
    #    10 s — honest group delay, present in the 20 Hz version too).
    #    The per-epoch sensitivity metric is the 2nd-difference sigma,
    #    which is immune to it by construction.
    disp = np.array([e["disp_mm"] if e["disp_mm"] is not None else np.nan
                     for e in eps])
    wa = (ts >= t_lock + 1.5) & (ts <= FADE_T[0] - 0.2)
    res_a = disp[wa] - (truth[wa] - truth[lock_idx])
    sig_a = robust_sig_d2(res_a)
    check("per-epoch sigma, pre-fade", sig_a < 1.2,
          f"robust 2nd-diff sigma {sig_a:.3f} mm/epoch "
          f"(thermal theory {sig_theory:.3f} mm)")

    # 4. frequency steering converges to the injected (wandering) residual
    for tq in (10.0, 14.0, 25.0):
        eo = next(e for e in eps if e["t"] >= tq and e["freq_off_hz"] is not None)
        f_true = F_RES0 + WANDER_A * np.sin(2 * np.pi * eo["t"] / WANDER_T)
        check(f"f_res steering @ {tq:.0f} s",
              abs(eo["freq_off_hz"] - f_true) < 0.2,
              f"steered {eo['freq_off_hz']:+.3f} Hz vs true "
              f"{f_true:+.3f} Hz at t={eo['t']:.2f} s")

    # 5. fade: epochs flagged, lock survives, track resumes
    dark = [e for e in eps if FADE_T[0] + 0.05 <= e["t"] <= FADE_T[1] - 0.05]
    post = next(e for e in eps if e["t"] >= FADE_T[1] + 0.2)
    check("fade flagged, lock survives",
          all(not e["good"] for e in dark) and post["locked"],
          f"{len(dark)} dark epochs flagged, locked at "
          f"t={post['t']:.2f} s: {post['locked']}")

    wb = (ts >= FADE_T[1] + 1.5) & (ts <= DUR_S - 0.5)
    res_b = disp[wb] - truth[wb]
    res_b = res_b - np.median(res_b)      # fade random-walk jump -> re-offset
    sig_b = robust_sig_d2(res_b)
    check("post-fade track", sig_b < 1.5,
          f"robust 2nd-diff sigma {sig_b:.3f} mm/epoch after refit")

    # producer's own published sigma statistic, settled pre-fade
    e_sig = next(e for e in reversed(eps[:int(FADE_T[0] * pp.EPOCH_HZ)])
                 if e["sigma_mm"] is not None)
    print(f"  info  producer sigma_mm at t={e_sig['t']:.1f} s: "
          f"{e_sig['sigma_mm']:.3f} mm (10 s window)")

    # ---- scenario B: clean motion, no wander/fade (fidelity) ----
    # Without reference wander the steer loop sits still, so the RAW
    # residual vs injected truth must be mm-class outright.
    print("scenario B: clean motion, no wander (absolute fidelity)")
    tr2 = pp.Tracker(-500e3 + F_RES0)
    eps2 = [tr2.process(iq, t)
            for iq, t, _ in synth_blocks(seed=99, wander_a=0.0, fade=False)]
    truth2 = np.array([m for _, _, m in synth_blocks(seed=99, wander_a=0.0,
                                                     fade=False)])
    ts2 = np.array([e["t"] for e in eps2])
    disp2 = np.array([e["disp_mm"] if e["disp_mm"] is not None else np.nan
                      for e in eps2])
    lock2 = next((i for i, e in enumerate(eps2) if e["locked"]), None)
    w2 = ts2 >= ts2[lock2] + 1.5
    res2 = disp2[w2] - (truth2[w2] - truth2[lock2])
    trend2 = np.polyfit(ts2[w2] - ts2[w2][0], res2, 1)   # steer transient ramp
    res2_dt = res2 - np.polyval(trend2, ts2[w2] - ts2[w2][0])
    sig2 = robust_sig_d2(res2)
    p2p2 = float(np.max(np.abs(res2_dt)))
    check("absolute motion recovery", sig2 < 1.2 and p2p2 < 8.0,
          f"sigma {sig2:.3f} mm/epoch, detrended peak {p2p2:.2f} mm "
          f"(noise max + steer response to the motion slope) "
          f"vs injected 2.5 mm @ 0.5 Hz + 0.3 mm @ 2 Hz + ramp")

    print("SYNTHETIC 60 Hz VALIDATION:", "FAIL" if fails else "ALL PASS")
    return not fails


# ---------------- SNR-floor scenarios (2026-08-28 audit fix) ----------------
# The audit found the producer fabricated clock rows with the pilot dark:
# the initial-estimate FFT's max bin in its 4 kHz window is then a NOISE
# PEAK (~9-11 dB over median), there was no SNR floor, and the tracker
# locked on noise (the "-1 ppm One fell off the chain" finding was
# manufactured this way). The fix: acquisitions below pp.SNR_FLOOR_DB
# never seed tracking, and values publish only when locked on an
# above-floor acquisition. These scenarios drive the SAME Tracker the
# live loop runs.

def _check(fails, name, ok, detail):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail}")
    if not ok:
        fails.append(name)


def est_snr(blocks, lo=-502e3, hi=-498e3):
    """The producer's acquisition statistic (max-bin/median over the
    window) on the first ~0.7 s of a block stream — same math as
    phase_producer.estimate_freq, stream-IO-free."""
    iq = np.concatenate([b for b, _, _ in blocks[:42]])   # >= 1<<22 samples
    return find_line(iq, pp.FS, lo, hi)


def scenario_noise_only():
    """(a) No pilot anywhere: the acquisition statistic must read
    sub-floor, and a tracker gated on it must never lock or emit a
    value-carrying epoch."""
    fails = []
    dur = 20.0
    blocks = list(synth_blocks(dur_s=dur, seed=7, fade=False, pilot=False))
    f, snr = est_snr(blocks)
    _check(fails, "noise acquisition below floor", snr < pp.SNR_FLOOR_DB,
           f"max-bin SNR {snr:.1f} dB < floor {pp.SNR_FLOOR_DB:.0f} dB "
           f"(peak at {f:+.1f} Hz is a noise bin, not the pilot)")
    tr = pp.Tracker(f, acq_snr_db=snr)
    eps = [tr.process(iq, t) for iq, t, _ in blocks]
    _check(fails, "noise-only never locks",
           not any(e["locked"] for e in eps),
           f"0 of {len(eps)} epochs locked")
    _check(fails, "noise-only publishes no values",
           all(e["disp_mm"] is None and e["freq_off_hz"] is None
               and e["ppm"] is None and e["sigma_mm"] is None for e in eps),
           "every epoch is the None dark-heartbeat shape")
    print("scenario SNR-floor (a) noise-only:", "FAIL" if fails else "ALL PASS")
    return not fails


def scenario_real_pilot():
    """(b) A pilot at the real level: acquisition clears the floor, the
    tracker locks, and locked epochs carry values."""
    fails = []
    dur = 12.0
    blocks = list(synth_blocks(dur_s=dur, seed=11, fade=False))
    f, snr = est_snr(blocks)
    _check(fails, "pilot acquisition above floor", snr >= pp.SNR_FLOOR_DB,
           f"max-bin SNR {snr:.1f} dB >= floor {pp.SNR_FLOOR_DB:.0f} dB "
           f"(real acquisitions measure 15-22 dB)")
    tr = pp.Tracker(f, acq_snr_db=snr)
    eps = [tr.process(iq, t) for iq, t, _ in blocks]
    ts = np.array([e["t"] for e in eps])
    lock_idx = next((i for i, e in enumerate(eps) if e["locked"]), None)
    t_lock = ts[lock_idx] if lock_idx is not None else None
    _check(fails, "real-level pilot locks", t_lock is not None and t_lock < 5.0,
           f"t_lock = {t_lock and round(t_lock, 2)} s")
    locked = [e for e in eps if e["locked"]]
    _check(fails, "locked epochs publish values",
           bool(locked) and all(e["disp_mm"] is not None
                                and e["freq_off_hz"] is not None
                                and e["ppm"] is not None for e in locked)
           and any(e["sigma_mm"] is not None for e in locked),
           f"{len(locked)} locked epochs, all with disp/freq/ppm, "
           f"sigma live after the 100-epoch warmup")
    print("scenario SNR-floor (b) real-level pilot:",
              "FAIL" if fails else "ALL PASS")
    return not fails


def scenario_midrun_drop():
    """(c) Pilot drops below the floor mid-run: the existing amplitude
    path must drop the lock inside the lost-blocks horizon (5 s) and
    values must stop; the dead pilot re-estimates sub-floor, so a
    re-acquire would refuse to re-seed."""
    fails = []
    drop_t, dur = 15.0, 30.0
    blocks = list(synth_blocks(dur_s=dur, seed=23, fade=False,
                               dark_after=drop_t))
    f, snr = est_snr(blocks)                     # pre-drop: real pilot
    tr = pp.Tracker(f, acq_snr_db=snr)
    eps = [tr.process(iq, t) for iq, t, _ in blocks]
    t_lock = next((e["t"] for e in eps if e["locked"]), None)
    _check(fails, "locks before the drop",
           t_lock is not None and t_lock < 5.0,
           f"t_lock = {t_lock and round(t_lock, 2)} s")
    t_lost = next((e["t"] for e in eps if e["event"] == "lost"), None)
    horizon = tr.lost_blocks / tr.rate
    _check(fails, "lock lost within the lost-blocks horizon",
           t_lost is not None and drop_t < t_lost <= drop_t + horizon + 0.5,
           f"drop at {drop_t:.0f} s, lost at "
           f"{t_lost and round(t_lost, 2)} s (horizon {horizon:.0f} s)")
    if t_lost is not None:
        after = [e for e in eps if e["t"] >= t_lost]
        _check(fails, "publication unseeds after the drop",
               after and not any(e["locked"] for e in after)
               and all(e["disp_mm"] is None and e["ppm"] is None
                       and e["freq_off_hz"] is None for e in after),
               f"{len(after)} post-loss epochs: unlocked, all None values")
    f_dark, snr_dark = est_snr(blocks[int((drop_t + 1.0) * pp.EPOCH_HZ):])
    _check(fails, "dark pilot re-estimates sub-floor",
           snr_dark < pp.SNR_FLOOR_DB,
           f"re-acquire would refuse to seed (SNR {snr_dark:.1f} dB < "
           f"floor {pp.SNR_FLOOR_DB:.0f} dB, peak at {f_dark:+.1f} Hz)")
    print("scenario SNR-floor (c) mid-run drop:", "FAIL" if fails else "ALL PASS")
    return not fails


def run_snr_floor():
    print("SNR-floor scenarios (2026-08-28 audit: noise locks fabricated "
          "clock rows)")
    ok = [scenario_noise_only(), scenario_real_pilot(), scenario_midrun_drop()]
    print("SNR-FLOOR VALIDATION:", "ALL PASS" if all(ok) else "FAIL")
    return all(ok)


# pytest entry points (the synthetic/capture suites stay script-driven;
# the SNR-floor scenarios are the regression net for the 2026-08-28 fix)

def test_noise_only_never_seeds():
    assert scenario_noise_only()


def test_real_pilot_locks_and_publishes():
    assert scenario_real_pilot()


def test_midrun_drop_unseeds_publication():
    assert scenario_midrun_drop()


# ---------------- recorded-capture mode ----------------

def find_line(iq, fs, lo, hi):
    n = min(len(iq), 1 << 22)
    w = iq[:n].astype(np.complex64) * np.hanning(n).astype(np.float32)
    spec = np.abs(np.fft.fftshift(np.fft.fft(w))) ** 2
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1 / fs))
    m = (freqs >= lo) & (freqs <= hi)
    idx = np.flatnonzero(m)
    i = idx[int(np.argmax(spec[m]))]
    y0, y1, y2 = spec[i - 1], spec[i], spec[i + 1]
    denom = y0 - 2 * y1 + y2
    frac = 0.5 * (y0 - y2) / denom if denom > 0 else 0.0
    f = float(freqs[i] + frac * (freqs[1] - freqs[0]))
    snr = float(10 * np.log10(y1 / np.median(spec)))
    return f, snr


def run_capture(path, fs_in, line_hz):
    from fractions import Fraction
    from scipy.signal import resample_poly

    raw = np.fromfile(path, dtype=np.int8)
    iq = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)
    dur = len(iq) / fs_in
    print(f"capture: {len(iq)} samples = {dur:.1f} s at {fs_in/1e6:.1f} Msps")

    f_meas, snr = find_line(iq, fs_in, line_hz - 2e3, line_hz + 2e3)
    print(f"pilot line at {f_meas:+.2f} Hz baseband "
          f"({f_meas - line_hz:+.2f} Hz vs expected), SNR {snr:.0f} dB")

    # resample to the producer's FS (absolute frequencies are preserved),
    # then move the line into the nominal -500 kHz slot so the deployed
    # Tracker path applies unchanged
    r = Fraction(int(pp.FS), int(fs_in))
    if r.numerator != r.denominator:
        iq = resample_poly(iq, r.numerator, r.denominator).astype(np.complex64)
    t = np.arange(len(iq), dtype=np.float64) / pp.FS
    iq = iq * np.exp(-1j * 2 * np.pi * (f_meas + 500e3)
                     * t).astype(np.complex64)
    f_line, _ = find_line(iq, pp.FS, -502e3, -498e3)
    print(f"shifted to {f_line:+.2f} Hz (nominal -500 kHz slot)")

    # short captures (5 s) can't afford the live 2 s + 2 s windows;
    # DSP path identical, only the warmup/lock bookkeeping shortened
    tr = pp.Tracker(f_line, warmup_s=1.0, lock_s=1.0)
    eps = []
    for b in range(len(iq) // tr.block):
        eps.append(tr.process(iq[b * tr.block:(b + 1) * tr.block],
                              (b + 0.5) * tr.block / pp.FS))
    ts = np.array([e["t"] for e in eps])
    disp = np.array([e["disp_mm"] if e["disp_mm"] is not None else np.nan
                     for e in eps])
    locked = np.array([e["disp_mm"] is not None for e in eps])
    t_lock = next((e["t"] for e in eps if e["locked"]), None)
    good = ~np.isnan(disp)
    sig = robust_sig_d2(disp[good]) if good.sum() > 10 else float("nan")
    e_sig = next((e for e in reversed(eps) if e["sigma_mm"] is not None), None)
    print(f"REAL CAPTURE @ {pp.EPOCH_HZ} Hz: {len(eps)} epochs, "
          f"lock at {t_lock and round(t_lock, 2)} s, "
          f"{int(locked.sum())} good epochs")
    print(f"  per-epoch sigma (robust 2nd diff, {good.sum()} epochs): "
          f"{sig:.3f} mm")
    if e_sig:
        print(f"  producer sigma_mm at t={e_sig['t']:.2f} s: "
              f"{e_sig['sigma_mm']:.3f} mm")
    if eps[-1]["freq_off_hz"] is not None:
        print(f"  freq_off {eps[-1]['freq_off_hz']:+.2f} Hz "
              f"({eps[-1]['ppm']:+.4f} ppm)")
    ok = t_lock is not None and good.sum() >= 60 and np.isfinite(sig)
    print("CAPTURE VALIDATION:", "PASS" if ok else "FAIL")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--capture", help="recorded int8 I/Q file to validate on")
    ap.add_argument("--fs-in", type=float, default=8e6,
                    help="capture sample rate (default 8 Msps)")
    ap.add_argument("--line-hz", type=float, default=809.44e3,
                    help="expected pilot line freq in capture baseband")
    args = ap.parse_args()
    if args.capture:
        ok = run_capture(args.capture, args.fs_in, args.line_hz)
    else:
        ok = run_synthetic()
        ok = run_snr_floor() and ok
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
