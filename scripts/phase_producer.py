#!/usr/bin/env python3
"""60 Hz carrier-phase producer for the /sync panel ("phase producer").

Tracks the ATSC ch35 pilot (true 602.30944 MHz, GPS-disciplined Tx,
~+43 dB over noise on the ClearStream) on the HackRF One, which is
cabled CLKOUT→CLKIN to the Pro's 10 MHz. The cable is in, but the lock
is NOT verified: this producer holds the One full-time, so the
CLKIN-detection read can't open it (clkin_signal_present stays null).
Until a verified lock exists, ATSC rows are labeled cabled-but-unverified
and excluded from the clock consensus vote. One continuous
hackrf_transfer is held for the producer's whole lifetime, tuned
500 kHz above the pilot so the line sits at -500 kHz (off the DC spike).
Samples arrive over a FIFO (mkfifo) — never a spooling file — so disk
usage is zero regardless of run length.

Epoch rate: 60 Hz (was 20 Hz). FS dropped 8 -> 6 Msps so a 60 Hz epoch is
an integer 100000-sample block (16.67 ms); USB load drops 25% as a bonus.
Per-epoch noise bandwidth: one epoch is a coherent average over
T_b = 1/60 s -> equivalent noise bandwidth B = 1/(2 T_b) = 30 Hz. With the
ch35 pilot at C/N0 ~ 54-55 dB-Hz (58 dB over the median 0.36 Hz acquire-FFT
bin), sigma_phi = sqrt(B / (C/N0)) ~ sqrt(30 / 3e5) ~ 0.010 rad ->
sigma_d = 0.010 / (2 pi) * 497.7 mm ~ 0.8 mm per epoch (thermal). The
robust 2nd-difference sigma reported live adds the mm-class multipath
jiggle on top. Dynamics are not a constraint: multipath/Tx wander is
<< 1 Hz, far inside a 30 Hz ENBW; the 2-s phase-slope integral steer
(1 Hz updates) keeps the derotation centered on the wandering reference
chain. Validated offline by scripts/test_phase_tracker.py (synthetic
known-truth motion + a recorded ch35 capture) — see that harness's
numbers before trusting a deploy.

Per 16.67 ms block (100k samples, 60 epochs/s):
  - derotate the pilot to baseband with a continuous phase accumulator
    (block boundaries are seamless),
  - block-average to one complex point, atan2 -> phase, unwrap across
    blocks,
  - displacement = unwrapped cycles * lambda (lambda = c/f = 497.7 mm),
  - residual frequency offset: the derotation is steered by the phase
    slope (integral control, 1-s updates) because the reference chain
    wanders ~1 Hz/min; the steered total vs nominal is the sub-Hz
    clock-drift observable (~100x finer than the FFT method),
  - per-epoch sigma: robust MAD std of the 2nd difference of the
    displacement epochs (drift/curvature and fade cycle-slips removed),
  - lock quality = block-mean magnitude (pilot amplitude); epochs where
    it collapses are flagged/dropped, sustained collapse drops the lock.

If the transfer dies (USB hiccup), the stream is reopened and the phase
re-locked: unwrap reference reset, lock=false until 2 s of stable
amplitude.

Acquisition SNR floor (SNR_FLOOR_DB, 2026-08-28 audit): the initial
estimate AND every re-acquire must measure max-bin/median >= the floor or
the producer refuses to seed — sub-floor means the pilot is dark and the
FFT window max is a noise peak. Dark epochs keep the lock:false
None-heartbeat; no phase/history/residual value is published unless the
tracker is locked on an above-floor acquisition.

Publishes ONLY its own keys to its OWN file,
observations/state.phase.json (the Rust server deep-merges all
observations/state.*.json with the legacy sync_state.json at /api/sync
read time, so no shared-file race is possible):
  state["phase"]  — epoch/rate/lambda/disp/sigma/freq_off/lock/series
  sources row band "ATSC ch35" (kind ClockDriftPpm) — this REPLACES the
  old FFT-based ch35 row band_producer used to write (that code was
  removed; the phase producer owns the One full-time now).
  state["clock"]["residual_ppm"] — kept live from this measurement.
Appends observations/phase_history.jsonl at ~1 Hz.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque

import numpy as np

TOOLS = "/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src"
ENV = dict(os.environ,
           DYLD_LIBRARY_PATH="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/libhackrf/src")
ONE = "0000000000000000922c63dc21748847"
F_PILOT = 602.30944e6
F_TUNE = F_PILOT + 500e3          # pilot line lands at -500 kHz
FS = 6e6
EPOCH_HZ = 60                     # epochs/s; FS / EPOCH_HZ must be integral
C_MPS = 299792458.0
LAMBDA_MM = C_MPS / F_PILOT * 1000.0
DEC = 50                          # segment decimation; must divide the block
FIFO = "/tmp/phase_producer.iq"
STATE = "/Volumes/Radiator 8TB/gnss/observations/state.phase.json"
HIST = "/Volumes/Radiator 8TB/gnss/observations/phase_history.jsonl"
SERIES_STATE = "/Volumes/Radiator 8TB/gnss/observations/state.series.json"


def _clkin_label():
    """Anchor label from series_producer's drift-lock verdict (round-14):
    true -> measured drift-lock; false -> evidence AGAINST the chain; null/
    unreadable -> unverified. This producer never opens a radio for it."""
    try:
        with open(SERIES_STATE) as f:
            v = json.load(f).get("clkin_soft_verified")
    except Exception:
        v = None
    return ("drift-locked (soft-verified)" if v is True
            else "chain drift evidence NEGATIVE" if v is False
            else "lock unverified")
MY_BAND = "ATSC ch35"
EST_SAMPLES = 1 << 24             # 2.8 s coherent FFT @ 6 Msps for initial freq
AMP_DROP = 0.35                   # epoch low-flag: amp < 35% of running median
AMP_LOST = 0.20                   # sustained below 20% of median -> lock lost
# Acquisition SNR floor (2026-08-28 audit). estimate_freq's statistic is
# max-bin/median over the 4 kHz pilot window (~1.3e3 effectively
# independent Rayleigh power bins): with the pilot DARK the window max is
# a pure noise peak — expected max/median ~ln(N)/ln2 ~ 10 dB, observed
# 9-11 dB, p99 ~11.5 dB. Real pilot acquisitions measure 15-22 dB in the
# same statistic (07:16 today: 22 dB, consistent with the C/N0 ~ 54 dB-Hz
# above). 14 dB splits the two with >2 dB margin on both sides. Seeding
# below it is what manufactured the fabricated "-1 ppm One fell off the
# clock chain" finding: the tracker locked on a noise bin and published
# random-walk phase as clock drift. Below the floor we do not seed, and
# no value is published unless locked on an above-floor acquisition.
SNR_FLOOR_DB = 14.0

_proc = None                      # current hackrf_transfer child


def log(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def one_busy():
    """True if a hackrf_transfer command line mentions the One's serial."""
    try:
        out = subprocess.run(["pgrep", "-fl", "hackrf_transfer"],
                             capture_output=True, text=True).stdout
    except Exception:
        return False
    return any(ONE in line and str(os.getpid()) not in line
               for line in out.splitlines())


def wait_for_one():
    while one_busy():
        log("another hackrf_transfer holds the One — waiting")
        time.sleep(3)


def open_stream():
    """Launch one continuous transfer into the FIFO; return (proc, fd)."""
    global _proc
    wait_for_one()
    try:
        os.mkfifo(FIFO)
    except FileExistsError:
        pass
    cmd = [f"{TOOLS}/hackrf_transfer", "-d", ONE, "-f", str(int(F_TUNE)),
           "-s", str(int(FS)), "-l", "40", "-g", "44", "-a", "0", "-r", FIFO]
    log("launching: " + " ".join(cmd))
    proc = subprocess.Popen(cmd, env=ENV,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    _proc = proc
    # open non-blocking so a transfer that fails to start can't wedge us
    fd = os.open(FIFO, os.O_RDONLY | os.O_NONBLOCK)
    for _ in range(60):
        if proc.poll() is not None:
            os.close(fd)
            raise RuntimeError(f"hackrf_transfer exited immediately "
                               f"(rc={proc.returncode})")
        r, _, _ = __import__("select").select([fd], [], [], 0.5)
        if r:
            break
    else:
        os.close(fd)
        raise RuntimeError("no data on FIFO within 30 s of launch")
    os.set_blocking(fd, True)
    return proc, fd


def read_into(fd, buf, nbytes):
    """Top up buf to nbytes; return False on EOF (writer died)."""
    while len(buf) < nbytes:
        try:
            chunk = os.read(fd, nbytes - len(buf))
        except InterruptedError:
            continue
        if not chunk:
            return False
        buf.extend(chunk)
    return True


class StreamDead(Exception):
    pass


def take(buf, nbytes):
    out = bytes(buf[:nbytes])
    del buf[:nbytes]
    return out


def to_iq(raw):
    d = np.frombuffer(raw, dtype=np.int8).astype(np.float32)
    return d[0::2] + 1j * d[1::2]


def estimate_freq(fd, buf, blk_bytes):
    """Coherent FFT of the first EST_SAMPLES: precise pilot line frequency
    in baseband (nominally -500 kHz). Also discards 0.5 s of AGC settling.

    The FFT runs in a worker thread while the main thread keeps draining
    the FIFO: hackrf_transfer exits if it can't move bytes for 1 s, and a
    multi-second numpy burst on the read path (worse under CPU contention
    from band_producer's acquisitions) trips exactly that watchdog."""
    if not read_into(fd, buf, int(FS * 0.5) * 2):
        raise StreamDead()
    del buf[: int(FS * 0.5) * 2]
    if not read_into(fd, buf, EST_SAMPLES * 2):
        raise StreamDead()
    raw = take(buf, EST_SAMPLES * 2)
    result = {}

    def work():
        try:
            iq = to_iq(raw)
            n = EST_SAMPLES
            spec = np.abs(np.fft.fftshift(np.fft.fft(iq * np.hanning(n)))) ** 2
            freqs = np.fft.fftshift(np.fft.fftfreq(n, 1 / FS))
            m = (freqs >= -502e3) & (freqs <= -498e3)
            idx = np.flatnonzero(m)
            i = idx[int(np.argmax(spec[m]))]
            y0, y1, y2 = spec[i - 1], spec[i], spec[i + 1]
            denom = y0 - 2 * y1 + y2
            frac = 0.5 * (y0 - y2) / denom if denom > 0 else 0.0
            f_line = float(freqs[i] + frac * (freqs[1] - freqs[0]))
            snr_db = float(10 * np.log10(y1 / np.median(spec)))
            result.update(f=f_line, snr=snr_db)   # single atomic publish
        except Exception as e:
            result["err"] = repr(e)

    threading.Thread(target=work, daemon=True).start()
    while not result:
        if not read_into(fd, buf, blk_bytes):
            raise StreamDead("FIFO EOF during initial estimate")
        del buf[:blk_bytes]
    if "err" in result:
        raise RuntimeError(f"estimate FFT failed: {result['err']}")
    f_line, snr_db = result["f"], result["snr"]
    log(f"initial estimate: pilot line at {f_line:.2f} Hz baseband "
        f"(offset {f_line + 500e3:+.2f} Hz = {(f_line + 500e3) / F_PILOT * 1e6:+.4f} ppm), "
        f"SNR {snr_db:.0f} dB")
    return f_line, snr_db


class Tracker:
    """Stream-IO-free ch35 pilot carrier-phase tracker: one process() call
    per epoch block. The live FIFO loop below and the offline harness
    (scripts/test_phase_tracker.py) drive the IDENTICAL code path, so the
    harness validates exactly what gets deployed.

    Two-stage derotation (fast path, ~1 ms/block vs ~15 ms for a full-rate
    complex128 exp — the consumer must stay well under the 16.67 ms block
    budget or the FIFO backpressures the USB stream and drops samples):
      1. coarse mix with ONE precomputed block-length complex64 LUT at
         f_coarse = round(-500 kHz / rate) * rate — an integer number of
         cycles per block BY CONSTRUCTION, so the same LUT is seamless at
         block boundaries for any epoch rate that divides FS. (The old
         16-entry-tile trick only worked because 500 kHz * 16 / 8 MHz == 1
         exactly; at 6 Msps it would slip 1/3 cycle per 16 samples.)
         report_offset shifts the reported freq_off back into the
         nominal -500 kHz frame (20 Hz at 60 Hz/6 Msps, 0 at 20 Hz/8 Msps).
      2. decimate by DEC (segment means), fine-rotate at the residual
         f_res (~-305 Hz, steered) with a float64 phase accumulator.

    acq_snr_db is the acquisition estimate's SNR. Below SNR_FLOOR_DB the
    "line" is a noise peak (pilot dark): the tracker then never locks and
    every epoch is the dark None-heartbeat shape. (The live loop also
    refuses to seed sub-floor; this gate keeps the same law inside the
    stream-IO-free path the offline harness drives.)
    """

    def __init__(self, f_line, fs=FS, rate_hz=EPOCH_HZ, dec=DEC,
                 warmup_s=2.0, lock_s=2.0, lost_s=5.0, dark_s=60.0,
                 acq_snr_db=None):
        blk = fs / rate_hz
        if blk != int(blk):
            raise ValueError(f"rate {rate_hz} Hz at FS {fs}: block "
                             f"{blk} samples not integral")
        blk = int(blk)
        if blk % dec:
            raise ValueError(f"block {blk} not a multiple of DEC={dec}")
        self.fs = float(fs)
        self.rate = rate_hz
        self.block = blk
        self.dec = dec
        self.nseg = blk // dec
        self.f_coarse = round(-500e3 / rate_hz) * rate_hz
        self.report_offset = 500e3 + self.f_coarse
        self.f_res = f_line - self.f_coarse
        self.fine_step = 2 * np.pi * self.f_res / (self.fs / dec)
        self.phi0f = 0.0
        self.lut = np.exp(-1j * 2 * np.pi * self.f_coarse / self.fs
                          * np.arange(blk)).astype(np.complex64)
        self.seg_ar = np.arange(self.nseg, dtype=np.float64) + 0.5
        self.warmup_blocks = max(1, int(round(warmup_s * rate_hz)))
        self.lock_blocks = max(1, int(round(lock_s * rate_hz)))
        self.lost_blocks = max(1, int(round(lost_s * rate_hz)))
        self.dark_reacq_blocks = max(1, int(round(dark_s * rate_hz)))
        self.amps = deque(maxlen=10 * rate_hz)      # 10 s amplitude history
        self.amp_med = None
        self.unwrapped = 0.0                # radians
        self.prev_phi = None
        self.unwrap_ref = None              # radians at lock moment
        self.locked = False
        self.stable = 0                     # consecutive good-amplitude blocks
        self.collapsed = 0                  # consecutive collapsed blocks
        self.dark_blocks = 0                # consecutive not-good blocks
        self.phase10 = deque()              # (t, cycles) for slope, 10 s
        self.disp10 = deque()               # (t, disp_mm) for sigma, 10 s
        self.last_steer = 0.0
        self.acq_snr_db = acq_snr_db
        # None = acquisition SNR unknown (offline capture harness): the
        # amplitude path alone decides lock, the legacy behavior.
        self.acq_ok = acq_snr_db is None or acq_snr_db >= SNR_FLOOR_DB

    def process(self, iq, t):
        """One block of complex64 samples (len == self.block) at epoch time
        t (block center, seconds). Returns a per-epoch dict; disp_mm is
        None on dark/pre-lock epochs (caller heartbeats those)."""
        if not self.acq_ok:
            # seeded on a sub-floor acquisition (pilot dark): never lock,
            # never emit a value — permanent dark/pre-lock epoch
            return {"t": t, "amp": 0.0, "good": False, "locked": False,
                    "event": None, "dark_reacq": False,
                    "disp_mm": None, "sigma_mm": None,
                    "freq_off_hz": None, "ppm": None}
        mixed = iq * self.lut                       # pilot -> ~f_res
        seg = mixed.reshape(self.nseg, self.dec).mean(axis=1)
        ang = (self.phi0f + self.fine_step * self.seg_ar) % (2 * np.pi)
        self.phi0f = float((self.phi0f + self.fine_step * self.nseg)
                           % (2 * np.pi))
        z = np.mean(seg * np.exp(-1j * ang))
        amp = float(abs(z))
        phi = float(np.arctan2(z.imag, z.real))

        if self.prev_phi is not None:
            self.unwrapped += (phi - self.prev_phi + np.pi) % (2 * np.pi) - np.pi
        self.prev_phi = phi

        amps = self.amps
        amps.append(amp)
        if self.amp_med is None and len(amps) >= self.warmup_blocks:
            self.amp_med = float(np.median(amps))
        elif (self.amp_med is not None and len(amps) >= amps.maxlen
                and amp > AMP_LOST * self.amp_med):
            # adapt only from healthy blocks — following a collapse
            # down to 0 makes `good`/`collapsed` vacuous (0 > 0.6*0)
            # and the loop "locks" on noise forever (the 2026-08-24
            # stall: amp 0.0/0.0 with lock True for 40+ min)
            self.amp_med = float(np.median(amps))

        good = self.amp_med is None or amp > AMP_DROP * self.amp_med
        if self.amp_med is not None and amp < AMP_LOST * self.amp_med:
            self.collapsed += 1
        else:
            self.collapsed = 0
        event = None
        if self.locked and self.collapsed >= self.lost_blocks:
            self.locked = False
            self.stable = 0
            self.unwrap_ref = None
            event = "lost"
        if not self.locked and self.amp_med is not None:
            self.stable = self.stable + 1 if good else 0
            if self.stable >= self.lock_blocks:
                self.locked = True
                self.unwrap_ref = self.unwrapped
                event = "lock"

        # 60 s of dark with a live stream means the TRACKING state is
        # lost (mis-steered derotation), not the signal — the pilot is
        # 58 dB SNR; only a full FFT re-acquisition recovers. The caller
        # forces it through the outer reopen loop (stream keeps running).
        self.dark_blocks = self.dark_blocks + 1 if not good else 0
        dark_reacq = self.dark_blocks >= self.dark_reacq_blocks

        ep = {"t": t, "amp": amp, "good": good, "locked": self.locked,
              "event": event, "dark_reacq": dark_reacq,
              "disp_mm": None, "sigma_mm": None,
              "freq_off_hz": None, "ppm": None}
        if not good or self.unwrap_ref is None:
            return ep

        disp_mm = (self.unwrapped - self.unwrap_ref) / (2 * np.pi) * LAMBDA_MM
        cycles = self.unwrapped / (2 * np.pi)
        self.phase10.append((t, cycles))
        self.disp10.append((t, disp_mm))
        while self.phase10 and t - self.phase10[0][0] > 10.5:
            self.phase10.popleft()
        while self.disp10 and t - self.disp10[0][0] > 10.5:
            self.disp10.popleft()

        # residual frequency: steer the fine derotation with the
        # 2-s phase slope once a second (the reference chain wanders
        # ~1 Hz/min and multipath jitters the phase at the 1-s scale;
        # a fixed derotation would integrate that into a runaway
        # displacement ramp and eventually break the unwrap).
        # Reported freq_off = the steered residual itself.
        recent = [p for p in self.phase10 if t - p[0] <= 2.5]
        if len(recent) >= int(2.0 * self.rate) and t - self.last_steer >= 1.0:
            self.last_steer = t
            tt = np.array([p[0] for p in recent])
            cc = np.array([p[1] for p in recent])
            slope2 = float(np.polyfit(tt - tt[0], cc, 1)[0])
            self.f_res += slope2
            self.fine_step = 2 * np.pi * self.f_res / (self.fs / self.dec)
        freq_off_hz = self.f_res + self.report_offset
        ppm = freq_off_hz / F_PILOT * 1e6

        # per-epoch sensitivity: robust std of the 2nd difference
        # (kills drift/curvature, immune to fade cycle-slips);
        # for white per-epoch noise std(d2)/sqrt(6) = sigma_epoch
        sigma_mm = None
        if len(self.disp10) >= max(100, int(2.0 * self.rate)):
            dd = np.array([p[1] for p in self.disp10])
            d2 = np.diff(dd, 2)
            sigma_mm = float(1.4826 * np.median(np.abs(d2 - np.median(d2)))
                             / np.sqrt(6))

        ep.update(disp_mm=disp_mm, sigma_mm=sigma_mm,
                  freq_off_hz=freq_off_hz, ppm=ppm, f_res=self.f_res)
        return ep


def merge_state(phase, row, ppm):
    # ONLY this producer's keys go to its own file; the server merges
    # per-band "sources" rows and shallow-merges "clock" across files, so
    # nobody read-modify-writes shared state anymore.
    state = {
        "sources": [row],
        "phase": phase,
        "epoch": phase["epoch"],
        # the panel's "residual (live, from GPS-disciplined ATSC pilots)" line
        # is this measurement now — band_producer no longer touches the One
        "clock": {"residual_ppm": round(ppm, 3)},
    }
    # NOTE: the old atsc_spread_ppm cleanup is gone — that key only ever
    # lived in the shared file, and band_producer no longer writes it.
    tmp = STATE + ".phase.tmp"      # unique tmp; atomic publish via os.replace
    json.dump(state, open(tmp, "w"), indent=1)
    os.replace(tmp, STATE)


def shutdown(*_):
    try:
        if _proc and _proc.poll() is None:
            _proc.terminate()
    except Exception:
        pass
    sys.exit(0)


def main():
    ap = argparse.ArgumentParser(description="ch35 pilot carrier-phase producer")
    ap.add_argument("--rate-hz", type=int, default=EPOCH_HZ,
                    help=f"epochs/s (must divide FS={int(FS)} with a "
                         f"DEC={DEC}-multiple block; default {EPOCH_HZ})")
    args = ap.parse_args()
    rate = args.rate_hz
    blk = FS / rate
    if blk != int(blk) or int(blk) % DEC:
        sys.exit(f"--rate-hz {rate}: block {blk} samples at FS {int(FS)} "
                 f"is not integral or not a multiple of DEC={DEC}")
    blk = int(blk)
    blk_bytes = blk * 2               # interleaved int8 I/Q

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    log(f"phase producer starting — ch35 pilot {F_PILOT/1e6:.5f} MHz, "
        f"lambda {LAMBDA_MM:.2f} mm, {rate} epochs/s, One {ONE}")

    while True:
        proc, fd, buf = None, None, None
        try:
            proc, fd = open_stream()
            buf = bytearray()
            last_row = None                 # last published drift row (heartbeat)
            last_ppm = 0.0
            # Acquisition floor — this estimate runs at startup AND on
            # every re-acquire (stream died / pilot dark 60 s), so one
            # gate covers both. Sub-floor = pilot dark, the window max is
            # a noise peak: never seed tracking, heartbeat dark, retry the
            # estimate on its normal cadence (~3.3 s: 0.5 s AGC discard +
            # 2.8 s coherent capture; the FIFO keeps draining meanwhile).
            while True:
                f_line, snr_db = estimate_freq(fd, buf, blk_bytes)
                if snr_db >= SNR_FLOOR_DB:
                    break
                log(f"pilot dark (SNR {snr_db:.1f} dB < floor "
                    f"{SNR_FLOOR_DB:.0f} dB) — not seeding")
                if last_row is not None:
                    phase = {
                        "epoch": round(time.time(), 2), "rate_hz": rate,
                        "lambda_mm": round(LAMBDA_MM, 1),
                        "disp_mm": None, "sigma_mm": None,
                        "freq_off_hz": None, "lock": False,
                        "series": [],
                    }
                    try:
                        merge_state(phase, last_row, last_ppm)
                    except Exception as e:
                        log(f"dark heartbeat publish failed: {e}")
            tracker = Tracker(f_line, rate_hz=rate, acq_snr_db=snr_db)
            t0 = time.time()                # wall clock at estimate end
            n_done = 0                      # samples tracked since t0
            ring = deque(maxlen=60 * rate)  # 60 s of (t, disp_mm)
            last_pub = 0.0

            while True:
                if proc.poll() is not None:
                    raise StreamDead(f"transfer exited rc={proc.returncode}")
                if not read_into(fd, buf, blk_bytes):
                    raise StreamDead("FIFO EOF — writer died")
                iq = to_iq(take(buf, blk_bytes))
                t = t0 + (n_done + blk / 2) / FS
                n_done += blk
                ep = tracker.process(iq, t)

                if ep["dark_reacq"]:
                    raise StreamDead("pilot dark 60 s — full re-acquire")
                if ep["event"] == "lost":
                    log(f"LOCK LOST — pilot amplitude collapsed "
                        f"(amp {ep['amp']:.1f} vs median {tracker.amp_med:.1f})")
                elif ep["event"] == "lock":
                    log(f"LOCK — amp median {tracker.amp_med:.1f}, phase ref reset")

                # Publication law (2026-08-28 audit): values — the state
                # phase/series block, the ATSC sources row, residual_ppm,
                # and history rows — flow ONLY when locked on an
                # above-floor acquisition. Dark/unlocked/sub-floor epochs
                # take the None heartbeat below; nothing fabricated.
                if ep["disp_mm"] is None or not (ep["locked"]
                                                 and tracker.acq_ok):
                    # heartbeat while dark: a monitor that goes silent
                    # exactly when the signal is lost displays its last
                    # "LOCKED" epoch forever. Publish lock:false with a
                    # FRESH epoch once a second; the drift row keeps its
                    # old epoch so the panel ages it stale — the truth.
                    if t - last_pub >= 1.0 and last_row is not None:
                        last_pub = t
                        phase = {
                            "epoch": round(t, 2), "rate_hz": rate,
                            "lambda_mm": round(LAMBDA_MM, 1),
                            "disp_mm": None, "sigma_mm": None,
                            "freq_off_hz": None, "lock": False,
                            "series": [],
                        }
                        try:
                            merge_state(phase, last_row, last_ppm)
                        except Exception as e:
                            log(f"heartbeat publish failed: {e}")
                    continue

                disp_mm = ep["disp_mm"]
                ring.append((t, disp_mm))

                if t - last_pub >= 1.0:
                    last_pub = t
                    step = max(1, rate // 5)        # series at ~5 points/s
                    series = [[round(pt, 1), round(pd, 3)]
                              for pt, pd in list(ring)[::step][-300:]]
                    phase = {
                        "epoch": round(t, 2), "rate_hz": rate,
                        "lambda_mm": round(LAMBDA_MM, 1),
                        "disp_mm": round(disp_mm, 3),
                        "sigma_mm": round(ep["sigma_mm"], 3)
                                    if ep["sigma_mm"] is not None else None,
                        "freq_off_hz": round(ep["freq_off_hz"], 3),
                        "lock": ep["locked"],
                        "series": series,
                    }
                    ppm = ep["ppm"]
                    row = {
                        "band": MY_BAND,
                        "name": f"TV pilot {F_PILOT/1e6:.5f} MHz · carrier-phase "
                                f"track · One+ClearStream",
                        "kind": "ClockDriftPpm",
                        "value": round(ppm, 4), "sigma": 0.005,
                        "ref_hz": F_PILOT, "epoch": round(t, 2),
                        "sats": ["GPS-disciplined Tx"],
                        # honesty (round-13): the One is CABLED to the Pro's
                        # CLKOUT. Round-14 added the soft proof: when
                        # series_producer's drift-lock verifier (state.series
                        # .json clkin_soft_verified) reads true, the ATSC−WAAS
                        # series move 1:1 — the chain is measured, not assumed.
                        "anchor": "One ← Pro CLKOUT cable (" + _clkin_label() + ")",
                        "ns_per_s": round(ppm * 1000.0, 1),
                        "m_per_s": round(ppm * 1e-6 * C_MPS, 2),
                    }
                    try:
                        last_row, last_ppm = row, ppm
                        merge_state(phase, row, ppm)
                        with open(HIST, "a") as fh:
                            fh.write(json.dumps({
                                "t": round(t, 2), "disp_mm": round(disp_mm, 3),
                                "freq_off_hz": round(ep["freq_off_hz"], 3),
                                "sigma_mm": phase["sigma_mm"],
                                "lock": ep["locked"]}) + "\n")
                    except Exception as e:
                        log(f"publish error: {e}")
                    log(f"disp {disp_mm:+9.3f} mm  sigma {phase['sigma_mm']} mm  "
                        f"foff {ep['freq_off_hz']:+8.3f} Hz ({ppm:+.4f} ppm)  "
                        f"amp {ep['amp']:.1f}/{tracker.amp_med:.1f}  "
                        f"lock {ep['locked']}")

        except StreamDead as e:
            log(f"stream died: {e} — reopening, phase will re-lock")
        except Exception as e:
            log(f"unexpected error: {e!r} — reopening")
        finally:
            try:
                if fd is not None:
                    os.close(fd)
            except Exception:
                pass
            try:
                if proc and proc.poll() is None:
                    proc.terminate()
                    proc.wait(timeout=5)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
        time.sleep(2)


if __name__ == "__main__":
    main()
