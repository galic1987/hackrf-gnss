#!/usr/bin/env python3
"""20 Hz carrier-phase producer for the /sync panel ("phase producer").

Tracks the ATSC ch35 pilot (true 602.30944 MHz, GPS-disciplined Tx,
~+43 dB over noise on the ClearStream) on the HackRF One, which is
CLKIN-locked to the Pro's disciplined 10 MHz. One continuous
hackrf_transfer is held for the producer's whole lifetime, tuned
500 kHz above the pilot so the line sits at -500 kHz (off the DC spike).
Samples arrive over a FIFO (mkfifo) — never a spooling file — so disk
usage is zero regardless of run length.

Per 50 ms block (400k samples, 20 epochs/s):
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
FS = 8e6
BLOCK = 400000                    # 50 ms -> 20 epochs/s
BLK_BYTES = BLOCK * 2             # interleaved int8 I/Q
C_MPS = 299792458.0
LAMBDA_MM = C_MPS / F_PILOT * 1000.0
FIFO = "/tmp/phase_producer.iq"
STATE = "/Volumes/Radiator 8TB/gnss/observations/state.phase.json"
HIST = "/Volumes/Radiator 8TB/gnss/observations/phase_history.jsonl"
MY_BAND = "ATSC ch35"
EST_SAMPLES = 1 << 24             # 2.1 s coherent FFT for initial freq
AMP_DROP = 0.35                   # epoch low-flag: amp < 35% of running median
AMP_LOST = 0.20                   # sustained below 20% of median -> lock lost
LOST_BLOCKS = 100                 # 5 s of collapse before lock drops
LOCK_BLOCKS = 40                  # 2 s of stable amplitude to (re)lock

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
           "-s", "8000000", "-l", "40", "-g", "44", "-a", "0", "-r", FIFO]
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


def estimate_freq(fd, buf):
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
        if not read_into(fd, buf, BLK_BYTES):
            raise StreamDead("FIFO EOF during initial estimate")
        del buf[:BLK_BYTES]
    if "err" in result:
        raise RuntimeError(f"estimate FFT failed: {result['err']}")
    f_line, snr_db = result["f"], result["snr"]
    log(f"initial estimate: pilot line at {f_line:.2f} Hz baseband "
        f"(offset {f_line + 500e3:+.2f} Hz = {(f_line + 500e3) / F_PILOT * 1e6:+.4f} ppm), "
        f"SNR {snr_db:.0f} dB")
    return f_line, snr_db


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
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    log(f"phase producer starting — ch35 pilot {F_PILOT/1e6:.5f} MHz, "
        f"lambda {LAMBDA_MM:.2f} mm, One {ONE}")
    # two-stage derotation (fast path, ~4 ms/block vs ~35 ms for a full-rate
    # complex128 exp — the consumer must stay well under the 50 ms block
    # budget or the FIFO backpressures the USB stream and drops samples):
    #   1. coarse mix with a tiled 16-entry LUT at exactly -500 kHz
    #      (BLOCK % 16 == 0, so block boundaries are seamless),
    #   2. decimate by 64 (segment means), fine-rotate at the residual
    #      f_res (~-285 Hz, steered) with a float64 phase accumulator.
    DEC = 64
    NSEG = BLOCK // DEC
    lut = np.exp(-1j * 2 * np.pi * (-500e3) / FS
                 * np.arange(16)).astype(np.complex64)
    lut_tile = np.tile(lut, BLOCK // 16)
    seg_ar = np.arange(NSEG, dtype=np.float64) + 0.5   # segment centers

    while True:
        proc, fd, buf = None, None, None
        try:
            proc, fd = open_stream()
            buf = bytearray()
            f_line, snr_db = estimate_freq(fd, buf)    # baseband pilot Hz
            f_res = f_line + 500e3                     # residual vs -500 kHz
            fine_step = 2 * np.pi * f_res / (FS / DEC)
            phi0f = 0.0
            t0 = time.time()                # wall clock at estimate end
            n_done = 0                      # samples tracked since t0

            locked = False
            stable = 0                      # consecutive good-amplitude blocks
            collapsed = 0                   # consecutive collapsed blocks
            unwrapped = 0.0                 # radians
            prev_phi = None
            unwrap_ref = None               # radians at lock moment
            amps = deque(maxlen=200)        # 10 s amplitude history
            amp_med = None
            phase10 = deque()               # (t, cycles) for slope, 10 s
            disp10 = deque()                # (t, disp_mm) for sigma, 10 s
            ring = deque(maxlen=1200)       # 60 s of (t, disp_mm) @ 20 Hz
            last_pub = 0.0

            while True:
                if proc.poll() is not None:
                    raise StreamDead(f"transfer exited rc={proc.returncode}")
                if not read_into(fd, buf, BLK_BYTES):
                    raise StreamDead("FIFO EOF — writer died")
                iq = to_iq(take(buf, BLK_BYTES))

                mixed = iq * lut_tile                       # pilot -> ~f_res
                seg = mixed.reshape(NSEG, DEC).mean(axis=1)
                ang = (phi0f + fine_step * seg_ar) % (2 * np.pi)
                phi0f = float((phi0f + fine_step * NSEG) % (2 * np.pi))
                z = np.mean(seg * np.exp(-1j * ang))
                amp = float(abs(z))
                phi = float(np.arctan2(z.imag, z.real))
                t = t0 + (n_done + BLOCK / 2) / FS
                n_done += BLOCK

                if prev_phi is not None:
                    unwrapped += (phi - prev_phi + np.pi) % (2 * np.pi) - np.pi
                prev_phi = phi

                amps.append(amp)
                if amp_med is None and len(amps) >= 40:
                    amp_med = float(np.median(amps))
                elif amp_med is not None and len(amps) >= 200:
                    amp_med = float(np.median(amps))

                good = amp_med is None or amp > AMP_DROP * amp_med
                if amp_med is not None and amp < AMP_LOST * amp_med:
                    collapsed += 1
                else:
                    collapsed = 0
                if locked and collapsed >= LOST_BLOCKS:
                    locked = False
                    stable = 0
                    unwrap_ref = None
                    log(f"LOCK LOST — pilot amplitude collapsed "
                        f"(amp {amp:.1f} vs median {amp_med:.1f})")
                if not locked and amp_med is not None:
                    stable = stable + 1 if good else 0
                    if stable >= LOCK_BLOCKS:
                        locked = True
                        unwrap_ref = unwrapped
                        log(f"LOCK — amp median {amp_med:.1f}, phase ref reset")

                if not good or unwrap_ref is None:
                    continue

                disp_mm = (unwrapped - unwrap_ref) / (2 * np.pi) * LAMBDA_MM
                cycles = unwrapped / (2 * np.pi)
                ring.append((t, disp_mm))
                phase10.append((t, cycles))
                disp10.append((t, disp_mm))
                while phase10 and t - phase10[0][0] > 10.5:
                    phase10.popleft()
                while disp10 and t - disp10[0][0] > 10.5:
                    disp10.popleft()

                # residual frequency: steer the fine derotation with the
                # 2-s phase slope once a second (the reference chain wanders
                # ~1 Hz/min and multipath jitters the phase at the 1-s scale;
                # a fixed derotation would integrate that into a runaway
                # displacement ramp and eventually break the unwrap).
                # Reported freq_off = the steered residual itself.
                recent = [p for p in phase10 if t - p[0] <= 2.5]
                if len(recent) >= 40 and t - last_pub >= 1.0:
                    tt = np.array([p[0] for p in recent])
                    cc = np.array([p[1] for p in recent])
                    slope2 = float(np.polyfit(tt - tt[0], cc, 1)[0])
                    f_res += slope2
                    fine_step = 2 * np.pi * f_res / (FS / DEC)
                freq_off_hz = f_res
                ppm = freq_off_hz / F_PILOT * 1e6

                # per-epoch sensitivity: robust std of the 2nd difference
                # (kills drift/curvature, immune to fade cycle-slips);
                # for white per-epoch noise std(d2)/sqrt(6) = sigma_epoch
                sigma_mm = None
                if len(disp10) >= 100:
                    dd = np.array([p[1] for p in disp10])
                    d2 = np.diff(dd, 2)
                    sigma_mm = float(1.4826 * np.median(np.abs(d2 - np.median(d2)))
                                     / np.sqrt(6))

                if t - last_pub >= 1.0:
                    last_pub = t
                    series = [[round(pt, 1), round(pd, 3)]
                              for pt, pd in list(ring)[::4][-300:]]
                    phase = {
                        "epoch": round(t, 2), "rate_hz": 20,
                        "lambda_mm": round(LAMBDA_MM, 1),
                        "disp_mm": round(disp_mm, 3),
                        "sigma_mm": round(sigma_mm, 3) if sigma_mm is not None else None,
                        "freq_off_hz": round(freq_off_hz, 3),
                        "lock": locked,
                        "series": series,
                    }
                    row = {
                        "band": MY_BAND,
                        "name": f"TV pilot {F_PILOT/1e6:.5f} MHz · carrier-phase "
                                f"track · One+ClearStream",
                        "kind": "ClockDriftPpm",
                        "value": round(ppm, 4), "sigma": 0.005,
                        "ref_hz": F_PILOT, "epoch": round(t, 2),
                        "sats": ["GPS-disciplined Tx"],
                        "anchor": "CLKIN-locked to Pro",
                        "ns_per_s": round(ppm * 1000.0, 1),
                        "m_per_s": round(ppm * 1e-6 * C_MPS, 2),
                    }
                    try:
                        merge_state(phase, row, ppm)
                        with open(HIST, "a") as fh:
                            fh.write(json.dumps({
                                "t": round(t, 2), "disp_mm": round(disp_mm, 3),
                                "freq_off_hz": round(freq_off_hz, 3),
                                "sigma_mm": phase["sigma_mm"],
                                "lock": locked}) + "\n")
                    except Exception as e:
                        log(f"publish error: {e}")
                    log(f"disp {disp_mm:+9.3f} mm  sigma {phase['sigma_mm']} mm  "
                        f"foff {freq_off_hz:+8.3f} Hz ({ppm:+.4f} ppm)  "
                        f"amp {amp:.1f}/{amp_med:.1f}  lock {locked}")

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
