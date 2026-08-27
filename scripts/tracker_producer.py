#!/usr/bin/env python3
"""1 Hz live multi-constellation tracker producer for the /sync panel.

Wraps examples/live_radio, which OWNS the HackRF Pro: 16 Msps centred on
1568.25 MHz (spans 1560.25-1576.25: BeiDou B1I 1561.098 + GPS L1 1575.42 +
Galileo E1 + WAAS/SBAS in one capture), bias-tee ON for the AA.250. The
radio-owning design replaced hackrf_transfer + FIFO because hackrf_open is
exclusive: with a transfer process holding the claim, the clock-correction
register was unreachable and the v1 discipline loop had no actuator.
live_radio closes the clock loop in-process (WAAS GEO Doppler -> correction
register, slew-limited, stall-detected, sign-probed) and prints one JSON
line per tracked PRN per second plus {"discipline": {...}} per cycle.

This wrapper maintains per-satellite state and publishes ONLY its own file,
observations/state.tracker.json (tmp + os.replace; the Rust server
deep-merges all observations/state.*.json at /api/sync read time):
  state["tracker"] = {"sats": [{prn, sys, doppler_hz, ppm, vs_ppm,
                                cn0_proxy, code_phase, lock_s, epoch,
                                carrier_cycles, phase_frac, slip}, ...]}
  carrier phase fields (from live.rs, passed through verbatim):
    carrier_cycles — integrated replica carrier phase in cycles, zero at
      channel (re)seed; continuous while locked; rate == Doppler; the
      absolute value carries the Costas 180 deg ambiguity
    phase_frac — fractional phase at the report instant, cycles, modulo
      the data-bit half-cycle (Costas), always in [0, 0.5)
    slip — a phase break happened this second (lock watchdog fired or the
      channel re-seeded and carrier_cycles re-zeroed)
  state["discipline"] = latest discipline line (forwarded verbatim)
  sources row: WAAS GEO mean Doppler as a ClockDriftPpm row with band
  "L1 / WAAS (live)" (distinct from band_producer's snapshot row
  "L1 / WAAS") — this upgrades the consensus voter to a 1 Hz cadence.

Radio discipline: band_producer still runs and keeps taking its snapshot
transfers on the Pro; while this producer owns the radio those fail and its
rows go stale — ACCEPTABLE (its retention logic keeps old rows dimmed; the
upper-band rotation is effectively paused). band_producer is NOT killed.
sync_producer is paused (SIGSTOP) only during startup. The OTHER
radio (HackRF One ...922c63dc21748847, phase producer) is strictly
off-limits — this script never touches it.

If the tracker dies (USB hiccup), it is relaunched after a short pause; the
seed cache brings tracking back in ~15-30 s.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time

TOOLS = "/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src"
ENV = dict(os.environ,
           DYLD_LIBRARY_PATH="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/libhackrf/src")
PRO = os.environ.get("PRO_SERIAL", "0000000000000000645061de252d6613")  # Pro#2 (Pro#1 977c… dead 2026-08-27, hw power fault)
FS = 16_000_000
FC = 1_568_250_000
STATE = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json"
# live_radio owns the radio itself (hackrf_open is exclusive — with a
# separate hackrf_transfer holding the claim, NOTHING else could steer the
# clock-correction register; that was the v1 discipline actuator dead-end).
TRACKER = "/Volumes/Radiator 8TB/gnss/hackrf_gnss/target/release/examples/live_radio"
L1_HZ = 1575.42e6
C_MPS = 299792458.0
MY_BAND = "L1 / WAAS (live)"

_xfer = None          # hackrf_transfer child
_rust = None          # live_track child


def log(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def pro_busy():
    """True if another hackrf_transfer command line mentions the Pro."""
    try:
        out = subprocess.run(["pgrep", "-fl", "hackrf_transfer"],
                             capture_output=True, text=True).stdout
    except Exception:
        return False
    me = str(os.getpid())
    return any(PRO in line and me not in line for line in out.splitlines())


def wait_for_pro():
    while pro_busy():
        log("another hackrf_transfer holds the Pro — waiting")
        time.sleep(3)


def sync_producer_pids():
    try:
        out = subprocess.run(["pgrep", "-f", "sync_producer.py"],
                             capture_output=True, text=True).stdout
        return [int(p) for p in out.split() if int(p) != os.getpid()]
    except Exception:
        return []


def pause_sync_producer():
    """SIGSTOP sync_producer during transfer startup only (it polls the same
    USB); returns the pids so the caller can SIGCONT them right after."""
    pids = sync_producer_pids()
    for p in pids:
        try:
            os.kill(p, signal.SIGSTOP)
        except Exception:
            pass
    return pids


def open_stream():
    """Launch the radio-owning tracker; return the process."""
    global _rust
    wait_for_pro()
    stopped = pause_sync_producer()
    try:
        rust_err = open("/tmp/live_track.stderr.log", "ab", buffering=0)
        rust = subprocess.Popen([TRACKER, PRO], env=ENV,
                                stdout=subprocess.PIPE,
                                stderr=rust_err, text=False)
        _rust = rust
        log(f"tracker (radio-owning) pid {rust.pid}")
        return None, rust
    finally:
        for p in stopped:
            try:
                os.kill(p, signal.SIGCONT)
            except Exception:
                pass


def read_consensus():
    """Cross-producer consensus clock drift (ppm), if the series producer
    has published one recently. Suspect consensus (unarbitrated 2-voter
    midpoint / single voter) publishes consensus_ppm:null plus a
    candidate_value_ppm — the candidate is diagnostics, never a voter
    input (round-8 null-consensus law)."""
    try:
        d = json.load(open("/Volumes/Radiator 8TB/gnss/observations/state.series.json"))
        if time.time() - d.get("epoch", 0) < 600 and not d.get("consensus_suspect"):
            return d.get("consensus_ppm")
    except Exception:
        pass
    return None


CARRIER = {"gps": L1_HZ, "sbas": L1_HZ, "galileo": L1_HZ, "beidou": 1561.098e6}


def publish(sats, now, disc=None):
    """Write state.tracker.json atomically: per-PRN rows + the live WAAS
    ClockDriftPpm sources row (mean Doppler of locked SBAS GEOs)."""
    consensus = read_consensus()
    sats_out = sorted(sats.values(),
                      key=lambda s: ({"gps": 0, "galileo": 1, "beidou": 2,
                                      "sbas": 3}.get(s["sys"], 9), s["prn"]))
    for s in sats_out:
        # per-satellite drift in ppm of its own carrier, and offset vs the
        # consensus. For MEO sats this is dominated by orbital motion
        # Doppler — only the SBAS GEO rows are pure clock measurements.
        ppm = s["doppler_hz"] / CARRIER.get(s["sys"], L1_HZ) * 1e6
        s["ppm"] = round(ppm, 3)
        if consensus is not None:
            s["vs_ppm"] = round(ppm - consensus, 3)
    state = {
        "epoch": round(now, 2),
        "ttl_s": 30,
        "tracker": {"sats": sats_out},
    }
    if disc:
        state["discipline"] = disc
    # In-band presence rows: every constellation the tracker sees live at
    # 1568.25 MHz — these make the panel's GPS L1 / Galileo E1 / BeiDou B1I
    # rows live instead of depending on (paused) snapshot captures.
    BANDS = {"gps": ("GPS L1 C/A", "C/A live track"),
             "galileo": ("Galileo E1", "E1B BOC(1,1) live track"),
             "beidou": ("BeiDou B1I", "B1I live track")}
    srcs = []
    for sysname, (band, desc) in BANDS.items():
        chans = [s for s in sats_out if s["sys"] == sysname]
        if not chans:
            continue
        srcs.append({
            "band": band,
            "name": f"{desc} · Pro+AA.250, 16 Msps @ 1568.25 · Presence",
            "kind": "Presence",
            "value": None, "sigma": None,
            "epoch": round(now, 2),
            "sats": [f"PRN {s['prn']}" for s in chans],
            "anchor": "Pro live track @ 1568.25 MHz",
        })
    waas = [s for s in sats_out if s["sys"] == "sbas" and s["lock_s"] > 0]
    if waas:
        mean_d = sum(s["doppler_hz"] for s in waas) / len(waas)
        ppm = mean_d / L1_HZ * 1e6
        # doppler_hz is measured AFTER the hardware clock-correction register
        # (resid = raw - corr, per the discipline march). Every other drift
        # voter (ATSC ch35 via CLKOUT, PC clock) measures the RAW TCXO, so
        # add the correction back — otherwise the row votes ~0 into a
        # consensus of -0.47 and pulls it to a meaningless midpoint.
        # BUT (review round 6): the cache is historical intent, not applied
        # truth — after the restart procedure's board reset the register is
        # unity while the cache still believes. Add back only a correction
        # this live_radio process verifiably wrote (actuate + corr_applied).
        disc_d = disc or {}
        corr = disc_d.get("correction_ppm") or 0.0
        if disc_d.get("actuate") and disc_d.get("corr_applied"):
            ppm += corr
        srcs.append({
            "band": MY_BAND,
            "name": "WAAS GEO live Doppler + corr register · Pro+AA.250, 1 Hz tracker",
            "kind": "ClockDriftPpm",
            # sigma floors at GEO motion Doppler (+-0.025 ppm — covers the
            # +-0.01 ppm range-rate bound: +-0.5-3 m/s line of sight / c,
            # plus inter-source margin), not the PLL's
            # short-term precision — path systematics dominate inter-source
            # comparison
            "value": round(ppm, 4), "sigma": 0.03,
            "ref_hz": L1_HZ, "epoch": round(now, 2),
            "sats": [f"PRN {s['prn']}" for s in waas],
            "anchor": "Pro live track @ 1568.25 MHz",
            "ns_per_s": round(ppm * 1000.0, 1),
            "m_per_s": round(ppm * 1e-6 * C_MPS, 2),
        })
    if srcs:
        state["sources"] = srcs
    tmp = STATE + ".tracker.tmp"
    json.dump(state, open(tmp, "w"), indent=1)
    os.replace(tmp, STATE)
    return len(waas)


def shutdown(*_):
    for p in (_rust, _xfer):
        try:
            if p and p.poll() is None:
                p.terminate()
        except Exception:
            pass
    sys.exit(0)


def main():
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    log(f"tracker producer starting — {FS/1e6:.0f} Msps @ {FC/1e6:.3f} MHz "
        f"(B1I+L1+E1+SBAS), Pro {PRO}")
    while True:
        xfer, rust = None, None
        try:
            xfer, rust = open_stream()
            sats = {}            # (sys, prn) -> latest report dict
            disc = {}            # latest discipline line from live_radio
            lock = threading.Lock()
            dead = threading.Event()

            def reader():
                while True:
                    line = rust.stdout.readline()
                    if not line:
                        dead.set()
                        return
                    try:
                        r = json.loads(line)
                    except Exception:
                        continue
                    with lock:
                        if "discipline" in r:
                            disc.clear()
                            disc.update(r["discipline"])
                        else:
                            sats[(r["sys"], r["prn"])] = r

            threading.Thread(target=reader, daemon=True).start()
            last_pub = 0.0
            last_n = -1
            while not dead.is_set():
                if rust.poll() is not None:
                    raise RuntimeError(f"tracker exited rc={rust.returncode}")
                now = time.time()
                if now - last_pub >= 1.0:
                    last_pub = now
                    with lock:
                        snap = dict(sats)
                        disc_snap = dict(disc)
                    # drop PRNs silent for >10 s (channel dropped/re-acq)
                    snap = {k: v for k, v in snap.items()
                            if now - v.get("epoch", 0) < 10}
                    try:
                        nwaas = publish(snap, now, disc_snap)
                    except Exception as e:
                        log(f"publish error: {e}")
                        nwaas = 0
                    nlock = sum(1 for v in snap.values() if v["lock_s"] > 0)
                    if len(snap) != last_n:
                        log(f"{len(snap)} channels, {nlock} locked, "
                            f"{nwaas} WAAS in drift row")
                        last_n = len(snap)
                time.sleep(0.2)
            raise RuntimeError("tracker stdout EOF")
        except Exception as e:
            log(f"stream problem: {e} — reopening")
        finally:
            for p in (rust, xfer):
                try:
                    if p and p.poll() is None:
                        p.terminate()
                        p.wait(timeout=5)
                except Exception:
                    try:
                        p.kill()
                    except Exception:
                        pass
        time.sleep(2)


if __name__ == "__main__":
    main()
