#!/usr/bin/env python3
"""1 Hz live multi-constellation tracker producer for the /sync panel.

Wraps examples/live_radio, which OWNS the HackRF Pro: 16 Msps centred on
1568.25 MHz (spans 1560.25-1576.25: BeiDou B1I 1561.098 + GPS L1 1575.42 +
Galileo E1 + WAAS/SBAS in one capture), bias-tee ON for the AA.250. The
radio-owning design replaced hackrf_transfer + FIFO because hackrf_open is
exclusive.  live_radio computes and publishes the WAAS-GEO clock-discipline
proposal in SHADOW mode, but it has no correction-write call and the streaming
control handle exposes no clock-correction command: the 122/122 historical
writes collapsed all tracker locks. The Rust transport exposes no direct or
streaming clock-correction write API.
It prints one JSON line per tracked PRN per second plus
{"discipline": {...}} per cycle.

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

Radio discipline: this wrapper and band_producer share the atomic lease in
scripts/pro_lease.py.  The tracker holds it for the complete live_radio child
lifetime; band snapshots skip while it is held and their rows age.  A
maintenance gate blocks both before the tracker is stopped, so band_producer
cannot seize the just-freed device.  pgrep is diagnostic only, never the
ownership primitive. band_producer is NOT killed during normal operation.
sync_producer is paused (SIGSTOP) only during startup. The OTHER
radio (HackRF One ...922c63dc21748847, phase producer) is strictly
off-limits — this script never touches it.

If the tracker dies (USB hiccup/EOF), this wrapper raises a durable
reset-required maintenance gate and exits 78. It never reopens the Pro in the
same process: hardware history requires an immediate board reset before a
reliable restart. The seed cache only accelerates the post-reset reacquisition.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time

import pro_lease

TOOLS = "/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src"
ENV = dict(os.environ,
           DYLD_LIBRARY_PATH="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/libhackrf/src")
PRO = os.environ.get("PRO_SERIAL", "0000000000000000645061de252d6613")  # Pro#2 (Pro#1 977c… dead 2026-08-27, hw power fault)
FS = 16_000_000
FC = 1_568_250_000
STATE = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json"
FAILURE_TOKEN_FILE = (pro_lease.DEFAULT_OBS /
                      "tracker-reset-required.maintenance.token")
# live_radio owns the radio itself (hackrf_open is exclusive — with a
# separate hackrf_transfer holding the claim, NOTHING else could steer the
# clock-correction register; that was the v1 discipline actuator dead-end).
TRACKER = "/Volumes/Radiator 8TB/gnss/hackrf_gnss/target/release/examples/live_radio"
UNBLOCKED_EXEC = "/Volumes/Radiator 8TB/gnss/hackrf_gnss/scripts/exec_unblocked.py"
L1_HZ = 1575.42e6
C_MPS = 299792458.0
MY_BAND = "L1 / WAAS (live)"

_xfer = None          # hackrf_transfer child
_rust = None          # live_track child
_pro_lease = None     # atomic lease held for the complete child lifetime


def log(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def unmanaged_pro_busy():
    """Transitional diagnostic after acquiring the real lease.

    This catches an old, non-lease-aware live_radio/hackrf_transfer process.
    Query failure is busy (fail closed).  It is not the mutex.
    """
    try:
        for pattern in ("hackrf_transfer", "examples/live_radio"):
            result = subprocess.run(["pgrep", "-fl", pattern],
                                    capture_output=True, text=True, timeout=5)
            if result.returncode not in (0, 1):
                return True
            if any(PRO in line for line in result.stdout.splitlines()):
                return True
        return False
    except Exception:
        return True


def wait_for_pro_lease():
    """Wait without opening hardware until the gate and lease both permit it."""
    global _pro_lease
    while True:
        previous_mask = None
        try:
            # Do not allow a graceful signal in the mkdir→global-owner gap;
            # SIGKILL may still leave a stale lock, intentionally fail-closed.
            if hasattr(signal, "pthread_sigmask"):
                previous_mask = signal.pthread_sigmask(
                    signal.SIG_BLOCK, {signal.SIGTERM, signal.SIGINT})
            lease = pro_lease.acquire_client(
                pro_lease.DEFAULT_OBS, "tracker_producer/live_radio", PRO)
            _pro_lease = lease
        except pro_lease.ProLeaseUnavailable as exc:
            log(f"Pro lease unavailable — {exc}; waiting")
            time.sleep(3)
            continue
        except pro_lease.ProLeaseError as exc:
            log(f"FATAL: Pro lease protocol failure — {exc}")
            raise
        finally:
            if previous_mask is not None:
                signal.pthread_sigmask(signal.SIG_SETMASK, previous_mask)
        if unmanaged_pro_busy():
            lease.release()
            _pro_lease = None
            log("unmanaged legacy Pro owner detected after lease — waiting")
            time.sleep(3)
            continue
        return lease


def sync_producer_pids():
    """Real Python pollers only; parent-shell launch text never matches."""
    return pro_lease.running_python_script_pids("sync_producer.py")


def pause_sync_producer():
    """SIGSTOP sync_producer during transfer startup only (it polls the same
    USB); returns the pids so the caller can SIGCONT them right after."""
    stopped = []
    for p in sync_producer_pids():
        try:
            os.kill(p, signal.SIGSTOP)
            stopped.append(p)
        except OSError as exc:
            for prior in stopped:
                try:
                    os.kill(prior, signal.SIGCONT)
                except OSError:
                    pass
            raise pro_lease.ProLeaseProtocolError(
                f"could not pause exact sync_producer pid {p}: {exc}") from exc
    return stopped


def open_stream():
    """Acquire the lease, launch the tracker, and return both."""
    global _rust, _pro_lease
    lease = wait_for_pro_lease()
    stopped = []
    rust = None
    previous_mask = None
    try:
        stopped = pause_sync_producer()
        # Keep graceful shutdown signals blocked from immediately before
        # Popen until the child is stored globally.  Without this, SIGTERM in
        # the Popen->_rust gap can release the lease while live_radio remains
        # alive and owns the USB device.
        if hasattr(signal, "pthread_sigmask"):
            previous_mask = signal.pthread_sigmask(
                signal.SIG_BLOCK, {signal.SIGTERM, signal.SIGINT})
        child_env = dict(ENV)
        child_env["HACKRF_PRO_LEASE_TOKEN"] = lease.token
        with open("/tmp/live_track.stderr.log", "ab", buffering=0) as rust_err:
            rust = subprocess.Popen([sys.executable, UNBLOCKED_EXEC, TRACKER, PRO],
                                    env=child_env,
                                    stdout=subprocess.PIPE,
                                    stderr=rust_err, text=False)
        _rust = rust
        log(f"tracker (radio-owning) pid {rust.pid}")
        return None, rust, lease
    except Exception:
        # A child that reached Popen may already have opened the board.  Prove
        # it stopped and arm the reset-required gate before freeing ownership.
        child_stopped = terminate_child(rust)
        gate_safe = rust is None
        if rust is not None and child_stopped:
            gate_safe = arm_reset_required_gate()
        if child_stopped and gate_safe:
            try:
                lease.release()
                if _pro_lease is lease:
                    _pro_lease = None
            except pro_lease.ProLeaseError as exc:
                log(f"FATAL: startup lease release failed: {exc}")
        else:
            log("FATAL: startup child/gate state unproven; retaining Pro lease")
        raise
    finally:
        for p in stopped:
            try:
                os.kill(p, signal.SIGCONT)
            except Exception:
                pass
        if previous_mask is not None:
            signal.pthread_sigmask(signal.SIG_SETMASK, previous_mask)


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
        import sys
        if "/Volumes/Radiator 8TB/gnss/hackrf_gnss/scripts" not in sys.path:
            sys.path.append("/Volumes/Radiator 8TB/gnss/hackrf_gnss/scripts")
        import geocorrector_helper
        site_ecef = geocorrector_helper.get_site_ecef()
        
        sum_d = 0.0
        for s in waas:
            # subtract satellite LOS motion and clock drift from the raw Doppler (MT9 GeoCorrector)
            geo_d = geocorrector_helper.calc_geo_doppler_hz(s.get("sbas_geonav"), site_ecef, L1_HZ, t_unix=now)
            sum_d += (s["doppler_hz"] - geo_d)
            
        mean_d = sum_d / len(waas)
        ppm = mean_d / L1_HZ * 1e6
        # Production live_radio is enforced SHADOW-only. correction_ppm is a
        # proposal and applied_correction_readback is false, so it must never
        # be added to this measured Doppler row as though it were hardware
        # truth.
        srcs.append({
            "band": MY_BAND,
            "name": "WAAS GEO Doppler (MT9 GeoCorrected) · Pro+AA.250, 1 Hz tracker",
            "kind": "ClockDriftPpm",
            # Conservative sigma bound (0.03 ppm) until full common-mode calibration is complete
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
    with open(tmp, "w") as fh:
        json.dump(state, fh, indent=1)
    os.replace(tmp, STATE)
    return len(waas)


def terminate_child(p):
    """Gracefully stop one child; never free its lease while it may be alive."""
    if not p or p.poll() is not None:
        return True
    try:
        p.terminate()
        p.wait(timeout=5)
        return p.poll() is not None
    except Exception as exc:
        log(f"child pid {getattr(p, 'pid', '?')} did not stop cleanly: {exc}")
        return False


def arm_reset_required_gate():
    """Block every new Pro owner after an unexpected tracker failure.

    This runs while the tracker lease is still held.  If an operator already
    gated maintenance, that gate is sufficient.  Otherwise create a durable
    token next to observations so the reset runbook can acquire/release it.
    Failure to prove a gate returns False and the caller retains its lease.
    """
    if pro_lease.maintenance_gate_active(pro_lease.DEFAULT_OBS):
        log("maintenance gate already active after tracker failure")
        return True
    try:
        pro_lease.create_maintenance_gate(
            pro_lease.DEFAULT_OBS,
            "tracker-failure-reset-required",
            FAILURE_TOKEN_FILE,
            PRO,
        )
        log(f"RESET REQUIRED: maintenance gate armed; token {FAILURE_TOKEN_FILE}")
        return True
    except pro_lease.ProLeaseError as exc:
        # A concurrent operator gate may have won after our first check.
        if pro_lease.maintenance_gate_active(pro_lease.DEFAULT_OBS):
            log("maintenance gate won tracker-failure race")
            return True
        log(f"FATAL: could not arm reset-required gate: {exc}")
        return False


def shutdown(*_):
    global _pro_lease
    children_stopped = all([terminate_child(p) for p in (_rust, _xfer)])
    gate_safe = _pro_lease is None
    release_ok = True
    if children_stopped and _pro_lease:
        gate_safe = arm_reset_required_gate()
    if children_stopped and gate_safe:
        try:
            if _pro_lease:
                _pro_lease.release()
                _pro_lease = None
        except Exception as exc:
            log(f"FATAL: Pro lease release failed during shutdown: {exc}")
            release_ok = False
    else:
        log("FATAL: gate/child state unproven; leaving atomic lease in place")
    sys.exit(0 if children_stopped and gate_safe and release_ok else 78)


def main():
    global _pro_lease
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    if "HACKRF_GNSS_ACTUATE" in os.environ:
        log("FATAL: HACKRF_GNSS_ACTUATE is retired; production tracker is SHADOW-only")
        return 78
    if PRO != pro_lease.PRODUCTION_SERIAL:
        log(f"FATAL: PRO_SERIAL must be exact production serial {pro_lease.PRODUCTION_SERIAL}")
        return 78
    log(f"tracker producer starting — {FS/1e6:.0f} Msps @ {FC/1e6:.3f} MHz "
        f"(B1I+L1+E1+SBAS), Pro {PRO}")
    while True:
        xfer, rust, lease = None, None, None
        try:
            xfer, rust, lease = open_stream()
            sats = {}            # (sys, prn) -> latest report dict
            disc = {}            # latest discipline line from live_radio
            lock = threading.Lock()
            dead = threading.Event()
            contract_violation = []

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
                            d = r["discipline"]
                            if d.get("actuate") or d.get("corr_applied"):
                                contract_violation.append(
                                    "live_radio reported forbidden clock actuation")
                                dead.set()
                                return
                            disc.clear()
                            disc.update(d)
                        else:
                            # r["epoch"] is the SAMPLE-ACCURATE stream epoch
                            # (3107967) and lags wall clock by the engine's
                            # processing backlog — it must not be compared
                            # against wall time. Stamp arrival for liveness.
                            r["_rx"] = time.time()
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
                    # drop PRNs silent for >10 s (channel dropped/re-acq).
                    # Liveness = arrival time, NOT row epoch: the stream
                    # epoch lags wall clock by the processing backlog, and
                    # gating on it silently blanked the whole sat table
                    # (2026-08-31 07:15 session, backlog ~4.6%/s growth).
                    snap = {k: v for k, v in snap.items()
                            if now - v.get("_rx", 0) < 10}
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
            if contract_violation:
                raise RuntimeError(contract_violation[0])
            raise RuntimeError("tracker stdout EOF")
        except Exception as e:
            log(f"stream problem: {e} — automatic reopen forbidden; board reset required")
        finally:
            children_stopped = all([terminate_child(p) for p in (rust, xfer)])
            gate_safe = lease is None
            if children_stopped and lease:
                gate_safe = arm_reset_required_gate()
            if children_stopped and gate_safe:
                try:
                    if lease:
                        lease.release()
                except Exception as e:
                    log(f"FATAL: Pro lease release failed: {e}")
                else:
                    if _pro_lease is lease:
                        _pro_lease = None
            else:
                log("FATAL: reset gate/child state unproven; lease retained, wrapper stopping")
        return 78


if __name__ == "__main__":
    raise SystemExit(main())
