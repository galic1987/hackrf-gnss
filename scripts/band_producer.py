#!/usr/bin/env python3
"""Real-time per-band drift producer for the /sync panel.

Sources, all against the Pro's disciplined 10 MHz reference:
  - WAAS GEO PRNs from a discrete 5-s L1 snapshot taken on the Pro each
    cycle (no endless stream — the open-ended capture was a disk bomb).
  - Presence rows for every validated GNSS band, one rotation slot per
    cycle (SLOTS): GPS L1 C/A + Galileo E1 share the L1 baseband; GLO G1,
    BeiDou B1I, GPS L5, Galileo E5a, BeiDou B2a, Galileo E5b, GLONASS G2,
    GPS L2C, BeiDou B3I and Galileo E6 each take their own 5-6 s Pro
    snapshot at band centre. Marginal detections (metric 2.5-3.5) are
    prefixed "~" in the sats list.

The ATSC pilot measurements on the HackRF One were RETIRED: the One is
owned full-time by phase_producer.py (60 Hz carrier-phase track of the
ch35 pilot), which writes the "ATSC ch35" row and keeps
clock.residual_ppm live. Do not add One-side transfers back here.

Integrity layer:
  - consensus vote across independent references; >3 sigma divergence flags
  - per-PRN Doppler spread inside the WAAS pack (single-satellite spoof)
  - lost-source detector (silence = jamming signature), threshold 2x cycle
  - L1 band health: noise-floor rise / ADC clipping

Merges rows into observations/sync_state.json (keyed by band; other
producers' rows preserved) and appends band_drift_history.jsonl.
"""
import json, os, re, subprocess, time

import numpy as np

TOOLS = "/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src"
ENV = dict(os.environ,
           DYLD_LIBRARY_PATH="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/libhackrf/src")
ONE = "0000000000000000922c63dc21748847"
PRO = os.environ.get("PRO_SERIAL", "0000000000000000645061de252d6613")  # Pro#2 (Pro#1 977c… dead 2026-08-27, hw power fault)
EX = "/Volumes/Radiator 8TB/gnss/hackrf_gnss/target/release/examples"
SBAS = f"{EX}/sbas_acq"
ACQ = f"{EX}/acquire_file"
GLO = f"{EX}/glonass_acq"
BDS = f"{EX}/beidou_acq"
GAL = f"{EX}/galileo_acq"
E5 = f"{EX}/e5_acq"
E5B = f"{EX}/e5b_acq"
GLO2 = f"{EX}/glo_g2_acq"
L2C = f"{EX}/l2c_acq"
B3I = f"{EX}/b3i_acq"
E6 = f"{EX}/e6_acq"
STATE = "/Volumes/Radiator 8TB/gnss/observations/state.band.json"
HIST = "/Volumes/Radiator 8TB/gnss/observations/band_drift_history.jsonl"
L1_SNAP = "/tmp/band_l1.iq"
GLO_SNAP = "/tmp/band_glo.iq"
BDS_SNAP = "/tmp/band_bds.iq"
L5_SNAP = "/tmp/band_l5.iq"
E5A_SNAP = "/tmp/band_e5a.iq"
B2A_SNAP = "/tmp/band_b2a.iq"
E5B_SNAP = "/tmp/band_e5b.iq"
G2_SNAP = "/tmp/band_g2.iq"
L2C_SNAP = "/tmp/band_l2c.iq"
B3I_SNAP = "/tmp/band_b3i.iq"
E6_SNAP = "/tmp/band_e6.iq"
L1_HZ = 1575.42e6
GLO_HZ = 1602.0e6
B1I_HZ = 1561.098e6
L5_HZ = 1176.45e6
E5B_HZ = 1207.14e6
L2_HZ = 1227.60e6
G2_HZ = 1246.0e6
B3I_HZ = 1268.52e6
E6_HZ = 1278.75e6
C_MPS = 299792458.0
FS = 8e6
LOST_S = 360.0          # ~2x the measured cycle time; shorter = false jam alarms
PRN_SPREAD_HZ = 300.0   # WAAS GEOs share near-identical Doppler; bigger = spoof
# rotation slots: one band per cycle, full sweep every len(SLOTS) cycles
# (measured 5-8 min/cycle -> ~60-90 min/sweep under load; panel staleness
# dimming is 1 h); the lost-source alarm must not apply to any of these
SLOTS = ["gps+galileo", "glo_g1", "beidou_b1i", "l5", "e5a", "b2a",
         "e5b", "glo_g2", "l2c", "b3i", "e6"]
ROTATING = {"GPS L1 C/A", "Galileo E1", "GLONASS G1", "BeiDou B1I",
            "GPS L5", "Galileo E5a", "BeiDou B2a", "Galileo E5b",
            "GLONASS G2", "GPS L2C", "BeiDou B3I", "Galileo E6"}

# ATSC pilots are owned by phase_producer.py now — see module docstring.


def pro_owned():
    """True while live_radio owns the Pro (the 24/7 tracker — AGENTS.md law).
    The gate is the law itself, not a micro-mechanism: a second
    hackrf_transfer against the busy Pro fails inside hackrf_open
    (libusb set_configuration + claim_interface — the claim is at OPEN,
    not start_rx), so no register writes ever land; the earlier commit's
    EP0-retune story was WRONG (round-14 review). What contending cycles
    actually did: process churn + 80 MB /tmp snapshot writes + acquisition
    spawns coincided with stream gaps and tracker-wide reseeds (the
    14:04:40 collapse followed a 131 ms stream gap; the gap's source is
    correlation, not proof). What is verified: zero realigns and zero big
    gaps since this gate went in. Unknown -> owned (fail closed)."""
    try:
        out = subprocess.run(["pgrep", "-f", "examples/live_radio"],
                             capture_output=True, text=True).stdout.split()
        return bool(out)
    except Exception:
        return True


def transfer(serial, f_hz, seconds, path, lna="32", vga="44", bias=False):
    if serial == PRO and pro_owned():
        return False
    n = int(FS * seconds)
    # Never let a leftover capture pass for a fresh one (round-11 review: a
    # days-old file of the right size used to count as a successful capture
    # and published stale IQ under a fresh epoch).
    try:
        os.unlink(path)
    except OSError:
        pass
    t0 = time.time()
    cmd = [f"{TOOLS}/hackrf_transfer", "-d", serial, "-f", str(int(f_hz)),
           "-s", "8000000", "-l", lna, "-g", vga, "-a", "0",
           "-n", str(n), "-r", path]
    if bias:
        cmd += ["-p", "1"]
    try:
        subprocess.run(cmd, capture_output=True, text=True,
                       timeout=seconds + 30, env=ENV)
    except Exception:
        return False
    try:
        st = os.stat(path)
    except OSError:
        return False
    return st.st_size >= n * 2 and st.st_mtime >= t0 - 1.0


def sync_producer_ctl(sig):
    """Pause/resume the 2-s SPI poller so Pro snapshots open cleanly."""
    try:
        out = subprocess.run(["pgrep", "-f", "sync_producer.py"],
                             capture_output=True, text=True).stdout.split()
        for pid in out:
            subprocess.run(["kill", sig, pid], capture_output=True)
    except Exception:
        pass


def clkin_ok():
    """CLKIN-lock check on the One. The One is now held full-time by
    phase_producer.py, so the device usually can't be opened here —
    return None (unknown) in that case instead of a false LOST alarm."""
    try:
        r = subprocess.run([f"{TOOLS}/hackrf_clock", "-d", ONE, "-i"],
                           capture_output=True, text=True, timeout=10, env=ENV)
        if "CLKIN status" not in r.stdout:
            return None                     # busy / unreadable, not "lost"
        return "clock signal detected" in r.stdout
    except Exception:
        return None


def measure_waas():
    """Discrete 5-s L1 snapshot on the Pro -> WAAS GEO Dopplers via sbas_acq."""
    sync_producer_ctl("-STOP")
    try:
        ok = transfer(PRO, L1_HZ, 5.0, L1_SNAP, lna="40", vga="46", bias=True)
    finally:
        sync_producer_ctl("-CONT")
    if not ok:
        return None, None
    d = np.fromfile(L1_SNAP, dtype=np.int8).astype(np.float32)
    health = (float(np.std(d)), float(np.mean(np.abs(d) > 120) * 100))
    iq = np.empty(len(d) // 2, dtype=np.complex64)
    iq.real, iq.imag = d[0::2], d[1::2]
    iq.tofile("/tmp/band_waas.f32")
    try:
        out = run_acq([SBAS, "/tmp/band_waas.f32", "8000000", "4000"], timeout=240)
    except Exception:
        return None, health
    if out is None:
        return None, health
    hits = []
    for line in out.splitlines():
        m = re.search(r"PRN\s+(\d+)\s+metric\s+([\d.]+)\s+dopp\s+([+-]?\d+)", line)
        if m and "ACQUIRED" in line:
            hits.append((int(m.group(1)), float(m.group(2)), float(m.group(3))))
    return (hits or None), health


def drift_row(band, name, ref_hz, off_hz, sigma_hz, epoch, sats, anchor):
    ppm = off_hz / ref_hz * 1e6
    return {
        "band": band, "name": name, "kind": "ClockDriftPpm",
        "value": round(ppm, 4), "sigma": round(sigma_hz / ref_hz * 1e6, 4),
        "ref_hz": ref_hz, "epoch": epoch, "sats": sats, "anchor": anchor,
        "ns_per_s": round(ppm * 1000.0, 1),          # 1 ppm = 1000 ns/s
        "m_per_s": round(ppm * 1e-6 * C_MPS, 2),     # range-rate equivalent
    }


# not "ATSC ch35": that row belongs to phase_producer.py and must survive
MY_BANDS = {"L1 / WAAS"} | ROTATING
_seen = {}              # band -> last epoch with data


def _nice19():
    """Child-side nice(19): acquisition analysis is offline CPU-heavy work
    and must never out-compete live_radio — a >115 ms scheduling stall
    overflows the ~190 ms USB queue and realigns every tracker channel
    (the 2026-08-26 galileo_acq/glonass_acq/e5_acq incidents)."""
    os.nice(19)


# The acq binaries are rayon-parallel and default to all 12 cores — under
# host load they oversubscribe the machine and the tracker stalls anyway
# (nice(19) alone did not prevent the 08:56 realign). Cap their thread
# pool so live_radio always has headroom on this 12-core host.
_ACQ_ENV = {**os.environ, "RAYON_NUM_THREADS": "4"}


def run_acq(cmd, timeout=600):
    """Run an acquisition binary; return stdout or None on any failure."""
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout, preexec_fn=_nice19,
                              env=_ACQ_ENV).stdout
    except Exception:
        return None


PRN_RE = re.compile(r"PRN\s+(\d+)\s+metric\s+([\d.]+)\s+dopp\s+([+-]?\d+)")


def parse_prn(out):
    """(prn, metric, dopp) for every ACQUIRED line of an acq binary."""
    hits = []
    for line in out.splitlines():
        if "ACQUIRED" not in line:
            continue
        m = PRN_RE.search(line)
        if m:
            hits.append((int(m.group(1)), float(m.group(2)), float(m.group(3))))
    return hits


def sat_list(hits):
    """metric 2.5-3.5 is a marginal detection: flag it with a ~ prefix."""
    return [(f"PRN {p}" if m > 3.5 else f"~PRN {p}") for p, m, _ in hits]


def snap(f_hz, secs, path):
    """Discrete Pro snapshot, guarded against the 2-s SPI poller."""
    sync_producer_ctl("-STOP")
    try:
        return transfer(PRO, f_hz, secs, path, lna="40", vga="46", bias=True)
    finally:
        sync_producer_ctl("-CONT")


def measure_galileo():
    """E1B on the same L1 baseband the WAAS run just used — free row."""
    out = run_acq([GAL, "/tmp/band_waas.f32", "8000000", "4000"])
    return (sat_list(parse_prn(out)) or None) if out else None


def measure_upper(snap_hz, secs, snap_path, cmd):
    """Fresh Pro snapshot at snap_hz, then a PRN-printing acq binary."""
    if not snap(snap_hz, secs, snap_path):
        return None
    out = run_acq(cmd)
    return (sat_list(parse_prn(out)) or None) if out else None


def measure_glo_g2():
    if not snap(G2_HZ, 5.0, G2_SNAP):
        return None
    out = run_acq([GLO2, G2_SNAP, "8000000", "1246000000", "5"])
    if not out:
        return None
    hits = re.findall(r"chan\s+([+-]?\d+)\s+\(([\d.]+) MHz\):\s+metric\s+([\d.]+)\s+dopp\s+([+-]?\d+)\s+<== SATELLITE", out)
    sats = []
    for k, mhz, m, d in hits:
        tag = f"k={int(k):+d} ({float(mhz):.1f} MHz)"
        sats.append(tag if float(m) > 3.5 else "~" + tag)
    return sats or None


def measure_gps():
    """GPS C/A PRNs on the same baseband the WAAS run just used — free row."""
    try:
        out = run_acq([ACQ, "/tmp/band_waas.f32", "8000000",
                       "-3000", "3000", "250", "4000"], timeout=300)
        if out is None:
            return None
        res = json.loads(out)
        return [r for r in res if r.get("acquired") and r["prn"] <= 32]
    except Exception:
        return None


def measure_glonass():
    sync_producer_ctl("-STOP")
    try:
        ok = transfer(PRO, GLO_HZ, 5.0, GLO_SNAP, lna="40", vga="46", bias=True)
    finally:
        sync_producer_ctl("-CONT")
    if not ok:
        return None
    try:
        out = run_acq([GLO, GLO_SNAP, "8000000", "1600000000", "5"], timeout=300)
        if out is None:
            return None
    except Exception:
        return None
    hits = re.findall(r"chan\s+([+-]?\d+)\s+\(([\d.]+) MHz\):\s+metric\s+([\d.]+)\s+dopp\s+([+-]?\d+)\s+<== SATELLITE", out)
    return [(int(k), float(mhz), float(m), float(d)) for k, mhz, m, d in hits] or None


def measure_beidou():
    sync_producer_ctl("-STOP")
    try:
        ok = transfer(PRO, B1I_HZ, 6.0, BDS_SNAP, lna="40", vga="46", bias=True)
    finally:
        sync_producer_ctl("-CONT")
    if not ok:
        return None
    try:
        out = run_acq([BDS, BDS_SNAP, "8000000", "1561098000", "6"], timeout=300)
        if out is None:
            return None
    except Exception:
        return None
    hits = re.findall(r"PRN\s+(\d+)\s+metric\s+([\d.]+)\s+dopp\s+([+-]?\d+)\s+<== ACQUIRED", out)
    return [(int(p), float(m), float(d)) for p, m, d in hits] or None


def consensus(rows):
    vals = [(s["value"], max(s["sigma"], 1e-3)) for s in rows
            if s.get("kind") == "ClockDriftPpm" and s.get("value") is not None]
    if len(vals) < 2:
        return None
    w = [1.0 / (sig * sig) for _, sig in vals]
    return sum(v * wi for (v, _), wi in zip(vals, w)) / sum(w)


def main():
    while True:
        epoch = time.time()
        rows, hist = [], {"t": epoch}
        alerts = []

        locked = clkin_ok()
        hist["clkin"] = locked
        if locked is False:
            alerts.append("One CLKIN reference LOST — check the clock cable")

        hits, health = measure_waas()
        if hits:
            dopps = [h[2] for h in hits]
            mean_d = float(np.mean(dopps))
            rows.append(drift_row("L1 / WAAS", "WAAS GEO acquisition · Pro+AA.250, 5-s snapshot",
                                  L1_HZ, mean_d, 50.0, epoch,
                                  [f"PRN {h[0]}" for h in hits], "Pro snapshot @ 1575.42"))
            hist["waas"] = round(mean_d / L1_HZ * 1e6, 4)
            # per-PRN spread: meaconing shifts the whole pack (caught by
            # consensus); a single-satellite spoof stands out from the pack
            if len(hits) >= 2:
                worst = max(hits, key=lambda h: abs(h[2] - mean_d))
                dev = abs(worst[2] - mean_d)
                hist["waas_spread_hz"] = round(dev, 0)
                if dev > PRN_SPREAD_HZ:
                    alerts.append(f"WAAS PRN {worst[0]} Doppler {dev:.0f} Hz from pack mean "
                                  f"— single-satellite spoof candidate")
        if health:
            std, clip = health
            hist["l1_std"], hist["l1_clip"] = round(std, 1), round(clip, 3)
            if std > 36:
                alerts.append(f"L1 noise floor high (std {std:.0f} vs 24 nominal) — wideband interference?")
            if clip > 0.5:
                alerts.append(f"L1 ADC clipping {clip:.2f}% — overload, gains too hot")

        # One rotation slot per cycle — full 101-Doppler acquisitions cost
        # minutes of CPU each, so the bands spread thin: a full sweep takes
        # len(SLOTS) cycles (~25-40 min; panel staleness dimming is 1 h).
        # MEO Doppler is km/s-class: without ephemeris prediction these are
        # presence/health rows, not drift voters. A constellation vanishing
        # or a parade of impossible PRNs is itself the alarm.
        cycle = getattr(main, "_n", 0)
        main._n = cycle + 1
        slot = SLOTS[cycle % len(SLOTS)]
        if slot == "gps+galileo":
            # GPS C/A + Galileo E1B on the same L1 baseband, no new snapshot
            gps = None
            if health:
                try:
                    out = run_acq([ACQ, "/tmp/band_waas.f32", "8000000",
                                   "-3000", "3000", "500", "2000"], timeout=300)
                    # None = analysis failed — skip only this sub-step; a
                    # `continue` here would skip the loop's 45 s sleep and
                    # hot-spin the radio grabs (round-9b review).
                    if out is not None:
                        res = json.loads(out)
                        gps = [r for r in res if r.get("acquired") and r["prn"] <= 32] or None
                except Exception:
                    gps = None
            if gps:
                rows.append({"band": "GPS L1 C/A", "name": "C/A acquisition · same L1 snapshot",
                             "kind": "Presence", "value": None, "sigma": None, "epoch": epoch,
                             "sats": [f"PRN {r['prn']}" for r in gps],
                             "anchor": f"top metric {max(r['metric'] for r in gps):.1f}"})
                hist["gps_sats"] = len(gps)
            # measure_galileo reuses the WAAS snapshot — only while it is
            # FRESH (this cycle's). health is None when the snapshot failed
            # (e.g. Pro owned by the tracker), and a stale file would yield
            # days-old presence rows under a fresh epoch (round-11 review).
            gal = measure_galileo() if health else None
            if gal:
                rows.append({"band": "Galileo E1", "name": "E1B BOC(1,1) acquisition · same L1 snapshot",
                             "kind": "Presence", "value": None, "sigma": None, "epoch": epoch,
                             "sats": gal,
                             "anchor": "Pro snapshot @ 1575.42"})
                hist["gal_sats"] = len(gal)
        elif slot == "glo_g1":
            glo = measure_glonass()
            if glo:
                rows.append({"band": "GLONASS G1", "name": "FDMA channel search · 1602 MHz snapshot",
                             "kind": "Presence", "value": None, "sigma": None, "epoch": epoch,
                             "sats": [f"k={k:+d} ({m:.1f} MHz)" for k, mhz, m, d in glo],
                             "anchor": "Pro snapshot @ 1602"})
                hist["glo_chans"] = len(glo)
        elif slot == "beidou_b1i":
            bds = measure_beidou()
            if bds:
                rows.append({"band": "BeiDou B1I", "name": "B1I acquisition · 1561.098 MHz snapshot",
                             "kind": "Presence", "value": None, "sigma": None, "epoch": epoch,
                             "sats": [f"PRN {p}" for p, m, d in bds],
                             "anchor": "Pro snapshot @ 1561.1"})
                hist["bds_sats"] = len(bds)
        elif slot == "l5":
            sats = measure_upper(L5_HZ, 5.0, L5_SNAP,
                                 [E5, L5_SNAP, "8000000", "1176450000", "5", "l5"])
            if sats:
                rows.append({"band": "GPS L5", "name": "L5 Q5-pilot acquisition · 1176.45 MHz snapshot",
                             "kind": "Presence", "value": None, "sigma": None, "epoch": epoch,
                             "sats": sats, "anchor": "Pro snapshot @ 1176.45"})
                hist["l5_sats"] = len(sats)
        elif slot == "e5a":
            sats = measure_upper(L5_HZ, 5.0, E5A_SNAP,
                                 [E5, E5A_SNAP, "8000000", "1176450000", "5", "e5a"])
            if sats:
                rows.append({"band": "Galileo E5a", "name": "E5a-I acquisition · 1176.45 MHz snapshot",
                             "kind": "Presence", "value": None, "sigma": None, "epoch": epoch,
                             "sats": sats, "anchor": "Pro snapshot @ 1176.45"})
                hist["e5a_sats"] = len(sats)
        elif slot == "b2a":
            sats = measure_upper(L5_HZ, 5.0, B2A_SNAP,
                                 [E5, B2A_SNAP, "8000000", "1176450000", "5", "b2a"])
            if sats:
                rows.append({"band": "BeiDou B2a", "name": "B2a acquisition · 1176.45 MHz snapshot",
                             "kind": "Presence", "value": None, "sigma": None, "epoch": epoch,
                             "sats": sats, "anchor": "Pro snapshot @ 1176.45"})
                hist["b2a_sats"] = len(sats)
        elif slot == "e5b":
            sats = measure_upper(E5B_HZ, 5.0, E5B_SNAP,
                                 [E5B, E5B_SNAP, "8000000", "1207140000", "5", "e5b"])
            if sats:
                rows.append({"band": "Galileo E5b", "name": "E5b-I acquisition · 1207.14 MHz snapshot",
                             "kind": "Presence", "value": None, "sigma": None, "epoch": epoch,
                             "sats": sats, "anchor": "Pro snapshot @ 1207.14"})
                hist["e5b_sats"] = len(sats)
        elif slot == "glo_g2":
            sats = measure_glo_g2()
            if sats:
                rows.append({"band": "GLONASS G2", "name": "L2OF FDMA channel search · 1246 MHz snapshot",
                             "kind": "Presence", "value": None, "sigma": None, "epoch": epoch,
                             "sats": sats, "anchor": "Pro snapshot @ 1246"})
                hist["g2_chans"] = len(sats)
        elif slot == "l2c":
            sats = measure_upper(L2_HZ, 6.0, L2C_SNAP,
                                 [L2C, L2C_SNAP, "8000000", "1227600000", "6"])
            if sats:
                rows.append({"band": "GPS L2C", "name": "L2C CM acquisition · 1227.60 MHz snapshot",
                             "kind": "Presence", "value": None, "sigma": None, "epoch": epoch,
                             "sats": sats, "anchor": "Pro snapshot @ 1227.60"})
                hist["l2c_sats"] = len(sats)
        elif slot == "b3i":
            sats = measure_upper(B3I_HZ, 6.0, B3I_SNAP,
                                 [B3I, B3I_SNAP, "8000000", "1268520000", "6"])
            if sats:
                rows.append({"band": "BeiDou B3I", "name": "B3I acquisition · 1268.52 MHz snapshot",
                             "kind": "Presence", "value": None, "sigma": None, "epoch": epoch,
                             "sats": sats, "anchor": "Pro snapshot @ 1268.52"})
                hist["b3i_sats"] = len(sats)
        elif slot == "e6":
            sats = measure_upper(E6_HZ, 6.0, E6_SNAP,
                                 [E6, E6_SNAP, "8000000", "1278750000", "6"])
            if sats:
                rows.append({"band": "Galileo E6", "name": "E6-B/C acquisition · 1278.75 MHz snapshot",
                             "kind": "Presence", "value": None, "sigma": None, "epoch": epoch,
                             "sats": sats, "anchor": "Pro snapshot @ 1278.75"})
                hist["e6_sats"] = len(sats)

        if not rows:
            # Heartbeat even when every snapshot failed (radio busy with the
            # 24/7 tracker): the server merge expires files by mtime, so a
            # silent producer's rows would VANISH from the panel instead of
            # aging visibly. Touch the state with a fresh epoch; per-row
            # epochs carry the staleness.
            try:
                state = json.load(open(STATE))
            except Exception:
                state = {}
            state["epoch"] = epoch
            tmp = STATE + ".band.tmp"
            json.dump(state, open(tmp, "w"), indent=1)
            os.replace(tmp, STATE)
            time.sleep(45)
            continue

        # consensus vote across the drift rows this producer measured
        voters = rows
        mean = consensus(voters)
        for s in rows:
            _seen[s["band"]] = epoch
            if mean is not None and s.get("value") is not None:
                dev = s["value"] - mean
                z = dev / max(s["sigma"], 1e-3)
                s["vs_consensus_ppm"] = round(dev, 4)
                s["z"] = round(z, 2)
                if abs(z) > 3 and s in voters:
                    s["alert"] = "DIVERGES"
                    alerts.append(f"{s['band']}: {dev:+.3f} ppm from consensus ({z:+.1f} sigma)")
        for band, last in list(_seen.items()):
            if band in ROTATING:
                continue        # per-slot cadence; silence is scheduled
            if epoch - last > LOST_S and band not in {r["band"] for r in rows}:
                alerts.append(f"{band}: source LOST (last seen {(epoch-last)/60:.0f} min ago) — jamming?")

        state = {}
        try:
            state = json.load(open(STATE))
        except Exception:
            pass
        # keep prior rows of my rotating bands that this cycle did not
        # refresh — the panel dims them after 1 h instead of dropping them
        refreshed = {r["band"] for r in rows}
        srcs = [s for s in state.get("sources", [])
                if s.get("band") not in MY_BANDS or s.get("band") not in refreshed]
        srcs.extend(rows)
        state["sources"] = srcs
        state["epoch"] = epoch
        state.setdefault("clock", {})
        # ATSC rows/residual_ppm/atsc_spread_ppm are owned by
        # phase_producer.py now; nothing to update here.
        if mean is not None:
            # single-owner law (round-10): the cross-producer consensus_ppm
            # belongs to series_producer (state.series.json); this band-local
            # mean keeps its own key so the two never merge-race.
            state["band_consensus_ppm"] = round(mean, 4)
        state["alerts"] = alerts
        # CLKIN tri-state (round-8/9b reviews): the r9 probe measures the
        # CLKIN pin frequency in a 9-11 MHz window — i.e. "a 10 MHz-class
        # signal is present", NOT proof the One's clocks run from it (that
        # depends on the input switch state, which this read can't see; and
        # the One is held full-time by phase_producer so the read usually
        # can't even open it -> None). Never let "cabled" read as "locked".
        state.setdefault("clock", {})["clkin_signal_present"] = locked
        tmp = STATE + ".band.tmp"   # unique tmp: sync/phase producers share STATE
        json.dump(state, open(tmp, "w"), indent=1)
        os.replace(tmp, STATE)
        with open(HIST, "a") as fh:
            fh.write(json.dumps(hist) + "\n")
        time.sleep(45)


if __name__ == "__main__":
    main()
