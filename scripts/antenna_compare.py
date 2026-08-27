#!/usr/bin/env python3
"""Antenna A/B comparison on the BENCH HackRF Pro (spare/testing radio).

Captures per-band snapshots, each tuned so the target signal sits near
zero IF (the acquisition binaries search Doppler around 0 IF and do NOT
mix a band offset): L1 at 1575.42 MHz (GPS C/A + Galileo E1 + SBAS share
it) and B1I at 1561.098 MHz, both at 8 Msps. Two prior revisions failed
for window reasons, not antenna reasons: 8 Msps at 1568.25 spans neither
signal, and 16 Msps at 1568.25 leaves L1 at +7.17 MHz — far outside the
acq Doppler search. (Round-15 review chain.)

Runs the crate's offline acquisition binaries and prints a per-PRN metric
table. Two runs (A and B) can then be
diffed:  antenna_compare.py run A --bias   …swap antennas…
         antenna_compare.py run B          ; antenna_compare.py diff A B

LAWS: the bench Pro (serial 645061de…) is the ONLY radio this tool may
touch — the tracker owns Pro#1 and phase_producer owns the One 24/7
(AGENTS.md); any other --serial is refused. Gain is FIXED at the tracker
values (lna 40 / vga 46) so runs are comparable — never A/B with different
gain. Bias-tee is OFF unless --bias is passed (active patch antennas want
it; a passive whip does not; check the antenna is not a DC short first).
Since the bench radio sits on the GPSDO chain, frequency error is identical
across runs — differences are the antenna.
"""
import json, os, subprocess, sys, time

import numpy as np

TOOLS = "/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src"
EX = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "target", "release", "examples")
BENCH = "0000000000000000645061de252d6613"   # Pro#2 — the ONLY allowed radio
FS = 8_000_000                   # per-band captures; signals sit near zero IF
LNA, VGA = "40", "46"

_ACQ_ENV = {**os.environ, "RAYON_NUM_THREADS": "4"}


def _nice19():
    os.nice(19)


def run(cmd, timeout):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                              preexec_fn=_nice19, env=_ACQ_ENV).stdout
    except Exception:
        return None


BANDS = [("l1", 1575420000), ("b1i", 1561098000)]   # zero-IF per band


def capture(label, seconds, bias, band):
    f_hz = dict(BANDS)[band]
    raw = f"/tmp/antenna_{label}_{band}.iq"
    try:
        os.unlink(raw)
    except OSError:
        pass
    n = int(FS * seconds)
    cmd = [f"{TOOLS}/hackrf_transfer", "-d", BENCH, "-f", str(f_hz), "-s", str(FS),
           "-l", LNA, "-g", VGA, "-a", "0", "-n", str(n), "-r", raw]
    if bias:
        cmd += ["-p", "1"]
    t0 = time.time()
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=seconds + 30)
    try:
        st = os.stat(raw)
    except OSError:
        sys.exit(f"capture failed: no output file\n{r.stderr.strip()}")
    if st.st_size < n * 2 or st.st_mtime < t0 - 1:
        sys.exit(f"capture short/stale ({st.st_size} of {n*2} bytes)\n{r.stderr.strip()}")
    d = np.fromfile(raw, dtype=np.int8).astype(np.float32)
    iq = np.empty(len(d) // 2, dtype=np.complex64)
    iq.real, iq.imag = d[0::2], d[1::2]
    f32 = f"/tmp/antenna_{label}_{band}.f32"
    iq.tofile(f32)
    health = (float(np.std(d)), float(np.mean(np.abs(d) > 120) * 100))
    return f32, health


def acquire(f32_l1, f32_b1i):
    """Per-PRN acquisition metrics; each band's capture is zero-IF."""
    out = {}
    f32 = f32_l1
    # GPS C/A (JSON rows)
    o = run([f"{EX}/acquire_file", f32, str(FS), "-3000", "3000", "500", "2000"], 300)
    if o:
        try:
            for r in json.loads(o):
                if r.get("acquired") and r["prn"] <= 32:
                    out[f"G{r['prn']:02d}"] = round(float(r["metric"]), 2)
        except Exception:
            pass
    # Galileo E1B — per-line ACQUIRED law, same as band_producer.parse_prn
    import re
    prn_re = re.compile(r"PRN\s+(\d+)\s+metric\s+([\d.]+)\s+dopp\s+([+-]?\d+)")
    o = run([f"{EX}/galileo_acq", f32, str(FS), "4000"], 300)
    if o:
        for line in o.splitlines():
            m = prn_re.search(line)
            if m and "ACQUIRED" in line:
                out[f"E{int(m.group(1)):02d}"] = round(float(m.group(2)), 2)
    # BeiDou B1I — on the b1i-tuned capture
    o = run([f"{EX}/beidou_acq", f32_b1i, str(FS), "1561098000", "6"], 300)
    if o:
        import re
        for m in re.finditer(r"PRN\s+(\d+)\s+metric\s+([\d.]+)\s+dopp\s+([+-]?\d+)\s+<== ACQUIRED", o):
            out[f"C{int(m.group(1)):02d}"] = round(float(m.group(2)), 2)
    # SBAS GEOs — same per-line law
    o = run([f"{EX}/sbas_acq", f32, str(FS), "4000"], 240)
    if o:
        for line in o.splitlines():
            m = prn_re.search(line)
            if m and "ACQUIRED" in line:
                out[f"S{int(m.group(1))}"] = round(float(m.group(2)), 2)
    return out


def main():
    if len(sys.argv) < 3 or sys.argv[1] not in ("run", "diff"):
        sys.exit(__doc__)
    if sys.argv[1] == "diff":
        a, b = sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else "B"
        A = json.load(open(f"/tmp/antenna_{a}.json"))
        B = json.load(open(f"/tmp/antenna_{b}.json"))
        prns = sorted(set(A) | set(B))
        print(f"{'PRN':6} {a:>8} {b:>8} {'dB-ish Δ':>9}")
        for p in prns:
            va, vb = A.get(p), B.get(p)
            d = (20 * np.log10(vb / va)) if (va and vb) else None
            print(f"{p:6} {va if va is not None else '—':>8} "
                  f"{vb if vb is not None else '—':>8} "
                  f"{('%+.1f' % d) if d is not None else '—':>9}")
        both = [p for p in prns if A.get(p) and B.get(p)]
        if both:
            md = float(np.median([20 * np.log10(B[p] / A[p]) for p in both]))
            print(f"\nmedian Δ over {len(both)} shared PRNs: {md:+.1f} dB-ish "
                  f"(positive = {b} better)")
        return

    label = sys.argv[2]
    bias = "--bias" in sys.argv[3:]
    seconds = 60
    for i, a in enumerate(sys.argv[3:]):
        if a == "--seconds" and i + 4 < len(sys.argv):
            seconds = float(sys.argv[i + 4])
    caps = {}
    for band, _hz in BANDS:
        print(f"capturing {seconds:.0f}s {band} on the BENCH Pro (bias {'ON' if bias else 'OFF'}, "
              f"gain {LNA}/{VGA} fixed)…", flush=True)
        f32, (std, clip) = capture(label, seconds, bias, band)
        print(f"{band} health: std {std:.1f} (24 nominal), clip {clip:.2f}% (>0.5% = gain too hot)")
        caps[band] = f32
    res = acquire(caps["l1"], caps["b1i"])
    json.dump(res, open(f"/tmp/antenna_{label}.json", "w"))
    print(f"{len(res)} PRNs acquired:")
    for p in sorted(res):
        print(f"  {p:5} metric {res[p]}")
    print(f"saved /tmp/antenna_{label}.json — run the other antenna, then: "
          f"antenna_compare.py diff {label} <other>")


if __name__ == "__main__":
    main()
