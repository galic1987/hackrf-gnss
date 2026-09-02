#!/usr/bin/env python3
"""Retired antenna A/B comparison; offline ``diff`` mode remains available.

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

MEASUREMENT DISCIPLINE (round-18 review): use an ABBA sequence (known-good
/ unknown / unknown / known-good) without touching receiver, cable, adapter,
or gain between runs; report matched-PRN acquisition probability and metric
differences per band; the acq metric is a correlation figure — do NOT quote
20*log10 of its ratio as antenna gain. Start one gain step BELOW production
and confirm clip stays < 0.5% before using lna 40 / vga 46; RF amp stays off.
Active antennas: the Pro bias-tee is 3.3 V/50 mA max — the antenna's 3-5 V
rating is compatible only if its steady-state draw is under 50 mA; get the
part's current rating before enabling bias.

QUARANTINE: there is no spare bench Pro in the current station. Pro #1 is
dead; Pro #2 is the production GNSS receiver owned by the tracker; and the
One is the ClearStream ATSC receiver owned by phase_producer. ``run`` exits
before opening a radio. Existing JSON results can still be compared with
``diff``; do not infer antenna gain from its "dB-ish" correlation ratio.
"""
import json, os, subprocess, sys, time

import numpy as np

TOOLS = "/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src"
EX = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "target", "release", "examples")
BENCH = "QUARANTINED_NO_SERIAL"
FS = 8_000_000                   # per-band captures; signals sit near zero IF
LNA, VGA = "40", "46"

_ACQ_ENV = {**os.environ, "RAYON_NUM_THREADS": "4"}


def _nice19():
    os.nice(19)


def run(cmd, timeout):
    """(stdout, error-note). Failures are REPORTED, never read as 'no
    acquisitions' (round-18: silent None conversion made tool crashes
    indistinguishable from a dead antenna)."""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                           preexec_fn=_nice19, env=_ACQ_ENV)
        if r.returncode != 0:
            return None, f"exit {r.returncode}: {r.stderr.strip()[:120]}"
        return r.stdout, None
    except subprocess.TimeoutExpired:
        return None, f"timeout >{timeout} s"
    except Exception as e:
        return None, str(e)


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
    return f32, health, raw


def acquire(f32_l1, raw_b1i):
    """Per-PRN acquisition metrics + per-engine error report. `f32_l1` is the
    L1-centered complex64 capture (GPS/GAL/SBAS engines want zero-IF f32);
    `raw_b1i` is the B1I-centered RAW int8 capture path (beidou_acq reads
    int8 and mixes the IF itself — passing f32 was the round-18 bug that
    made the B1I leg silently invalid)."""
    import re
    out, errs = {}, {}
    prn_re = re.compile(r"PRN\s+(\d+)\s+metric\s+([\d.]+)\s+dopp\s+([+-]?\d+)")
    engines = [
        ("GPS", [f"{EX}/acquire_file", f32_l1, str(FS), "-3000", "3000", "500", "2000"], 300),
        ("GAL", [f"{EX}/galileo_acq", f32_l1, str(FS), "4000"], 300),
        ("BDS", [f"{EX}/beidou_acq", raw_b1i, str(FS), "1561098000", "6"], 300),
        ("SBAS", [f"{EX}/sbas_acq", f32_l1, str(FS), "4000"], 240),
    ]
    for name, cmd, to in engines:
        o, err = run(cmd, to)
        if err:
            errs[name] = err
            continue
        if not o:
            errs[name] = "no output"
            continue
        if name == "GPS":
            try:
                for r in json.loads(o):
                    if r.get("acquired") and r["prn"] <= 32:
                        out[f"G{r['prn']:02d}"] = round(float(r["metric"]), 2)
            except Exception as e:
                errs[name] = f"parse: {e}"
            continue
        for line in o.splitlines():
            m = prn_re.search(line)
            if not m:
                continue
            if name == "BDS":
                if "<== ACQUIRED" in line:
                    out[f"C{int(m.group(1)):02d}"] = round(float(m.group(2)), 2)
            elif "ACQUIRED" in line:
                pre = "E" if name == "GAL" else "S"
                out[f"{pre}{int(m.group(1)):02d}" if name == "GAL" else f"S{int(m.group(1))}"] = \
                    round(float(m.group(2)), 2)
    return out, errs


def main():
    if len(sys.argv) < 3 or sys.argv[1] not in ("run", "diff"):
        sys.exit(__doc__)
    if sys.argv[1] == "run":
        print(
            "QUARANTINED: Pro #2 is the tracker-owned production radio, not a "
            "spare bench receiver; no radio was opened.",
            file=sys.stderr,
        )
        raise SystemExit(78)
    if sys.argv[1] == "diff":
        a, b = sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else "B"
        A = json.load(open(f"/tmp/antenna_{a}.json")).get("metrics", {})
        B = json.load(open(f"/tmp/antenna_{b}.json")).get("metrics", {})
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
    caps, raws = {}, {}
    for band, _hz in BANDS:
        print(f"capturing {seconds:.0f}s {band} on the BENCH Pro (bias {'ON' if bias else 'OFF'}, "
              f"gain {LNA}/{VGA} fixed)…", flush=True)
        f32, (std, clip), raw = capture(label, seconds, bias, band)
        print(f"{band} health: std {std:.1f} (24 nominal), clip {clip:.2f}% (>0.5% = gain too hot)")
        caps[band], raws[band] = f32, raw
    res, errs = acquire(caps["l1"], raws["b1i"])
    json.dump({"metrics": res, "errors": errs}, open(f"/tmp/antenna_{label}.json", "w"))
    if errs:
        print("ENGINE FAILURES (these mean the run is INVALID, not a weak antenna):")
        for k, v in errs.items():
            print(f"  {k}: {v}")
    print(f"{len(res)} PRNs acquired:")
    for p in sorted(res):
        print(f"  {p:5} metric {res[p]}")
    print(f"saved /tmp/antenna_{label}.json — run the other antenna, then: "
          f"antenna_compare.py diff {label} <other>")


if __name__ == "__main__":
    main()
