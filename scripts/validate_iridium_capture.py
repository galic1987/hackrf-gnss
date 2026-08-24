#!/usr/bin/env python3
"""Validate the end-to-end Rust Iridium decoder (find_bursts2 + demod3) against
the Python oracle on a REAL capture. Both find bursts and demodulate them; the
burst lists and decoded frame bits are compared.

Because a live capture is written continuously, this first FREEZES a byte-exact
snapshot so both decoders see identical input.

Build first: cargo build --release --example find_bursts --example iridium_decode
usage: validate_iridium_capture.py <capture.iq> [dur_s]
"""
import numpy as np, json, os, sys, subprocess, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "validation"))
import demod3 as D  # noqa: E402

FC, FS = 1626.25e6, 4e6
FB = os.path.join(ROOT, "hackrf_gnss", "target", "release", "examples", "find_bursts")
DEC = os.path.join(ROOT, "hackrf_gnss", "target", "release", "examples", "iridium_decode")


def main():
    src = sys.argv[1]
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    if not (os.path.exists(FB) and os.path.exists(DEC)):
        print("build: cargo build --release --example find_bursts --example iridium_decode")
        return 2

    # freeze a byte-exact snapshot (a live capture keeps growing)
    frozen = os.path.join(tempfile.gettempdir(), "cap_frozen.iq")
    need = int((dur + 4.2) * FS) * 2
    with open(src, "rb") as fi, open(frozen, "wb") as fo:
        fo.write(fi.read(need))
    print(f"frozen {os.path.getsize(frozen)/1e6:.0f} MB")

    # ---- bursts ----
    pyb = D.find_bursts2(frozen, FC, FS, dur, lo=1626.0e6, hi=1626.5e6, thr_db=4.0)
    rsb = json.loads(subprocess.run([FB, frozen, str(dur)], capture_output=True, text=True).stdout)
    rsb = [(b["t"], b["dur"], b["fcen"]) for b in rsb]

    def near(b, arr):
        return any(abs(a[0] - b[0]) < 0.006 and abs(a[2] - b[2]) < 20000 for a in arr)
    pbm = sum(1 for b in pyb if near(b, rsb))
    rbm = sum(1 for b in rsb if near(b, pyb))
    print(f"bursts: python {len(pyb)}, rust {len(rsb)}; "
          f"py-matched {pbm}/{len(pyb)}, rs-matched {rbm}/{len(rsb)}")

    # ---- frames ----
    _, pyl = D.run_file(frozen, FC, FS, dur, lo=1626.0e6, hi=1626.5e6, thr_db=4.0, jobs=4)
    pybits = sorted(l.split()[-1] for l in pyl)
    rsout = subprocess.run([DEC, frozen, str(dur)], capture_output=True, text=True).stdout
    rsbits = sorted(l.split()[-1] for l in rsout.splitlines() if l.strip())

    # pair frames by length, min Hamming
    used, tot, totbits, exact = set(), 0, 0, 0
    for pb in pybits:
        best = None
        for i, rb in enumerate(rsbits):
            if i in used or len(rb) != len(pb):
                continue
            h = sum(1 for a, b in zip(pb, rb) if a != b)
            if best is None or h < best[1]:
                best = (i, h)
        if best is None:
            continue
        used.add(best[0]); tot += best[1]; totbits += len(pb)
        exact += best[1] == 0
    agree = 100 * (1 - tot / totbits) if totbits else 0
    print(f"frames: python {len(pyl)}, rust {len(rsbits)}; "
          f"exact {exact}/{len(pyl)}, bit agreement {agree:.3f}%")
    os.remove(frozen)

    ok = (pbm == len(pyb) and rbm == len(rsb) and len(rsbits) == len(pyl) and agree > 99.0)
    print("\n" + ("OK: Rust end-to-end Iridium matches Python on real capture"
                  if ok else "REVIEW: agreement below expectation"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
