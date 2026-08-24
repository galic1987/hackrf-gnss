#!/usr/bin/env python3
"""Validate the Rust demod3 port against the Python oracle on MANY real Iridium
burst snippets. For each snippet both decoders run; the differential symbols and
the decoded RWA frame bits are compared. Reports exact-match rates.

Build first: cargo build --release --example demod3_stage
usage: validate_demod3_realdata.py [N]   (default 200 snippets)
"""
import numpy as np, glob, re, os, sys, subprocess, json

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "validation"))
import demod3 as D  # noqa: E402

FC, FS = 1626.25e6, 4e6
T0, DUR = 0.002, 0.0204
EXE = os.path.join(ROOT, "hackrf_gnss", "target", "release", "examples", "demod3_stage")


def python_decode(path, fcen):
    r = D.demod(path, T0, DUR, fcen, FC, FS)
    if not r:
        return None, None
    ds = "".join(map(str, r["ds"].tolist()))
    ln = D.to_line(r, T0 * 1000.0, 0)
    bits = ln.split()[-1] if ln else None
    return ds, bits


def rust_decode(path, fcen):
    out = subprocess.run([EXE, path, str(fcen)], capture_output=True, text=True)
    if out.returncode != 0:
        return None, None
    j = json.loads(out.stdout)
    return j["ds"], (j["rwa_bits"] or None)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    snips = sorted(glob.glob(os.path.join(ROOT, "observations", "snippets", "*.iq")))
    # spread across the set rather than the first N (all from one cycle)
    step = max(1, len(snips) // n)
    snips = snips[::step][:n]
    print(f"comparing {len(snips)} real snippets")

    ds_exact = bits_exact = both_decoded = py_only = rust_only = 0
    ds_ham = []
    for p in snips:
        m = re.search(r"_(\d+\.\d+)\.iq$", p)
        if not m:
            continue
        fcen = float(m.group(1)) * 1e6
        pds, pbits = python_decode(p, fcen)
        rds, rbits = rust_decode(p, fcen)
        if pbits and rbits:
            both_decoded += 1
            if pbits == rbits:
                bits_exact += 1
            if pds is not None and rds is not None:
                k = min(len(pds), len(rds))
                h = sum(1 for i in range(k) if pds[i] != rds[i]) + abs(len(pds) - len(rds))
                ds_ham.append(h)
                if h == 0:
                    ds_exact += 1
        elif pbits and not rbits:
            py_only += 1
        elif rbits and not pbits:
            rust_only += 1

    print(f"\nboth produced a frame: {both_decoded}")
    print(f"  identical RWA bits:   {bits_exact}/{both_decoded} "
          f"({100*bits_exact/max(both_decoded,1):.1f}%)")
    print(f"  identical symbols:    {ds_exact}/{both_decoded} "
          f"({100*ds_exact/max(both_decoded,1):.1f}%)")
    if ds_ham:
        a = np.array(ds_ham)
        print(f"  symbol Hamming dist:  median {np.median(a):.0f}, "
              f"mean {a.mean():.2f}, p95 {np.percentile(a,95):.0f}, max {a.max()}")
    print(f"only Python decoded: {py_only}, only Rust decoded: {rust_only}")
    ok = bits_exact >= 0.95 * both_decoded and both_decoded > 0
    print("\n" + ("OK: Rust demod3 matches Python on real bursts"
                  if ok else "REVIEW: agreement below 95%"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
