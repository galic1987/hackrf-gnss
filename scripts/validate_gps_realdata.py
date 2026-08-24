#!/usr/bin/env python3
"""Validate the Rust GPS acquisition against the Python oracle on REAL captured
IQ, not synthetic fixtures.

Python's front end (validation/acquire.py) mixes and decimates a real segment of
hackrf_gnss/wideband_l1_b1.iq to baseband; both the Python acquire() and the Rust
`acquire_file` example then run the *identical* samples, and the per-PRN metric
and Doppler are compared. On this station's no-sky-view capture the correct
result is agreement on the ABSENCE of GPS (both < 2.5), with matching metrics.

Build the Rust side first:
    cargo build --release --example acquire_file
Then run this script. Exits non-zero if Rust and Python disagree.
"""
import numpy as np, json, subprocess, os, sys, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "validation"))
from acquire import gps_ca, acquire  # noqa: E402
from scipy.signal import decimate  # noqa: E402

FN = os.path.join(ROOT, "hackrf_gnss", "wideband_l1_b1.iq")
EXE = os.path.join(ROOT, "hackrf_gnss", "target", "release", "examples", "acquire_file")
FS, F_IF, DEC = 20e6, 7.161e6, 4          # LO 1568.259 -> GPS L1 1575.42 MHz
T0, DUR, NBLOCKS = 1.0, 0.05, 20
DLO, DHI, DSTEP = -60000.0, 60000.0, 500.0
# agreement tolerances between the double-precision numpy FFT and rustfft's f32
TOL_REL_METRIC = 0.02
TOL_ABS_METRIC = 0.05


def main():
    if not os.path.exists(FN):
        print(f"no real capture at {FN}"); return 2
    if not os.path.exists(EXE):
        print("build first: cargo build --release --example acquire_file"); return 2

    off = int(T0 * FS) * 2
    n = int(DUR * FS) * 2
    with open(FN, "rb") as f:
        f.seek(off); raw = np.frombuffer(f.read(n), dtype=np.int8).astype(np.float32)
    x = raw[0::2] + 1j * raw[1::2]; x = x - x.mean()
    k = np.arange(len(x))
    x = x * np.exp(-2j * np.pi * F_IF * k / FS)
    sig = decimate(x, DEC, ftype="fir", zero_phase=True).astype(np.complex64)
    fs = FS / DEC
    print(f"real baseband: {len(sig)} samples @ {fs/1e6:.1f} Msps, std={sig.real.std():.2f}")

    bb = os.path.join(tempfile.gettempdir(), "real_bb_gps.f32")
    inter = np.empty(2 * len(sig), dtype=np.float32)
    inter[0::2] = sig.real; inter[1::2] = sig.imag
    inter.tofile(bb)

    dop = np.arange(DLO, DHI + 1, DSTEP, dtype=float)
    py = {}
    for prn in range(1, 33):
        m, d, cp, _ = acquire(sig, fs, gps_ca(prn), 1.023e6, dop, NBLOCKS)
        py[prn] = (float(m), float(d))

    out = subprocess.run([EXE, bb, str(fs), str(DLO), str(DHI), str(DSTEP), str(NBLOCKS)],
                         capture_output=True, text=True)
    os.remove(bb)
    if out.returncode != 0:
        print("RUST ERR:", out.stderr); return 1
    rs = {r["prn"]: (r["metric"], r["doppler"]) for r in json.loads(out.stdout)}

    fails = 0
    pym = np.array([py[p][0] for p in range(1, 33)])
    rsm = np.array([rs[p][0] for p in range(1, 33)])
    for prn in range(1, 33):
        pm, pd = py[prn]; rm, rd = rs[prn]
        rel = abs(rm - pm) / pm
        if abs(rm - pm) > TOL_ABS_METRIC and rel > TOL_REL_METRIC:
            print(f"  PRN {prn}: metric {rm:.3f} vs python {pm:.3f}"); fails += 1
        # Doppler only meaningful where a peak stands out
        if (pm > 2.0 or rm > 2.0) and abs(rd - pd) > DSTEP + 1:
            print(f"  PRN {prn}: doppler {rd} vs python {pd}"); fails += 1

    print(f"metric correlation: {np.corrcoef(pym, rsm)[0,1]:.4f}, "
          f"max abs Δ={np.abs(pym-rsm).max():.3f}")
    print(f"Python max {pym.max():.2f} PRN{int(pym.argmax()+1)}, "
          f"Rust max {rsm.max():.2f} PRN{int(rsm.argmax()+1)}")
    print(f"detections >2.5  python={[p for p in range(1,33) if py[p][0]>2.5]} "
          f"rust={[p for p in range(1,33) if rs[p][0]>2.5]}")
    if fails:
        print(f"FAIL: {fails} disagreement(s)"); return 1
    print("OK: Rust matches Python on real captured IQ")
    return 0


if __name__ == "__main__":
    sys.exit(main())
