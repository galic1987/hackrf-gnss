#!/usr/bin/env python3
"""Truth oracle for porting demod3 to Rust: run one REAL Iridium burst snippet
through validation/demod3.py stage by stage, dumping every intermediate so each
Rust stage can be validated against it (per the dsp-pipeline-diagnosis method).

Dumps (to <outdir>):
  y.f32       baseband after load_bb (interleaved f32 I,Q) at fsy = fs/8
  z2.f32      after resample_poly + RRC (interleaved f32)
  sym.f32     the sampled complex symbols (interleaved f32)
  stages.json scalars: fsy, i0, i1, df, psnr, frac, s0, hint, ds, conf, rwa

usage: demod3_oracle.py <snippet.iq> <outdir>   (fcen parsed from filename)
"""
import numpy as np, sys, os, re, json

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "validation"))
import demod3 as D  # noqa: E402

FC, FS = 1626.25e6, 4e6
T0, DUR = 0.002, 0.0204


def dump_c(path, z):
    a = np.asarray(z, dtype=np.complex64)
    inter = np.empty(2 * len(a), dtype=np.float32)
    inter[0::2] = a.real
    inter[1::2] = a.imag
    inter.tofile(path)


def main():
    snip, outdir = sys.argv[1], sys.argv[2]
    os.makedirs(outdir, exist_ok=True)
    fcen = float(re.search(r"_(\d+\.\d+)\.iq$", snip).group(1)) * 1e6

    # ---- replicate demod3.demod() with default params, dumping intermediates ----
    y, fsy = D.load_bb(snip, T0, DUR, fcen, FC, FS)
    dump_c(os.path.join(outdir, "y.f32"), y)
    i0, i1 = D.burst_edges(y, fsy)
    df, psnr = D.preamble_carrier(y, fsy, i0, pad=16, ls=False, fwin=0.0)

    z = y * np.exp(-2j * np.pi * df * np.arange(len(y)) / fsy)
    g = int(fsy * 0.0002)
    a0 = max(0, i0 - g)
    lead = i0 - a0
    z = z[a0:]
    nb = i1 - a0

    up, dn = int(D.BAUD * D.SPS), int(round(fsy))
    from scipy.signal import resample_poly
    z2 = resample_poly(z, up, dn)
    z2 = np.convolve(z2, D.rrc_c(0.4, D.SPS, 8), "same")
    dump_c(os.path.join(outdir, "z2.f32"), z2)

    sc = (D.BAUD * D.SPS) / fsy
    n0 = int(lead * sc * 0.25)
    n1 = min(len(z2) - D.SPS - 4, int(nb * sc))
    frac = D.timing_om(z2, int(lead * sc), n1)

    K0 = 1
    kk = np.arange(K0, int((n1 - frac) / D.SPS) - 1)
    sym = D._interp(z2, kk * D.SPS + frac, "lin")
    dump_c(os.path.join(outdir, "sym.f32"), sym)
    s0 = int(round(lead * sc / D.SPS))
    b0 = max(0, s0 - K0 - 4)
    b1 = min(len(sym), s0 - K0 + D.NSYM_BURST + 4)
    ds, q_b = D.detect(sym[b0:b1], det="coh", L=33, ddit=2, phase="none",
                       nblk=4, prerot=False, kern="box")
    conf = int(100 * np.mean(q_b > 0.5)) if len(q_b) else 0
    hint = (s0 - K0 - b0) + D.PREAMBLE - 2

    r = {"ds": ds, "q": q_b, "lock": 0.0, "fabs": fcen + df,
         "conf": conf, "psnr": float(psnr), "hint": hint}
    rwa = D.to_line(r, T0 * 1000.0, 0)

    dss = "".join(map(str, ds.tolist()))
    stages = {
        "snippet": os.path.basename(snip), "fcen": fcen, "fc": FC, "fs": FS,
        "t0": T0, "dur": DUR, "fsy": float(fsy), "n_y": int(len(y)),
        "i0": int(i0), "i1": int(i1), "df": float(df), "psnr": float(psnr),
        "n_z2": int(len(z2)), "frac": float(frac), "s0": int(s0),
        "b0": int(b0), "b1": int(b1), "hint": int(hint), "conf": int(conf),
        "ds": dss, "uw_pos": dss.find(D.UW_SYMS), "n_sym": int(len(ds)),
        "rwa": rwa,
    }
    json.dump(stages, open(os.path.join(outdir, "stages.json"), "w"), indent=1)
    print(f"fsy={fsy:.0f} i0={i0} i1={i1} df={df:.1f} frac={frac:.3f} "
          f"hint={hint} uw_pos={stages['uw_pos']} conf={conf}")
    print("RWA:", (rwa or "NONE")[:80])


if __name__ == "__main__":
    main()
