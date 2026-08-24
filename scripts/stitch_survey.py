#!/usr/bin/env python3
"""stitch_survey.py — find narrowband tone candidates in an interleaved-i8 IQ
capture for the T8 stitch test. Reports sample stats (clip check) and the top
spectral peaks above the noise floor in a search window, plus whether the same
peak dominates both halves of the capture (a coarse stability check).

usage: stitch_survey.py <cap.iq> <fs> [f_min] [f_max]
"""
import numpy as np
import sys

path, fs = sys.argv[1], float(sys.argv[2])
fmin = float(sys.argv[3]) if len(sys.argv) > 3 else 100e3
fmax = float(sys.argv[4]) if len(sys.argv) > 4 else 0.45 * fs

raw = np.fromfile(path, dtype=np.int8)
i, q = raw[0::2], raw[1::2]
x = i.astype(np.float32) + 1j * q.astype(np.float32)
print(f"{path}: {len(x)} samples")
print(
    f"  std I/Q {i.std():.1f}/{q.std():.1f}  "
    f"clip|>120|: {100 * (np.abs(raw) > 120).mean():.3f}%  "
    f"peak|raw|: {np.abs(raw).max()}"
)

nfft = 1 << 22


def peaks(seg):
    X = np.fft.fft(seg[:nfft], nfft)
    freqs = np.fft.fftfreq(nfft, 1.0 / fs)
    mag = np.abs(X)
    sel = np.where((freqs >= fmin) & (freqs <= fmax))[0]
    floor = np.median(mag[sel])
    out, used = [], []
    for k in sel[np.argsort(mag[sel])[::-1]]:
        if mag[k] < floor * 20:  # < ~26 dB over the floor: stop
            break
        if any(abs(k - u) < 100 for u in used):
            continue
        used.append(k)
        out.append((freqs[k], 20 * np.log10(mag[k] / floor)))
        if len(out) >= 5:
            break
    return out


half = min(len(x), 2 * nfft) // 2
p1, p2 = peaks(x[:half]), peaks(x[half : 2 * half])
for tag, pp in (("first half", p1), ("second half", p2)):
    print(f"  {tag}: " + (", ".join(f"{f / 1e3:.1f} kHz ({snr:.0f} dB)" for f, snr in pp) or "no peaks"))
if p1 and p2 and abs(p1[0][0] - p2[0][0]) < 500:
    print(f"  STABLE: top peak repeats at {p1[0][0] / 1e3:.1f} kHz in both halves")
else:
    print("  no stable dominant tone in window")
