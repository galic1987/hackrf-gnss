#!/usr/bin/env python3
# extprec_convert.py — convert HackRF Pro extended-precision RX captures.
#
# The 2_extprec_rx gateware emits 4 bytes per sample over SGPIO:
#   [I_lo, I_hi, Q_lo, Q_hi]  where each int16 is little-endian and holds a
#   12-bit SQ(11) sample right-justified (significant range +-2048). The top
#   nibble of each int16 carries the rotating timestamp nibble (see
#   TimestampNibbler in firmware/fpga/dsp/timestamp.py), NOT sign extension,
#   so parsers must mask to 12 bits and re-derive the sign from bit 11.
#
# This tool converts that raw stream to:
#   cs16 (default) : interleaved int16 I/Q — for gnss-sdr and other 16-bit sinks
#   cs8            : interleaved int8 I/Q  — for pipelines expecting HackRF's
#                    native format (e.g. hackrf_gnss Rust examples)
#
# It also removes residual DC (the ext bitstream boots with its DC blocker
# off, and FPGA register state is lost on every bitstream switch).
#
# usage: extprec_convert.py in.iq out.iq [--format cs16|cs8] [--scale N|auto]
#        [--stats-only]

import argparse
import sys

import numpy as np


def read_extprec(path):
    raw = np.fromfile(path, dtype=np.uint8)
    if len(raw) % 4:
        raw = raw[: len(raw) // 4 * 4]
    w = raw.reshape(-1, 4)
    i = w[:, 0].astype(np.int16) | (w[:, 1].astype(np.int16) << 8)
    q = w[:, 2].astype(np.int16) | (w[:, 3].astype(np.int16) << 8)
    # Mask off the timestamp nibble and re-extend the sign from bit 11.
    i = (i << 4) >> 4
    q = (q << 4) >> 4
    return i, q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output", nargs="?")
    ap.add_argument("--format", choices=["cs16", "cs8", "cf32"], default="cs16")
    ap.add_argument(
        "--mix-if",
        type=float,
        default=None,
        metavar="HZ",
        help="rotate spectrum by -HZ before output (brings a signal at +HZ IF "
        "to baseband). Requires --fs for the rotation rate.",
    )
    ap.add_argument("--fs", type=float, default=None, help="sample rate (needed with --mix-if)")
    ap.add_argument(
        "--scale",
        default="auto",
        help="cs8 only: integer right-shift (default) or 'auto' to normalize "
        "RMS to ~24 counts before truncation (keeps weak-signal detail)",
    )
    ap.add_argument("--stats-only", action="store_true")
    args = ap.parse_args()

    i, q = read_extprec(args.input)

    used = int(np.ceil(np.log2(max(np.abs(i).max(), np.abs(q).max(), 1))))
    print(f"samples: {len(i):,}")
    print(f"I range [{i.min()},{i.max()}] sigma {i.std():.1f} mean {i.mean():.2f}")
    print(f"Q range [{q.min()},{q.max()}] sigma {q.std():.1f} mean {q.mean():.2f}")
    print(f"significant bits used: ~{used} of 12")
    clip12 = 100 * np.mean((np.abs(i) >= 2047) | (np.abs(q) >= 2047))
    print(f"12-bit clipping: {clip12:.4f}%")
    if args.stats_only or not args.output:
        return

    # Remove residual DC (ext bitstream's DC blocker defaults off).
    i = i - int(np.round(i.mean()))
    q = q - int(np.round(q.mean()))

    if args.mix_if is not None:
        if not args.fs:
            ap.error("--mix-if requires --fs")
        t = np.arange(len(i), dtype=np.float64) / args.fs
        rot = np.exp(-2j * np.pi * args.mix_if * t).astype(np.complex64)
        sig = (i.astype(np.float32) + 1j * q.astype(np.float32)) * rot
        i = sig.real.astype(np.int16)
        q = sig.imag.astype(np.int16)

    if args.format == "cf32":
        sig = i.astype(np.float32) + 1j * q.astype(np.float32)
        out = np.empty(2 * len(i), dtype=np.float32)
        out[0::2] = sig.real
        out[1::2] = sig.imag
    elif args.format == "cs16":
        out = np.empty(2 * len(i), dtype=np.int16)
        out[0::2] = i
        out[1::2] = q
    else:
        if args.scale == "auto":
            target = 24.0
            rms = np.sqrt(np.mean(i.astype(np.float64) ** 2 + q.astype(np.float64) ** 2))
            gain = target / max(rms, 1e-9)
            fi = np.clip(np.round(i * gain), -127, 127)
            fq = np.clip(np.round(q * gain), -127, 127)
            clip = 100 * (np.mean((np.abs(fi) >= 127) | (np.abs(fq) >= 127)))
            print(f"auto scale x{gain:.2f}, cs8 sigma {fi.std():.1f}, clipping {clip:.4f}%")
        else:
            shift = int(args.scale)
            fi = np.clip(i >> shift, -127, 127)
            fq = np.clip(q >> shift, -127, 127)
        out = np.empty(2 * len(i), dtype=np.int8)
        out[0::2] = fi.astype(np.int8)
        out[1::2] = fq.astype(np.int8)

    out.tofile(args.output)
    print(f"wrote {args.output} ({len(out):,} bytes, {args.format})")


if __name__ == "__main__":
    sys.exit(main())
