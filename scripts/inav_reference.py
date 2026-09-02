#!/usr/bin/env python3
"""Galileo E1B I/NAV reference implementation (pure stdlib python).

Work package A of the Galileo I/NAV ranging lever
(docs/superpowers/specs/2026-09-02-galileo-inav-ranging-spec.md).

This module is the PROVEN oracle for the Rust `src/galileo_inav.rs` decoder:
every numeric behavior of the transmit chain (word -> CRC-24Q -> tails ->
K=7 conv encode with inverted G2 -> 30x8 block interleave -> sync prepend),
the receive inverse (sync search both polarities, deinterleave, soft Viterbi,
CRC zero-syndrome check, word-type parse) and the ephemeris/clock/BGD/GGTO
evaluation is implemented here, exercised by scripts/test_inav_reference.py,
and pinned into JSON vectors under tests/fixtures/inav/ which the Rust tests
consume at the maintenance window.

Conventions mirrored from this codebase (so Rust reuse is byte-identical):
- src/sbas.rs crc24q(): bitwise MSB-first, poly 0x1864CFB, 24-bit register.
- src/sbas.rs conv_tables(): full = (bit << 6) | state; c0 = parity(full &
  0o171); c1 = parity(full & 0o133) ^ invert_g2; next = full >> 1.
- src/sbas.rs viterbi(): soft > 0 means symbol bit 0 (soft = 1 - 2*sym).
- src/gps/broadcast.rs kepler_e(): 12 Newton iterations from ek = m.
- src/gps/broadcast.rs wrap_tk(): fold into (-302400, +302400].
- src/beidou_d1.rs sat_at_txtime_bds(): tau = 0.075 start, 2 iterations,
  Sagnac rotation by omega_e*tau, clock evaluated at t_tx - tau.
- sbas.rs test Rng: xorshift64 (<<13, >>7, <<17), gauss via Box-Muller on
  (next()>>11 + 0.5)/2^53.

ICD references (Galileo OS SIS ICD v2.1, Nov 2023) are cited inline; the
constants were adversarially verified against the ICD PDF per the spec.
"""

import json
import math
import os

# ---------------------------------------------------------------------------
# constants (ICD-verified per the spec)
# ---------------------------------------------------------------------------

CRC24Q_POLY = 0x1864CFB          # ICD 5.1.9.4 == sbas.rs CRC24Q_POLY
K = 7
G1 = 0o171                       # ICD Table 24
G2 = 0o133                       # ICD Table 24 (output INVERTED on air)
NSTATES = 1 << (K - 1)

SYNC = [0, 1, 0, 1, 1, 0, 0, 0, 0, 0]   # ICD 4.3.2.1, not encoded/interleaved

INTER_COLS = 30                  # ICD Table 25: written column count
INTER_ROWS = 8                   # read row count
PART_SYMS = INTER_COLS * INTER_ROWS      # 240 coded symbols per page part
PART_BITS = 120                  # 114 information + 6 tail
PAGE_SYMS = 2 * (len(SYNC) + PART_SYMS)  # 500 (even + odd, syncs included)
WORD_BITS = 128                  # Data(1/2) 112 + Data(2/2) 16
CRC_SPAN_BITS = 196              # even 114 + odd 82 (ICD Table 36 note)
MIN_SPAN_WEIGHT = 12             # sbas.rs MIN_BLOCK_WEIGHT applied to the span

MU_GAL = 3.986004418e14          # ICD Table 66 (== MU_BDS, != GPS MU_E)
OMEGA_E = 7.2921151467e-5        # ICD Table 66 (GPS value, != BDS omega_e)
F_REL_GAL = -4.442807309e-10     # ICD Eq. 15 (GPS: -4.442807633e-10)
C_LIGHT = 299_792_458.0
WEEK_S = 604800.0
GST_GPS_WEEK_OFFSET = 1024       # GST WN 0 == GPS week 1024 (ICD 5.1.2)

# RINEX 3.04 Galileo Data Sources bits (gLAB reference, verified on live
# brdc_latest.rnx records this session: 258 = F/NAV (bit1|bit8),
# 513/516/517 = I/NAV (bit0 and/or bit2, always with bit9)).
DS_INAV_E1B = 1 << 0             # I/NAV E1-B
DS_INAV_E5B = 1 << 2             # I/NAV E5b-I (same message)
DS_CLOCK_E5B_E1 = 1 << 9         # af0-2 are the (E5b,E1) pair -> I/NAV clock


# ---------------------------------------------------------------------------
# deterministic Rng mirroring sbas.rs tests (xorshift64)
# ---------------------------------------------------------------------------

M64 = (1 << 64) - 1


class Rng:
    """xorshift64, byte-identical to the sbas.rs test Rng."""

    def __init__(self, seed):
        self.s = seed & M64

    def next(self):
        s = self.s
        s = (s ^ (s << 13)) & M64
        s ^= s >> 7
        s = (s ^ (s << 17)) & M64
        self.s = s
        return s

    def bit(self):
        return self.next() & 1

    def below(self, n):
        return self.next() % n

    def gauss(self):
        u1 = ((self.next() >> 11) + 0.5) / float(1 << 53)
        u2 = ((self.next() >> 11) + 0.5) / float(1 << 53)
        return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)


# ---------------------------------------------------------------------------
# bit helpers
# ---------------------------------------------------------------------------

def ubits(bits, off, n):
    """Unsigned int from bits[off:off+n], MSB first."""
    v = 0
    for b in bits[off:off + n]:
        v = (v << 1) | (b & 1)
    return v


def sbits(bits, off, n):
    """Two's-complement int from bits[off:off+n]."""
    v = ubits(bits, off, n)
    if v >= 1 << (n - 1):
        v -= 1 << n
    return v


def put_bits(bits, off, n, value):
    """Write `value` (may be negative; masked to n bits) MSB-first."""
    v = value & ((1 << n) - 1)
    for i in range(n):
        bits[off + i] = (v >> (n - 1 - i)) & 1


def bits_to_hex(bits):
    """Pack MSB-first into hex; pads with zeros to a byte boundary."""
    v = 0
    for b in bits:
        v = (v << 1) | (b & 1)
    v <<= (-len(bits)) % 8
    nb = (len(bits) + 7) // 8
    return v.to_bytes(nb, "big").hex()


def hex_to_bits(hexstr, nbits):
    raw = bytes.fromhex(hexstr)
    bits = []
    for byte in raw:
        for i in range(7, -1, -1):
            bits.append((byte >> i) & 1)
    return bits[:nbits]


# ---------------------------------------------------------------------------
# CRC-24Q (bitwise port of sbas.rs crc24q)
# ---------------------------------------------------------------------------

def crc24q(bits):
    mask = CRC24Q_POLY & 0xFFFFFF
    reg = 0
    for b in bits:
        top = ((reg >> 23) & 1) ^ (b & 1)
        reg = (reg << 1) & 0xFFFFFF
        if top:
            reg ^= mask
    return reg


# ---------------------------------------------------------------------------
# K=7 rate-1/2 convolutional code (port of sbas.rs conv machinery)
# ---------------------------------------------------------------------------

def _parity(x):
    x ^= x >> 4
    x ^= x >> 2
    x ^= x >> 1
    return x & 1


def conv_tables(invert_g2):
    out = [[[0, 0], [0, 0]] for _ in range(NSTATES)]
    nxt = [[0, 0] for _ in range(NSTATES)]
    for s in range(NSTATES):
        for b in range(2):
            full = (b << (K - 1)) | s
            c0 = _parity(full & G1)
            c1 = _parity(full & G2)
            if invert_g2:
                c1 ^= 1
            out[s][b] = [c0, c1]
            nxt[s][b] = full >> 1
    return out, nxt


def conv_encode(bits, invert_g2, state=0):
    """G1 symbol first; 2 symbols per bit. Same signature shape as sbas.rs."""
    out, nxt = conv_tables(invert_g2)
    sym = []
    s = state
    for b in bits:
        b &= 1
        sym.append(out[s][b][0])
        sym.append(out[s][b][1])
        s = nxt[s][b]
    return sym


def viterbi(soft, invert_g2):
    """Soft-decision Viterbi; soft[i] > 0 means symbol bit 0 (1 - 2*sym).

    Structural port of sbas.rs viterbi(): unknown initial state, add-compare-
    select over 64 states, per-step metric normalization, traceback from the
    best final state. Output is len(soft)//2 bits.
    """
    out, _nxt = conv_tables(invert_g2)
    m = len(soft) // 2
    s0 = [[0.0, 0.0] for _ in range(NSTATES)]
    s1 = [[0.0, 0.0] for _ in range(NSTATES)]
    for ns in range(NSTATES):
        b = (ns >> 5) & 1
        for j in range(2):
            ps = 2 * (ns & 31) + j
            s0[ns][j] = 1.0 - 2.0 * out[ps][b][0]
            s1[ns][j] = 1.0 - 2.0 * out[ps][b][1]
    metric = [0.0] * NSTATES
    dec = [0] * (m * NSTATES)
    for k in range(m):
        r0 = soft[2 * k]
        r1 = soft[2 * k + 1]
        nxt_metric = [0.0] * NSTATES
        for ns in range(NSTATES):
            best = -math.inf
            bestj = 0
            for j in range(2):
                ps = 2 * (ns & 31) + j
                c = metric[ps] + r0 * s0[ns][j] + r1 * s1[ns][j]
                if c > best:
                    best = c
                    bestj = j
            nxt_metric[ns] = best
            dec[k * NSTATES + ns] = bestj
        mx = max(nxt_metric)
        metric = [v - mx for v in nxt_metric]
    s = max(range(NSTATES), key=lambda i: metric[i])
    bits = [0] * m
    for k in range(m - 1, -1, -1):
        bits[k] = (s >> 5) & 1
        s = 2 * (s & 31) + dec[k * NSTATES + s]
    return bits


def viterbi_hard(hard, invert_g2):
    """Hard-decision decode via the soft engine (+-1.0 inputs)."""
    return viterbi([1.0 - 2.0 * (b & 1) for b in hard], invert_g2)


# ---------------------------------------------------------------------------
# 30x8 block interleaver (ICD Table 25)
# ---------------------------------------------------------------------------
# Transmit: 240 encoded symbols are WRITTEN into a matrix of 30 columns x
# 8 rows column-by-column and READ row-by-row. Hence
#   transmitted[30*row + col] = encoded[8*col + row].
# Receiver inverse (PocketSDR: syms.reshape(8, 30).T.ravel()):
#   decoded_input[8*col + row] = received[30*row + col].

def interleave(sym):
    assert len(sym) == PART_SYMS
    out = [0] * PART_SYMS
    for row in range(INTER_ROWS):
        for col in range(INTER_COLS):
            out[INTER_COLS * row + col] = sym[INTER_ROWS * col + row]
    return out


def deinterleave(sym):
    assert len(sym) == PART_SYMS
    out = [0] * PART_SYMS
    for row in range(INTER_ROWS):
        for col in range(INTER_COLS):
            out[INTER_ROWS * col + row] = sym[INTER_COLS * row + col]
    return out


# ---------------------------------------------------------------------------
# page forward chain (ICD Table 36, E1-B column)
# ---------------------------------------------------------------------------

def build_page_parts(word_bits, osnma=None, sar=None, spare=None, ssp=None,
                     page_type=0):
    """word (128 bits) -> (even 120 bits, odd 120 bits), CRC included.

    Even: E/O=0 | PT | Data(1/2)[112]              | tail 6
    Odd:  E/O=1 | PT | Data(2/2)[16] | OSNMA 40 | SAR 22 | spare 2
          | CRC 24 | SSP 8 | tail 6
    CRC-24Q covers even[0:114] ++ odd[0:82] (196 bits); SSP/tails excluded.
    """
    assert len(word_bits) == WORD_BITS
    osnma = osnma if osnma is not None else [0] * 40
    sar = sar if sar is not None else [0] * 22
    spare = spare if spare is not None else [0] * 2
    ssp = ssp if ssp is not None else [0] * 8
    assert (len(osnma), len(sar), len(spare), len(ssp)) == (40, 22, 2, 8)
    even114 = [0, page_type & 1] + list(word_bits[:112])
    odd82 = [1, page_type & 1] + list(word_bits[112:]) + osnma + sar + spare
    crc = crc24q(even114 + odd82)
    crc_bits = [(crc >> (23 - i)) & 1 for i in range(24)]
    even = even114 + [0] * 6
    odd = odd82 + crc_bits + ssp + [0] * 6
    assert len(even) == PART_BITS and len(odd) == PART_BITS
    return even, odd


def encode_part(part_bits, invert_g2=True):
    """120 bits -> 250 symbols (sync ++ interleave(conv_encode)).

    Each part encodes independently from state 0: the previous part's 6-bit
    zero tail returns the encoder to state 0 (ICD 4.3.2.2).
    """
    assert len(part_bits) == PART_BITS
    return list(SYNC) + interleave(conv_encode(part_bits, invert_g2, 0))


def encode_page(word_bits, invert_g2=True, **part_kw):
    """word -> 500 hard symbols (even part then odd part, syncs included)."""
    even, odd = build_page_parts(word_bits, **part_kw)
    return encode_part(even, invert_g2) + encode_part(odd, invert_g2)


# ---------------------------------------------------------------------------
# receive inverse
# ---------------------------------------------------------------------------

def find_sync(soft, start=0):
    """First sync at/after `start`, trying both polarities.

    Returns (index, polarity) with polarity +1 (soft>0 == bit 0, upright) or
    -1 (inverted), or None. A candidate matches when every one of the 10
    sync positions has the right sign; zero-valued symbols never match.
    """
    n = len(soft)
    want = [1.0 - 2.0 * b for b in SYNC]
    for i in range(start, n - len(SYNC) + 1):
        up = all(soft[i + k] * want[k] > 0.0 for k in range(len(SYNC)))
        if up:
            return i, +1
        dn = all(soft[i + k] * want[k] < 0.0 for k in range(len(SYNC)))
        if dn:
            return i, -1
    return None


def decode_part(soft240, polarity=1):
    """240 soft symbols (sync stripped) -> 120 decoded bits."""
    assert len(soft240) == PART_SYMS
    s = [polarity * v for v in soft240]
    return viterbi(deinterleave(s), invert_g2=True)


def check_page(even120, odd120):
    """CRC/structure check on two decoded parts.

    Returns dict: crc_ok, weight_ok, eo_ok, page_type, word (128 bits, only
    when crc_ok and nominal), alert (True when PT=1 CRC-clean -> content is
    discarded, fail-closed per spec Section 2).
    """
    span = even120[:114] + odd120[:82]
    crc_rx = odd120[82:106]
    syndrome = crc24q(span + crc_rx)
    w = sum(span)
    weight_ok = MIN_SPAN_WEIGHT <= w <= CRC_SPAN_BITS - MIN_SPAN_WEIGHT
    crc_ok = syndrome == 0 and weight_ok
    eo_ok = even120[0] == 0 and odd120[0] == 1
    page_type = even120[1]
    out = {
        "crc_ok": crc_ok,
        "weight_ok": weight_ok,
        "eo_ok": eo_ok,
        "page_type": page_type,
        "alert": bool(crc_ok and eo_ok and page_type == 1),
        "word": None,
    }
    if crc_ok and eo_ok and page_type == 0 and odd120[1] == 0:
        out["word"] = even120[2:114] + odd120[2:18]
    return out


def decode_page_at(soft, i, polarity):
    """Decode the 500-symbol page whose even-part sync starts at soft[i]."""
    if i + PAGE_SYMS > len(soft):
        return None
    even = decode_part(soft[i + 10:i + 250], polarity)
    odd = decode_part(soft[i + 260:i + 500], polarity)
    res = check_page(even, odd)
    res["sym_index"] = i
    res["polarity"] = polarity
    return res


def find_pages(soft):
    """Scan a soft-symbol stream; return CRC-validated pages newest-last.

    Mirrors the spec's find_pages contract: sync candidates at every offset,
    both polarities, page accepted only on CRC pass (alert pages surface with
    word=None). Advances by a full page on success, one symbol otherwise.
    """
    pages = []
    i = 0
    while i + PAGE_SYMS <= len(soft):
        hit = find_sync(soft, i)
        if hit is None:
            break
        j, pol = hit
        res = decode_page_at(soft, j, pol)
        if res is not None and res["crc_ok"] and res["eo_ok"]:
            pages.append(res)
            i = j + PAGE_SYMS
        else:
            i = j + 1
    return pages


# ---------------------------------------------------------------------------
# word parsers (ICD 4.3.5; offsets per spec Section 3, 0-based from word MSB)
# ---------------------------------------------------------------------------

PI = math.pi  # semicircle -> rad multiplier, GPS/BDS parser convention


def parse_word(word):
    """128-bit word -> {'word_type', 'raw': {...}, 'fields': {...}} or None.

    Raw values are the transmitted integers (signed already sign-extended);
    fields carry the scaled engineering values (semicircle fields in rad).
    Word types outside {0,1,2,3,4,5,6,10} return None (ignored, not error).
    """
    assert len(word) == WORD_BITS
    wt = ubits(word, 0, 6)
    raw = {}
    fields = {}

    if wt == 1:  # ephemeris 1/4, Table 40
        raw = {"iodnav": ubits(word, 6, 10), "t0e": ubits(word, 16, 14),
               "m0": sbits(word, 30, 32), "e": ubits(word, 62, 32),
               "sqrt_a": ubits(word, 94, 32)}
        fields = {"iodnav": raw["iodnav"], "toe": raw["t0e"] * 60.0,
                  "m0": raw["m0"] * 2.0**-31 * PI,
                  "e": raw["e"] * 2.0**-33,
                  "sqrt_a": raw["sqrt_a"] * 2.0**-19}
    elif wt == 2:  # ephemeris 2/4, Table 41
        raw = {"iodnav": ubits(word, 6, 10), "omega0": sbits(word, 16, 32),
               "i0": sbits(word, 48, 32), "omega": sbits(word, 80, 32),
               "idot": sbits(word, 112, 14)}
        fields = {"iodnav": raw["iodnav"],
                  "omega0": raw["omega0"] * 2.0**-31 * PI,
                  "i0": raw["i0"] * 2.0**-31 * PI,
                  "omega": raw["omega"] * 2.0**-31 * PI,
                  "idot": raw["idot"] * 2.0**-43 * PI}
    elif wt == 3:  # ephemeris 3/4 + SISA, Table 42
        raw = {"iodnav": ubits(word, 6, 10), "omega_dot": sbits(word, 16, 24),
               "delta_n": sbits(word, 40, 16), "cuc": sbits(word, 56, 16),
               "cus": sbits(word, 72, 16), "crc": sbits(word, 88, 16),
               "crs": sbits(word, 104, 16), "sisa": ubits(word, 120, 8)}
        fields = {"iodnav": raw["iodnav"],
                  "omega_dot": raw["omega_dot"] * 2.0**-43 * PI,
                  "delta_n": raw["delta_n"] * 2.0**-43 * PI,
                  "cuc": raw["cuc"] * 2.0**-29, "cus": raw["cus"] * 2.0**-29,
                  "crc": raw["crc"] * 2.0**-5, "crs": raw["crs"] * 2.0**-5,
                  "sisa": raw["sisa"]}
    elif wt == 4:  # ephemeris 4/4 + clock, Table 43
        raw = {"iodnav": ubits(word, 6, 10), "svid": ubits(word, 16, 6),
               "cic": sbits(word, 22, 16), "cis": sbits(word, 38, 16),
               "t0c": ubits(word, 54, 14), "af0": sbits(word, 68, 31),
               "af1": sbits(word, 99, 21), "af2": sbits(word, 120, 6)}
        fields = {"iodnav": raw["iodnav"], "svid": raw["svid"],
                  "cic": raw["cic"] * 2.0**-29, "cis": raw["cis"] * 2.0**-29,
                  "toc": raw["t0c"] * 60.0, "af0": raw["af0"] * 2.0**-34,
                  "af1": raw["af1"] * 2.0**-46, "af2": raw["af2"] * 2.0**-59}
    elif wt == 5:  # iono + BGD + health + GST, Table 44
        raw = {"ai0": ubits(word, 6, 11), "ai1": sbits(word, 17, 11),
               "ai2": sbits(word, 28, 14),
               "region": [ubits(word, 42 + i, 1) for i in range(5)],
               "bgd_e1e5a": sbits(word, 47, 10),
               "bgd_e1e5b": sbits(word, 57, 10),
               "e5b_hs": ubits(word, 67, 2), "e1b_hs": ubits(word, 69, 2),
               "e5b_dvs": ubits(word, 71, 1), "e1b_dvs": ubits(word, 72, 1),
               "wn": ubits(word, 73, 12), "tow": ubits(word, 85, 20)}
        fields = {"ai0": raw["ai0"] * 2.0**-2, "ai1": raw["ai1"] * 2.0**-8,
                  "ai2": raw["ai2"] * 2.0**-15,
                  "bgd_e1e5a": raw["bgd_e1e5a"] * 2.0**-32,
                  "bgd_e1e5b": raw["bgd_e1e5b"] * 2.0**-32,
                  "e5b_hs": raw["e5b_hs"], "e1b_hs": raw["e1b_hs"],
                  "e5b_dvs": raw["e5b_dvs"], "e1b_dvs": raw["e1b_dvs"],
                  "wn": raw["wn"], "tow": raw["tow"]}
    elif wt == 6:  # GST-UTC, Table 45
        raw = {"a0": sbits(word, 6, 32), "a1": sbits(word, 38, 24),
               "dt_ls": sbits(word, 62, 8), "t0t": ubits(word, 70, 8),
               "wn0t": ubits(word, 78, 8), "wn_lsf": ubits(word, 86, 8),
               "dn": ubits(word, 94, 3), "dt_lsf": sbits(word, 97, 8),
               "tow": ubits(word, 105, 20)}
        fields = {"a0": raw["a0"] * 2.0**-30, "a1": raw["a1"] * 2.0**-50,
                  "dt_ls": raw["dt_ls"], "t0t": raw["t0t"] * 3600.0,
                  "wn0t": raw["wn0t"], "wn_lsf": raw["wn_lsf"],
                  "dn": raw["dn"], "dt_lsf": raw["dt_lsf"],
                  "tow": raw["tow"]}
    elif wt == 0:  # spare word with time, Table 52
        time_flag = ubits(word, 6, 2)
        raw = {"time_flag": time_flag, "wn": ubits(word, 96, 12),
               "tow": ubits(word, 108, 20)}
        fields = {"time_valid": time_flag == 2}
        if time_flag == 2:  # WN/TOW valid ONLY when Time == binary '10'
            fields["wn"] = raw["wn"]
            fields["tow"] = raw["tow"]
    elif wt == 10:  # almanac tail + GGTO, Table 49
        raw = {"ioda": ubits(word, 6, 4),
               "alm_omega0": sbits(word, 10, 16),
               "alm_omega_dot": sbits(word, 26, 11),
               "alm_m0": sbits(word, 37, 16), "alm_af0": sbits(word, 53, 16),
               "alm_af1": sbits(word, 69, 13),
               "alm_e5b_hs": ubits(word, 82, 2),
               "alm_e1b_hs": ubits(word, 84, 2),
               "a0g": sbits(word, 86, 16), "a1g": sbits(word, 102, 12),
               "t0g": ubits(word, 114, 8), "wn0g": ubits(word, 122, 6)}
        # GGTO invalid sentinel: all four fields all-ones (ICD 5.1.8).
        sentinel = (ubits(word, 86, 16) == 0xFFFF
                    and ubits(word, 102, 12) == 0xFFF
                    and ubits(word, 114, 8) == 0xFF
                    and ubits(word, 122, 6) == 0x3F)
        fields = {"ggto_valid": not sentinel}
        if not sentinel:
            fields.update({"a0g": raw["a0g"] * 2.0**-35,
                           "a1g": raw["a1g"] * 2.0**-51,
                           "t0g": raw["t0g"] * 3600.0,
                           "wn0g": raw["wn0g"]})
    else:
        return None
    return {"word_type": wt, "raw": raw, "fields": fields}


# --------------------------- word builders (raw ints -> 128 bits) ----------

def _new_word(wt):
    w = [0] * WORD_BITS
    put_bits(w, 0, 6, wt)
    return w


def make_word1(iodnav, t0e, m0, e, sqrt_a):
    w = _new_word(1)
    put_bits(w, 6, 10, iodnav)
    put_bits(w, 16, 14, t0e)
    put_bits(w, 30, 32, m0)
    put_bits(w, 62, 32, e)
    put_bits(w, 94, 32, sqrt_a)
    return w


def make_word2(iodnav, omega0, i0, omega, idot):
    w = _new_word(2)
    put_bits(w, 6, 10, iodnav)
    put_bits(w, 16, 32, omega0)
    put_bits(w, 48, 32, i0)
    put_bits(w, 80, 32, omega)
    put_bits(w, 112, 14, idot)
    return w


def make_word3(iodnav, omega_dot, delta_n, cuc, cus, crc_h, crs, sisa):
    w = _new_word(3)
    put_bits(w, 6, 10, iodnav)
    put_bits(w, 16, 24, omega_dot)
    put_bits(w, 40, 16, delta_n)
    put_bits(w, 56, 16, cuc)
    put_bits(w, 72, 16, cus)
    put_bits(w, 88, 16, crc_h)
    put_bits(w, 104, 16, crs)
    put_bits(w, 120, 8, sisa)
    return w


def make_word4(iodnav, svid, cic, cis, t0c, af0, af1, af2):
    w = _new_word(4)
    put_bits(w, 6, 10, iodnav)
    put_bits(w, 16, 6, svid)
    put_bits(w, 22, 16, cic)
    put_bits(w, 38, 16, cis)
    put_bits(w, 54, 14, t0c)
    put_bits(w, 68, 31, af0)
    put_bits(w, 99, 21, af1)
    put_bits(w, 120, 6, af2)
    return w


def make_word5(ai0, ai1, ai2, bgd_e1e5a, bgd_e1e5b, e5b_hs, e1b_hs,
               e5b_dvs, e1b_dvs, wn, tow, region=(0, 0, 0, 0, 0)):
    w = _new_word(5)
    put_bits(w, 6, 11, ai0)
    put_bits(w, 17, 11, ai1)
    put_bits(w, 28, 14, ai2)
    for i, r in enumerate(region):
        put_bits(w, 42 + i, 1, r)
    put_bits(w, 47, 10, bgd_e1e5a)
    put_bits(w, 57, 10, bgd_e1e5b)
    put_bits(w, 67, 2, e5b_hs)
    put_bits(w, 69, 2, e1b_hs)
    put_bits(w, 71, 1, e5b_dvs)
    put_bits(w, 72, 1, e1b_dvs)
    put_bits(w, 73, 12, wn)
    put_bits(w, 85, 20, tow)
    return w


def make_word6(a0, a1, dt_ls, t0t, wn0t, wn_lsf, dn, dt_lsf, tow):
    w = _new_word(6)
    put_bits(w, 6, 32, a0)
    put_bits(w, 38, 24, a1)
    put_bits(w, 62, 8, dt_ls)
    put_bits(w, 70, 8, t0t)
    put_bits(w, 78, 8, wn0t)
    put_bits(w, 86, 8, wn_lsf)
    put_bits(w, 94, 3, dn)
    put_bits(w, 97, 8, dt_lsf)
    put_bits(w, 105, 20, tow)
    return w


def make_word0(time_flag, wn, tow):
    w = _new_word(0)
    put_bits(w, 6, 2, time_flag)
    put_bits(w, 96, 12, wn)
    put_bits(w, 108, 20, tow)
    return w


def make_word10(ioda, a0g, a1g, t0g, wn0g, alm=None):
    w = _new_word(10)
    put_bits(w, 6, 4, ioda)
    alm = alm or {}
    put_bits(w, 10, 16, alm.get("omega0", 0))
    put_bits(w, 26, 11, alm.get("omega_dot", 0))
    put_bits(w, 37, 16, alm.get("m0", 0))
    put_bits(w, 53, 16, alm.get("af0", 0))
    put_bits(w, 69, 13, alm.get("af1", 0))
    put_bits(w, 82, 2, alm.get("e5b_hs", 0))
    put_bits(w, 84, 2, alm.get("e1b_hs", 0))
    put_bits(w, 86, 16, a0g)
    put_bits(w, 102, 12, a1g)
    put_bits(w, 114, 8, t0g)
    put_bits(w, 122, 6, wn0g)
    return w


# ---------------------------------------------------------------------------
# ephemeris batch assembly (words 1-4 same IODnav; word 4 carries clock)
# ---------------------------------------------------------------------------

def assemble_ephemeris(w1, w2, w3, w4, w5=None, gst_wn=None):
    """Parsed words -> BrdcEph-shaped dict, or None on IODnav mismatch.

    IODnav must match across words 1-4 (ICD 5.1.9.2). BGD/health ride word 5
    (no IODnav — paired by recency, per spec 5.1). `gst_wn` (from word 5 or
    another TOW source) sets week = GST WN + 1024 (GPS-continuous alignment).
    """
    ws = [w1, w2, w3, w4]
    if any(w is None or w["word_type"] != i + 1 for i, w in enumerate(ws)):
        return None
    iods = {w["fields"]["iodnav"] for w in ws}
    if len(iods) != 1:
        return None
    f1, f2, f3, f4 = (w["fields"] for w in ws)
    eph = {
        "iodnav": f1["iodnav"], "toe": f1["toe"], "m0": f1["m0"],
        "e": f1["e"], "sqrt_a": f1["sqrt_a"],
        "omega0": f2["omega0"], "i0": f2["i0"], "omega": f2["omega"],
        "idot": f2["idot"],
        "omega_dot": f3["omega_dot"], "delta_n": f3["delta_n"],
        "cuc": f3["cuc"], "cus": f3["cus"], "crc": f3["crc"],
        "crs": f3["crs"], "sisa": f3["sisa"],
        "svid": f4["svid"], "cic": f4["cic"], "cis": f4["cis"],
        "toc": f4["toc"], "af0": f4["af0"], "af1": f4["af1"],
        "af2": f4["af2"],
        "bgd_e1e5b": 0.0, "health": 0,
    }
    if w5 is not None and w5["word_type"] == 5:
        f5 = w5["fields"]
        eph["bgd_e1e5b"] = f5["bgd_e1e5b"]
        # two-sided health law: composite HS<<1|DVS, 0 == healthy
        hs, dvs = f5["e1b_hs"], f5["e1b_dvs"]
        eph["health"] = 0 if (hs == 0 and dvs == 0) else ((hs << 1) | dvs)
        if gst_wn is None:
            gst_wn = f5["wn"]
    if gst_wn is not None:
        eph["week"] = gst_wn + GST_GPS_WEEK_OFFSET
    return eph


# ---------------------------------------------------------------------------
# Keplerian evaluation (GPS pipeline, Galileo constants — ICD Table 66)
# ---------------------------------------------------------------------------

def kepler_e(m, ecc):
    """12 Newton iterations from ek = m (src/gps/broadcast.rs kepler_e)."""
    ek = m
    for _ in range(12):
        ek -= (ek - ecc * math.sin(ek) - m) / (1.0 - ecc * math.cos(ek))
    return ek


def wrap_tk(tk):
    if tk > WEEK_S / 2.0:
        tk -= WEEK_S
    elif tk < -WEEK_S / 2.0:
        tk += WEEK_S
    return tk


def sat_pos_ecef_gal(e, t):
    """Satellite ECEF (m) at GST time-of-week t. Mirrors sat_pos_ecef."""
    a = e["sqrt_a"] * e["sqrt_a"]
    n0 = math.sqrt(MU_GAL / (a * a * a))
    tk = wrap_tk(t - e["toe"])
    mk = e["m0"] + (n0 + e["delta_n"]) * tk
    ek = kepler_e(mk, e["e"])
    se, ce = math.sin(ek), math.cos(ek)
    vk = math.atan2(math.sqrt(1.0 - e["e"] * e["e"]) * se, ce - e["e"])
    phik = vk + e["omega"]
    s2, c2 = math.sin(2.0 * phik), math.cos(2.0 * phik)
    uk = phik + e["cus"] * s2 + e["cuc"] * c2
    rk = a * (1.0 - e["e"] * ce) + e["crs"] * s2 + e["crc"] * c2
    ik = e["i0"] + e["cis"] * s2 + e["cic"] * c2 + e["idot"] * tk
    xp, yp = rk * math.cos(uk), rk * math.sin(uk)
    om = e["omega0"] + (e["omega_dot"] - OMEGA_E) * tk - OMEGA_E * e["toe"]
    co, so = math.cos(om), math.sin(om)
    ci, si = math.cos(ik), math.sin(ik)
    return [xp * co - yp * ci * so, xp * so + yp * ci * co, yp * si]


def sat_clock_gal(e, t):
    """dt_sv for the E1-only user: clock poly + relativity - BGD(E1,E5b).

    ICD Eq. 13-14 give the (E1,E5b) clock; Eq. 17 (f1 = E1) subtracts the
    broadcast BGD(E1,E5b) — the exact analogue of GPS `- e.tgd`.
    """
    dt = wrap_tk(t - e["toc"])
    poly = e["af0"] + e["af1"] * dt + e["af2"] * dt * dt
    a = e["sqrt_a"] * e["sqrt_a"]
    n0 = math.sqrt(MU_GAL / (a * a * a))
    tk = wrap_tk(t - e["toe"])
    mk = e["m0"] + (n0 + e["delta_n"]) * tk
    ek = kepler_e(mk, e["e"])
    rel = F_REL_GAL * e["e"] * e["sqrt_a"] * math.sin(ek)
    return poly + rel - e.get("bgd_e1e5b", 0.0)


def sat_at_txtime_gal(e, t_tx, rx_m):
    """(sat ECEF Sagnac-rotated, dt_sv, geometric range) at transmit time.

    Structural twin of beidou_d1::sat_at_txtime_bds (tau=0.075 start, two
    iterations, rotation by OMEGA_E*tau, clock at t_tx - tau) with Galileo
    constants.
    """
    tau = 0.075
    s = [0.0, 0.0, 0.0]
    for _ in range(2):
        s0 = sat_pos_ecef_gal(e, t_tx - tau)
        th = OMEGA_E * tau
        ct, st = math.cos(th), math.sin(th)
        s = [s0[0] * ct + s0[1] * st, -s0[0] * st + s0[1] * ct, s0[2]]
        d = [s[i] - rx_m[i] for i in range(3)]
        tau = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2) / C_LIGHT
    dt = sat_clock_gal(e, t_tx - tau)
    d = [s[i] - rx_m[i] for i in range(3)]
    rng = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
    return s, dt, rng


# ---------------------------------------------------------------------------
# GGTO (ICD 5.1.8 Eq. 23) — spec decision Option B
# ---------------------------------------------------------------------------

def ggto_offset(a0g, a1g, t0g, wn0g, tow, wn):
    """dt_systems = t_Galileo - t_GPS at (wn, tow) GST.

    wn0g is GST week mod 64; roll-over law |dW| <= 31. The anchor converts
    t_tx_gpst = t_tx_gst - dt_systems.
    """
    dw = (wn - wn0g) % 64
    if dw > 31:
        dw -= 64
    return a0g + a1g * (tow - t0g + WEEK_S * dw)


def apply_ggto(t_tx_gst, word10_fields, tow, wn):
    """t_tx GST -> GPST using a parsed word 10; identity when invalid."""
    f = word10_fields
    if not f or not f.get("ggto_valid"):
        return t_tx_gst  # fail-closed: apply zero, residual folds into ISB
    dt = ggto_offset(f["a0g"], f["a1g"], f["t0g"], f["wn0g"], tow, wn)
    return t_tx_gst - dt


# ---------------------------------------------------------------------------
# RINEX 3 Galileo (E) records — hand parser for fixture generation
# ---------------------------------------------------------------------------

def _rnx_f(s):
    s = s.strip().replace("D", "e").replace("d", "e")
    return float(s) if s else 0.0


def _jdn(y, m, d):
    a = (14 - m) // 12
    yy = y + 4800 - a
    mm = m + 12 * a - 3
    return d + (153 * mm + 2) // 5 + 365 * yy + yy // 4 - yy // 100 \
        + yy // 400 - 32045


def gps_week_sow(y, mo, d, h, mi, s):
    """Calendar (GAL/GPS time frame) -> (continuous GPS week, seconds-of-week)."""
    days = _jdn(y, mo, d) - _jdn(1980, 1, 6)
    return days // 7, (days % 7) * 86400 + h * 3600 + mi * 60 + s


def parse_rinex_gal_record(lines8):
    """One 8-line RINEX 3.04 'E' record -> eph dict (fields in radians/s/m).

    Layout per spec 5.2: line0 toc/af0-2; orbit1 IODnav,Crs,delta_n,M0;
    orbit2 Cuc,e,Cus,sqrtA; orbit3 toe,Cic,OMEGA0,Cis; orbit4 i0,Crc,omega,
    OMEGAdot; orbit5 idot,DataSources,GALweek; orbit6 SISA,health,BGD(E1,E5a),
    BGD(E1,E5b); orbit7 t_tm. RINEX GAL angles are already radians.
    """
    l0 = lines8[0]
    sv = l0[:3].strip()
    y, mo, d = int(l0[4:8]), int(l0[9:11]), int(l0[12:14])
    h, mi, s = int(l0[15:17]), int(l0[18:20]), int(l0[21:23])
    vals = [_rnx_f(l0[23 + k * 19: 23 + (k + 1) * 19]) for k in range(3)]
    for li in range(1, 8):
        line = lines8[li]
        for k in range(4):
            off = 4 + k * 19
            vals.append(_rnx_f(line[off:off + 19]) if len(line) > off else 0.0)
    week, sow = gps_week_sow(y, mo, d, h, mi, s)
    eph = {
        "sv": sv, "prn": int(sv[1:3]),
        "toc": float(sow), "af0": vals[0], "af1": vals[1], "af2": vals[2],
        "iodnav": int(vals[3]), "crs": vals[4], "delta_n": vals[5],
        "m0": vals[6], "cuc": vals[7], "e": vals[8], "cus": vals[9],
        "sqrt_a": vals[10], "toe": vals[11], "cic": vals[12],
        "omega0": vals[13], "cis": vals[14], "i0": vals[15],
        "crc": vals[16], "omega": vals[17], "omega_dot": vals[18],
        "idot": vals[19], "data_sources": int(vals[20]),
        "week": int(vals[21]),  # continuous GPS-aligned GAL week
        "sisa": vals[23], "health": int(vals[24]),
        "bgd_e1e5a": vals[25], "bgd_e1e5b": vals[26], "t_tm": vals[27],
        "toc_week": week,
    }
    return eph


def rinex_gal_record_is_inav(eph):
    """I/NAV clock-pair selection gate (spec 5.2, verified on live records):
    bit 9 (af0-2 are the E5b,E1 pair) is the discriminator; bit 0/2 flag the
    I/NAV broadcast signal. F/NAV records (258 = bit1|bit8) are rejected —
    their clock/BGD slots are the (E1,E5a) pair."""
    ds = eph["data_sources"]
    return bool(ds & DS_CLOCK_E5B_E1) and bool(ds & (DS_INAV_E1B | DS_INAV_E5B))


def parse_rinex_gal(text):
    """All 'E' records in a RINEX 3 nav file body -> list of eph dicts."""
    lines = text.splitlines()
    try:
        h = next(i for i, l in enumerate(lines) if "END OF HEADER" in l)
    except StopIteration:
        h = -1
    out = []
    i = h + 1
    while i < len(lines):
        l = lines[i]
        if len(l) >= 3 and l[0] == "E" and l[1:3].strip().isdigit():
            out.append(parse_rinex_gal_record(lines[i:i + 8]))
            i += 8
        else:
            i += 1
    return out


# ---------------------------------------------------------------------------
# fixture generation
# ---------------------------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURE_DIR = os.path.normpath(os.path.join(HERE, "..", "tests", "fixtures",
                                            "inav"))
BRDC_PATH = "/Volumes/Radiator 8TB/gnss/observations/brdc_latest.rnx"

_PROV = ("Generated by scripts/inav_reference.py (pure-python Galileo I/NAV "
         "reference, work package A of "
         "docs/superpowers/specs/2026-09-02-galileo-inav-ranging-spec.md). "
         "Regenerate: python3 scripts/inav_reference.py --gen-fixtures. "
         "Pinned 2026-09-02 after scripts/test_inav_reference.py passed "
         "against the running python implementation.")


def _noisy(sym, sigma, rng):
    # xorshift64 needs warm-up: with a small seed the first next() values
    # have few high bits set, so the first Box-Muller u1 is tiny and the
    # first gauss draw is a huge outlier (enough to flip a sync symbol).
    # 16 discarded draws, mirrored exactly by the Rust fixture consumers.
    for _ in range(16):
        rng.next()
    return [(1.0 - 2.0 * s) + sigma * rng.gauss() for s in sym]


def gen_crc_vectors():
    ascii_bits = []
    for byte in b"123456789":
        ascii_bits += [(byte >> i) & 1 for i in range(7, -1, -1)]
    rng = Rng(0x1CD24)
    span = [rng.bit() for _ in range(CRC_SPAN_BITS)]
    crc = crc24q(span)
    crc_bits = [(crc >> (23 - i)) & 1 for i in range(24)]
    return {
        "_provenance": _PROV + " CRC-24Q check values: the Qualcomm standard "
        "'123456789' vector and the sbas.rs crc24q_known_vectors "
        "degenerate cases (src/sbas.rs tests), plus an ICD-form 196-bit "
        "page span (Rng(0x1CD24) xorshift64 bits) with its appended-CRC "
        "zero-syndrome property (220-bit form, ICD 5.1.9.4).",
        "poly": CRC24Q_POLY,
        "vectors": [
            {"name": "qualcomm_123456789", "bits_hex": bits_to_hex(ascii_bits),
             "nbits": len(ascii_bits), "crc": crc24q(ascii_bits),
             "crc_expect": 0xCDE703},
            {"name": "all_zeros_250", "bits_hex": bits_to_hex([0] * 250),
             "nbits": 250, "crc": 0},
            {"name": "all_ones_250", "bits_hex": bits_to_hex([1] * 250),
             "nbits": 250, "crc": crc24q([1] * 250), "crc_expect": 0xEF339D},
            {"name": "inav_196_span_rng0x1CD24",
             "bits_hex": bits_to_hex(span), "nbits": CRC_SPAN_BITS,
             "crc": crc, "zero_syndrome_over_220": crc24q(span + crc_bits)},
        ],
    }


def gen_conv_vectors():
    rng = Rng(0xC0DE)
    cases = []
    # noiseless roundtrip, both G2 conventions
    for inv in (True, False):
        bits = [rng.bit() for _ in range(PART_BITS)]
        sym = conv_encode(bits, inv, 0)
        dec = viterbi([1.0 - 2.0 * s for s in sym], inv)
        cases.append({"name": f"noiseless_invert_g2_{inv}",
                      "invert_g2": inv, "bits_hex": bits_to_hex(bits),
                      "nbits": PART_BITS, "sym_hex": bits_to_hex(sym),
                      "nsym": len(sym), "decoded_matches": dec == bits})
    # gaussian-noise soft case (sigma 0.4)
    bits = [rng.bit() for _ in range(PART_BITS)]
    sym = conv_encode(bits, True, 0)
    nrng = Rng(0x50F7)
    soft = _noisy(sym, 0.4, nrng)
    dec = viterbi(soft, True)
    cases.append({"name": "gauss_sigma0.4_invert_g2_true", "invert_g2": True,
                  "bits_hex": bits_to_hex(bits), "nbits": PART_BITS,
                  "soft": soft, "decoded_matches": dec == bits,
                  "noise": "Rng(0x50F7) gauss after 16 warmup next() draws, "
                  "sigma=0.4, soft=1-2*sym+n"})
    # hard symbol flips (12 of 240, spread)
    bits = [rng.bit() for _ in range(PART_BITS)]
    sym = conv_encode(bits, True, 0)
    flipped = list(sym)
    flip_idx = [7 + 20 * i for i in range(12)]
    for i in flip_idx:
        flipped[i] ^= 1
    dec = viterbi([1.0 - 2.0 * s for s in flipped], True)
    cases.append({"name": "hard_12flips_invert_g2_true", "invert_g2": True,
                  "bits_hex": bits_to_hex(bits), "nbits": PART_BITS,
                  "sym_flipped_hex": bits_to_hex(flipped), "nsym": len(sym),
                  "flip_indices": flip_idx, "decoded_matches": dec == bits})
    # polarity complement decodes to complement-adjacent garbage -> pinned
    # via the negative page cases instead; here pin the invert_g2 guard:
    bits = [rng.bit() for _ in range(PART_BITS)]
    sym_true = conv_encode(bits, True, 0)
    sym_false = conv_encode(bits, False, 0)
    cases.append({"name": "g2_convention_differs", "bits_hex": bits_to_hex(bits),
                  "nbits": PART_BITS, "sym_invert_hex": bits_to_hex(sym_true),
                  "sym_noninvert_hex": bits_to_hex(sym_false),
                  "identical": sym_true == sym_false})
    return {
        "_provenance": _PROV + " Conv K=7 G1=171o G2=133o vectors generated "
        "with the sbas.rs state convention (full=(b<<6)|s, next=full>>1); "
        "invert_g2=True is the on-air Galileo convention (ICD Table 24 / "
        "Fig. 13). Bit sources: Rng(0xC0DE); noise Rng(0x50F7).",
        "cases": cases,
    }


def gen_interleaver_vectors():
    ident = list(range(PART_SYMS))
    perm = [0] * PART_SYMS
    for row in range(INTER_ROWS):
        for col in range(INTER_COLS):
            perm[INTER_COLS * row + col] = INTER_ROWS * col + row
    rng = Rng(0x1B1B)
    pat = [rng.bit() for _ in range(PART_SYMS)]
    return {
        "_provenance": _PROV + " 30x8 block interleaver (ICD Table 25): "
        "written 30 columns x 8 rows column-by-column, read row-by-row; "
        "transmitted[30*row+col] = encoded[8*col+row]. permutation[i] gives "
        "the encoded-stream index transmitted at position i. Bit pattern "
        "from Rng(0x1B1B).",
        "rows": INTER_ROWS, "cols": INTER_COLS,
        "permutation": perm,
        "identity_interleaved": interleave(ident),
        "pattern_in_hex": bits_to_hex(pat),
        "pattern_out_hex": bits_to_hex(interleave(pat)),
        "nsym": PART_SYMS,
        "roundtrip_ok": deinterleave(interleave(pat)) == pat,
    }


def _page_case(name, word, note, sigma=None, noise_seed=None, polarity=1,
               invert_g2=True, page_type=0, osnma_seed=None):
    kw = {}
    if osnma_seed is not None:
        orng = Rng(osnma_seed)
        kw = {"osnma": [orng.bit() for _ in range(40)],
              "sar": [orng.bit() for _ in range(22)],
              "spare": [orng.bit() for _ in range(2)],
              "ssp": [orng.bit() for _ in range(8)]}
    kw["page_type"] = page_type
    sym = encode_page(word, invert_g2=invert_g2, **kw)
    if sigma is not None:
        soft = _noisy(sym, sigma, Rng(noise_seed))
    else:
        soft = [1.0 - 2.0 * s for s in sym]
    if polarity < 0:
        soft = [-v for v in soft]
    parsed = parse_word(word)
    case = {
        "name": name, "note": note, "invert_g2": invert_g2,
        "page_type": page_type, "polarity": polarity,
        "word_hex": bits_to_hex(word),
        "symbols_hex": bits_to_hex(sym), "nsym": len(sym),
        "parsed": parsed,
    }
    if sigma is not None:
        case["soft"] = soft
        case["noise"] = (f"Rng({noise_seed:#x}) gauss sigma={sigma}, "
                         "16 warmup next() draws discarded")
    # prove against the running implementation before pinning
    res = decode_page_at(soft, 0, find_sync(soft)[1])
    case["decode_crc_ok"] = res["crc_ok"]
    case["decode_alert"] = res["alert"]
    case["decode_word_matches"] = (res["word"] == word) if res["word"] else False
    return case


def gen_page_vectors(rinex_eph):
    """Full synthetic pages words 1..5 (+0, 6, 10) built from a REAL RINEX
    ephemeris (raw ints derived by inverse scaling), plus negative cases."""
    e = rinex_eph

    def r(v, scale):
        return int(round(v / scale))

    iod = e["iodnav"]
    gst_wn = e["week"] - GST_GPS_WEEK_OFFSET
    w1 = make_word1(iod, r(e["toe"], 60.0), r(e["m0"] / PI, 2.0**-31),
                    r(e["e"], 2.0**-33), r(e["sqrt_a"], 2.0**-19))
    w2 = make_word2(iod, r(e["omega0"] / PI, 2.0**-31),
                    r(e["i0"] / PI, 2.0**-31), r(e["omega"] / PI, 2.0**-31),
                    r(e["idot"] / PI, 2.0**-43))
    w3 = make_word3(iod, r(e["omega_dot"] / PI, 2.0**-43),
                    r(e["delta_n"] / PI, 2.0**-43), r(e["cuc"], 2.0**-29),
                    r(e["cus"], 2.0**-29), r(e["crc"], 2.0**-5),
                    r(e["crs"], 2.0**-5), 107)
    w4 = make_word4(iod, e["prn"], r(e["cic"], 2.0**-29),
                    r(e["cis"], 2.0**-29), r(e["toc"], 60.0),
                    r(e["af0"], 2.0**-34), r(e["af1"], 2.0**-46),
                    r(e["af2"], 2.0**-59))
    w5 = make_word5(ai0=45, ai1=-12, ai2=7,
                    bgd_e1e5a=r(e["bgd_e1e5a"], 2.0**-32),
                    bgd_e1e5b=r(e["bgd_e1e5b"], 2.0**-32),
                    e5b_hs=0, e1b_hs=0, e5b_dvs=0, e1b_dvs=0,
                    wn=gst_wn % 4096, tow=int(e["toe"]) + 25)
    w6 = make_word6(a0=-17, a1=3, dt_ls=18, t0t=int(e["toe"]) // 3600,
                    wn0t=(gst_wn % 256), wn_lsf=(gst_wn % 256), dn=3,
                    dt_lsf=18, tow=int(e["toe"]) + 5)
    w0 = make_word0(time_flag=2, wn=gst_wn % 4096, tow=int(e["toe"]) + 55)
    w10 = make_word10(ioda=7, a0g=-232, a1g=5, t0g=int(e["toe"]) // 3600,
                      wn0g=gst_wn % 64,
                      alm={"omega0": 1234, "omega_dot": -55, "m0": -321,
                           "af0": 99, "af1": -3, "e5b_hs": 0, "e1b_hs": 0})
    w0_invalid = make_word0(time_flag=1, wn=gst_wn % 4096,
                            tow=int(e["toe"]) + 55)
    w10_sentinel = make_word10(ioda=7, a0g=0xFFFF, a1g=0xFFF, t0g=0xFF,
                               wn0g=0x3F, alm={"omega0": 1234})

    src = f"raw ints inverse-scaled from RINEX {e['sv']} toe={e['toe']:.0f} " \
          f"week={e['week']} (brdc_latest.rnx, see rinex_gal_eval.json)"
    cases = [
        _page_case("word1_clean", w1, "ephemeris 1/4; " + src,
                   osnma_seed=0x05A1),
        _page_case("word2_noisy", w2, "ephemeris 2/4, gauss sigma 0.35; " + src,
                   sigma=0.35, noise_seed=0xA2A2, osnma_seed=0x05A2),
        _page_case("word3_inverted_polarity", w3,
                   "ephemeris 3/4, 180-deg Costas polarity; " + src,
                   polarity=-1, osnma_seed=0x05A3),
        _page_case("word4_noisy", w4, "ephemeris 4/4 + clock, gauss sigma "
                   "0.35; " + src, sigma=0.35, noise_seed=0xA4A4,
                   osnma_seed=0x05A4),
        _page_case("word5_clean", w5, "iono/BGD/health/GST; BGD from " + src,
                   osnma_seed=0x05A5),
        _page_case("word6_clean", w6, "GST-UTC; synthetic values",
                   osnma_seed=0x05A6),
        _page_case("word0_time_valid", w0, "spare word, Time=2 -> WN/TOW "
                   "valid", osnma_seed=0x05A0),
        _page_case("word0_time_invalid", w0_invalid, "Time=1 -> WN/TOW must "
                   "be refused (fields.time_valid false)",
                   osnma_seed=0x15A0),
        _page_case("word10_ggto", w10, "almanac tail + GGTO",
                   osnma_seed=0x05AA),
        _page_case("word10_ggto_sentinel", w10_sentinel, "GGTO all-ones "
                   "sentinel -> ggto_valid false (fail-closed)",
                   osnma_seed=0x15AA),
        _page_case("alert_page_pt1", w1, "Page Type 1: CRC-clean but MUST be "
                   "refused as a word (decode_alert true, word None)",
                   page_type=1, osnma_seed=0x25A1),
        _page_case("negative_g2_not_inverted", w1, "encoded WITHOUT the G2 "
                   "inversion: the invert_g2=true decoder must fail CRC "
                   "(guards the classic gotcha)", invert_g2=False,
                   osnma_seed=0x35A1),
    ]
    # wrong deinterleave orientation negative: symbols interleaved with the
    # TRANSPOSED (8-written x 30-read) orientation must fail CRC.
    even, odd = build_page_parts(w1, page_type=0)
    wrong = list(SYNC) + deinterleave(conv_encode(even, True, 0)) \
        + list(SYNC) + deinterleave(conv_encode(odd, True, 0))
    soft = [1.0 - 2.0 * s for s in wrong]
    res = decode_page_at(soft, 0, 1)
    cases.append({
        "name": "negative_wrong_interleave_orientation",
        "note": "encoder applied the INVERSE permutation (write row-wise / "
        "8x30) — the standard deinterleaver must fail CRC",
        "invert_g2": True, "page_type": 0, "polarity": 1,
        "word_hex": bits_to_hex(w1), "symbols_hex": bits_to_hex(wrong),
        "nsym": len(wrong), "parsed": None,
        "decode_crc_ok": res["crc_ok"], "decode_alert": res["alert"],
        "decode_word_matches": False,
    })
    return {
        "_provenance": _PROV + " Synthetic full pages: forward chain "
        "word -> 196-bit CRC span -> +CRC -> tails -> conv(invert_g2=true) "
        "-> 30x8 interleave -> sync prepend (ICD Tables 24/25/36); decoded "
        "back through the inverse chain and field-compared bit-exact before "
        "pinning (decode_* flags record the proven outcome). OSNMA/SAR/SSP "
        "carry nonzero Rng bits — decoders must NOT reject them. "
        "Word raw ints derived from the live RINEX record named per case.",
        "sync": SYNC,
        "cases": cases,
    }


def pick_rinex_records(text, svs=("E02", "E11", "E25")):
    """Deterministic pick: for each SV the first healthy I/NAV record."""
    lines = text.splitlines()
    h = next(i for i, l in enumerate(lines) if "END OF HEADER" in l)
    picks = {}
    i = h + 1
    while i < len(lines):
        l = lines[i]
        if len(l) >= 3 and l[0] == "E" and l[1:3].strip().isdigit():
            block = lines[i:i + 8]
            eph = parse_rinex_gal_record(block)
            sv = eph["sv"]
            if sv in svs and sv not in picks and eph["health"] == 0 \
                    and rinex_gal_record_is_inav(eph):
                picks[sv] = (block, eph)
            i += 8
        else:
            i += 1
    return [picks[sv] for sv in svs if sv in picks]


def gen_rinex_eval(brdc_path=BRDC_PATH):
    text = open(brdc_path, errors="replace").read()
    picked = pick_rinex_records(text)
    rx = [4278600.0, 636800.0, 4672300.0]  # fixed test receiver ECEF (m)
    records = []
    for block, eph in picked:
        evals = []
        for dt_off in (0.0, 900.0, 1800.0):
            t = eph["toe"] + dt_off
            pos = sat_pos_ecef_gal(eph, t)
            clk_pair = sat_clock_gal({**eph, "bgd_e1e5b": 0.0}, t)
            clk_e1 = sat_clock_gal(eph, t)
            s, dts, rng = sat_at_txtime_gal(eph, t, rx)
            evals.append({
                "t_sow": t,
                "pos_ecef_m": pos,
                "clock_e1e5b_s": clk_pair,
                "clock_e1_s": clk_e1,       # = clock_e1e5b - BGD(E1,E5b)
                "txtime_sat_ecef_m": s,
                "txtime_clock_s": dts,
                "txtime_range_m": rng,
            })
        records.append({
            "rinex_lines": block,
            "parsed": eph,
            "rx_ecef_m": rx,
            "evaluations": evals,
        })
    return {
        "_provenance": _PROV + " RINEX source: /Volumes/Radiator 8TB/gnss/"
        "observations/brdc_latest.rnx as read 2026-09-02 (file refreshes "
        "hourly — the verbatim record lines are embedded so the fixture is "
        "self-contained). Selection: first healthy I/NAV record (Data "
        "Sources bit9 + bit0/bit2; F/NAV 258-records rejected) per SV of "
        "E02/E11/E25. Evaluation: Galileo Kepler pipeline (mu=3.986004418e14"
        ", omega_e=7.2921151467e-5, F=-4.442807309e-10), clock_e1 subtracts "
        "BGD(E1,E5b) per ICD Eq. 17. These pin the Rust parse_rinex_gal + "
        "sat_at_txtime_gal cross-check.",
        "constants": {"mu": MU_GAL, "omega_e": OMEGA_E, "f_rel": F_REL_GAL,
                      "c": C_LIGHT},
        "records": records,
    }


def gen_ggto_bgd_vectors(rinex_eval):
    # GGTO application vectors (raw broadcast ints -> seconds), incl. the
    # week rollover law and the invalid sentinel.
    ggto_cases = []
    for name, a0g, a1g, t0g_h, wn0g, tow, wn in [
        ("nominal", -232, 5, 68, 10, 245425, 10),
        ("negative_a0g_next_week", -232, 5, 68, 63, 3625, 0),  # dW=+1 via mod64
        ("rollover_dw_minus", 100, -3, 0, 1, 1000, 63),  # (63-1)%64=62 -> -2
    ]:
        a0 = a0g * 2.0**-35
        a1 = a1g * 2.0**-51
        dt = ggto_offset(a0, a1, t0g_h * 3600.0, wn0g, tow, wn)
        ggto_cases.append({
            "name": name, "raw": {"a0g": a0g, "a1g": a1g, "t0g": t0g_h,
                                  "wn0g": wn0g}, "tow": tow, "wn": wn,
            "dt_systems_s": dt,
            "t_tx_gst": float(tow), "t_tx_gpst": float(tow) - dt,
        })
    ggto_cases.append({
        "name": "sentinel_all_ones_invalid",
        "raw": {"a0g": 0xFFFF, "a1g": 0xFFF, "t0g": 0xFF, "wn0g": 0x3F},
        "tow": 245425, "wn": 10, "dt_systems_s": None,
        "t_tx_gst": 245425.0, "t_tx_gpst": 245425.0,
        "note": "ICD 5.1.8 invalid sentinel: apply zero (fail-closed)",
    })
    # BGD vector straight off the first pinned RINEX record
    r0 = rinex_eval["records"][0]
    ev0 = r0["evaluations"][0]
    bgd_case = {
        "sv": r0["parsed"]["sv"],
        "bgd_e1e5b_s": r0["parsed"]["bgd_e1e5b"],
        "t_sow": ev0["t_sow"],
        "clock_e1e5b_s": ev0["clock_e1e5b_s"],
        "clock_e1_s": ev0["clock_e1_s"],
        "identity": "clock_e1 = clock_e1e5b - bgd_e1e5b (ICD Eq. 17, f1=E1; "
        "the exact analogue of GPS -tgd)",
    }
    return {
        "_provenance": _PROV + " GGTO per ICD 5.1.8 Eq. 23 (A0G 2^-35 s, "
        "A1G 2^-51 s/s, t0G x3600 s, WN0G mod 64, |dW|<=31 rollover; "
        "all-ones sentinel -> apply zero, spec 5.4 Option B). BGD case "
        "cross-references rinex_gal_eval.json record 0.",
        "ggto_cases": ggto_cases,
        "bgd_case": bgd_case,
    }


def gen_all_fixtures(out_dir=FIXTURE_DIR, brdc_path=BRDC_PATH):
    os.makedirs(out_dir, exist_ok=True)
    rinex_eval = gen_rinex_eval(brdc_path)
    fixtures = {
        "crc24q_vectors.json": gen_crc_vectors(),
        "conv_viterbi_vectors.json": gen_conv_vectors(),
        "interleaver_vectors.json": gen_interleaver_vectors(),
        "rinex_gal_eval.json": rinex_eval,
        "pages_synthetic.json": gen_page_vectors(
            rinex_eval["records"][0]["parsed"]),
        "ggto_bgd_vectors.json": gen_ggto_bgd_vectors(rinex_eval),
    }
    for name, data in fixtures.items():
        path = os.path.join(out_dir, name)
        with open(path, "w") as f:
            json.dump(data, f, indent=1)
            f.write("\n")
    return sorted(fixtures)


if __name__ == "__main__":
    import sys
    if "--gen-fixtures" in sys.argv:
        names = gen_all_fixtures()
        print("wrote", len(names), "fixtures to", FIXTURE_DIR)
        for n in names:
            print(" ", n)
    else:
        print(__doc__)
