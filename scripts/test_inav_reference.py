#!/usr/bin/env python3
"""Tests for scripts/inav_reference.py (Galileo E1B I/NAV reference).

Runs NOW (hardware-free, stdlib+pytest). These tests PROVE the python
implementation before its outputs are pinned into tests/fixtures/inav/;
the fixture-pinning tests at the bottom then assert that the JSON on disk
is exactly what the proven implementation produces (regenerated from the
fixtures' own embedded RINEX lines, so the hourly brdc refresh cannot
invalidate them).
"""

import json
import math
import os

import pytest

import inav_reference as ir

FIX = ir.FIXTURE_DIR


def load(name):
    with open(os.path.join(FIX, name)) as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# CRC-24Q
# ---------------------------------------------------------------------------

def ascii_bits(b):
    out = []
    for byte in b:
        out += [(byte >> i) & 1 for i in range(7, -1, -1)]
    return out


def test_crc24q_known_vectors():
    # Qualcomm standard check value, same as sbas.rs crc24q_known_vectors
    assert ir.crc24q(ascii_bits(b"123456789")) == 0xCDE703
    assert ir.crc24q([0] * 250) == 0
    assert ir.crc24q([1] * 250) == 0xEF339D


def test_crc24q_zero_syndrome_form():
    # appended-CRC linear-code property: crc(span ++ crc_bits) == 0
    rng = ir.Rng(0xBEEF)
    span = [rng.bit() for _ in range(ir.CRC_SPAN_BITS)]
    crc = ir.crc24q(span)
    crc_bits = [(crc >> (23 - i)) & 1 for i in range(24)]
    assert ir.crc24q(span + crc_bits) == 0
    # single-bit damage anywhere breaks it
    span[57] ^= 1
    assert ir.crc24q(span + crc_bits) != 0


# ---------------------------------------------------------------------------
# convolutional code + Viterbi
# ---------------------------------------------------------------------------

def test_conv_encode_rate_and_conventions():
    bits = [1, 0, 1, 1, 0, 0, 1]
    st, sf = ir.conv_encode(bits, True, 0), ir.conv_encode(bits, False, 0)
    assert len(st) == 2 * len(bits) == len(sf)
    # G2 inversion flips exactly the odd (second-branch) symbols
    assert all((a ^ b) == (i % 2) for i, (a, b) in enumerate(zip(st, sf)))


@pytest.mark.parametrize("inv", [True, False])
def test_viterbi_noiseless_roundtrip(inv):
    rng = ir.Rng(0x1234)
    bits = [rng.bit() for _ in range(ir.PART_BITS)]
    sym = ir.conv_encode(bits, inv, 0)
    assert ir.viterbi_hard(sym, inv) == bits


def test_viterbi_wrong_g2_convention_fails():
    rng = ir.Rng(0x77)
    bits = [rng.bit() for _ in range(ir.PART_BITS)]
    sym = ir.conv_encode(bits, False, 0)
    assert ir.viterbi_hard(sym, True) != bits


@pytest.mark.parametrize("seed", [1, 2, 3, 4, 5])
def test_viterbi_corrects_gaussian_noise(seed):
    rng = ir.Rng(seed)
    bits = [rng.bit() for _ in range(ir.PART_BITS)]
    sym = ir.conv_encode(bits, True, 0)
    nrng = ir.Rng(0x9000 + seed)
    for _ in range(16):
        nrng.next()  # xorshift64 warm-up (see inav_reference._noisy)
    soft = [(1.0 - 2.0 * s) + 0.45 * nrng.gauss() for s in sym]
    assert ir.viterbi(soft, True) == bits


def test_viterbi_corrects_hard_flips():
    rng = ir.Rng(0x5150)
    bits = [rng.bit() for _ in range(ir.PART_BITS)]
    sym = ir.conv_encode(bits, True, 0)
    for i in range(12):
        sym[3 + 20 * i] ^= 1  # spread flips, > free distance apart
    assert ir.viterbi_hard(sym, True) == bits


# ---------------------------------------------------------------------------
# interleaver
# ---------------------------------------------------------------------------

def test_interleaver_law_and_inverse():
    rng = ir.Rng(0xABCD)
    x = [rng.bit() for _ in range(ir.PART_SYMS)]
    y = ir.interleave(x)
    # transmitted[30*row+col] == encoded[8*col+row]  (ICD Table 25)
    for row in range(8):
        for col in range(30):
            assert y[30 * row + col] == x[8 * col + row]
    assert ir.deinterleave(y) == x
    assert ir.interleave(ir.deinterleave(x)) == x
    # the permutation is not the identity and not its own inverse
    ident = list(range(ir.PART_SYMS))
    assert ir.interleave(ident) != ident
    assert ir.interleave(ir.interleave(ident)) != ident


# ---------------------------------------------------------------------------
# page forward/inverse chain
# ---------------------------------------------------------------------------

def word_menu():
    return [
        ir.make_word1(23, 4080, -123456789, 2661000, 2852000000),
        ir.make_word2(23, -1414812757, 655360001, -2, 4321),
        ir.make_word3(23, -15265, 25808, -21, 105, 2445, 4631, 107),
        ir.make_word4(23, 2, -160, -240, 4080, 1134000, 189, -1),
        ir.make_word5(45, -12, 7, -13, -17, 0, 0, 0, 0, 1410, 244825),
        ir.make_word6(-17, 3, 18, 68, 130, 130, 3, 18, 244805),
        ir.make_word0(2, 1410, 244855),
        ir.make_word10(7, -232, 5, 68, 2, alm={"omega0": 1234}),
    ]


@pytest.mark.parametrize("wi", range(8))
def test_page_roundtrip_clean(wi):
    word = word_menu()[wi]
    rng = ir.Rng(0xE1B0 + wi)
    sym = ir.encode_page(word, osnma=[rng.bit() for _ in range(40)],
                         sar=[rng.bit() for _ in range(22)],
                         spare=[rng.bit() for _ in range(2)],
                         ssp=[rng.bit() for _ in range(8)])
    assert len(sym) == ir.PAGE_SYMS
    soft = [1.0 - 2.0 * s for s in sym]
    i, pol = ir.find_sync(soft)
    assert (i, pol) == (0, 1)
    res = ir.decode_page_at(soft, 0, pol)
    assert res["crc_ok"] and res["eo_ok"] and not res["alert"]
    assert res["word"] == word  # bit-exact recovery


@pytest.mark.parametrize("wi", range(8))
def test_page_roundtrip_noisy_and_inverted(wi):
    word = word_menu()[wi]
    sym = ir.encode_page(word)
    nrng = ir.Rng(0xF00 + wi)
    for _ in range(16):
        nrng.next()  # xorshift64 warm-up (see inav_reference._noisy)
    soft = [-((1.0 - 2.0 * s) + 0.35 * nrng.gauss()) for s in sym]  # inverted
    hit = ir.find_sync(soft)
    assert hit is not None
    i, pol = hit
    assert pol == -1
    res = ir.decode_page_at(soft, i, pol)
    assert res["crc_ok"] and res["word"] == word


def test_find_pages_stream_with_garbage():
    words = word_menu()[:3]
    rng = ir.Rng(0xDEAD)
    stream = []
    for w in words:
        stream += ir.encode_page(w, osnma=[rng.bit() for _ in range(40)])
    soft = [1.0 - 2.0 * s for s in stream]
    lead = [0.9 * (1.0 - 2.0 * rng.bit()) for _ in range(37)]
    pages = ir.find_pages(lead + soft)
    assert len(pages) == 3
    for p, w in zip(pages, words):
        assert p["word"] == w
    # sym_index points at the even part's first sync symbol
    assert pages[0]["sym_index"] == len(lead)
    assert pages[1]["sym_index"] == len(lead) + ir.PAGE_SYMS


def test_alert_page_refused_as_word():
    word = word_menu()[0]
    sym = ir.encode_page(word, page_type=1)
    soft = [1.0 - 2.0 * s for s in sym]
    res = ir.decode_page_at(soft, 0, 1)
    assert res["crc_ok"]          # CRC-clean...
    assert res["alert"]           # ...but flagged alert
    assert res["word"] is None    # and refused as a word (fail-closed)


def test_g2_not_inverted_fails_crc():
    word = word_menu()[0]
    sym = ir.encode_page(word, invert_g2=False)
    soft = [1.0 - 2.0 * s for s in sym]
    res = ir.decode_page_at(soft, 0, 1)
    assert not res["crc_ok"]


def test_wrong_deinterleave_orientation_fails_crc():
    word = word_menu()[0]
    even, odd = ir.build_page_parts(word)
    wrong = (list(ir.SYNC) + ir.deinterleave(ir.conv_encode(even, True, 0))
             + list(ir.SYNC) + ir.deinterleave(ir.conv_encode(odd, True, 0)))
    soft = [1.0 - 2.0 * s for s in wrong]
    res = ir.decode_page_at(soft, 0, 1)
    assert not res["crc_ok"]


def test_degenerate_zero_span_rejected():
    # all-zero span + all-zero CRC passes arithmetically; weight guard fires
    even = [0] * ir.PART_BITS
    odd = [0] * ir.PART_BITS
    res = ir.check_page(even, odd)
    assert not res["weight_ok"]
    assert not res["crc_ok"]


def test_nonzero_osnma_sar_ssp_not_rejected():
    word = word_menu()[4]
    sym = ir.encode_page(word, osnma=[1] * 40, sar=[1] * 22, spare=[1, 0],
                         ssp=[1, 0, 1, 0, 1, 0, 1, 0])
    soft = [1.0 - 2.0 * s for s in sym]
    res = ir.decode_page_at(soft, 0, 1)
    assert res["crc_ok"] and res["word"] == word


# ---------------------------------------------------------------------------
# word parsers: sign extension, scaling, refusal laws
# ---------------------------------------------------------------------------

def test_sign_extension_extremes():
    # every signed field at its most-negative / most-positive value
    w = ir.make_word1(0, 0, -(1 << 31), (1 << 32) - 1, (1 << 32) - 1)
    p = ir.parse_word(w)
    assert p["raw"]["m0"] == -(1 << 31)
    assert p["raw"]["e"] == (1 << 32) - 1          # unsigned stays positive
    w = ir.make_word2(1023, (1 << 31) - 1, -1, -(1 << 31), -(1 << 13))
    p = ir.parse_word(w)
    assert p["raw"]["omega0"] == (1 << 31) - 1
    assert p["raw"]["i0"] == -1
    assert p["raw"]["omega"] == -(1 << 31)
    assert p["raw"]["idot"] == -(1 << 13)
    w = ir.make_word4(1, 36, -1, 1, 16383, -(1 << 30), (1 << 20) - 1, -32)
    p = ir.parse_word(w)
    assert p["raw"]["af0"] == -(1 << 30)
    assert p["raw"]["af1"] == (1 << 20) - 1
    assert p["raw"]["af2"] == -32
    assert p["fields"]["toc"] == 16383 * 60.0
    w = ir.make_word5(2047, -1024, 8191, -512, 511, 3, 3, 1, 1, 4095, 604799)
    p = ir.parse_word(w)
    assert p["raw"]["ai1"] == -1024
    assert p["raw"]["bgd_e1e5a"] == -512
    assert p["raw"]["bgd_e1e5b"] == 511
    assert p["raw"]["tow"] == 604799
    assert p["fields"]["bgd_e1e5b"] == 511 * 2.0**-32


def test_scaling_conventions():
    p = ir.parse_word(ir.make_word1(10, 4080, 1 << 30, 1 << 31, 1 << 30))
    assert p["fields"]["toe"] == 244800.0
    assert p["fields"]["m0"] == pytest.approx((1 << 30) * 2**-31 * math.pi)
    assert p["fields"]["e"] == (1 << 31) * 2.0**-33
    assert p["fields"]["sqrt_a"] == (1 << 30) * 2.0**-19
    p = ir.parse_word(ir.make_word6(1 << 20, 1 << 10, -18, 100, 5, 6, 7, 18,
                                    12345))
    assert p["fields"]["a0"] == (1 << 20) * 2.0**-30
    assert p["fields"]["a1"] == (1 << 10) * 2.0**-50
    assert p["fields"]["t0t"] == 360000.0
    assert p["fields"]["dt_ls"] == -18


def test_word0_time_gate():
    ok = ir.parse_word(ir.make_word0(2, 1410, 100))
    assert ok["fields"]["time_valid"] and ok["fields"]["tow"] == 100
    for bad_flag in (0, 1, 3):
        bad = ir.parse_word(ir.make_word0(bad_flag, 1410, 100))
        assert not bad["fields"]["time_valid"]
        assert "tow" not in bad["fields"]  # fail-closed


def test_word10_ggto_sentinel():
    ok = ir.parse_word(ir.make_word10(1, -232, 5, 68, 2))
    assert ok["fields"]["ggto_valid"]
    assert ok["fields"]["a0g"] == -232 * 2.0**-35
    bad = ir.parse_word(ir.make_word10(1, 0xFFFF, 0xFFF, 0xFF, 0x3F))
    assert not bad["fields"]["ggto_valid"]
    assert "a0g" not in bad["fields"]
    # all-ones in only SOME fields is still valid (sentinel is all four)
    part = ir.parse_word(ir.make_word10(1, 0xFFFF, 0xFFF, 0xFF, 0x3E))
    assert part["fields"]["ggto_valid"]


def test_unknown_word_type_ignored():
    w = ir._new_word(63)
    assert ir.parse_word(w) is None


# ---------------------------------------------------------------------------
# ephemeris assembly + evaluation
# ---------------------------------------------------------------------------

def parsed_menu():
    return [ir.parse_word(w) for w in word_menu()]


def test_assemble_ephemeris_iodnav_gate():
    ws = parsed_menu()
    eph = ir.assemble_ephemeris(ws[0], ws[1], ws[2], ws[3], ws[4])
    assert eph is not None
    assert eph["iodnav"] == 23
    assert eph["week"] == 1410 + 1024      # GST WN -> continuous GPS week
    assert eph["health"] == 0
    # IODnav mismatch across the batch -> refused
    w2_bad = ir.parse_word(ir.make_word2(24, 1, 2, 3, 4))
    assert ir.assemble_ephemeris(ws[0], w2_bad, ws[2], ws[3]) is None


def test_assemble_health_law():
    ws = parsed_menu()
    w5_sick = ir.parse_word(
        ir.make_word5(45, -12, 7, -13, -17, 0, 2, 0, 1, 1410, 244825))
    eph = ir.assemble_ephemeris(ws[0], ws[1], ws[2], ws[3], w5_sick)
    assert eph["health"] == (2 << 1) | 1   # HS<<1 | DVS composite


def test_kepler_constants_differ_from_gps():
    assert ir.MU_GAL == 3.986004418e14
    assert ir.MU_GAL != 3.986005e14              # GPS MU_E
    assert ir.OMEGA_E == 7.2921151467e-5         # NOT the BDS 7.2921150e-5
    assert ir.F_REL_GAL == -4.442807309e-10
    # F is -2 sqrt(mu)/c^2 with the Galileo mu, to ICD print precision
    calc = -2.0 * math.sqrt(ir.MU_GAL) / ir.C_LIGHT**2
    assert abs(calc - ir.F_REL_GAL) < 1e-19


def test_wrap_tk():
    assert ir.wrap_tk(302401.0) == 302401.0 - 604800.0
    assert ir.wrap_tk(-302401.0) == -302401.0 + 604800.0
    assert ir.wrap_tk(1000.0) == 1000.0


def test_orbit_evaluation_sanity_real_record():
    fx = load("rinex_gal_eval.json")
    for rec in fx["records"]:
        eph = rec["parsed"]
        for ev in rec["evaluations"]:
            pos = ev["pos_ecef_m"]
            r = math.sqrt(sum(c * c for c in pos))
            # Galileo MEO semi-major axis ~29600 km, e small
            assert 2.90e7 < r < 3.02e7
            # GAL af0 range is a few ms (E24 broadcasts ~5.7 ms today)
            assert abs(ev["clock_e1e5b_s"]) < 1e-2
            # BGD identity: clock_e1 = clock_e1e5b - BGD(E1,E5b)
            assert ev["clock_e1_s"] == pytest.approx(
                ev["clock_e1e5b_s"] - eph["bgd_e1e5b"], abs=1e-18)
            # geometric range from a fixed Earth-surface point to a MEO
            # sphere point (visibility NOT enforced): r-Re .. r+Re
            assert 1.9e7 < ev["txtime_range_m"] < 3.7e7


def test_sat_at_txtime_matches_direct_evaluation():
    fx = load("rinex_gal_eval.json")
    rec = fx["records"][0]
    eph = rec["parsed"]
    rx = rec["rx_ecef_m"]
    t = eph["toe"] + 300.0
    s, dts, rng = ir.sat_at_txtime_gal(eph, t, rx)
    tau = rng / ir.C_LIGHT
    raw = ir.sat_pos_ecef_gal(eph, t - tau)
    th = ir.OMEGA_E * tau
    rot = [raw[0] * math.cos(th) + raw[1] * math.sin(th),
           -raw[0] * math.sin(th) + raw[1] * math.cos(th), raw[2]]
    for a, b in zip(s, rot):
        assert a == pytest.approx(b, abs=1e-3)
    assert dts == pytest.approx(ir.sat_clock_gal(eph, t - tau), abs=1e-15)


# ---------------------------------------------------------------------------
# GGTO
# ---------------------------------------------------------------------------

def test_ggto_hand_computation():
    a0g, a1g, t0g, wn0g = -232 * 2.0**-35, 5 * 2.0**-51, 68 * 3600.0, 10
    dt = ir.ggto_offset(a0g, a1g, t0g, wn0g, tow=245425, wn=10)
    assert dt == pytest.approx(a0g + a1g * (245425 - 244800), abs=1e-24)


def test_ggto_week_rollover():
    # wn0g=63, wn=0 -> dW = +1
    dt = ir.ggto_offset(0.0, 1.0, 0.0, 63, tow=0, wn=0)
    assert dt == ir.WEEK_S
    # wn0g=1, wn=63 -> (62 mod 64) > 31 -> -2
    dt = ir.ggto_offset(0.0, 1.0, 0.0, 1, tow=0, wn=63)
    assert dt == -2 * ir.WEEK_S


def test_apply_ggto_fail_closed():
    valid = ir.parse_word(ir.make_word10(1, -232, 5, 68, 10))["fields"]
    t = ir.apply_ggto(245425.0, valid, tow=245425, wn=10)
    assert t != 245425.0
    invalid = ir.parse_word(ir.make_word10(1, 0xFFFF, 0xFFF, 0xFF,
                                           0x3F))["fields"]
    assert ir.apply_ggto(245425.0, invalid, 245425, 10) == 245425.0
    assert ir.apply_ggto(245425.0, None, 245425, 10) == 245425.0


# ---------------------------------------------------------------------------
# RINEX GAL parsing
# ---------------------------------------------------------------------------

def test_rinex_week_sow_arithmetic():
    # 2026-09-01 is a Tuesday; GPS week 2434 began Sunday 2026-08-30
    week, sow = ir.gps_week_sow(2026, 9, 1, 20, 0, 0)
    assert week == 2434
    assert sow == 2 * 86400 + 20 * 3600
    week, sow = ir.gps_week_sow(1980, 1, 6, 0, 0, 0)
    assert (week, sow) == (0, 0)


def test_rinex_parse_embedded_records():
    fx = load("rinex_gal_eval.json")
    assert len(fx["records"]) >= 2
    for rec in fx["records"]:
        eph = ir.parse_rinex_gal_record(rec["rinex_lines"])
        assert eph == rec["parsed"]
        assert ir.rinex_gal_record_is_inav(eph)
        assert eph["health"] == 0
        assert eph["week"] == eph["toc_week"]  # week field matches calendar
        assert eph["toe"] % 60.0 == 0.0


def test_rinex_data_sources_gate():
    fake = {"data_sources": 258}
    assert not ir.rinex_gal_record_is_inav(fake)   # F/NAV: bit1|bit8
    for ds in (513, 516, 517):                     # live I/NAV values
        assert ir.rinex_gal_record_is_inav({"data_sources": ds})
    assert not ir.rinex_gal_record_is_inav({"data_sources": 1})  # no bit9


def test_parse_rinex_gal_full_text():
    fx = load("rinex_gal_eval.json")
    body = "x END OF HEADER\n" + "\n".join(
        "\n".join(rec["rinex_lines"]) for rec in fx["records"])
    ephs = ir.parse_rinex_gal(body)
    assert len(ephs) == len(fx["records"])
    for eph, rec in zip(ephs, fx["records"]):
        assert eph == rec["parsed"]


# ---------------------------------------------------------------------------
# fixture pinning: JSON on disk == output of the proven implementation
# ---------------------------------------------------------------------------

def test_fixture_crc_pinned():
    assert load("crc24q_vectors.json") == ir.gen_crc_vectors()
    fx = load("crc24q_vectors.json")
    for v in fx["vectors"]:
        bits = ir.hex_to_bits(v["bits_hex"], v["nbits"])
        assert ir.crc24q(bits) == v["crc"]
        if "crc_expect" in v:
            assert v["crc"] == v["crc_expect"]
        if "zero_syndrome_over_220" in v:
            assert v["zero_syndrome_over_220"] == 0


def test_fixture_conv_pinned():
    assert load("conv_viterbi_vectors.json") == ir.gen_conv_vectors()
    for c in load("conv_viterbi_vectors.json")["cases"]:
        if "decoded_matches" in c:
            assert c["decoded_matches"], c["name"]
        if c["name"] == "g2_convention_differs":
            assert not c["identical"]


def test_fixture_interleaver_pinned():
    fx = load("interleaver_vectors.json")
    assert fx == ir.gen_interleaver_vectors()
    assert fx["roundtrip_ok"]
    perm = fx["permutation"]
    assert sorted(perm) == list(range(240))
    assert perm == fx["identity_interleaved"]


def test_fixture_rinex_pinned():
    """Re-evaluate from the fixture's own embedded record lines (the live
    brdc_latest.rnx refreshes hourly; the fixture is self-contained)."""
    fx = load("rinex_gal_eval.json")
    rx = None
    for rec in fx["records"]:
        eph = ir.parse_rinex_gal_record(rec["rinex_lines"])
        rx = rec["rx_ecef_m"]
        for ev in rec["evaluations"]:
            t = ev["t_sow"]
            assert ir.sat_pos_ecef_gal(eph, t) == ev["pos_ecef_m"]
            assert ir.sat_clock_gal({**eph, "bgd_e1e5b": 0.0}, t) \
                == ev["clock_e1e5b_s"]
            assert ir.sat_clock_gal(eph, t) == ev["clock_e1_s"]
            s, dts, rng = ir.sat_at_txtime_gal(eph, t, rx)
            assert s == ev["txtime_sat_ecef_m"]
            assert dts == ev["txtime_clock_s"]
            assert rng == ev["txtime_range_m"]


def test_fixture_pages_pinned():
    fx = load("pages_synthetic.json")
    rinex = load("rinex_gal_eval.json")
    assert fx == ir.gen_page_vectors(rinex["records"][0]["parsed"])
    n_pos = n_neg = 0
    for c in fx["cases"]:
        sym = ir.hex_to_bits(c["symbols_hex"], c["nsym"])
        if "soft" in c:
            soft = c["soft"]
        else:
            soft = [c["polarity"] * (1.0 - 2.0 * s) for s in sym]
        hit = ir.find_sync(soft)
        assert hit is not None and hit[0] == 0
        res = ir.decode_page_at(soft, 0, hit[1])
        if c["name"].startswith("negative"):
            assert not res["crc_ok"], c["name"]
            n_neg += 1
            continue
        assert res["crc_ok"], c["name"]
        if c["page_type"] == 1:
            assert res["alert"] and res["word"] is None
            n_neg += 1
            continue
        word = ir.hex_to_bits(c["word_hex"], ir.WORD_BITS)
        assert res["word"] == word, c["name"]     # bit-exact recovery
        parsed = ir.parse_word(word)
        assert parsed == c["parsed"], c["name"]   # field-exact recovery
        n_pos += 1
    assert n_pos >= 8 and n_neg >= 3


def test_fixture_pages_cover_words_1_to_5():
    fx = load("pages_synthetic.json")
    types = set()
    for c in fx["cases"]:
        if c.get("parsed") and not c["name"].startswith(("negative", "alert")):
            types.add(c["parsed"]["word_type"])
    assert {1, 2, 3, 4, 5} <= types


def test_fixture_ggto_bgd_pinned():
    fx = load("ggto_bgd_vectors.json")
    rinex = load("rinex_gal_eval.json")
    assert fx == ir.gen_ggto_bgd_vectors(rinex)
    for c in fx["ggto_cases"]:
        if c["dt_systems_s"] is None:
            assert c["t_tx_gpst"] == c["t_tx_gst"]  # sentinel: apply zero
            continue
        r = c["raw"]
        dt = ir.ggto_offset(r["a0g"] * 2.0**-35, r["a1g"] * 2.0**-51,
                            r["t0g"] * 3600.0, r["wn0g"], c["tow"], c["wn"])
        assert dt == c["dt_systems_s"]
        assert c["t_tx_gpst"] == c["t_tx_gst"] - dt
    b = fx["bgd_case"]
    assert b["clock_e1_s"] == pytest.approx(
        b["clock_e1e5b_s"] - b["bgd_e1e5b_s"], abs=1e-18)


def test_all_fixture_files_have_provenance():
    for name in os.listdir(FIX):
        if name.endswith(".json"):
            assert "_provenance" in load(name), name


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
