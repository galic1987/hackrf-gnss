> **Line-reference pin (2026-09-02):** src/live.rs references in this spec
> were surveyed at commit 0f5c669, BEFORE the SBAS-ranging package edited
> live.rs in the same working tree — refs past ~line 210 have shifted by
> +20..170 lines (e.g. the catch-all anchor arm `_ => 0` is now ~:2409).
> Re-pin against HEAD at implementation time; symbol names are stable.

# Galileo E1B I/NAV ranging — implementation spec (2026-09-02)

Availability lever 1 of docs/superpowers/plans/2026-09-01-availability-design.md
(+1.64 sats mean; P(n_sat>=5) 0.371 -> 0.598 alone). The tracker already locks
E1B (mean 1.64 sats/epoch, up to 30 PRNs/day) but rho_m is null on 100% of
298,027 tracked GAL rows because (a) live.rs establishes the pseudorange nav
anchor only in the GPS-LNAV and BDS-D1 match arms (the `_ => 0` at
src/live.rs:2241) and (b) the clock_bias emitter filters sys to {gps, beidou}
(examples/clock_bias.rs:435-439). This spec defines the I/NAV decoder, the
anchor, and the solve integration precisely enough to implement without
re-research. Every ICD constant below was verified on 2026-09-02 against the
Galileo OS SIS ICD Issue 2.1 (November 2023) PDF itself (table numbers cited;
the printed page = the cited page) and cross-checked against PocketSDR's
`sdr_nav.py` I/NAV decoder where noted. Status: SPEC ONLY — no code in this
change; Rust builds are deferred to the maintenance window per station law.

## 0. Acceptance criteria (from the availability design)

- GAL rho_m non-null on live rows while an E1B channel is locked with a fresh
  page anchor (same freshness law as GPS/BDS: ANCHOR_MAX_AGE_S).
- Residual RMS not degraded when GAL rows join the solve. The GPS-Galileo time
  offset + receiver inter-signal bias folds into the existing unmodeled ISB —
  the analyzer must be able to quantify it (accepted per-constellation counts
  in every row), exactly as the BDS precedent.
- A/B membership gate exercised by the new rows (no change to its law here;
  lever 6 fixes its observability separately).
- All emitter/analyzer changes land together, consciously (Section 6.3 — the
  change is NOT purely additive; see the n_gps+n_bds==n_sat trap).

## 1. Reuse inventory — what exists vs. new work

| I/NAV need | Existing code | Status |
|---|---|---|
| E1B primary codes (4092-chip memory codes, all 50 PRNs, dual-mirror verified) | `src/galileo.rs` `e1b_code()` (`E1B_HEX`, cross-checked GNSS-SDR + PocketSDR 2026-08-23) | REUSE as-is |
| E1B acquisition (4 ms coherent, BOC(1,1) replica, code-Doppler roll) | `src/galileo.rs` `acquire_e1b()`; wired into live.rs seeding (src/live.rs:2407, 2455-2457) | REUSE as-is |
| E1B tracking channel (4 ms epoch, BOC(1,1) chip shaping, Costas+FLL, DLL spacing 0.125) | `Channel::new` Sys::Galileo arm (src/live.rs:457-464), `chip_at` BOC flip (:593) | REUSE as-is |
| K=7 rate-1/2 Viterbi with INVERTIBLE G2 branch | `src/sbas.rs` `viterbi(soft, invert_g2)` / `conv_encode(bits, invert_g2, state)` (:108-199) — `invert_g2: true` is exactly the Galileo convention | REUSE; call with `invert_g2 = true` |
| CRC-24Q (poly 0x1864CFB, appended-CRC zero-syndrome check) | `src/sbas.rs` `crc24q()` (:66-77) — same polynomial as I/NAV (ICD §5.1.9.4 Eq. 24-25, G(X)=(1+X)P(X)) | REUSE as-is |
| Per-epoch prompt collection for nav demod | `process_epoch` pushes `ip` only for Gps/Beidou/Sbas (src/live.rs:759-766) | EXTEND: include Sys::Galileo (one prompt per 4 ms epoch = one 250 sym/s symbol, exactly) |
| Bit sync | GPS transition histogram / BDS NH20 (`nav_tick_*`) | NOT NEEDED for E1B: symbol boundary == code-period boundary by construction (1 symbol = 1 code period = 1 tracking epoch). Page sync replaces bit sync. |
| Deinterleaver (30x8), page sync, page assembly, word parse | — | NEW: `src/galileo_inav.rs` (Section 3) |
| TOW -> stream-time anchor | `anchor_stream_time` (src/live.rs:113-206) | GENERALIZE: 20 ms/1 ms constants are hardcoded (Section 4.2) |
| Anchor arm in `end_second` | GPS/BDS arms (src/live.rs:2123-2242) | NEW Sys::Galileo arm mirroring GPS (Section 4.1) |
| Orbit/clock evaluation | GPS: `gps/broadcast.rs` (MU_E=3.986005e14, F_REL=-4.442807633e-10); BDS: `beidou_d1.rs` (MU_BDS=3.986004418e14) | NEW thin variant: Galileo mu = 3.986004418e14 (== MU_BDS, != GPS), omega_e = 7.2921151467e-5, F = -4.442807309e-10 (ICD Table 66 + Eq. 15). The Keplerian algorithm itself is identical to GPS (Table 66 == IS-GPS-200 form) — parameterize, don't fork. |
| RINEX ephemeris source | `parse_rinex_gps` / `parse_rinex_bds` | NEW `parse_rinex_gal` (Section 5.2) |
| Solve | `solve_clock_only` (gps/pvt.rs:200) is constellation-blind (works on Meas) | REUSE; emitter changes in Section 6 |
| Iono/tropo for GAL rows | SBAS IGP grid + tropo model in `build_meas` (examples/clock_bias.rs) | REUSE grid + tropo (E1 is the SAME 1575.42 MHz as L1 — no frequency scaling, unlike the BDS lever-7 case); NO SBAS PRC/LT/DNU (GPS-PRN products) |

## 2. Physical layer (verified: OS SIS ICD 2.1)

E1B data component: 1575.42 MHz carrier (same as GPS L1 -> carrier prediction
wavelength is the existing `LAM_L1`), CBOC(6,1,1/11) modulation tracked with a
BOC(1,1) replica (already the live channel's replica), 1.023 Mcps, 4092-chip
primary code, 4 ms code period, NO secondary code on E1B (the CS25 secondary
code is on the E1C pilot only — not tracked here).

- Symbol rate 250 sym/s; data rate 125 bit/s (rate-1/2 FEC). One tracking
  epoch (4 ms) integrates exactly one symbol.
- Nominal page = 2 s, two 1 s parts, "even" then "odd", sequential on the same
  frequency ("vertical page") — ICD §4.3.2, Table 35/36 (p. 37-38).
- Each 1 s page part = 10-symbol sync pattern + 240 coded symbols (250 total).
- Sync pattern: `0101100000`, NOT encoded/interleaved (ICD §4.3.2.1). The
  Costas 180-degree ambiguity means the correlator must try both polarities;
  latch polarity on the first CRC-passing page (mirror the LNAV
  both-polarities pattern), and treat a polarity flip like sbas_tick treats a
  pairing flip: only a CRC-locked decode on the opposite polarity proves a
  real break.
- FEC (ICD §4.1.4.1, Table 24, p. 28): rate 1/2, K=7, G1=171o, G2=133o,
  encoding sequence G1 then G2, and the SECOND BRANCH OUTPUT IS INVERTED
  (Figure 13 note: "an encoder where the second branch is inverted at the
  end") — the classic Galileo gotcha. `sbas::viterbi(soft, /*invert_g2=*/true)`
  models exactly this; do NOT also pre-invert symbols (PocketSDR does the
  equivalent by XOR-ing odd symbols then decoding non-inverted — pick ONE).
- Interleaver (ICD §4.1.4.2, Table 25, p. 29): block interleaver, 240 symbols,
  30 columns (written) x 8 rows (read). Receiver inverse: write the 240
  received soft symbols row-wise into an 8x30 matrix, read column-wise
  (PocketSDR: `syms.reshape(8, 30).T.ravel()`).
- Tail: 6 zero bits close each page part (ICD §4.3.2.2), so each 240-symbol
  part Viterbi-decodes INDEPENDENTLY to 120 bits (114 information + 6 tail) —
  the decoder needs no cross-part state, unlike the continuous SBAS stream.
  ICD Annex D carries FEC + interleaving numerical examples: use them as the
  golden encoder vectors.
- E1-B nominal page part layouts (ICD Table 36, p. 38 — E1-B differs from
  E5b-I; use the E1-B column):
  - Even part (120 bits): Even/Odd=0 (1) | Page Type (1) | Data (1/2) (112) | Tail (6)
  - Odd part (120 bits): Even/Odd=1 (1) | Page Type (1) | Data (2/2) (16) | OSNMA (40) | SAR (22) | Spare (2) | CRC (24) | SSP (8) | Tail (6)
  - Data (1/2) + Data (2/2) = the 128-bit I/NAV word.
  - CRC-24Q covers (ICD Table 36 note): even E/O + PT + Data(1/2) [114 bits]
    then odd E/O + PT + Data(2/2) + OSNMA + SAR + Spare [82 bits] = 196 bits.
    Equivalently: `sbas::crc24q(those 196 bits ++ the 24 CRC bits) == 0`
    (220-bit zero-syndrome form — what PocketSDR checks). SSP and Tail are NOT
    CRC-protected.
  - Page Type: 0 = nominal, 1 = alert. Alert pages (Table 37) carry no words —
    on Page Type 1, still CRC-check, then discard content (fail-closed: an
    alert page must not be parsed as a word).
  - Do NOT reject a page on nonzero OSNMA/SAR/SSP content (live SVs transmit
    all three); the sbas.rs MIN_BLOCK_WEIGHT-style degenerate-zero guard should
    apply to the 196-bit CRC-covered span (an all-zero span with all-zero CRC
    passes arithmetically; a real page has E/O=1 in the odd half, but the
    all-zeros Viterbi-collapse false-positive from sbas.rs:40-45 applies here
    identically).
- Sub-frame (ICD §4.3.3 Table 38, p. 40-41): 30 s = 15 nominal pages, T0
  synchronized to GST mod 30 s. On E1-B the EVEN part starts at ODD GST
  seconds (Table 38: T0=0 carries an odd part on E1-B). Nominal E1-B word
  slots per sub-frame: 2,4,6,(7|9),(8|10),(17|18),(19|20),16,0,22,1,3,5,0,16 —
  but the ICD says the sequence is INDICATIVE and deviations may appear per
  satellite. LAW: dispatch on the decoded word type, never on the nominal
  slot. Words 1-4 arrive once per 30 s, word 5 once per 30 s, word 6 and word
  0 at least once per 30 s, word 10 nominally once per 60 s (Table 39
  almanac alternation 7/8 vs 9/10).

## 3. Word types and bit offsets (verified: ICD §4.3.5 Tables 40-45, 49, 52 + §5.1 Tables 65-74)

Offsets are 0-based from the MSB of the assembled 128-bit word
(Data(1/2)[112] ++ Data(2/2)[16]). Fields marked * are two's-complement
(sign bit in MSB); unmarked are unsigned. "sc" = semi-circles (multiply by pi
for radians — same convention as the GPS/BDS parsers).

Word type = bits 0-5 in every word.

Word 1 (ephemeris 1/4, Table 40): IODnav 6-15 (10b) | t0e 16-29 (14b, x60 s) |
M0 30-61 (32b*, 2^-31 sc) | e 62-93 (32b, 2^-33) | sqrtA 94-125 (32b, 2^-19
m^1/2) | reserved 126-127.

Word 2 (ephemeris 2/4, Table 41): IODnav 6-15 | Omega0 16-47 (32b*, 2^-31 sc)
| i0 48-79 (32b*, 2^-31 sc) | omega 80-111 (32b*, 2^-31 sc) | idot 112-125
(14b*, 2^-43 sc/s) | reserved 126-127. (idot spans the odd-part 16 bits —
assemble the word before parsing.)

Word 3 (ephemeris 3/4 + SISA, Table 42): IODnav 6-15 | Omegadot 16-39 (24b*,
2^-43 sc/s) | delta_n 40-55 (16b*, 2^-43 sc/s) | Cuc 56-71 (16b*, 2^-29 rad) |
Cus 72-87 (16b*, 2^-29 rad) | Crc 88-103 (16b*, 2^-5 m) | Crs 104-119 (16b*,
2^-5 m) | SISA(E1,E5b) 120-127 (8b index).

Word 4 (ephemeris 4/4 + clock, Table 43): IODnav 6-15 | SVID 16-21 (6b) | Cic
22-37 (16b*, 2^-29 rad) | Cis 38-53 (16b*, 2^-29 rad) | t0c 54-67 (14b, x60 s)
| af0 68-98 (31b*, 2^-34 s) | af1 99-119 (21b*, 2^-46 s/s) | af2 120-125
(6b*, 2^-59 s/s^2) | spare 126-127.

Word 5 (iono + BGD + health + GST, Table 44): ai0 6-16 (11b, 2^-2 sfu) | ai1
17-27 (11b*, 2^-8 sfu/deg) | ai2 28-41 (14b*, 2^-15 sfu/deg^2) | SF1-SF5
42-46 (5x1b, reserved) | BGD(E1,E5a) 47-56 (10b*, 2^-32 s) | BGD(E1,E5b)
57-66 (10b*, 2^-32 s) | E5b_HS 67-68 (2b) | E1B_HS 69-70 (2b) | E5b_DVS 71
(1b) | E1B_DVS 72 (1b) | WN 73-84 (12b, GST weeks mod 4096) | TOW 85-104
(20b, s, 0..604799) | spare 105-127. TOW sits entirely inside the even part
(<= bit 111) — the ranging word.

Word 6 (GST-UTC, Table 45 + Table 73): A0 6-37 (32b*, 2^-30 s) | A1 38-61
(24b*, 2^-50 s/s) | dtLS 62-69 (8b*, s) | t0t 70-77 (8b, x3600 s) | WN0t
78-85 (8b) | WN_LSF 86-93 (8b) | DN 94-96 (3b, 1..7) | dtLSF 97-104 (8b*, s)
| TOW 105-124 (20b) | spare 125-127. Second TOW source for the anchor.

Word 0 (spare word, Table 52): Time 6-7 (2b) | spare 8-95 (88b) | WN 96-107 |
TOW 108-127. WN/TOW are VALID ONLY when Time == binary '10' (= 2) — gate hard
(fail-closed), this is the third TOW source.

Word 10 (almanac 3/2/2 + GGTO, Table 49 + Table 74): IODa 6-9 | [almanac SVID3
2/2: Omega0 10-25, Omegadot 26-36, M0 37-52, af0 53-68, af1 69-81, E5b_HS
82-83, E1B_HS 84-85] | A0G 86-101 (16b*, 2^-35 s) | A1G 102-113 (12b*, 2^-51
s/s) | t0G 114-121 (8b, x3600 s) | WN0G 122-127 (6b, GST week mod 64).
GGTO INVALID sentinel: all four fields all-ones -> not valid (ICD §5.1.8) —
publish nothing (fail-closed).

Sanity checks (soft, log-only): decoded TOW on E1-B should be odd (E1-B even
parts start at odd GST seconds, Table 38); word-5 TOW mod 30 nominally == 25
and word-6 TOW mod 30 == 5 on E1-B — nominal only, never a gate.

## 4. The ranging anchor

### 4.1 Time frame and the Sys::Galileo arm

GST epoch: 1999-08-22 00:00:00 UTC, at which GST read 13 s (ICD §5.1.2) —
identical to GPST's leap-second state at that date, so GST is aligned to GPST:
GST TOW == GPST TOW and GST WN 0 == GPS week 1024. The transmitted TOW/WN
refer to "the leading edge of the first chip of the first code sequence of the
first page symbol" of the page containing them (ICD §4.1.5 + §5.1.2) — i.e.
the start of the EVEN part's sync pattern, which is a code-period boundary.

Therefore, in the new `Sys::Galileo` arm of `end_second` (mirror the GPS arm
at src/live.rs:2123-2185):

- `t_tx = decoded TOW` directly — no BDS-style +14 s conversion. It is
  GPST-equivalent up to the GST-GPST offset (GGTO, tens of ns; Section 5.4).
  Unlike GPS ((tow_next-1)*6 names the NEXT subframe, live.rs:2130) and BDS
  (SOW at THIS preamble, +14 s, :2196), Galileo TOW directly names the start
  of its own page: `abs_sym` = absolute symbol index of the first sync symbol
  of the page whose CRC validated.
- `find_pages(&ch.nav_bits[from..])` (new, in galileo_inav.rs) mirrors
  `find_subframes`: scan the symbol stream for sync-pattern candidates every
  250 symbols... actually at EVERY offset (first sync), then re-validate on
  the 500-symbol page lattice; return validated pages newest-last with
  `sym_index` of the even part's first sync symbol and the decoded word.
  Rescan-back constant: one full page + slack = `GAL_RESCAN_BACK: usize = 502`
  symbols (the page needs nothing from the previous page — 500 + 2 slack),
  keeping the every-second re-anchor law (see GPS_RESCAN_BACK comment,
  src/live.rs:219-229).
- Publish rho_m/t_tx under the SAME `ch.locked && anchor_fresh` law
  (src/live.rs:2251-2270); ANCHOR_MAX_AGE_S = 30 s works unchanged: pages
  carrying TOW arrive at least ~2-3x per 30 s (words 0/5/6).
- `sys: "galileo"` already serializes (src/live.rs:74).

Storage decision: `ch.nav_bits: Vec<u8>` currently stores hard bits. For
Galileo store one HARD SYMBOL per 4 ms epoch in `nav_bits` (sign of prompt I)
and keep the last >=500 SOFT prompts (reuse `nav_ms` before draining) for the
Viterbi. Simplest honest structure: a per-channel `gal_syms: Vec<f32>` soft
ring (capped like nav_ms) drained by `nav_tick_gal`; `nav_bits` then stores
the DECODED 125 bit/s bits only for observability (nav_bits count in
SatReport), while page finding runs on the soft ring. Implementer may instead
run sync on hard symbols and Viterbi on the soft window — either is fine; the
spec's contract is only: (a) sync tries both polarities, (b) Viterbi input is
soft (sbas.rs convention: soft > 0 means symbol 0), (c) the anchor's
`abs_sym` indexes the same stream `t_bit_approx` bookkeeping counts.

### 4.2 Generalizing `anchor_stream_time` (the 4 ms ambiguity)

`anchor_stream_time` (src/live.rs:113-206) hardcodes three GPS/BDS facts that
break for E1B — all three must be parameterized (per-channel, from `sys`):

1. `0.02 * (abs_bit - nav_bits.len())` (line 119-120): 20 ms per nav unit.
   Galileo: the nav unit is the 4 ms symbol -> `0.004` (or `ch.ns_epoch as f64
   / ch.fs`, which is exact for every sys and the better refactor).
2. `ch.nav_ms.len() as f64 / 1000.0` (line 119): 1 ms per pending prompt.
   Galileo: pending prompts are 4 ms each -> divide by 250.0 (same
   `ns_epoch/fs` refactor covers it).
3. `(d_tx * 1000.0).round()` teeth-per-second (lines 160, 188): 1000 code
   periods/s. Galileo: t_code = 4 ms -> 250 periods/s. Refactor to
   `(d_tx / ch.t_code_nominal)` with `t_code_nominal = period_ms / 1000.0`
   (do NOT use `t_code_meas` for the count — the tooth COUNT is exact by
   transmit-synchronism; the measured period only places the teeth).

The 4 ms code-period ambiguity is structurally EASIER than GPS: one symbol =
one code period, so every symbol edge IS a code wrap and there is no
within-bit epoch ambiguity at all — the flip-dip audit (edge_off machinery,
GPS-only) is unnecessary for Galileo; leave `edge_off_valid=false` on GAL
channels and let the nearest-tooth snap resolve on the 4 ms lattice. The
snap's worst case is +-1 tooth = +-4 ms; the wrong-tooth self-heal and the
15 ms sanity gate (lines 143-198) work unchanged (a 4 ms tooth error is
caught by the same `(0.5..=1.5)` teeth window; a bit-slip equivalent is a
4 ms multiple, still far under the 15 ms gate only when wrong — verify the
gate arithmetic against 4 ms teeth in the window-time unit tests). The
tooth-exact propagation ("boundaries t_tx seconds apart are exactly N code
periods apart") holds identically at 250 teeth/s.

`nav_obs` (src/live.rs:2306) and the SatReport rho/t_tx path are
constellation-blind once the anchor exists — no change.

## 5. Ephemeris, clock, BGD, GGTO

### 5.1 Self-decoded ephemeris (words 1-4 + 5)

Batch assembly gate: words 1, 2, 3, 4 must carry the SAME IODnav (ICD
§5.1.9.2 — the IOD identifies the batch; this replaces the GPS practice of
matching IODE across subframes). Clock (af0/af1/af2, t0c) rides word 4;
BGD(E1,E5b) + E1B_HS + E1B_DVS ride word 5, which carries NO IODnav — ICD
§5.1.9.2 note: BGD/health are NOT issue-tagged; pair the newest valid word 5
with the current batch by recency, never by IOD.

Health law (two-sided, mirroring the round-14 GPS/BDS law in live.rs
:2144-2180): reject the batch when E1B_HS != 0 (0 = Signal OK; 1 = out of
service, 2 = Extended Operations Mode, 3 = in test — Table 82) OR E1B_DVS != 0
(0 = valid, 1 = working without guarantee — Table 79); a strictly newer
unhealthy issue evicts a stale-healthy incumbent; a same-issue health flip
evicts identically. Store health in `BrdcEph.health` (0 = healthy composite;
pack HS<<1|DVS nonzero otherwise) so the existing belts fire unmodified.

Newest-issue ordering: `(week, toe)` as GPS/BDS. Store `week` as the GPS
continuous week = GST WN + 1024 (valid until the GST 4096-week rollover,
~2077), matching what RINEX GAL records carry (RINEX aligns the GAL week to
the continuous GPS week number), so self-decode and BRDC records order
against each other correctly — same trick as BDS toe/toc GPST-equivalence.

Orbit/clock evaluation: identical Keplerian pipeline to GPS (ICD Table 66,
p. 55-56) with Galileo constants (Table 66 + Eq. 14-15, p. 55/58):
mu = 3.986004418e14 m^3/s^2 (== beidou_d1::MU_BDS, != gps MU_E 3.986005e14),
omega_E = 7.2921151467e-5 rad/s, F = -2 sqrt(mu)/c^2 = -4.442807309e-10
s/m^1/2 (GPS: -4.442807633e-10). Clock: dt_sv = af0 + af1*(t-t0c) +
af2*(t-t0c)^2 + F*e*sqrtA*sin(Ek) (Eq. 13-14), wrap-aware in t-t0c
(wrap_tk). Recommended shape: a `sat_at_txtime_gal` in a new
`src/galileo_inav.rs` (or a mu/F-parameterized core shared with
beidou_d1's — beidou_d1::sat_at_txtime_bds already uses the right mu, but
BDS's omega_e is 7.2921150e-5: do NOT reuse it blind).

### 5.2 RINEX BRDC source (`parse_rinex_gal`)

Same brdc_latest.rnx the GPS/BDS loaders read (hourly refresh cadence,
examples/clock_bias.rs:360-364). New strict RINEX-3 `E` record parser
mirroring `parse_rinex_bds` laws (unit votes, newest-issue, hard health
exclusion at selection). Record layout (RINEX 3.04): line 0: toc, af0, af1,
af2; orbit 1: IODnav, Crs, delta_n, M0; orbit 2: Cuc, e, Cus, sqrtA; orbit 3:
toe, Cic, OMEGA0, Cis; orbit 4: i0, Crc, omega, OMEGAdot; orbit 5: idot,
Data Sources, GAL week (continuous, GPS-aligned); orbit 6: SISA, SV health,
BGD(E1,E5a), BGD(E1,E5b); orbit 7: transmission time.

CRITICAL selection gate: GAL records appear TWICE per SV (I/NAV and F/NAV
issues) with DIFFERENT clock parameters — F/NAV af0-2 are the (E1,E5a) pair,
I/NAV the (E1,E5b) pair (ICD Table 69). Select ONLY records whose Data
Sources field has bit 0 set (I/NAV E1-B; bit 9 additionally flags the
E5b,E1 clock pair) — an F/NAV record's clock is wrong for the E1 user
equation below at the ns level AND its BGD slot differs. Mixing issues also
breaks newest-issue ordering. (Data Sources bit semantics: RINEX 3.04 §
Galileo record table / gLAB reference; verify against one live record from
brdc_latest.rnx at implementation — the file's 2026-08-29 issue carries E
records.) SV health field: bits for E1B DVS/HS included — exclude nonzero
E1B bits at selection, same as SatH1/GPS-health laws.

Precedence: RINEX base + self-decoded I/NAV batches win only when strictly
newer valid issue (mirror load_ephs; extending the tracker_eph.json merge to
GAL can follow the GPS pattern later — the BDS precedent shipped RINEX-only
first, examples/clock_bias.rs:154-161).

### 5.3 BGD for the E1-only user

I/NAV af0/af1/af2 are referenced to the (E1,E5b) ionosphere-free combination.
The single-frequency E1 user corrects (ICD §5.1.5 Eq. 17, f1 = E1):

    dt_sv(E1) = dt_sv(E1,E5b) - BGD(E1,E5b)

i.e. SUBTRACT the broadcast BGD(E1,E5b) (word 5 bits 57-66, 2^-32 s;
RINEX orbit-6 field 4) from the evaluated clock. This is the exact analogue
of the GPS `- e.tgd` term (gps/broadcast.rs:568) and BDS TGD1 — fold it into
the GAL clock evaluation the same way so build_meas_gal needs no extra term.
Magnitude: |BGD| is typically a few ns (meters) — invisible under the 1000 m
gate but free accuracy.

### 5.4 GGTO — decision: apply broadcast word 10, fold residual into ISB (Option B)

Both options against THIS solve architecture:

- Option A (second clock state): `solve_clock_only` is a deliberate 1-unknown
  weighted mean; a per-constellation clock state is exactly the ISB surgery
  the emitter already declares out of scope (examples/clock_bias.rs:31-36,
  569 — the n>=4 emit gate is in tension with any 2-state solve, and the
  paired-A/B membership law would need per-state membership). `solve_mixed`
  exists (gps/pvt.rs:358) but is a 5-unknown position solver, not the
  clock-only path. Option A is real future work, already queued with ISB
  surgery — NOT this change.
- Option B (apply broadcast GGTO): dt_systems = t_Galileo - t_GPS = A0G +
  A1G*(TOW - t0G + 604800*(WN - WN0G)) (ICD §5.1.8 Eq. 23; A0G 2^-35 s, A1G
  2^-51 s/s, t0G x3600 s, WN0G mod 64, roll-over law |WN-WN0G| <= 31;
  all-ones = invalid). Convert the anchor at measurement build time:
  `t_tx_gpst = t_tx_gst - dt_systems`. Costs one word-10 decode + one small
  cache; no solver change; the A/B law is untouched.

DECISION: Option B. Justification: it matches the station's BDS precedent
(constant frame conversion at the anchor + accepted unmodeled residual ISB),
requires zero solver surgery, and is fail-safe — when word 10 is absent or
all-ones, apply zero and let the raw GGTO (operationally tens of ns ~ meters;
even the full +-2^15*2^-35 s ~ +-0.95 us encodable range is ~285 m, under the
1000 m studentized gate) fold into the same unmodeled ISB the BDS rows
already ride, quantified per-row by the new n_gal field. What broadcast GGTO
can NEVER remove is the receiver's own E1B-vs-C/A inter-signal hardware/
correlation bias — that part is irreducibly ISB in a 1-state solve and is
precisely what the acceptance criterion tells the analyzer to watch.

Plumbing: the channel caches the newest valid word-10 GGTO (age-stamped by
t_proc like the SBAS caches); SatReport gains OPTIONAL additive fields
`ggto_ns` + `ggto_age_s` (null when invalid/absent — never 0.0-as-unknown;
fail-closed). The emitter applies it when present and finite.

### 5.5 Iono/tropo for GAL rows in build_meas_gal

E1 is 1575.42 MHz == L1: the SBAS IGP slant delay applies UNSCALED (unlike
the BDS lever-7 x1.01845 proposal). Apply igp_delay + tropo exactly as the
GPS branch; do NOT apply SBAS PRC/LT/DNU (per-GPS-PRN products, meaningless
for GAL). NeQuick-G from word-5 ai0-2 is OUT OF SCOPE (needs the MODIP grid
and the full RD1 algorithm; the SBAS grid is measured, local, and already
plumbed). Label: the SBAS grid is a GPS-L1-certified product applied to an
uncertified constellation at the same frequency — physically sound,
uncertified (same posture as lever 7).

## 6. Solve integration (examples/clock_bias.rs + analyzer)

### 6.1 Emitter changes

- Constellation split (clock_bias.rs:435-439): add `Some("galileo")`. THIRD
  smoother/prev chain pair (`smoothers_gal`, `prev_gal`) — PRNs collide
  across constellations (GAL E1B PRNs 1-36 overlap GPS 1-32; the existing
  comment at :341-346 states the law). Carrier wavelength: `LAM_L1` (E1
  carrier == L1). Carrier sign: inherited negated-carrier convention (same
  tracker NCO machinery; same "live verification pending" honesty label as
  BDS, clock_bias.rs:37-40 — copy that caveat, do not claim verified).
- `build_meas_gal`: mirror build_meas_bds's transmit-time iteration through
  `sat_at_txtime_gal`, plus GGTO conversion of t_tx (5.4), iono + tropo
  (5.5). BGD is inside the clock evaluation (5.3), like TGD1.
- Staircase/prediction machinery (freeze detect, 12 s window, innovation
  gate): unchanged — it is constellation-blind once the chain maps split.

### 6.2 Row fields (schema clock_bias-v4, ADDITIVE + one conscious analyzer change)

New row fields: `n_gal` (accepted post-rejection GAL count),
`n_gal_pre_reject`, `ggto_applied` (bool: any accepted GAL row used a
broadcast GGTO). `n_gps` must become an explicit count — it is currently
DERIVED as `n_sat - n_bds` (clock_bias.rs:588-593), which silently misattributes
GAL rows to GPS. Compute all three from accepted_indices against a per-row
sys tag (extend the `meas_is_bds: Vec<bool>` unzip to a sys enum vec).

### 6.3 THE TRAP — this change is NOT purely additive

scripts/clock_bias_analyzer.py:295 hard-gates `n_gps + n_bds != n_sat` (row
rejected). The moment a GAL row is accepted into n_sat, every such epoch
fails the analyzer's membership identity. Per station law this forces the
conscious same-change analyzer update: read `n_gal = row.get("n_gal", 0)`
(missing -> 0 keeps every historical v4 row passing unchanged) and generalize
the identity to `n_gps + n_bds + n_gal != n_sat`; extend
scripts/test_clock_bias_analyzer.py with (a) a GAL-bearing row passing, (b) a
GAL-bearing row violating the 3-way identity failing, (c) a historical row
without n_gal still passing. Schema string STAYS "clock_bias-v4" (the change
is field-additive and the identity generalization is backward-compatible; a
v5 bump would orphan the live series for no consumer gain). The gen id rolls
at producer restart anyway, marking the deploy boundary. pytest for these is
hardware-free and runnable NOW (before the Rust window).

### 6.4 A/B gate

Unchanged law: publish only when weighted/unweighted accepted_indices match
(clock_bias.rs:585-587). GAL rows flow through it like any others. The
silent-continue observability fix is lever 6, a separate trivial change — do
not entangle it here, but expect GAL's larger ISB spread to be the likeliest
first source of real mismatches once lever 6 lands (that is information, not
a defect).

### 6.5 Emit-gate note

The n>=4 gate (clock_bias.rs:569) and its recorded ISB tension now covers a
THREE-constellation single-state solve. The existing caveat comment must be
extended to name GAL when the arm lands — honesty rule: the comment currently
says "two constellations" and would become false.

## 7. Test plan

Rust tests compile-and-run only at the maintenance window (station law:
NEVER cargo now); write them WITH the code, claim nothing tested until the
window. Python tests run now.

1. Encoder/decoder vectors (unit, galileo_inav.rs): round-trip a constructed
   page — word -> 196-bit CRC span -> +CRC -> split even/odd -> +tails ->
   `sbas::conv_encode(bits, /*invert_g2=*/true, 0)` per part -> 30x8
   interleave -> prepend sync -> soft symbols (+noise via the sbas.rs test
   Rng) -> full decode -> bit-exact word + CRC pass. Cross-check the encoder
   against ICD Annex D "FEC Coding and Interleaving Numerical Examples"
   (transcribe at implementation — the Annex is in the same v2.1 PDF).
   Negative: G2 NOT inverted must fail CRC (guards the gotcha); wrong
   deinterleave orientation (30x8 written row-wise) must fail; alert page
   (PT=1) must decode CRC-clean and be refused as a word.
2. Word parsers (unit): synthetic words 1-6, 0, 10 at the Section 3 offsets,
   including sign-extension of every * field, t0e/t0c x60 scaling, the word-0
   Time != 2 refusal, the GGTO all-ones refusal, and IODnav batch-mismatch
   refusal across words 1-4.
3. Viterbi reuse guard (unit): `sbas::viterbi(soft, true)` decodes a
   conv_encode(_, true, _) stream — already proven machinery, one test pins
   the invert_g2=true path against a fixed vector so a sbas.rs refactor can't
   silently break Galileo.
4. anchor_stream_time generalization (unit): extend the existing anchor-path
   immunity tests (live.rs test helpers at :2997+) with a 4 ms-period channel:
   tooth-exact propagation at 250 teeth/s, +-1-tooth self-heal at 4 ms, and
   the d_tx==0 refresh law.
5. Fixture pages from a real capture: `wideband_l1_b1.iq` (20 Msps at
   1568.259 MHz, ~20 s, src/main.rs:1674) spans 1575.42 MHz; `examples/
   galileo_acq.rs` already acquires from file. At the window: track the
   strongest acquired E1B PRN through the new channel path over the replay
   (~10 pages/SV) and archive >=2 CRC-valid pages (with their soft symbols)
   as a checked-in fixture (tests/fixtures/), like the SBAS golden tests. If
   no PRN decodes from that capture (20 s is tight for a marginal SV), the
   capture procedure is: OPERATOR-ONLY at a maintenance window under the
   atomic pro_lease (the live tracker owns the radio — no hackrf_* from this
   work, ever), bounded single capture, >= 4 Msps (BOC(1,1) main lobes at
   +-1.023 MHz; 4 Msps covers +-2 MHz), centred 1575.42 MHz, 60 s (>= 25
   pages), to the observations/ archive.
6. RINEX cross-check (integration, window): decode words 1-4 live (or from
   the fixture stream), find the matching IODnav E record in brdc_latest.rnx
   via parse_rinex_gal, assert position agreement < 1 m and clock < 1 ns at
   the common toe, and BGD(E1,E5b) bit-exact — proves offsets, scale factors,
   sign extension, and the week alignment in one shot. (eph_cmp.rs is the
   existing pattern for this shape of test.)
7. Analyzer (pytest, NOW): the Section 6.3 cases; `python3 -m py_compile` on
   the touched scripts.
8. Live acceptance (post-deploy): GAL rho_m non-null on live rows; n_gal > 0
   accepted epochs; residual_rms_m distribution vs. the pre-deploy week (not
   degraded beyond the BDS-precedent ISB allowance); at least one epoch with
   all three constellations accepted.

## 8. Effort estimate by component

| Component | New/changed | Est. |
|---|---|---|
| galileo_inav.rs: sync + deinterleave + Viterbi glue + CRC + word structs/parsers (words 0-6, 10) + tests 1-3 | new file, ~600-800 lines incl. tests | 2-3 days |
| live.rs: Galileo prompt collection, nav_tick_gal, Sys::Galileo anchor arm, anchor_stream_time generalization + tests 4 | ~200-300 lines changed | 1-2 days (highest-risk: the tooth arithmetic; the unit tests carry it) |
| sat_at_txtime_gal + clock/BGD eval (parameterized core) | ~100 lines | 0.5 day |
| parse_rinex_gal (+ data-sources gate + health) + tests | ~150-200 lines | 1 day |
| clock_bias.rs emitter: third chain, build_meas_gal, GGTO apply, sys-tagged counts | ~150 lines | 0.5-1 day |
| Analyzer + pytest (Section 6.3) | ~20 lines + 3 tests | 0.5 day (runnable now) |
| Window validation: replay fixture extraction, RINEX cross-check, live soak | — | 1 window + 1 day soak |
| Total | | ~6-8 engineering days + 1 maintenance window |

## 9. Sources

- Galileo OS SIS ICD, Issue 2.1, November 2023 (gsc-europa.eu PDF; verified
  directly this session): Table 24/Fig. 13 (FEC, G2 inversion, p. 28), Table
  25 (30x8 interleaver, p. 29), §4.1.5 (TOW reference instant, p. 29), §4.3.2
  Tables 35-37 (page layout, sync 0101100000, CRC coverage, p. 37-39), §4.3.3
  Table 38 (sub-frame timing, p. 40-41), §4.3.5 Tables 40-52 (word bit
  allocations, p. 43-47), Table 65-66 (ephemeris + user algorithm + mu/
  omega_E, p. 54-56), §5.1.2 (GST epoch/WN/TOW, p. 56), Table 68-69 (clock,
  E1/E5b pair, p. 57), Eq. 13-15 (clock + relativistic F, p. 58), §5.1.5 Eq.
  16-18 + Tables 70-71 (BGD, p. 58-59), Table 72 (iono, p. 59-60), Table 73
  (GST-UTC, p. 61), §5.1.8 Eq. 23 + Table 74 (GGTO + invalid sentinel, p.
  62-63), Tables 77-82 (DVS/HS, p. 64-65), §5.1.9.4 (CRC-24Q polynomial, p.
  65), §5.1.9.5 Table 83 (SSP, p. 66). NOTE: the fetched copy is stamped
  "Superseded" (a 2.2 exists); the fields cited here are stable, but the
  implementer should diff 2.2's change record at implementation time.
  https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v2.1.pdf
- PocketSDR sdr_nav.py (T. Takasu) — independent implementation cross-check:
  sync pattern, 8x30 transpose deinterleave, G2 inversion, 220-bit
  zero-syndrome CRC form.
  https://raw.githubusercontent.com/tomojitakasu/PocketSDR/master/python/sdr_nav.py
- ESA Navipedia "Galileo Navigation Message" (page-part framing cross-check).
  https://gssc.esa.int/navipedia/index.php/Galileo_Navigation_Message
- RINEX 3.04 (IGS) + gLAB GALILEO Navigation RINEX 3.04 reference (Data
  Sources bit 0 = I/NAV E1-B, bit 9 = af0-2 for E5b,E1; verify on a live E
  record at implementation).
  http://acc.igs.org/misc/rinex304.pdf /
  https://server.gage.upc.edu/gLAB/HTML/GALILEO_Navigation_Rinex_v3.04.html
- This codebase: src/sbas.rs, src/galileo.rs, src/live.rs, examples/
  clock_bias.rs, src/gps/{broadcast,pvt}.rs, src/beidou_d1.rs,
  scripts/clock_bias_analyzer.py (+ its pytest), docs/superpowers/plans/
  2026-09-01-availability-design.md — line references inline above, read at
  HEAD 0f5c669 working tree this session.
