#!/usr/bin/env python3
"""Carrier-phase-derived clock-drift producer ("phase drift producer").

The station's highest-precision drift source. Reads ONLY
observations/state.tracker.json (the 1 Hz live tracker's per-satellite
carrier phase, commit dcfcfaa) and writes ONLY its own
observations/state.phase_drift.json (tmp + os.replace; the Rust server
merges all observations/state.*.json at /api/sync read time, so no
shared-file race is possible). Pure reader/writer — touches no radio,
signals no process.

Measurement: per locked SBAS/WAAS GEO satellite, the least-squares slope
of carrier_cycles over a sliding window (default 40 s, clamp 30-60) IS
the integrated Doppler in cycles/s = Hz — the same observable the code
row ("L1 / WAAS (live)") derives from the instantaneous PLL doppler_hz,
but at carrier-phase precision: live, the per-second NCO-advance noise
is ~1 cycle (the reported doppler_hz itself jitters +/-15 Hz — the
accumulator is the quiet observable), so the slope sigma over a 40 s
window is ~0.02 Hz = 1.27e-5 ppm class (0.02 / 1575.42e6 * 1e6) — ~3
orders of magnitude finer than the code Doppler's 0.01-0.05 ppm. TCXO
wander over the window shows up in the fit residuals, so the honest fit
sigma lands above the thermal floor — still well below any code-class
voter.

Convention (mirrors tracker_producer's WAAS ClockDriftPpm row exactly):
the measured slope is taken AFTER the hardware clock-correction register
(resid = raw - corr), so the published value adds the register back:
    ppm = slope_hz / L1_HZ * 1e6 + discipline.correction_ppm
ONLY when the discipline state says this live_radio process verifiably
wrote that correction (actuate AND corr_applied). The cache is intent,
not applied truth: in SHADOW mode the register is unity, the add-back
is 0, and rows carry corr_applied: false so consumers can tell.
Making the row a RAW-TCXO measurement comparable to every other voter
(ATSC ch35 via CLKOUT, PC clock). GEO line-of-sight motion Doppler IS
subtracted in the published rows (P0b below — promoted after the
2026-08-29 acceptance hour; before it, the rows matched the code row in
absorbing the range rate in the sigma floor). The honest fit sigma is
published as-is; inter-source systematics (residual ephemeris rate
error — the corrected GEOs agreed to 2.4e-7 ppm in the acceptance hour;
CLKOUT chain) are visible in the scatter.

NON-VOTING (systems review 2026-08-25, finding 6): this chain is an
implemented DIAGNOSTIC, not a discipline voter. It shares the radio and
the GEOs with the WAAS code row (one instrument must not vote twice),
and the uncertainty is white-noise OLS rather than correlated-residual
(the GEO range rate itself is removed — P0b below). The consensus row is
therefore published as ClockDriftPpmComponent — visible in the merged
sources table, excluded from series_producer's voting consensus — until
real correlated-residual uncertainty lands (the Tier-1 review).

P0b (PROMOTED 2026-08-29, after the acceptance hour): the tracker's MT9
(DO-229D A.4.5.1) sbas_geonav state vector lets a GeoCorrector per SBAS
channel remove the GEO line-of-sight range rate AND the GEO clock:
    corr(t) = cycles(t) + rho(t)/lambda_L1 - f_L1 * dt_geo(t)
(step-free-stitched across ephemeris swaps, re-anchored on slips). The
CORRECTED slope is now the emitted observable: per-sat rows carry the
corrected ppm and corrected sigma (the max(OLS, disjoint-window
scatter) machinery runs on the corrected fit history), row extra keeps
the raw value as evidence (p0b: true, uncorr_ppm), and the diag keeps
both (ppm = corrected = emitted; uncorr_ppm / uncorr_sigma_ppm; p0b_*
provenance). FAIL-CLOSED: when the corrector does not apply or the
corrected window has no fit (no-geonav / stale / ura / bad-geonav /
post-slip refill) the sat emits NO row and NO consensus vote — a GEO
row without the motion correction is not a clock observable. The
shadow jsonl keeps appending corrected vs uncorrected ppm per GEO per
cycle, verbatim (the promotion evidence chain). Acceptance hour
(2026-08-29, 6546 shadow rows): cross-PRN |median_131 - median_135|
2.36e-7 ppm corrected vs 2.12e-3 ppm uncorrected (~9000x; gate was
<2e-4) — both GEOs read the same -2.4e-5 ppm, the common receiver
clock; PRN 135's uncorrected 10-min medians marched +2.59e-3 ->
+1.72e-3 (diurnal geometry) while corrected stayed pinned at -2.4e-5
+- 1.3e-4; 17 IODN swaps per PRN stitched cleanly; within-bin MAD
unchanged (the correction is quasi-static). Rows stay
ClockDriftPpmComponent — observe-only; cross-producer Tier-1 vote
promotion is a separate review step.

Continuity guards (a window is only as good as its phase chain):
  - slip=true on any report -> the chain broke that second (watchdog or
    reseed): the window is flushed and the slip sample itself is dropped.
  - reseed without a slip flag shows as carrier_cycles snapping back to
    ~0: |new| < 2% of |prev| with |prev| > 50 cycles is a collapse, not
    physics (a GEO Doppler cannot walk the accumulator back to zero in
    one report). The increment-vs-median check below catches smaller
    breaks.
  - increments are compared against the chain's OWN median increment,
    NOT against the reported doppler_hz: live, the FLL's doppler state
    jitters +/-15 Hz second-to-second while the integrated NCO advance
    wanders only ~1 cycle/s — the accumulator is the quiet observable,
    the Doppler report is not. Break threshold: 25 cycles in one step
    (nothing physical moves a GEO Doppler 25 Hz in 1 s; loop events
    that could come with slip / lock_s = 0 anyway).
  - a discipline STEP mid-window (the APPLIED correction changed between
    two reports) retunes the LO and shifts every channel's measured rate
    by up to step·L1 (0.03 ppm = 47 Hz); the register add-back convention
    is only exact for a constant register, so the window flushes on any
    applied correction change (shadow-mode intent edits never touch
    hardware and don't flush). Once the loop is in its deadband this is
    rare.
  - report gaps > 5 s or non-monotonic epochs (producer restart, file
    rotation) flush the window.
  - gates: newest lock_s >= window length (locked for the whole window)
    and every in-window cn0_proxy >= 30 dB (the tracker's UNLOCK_DB).

Missing fields (pre-dcfcfaa state files lack carrier_cycles/phase_frac/
slip): those sats are skipped; if no sat carries the fields the producer
just heartbeats its file with no rows — rows vanish rather than freeze
(the tombstone class).

Publishes:
  one consensus row (band "L1 / WAAS GEO (phase)") = sigma-weighted
  median across GEOs, sigma =
  max(formal 1/sqrt(Σw), weighted scatter) — the scatter term keeps the
  consensus honest when GEOs disagree (post-P0b the corrected GEOs agree
  to ~2.4e-7 ppm — the acceptance hour; pre-promotion the uncorrected
  GEOs spread ~0.002 ppm from their differing line-of-sight rates).
  ALL rows (per-satellite and consensus) are kind ClockDriftPpmComponent
  — visible in the merged sources table but NON-VOTING (see the
  NON-VOTING note above): one instrument must vote once in
  series_producer's cross-producer consensus, and this chain shares the
  radio with the WAAS code row.
  state["phase_drift"] — diagnostics: per-sat slope/fit sigma/n/gates
  (all on the emitted, corrected series), plus uncorr_ppm /
  uncorr_sigma_ppm and the P0b provenance keys (p0b_*).

Unit tests: scripts/test_phase_drift_producer.py (synthetic fixtures,
no live observations touched).
"""
import argparse
import json
import math
import os
import time

OBS = "/Volumes/Radiator 8TB/gnss/observations"
TRACKER = f"{OBS}/state.tracker.json"
STATE = f"{OBS}/state.phase_drift.json"
L1_HZ = 1575.42e6
C_MPS = 299792458.0
MY_BAND = "L1 / WAAS GEO (phase)"

WINDOW_S = 40.0           # sliding fit window; clamped to [30, 60]
MIN_SAMPLES_FRAC = 0.7    # of window length (1 Hz reports)
MIN_CN0 = 30.0            # dB — the tracker's UNLOCK_DB
GAP_S = 5.0               # report gap longer than this breaks the chain
BREAK_CYC = 25.0          # one-step increment deviation = phase break
COLLAPSE_CYC = 50.0       # |accumulator| above this can reseed-collapse
COLLAPSE_FRAC = 0.02      # ...to below this fraction of its magnitude

SHADOW_JSONL = f"{OBS}/phase_drift_p0b_shadow.jsonl"  # P0b evidence file

# P0b SHADOW constants (GEO geometric correction from the tracker's MT9
# sbas_geonav publication, SI units — src/live.rs GeoNavPub)
GPS_UNIX_EPOCH = 315964800.0   # 1980-01-06T00:00:00 UTC as unix seconds
GPS_LEAP_S = 18.0              # GPS - UTC leap seconds (pinned; 18 since 2017)
GEO_MAX_DT_S = 3600.0          # 2nd-order Taylor propagation bound, s
GEO_MAX_URA = 7                # MT9 URA usability gate
# applied_t (stream-time of last MT9 application) advances only on message
# events — observed tens of seconds live. If it stops advancing for this
# long, the cached vector is no longer being maintained by fresh decodes:
# fail closed. 300 s tolerates several missed MT9 broadcasts.
GEO_MSG_FRESH_S = 300.0
# Largest accepted ephemeris-swap stitch, cycles (~95 m combined orbit+clock
# model difference). Operational MT9 refreshes are decimetre-class (a few
# cycles); a larger jump means a bogus/incompatible vector, and stitching it
# would poison the corrected series invisibly. The new vector is REJECTED
# (the known-good one is kept) instead of being absorbed.
GEO_STITCH_MAX_CYCLES = 500.0
LAM_L1_M = C_MPS / L1_HZ       # L1 carrier wavelength (~0.1903 m)
WGS84_A = 6378137.0            # semi-major axis, m
WGS84_F = 1.0 / 298.257223563  # flattening


def log(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def fit_drift(samples):
    """Least-squares slope of (t, cycles) samples.

    Returns (slope_hz, sigma_hz, n) or None when the window is degenerate
    (<3 points or zero time span). sigma_hz is the 1-sigma slope
    uncertainty from the fit residuals: sqrt(Σr²/(n-2) / Σ(t-t̄)²).

    KNOWN LIMITATION (measured 2026-08-28, Monte Carlo, 4000 trials,
    AR(1) ρ=0.95, n=40): when the phase residuals are strongly
    autocorrelated, detrending absorbs the low-frequency wander into the
    fit and the residual-based OLS sigma UNDERESTIMATES the true slope
    scatter ~5× (empirical 1.15e-3 vs OLS 2.4e-4, 17% 1-σ coverage). A
    Newey-West HAC (L=4) on the same residuals is WORSE (19× under, 4%
    coverage — the fit destroyed the low-frequency information the HAC
    needs), so the round-13 HAC prescription was implemented, measured
    and reverted. Calibrated paths, in order: (a) cross-window empirical
    slope scatter (the producer fits every window; the scatter IS the
    honest sigma) — LANDED 2026-08-29: emitted GEO sigmas are
    max(fit_sigma, scatter) once >=5 disjoint windows exist, and the
    provisional 5x inflation of this OLS value before that;
    (b) parametric AR with an externally pinned ρ band."""
    n = len(samples)
    if n < 3:
        return None
    t0 = samples[0][0]
    xs = [t - t0 for t, _ in samples]
    ys = [c for _, c in samples]
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0.0:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx
    s2 = sum((y - my - slope * (x - mx)) ** 2 for x, y in zip(xs, ys)) / (n - 2)
    return slope, math.sqrt(s2 / sxx), n


def weighted_median(values_sigmas):
    """Sigma-weighted median of (value, sigma) pairs.

    Returns (value, sigma); sigma = max(formal 1/sqrt(Σ 1/σᵢ²),
    weighted scatter about the median) — the scatter term keeps the
    consensus honest when the GEOs genuinely disagree."""
    pts = sorted((v, max(s, 1e-12)) for v, s in values_sigmas)
    w = [1.0 / (s * s) for _, s in pts]
    tot = sum(w)
    acc = 0.0
    med = pts[-1][0]
    for (v, _), wi in zip(pts, w):
        acc += wi
        if acc >= 0.5 * tot:
            med = v
            break
    formal = math.sqrt(1.0 / tot)
    scatter = math.sqrt(sum(wi * (v - med) ** 2 for (v, _), wi in zip(pts, w)) / tot)
    return med, max(formal, scatter)


def scatter_sigma(slopes):
    """Robust sigma of DISJOINT-window slope estimates: 1.4826*MAD.

    The calibrated answer to the autocorrelated-residual problem
    (fit_drift docstring, 2026-08-29): per-window OLS sigmas understate
    ~5x on AR(1) phase residuals and a residual-based HAC is worse still;
    the scatter of fits over non-overlapping windows is the honest slope
    sigma because each fit is an independent draw. Real GEO drift
    variation inside the span inflates it slightly — the conservative
    direction. Needs >= 5 fits for a meaningful MAD."""
    if len(slopes) < 5:
        return None
    med = sorted(slopes)[len(slopes) // 2]
    mad = sorted(abs(s - med) for s in slopes)[len(slopes) // 2]
    return 1.4826 * mad


def gps_tod_s(unix_t):
    """GPS time-of-day (s into the GPS day) for a unix timestamp.

    MT9's t0 is broadcast as seconds into the GPS day (13 bits x 16 s),
    so the corrector works entirely in the GPS-day frame. The leap count
    is pinned as a module constant (18 s since 2017-01-01) rather than
    read from a table: every discipline on this station lives in the
    current era."""
    return (unix_t - GPS_UNIX_EPOCH + GPS_LEAP_S) % 86400.0


def wrap_tod(dt):
    """Wrap a time-of-day difference to [-43200, +43200) s — the
    day-boundary-safe propagation argument against MT9's t0 (a state
    vector applied just before GPS midnight is evaluated just after)."""
    return (dt + 43200.0) % 86400.0 - 43200.0


def llh_to_ecef(lat_deg, lon_deg, h_m):
    """WGS-84 geodetic -> ECEF metres (constants pinned above; no dep)."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    e2 = WGS84_F * (2.0 - WGS84_F)
    s = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - e2 * s * s)
    return ((n + h_m) * math.cos(lat) * math.cos(lon),
            (n + h_m) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - e2) + h_m) * math.sin(lat))


def load_site_ecef(path=f"{OBS}/site.json"):
    """Canonical site anchor -> ECEF, or None. Fail-closed: no anchor, no
    correction — a wrong anchor is worse than none (sky_producer treats
    the same file as the one canonical anchor, never a guess)."""
    try:
        with open(path) as f:
            site = json.load(f)
        return llh_to_ecef(site["lat"], site["lon"], site["h_m"])
    except Exception:
        return None


class GeoCorrector:
    """P0b geometric correction for one SBAS channel, from the tracker's
    published MT9 state vector (sbas_geonav, SI units — DO-229D A.4.5.1,
    src/live.rs GeoNavPub). PROMOTED 2026-08-29: the corrected fit is
    the emitted observable (fail-closed via correct()'s gates).

    Sign derivation (pinned by the synthetic tests): the measured carrier
    slope is -rho_dot/lambda + f_L1*agf1 + f_clock (range shortening
    advances the phase — the tracker's f_D < 0 when the satellite
    recedes — and the GEO clock polynomial rides on top), so the
    correction ADDS the geometric/clock term back:
        corr(t) = cycles(t) + rho(t)/lambda_L1 - f_L1 * dt_geo(t)
    leaving d(corr)/dt = f_clock — the oscillator observable, with the
    GEO's line-of-sight range rate (+-1 Hz class, the +-0.025 ppm floor
    the uncorrected rows absorb in their sigma) removed.

    rho(t) is the geometric range site->GEO with the state vector
    propagated by the 2nd-order Taylor series of DO-229D A.4.5.1.1
    (pos = p0 + v0*dt + a*dt^2/2) and dt wrapped across the GPS-day
    boundary; dt_geo = agf0 + agf1*dt is the GEO clock polynomial.

    Continuity across ephemeris swaps: MT9 updates arrive on a ~2-minute
    cadence with a fresh t0/iodn, and two consecutive state vectors
    disagree by decimetre-class extrapolation differences — ~cycle-class
    jumps in rho/lambda. A naive swap would step the corrected series and
    poison every slope fit spanning the swap, so update() STITCHES: at
    the swap instant t it adds (old_term(t) - new_term(t)) to a running
    offset, making the corrected series step-free by construction. A
    channel slip/reseed breaks the phase chain itself, so stitching is
    pointless there — re-anchor (offset reset to 0) instead; a step is
    then legal."""

    def __init__(self, site_ecef):
        self.site = site_ecef
        self.geo = None             # latest accepted sbas_geonav dict (SI)
        self.offset_cycles = 0.0    # running stitch across ephemeris swaps
        self.last_dt = None         # propagation dt of the last applied sample
        self._last_applied_t = None  # last seen geonav["applied_t"] value
        self._msg_refresh_unix = None  # t_unix when applied_t last advanced
        self.rejected_swaps = 0     # malformed/oversized swap rejections
        self._rejected_fingerprint = None

    @staticmethod
    def _fingerprint(geo):
        """Identity of every correction-bearing field in a GEO vector.

        `iodn` and `t0_s` alone are not enough: a corrected retry can retain
        both while changing a malformed position/velocity/clock term. The
        decode timestamp is deliberately excluded so repeated publication of
        the same bad vector stays quiet without refreshing accepted data.
        """
        if not isinstance(geo, dict):
            return ("not-a-dict", repr(geo))

        def atom(value):
            if isinstance(value, float) and not math.isfinite(value):
                return ("nonfinite", repr(value))
            return value

        def vector(name):
            value = geo.get(name)
            return (tuple(atom(item) for item in value)
                    if isinstance(value, (list, tuple)) else atom(value))

        return (
            atom(geo.get("iodn")), atom(geo.get("t0_s")), atom(geo.get("ura")),
            vector("pos_m"), vector("vel_mps"), vector("acc_mps2"),
            atom(geo.get("agf0_s")), atom(geo.get("agf1_sps")),
        )

    @staticmethod
    def _normalize_geo(geo):
        """Return (normalized vector, error) for a plausible MT9 vector."""
        if not isinstance(geo, dict):
            return None, "not-an-object"
        try:
            iodn = geo["iodn"]
            ura = geo["ura"]
            if type(iodn) is not int or type(ura) is not int:
                raise ValueError("integer field is not an exact JSON integer")

            def number(value):
                if type(value) not in (int, float):
                    raise ValueError("not a JSON number")
                value = float(value)
                if not math.isfinite(value):
                    raise ValueError("nonfinite")
                return value

            t0_s = number(geo["t0_s"])
            applied_t = number(geo["applied_t"])
            if not all(isinstance(geo[name], (list, tuple))
                       for name in ("pos_m", "vel_mps", "acc_mps2")):
                raise ValueError("vector is not an array")
            pos = [number(x) for x in geo["pos_m"]]
            vel = [number(x) for x in geo["vel_mps"]]
            acc = [number(x) for x in geo["acc_mps2"]]
            agf0 = number(geo["agf0_s"])
            agf1 = number(geo["agf1_sps"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return None, "missing-or-nonnumeric-field"
        if any(len(v) != 3 for v in (pos, vel, acc)):
            return None, "vector-length"
        radius = math.sqrt(sum(v * v for v in pos))
        if not (0 <= iodn <= 255 and 0.0 <= t0_s < 86400.0
                and 0 <= ura <= 15 and applied_t >= 0.0):
            return None, "scalar-range"
        # Generous physics bounds, intended to catch corruption rather than
        # police valid SBAS quantization at its edge.
        if not (20e6 <= radius <= 60e6):
            return None, "position-range"
        if max(abs(v) for v in vel) > 10_000.0:
            return None, "velocity-range"
        if max(abs(v) for v in acc) > 100.0:
            return None, "acceleration-range"
        if abs(agf0) > 1.0 or abs(agf1) > 1e-4:
            return None, "clock-range"
        return {
            "iodn": iodn, "t0_s": t0_s, "ura": ura,
            "applied_t": applied_t, "pos_m": tuple(pos),
            "vel_mps": tuple(vel), "acc_mps2": tuple(acc),
            "agf0_s": agf0, "agf1_sps": agf1,
        }, None

    @classmethod
    def _validate_geo(cls, geo):
        """Compatibility helper: None means the vector normalizes cleanly."""
        _, error = cls._normalize_geo(geo)
        return error

    def _term(self, geo, t_unix):
        """Correction term rho/lambda_L1 - f_L1*dt_geo (cycles) and the
        day-wrapped propagation dt (s) for one state vector."""
        dt = wrap_tod(gps_tod_s(t_unix) - geo["t0_s"])
        dt2 = dt * dt
        px = geo["pos_m"][0] + geo["vel_mps"][0] * dt \
            + 0.5 * geo["acc_mps2"][0] * dt2
        py = geo["pos_m"][1] + geo["vel_mps"][1] * dt \
            + 0.5 * geo["acc_mps2"][1] * dt2
        pz = geo["pos_m"][2] + geo["vel_mps"][2] * dt \
            + 0.5 * geo["acc_mps2"][2] * dt2
        dx, dy, dz = px - self.site[0], py - self.site[1], pz - self.site[2]
        rho = math.sqrt(dx * dx + dy * dy + dz * dz)
        clk = geo["agf0_s"] + geo["agf1_sps"] * dt
        term = rho / LAM_L1_M - L1_HZ * clk
        if not all(math.isfinite(v) for v in (term, dt, rho, clk)):
            raise ValueError("nonfinite geonav correction")
        return term, dt

    def update(self, geonav, t_unix, slip=False):
        """Accept the tracker's latest sbas_geonav (called each cycle;
        None = field absent, keep the cached vector)."""
        if slip:
            # a slip/reseed broke the phase chain: re-anchor, don't stitch
            self.offset_cycles = 0.0
        if geonav is None:
            return
        # Message freshness is committed only after this candidate is
        # accepted. A rejected vector must never keep the old accepted vector
        # alive by advancing applied_t.
        fingerprint = self._fingerprint(geonav)
        normalized, invalid = self._normalize_geo(geonav)
        try:
            finite_t = (type(t_unix) in (int, float)
                        and math.isfinite(float(t_unix)))
        except (TypeError, ValueError, OverflowError):
            finite_t = False
        if not finite_t:
            invalid = invalid or "nonfinite-update-time"
        if invalid is not None:
            if fingerprint == self._rejected_fingerprint:
                return
            self.rejected_swaps += 1
            self._rejected_fingerprint = fingerprint
            print(f"p0b: REJECTED malformed geonav ({invalid})", flush=True)
            return
        geonav = normalized
        t_unix = float(t_unix)
        fingerprint = self._fingerprint(geonav)
        at = geonav["applied_t"]
        if (self._last_applied_t is not None and at < self._last_applied_t):
            if fingerprint != self._rejected_fingerprint:
                self.rejected_swaps += 1
                self._rejected_fingerprint = fingerprint
                print("p0b: REJECTED regressed geonav applied_t", flush=True)
            return
        correction_changed = (self.geo is not None
                              and fingerprint != self._fingerprint(self.geo))
        if not slip and correction_changed:
            # ephemeris swap mid-chain: VALIDATE the new vector before it
            # may replace the old one, then stitch the corrected series
            # step-free at t_swap — offset += old_term(t) - new_term(t)
            if fingerprint == self._rejected_fingerprint:
                return      # already rejected this exact vector; stay quiet
            try:
                old_term, _ = self._term(self.geo, t_unix)
                new_term, _ = self._term(geonav, t_unix)
            except (KeyError, TypeError, ValueError, IndexError, OverflowError):
                # malformed vector: reject it outright — keep the
                # known-good vector rather than swapping in garbage
                self.rejected_swaps += 1
                self._rejected_fingerprint = fingerprint
                print(f"p0b: REJECTED malformed geonav swap "
                      f"(iodn {self.geo.get('iodn')} -> "
                      f"{geonav.get('iodn')})", flush=True)
                return
            jump = old_term - new_term
            if not math.isfinite(jump) or abs(jump) > GEO_STITCH_MAX_CYCLES:
                print(f"p0b: REJECTED oversized geonav swap {jump:.1f} cycles "
                      f"> {GEO_STITCH_MAX_CYCLES:.0f} "
                      f"(iodn {self.geo.get('iodn')} -> "
                      f"{geonav.get('iodn')})", flush=True)
                self.rejected_swaps += 1
                self._rejected_fingerprint = fingerprint
                return
            self.offset_cycles += jump
            self._rejected_fingerprint = None
            print(f"p0b: stitched geonav swap, {jump:+.2f} cycles "
                  f"(iodn {self.geo.get('iodn')} -> {geonav.get('iodn')})",
                  flush=True)
        self.geo = geonav
        self._rejected_fingerprint = None
        if at is not None and at != self._last_applied_t:
            self._last_applied_t = at
            self._msg_refresh_unix = t_unix

    def correct(self, t_unix, cycles):
        """-> (corrected_cycles, applied, reason). Fail-closed gates: the
        correction is applied ONLY from a present, usable (ura <= 7),
        fresh (|dt| <= 3600 s after day-wrap) state vector — anything
        else returns the input unchanged with a short reason."""
        geo = self.geo
        if geo is None:
            return cycles, False, "no-geonav"
        try:
            t_unix = float(t_unix)
            cycles_value = float(cycles)
        except (TypeError, ValueError, OverflowError):
            return cycles, False, "nonfinite-input"
        if not math.isfinite(t_unix) or not math.isfinite(cycles_value):
            return cycles, False, "nonfinite-input"
        ura = geo.get("ura")
        if ura is None or ura > GEO_MAX_URA:
            return cycles, False, "ura"
        message_age = (None if self._msg_refresh_unix is None
                       else t_unix - self._msg_refresh_unix)
        if (message_age is None or not math.isfinite(message_age)
                or not 0.0 <= message_age <= GEO_MSG_FRESH_S):
            # the tracker stopped refreshing this vector (MT9 decode
            # outage or frozen publication) — propagation age from t0 is
            # not evidence the message itself is still current
            return cycles, False, "msg-stale"
        try:
            term, dt = self._term(geo, t_unix)
        except (KeyError, TypeError, ValueError, IndexError, OverflowError):
            return cycles, False, "bad-geonav"
        if abs(dt) > GEO_MAX_DT_S:
            return cycles, False, "stale"
        self.last_dt = dt
        corrected = cycles_value + term + self.offset_cycles
        if not math.isfinite(corrected):
            return cycles, False, "nonfinite-correction"
        return corrected, True, None


class SatWindow:
    """Sliding window of continuity-verified (t, cycles, cn0) samples for
    one channel. Any phase break — slip flag, reseed collapse, increment
    jump vs the chain's own median, report gap, epoch going backwards, or
    a discipline-step retune — flushes the window; only samples on one
    unbroken chain may join a fit."""

    def __init__(self, window_s):
        self.window_s = float(window_s)
        self.samples = []          # (t, cycles, cn0) on the current chain
        self.incs = []             # per-step cycle increments on the chain
        self.last = None           # last seen (t, cycles, corr)
        self.lock_s = 0.0          # lock_s of the newest report
        self.fit_hist = []         # slopes of DISJOINT windows (<=12 kept)
        self.last_fit_t = -1e18    # t of the last recorded disjoint fit
        self.generation = 0        # increments at every phase-chain break

    def _break_chain(self):
        """Start a new statistical generation after a phase discontinuity."""
        self.samples = []
        self.incs = []
        self.last = None
        # Fits on opposite sides of a slip/reseed/retune are not draws from
        # one continuous phase process. Never carry their scatter estimate
        # across the boundary.
        self.fit_hist = []
        self.last_fit_t = -1e18
        self.generation += 1

    def add(self, t, cycles, slip=False, cn0=None, lock_s=None, corr=None):
        """Ingest one 1 Hz report. Returns True if it joined the chain."""
        def finite_number(value, *, optional=False):
            if optional and value is None:
                return None
            if type(value) not in (int, float):
                raise ValueError("not a JSON number")
            value = float(value)
            if not math.isfinite(value):
                raise ValueError("nonfinite")
            return value

        try:
            t = finite_number(t)
            cycles = finite_number(cycles)
            cn0 = finite_number(cn0, optional=True)
            lock_s = finite_number(lock_s, optional=True)
            corr = finite_number(corr, optional=True)
            if type(slip) is not bool:
                raise ValueError("slip is not boolean")
        except (TypeError, ValueError, OverflowError):
            self._break_chain()
            return False
        if lock_s is not None:
            self.lock_s = lock_s
        if self.last is not None:
            pt, pc, pcorr = self.last
            gap = t - pt
            broken = gap <= 0.0 or gap > GAP_S or slip or corr != pcorr
            if not broken:
                # reseed collapse: the accumulator snapped back to ~0.
                # A GEO Doppler cannot walk |pc| > 50 cycles back to zero
                # in one report — this is a chain break, flag or no flag.
                broken = (abs(pc) > COLLAPSE_CYC
                          and abs(cycles) < COLLAPSE_FRAC * abs(pc))
            if not broken and self.incs:
                # increment vs the chain's OWN median: the reported
                # doppler_hz (FLL state) jitters +/-15 Hz live while the
                # NCO advance wanders ~1 cycle/s, so only the chain's own
                # statistics are a quiet enough reference
                med = sorted(self.incs)[len(self.incs) // 2]
                broken = abs((cycles - pc) - med * gap) > BREAK_CYC
            if broken:
                self._break_chain()
                return False         # the breaking sample joins no chain
        elif slip:
            self._break_chain()
            return False             # first sighting is a break — drop it
        if self.last is not None:
            self.incs.append((cycles - pc) / (t - pt))
        self.samples.append((t, cycles, cn0))
        self.last = (t, cycles, corr)
        cut = t - self.window_s - 2.0 * GAP_S
        while self.samples and self.samples[0][0] < cut:
            self.samples.pop(0)
        del self.incs[:-max(1, len(self.samples))]
        return True

    def evaluate(self, min_lock_s, min_cn0, min_samples):
        """Fit the current window. Returns dict(slope_hz, sigma_hz, n) or
        None (window too short, lock younger than the window, or a weak
        C/N0 second inside it)."""
        if not self.samples:
            return None
        t_new = self.samples[-1][0]
        win = [s for s in self.samples if t_new - s[0] <= self.window_s]
        if len(win) < min_samples:
            return None
        if self.lock_s < min_lock_s:
            return None
        if any(cn0 is None or cn0 < min_cn0 for _, _, cn0 in win):
            return None
        fit = fit_drift([(t, c) for t, c, _ in win])
        if fit is None:
            return None
        slope, sigma, n = fit
        return {"slope_hz": slope, "sigma_hz": sigma, "n": n}


class P0bShadow:
    """P0b geometric-correction state, persisted across process_state
    calls: per-sat GeoCorrector + the corrected SatWindow, plus the
    jsonl convergence-evidence file.

    PROMOTED 2026-08-29 (acceptance hour: corrected 131/135 agreed to
    2.36e-7 ppm vs 2.12e-3 ppm uncorrected): the corrected window's fit
    IS the emitted observable — process_state emits it fail-closed (no
    corrected fit -> no row, no vote) and runs the calibrated scatter
    sigma on this window's fit history. The evidence file keeps
    appending corrected vs uncorrected ppm per GEO per cycle, verbatim
    — it is the promotion evidence chain and stays on."""

    def __init__(self, site_ecef, shadow_path=SHADOW_JSONL):
        self.site = site_ecef
        self.shadow_path = shadow_path
        self.correctors = {}        # (sys, prn) -> GeoCorrector
        self.windows = {}           # (sys, prn) -> corrected SatWindow

    def ingest(self, key, s, t, cycles, corr, window_s,
               min_lock_s, min_cn0, min_samples):
        """Feed one 1 Hz sbas report through the correction and fit the
        corrected window. Returns the diag fragment (p0b_* keys; the
        private "_ev" carries the corrected fit dict to process_state —
        it is the emitted series — and is popped before the diag is
        published)."""
        if self.site is None:
            return {"p0b_applied": False, "p0b_reason": "no-site"}
        gc = self.correctors.setdefault(key, GeoCorrector(self.site))
        cw = self.windows.setdefault(key, SatWindow(window_s))
        cw.window_s = window_s
        slip = bool(s.get("slip"))
        gc.update(s.get("sbas_geonav"), t, slip=slip)
        corrected, applied, reason = gc.correct(t, cycles)
        frag = {"p0b_applied": applied}
        if gc.geo is not None:
            frag["p0b_iodn"] = gc.geo.get("iodn")
        if not applied:
            frag["p0b_reason"] = reason
            if slip:
                # the chain broke and no correction re-anchors it: flush
                # the parallel window too (a slip sample only ever flushes,
                # it never joins a chain)
                cw.add(t, cycles, slip=True)
            return frag
        frag["p0b_age_s"] = round(abs(gc.last_dt), 1)
        if gc._msg_refresh_unix is not None:
            frag["p0b_msg_age_s"] = round(t - gc._msg_refresh_unix, 1)
        # only APPLIED samples join the corrected chain: a gated-out second
        # contributes nothing (SatWindow's own >5 s gap check flushes the
        # chain if the outage stretches — a gap in the correction is not
        # phase-continuous evidence)
        cw.add(t, corrected, slip=slip, cn0=s.get("cn0_proxy"),
               lock_s=s.get("lock_s"), corr=corr)
        ev = cw.evaluate(min_lock_s, min_cn0, min_samples)
        if ev is not None:
            # same register add-back convention as the published rows, so
            # corrected and uncorrected ppm are directly comparable
            frag["p0b_ppm"] = round(ev["slope_hz"] / L1_HZ * 1e6 + corr, 9)
            frag["p0b_sigma_ppm"] = round(ev["sigma_hz"] / L1_HZ * 1e6, 9)
            frag["_ev"] = ev
        return frag

    def break_chain(self, key):
        """Fail closed for one malformed/missing channel epoch."""
        if key in self.windows:
            self.windows[key]._break_chain()
        if key in self.correctors:
            self.correctors[key].offset_cycles = 0.0

    def append(self, line):
        """One evidence line per sbas sat per cycle. Shadow I/O must never
        take the published path down — log and drop on failure."""
        try:
            with open(self.shadow_path, "a") as f:
                f.write(json.dumps(line) + "\n")
        except Exception as e:
            log(f"p0b shadow append error: {e}")


def row(band, name, ppm, sigma, epoch, sats, extra=None,
        kind="ClockDriftPpm"):
    r = {
        "band": band,
        "name": name,
        "kind": kind,
        "value": round(ppm, 9), "sigma": round(sigma, 9),
        "ref_hz": L1_HZ, "epoch": round(epoch, 2),
        "sats": sats,
        "anchor": "Pro live track @ 1568.25 MHz",
        "ns_per_s": round(ppm * 1000.0, 6),
        "m_per_s": round(ppm * 1e-6 * C_MPS, 6),
    }
    if extra:
        r.update(extra)
    return r


def process_state(state, windows, window_s, min_lock_s, min_cn0, min_samples,
                  p0b=None):
    """Ingest one tracker state file. Returns (rows, diag): sources rows
    (per-GEO + consensus) and per-sat diagnostics for the phase_drift
    key. `windows` persists across calls (keyed by (sys, prn)). `p0b` is
    the optional P0bShadow state — when given, sbas reports ALSO feed the
    shadow geometric correction (diag-only p0b_* keys + evidence jsonl;
    the published rows are computed exactly as without it)."""
    # The discipline cache is historical intent, not applied truth (same
    # fix as tracker_producer review round 6): in SHADOW mode nothing was
    # written to hardware, so the measured slope carries NO register
    # offset and the cached correction must NOT be added back. Use it
    # only when this live_radio process verifiably wrote it (actuate +
    # corr_applied); otherwise the applied correction is 0.
    if not isinstance(state, dict):
        return [], {"input": {"ok": False, "reason": "state-not-object"}}

    def number(value):
        if type(value) not in (int, float):
            raise ValueError("not a JSON number")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("nonfinite")
        return value

    disc_d = state.get("discipline")
    if disc_d is None:
        disc_d = {}
    if not isinstance(disc_d, dict):
        return [], {"input": {"ok": False,
                               "reason": "discipline-not-object"}}
    corr_applied = (disc_d.get("actuate") is True
                    and disc_d.get("corr_applied") is True)
    try:
        corr = number(disc_d["correction_ppm"]) if corr_applied else 0.0
    except (KeyError, TypeError, ValueError, OverflowError):
        return [], {"input": {"ok": False,
                               "reason": "invalid-applied-correction"}}

    tracker = state.get("tracker")
    if tracker is None:
        tracker = {}
    if not isinstance(tracker, dict):
        return [], {"input": {"ok": False, "reason": "tracker-not-object"}}
    sats = tracker.get("sats")
    if sats is None:
        sats = []
    if not isinstance(sats, list):
        return [], {"input": {"ok": False, "reason": "sats-not-array"}}
    try:
        file_epoch = (number(state["epoch"]) if "epoch" in state
                      else time.time())
    except (TypeError, ValueError, OverflowError):
        return [], {"input": {"ok": False, "reason": "invalid-state-epoch"}}

    def break_key(key):
        if key in windows:
            windows[key]._break_chain()
        if p0b is not None:
            p0b.break_chain(key)

    votes = []
    diag = {}
    for index, s in enumerate(sats):
        if not isinstance(s, dict):
            diag[f"invalid[{index}]"] = {
                "ok": False, "reason": "sat-not-object"
            }
            continue
        sysname = s.get("sys")
        prn = s.get("prn")
        cycles = s.get("carrier_cycles")
        if sysname is None or prn is None or cycles is None:
            continue               # pre-dcfcfaa row: no carrier phase
        key = (sysname, prn)
        reason = None
        try:
            if not isinstance(sysname, str) or not sysname:
                raise ValueError("invalid-system")
            if type(prn) is not int or prn < 0:
                raise ValueError("invalid-prn")
            key = (sysname, prn)
            cycles = number(cycles)
            t = number(s["epoch"]) if s.get("epoch") is not None else file_epoch
            cn0 = number(s["cn0_proxy"])
            lock_s = number(s["lock_s"])
            slip = s.get("slip", False)
            if type(slip) is not bool:
                raise ValueError("invalid-slip")
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            reason = str(exc) or "invalid-satellite-field"
        if reason is not None:
            if (isinstance(sysname, str) and sysname
                    and type(prn) is int and prn >= 0):
                key = (sysname, prn)
                break_key(key)
                label = f"{sysname} {prn}"
            else:
                label = f"invalid[{index}]"
            diag[label] = {"ok": False, "reason": reason}
            continue
        key = (sysname, prn)
        w = windows.setdefault(key, SatWindow(window_s))
        w.window_s = window_s
        w.add(t, cycles, slip=slip, cn0=cn0, lock_s=lock_s, corr=corr)
        if sysname != "sbas":
            continue               # MEO slope is orbit-dominated; GEOs only
        # P0b (promoted 2026-08-29): feed the corrected chain; its fit is
        # the emitted observable below. A correction-path failure logs and
        # degrades to "no corrected fit" -> the fail-closed gate drops the
        # sat; it may not kill the OTHER sats' rows.
        frag = {}
        if p0b is not None:
            try:
                frag = p0b.ingest(key, s, t, cycles, corr, window_s,
                                  min_lock_s, min_cn0, min_samples)
            except Exception as e:
                log(f"p0b shadow error {sysname} {prn}: {e}")
                frag = {}
        ev = w.evaluate(min_lock_s, min_cn0, min_samples)
        # the UNCORRECTED fit is evidence only now (diag uncorr_ppm, row
        # extra, shadow jsonl) — P0b made the corrected fit the observable
        uncorr_ppm = uncorr_sig = None
        if ev is not None:
            uncorr_ppm = ev["slope_hz"] / L1_HZ * 1e6 + corr
            uncorr_sig = ev["sigma_hz"] / L1_HZ * 1e6
        # emission series, FAIL-CLOSED: with the corrector active this is
        # the CORRECTED fit — no corrected fit (no-geonav / stale / ura /
        # bad-geonav / post-slip refill) emits NO row and NO vote: a GEO
        # row without the motion correction is not a clock observable
        # (that was the whole point of P0b; never fall back to publishing
        # the uncorrected value). p0b=None keeps the pre-P0b uncorrected
        # emission for the legacy unit tests of that machinery —
        # production always passes a P0bShadow.
        if p0b is not None:
            emit_ev = frag.pop("_ev", None)
            emit_w = p0b.windows.get(key)
        else:
            emit_ev, emit_w = ev, w
        if emit_ev is None:
            d = {"ok": False, "lock_s": w.lock_s,
                 "n": len(emit_w.samples) if emit_w is not None
                 else len(w.samples),
                 "phase_generation": (emit_w.generation if emit_w is not None
                                      else w.generation)}
            if uncorr_ppm is not None:
                d["uncorr_ppm"] = round(uncorr_ppm, 9)
            d.update(frag)
            diag[f"{sysname} {prn}"] = d
            continue
        ppm = emit_ev["slope_hz"] / L1_HZ * 1e6 + corr
        sig_ppm = emit_ev["sigma_hz"] / L1_HZ * 1e6
        # disjoint-window fit history for the calibrated scatter sigma:
        # record a fit only once the window has fully advanced — fits that
        # share samples are not independent draws, and their scatter would
        # understate just like the per-window OLS sigma does. PROMOTED:
        # runs on the CORRECTED window when the corrector is active (the
        # uncorrected fit_hist is no longer needed for emission).
        # Do not seed the cross-window scatter from the first merely
        # min_samples-long fit.  It is not comparable with the later full
        # windows and would make the history look mature one interval too
        # early.  A history entry requires a full continuous window in the
        # current phase generation as well as non-overlap with the previous
        # entry.
        full_window = (bool(emit_w.samples)
                       and t - emit_w.samples[0][0] >= window_s)
        if full_window and t - emit_w.last_fit_t >= window_s:
            emit_w.fit_hist.append(emit_ev["slope_hz"])
            del emit_w.fit_hist[:-12]
            emit_w.last_fit_t = t
        sc = scatter_sigma(emit_w.fit_hist)
        sc_ppm = sc / L1_HZ * 1e6 if sc else None
        # round-16 review: the EMITTED sigma must be the calibrated one.
        # Once >=5 disjoint windows exist, every GEO row and consensus
        # weight uses max(OLS, cross-window scatter) — the OLS value alone
        # understates 6-20x on live AR(1) phase noise. Before 5 windows,
        # publish the MC-calibrated provisional 5x-OLS inflation (measured
        # 2026-08-28: 17% 1-sigma coverage at face value), explicitly
        # flagged — never the optimistic raw OLS number.
        if sc_ppm is not None:
            sig_emit, sig_prov = max(sig_ppm, sc_ppm), False
        else:
            sig_emit, sig_prov = 5.0 * sig_ppm, True
        votes.append((ppm, sig_emit, prn, t,
                      round(uncorr_ppm, 9) if uncorr_ppm is not None else None,
                      emit_w.generation))
        diag[f"{sysname} {prn}"] = {
            "ok": True, "n": emit_ev["n"],
            "phase_generation": emit_w.generation,
            "slope_hz": round(emit_ev["slope_hz"], 6),
            "fit_sigma_ppm": round(sig_ppm, 9),
            "scatter_sigma_ppm": round(sc_ppm, 9) if sc_ppm else None,
            "scatter_n": len(emit_w.fit_hist),
            "emitted_sigma_ppm": round(sig_emit, 9),
            "sigma_provisional": sig_prov,
            "ppm": round(ppm, 9),
        }
        if p0b is not None:
            # ppm above IS the corrected, emitted value; keep the raw one
            d = diag[f"{sysname} {prn}"]
            d["uncorr_ppm"] = (round(uncorr_ppm, 9)
                               if uncorr_ppm is not None else None)
            d["uncorr_sigma_ppm"] = (round(uncorr_sig, 9)
                                     if uncorr_sig is not None else None)
        if frag:
            diag[f"{sysname} {prn}"].update(frag)
            if uncorr_ppm is not None:
                # convergence evidence: corrected vs uncorrected ppm, one
                # line per GEO per cycle — the promotion evidence chain,
                # kept appending verbatim after promotion
                p0b.append({"epoch": round(t, 2), "prn": prn,
                            "p0b_ppm": frag["p0b_ppm"],
                            "p0b_sigma_ppm": frag["p0b_sigma_ppm"],
                            "uncorr_ppm": round(uncorr_ppm, 9),
                            "iodn": frag.get("p0b_iodn"), "n": emit_ev["n"],
                            "phase_generation": emit_w.generation})
    p0b_tag = " · P0b" if p0b is not None else ""
    rows = []
    for ppm, sig_ppm, prn, t, uc, generation in votes:
        # components are NOT "ClockDriftPpm": one instrument must vote
        # once in series_producer's cross-producer consensus — N near-
        # identical per-sat rows from the same phase chain would outvote
        # every independent path. The component kind keeps the rows
        # visible in the merged sources table without voting/charting.
        extra = {"corr_applied": corr_applied}
        if p0b is not None:
            # the emitted value is the GEO-corrected observable; the raw
            # range-rate-contaminated value stays on the row as evidence
            extra.update({"p0b": True, "uncorr_ppm": uc})
        extra["phase_generation"] = generation
        rows.append(row(f"L1 / WAAS {prn} (phase)",
                        f"WAAS PRN {prn} carrier-phase slope {window_s:.0f} s "
                        f"+ corr register · Pro+AA.250{p0b_tag}",
                        ppm, sig_ppm, t, [f"PRN {prn}"],
                        # explicit: was a hardware correction added back?
                        # (shadow mode -> False, value is the raw residual)
                        extra=extra,
                        kind="ClockDriftPpmComponent"))
    if votes:
        med, sig = weighted_median([(v, s) for v, s, _, _, _, _ in votes])
        t = max(t for _, _, _, t, _, _ in votes)
        # Consensus is a component too (systems review 2026-08-25 #6):
        # observe-only until the correlated-residual uncertainty lands;
        # it must not double-vote the radio it shares with the WAAS code
        # row. P0b promotion changed the observable, not the kind.
        rows.append(row(MY_BAND,
                        f"WAAS GEO carrier-phase consensus ({len(votes)} sats, "
                        f"{window_s:.0f} s slope) + corr register · Pro+AA.250 "
                        f"· observe-only{p0b_tag}",
                        med, sig, t,
                        [f"PRN {prn}" for _, _, prn, _, _, _ in votes],
                        extra={"n_sats": len(votes),
                               "corr_applied": corr_applied,
                               "phase_generations": {
                                   str(prn): generation
                                   for _, _, prn, _, _, generation in votes
                               }},
                        kind="ClockDriftPpmComponent"))
    return rows, diag


def publish(rows, diag, now):
    state = {
        "epoch": round(now, 2),
        "ttl_s": 30,
        "phase_drift": diag,
    }
    if rows:
        state["sources"] = rows
    tmp = STATE + ".phase_drift.tmp"
    json.dump(state, open(tmp, "w"), indent=1)
    os.replace(tmp, STATE)


def main():
    ap = argparse.ArgumentParser(description="carrier-phase clock-drift producer")
    ap.add_argument("--window-s", type=float, default=WINDOW_S,
                    help="fit window, seconds (clamped to [30, 60]; "
                         f"default {WINDOW_S:.0f})")
    args = ap.parse_args()
    window_s = min(60.0, max(30.0, args.window_s))
    min_lock_s = window_s          # locked for the whole window
    min_samples = max(10, int(MIN_SAMPLES_FRAC * window_s))
    log(f"phase drift producer starting — {window_s:.0f} s window, "
        f"min {min_samples} samples, cn0>={MIN_CN0:.0f} dB, GEOs only")
    windows = {}
    p0b = P0bShadow(load_site_ecef())
    if p0b.site is None:
        log("p0b shadow DISABLED — site anchor unreadable (fail-closed)")
    else:
        log("p0b PROMOTED — GEO-corrected values are the emitted observable")
    last_mtime = 0.0
    last_pub = 0.0
    while True:
        try:
            mtime = os.path.getmtime(TRACKER)
        except OSError:
            time.sleep(1.0)
            continue
        now = time.time()
        if mtime != last_mtime:
            last_mtime = mtime
            try:
                with open(TRACKER) as f:
                    state = json.load(f)
            except Exception:
                time.sleep(0.2)    # mid-replace read; retry next tick
                continue
            try:
                rows, diag = process_state(state, windows, window_s,
                                           min_lock_s, MIN_CN0, min_samples,
                                           p0b)
            except Exception as e:
                # Last-resort containment: malformed/unexpected tracker state
                # must age this producer out, never terminate it or preserve a
                # frozen vote. Field-level failures are handled inside
                # process_state; this guards programming/shape surprises.
                log(f"tracker state rejected: {e}")
                rows = []
                diag = {"input": {"ok": False,
                                   "reason": "state-processing-error"}}
            try:
                publish(rows, diag, now)
                last_pub = now
                nok = sum(1 for d in diag.values() if d.get("ok"))
                if rows:
                    cons = [r for r in rows if r["band"] == MY_BAND]
                    if cons:
                        log(f"{nok} GEOs — consensus {cons[0]['value']:+.9f} ppm "
                            f"(sigma {cons[0]['sigma']:.2e})")
            except Exception as e:
                log(f"publish error: {e}")
        elif now - last_pub >= 5.0:
            # tracker stalled: heartbeat with NO rows so the merged panel
            # ages/drops this source instead of voting a frozen value
            try:
                publish([], {}, now)
                last_pub = now
            except Exception as e:
                log(f"heartbeat error: {e}")
        time.sleep(0.25)


if __name__ == "__main__":
    main()
