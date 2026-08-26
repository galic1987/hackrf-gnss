#!/usr/bin/env python3
"""Tests for the position ENU-in-time panel section + history watcher.

Two halves, no live observations touched (HACKRF_GNSS_OBS sandbox):

1. JS math check: extract geodeticToEcef / ecefToEnu / fixToEnu /
   medianOf / madLimits / posEnuScale from web/sync.html and run them
   under macOS JavaScriptCore (osascript -l JavaScript). ENU results are
   compared against an independent python WGS84 implementation:
   - the surveyed site itself must map to (0, 0, 0)
   - several offset cases must match python to millimetres
   - the MAD scale must ignore an injected 500 km outlier (axis stays
     in metres) while the outlier point falls outside the axis (so the
     panel marks it at the edge instead of stretching).

2. Watcher check: position_watch.cycle() against a fake
   state.position.json — appends exactly one JSONL line per new trusted
   epoch (untrusted/missing-trust fixes route to the diagnostic log),
   rewrites state.position_history.json every cycle (heartbeat), and a
   corrupt state file raises (main()'s guard keeps the loop alive).
"""
import json
import math
import os
import re
import subprocess
import sys
import tempfile

CRATE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SYNC_HTML = f"{CRATE}/web/sync.html"
sys.path.insert(0, f"{CRATE}/scripts")

SITE = {"lat": 39.0032, "lon": -77.6058, "alt_m": 20.0}

JS_FUNCS = ["geodeticToEcef", "ecefToEnu", "fixToEnu",
            "medianOf", "madLimits", "posEnuScale"]


def extract_js():
    src = open(SYNC_HTML).read()
    out = []
    for name in JS_FUNCS:
        m = re.search(rf"function {name}\(.*?\n\}}", src, re.S)
        assert m, f"{name} not found in sync.html"
        out.append(m.group(0))
    return "\n".join(out)


def run_js(expr):
    js = extract_js() + f"\nvar POS_SITE = {json.dumps(SITE)};\nJSON.stringify({expr});"
    r = subprocess.run(["osascript", "-l", "JavaScript", "-e", js],
                       capture_output=True, text=True)
    assert r.returncode == 0, f"osascript failed: {r.stderr}"
    return json.loads(r.stdout.strip().strip('"')) if r.stdout.strip().startswith('"') \
        else json.loads(r.stdout.strip())


# --- independent python reference -------------------------------------------

def geodetic_to_ecef(lat_deg, lon_deg, h_m):
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = f * (2 - f)
    la, lo = math.radians(lat_deg), math.radians(lon_deg)
    n = a / math.sqrt(1 - e2 * math.sin(la) ** 2)
    return ((n + h_m) * math.cos(la) * math.cos(lo),
            (n + h_m) * math.cos(la) * math.sin(lo),
            (n * (1 - e2) + h_m) * math.sin(la))


def enu(lat, lon, h_m):
    ref = geodetic_to_ecef(SITE["lat"], SITE["lon"], SITE["alt_m"])
    p = geodetic_to_ecef(lat, lon, h_m)
    la, lo = math.radians(SITE["lat"]), math.radians(SITE["lon"])
    dx, dy, dz = p[0] - ref[0], p[1] - ref[1], p[2] - ref[2]
    e = -math.sin(lo) * dx + math.cos(lo) * dy
    n_ = (-math.sin(la) * math.cos(lo) * dx - math.sin(la) * math.sin(lo) * dy
          + math.cos(la) * dz)
    u = (math.cos(la) * math.cos(lo) * dx + math.cos(la) * math.sin(lo) * dy
         + math.sin(la) * dz)
    return e, n_, u


def test_enu():
    # site itself -> origin
    js = run_js(f"fixToEnu({{lat:{SITE['lat']},lon:{SITE['lon']},alt_km:{SITE['alt_m']/1000}}}, POS_SITE)")
    assert all(abs(v) < 1e-6 for v in js), f"site not at origin: {js}"

    cases = [
        (SITE["lat"] + 0.01, SITE["lon"], SITE["alt_m"]),          # ~1.1 km north
        (SITE["lat"], SITE["lon"] - 0.02, SITE["alt_m"]),          # ~1.7 km west
        (SITE["lat"], SITE["lon"], SITE["alt_m"] + 50.0),          # 50 m up
        (SITE["lat"] - 0.007, SITE["lon"] + 0.013, SITE["alt_m"] - 120.0),
    ]
    for lat, lon, h in cases:
        js = run_js(f"fixToEnu({{lat:{lat},lon:{lon},alt_km:{h/1000}}}, POS_SITE)")
        py = enu(lat, lon, h)
        for j, p, name in zip(js, py, "ENU"):
            assert abs(j - p) < 1e-3, f"{name}: js {j} vs py {p} (case {lat},{lon},{h})"
    # sanity on the offset magnitudes: +0.01 deg lat ≈ 1.11 km North
    n = enu(SITE["lat"] + 0.01, SITE["lon"], SITE["alt_m"])[1]
    assert 1100 < n < 1120, n
    u = enu(SITE["lat"], SITE["lon"], SITE["alt_m"] + 50.0)[2]
    assert abs(u - 50.0) < 0.01, u
    print("ENU conversion: site maps to origin; 4 offset cases match python to <1 mm")


def test_mad_scale():
    # 35 plausible fixes (±5 m jitter) + one 500 km outlier, per component
    vals = [3.0 * math.sin(i) for i in range(35)] + [500000.0]
    lo, hi = run_js(f"posEnuScale([[{','.join(str(v) for v in vals)}]])")
    axis = (lo, hi)
    assert -50 < lo and hi < 50, f"axis stretched by outlier: [{lo}, {hi}]"
    assert vals[-1] > hi, "outlier must fall outside the axis (edge-marked)"
    # degenerate: all identical values -> ±1 m fallback, no crash
    lo, hi = run_js("posEnuScale([[7,7,7,7]])")
    assert lo == 6 and hi == 8, (lo, hi)
    print(f"MAD scale: 500 km outlier ignored (axis [{axis[0]:.1f}, {axis[1]:.1f}] m), "
          "outlier outside axis; degenerate input safe")


def test_render_outlier():
    """Full render path: drawPosEnu/drawPosIsx against a synthetic history
    (35 plausible fixes + one 500 km blip) with a stubbed DOM under JXA.
    Confirms the axis labels stay metre-scale, the outlier is edge-marked
    (red ✕), and the isx sub-line renders for mixed fixes."""
    src = open(SYNC_HTML).read()
    funcs = "\n".join(
        re.search(rf"function {n}\(.*?\n\}}", src, re.S).group(0)
        for n in JS_FUNCS + ["drawPosEnu", "drawPosIsx", "posGlidePos"])
    vars_ = "\n".join(
        re.search(rf"var {n} = .*?;", src).group(0)
        for n in ["POS_SITE", "ENU_COLORS", "ENU_NAMES", "MIXED_EDGE"])
    harness = vars_ + "\nvar POS_ANIM = null; var POS_GLIDE_S = 2.0;\n" + funcs + r"""
var window = { devicePixelRatio: 1 };
function mkCtx() { return { calls: [], strokeStyle: "", fillStyle: "", lineWidth: 1, font: "",
  setTransform: function () {}, clearRect: function () {}, beginPath: function () {},
  moveTo: function () {}, lineTo: function () {},
  arc: function (x, y, r) { this.calls.push(["arc", x, y]); },
  fillRect: function () {},
  fill: function () { this.calls.push(["fill", this.fillStyle]); },
  stroke: function () { this.calls.push(["stroke", this.strokeStyle]); },
  fillText: function (t, x, y) { this.calls.push(["text", t]); } }; }
var __els = {};
var document = { getElementById: function (id) {
  if (!__els[id]) __els[id] = { clientWidth: 800, clientHeight: id === "posisx" ? 56 : 150,
    width: 0, height: 0, style: {}, _ctx: mkCtx(),
    getContext: function () { return this._ctx; }, innerHTML: "", textContent: "" };
  return __els[id]; } };
var fixes = [], now = Date.now() / 1000;
for (var i = 0; i < 35; i++) {
  fixes.push({ epoch: now - (34 - i) * 300,
    lat: 39.0032 + 0.00002 * Math.sin(i * 2.3), lon: -77.6058 + 0.00002 * Math.cos(i * 1.7),
    alt_km: 0.020 + 0.000003 * Math.sin(i),
    mode: i % 7 === 0 ? "3D(mixed GPS+BDS)" : "3D",
    gate: i % 5 === 0 ? "ungated — exact solve, unverifiable" : "redundant",
    gdop: 2.5, n_sats: 6, isx_km: i % 7 === 0 ? 12 + 0.5 * Math.sin(i) : null });
}
// injected 500 km blip (north), gated — must be marked, not stretch the axis
fixes.push({ epoch: now - 150, lat: 39.0032 + 4.5, lon: -77.6058, alt_km: 0.02,
  mode: "3D", gate: "redundant", gdop: 3, n_sats: 5, isx_km: null });
drawPosEnu(fixes); drawPosIsx(fixes);
var calls = __els["posenu"]._ctx.calls;
JSON.stringify({
  legend: __els["posenulegend"].innerHTML,
  texts: calls.filter(function (c) { return c[0] === "text"; }).map(function (c) { return c[1]; }),
  redX: calls.some(function (c) { return c[0] === "stroke" && c[1] === "#ef9a9a"; }),
  isxLegend: __els["posisxlegend"].innerHTML,
  isxShown: __els["posisx"].style.display });
"""
    r = subprocess.run(["osascript", "-l", "JavaScript", "-e", harness],
                       capture_output=True, text=True)
    assert r.returncode == 0, f"osascript failed: {r.stderr}"
    res = json.loads(r.stdout.strip())
    axis_vals = [abs(float(t.split(" m")[0])) for t in res["texts"] if t.endswith(" m")]
    assert axis_vals and max(axis_vals) < 100, f"axis stretched: {res['texts']}"
    assert res["redX"], "outlier not edge-marked (no red ✕ stroke)"
    assert "outlier(s) at edge" in res["legend"], res["legend"]
    assert res["isxShown"] == "block" and "GPS−BDS" in res["isxLegend"], res["isxLegend"]
    print(f"render: axis labels {sorted(set(res['texts']))} — metre-scale; "
          "500 km outlier edge-marked with red ✕; isx sub-line rendered")


def test_watcher():
    import position_watch
    with tempfile.TemporaryDirectory() as d:
        os.environ["HACKRF_GNSS_OBS"] = d
        # rebind the module's paths into the sandbox
        position_watch.OBS = d
        position_watch.STATE_IN = f"{d}/state.position.json"
        position_watch.HISTORY = f"{d}/position_history.jsonl"
        position_watch.HISTORY_DIAG = f"{d}/position_history_diagnostic.jsonl"
        position_watch.STATE_OUT = f"{d}/state.position_history.json"
        # the anchor the watcher re-reads on every state write
        json.dump(SITE, open(f"{d}/site.json", "w"))

        import time
        ep0 = time.time() - 600          # inside the 3 h window
        trusted = {"lat": SITE["lat"], "lon": SITE["lon"],
                   "alt_km": 0.02, "mode": "3D(mixed GPS+BDS)",
                   "gate": "redundant", "gdop": 2.1, "n_sat": 6,
                   "isx_km": 12.34, "residual_rms_m": 3.2,
                   "geometry_redundant": True,
                   "plausibility_pass": True,
                   "trusted_for_history": True,
                   "source": "test-fixture"}

        def write(doc):
            json.dump(doc, open(position_watch.STATE_IN, "w"))

        def nlines(path):
            return len(open(path).read().strip().splitlines())

        # trusted fix in "position" (round-13 law: position is trusted-only)
        write({"epoch": ep0, "position": dict(trusted, epoch=ep0)})
        seen, seen_cand, seen_diag = position_watch.seed_cursors()
        assert (seen, seen_cand, seen_diag) == (None, None, None)
        seen, seen_cand, seen_diag = position_watch.cycle(seen, seen_cand, seen_diag)
        assert seen == ep0
        lines = open(position_watch.HISTORY).read().strip().splitlines()
        assert len(lines) == 1
        rec = json.loads(lines[0])
        assert rec["n_sats"] == 6 and rec["isx_km"] == 12.34
        # trust fields + quality/provider metadata carried through ingest
        assert rec["trusted_for_history"] is True
        assert rec["residual_rms_m"] == 3.2 and rec["source"] == "test-fixture"
        st = json.load(open(position_watch.STATE_OUT))
        assert len(st["position_history"]) == 1 and st["site"]["lat"] == SITE["lat"]

        # same epoch again: heartbeat rewrites state, no duplicate history
        seen, seen_cand, seen_diag = position_watch.cycle(seen, seen_cand, seen_diag)
        assert nlines(position_watch.HISTORY) == 1

        # new epoch appends
        write({"epoch": ep0 + 300.0, "position": dict(trusted, epoch=ep0 + 300.0)})
        seen, seen_cand, seen_diag = position_watch.cycle(seen, seen_cand, seen_diag)
        assert nlines(position_watch.HISTORY) == 2

        # round-13: a plausible-but-EXACT solve publishes as
        # "position_candidate" — diagnostic history only, the trusted
        # history must not grow, and the preserved trusted "position"
        # (older epoch) must NOT be re-ingested
        cand = {"lat": SITE["lat"], "lon": SITE["lon"], "alt_km": 0.03,
                "mode": "3D", "gate": "ungated — exact solve, unverifiable",
                "gdop": 4.2, "n_sat": 4, "residual_rms_m": 0.0,
                "geometry_redundant": False, "plausibility_pass": True,
                "trusted_for_history": False, "source": "test-fixture"}
        write({"epoch": ep0 + 600.0,
               "position": dict(trusted, epoch=ep0 + 300.0),
               "position_candidate": dict(cand, epoch=ep0 + 600.0)})
        seen, seen_cand, seen_diag = position_watch.cycle(seen, seen_cand, seen_diag)
        assert nlines(position_watch.HISTORY) == 2
        dlines = open(position_watch.HISTORY_DIAG).read().strip().splitlines()
        assert len(dlines) == 1
        assert json.loads(dlines[0])["trusted_for_history"] is False

        # a plausibility-FAILING solve publishes only as
        # "position_diagnostic" — same diagnostic-history route
        write({"epoch": ep0 + 900.0,
               "position": dict(trusted, epoch=ep0 + 300.0),
               "position_diagnostic": dict(cand, epoch=ep0 + 900.0,
                                           residual_rms_m=179.4,
                                           plausibility_pass=False)})
        seen, seen_cand, seen_diag = position_watch.cycle(seen, seen_cand, seen_diag)
        assert nlines(position_watch.HISTORY) == 2
        assert nlines(position_watch.HISTORY_DIAG) == 2

        # restart dedupe (round-13 9b): fresh cursors via the production
        # seeding path — the still-current doc appends NOTHING again: not
        # the failing solve (the diagnostic tail cursor covers it) and not
        # the preserved trusted "position" (already the trusted tail; the
        # main cursor seeds from the max of BOTH tails)
        r_seen, r_cand, r_diag = position_watch.seed_cursors()
        assert r_seen == ep0 + 900.0
        position_watch.cycle(r_seen, r_cand, r_diag)
        assert nlines(position_watch.HISTORY) == 2
        assert nlines(position_watch.HISTORY_DIAG) == 2

        # defensive: a fix in "position" with a MISSING trust field counts
        # false and routes to the diagnostic history (round-13 producers
        # keep untrusted solves out of "position" entirely)
        legacy = dict(trusted, epoch=ep0 + 1200.0)
        del legacy["trusted_for_history"]              # missing == false
        write({"epoch": ep0 + 1200.0, "position": legacy})
        position_watch.cycle(seen, seen_cand, seen_diag)
        assert nlines(position_watch.HISTORY) == 2
        assert nlines(position_watch.HISTORY_DIAG) == 3

        open(position_watch.STATE_IN, "w").write("{corrupt")
        try:
            position_watch.cycle(seen, seen_cand, seen_diag)
            raise AssertionError("corrupt state did not raise")
        except json.JSONDecodeError:
            pass
        print("watcher: one JSONL line per new trusted epoch, candidate/"
              "failing -> diagnostic log, restart-dedupe, heartbeat state "
              "write, corrupt-state survivable")


if __name__ == "__main__":
    test_enu()
    test_mad_scale()
    test_render_outlier()
    test_watcher()
    print("all position-chart tests passed")
