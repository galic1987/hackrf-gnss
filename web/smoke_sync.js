// Regression smoke for sync.html — run: `node web/smoke_sync.js` (live http://localhost:8090/api/sync)
// or offline against a snapshot: `node web/smoke_sync.js web/test_fixture.json`. Exits nonzero on any failure.
//
// Extracts the page's <script>, runs it in a vm with a minimal DOM stub
// (canvas ops counted per canvas id; any non-finite draw argument is a
// violation), then drives render(live/empty/error/partial), the guided tour,
// 3D replay, the current-fix glide and the sky-dome lerp against real data.
const fs = require("fs"), path = require("path"), vm = require("vm");

const HTML_PATH = path.join(__dirname, "sync.html");
const API_URL = "http://localhost:8090/api/sync";

function loadData() {
  const arg = process.argv[2];
  if (arg) {
    console.log("fixture: " + arg);
    return Promise.resolve(JSON.parse(fs.readFileSync(arg, "utf8")));
  }
  console.log("live: " + API_URL);
  return fetch(API_URL).then(function (r) {
    if (!r.ok) throw new Error("HTTP " + r.status);
    return r.json();
  });
}

function main(live) {
  const html = fs.readFileSync(HTML_PATH, "utf8");
  const m = html.match(/<script>([\s\S]*)<\/script>/);
  if (!m) throw new Error("no <script> block in " + HTML_PATH);
  const src = m[1];

  const drawCounts = {}, nanViol = [];
  let anon = 0;
  function mkCtx(id) {
    const rec = (name, args) => {
      drawCounts[id] = (drawCounts[id] || 0) + 1;
      for (const a of args)
        if (typeof a === "number" && !isFinite(a)) nanViol.push(id + "." + name + "(" + a + ")");
    };
    return new Proxy({}, {
      get: (t, p) => {
        if (p === "measureText") return (s) => ({ width: String(s).length * 6 });
        return (...args) => rec(String(p), args);
      },
      set: () => true
    });
  }
  function mkEl(id) {
    return {
      id, children: [], style: {}, dataset: {},
      innerHTML: "", textContent: "", value: "", tabIndex: 0, className: "",
      clientWidth: 900, clientHeight: 200, offsetWidth: 100, offsetHeight: 40,
      offsetParent: { visible: true }, width: 0, height: 0,
      classList: { add() {}, remove() {}, contains() { return false; } },
      setAttribute() {}, getAttribute() { return null; }, hasAttribute() { return false; },
      appendChild(c) { this.children.push(c); return c; },
      replaceChild() {}, removeChild() {},
      addEventListener() {}, removeEventListener() {},
      getBoundingClientRect() { return { left: 10, top: 10, right: 300, bottom: 120, width: 290, height: 110 }; },
      scrollIntoView() {}, closest() { return null; }, contains() { return false; },
      getContext() { this._ctx = this._ctx || mkCtx(id || "anon" + (anon++)); return this._ctx; },
      querySelectorAll() { return []; }, querySelector() { return null; },
      focus() {}, blur() {}
    };
  }
  const els = {};
  const documentStub = {
    getElementById(id) { return els[id] || (els[id] = mkEl(id)); },
    createElement(tag) { return mkEl(tag + "_" + (anon++)); },
    createTextNode(t) { return { nodeValue: t }; },
    createDocumentFragment() { return mkEl("frag"); },
    createTreeWalker() { return { nextNode() { return null; } }; },
    querySelectorAll() { return []; },
    querySelector(sel) { return els[sel] || (els[sel] = mkEl(sel)); },
    addEventListener() {}, body: mkEl("body")
  };
  let rafCb = null;
  const ctxGlobal = {
    document: documentStub,
    fetch: () => Promise.resolve({ json: () => Promise.resolve(live) }),
    setInterval: () => 0, clearInterval() {},
    setTimeout: () => 0, clearTimeout() {},
    requestAnimationFrame: (cb) => { rafCb = cb; return 1; },
    localStorage: { getItem: () => "1", setItem() {}, removeItem() {} },
    NodeFilter: { SHOW_TEXT: 4, FILTER_REJECT: 2, FILTER_ACCEPT: 1 },
    console
  };
  ctxGlobal.window = { devicePixelRatio: 1, innerWidth: 1200, innerHeight: 800,
                       addEventListener() {}, console };
  vm.createContext(ctxGlobal);
  vm.runInContext(src, ctxGlobal, { filename: "sync.html<script>" });
  const g = ctxGlobal;

  let t = 1000;
  function frames(n, stepMs) {
    for (let i = 0; i < n; i++) {
      const cb = rafCb; rafCb = null;
      if (!cb) throw new Error("animLoop did not re-register rAF");
      cb(t += (stepMs || 16.7));
    }
  }
  const errors = [];
  function step(name, fn) {
    try { fn(); console.log("OK   " + name); }
    catch (e) { errors.push(name); console.log("FAIL " + name + " — " + (e && e.message || e)); }
  }

  step("render(data)", () => g.render(live));
  step("frames x5 after render", () => frames(5));
  step("render({}) empty", () => g.render({}));
  step("frames x3 on empty state", () => frames(3));
  step("render({error})", () => g.render({ error: "smoke test" }));
  step("render(data) again", () => g.render(live));
  // regression: partial producer write — position exists but residual_rms_m
  // is missing (used to throw at pos.residual_rms_m.toFixed)
  step("render(partial position: no residual_rms_m)", () => {
    const pp = JSON.parse(JSON.stringify(live));
    pp.position = { lat: 39.0035, lon: -77.6053, alt_km: 0.02, epoch: Date.now() / 1000,
                    mode: "3D", gate: "redundant", n_sat: 6, gdop: 4.2, source: "smoke-partial" };
    g.render(pp);
    if (els["position"].innerHTML.indexOf("partial write") < 0)
      throw new Error("position card did not render the partial-write pill");
  });
  step("render(data) after partial", () => g.render(live));

  // glide trigger: a newer, shifted fix (+ shifted sky) starts the lerps
  const live2 = JSON.parse(JSON.stringify(live));
  if (!Array.isArray(live2.position_history) || live2.position_history.length < 2) {
    const nowS = Date.now() / 1000;
    live2.position_history = [
      { lat: 39.0035, lon: -77.6053, alt_km: 0.02, epoch: nowS - 600, gate: "redundant", mode: "3D", n_sats: 6 },
      { lat: 39.00352, lon: -77.60532, alt_km: 0.02, epoch: nowS - 300, gate: "redundant", mode: "3D", n_sats: 6 }];
  }
  const lh = live2.position_history[live2.position_history.length - 1];
  lh.lat += 0.00003; lh.lon -= 0.00002; lh.epoch = Date.now() / 1000;
  if (live2.sky && Array.isArray(live2.sky.sats))
    live2.sky.sats.forEach((s) => {
      if (s.az_deg != null) { s.az_deg = (s.az_deg + 2.5) % 360; s.el_deg = (s.el_deg || 0) + 0.4; }
    });
  step("render(shifted) starts glide + dome lerp", () => g.render(live2));
  step("frames x90 (~1.5 s)", () => frames(90));
  step("glide position finite", () => {
    const p = g.posGlidePos();
    if (!p || !p.every(isFinite)) throw new Error("posGlidePos not finite");
  });
  step("dome lerp records finite", () => {
    Object.keys(g.DOME.anim).forEach((k) => {
      const r = g.DOME.anim[k];
      [r.az0, r.el0, r.az1, r.el1, r.t0].forEach((v) => { if (!isFinite(v)) throw new Error(k + " non-finite"); });
    });
  });

  step("tour: all stops", () => { for (let i = 0; i < 10; i++) g.tourShow(i); });
  step("tourEnd", () => g.tourEnd(true));
  step("replay: 40 frames", () => {
    g.S3.playing = true; g.S3.playT = 0;
    frames(40, 50);
    g.S3.playing = false;
  });
  step("scat mode toggle", () => { g.setScatMode("3h"); g.setScatMode("last60"); });
  step("wave gallery: 11 cards, all live lines", () => {
    if (g.WG.length !== 11) throw new Error("WG.length=" + g.WG.length);
    const wh = els["wavegallery"].innerHTML;
    g.WG.forEach((w) => {
      if (wh.indexOf('id="wg_' + w.id + '"') < 0) throw new Error("missing card wg_" + w.id);
      const le = els["wgl_" + w.id];
      if (!le || !le.innerHTML) throw new Error("no live line for " + w.id);
    });
  });
  step("physical chain: 11 stages rendered with tooltips", () => {
    const ph = els["physchain"].innerHTML;
    if (!ph) throw new Error("physchain empty");
    if ((ph.match(/class="jnode tip"/g) || []).length !== 11)
      throw new Error("expected 11 stage cards, got " + (ph.match(/class="jnode tip"/g) || []).length);
    ["The sky", "AA.250", "Feedline", "MAX2831", "Si5351", "SGPIO", "iCE40", "LPC4320",
     "USB 2.0", "live_radio", "State files"].forEach((k) => {
      if (ph.indexOf(k) < 0) throw new Error("missing stage keyword: " + k);
    });
    if ((ph.match(/What happens here —/g) || []).length !== 11)
      throw new Error("not every stage carries the three-part tooltip");
    if (ph.indexOf("jarrow flow") < 0) throw new Error("no animated connectors");
  });
  step("illustrative tags drawn on teaching canvases", () => {
    ["l0cv", "l1cv", "l2cv", "l4cv"].forEach((id) => { if (!(drawCounts[id] > 0)) throw new Error(id + " never drew"); });
  });
  step("heartbeat()", () => g.heartbeat());

  const ids = Object.keys(drawCounts).sort();
  // data-conditional canvases: with an empty-ish state they correctly never
  // paint (drawSpark needs clock.live_tick_hz, drawPhase needs phase,
  // posisx needs isx_km fixes inside its 3-h window) — only require them
  // when their data exists
  const expect = ["hbchart", "hbhisto", "posenu",
    "scat_xy", "scat_xz", "scat_yz", "scat3d", "prec", "sats", "skydome", "wavecv", "phdcv",
    "leadcv", "l0cv", "l1cv", "l2cv", "l4cv"].concat(g.WG.map((w) => "wg_" + w.id));
  if (live.clock && live.clock.live_tick_hz) expect.push("spark");
  if (live.phase) expect.push("phasechart");
  if ((live.position_history || []).some((f) => f.isx_km !== null && f.isx_km !== undefined &&
      f.epoch >= Date.now() / 1000 - 3 * 3600))
    expect.push("posisx");
  const blank = expect.filter((id) => !(drawCounts[id] > 0));
  console.log("canvases drew: " + ids.length + " · expected: " + expect.length +
    " · blank: " + (blank.length ? blank.join(",") : "none") +
    " · NaN draw args: " + nanViol.length + (nanViol.length ? " -> " + nanViol.slice(0, 5).join(" | ") : ""));
  if (blank.length) errors.push("blank canvases: " + blank.join(","));
  if (nanViol.length) errors.push("NaN draw args");

  console.log(errors.length ? "SMOKE: FAIL (" + errors.length + " failing steps)" : "SMOKE: PASS");
  process.exit(errors.length ? 1 : 0);
}

loadData().then(main).catch(function (e) {
  console.log("SMOKE: FAIL — " + (e && e.message || e));
  process.exit(1);
});
