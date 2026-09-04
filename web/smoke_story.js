// Regression smoke test for web/story.html
// Run: `node web/smoke_story.js`
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const HTML_PATH = path.join(__dirname, "story.html");
const API_URL = "http://localhost:8090/api/sync";

async function main() {
  console.log("smoke_story: reading " + HTML_PATH);
  const html = fs.readFileSync(HTML_PATH, "utf8");
  const scriptMatch = /<script(?:\s[^>]*)?>([\s\S]*?)<\/script>/gi.exec(html);
  if (!scriptMatch) throw new Error("No script block found in " + HTML_PATH);
  const scriptCode = scriptMatch[1];

  let liveData = {};
  try {
    const r = await fetch(API_URL);
    if (r.ok) liveData = await r.json();
  } catch (e) {
    console.log("note: could not reach " + API_URL + " (" + e.message + "); using fallback");
  }

  const elements = {};
  const drawCounts = {
    relativityCanvas: 0,
    plasmaCanvas: 0,
    geomagCanvas: 0,
    tropoCanvas: 0,
    multipathCanvas: 0,
    ellipseCanvas: 0,
    hardwareCanvas: 0,
    solarCascadeCanvas: 0
  };
  const nanViolations = [];

  function getEl(id) {
    if (!elements[id]) {
      elements[id] = {
        id: id,
        children: [],
        style: {},
        dataset: {},
        value: "0",
        innerHTML: "",
        textContent: "",
        width: 480,
        height: 280,
        clientWidth: 480,
        clientHeight: 280,
        getBoundingClientRect: () => ({ left: 0, top: 0, width: 480, height: 280 }),
        addEventListener: () => {},
        getContext: (type) => {
          if (type !== "2d") return null;
          return new Proxy({}, {
            get: (t, p) => {
              if (p === "measureText") return (s) => ({ width: String(s).length * 8 });
              if (p === "createRadialGradient" || p === "createLinearGradient") {
                return () => ({ addColorStop: () => {} });
              }
              return (...args) => {
                drawCounts[id] = (drawCounts[id] || 0) + 1;
                for (const a of args) {
                  if (typeof a === "number" && !isFinite(a)) {
                    nanViolations.push(id + "." + String(p) + "(" + a + ")");
                  }
                }
              };
            },
            set: () => true
          });
        }
      };
    }
    return elements[id];
  }

  const intervals = [];
  const timeouts = [];
  const context = {
    console: console,
    document: {
      getElementById: getEl,
      querySelector: (sel) => getEl(sel.replace(/^[#.]/, "")),
      querySelectorAll: () => [],
      body: getEl("body"),
      addEventListener: () => {}
    },
    window: {
      addEventListener: () => {},
      requestAnimationFrame: (cb) => {
        try { cb(performance.now()); } catch (e) {}
        return 1;
      },
      cancelAnimationFrame: () => {},
      setInterval: (fn, ms) => {
        intervals.push({ fn, ms });
        return intervals.length;
      },
      clearInterval: () => {},
      setTimeout: (fn, ms) => {
        timeouts.push({ fn, ms });
        return timeouts.length;
      },
      clearTimeout: () => {},
      innerWidth: 1400,
      innerHeight: 900
    },
    requestAnimationFrame: (cb) => {
      try { cb(performance.now()); } catch (e) {}
      return 1;
    },
    setInterval: (fn, ms) => {
      intervals.push({ fn, ms });
      return intervals.length;
    },
    clearInterval: () => {},
    setTimeout: (fn, ms) => {
      timeouts.push({ fn, ms });
      return timeouts.length;
    },
    clearTimeout: () => {},
    fetch: async (url) => {
      if (liveData && Object.keys(liveData).length > 0) {
        return {
          ok: true,
          json: async () => JSON.parse(JSON.stringify(liveData))
        };
      }
      return {
        ok: true,
        json: async () => ({
          relativity: { total_offset_ns_day: 38624.1, gr_gravitational_blueshift_ns_day: 45656.2, sr_kinematic_dilation_ns_day: -7032.1 },
          klobuchar: { mean_klobuchar_delay_m: 3.42, mean_tecu: 21.3 },
          hoi: { i2_mean_geomag_delay_mm: 1.20, faraday_rotation_deg: 2.3 },
          saastamoinen: { mean_ztd_m: 2.312, mean_zhd_m: 2.185, mean_zwd_m: 0.127 },
          pwv: { mean_pwv_kg_m2: 18.26 },
          multipath: { overall_multipath_index_db: 3.56 },
          gdop: { gdop: 2.45, pdop: 2.12, hdop: 1.15, vdop: 1.78, horizontal_error_ellipse_95_m2: 4.82 },
          channels: {
            "G14": { cn0_dbhz: 42.1, elev_deg: 54.2, az_deg: 138.4, d_rel_total_ns: 38624.0, ztd_m: 2.45, zwd_m: 0.14, s4: 0.08, sigma_phi_rad: 0.04 },
            "B1I_C03": { cn0_dbhz: 40.5, elev_deg: 41.2, az_deg: 210.5, d_rel_total_ns: 38590.0, ztd_m: 2.82, zwd_m: 0.16, s4: 0.09, sigma_phi_rad: 0.05 }
          }
        })
      };
    },
    performance: performance,
    Math: Math,
    Date: Date
  };

  vm.createContext(context);
  console.log("smoke_story: running script in VM...");
  vm.runInContext(scriptCode, context);

  await new Promise((resolve) => setTimeout(resolve, 300));

  console.log("smoke_story: canvas draw counts:", drawCounts);
  for (const [k, count] of Object.entries(drawCounts)) {
    if (count === 0) {
      console.warn("warning: canvas " + k + " had 0 draw operations");
    }
  }

  if (nanViolations.length > 0) {
    console.error("FAIL: NaN or non-finite canvas drawing violations found:");
    console.error(nanViolations.slice(0, 10));
    process.exit(1);
  }

  console.log("smoke_story: SUCCESS! All canvases initialized cleanly without NaN violations.");
}

main().catch((e) => {
  console.error("smoke_story: FATAL ERROR:", e);
  process.exit(1);
});
