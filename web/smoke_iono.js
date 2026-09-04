// Regression smoke test for web/iono.html
// Run: `node web/smoke_iono.js` (hits http://localhost:8090/api/sync)
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const HTML_PATH = path.join(__dirname, "iono.html");
const API_URL = "http://localhost:8090/api/sync";

async function main() {
  console.log("smoke_iono: reading " + HTML_PATH);
  const html = fs.readFileSync(HTML_PATH, "utf8");
  const scriptMatch = /<script(?:\s[^>]*)?>([\s\S]*?)<\/script>/gi.exec(html);
  if (!scriptMatch) throw new Error("No script block found in " + HTML_PATH);
  const scriptCode = scriptMatch[1];

  let liveData = {};
  try {
    const r = await fetch(API_URL);
    if (r.ok) liveData = await r.json();
  } catch (e) {
    console.log("warning: could not reach " + API_URL + " (" + e.message + ")");
  }

  const elements = {};
  const drawCounts = { skyCanvas: 0, stripCanvas: 0 };
  const nanViolations = [];

  function getEl(id) {
    if (!elements[id]) {
      elements[id] = {
        id: id,
        children: [],
        style: {},
        dataset: {},
        innerHTML: "",
        textContent: "",
        width: 880,
        height: 880,
        clientWidth: 880,
        clientHeight: 880,
        getBoundingClientRect: () => ({ left: 0, top: 0, width: 880, height: 880 }),
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

  // Pre-populate known element IDs
  [
    "weatherVal", "weatherSub", "sigmaVal", "s4Val", "tecVal", "rotiVal",
    "satTableBody",
    "gpsTrackBadge", "gpsTrackVal", "gpsTrackSub", "gpsMeanCn0", "gpsMaxLock",
    "bdsTrackBadge", "bdsTrackVal", "bdsTrackSub", "bdsMeanCn0", "bdsMaxLock",
    "atscBadge", "atscVal", "atscSub", "atscDisp", "atscSigma",
    "tdcBadge", "tdcVal", "tdcSub", "tdcResidual", "tdcGpsdo", "tdcSatCount",
    "isbBadge", "isbVal", "isbMad", "isbPath", "isbAdev", "isbSpan", "isbN",
    "thermalBadge", "thermalY", "thermalMhz", "thermalMmDay", "thermalDiurnal", "thermalPs",
    "thermalRms", "thermalHours",
    "syncStatus", "skyCanvas", "stripCanvas", "tooltip"
  ].forEach(getEl);

  const windowObj = {};
  const sandbox = {
    document: {
      getElementById: (id) => getEl(id),
      createElement: (tag) => getEl("dyn_" + tag)
    },
    window: windowObj,
    fetch: () => Promise.resolve({ json: () => Promise.resolve(liveData) }),
    setInterval: () => 1,
    setTimeout: (fn) => fn(),
    Date: Date,
    Math: Math,
    console: console
  };

  vm.createContext(sandbox);
  vm.runInContext(scriptCode, sandbox);

  // Directly drive updateUI with liveData
  if (typeof windowObj.updateUI === "function") {
    windowObj.updateUI(liveData);
  } else {
    throw new Error("window.updateUI was not exported");
  }

  // Check that canvases drew
  console.log("Canvas draw counts:", drawCounts);
  if (drawCounts.skyCanvas === 0) throw new Error("skyCanvas had 0 draw operations");
  if (drawCounts.stripCanvas === 0) throw new Error("stripCanvas had 0 draw operations");
  if (nanViolations.length > 0) throw new Error("NaN violations detected: " + nanViolations.join(", "));

  // Check that table has content
  const tableHtml = elements["satTableBody"].innerHTML;
  console.log("Table rendered rows length:", tableHtml.length);
  if (!tableHtml || tableHtml.length < 50) throw new Error("satTableBody was empty or unexpectedly short");

  // Check genuine physical observables & baseline diagnostics (Cards 1-6)
  console.log("Card 1 (GPS L1 C/A):", elements["gpsTrackVal"].innerHTML, "| Mean C/N0:", elements["gpsMeanCn0"].textContent, "| Max Lock:", elements["gpsMaxLock"].textContent);
  console.log("Card 2 (BeiDou B1I):", elements["bdsTrackVal"].innerHTML, "| Mean C/N0:", elements["bdsMeanCn0"].textContent, "| Max Lock:", elements["bdsMaxLock"].textContent);
  console.log("Card 3 (ATSC Ch35):", elements["atscVal"].innerHTML, "| Disp:", elements["atscDisp"].textContent, "| Sigma:", elements["atscSigma"].textContent);
  console.log("Card 4 (Hardware TDC):", elements["tdcVal"].innerHTML, "| Residual:", elements["tdcResidual"].textContent, "| GPSDO:", elements["tdcGpsdo"].textContent);
  console.log("Card 5 (ISB Baseline):", elements["isbVal"].innerHTML, "| Scatter:", elements["isbMad"].innerHTML, "| RF dL:", elements["isbPath"].innerHTML);
  console.log("Card 6 (Thermal Baseline):", elements["thermalY"].innerHTML, "| Secular:", elements["thermalMhz"].innerHTML, "| Drift:", elements["thermalMmDay"].innerHTML);

  // Assert essential fields are not empty
  if (!elements["gpsTrackVal"].innerHTML) throw new Error("gpsTrackVal is empty");
  if (!elements["bdsTrackVal"].innerHTML) throw new Error("bdsTrackVal is empty");
  if (!elements["atscVal"].innerHTML) throw new Error("atscVal is empty");
  if (!elements["tdcVal"].innerHTML) throw new Error("tdcVal is empty");
  if (!elements["isbVal"].innerHTML) throw new Error("isbVal is empty");
  if (!elements["thermalY"].innerHTML) throw new Error("thermalY is empty");

  console.log("SMOKE_IONO: PASS");
}

main().catch(err => {
  console.error("SMOKE_IONO FAIL:", err);
  process.exit(1);
});
