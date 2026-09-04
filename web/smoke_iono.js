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
    "satTableBody", "tidBadge", "tidVal", "tidAmp", "tidSnr", "tidSpan",
    "isbVal", "isbMad", "isbPath", "isbAdev", "isbSpan", "isbN",
    "thermalY", "thermalMhz", "thermalMmDay", "thermalDiurnal", "thermalPs",
    "thermalRms", "thermalHours", "mpiBadge", "mpiVal", "mpiReflective", "mpiClean",
    "mpiActiveRisk", "tropoBadge", "ztdVal", "zhdVal", "zwdVal", "ztdNs", "p0Val",
    "relBadge", "relVal", "relGr", "relSr", "relFactory", "relDrift", "relSagnac", "relEcc",
    "pwvBadge", "pwvVal", "pwvMass", "tmVal", "tdVal", "bevisPi", "regimeVal", "maxSwv",
    "rfiBadge", "tsysVal", "fsplVal", "marginVal", "n0Val", "tskyVal", "lnaNf", "meanCn0",
    "gdopBadge", "gdopVal", "hdopVal", "vdopVal", "ellipseVal", "gdopAdvVal", "areaAdvVal", "ellipseAz",
    "ccdBadge", "ccdVal", "hatchBiasVal", "cmcNoiseVal", "vpVal", "vgVal",
    "hoiBadge", "hoiVal", "nsAsymVal", "faradayVal", "hoiPsVal", "rayBendVal", "bdsHoiGain",
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

  // Check that experiment values are rendered
  console.log("TID Val:", elements["tidVal"].innerHTML);
  console.log("ISB Val:", elements["isbVal"].innerHTML);
  console.log("Thermal Val:", elements["thermalY"].innerHTML);
  console.log("MPI Val:", elements["mpiVal"].innerHTML);
  console.log("Cleanest Sector:", elements["mpiClean"].textContent);
  console.log("Reflective Sector:", elements["mpiReflective"].textContent);
  console.log("ZTD Val:", elements["ztdVal"].innerHTML);
  console.log("Relativity Val:", elements["relVal"].innerHTML);
  console.log("Relativity GR / SR:", elements["relGr"].textContent, "/", elements["relSr"].textContent);
  console.log("Relativity Uncompensated:", elements["relDrift"].textContent);
  console.log("Relativity Sagnac Max:", elements["relSagnac"].textContent);
  console.log("PWV Val:", elements["pwvVal"].innerHTML);
  console.log("PWV Mass / Tm / Td:", elements["pwvMass"].textContent, "/", elements["tmVal"].textContent, "/", elements["tdVal"].textContent);
  console.log("PWV Regime:", elements["regimeVal"].textContent);
  console.log("Radiometry Tsys:", elements["tsysVal"].innerHTML);
  console.log("Radiometry FSPL / Margin:", elements["fsplVal"].textContent, "/", elements["marginVal"].textContent);
  console.log("Radiometry Threat:", elements["rfiBadge"].textContent);
  console.log("GDOP Val:", elements["gdopVal"].innerHTML);
  console.log("GDOP HDOP / VDOP:", elements["hdopVal"].textContent, "/", elements["vdopVal"].textContent);
  console.log("GDOP 95% Ellipse:", elements["ellipseVal"].textContent, "@", elements["ellipseAz"].textContent);
  console.log("GDOP Advantage vs GPS:", elements["gdopAdvVal"].textContent, "(Area:", elements["areaAdvVal"].textContent + ")");
  console.log("CCD Val:", elements["ccdVal"].innerHTML);
  console.log("CCD Hatch Bias / Noise:", elements["hatchBiasVal"].textContent, "/", elements["cmcNoiseVal"].textContent);
  console.log("CCD Plasma v_p / v_g:", elements["vpVal"].textContent, "/", elements["vgVal"].textContent);
  console.log("CCD Status:", elements["ccdBadge"].textContent);
  console.log("HOI Val:", elements["hoiVal"].innerHTML);
  console.log("HOI N-S Asymmetry / Delay:", elements["nsAsymVal"].textContent, "/", elements["hoiPsVal"].textContent);
  console.log("HOI Faraday / Bending:", elements["faradayVal"].textContent, "/", elements["rayBendVal"].textContent);
  console.log("HOI Status:", elements["hoiBadge"].textContent);

  console.log("SMOKE_IONO: PASS");
}

main().catch(err => {
  console.error("SMOKE_IONO FAIL:", err);
  process.exit(1);
});
