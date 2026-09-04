#!/usr/bin/env node
"use strict";

const fs = require("fs");
const os = require("os");
const path = require("path");
const { pathToFileURL } = require("url");
const { spawn } = require("child_process");

const storyPath = path.join(__dirname, "story.html");
const html = fs.readFileSync(storyPath, "utf8");

function assert(condition, message) {
  if (!condition) throw new Error("story smoke: " + message);
}

assert(html.includes("<title>Photon Debugger"), "Photon Debugger title is missing");
assert((html.match(/<h1\b/g) || []).length === 1, "page must contain exactly one h1");
assert(html.includes("data:image/jpeg;base64,"), "generated orbit asset is not embedded");
assert(!html.includes("PHOTON_HERO_DATA"), "orbit asset placeholder remains");
assert(html.includes("window.__photonTest"), "deterministic browser test seam is missing");
assert(html.includes('fetch("/api/sync"'), "same-origin receiver summary fetch is missing");
assert(!html.includes("http://localhost:8090/api/sync"), "absolute localhost fallback must not return");
assert(!/setInterval\s*\(\s*poll/.test(html), "polling must be recursive and non-overlapping");

const prohibited = [
  "499 ps",
  "105.5 ps per tap",
  "exact thickness of space plasma",
  "TDEV < 0.244 ns",
  "ground-level muon detection"
];
for (const claim of prohibited) assert(!html.includes(claim), "unsupported claim remains: " + claim);

const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map((match) => match[1]);
assert(scripts.length === 1, "expected one inline application script");
for (const source of scripts) new Function(source);

const chromeCandidates = [
  process.env.CHROME_BIN,
  "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
  "/Applications/Chromium.app/Contents/MacOS/Chromium",
  "/usr/bin/google-chrome",
  "/usr/bin/chromium"
].filter(Boolean);
const chrome = chromeCandidates.find((candidate) => fs.existsSync(candidate));

if (!chrome) {
  console.log("story smoke PASS (static + JavaScript syntax; Chrome unavailable)");
  process.exit(0);
}

const url = pathToFileURL(storyPath).href + "?qa=1";

function dumpRenderedDom(profileDir) {
  return new Promise((resolve, reject) => {
    const maxBuffer = 8 * 1024 * 1024;
    let stdout = "";
    let stderr = "";
    let rendered = false;
    let timedOut = false;
    const detached = process.platform !== "win32";

    const child = spawn(chrome, [
      "--headless=new",
      "--no-sandbox",
      "--disable-gpu",
      "--disable-background-networking",
      "--disable-component-update",
      "--disable-extensions",
      "--no-first-run",
      "--hide-scrollbars",
      `--user-data-dir=${profileDir}`,
      "--window-size=1487,1058",
      "--virtual-time-budget=2200",
      "--dump-dom",
      url
    ], { detached, stdio: ["ignore", "pipe", "pipe"] });

    const stopChrome = () => {
      if (child.exitCode !== null || child.signalCode !== null) return;
      try {
        if (detached) process.kill(-child.pid, "SIGKILL");
        else child.kill("SIGKILL");
      } catch (error) {
        if (error.code !== "ESRCH") throw error;
      }
      child.stdout.destroy();
      child.stderr.destroy();
    };

    const timeout = setTimeout(() => {
      timedOut = true;
      stopChrome();
    }, 15000);

    child.stdout.setEncoding("utf8");
    child.stdout.on("data", (chunk) => {
      stdout += chunk;
      if (Buffer.byteLength(stdout) > maxBuffer) {
        stderr += "\nDOM output exceeded 8 MiB";
        stopChrome();
        return;
      }
      if (stdout.trimEnd().endsWith("</html>")) {
        rendered = true;
        stopChrome();
      }
    });

    child.stderr.setEncoding("utf8");
    child.stderr.on("data", (chunk) => {
      if (Buffer.byteLength(stderr) < maxBuffer) stderr += chunk;
    });

    child.on("error", (error) => {
      clearTimeout(timeout);
      reject(error);
    });

    child.on("close", (status, signal) => {
      clearTimeout(timeout);
      if (rendered) {
        resolve({ stdout, stderr, status, signal });
        return;
      }
      const reason = timedOut ? "timed out before a complete DOM" : `exited ${status ?? signal}`;
      reject(new Error(reason + ": " + stderr.slice(-800)));
    });
  });
}

(async () => {
  const profileDir = fs.mkdtempSync(path.join(os.tmpdir(), "photon-smoke-"));
  let result;
  try {
    result = await dumpRenderedDom(profileDir);
  } finally {
    fs.rmSync(profileDir, { recursive: true, force: true });
  }

  assert(result.stdout.includes('data-qa="passed"'), "in-browser interaction checks did not pass");
  assert(!result.stdout.includes('data-qa="failed"'), "in-browser interaction checks reported failure");
  assert((result.stdout.match(/class="stage-row"/g) || []).length === 9, "rendered signal stage count differs from nine");
  assert((result.stdout.match(/class="chapter-button"/g) || []).length === 8, "rendered chapter count differs from eight");
  assert(result.stdout.includes("ROWS ARRIVING"), "browser fixture did not reach processor-row state");
  assert(result.stdout.includes("No absolute TEC claim"), "scientific caveat is missing");
  assert(!/(Uncaught|ReferenceError|TypeError|SyntaxError):/.test(result.stderr || ""), "browser logged a JavaScript exception");

  console.log("story smoke PASS (static, syntax, browser render, interactions, error recovery)");
})().catch((error) => {
  console.error(error.stack || error);
  process.exit(1);
});
