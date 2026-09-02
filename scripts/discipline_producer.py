#!/usr/bin/env python3
"""Discipline producer: closes the clock loop.

Reads the cross-producer consensus residual from the panel API and steers the
Pro's clock-correction register toward zero residual. Design rules (from the
board timing protocol spec):

  - SLEW, never leap: at most STEP_PPM per cycle; the register applies a
    frequency offset, so time never steps and monotonicity is sacred.
  - Self-verifying sign: the first application is a probe — if |residual|
    grew instead of shrinking, the sign is flipped and latched for the
    session (and logged loudly). After that, sign errors are impossible.
  - Self-healing across flashes: a reflash zeroes the register; the residual
    then reappears and the loop simply re-converges. No state is trusted
    across silence: if the consensus goes stale, we stop steering.
  - Radio discipline: pause sync_producer, never touch a busy radio.

Publishes state.discipline.json: {epoch, ttl_s, clock.correction_ppm,
discipline:{target_ppm, last_step, note}}.
"""
import json, os, subprocess, time, urllib.request

TOOLS = "/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src"
ENV = dict(os.environ,
           DYLD_LIBRARY_PATH="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/libhackrf/src")
PRO = "QUARANTINED_NO_SERIAL"
API = "http://localhost:8090/api/sync"
STATE = "/Volumes/Radiator 8TB/gnss/observations/state.discipline.json"
POLL_S = 60.0
STEP_PPM = 0.1          # slew limit per cycle
MAX_CORR_PPM = 50.0
DEADBAND_PPM = 0.01     # don't hunt below this residual
STALE_S = 1800.0

corr = 0.0              # our model of the register (starts at 0 after flash)
sign = +1.0             # corr_new = corr - sign*resid; +1: negative residual (slow clock) gets positive correction
probed = False
prev_resid = None


def set_corr(ppm):
    try:
        out = subprocess.run([f"{TOOLS}/hackrf_pro", "-d", PRO, "--clock-corr", f"{ppm:.4f}"],
                             capture_output=True, text=True, timeout=15, env=ENV)
        return "clock" in (out.stdout + out.stderr).lower() or out.returncode == 0
    except Exception:
        return False


def sync_ctl(sig):
    try:
        for pid in subprocess.run(["pgrep", "-f", "sync_producer.py"],
                                  capture_output=True, text=True).stdout.split():
            subprocess.run(["kill", sig, pid], capture_output=True)
    except Exception:
        pass


def radio_busy():
    return subprocess.run(["pgrep", "-f", f"hackrf_transfer -d {PRO}"],
                          capture_output=True).returncode == 0


def main():
    print(
        "QUARANTINED: legacy live clock actuator targets the dead Pro #1 and "
        "violates the current shadow/unity policy; no radio was opened.",
        file=__import__("sys").stderr,
    )
    return 78

    global corr, sign, probed, prev_resid
    note = "loop started"
    while True:
        try:
            with urllib.request.urlopen(API, timeout=5) as r:
                st = json.load(r)
            resid = st.get("consensus_ppm")
            age = time.time() - st.get("epoch", 0)
            alerts = st.get("alerts") or []
        except Exception:
            resid, age, alerts = None, 1e9, []
        if resid is None or age > STALE_S:
            note = "consensus stale — holding"
        elif alerts:
            note = f"integrity alert active — holding ({alerts[0][:60]})"
        elif abs(resid) < DEADBAND_PPM:
            note = f"in deadband ({resid:+.4f} ppm) — loop closed"
        elif radio_busy():
            note = "radio busy — retry next cycle"
        else:
            target = max(-MAX_CORR_PPM, min(MAX_CORR_PPM, corr - sign * resid))
            step = max(-STEP_PPM, min(STEP_PPM, target - corr))
            newcorr = round(corr + step, 4)
            sync_ctl("-STOP")
            try:
                ok = set_corr(newcorr)
            finally:
                sync_ctl("-CONT")
            if ok:
                corr = newcorr
                note = f"applied {corr:+.4f} ppm (residual {resid:+.4f})"
                if probed is False and prev_resid is not None:
                    if abs(resid) > abs(prev_resid) * 1.5:
                        sign = -sign
                        note = f"SIGN FLIP latched (residual grew {prev_resid:+.4f} -> {resid:+.4f})"
                    probed = True
                prev_resid = resid
            else:
                note = "register write failed — retry next cycle"
        tmp = STATE + ".tmp"
        json.dump({"epoch": time.time(), "ttl_s": 300,
                   "clock": {"correction_ppm": corr},
                   "discipline": {"residual_ppm": resid, "step_limit_ppm": STEP_PPM,
                                  "sign": sign, "note": note}},
                  open(tmp, "w"), indent=1)
        os.replace(tmp, STATE)
        time.sleep(POLL_S)


if __name__ == "__main__":
    raise SystemExit(main())
