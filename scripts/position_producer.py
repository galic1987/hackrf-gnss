#!/usr/bin/env python3
"""Position producer: snapshot PVT from the live tracker + fresh ephemeris.

Every 5 min runs examples/live_fix, which reads the live tracker state
(per-PRN code phases) and solves a coarse-time snapshot fix against BRDC
broadcast ephemeris. The RINEX nav file is refreshed from BKG when older
than 1 h (BRDC00WRD_R = world multi-GNSS rapid, updated ~hourly; GPS
ephemeris validity ~2-4 h so freshness matters — a 6-hourly refresh left a
modeled-sky blind gap at the end of every cycle).

live_fix writes observations/state.position.json directly; this wrapper
just schedules it and logs.
"""
import os
import subprocess
import time
import urllib.request

OBS = "/Volumes/Radiator 8TB/gnss/observations"
RINEX = f"{OBS}/brdc_latest.rnx"
LIVE_FIX = "/Volumes/Radiator 8TB/gnss/hackrf_gnss/target/release/examples/live_fix"
SOLVE_EVERY_S = 300
# BKG updates the world rapid BRDC ~hourly and GPS ephemeris fit windows are
# ~4 h: a 6-hourly refresh guaranteed a modeled-sky collapse at the end of
# every cycle (seen live 2026-08-25 20:00 UTC). Refresh hourly instead.
REFRESH_S = 3600


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def refresh_rinex():
    """Download today's (or yesterday's) BRDC file from BKG if stale."""
    try:
        age = time.time() - os.path.getmtime(RINEX)
        if age < REFRESH_S:
            return
    except OSError:
        pass
    import gzip
    for doy_back in (0, 1):
        t = time.time() - doy_back * 86400
        doy = time.strftime("%j", time.gmtime(t))
        yr = time.strftime("%Y", time.gmtime(t))
        url = (f"https://igs.bkg.bund.de/root_ftp/IGS/BRDC/{yr}/{doy}/"
               f"BRDC00WRD_R_{yr}{doy}0000_01D_MN.rnx.gz")
        try:
            with urllib.request.urlopen(url, timeout=60) as r:
                data = r.read()
            text = gzip.decompress(data).decode("ascii", "replace")
            if "RINEX VERSION" not in text[:200]:
                raise ValueError("not a RINEX file")
            tmp = RINEX + ".tmp"
            with open(tmp, "w") as f:
                f.write(text)
            os.replace(tmp, RINEX)
            log(f"refreshed ephemeris from {url} ({len(text)//1024} KB)")
            return
        except Exception as e:
            log(f"ephemeris fetch failed ({url}): {e}")
    log("no fresh ephemeris available — keeping existing file")


def main():
    log("position producer starting")
    while True:
        refresh_rinex()
        r = subprocess.run([LIVE_FIX], capture_output=True, text=True, timeout=120)
        out = (r.stdout + r.stderr).strip()
        if out:
            log(out)
        time.sleep(SOLVE_EVERY_S)


if __name__ == "__main__":
    main()
