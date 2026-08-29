#!/usr/bin/env python3
"""gpsdo_probe.py — publish Leo Bodnar LBE-1421 GPSDO health to state.gpsdo.json.

The station's clock star (Bodnar OUT1/OUT2 -> Pro#2/One P1) is only trustworthy
while the GPSDO itself stays locked. Nothing else on the machine watches it:
this probe reads the NMEA stream on the Bodnar's USB CDC port and atomically
publishes fix quality, satellite count, HDOP and per-constellation SNR so a
reference degradation is visible in the observations directory (and, via
/api/sync, on the panel) instead of being discovered after the fact.

Stdlib only (macOS cu.* devices are plain ttys after stty). Publish cadence
~5 s, atomic (tmp + rename), with an explicit `ttl_s` so consumers can
fail-closed when the probe dies.

R15 origin: reviewer noted state.gpsdo.json was absent and nothing had read
the Bodnar serial since 2026-08-26 (/tmp/bodnar_nmea.log, gsv.log).
"""

import glob
import json
import os
import re
import subprocess
import sys
import tempfile
import time

OBS = os.path.join(os.path.dirname(__file__), "..", "..", "observations")
OUT = os.path.abspath(os.path.join(OBS, "state.gpsdo.json"))
PUBLISH_EVERY_S = 5.0
TTL_S = 30.0

GGA = re.compile(r"^\$G[NP]GGA,")
RMC = re.compile(r"^\$G[NP]RMC,")
GSV = re.compile(r"^\$G[NP]GSV,")


def open_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        return None, None
    port = ports[0]
    # USB CDC ignores baud, but the tty layer needs a sane raw config.
    subprocess.run(["stty", "-f", port, "9600", "raw", "-echo", "-icanon",
                    "min", "0", "time", "10"], check=False)
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    return port, fd


def parse_gga(fields, st):
    # $GNGGA,hhmmss.ss,lat,N,lon,E,q,nsat,hdop,alt,M,...
    try:
        st["fix_quality"] = int(fields[6])
        st["n_sat"] = int(fields[7])
        st["hdop"] = float(fields[8])
        st["alt_m"] = float(fields[9]) if fields[9] else None
        st["gga_epoch"] = time.time()
    except (ValueError, IndexError):
        pass


def parse_rmc(fields, st):
    # $GNRMC,hhmmss.ss,A,...,ddmmyy,...,D*..
    try:
        st["rmc_status"] = fields[2]          # A=valid, V=void
        st["rmc_mode"] = (fields[12].split("*")[0]
                          if len(fields) > 12 else None)  # D=DGPS
        st["rmc_epoch"] = time.time()
    except IndexError:
        pass


def parse_gsv(fields, st):
    # $GPGSV,n,m,total,(prn,el,az,snr)*4 — keep per-constellation SNR list.
    talker = fields[0][1:3]  # GP / GN / GL / GA / GB
    try:
        total = int(fields[3])
        snrs = []
        for i in range(4, min(len(fields) - 1, 19), 4):
            snr = fields[i + 3].split("*")[0]
            if snr:
                snrs.append(int(snr))
        if snrs:
            key = {"GP": "gps", "GL": "glo", "GA": "gal", "GB": "bds",
                   "GN": "mixed"}.get(talker, talker.lower())
            e = st.setdefault("gsv", {}).setdefault(
                key, {"n_in_view": 0, "snrs": [], "epoch": 0})
            e["n_in_view"] = max(e["n_in_view"], total)
            e["snrs"] = (e["snrs"] + snrs)[-16:]
            e["epoch"] = time.time()
    except (ValueError, IndexError):
        pass


def snapshot(st, port):
    now = time.time()
    gsv = {}
    for k, e in st.get("gsv", {}).items():
        if now - e["epoch"] < TTL_S and e["snrs"]:
            snrs = e["snrs"]
            gsv[k] = {"n_in_view": e["n_in_view"],
                      "snr_max": max(snrs),
                      "snr_med": sorted(snrs)[len(snrs) // 2]}
    fresh_gga = now - st.get("gga_epoch", 0) < TTL_S
    # Panel-merge law (src/main.rs): whole top-level key, later-file-wins —
    # everything lives under "gpsdo" so no other producer's keys can collide.
    return {
        "epoch": now,
        "ttl_s": TTL_S,
        "gpsdo": {
            "port": port,
            "lock": bool(fresh_gga and st.get("fix_quality", 0) > 0),
            "fix_quality": st.get("fix_quality") if fresh_gga else None,
            "n_sat": st.get("n_sat") if fresh_gga else None,
            "hdop": st.get("hdop") if fresh_gga else None,
            "alt_m": st.get("alt_m") if fresh_gga else None,
            "rmc_status": st.get("rmc_status") if now - st.get("rmc_epoch", 0) < TTL_S else None,
            "rmc_mode": st.get("rmc_mode") if now - st.get("rmc_epoch", 0) < TTL_S else None,
            "gsv": gsv,
        },
    }


def publish(doc):
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(OUT), suffix=".tmp")
    with os.fdopen(fd, "w") as f:
        json.dump(doc, f)
    os.rename(tmp, OUT)


def main():
    port, fd = open_port()
    if fd is None:
        publish({"epoch": time.time(), "ttl_s": TTL_S,
                 "gpsdo": {"port": None, "lock": False,
                           "error": "no /dev/cu.usbmodem* (GPSDO absent)"}})
        sys.exit("no GPSDO serial port found")
    print(f"gpsdo_probe: reading {port}", flush=True)
    st = {}
    buf = b""
    last_pub = 0.0
    while True:
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            chunk = b""
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    s = line.decode("ascii", "replace").strip()
                except Exception:
                    continue
                if not s.startswith("$"):
                    continue
                f = s.split(",")
                if GGA.match(s):
                    parse_gga(f, st)
                elif RMC.match(s):
                    parse_rmc(f, st)
                elif GSV.match(s):
                    parse_gsv(f, st)
        now = time.time()
        if now - last_pub >= PUBLISH_EVERY_S:
            publish(snapshot(st, port))
            last_pub = now
        time.sleep(0.2)


if __name__ == "__main__":
    main()
