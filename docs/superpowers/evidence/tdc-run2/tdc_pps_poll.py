#!/usr/bin/env python3
"""Poll HackRF Pro TDC status (0x31 bit0) and capture thermometer codes on toggles.

External-trigger TDC mode (0x30=0): the PPS edge on P2 drives the carry chain
directly (raw async, pre-synchronizer); the TDC freezes on the chain-detected
edge and flips the valid bit once per freeze. This loop watches the valid bit
at ~8 Hz and, on each flip, promptly reads the frozen 6-byte thermometer map
0x20-0x25. Static between freezes per top/timing.py, so no tearing.

Usage: tdc_pps_poll.py <serial> <duration_s> <outfile.jsonl>
"""
import json
import re
import subprocess
import sys
import time

HACKRF_PRO = "/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src/hackrf_pro"
RE_HEX = re.compile(r"0x([0-9a-fA-F]{2})")

serial, duration_s, outfile = sys.argv[1], float(sys.argv[2]), sys.argv[3]


def read_reg(addr, tries=3):
    for _ in range(tries):
        try:
            out = subprocess.run(
                [HACKRF_PRO, "-d", serial, "--read-reg", str(addr)],
                capture_output=True, text=True, timeout=5)
            m = RE_HEX.search(out.stdout)
            if m:
                return int(m.group(1), 16)
        except Exception:
            pass
        time.sleep(0.05)
    return None


def main():
    t0 = time.monotonic()
    deadline = t0 + duration_s
    n_poll = n_err = n_evt = 0
    last_valid = None
    with open(outfile, "a") as f:
        f.write(json.dumps({"kind": "start", "wall": time.time(),
                            "serial": serial, "dur": duration_s}) + "\n")
        while time.monotonic() < deadline:
            v = read_reg(0x31)
            n_poll += 1
            if v is None:
                n_err += 1
                continue
            valid = v & 1
            if last_valid is not None and valid != last_valid:
                n_evt += 1
                thermo = []
                ok = True
                for a in range(0x20, 0x26):
                    b = read_reg(a)
                    if b is None:
                        ok = False
                        break
                    thermo.append(b)
                rec = {"kind": "evt", "wall": time.time(),
                       "mono": time.monotonic(), "n": n_evt, "valid": valid,
                       "status": v}
                if ok:
                    rec["thermo"] = "".join(f"{b:02x}" for b in thermo)
                    rec["pop"] = sum(bin(b).count("1") for b in thermo)
                f.write(json.dumps(rec) + "\n")
                f.flush()
            last_valid = valid
            time.sleep(0.11)
        f.write(json.dumps({"kind": "stop", "wall": time.time(),
                            "polls": n_poll, "errors": n_err,
                            "events": n_evt}) + "\n")


if __name__ == "__main__":
    main()
