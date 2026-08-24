#!/usr/bin/env python3
"""Validate the Rust GPS verdict + ephemeris over the recorder's REAL log against
Python report.py. At the fixed 0.80 gate (null_trials=0) the two must match
EXACTLY. With the permutation-null calibration on, the confirmed PRN SET must
match; the pass COUNT may differ by a boundary pass because the null is Monte
Carlo (Rust xorshift vs numpy PCG64).

Build first: cargo build --release --example gps_verdict
usage: validate_gps_verdict.py [observations.jsonl] [tle]
"""
import json, os, sys, subprocess, re
from datetime import datetime
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "validation"))
from report import gnss_verdict  # noqa: E402

OBS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "observations", "observations.jsonl")
TLE = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, "observations", "tle_gps-ops.tle")
EXE = os.path.join(ROOT, "hackrf_gnss", "target", "release", "examples", "gps_verdict")


def rust(nt):
    out = subprocess.run([EXE, OBS, TLE, str(nt)], capture_output=True, text=True).stdout
    conf = re.search(r"confirmed (\d+) PRNs \[([\d, ]*)\]", out)
    prns = [int(x) for x in conf.group(2).split(",") if x.strip()]
    return int(conf.group(1)), prns


def py(nt):
    recs = [json.loads(l) for l in open(OBS) if l.strip()]
    gn = [(r["gnss"], datetime.fromisoformat(r["utc"]).timestamp())
          for r in recs if r.get("gnss") and not r["gnss"].get("error")]
    v = gnss_verdict(gn, null_trials=nt, tle_path=TLE, rx_latlon=(40.65, -73.80))
    return v["confirmed"], v["confirmed_prns"]


def main():
    if not os.path.exists(EXE):
        print("build: cargo build --release --example gps_verdict"); return 2
    rc, rp = rust(0); pc, pp = py(0)
    print(f"fixed gate:  rust {rc} {rp}\n             python {pc} {pp}")
    exact = (rc == pc and rp == pp)
    rc2, rp2 = rust(2000); pc2, pp2 = py(2000)
    print(f"calibrated:  rust {rc2} {sorted(set(rp2))}\n             python {pc2} {sorted(set(pp2))}")
    setmatch = sorted(set(rp2)) == sorted(set(pp2))
    ok = exact and setmatch
    print("\n" + ("OK: Rust verdict matches Python on the real log "
                  "(exact at fixed gate, PRN set matches calibrated)"
                  if ok else "REVIEW: verdict mismatch"))
    if not exact:
        print("  fixed-gate mismatch is a real bug")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
