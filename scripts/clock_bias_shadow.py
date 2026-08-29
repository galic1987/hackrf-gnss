#!/usr/bin/env python3
"""Leg 1 v2 shadow validator (read-only, tracker-safe).

Every fresh rho_m on a gated GPS channel, this logs the error the v2
skip-and-predict rule would have carried into that epoch had it predicted
from the previous fresh update:
    pred = rho_last + LAM_L1 * (carr_last - carr_now)   (negated-carrier rule)
    e    = pred - rho_now
With the sign fixed, |e| should sit at the code-carrier rate-mismatch class
(tens of metres at a ~6 s staircase age, worst ~few hundred on fast sats).
The v1 (wrong-sign) rule would land e at ~2x the true range advance —
km-class at the same age. So median |e| per PRN is the decisive A/B of the
carrier-sign fix on live data, hours before n>=5 lets the real solve run.

Output: /tmp/clock_bias_shadow.jsonl, one row per fresh update:
    {t, prn, age_s, pred_err_m, drho_m, lam_dcarr_m}
plus a 15-min stderr summary per PRN (n, median|e|, p90|e|, sign agreement).
Runs forever; kill by pattern."""
import json
import math
import statistics
import time

STATE = "/Volumes/Radiator 8TB/gnss/observations/state.tracker.json"
OUT = "/tmp/clock_bias_shadow.jsonl"
LAM_L1 = 299_792_458.0 / 1_575_420_000.0
SUMMARY_EVERY_S = 900.0


def gated_gps(st):
    for s in st["tracker"]["sats"]:
        if s.get("sys") != "gps":
            continue
        if s.get("cn0_proxy", 0.0) < 30.0 or s.get("lock_s", 0.0) < 20.0:
            continue
        if s.get("rho_m") is None or s.get("t_tx") is None:
            continue
        yield s


def main():
    prev = {}          # prn -> dict(rho, carr, lock_s)  (last seen, any epoch)
    fresh = {}         # prn -> dict(t, rho, carr)       (last FRESH point)
    stats = {}         # prn -> list of (age, e)
    last_epoch = 0.0
    last_summary = time.time()
    out = open(OUT, "a", buffering=1)
    while True:
        time.sleep(0.5)
        try:
            st = json.load(open(STATE))
        except Exception:
            continue
        epoch = st.get("epoch", 0.0)
        if epoch <= last_epoch:
            continue
        last_epoch = epoch
        for s in gated_gps(st):
            prn = s["prn"]
            rho = s["rho_m"]
            carr = s.get("carrier_cycles", 0.0)
            lock = s.get("lock_s", 0.0)
            p = prev.get(prn)
            if p is not None and lock < p["lock_s"]:
                fresh.pop(prn, None)       # relock: carrier origin changed
                stats.pop(prn, None)
            is_fresh = p is None or rho != p["rho"]
            if is_fresh:
                f = fresh.get(prn)
                if f is not None:
                    age = epoch - f["t"]
                    pred = f["rho"] + LAM_L1 * (f["carr"] - carr)
                    e = pred - rho
                    drho = rho - f["rho"]
                    lam_dc = LAM_L1 * (carr - f["carr"])
                    out.write(json.dumps({
                        "t": epoch, "prn": prn, "age_s": round(age, 2),
                        "pred_err_m": e, "drho_m": drho,
                        "lam_dcarr_m": lam_dc}) + "\n")
                    stats.setdefault(prn, []).append((age, e))
                fresh[prn] = {"t": epoch, "rho": rho, "carr": carr}
            prev[prn] = {"rho": rho, "carr": carr, "lock_s": lock}
        if time.time() - last_summary > SUMMARY_EVERY_S:
            last_summary = time.time()
            for prn, es in sorted(stats.items()):
                if not es:
                    continue
                ae = sorted(abs(e) for _, e in es)
                med = ae[len(ae) // 2]
                p90 = ae[min(len(ae) - 1, int(0.9 * len(ae)))]
                same = sum(1 for _, e in es if abs(e) < 1000.0) / len(es)
                print(f"shadow PRN {prn:2}: n={len(es)} median|e|={med:8.1f} m "
                      f"p90={p90:8.1f} m within-1km={same:.2f}",
                      flush=True)


if __name__ == "__main__":
    main()
