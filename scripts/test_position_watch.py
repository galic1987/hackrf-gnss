#!/usr/bin/env python3
"""Unit tests for position_watch's plausibility gate (review round 4:
impossible-altitude solves used to enter position_history ungated).
Plain asserts, no pytest; writes only to a temp dir.

  python3 scripts/test_position_watch.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import position_watch as pw

FAILURES = []


def check(name, cond, detail=""):
    print(f"{'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILURES.append(name)


def main():
    good = {"lat": 39.003, "lon": -77.606, "alt_km": 0.02}
    check("normal fix accepted", pw.fix_sane(good))
    check("road-level negative alt accepted", pw.fix_sane({**good, "alt_km": -0.05}))
    check("mountain road accepted", pw.fix_sane({**good, "alt_km": 3.0}))
    check("space rejected", not pw.fix_sane({**good, "alt_km": 100.0}))
    check("deep earth rejected", not pw.fix_sane({**good, "alt_km": -5.0}))
    check("missing alt rejected", not pw.fix_sane({"lat": 39.0, "lon": -77.6}))
    check("non-float alt rejected", not pw.fix_sane({**good, "alt_km": "high"}))
    check("off-planet lat rejected", not pw.fix_sane({**good, "lat": 123.0}))

    print(f"\n{len(FAILURES)} failure(s)" if FAILURES else "\nall tests passed")
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
