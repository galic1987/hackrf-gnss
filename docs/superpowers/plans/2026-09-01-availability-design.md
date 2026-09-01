# Availability Design — breaking the n_sat>=5 wall (2026-09-01)

Three-analyst panel + synthesis over 24 h of sky_history (2,890 epochs), 9 days of
satellite.parquet (2.59 M rows), live v4 clock_bias.jsonl (5,495 rows), and the
v4 solve path read end-to-end. Adversarially grounded; projection arithmetic was
in the session scratchpad (avail_design.json) — headline numbers reproduced here.

## The loss waterfall (median satellites per epoch)

| Stage | Sats | Loss |
|---|---|---|
| Sky offers (el>10, GPS+GAL+BDS; GLONASS untrackable) | 25 (+2 WAAS GEO; min 20) | — |
| Above the MEASURED building mask (N/NE/NW blocked to el 45–65) | 19 (+2; min 13) | −6 |
| Tracker locks | 9 (+2 SBAS; p10 7, p90 12) | −10 |
| **Ranging-capable (rho_m present; GPS+BDS only, by construction)** | **3.68 mean** | **−5.3 ← decides the gate** |
| Solve keeps (BDS rejection only 2.76% of sat-epochs) | n_sat mode 4 | ≈0 |
| Quality (n_sat>=5, slips==0) | 9.7% of span; best run 193 s | — |

Sky is NEVER binding: 0 of 2,893 epochs offered fewer than 5 above the measured
mask. The solver and A/B gate are never binding (<1%). The mission-deciding loss
is **tracked → ranged**: satellites the engine already locks contribute no
pseudorange.

## Structural facts

- **Galileo is tracked-but-unsolved**: E1B locks a mean 1.64 sats/epoch (up to 30
  PRNs/day) but live.rs establishes the nav anchor only in the GPS-LNAV and
  BDS-D1 arms (`_ => 0`), so rho_m is null on 100% of 298,027 tracked GAL rows,
  and clock_bias.rs:435-439 filters sys to {gps, beidou} anyway.
- **WAAS GEOs (PRN 131/135) are locked ~100% of the time**, stationary in the
  OPEN southern sky (immune to the building), MT9 ephemeris already decoded —
  used only as correction sources, never ranging.
- **Producer instability is the second drain**: 115 restarts in 8.8 days
  (202 stream-EOF events), median 498 s to first lock despite the seed cache —
  14.8% of wall time at zero locks.
- **The strict slips==0 gate makes a clean hour mathematically unreachable**:
  per-sat slip rate 0.26%/epoch ⇒ at 7.3 sats ~1.9% of epochs fail ⇒
  P(clean hour) ≈ 1e-11 even with perfect n_sat.
- The A/B membership gate silently `continue`s on mismatch, so
  ab_membership_match:true is tautological (5,495/5,495) — mismatches are
  currently unobservable.
- North-half sky: 54.9% of visible sat-epochs, lock rate 20.5% vs 44.1% south;
  passes acquired/lost at median el ~42° (culmination third only, median 17 min).

## Ranked levers

| # | Lever | Gain | Effort | Owner |
|---|---|---|---|---|
| 1 | Galileo E1B ranging (I/NAV TOW anchor, RINEX GAL eph + BGD, drop sys filter) | +1.64 sats; P(n_sat≥5) 0.371→0.598 alone | fleet-code (largest: I/NAV decode) | fleet |
| 2 | SBAS GEO ranging (PRN 131/135; MT9 eph already decoded) | +1.7 sats at ~100% duty, building-immune | fleet-code (small) | fleet |
| 3 | Producer stability (fix stream-EOF restarts, fast hot-recovery) + slip-tolerant gate (drop the slipped sat, keep the epoch; safe at n_sat≥6) | recovers ~4.5 h/day; MANDATORY for the hour | fleet-code | fleet |
| 4 | Acquisition tuning (rediscovery 900→120-300 s; threshold 2.5→~2.0 seeded) | attacks the −10 sky→tracker loss; est. +1–3 tracked | config/fleet | fleet |
| 5 | Antenna re-siting to full-sky/zenith view (the GPS patch — the ClearStream on the One is the ATSC anchor, not a GNSS lever) | +14 visible, ~+4 tracked; margin + geometry, NOT required for the hour | bench, scheduled window | operator |
| 6 | A/B gate: publish flagged (ab_membership_match:false + both n_sat) instead of silent skip | ~0 availability; pure observability | trivial | fleet |
| 7 | BDS iono via frequency-scaled SBAS grid (×(1575.42/1561.098)² = 1.01845) | accuracy (2–15 m), ~0 availability; physically sound, label uncertified | small | fleet |
| 8 | Elevation mask tuning | NOT a lever — no configured mask binds; the knob is #4 | — | — |

## Projection (levers 1–3)

Projected published n_sat {5:2%, 6:19%, 7:39%, 8:28%, 9:9%, 10:2%}, mean 7.3,
P(≥5)=1.00, P(≥6)=0.98. Measured counterfactual: a real 420 s total-outage window
re-scored with GAL+SBAS ranging → 412/412 epochs publishable, 95.4% quality.
Longest quality segment: 193 s today → tens of minutes on 1+2 → hour-scale once
restarts and slip handling (3) land. Fallback if strict slips==0 is
non-negotiable: re-register leg-1 as "≥98% quality epochs over 3,600 s, no gap
>30 s" — reachable on 1+2 alone.

## Acceptance criteria

- Lever 1: GAL rho_m non-null on live rows; residual RMS not degraded (GGTO/ISB
  folds into the existing unmodeled ISB — watch it); A/B gate exercised.
- Lever 2: PRN 131/135 rho rows flagged as GEO-class for leg-1 weighting.
- Lever 3: restarts <2/day or hot-recovery <5 s; slip-drop only at n_sat≥6.
- Lever 6: at least one flagged mismatch observed in a week, or the gate is
  provably never tripping (either answer is information — today it's neither).
