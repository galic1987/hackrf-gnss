# Sync panel "Academy" — panel pass 2 design

Date: 2026-08-25. Status: user-approved design, awaiting implementation
(starts after the sync.html redesign pass lands — same file, serialized).

## Goal

Turn `web/sync.html` into a "live visual academy": the dashboard stays dense
and correct, and every view also teaches — from zero knowledge up — using the
station's own live data wherever it exists.

## Hard constraints (inherited)

- Single self-contained `web/sync.html`: inline CSS/JS, NO external
  CDN/fonts/libs (offline box; tiny_http serves fixed routes).
- Preserve every existing section and the render pipeline (`render(st)`,
  `tick()`, `heartbeat()`, `annotate(root)` + `data-tip` tooltips).
- 3D = hand-rolled canvas projection (drag rotate/zoom), no WebGL.
- Live where real, storybook where not. Tooltips everywhere: bold title +
  1–3 sentence plain explanation; keyboard-focusable.
- Labels follow the review vocabulary: proven / diagnostic / projected.

## Dashboard features (from user asks, 2026-08-25)

1. Scatter: last 60 fixes @1 Hz per constellation (augment/replace 3-h decay).
2. Merge scat_xy/xz/yz into ONE rotatable 3D scatter, animated replay of the
   last 60 epochs ("playing" the recent positions).
3. Sky "log map": polar/dome view of the visible sky patch with the learned
   mask + 30-min encounter trails (sky_producer must add a recent-history
   field; data exists in `observations/sky_history.jsonl`).
4. Satellite table: add altitude, speed, direction (ephemeris-derived ECEF
   position/velocity; extend `scripts/sky_producer.py` — separate file).
5. Chart QA sweep: every canvas visible, labeled, working.

## Academy views (all live-parameterized where data exists)

A. Sky dome (3D): hemisphere of our sky patch, mask bins as terrain, sats
   moving in real time, constellation colors, Doppler hue shift; hover =
   PRN/el/doppler/lock/alt/speed/heading.
B. Wave arrival + Doppler stretching: expanding wavefronts sat→antenna at
   real range (~20,200 km) and travel time (~67 ms); wave train compresses/
   stretches with live `doppler_hz`.
C. Phase → distance: 19 cm carrier vs 300 m code chip animation, fed by live
   `carrier_cycles`/`phase_frac`; why phase is mm-precise but
   integer-ambiguous.
D. Inside the HackRF Pro: animated block diagram — antenna → MAX2831/MAX5864
   → SGPIO/CPLD → FPGA (CIC, nibble timestamps, TDC) → M0 → M4 → USB → PC,
   live values bolted on (tick rate, sample rate, correction ppm, overflows).
E. Sync-method leaderboard: measured σ per source (GPS C/A, SBAS-corrected,
   BDS, GAL, ATSC carrier, PC clock) from band_series + consensus, bars with
   error bars + "what each adds" deltas. Rigorous SBAS on/off A/B is a
   separate retro-analysis task (4-sat RMS is zero by construction — do NOT
   use it as the quality gate).

## Curriculum layers ("assume nothing"; collapsible learn-panels + tooltips + guided tour)

- L0 Quantum: Cs-133 hyperfine 9,192,631,770 Hz = the second; Rb clocks on
  sats; relativity pre-offset 10.22999999543 MHz (GR +45.7 µs/day, SR
  −7.2 µs/day; ~10 km/day error if ignored). Animated ground-vs-orbit clocks.
- L1 Physics: c=λf per band; RHCP/patch antenna; link budget (~−160 dBW,
  below noise — CDMA despreading animation); Doppler; iono (dispersive,
  1–15 m, MT26 grid shown live) vs tropo (~2.3 m, non-dispersive); multipath.
- L2 Math: trilateration, 4th unknown = clock; pseudorange equation; least
  squares + residuals; GDOP as visible geometry; correlation peaks; Allan
  deviation.
- L3 Hardware: sat payload; antenna; HackRF Pro chain (view D); Si5351 clock
  tree under discipline.
- L4 Software: acquisition (Doppler×code heatmap), DLL/PLL/FLL (live loop
  behavior), nav decode, ephemeris, PVT solver (live residuals), discipline
  loop (live correction_ppm), producer/merge architecture.
- Wave gallery: one animated card per receivable signal — GPS L1 C/A
  (1575.42 MHz), BDS B1I (1561.098), GAL E1, SBAS L1 (GEO corrections),
  Iridium (1626 MHz LEO bursts), ATSC ch35 (~599 MHz passive phase ref) —
  real frequency/wavelength, what equipment makes it, our live band stats.

## Data sources (verified 2026-08-25 via /api/sync)

sky.sats[] {prn,sys,az_deg,el_deg,doppler_hz,cn0,lock_s,cls},
sky.counts {tracked,expected,observed,absent,below,...}, sky.mask,
tracker.sats[] {prn,sys,cn0_proxy,lock_s,doppler_hz,carrier_cycles,
phase_frac,rho_m,t_tx,ppm,...}, position {lat,lon,alt_km,residual_rms_m,
mode,gate,n_sbas_corr,n_lt_corr,n_iono_corr,gdop}, position_history[],
band_series{name→[[t,ppm]...]}, consensus, clock{live_tick_hz,recent,
residual_ppm,tick_rate_drift_ppm_vs_session_ref}, discipline{correction_ppm,
note,waas_locked,...}, phase{disp_mm,sigma_mm,freq_off_hz,series@60Hz}.

## Producer-side dependency (separate file, no conflict)

`scripts/sky_producer.py`: add per-sat altitude/speed/heading (ephemeris
velocity) and a rolling ~30-min recent-encounter list into state.sky.json.

## QA checklist

- node --check on the extracted script; curl /sync 200 + sane size.
- Every canvas non-blank against live /api/sync; every new element has a tip.
- Guided tour starts, steps, dismisses; page works with partial/absent
  producers (all "waiting" states intact).
