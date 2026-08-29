# P0b Acceptance — Reissued Artifact (supersedes the 16:05 ledger claim)

2026-08-29 19:00 EDT. Round-19 review found the advertised acceptance
number ("corrected cross-PRN separation 2.36e-7 ppm, ~9000× improvement")
not self-reproducing. Recomputation confirms: the number is real
arithmetic on the wrong statistic. This artifact replaces it.

## What was claimed vs what reproduces

| statistic | advertised | recomputed | verdict |
|---|---|---|---|
| unpaired-median separation, corrected | 2.36e-7 ppm | 2.355e-7 ppm | reproduces exactly — but is the wrong metric |
| unpaired-median separation, uncorrected | 2.12e-3 ppm | 2.117e-3 ppm | reproduces |
| **paired-epoch diff, corrected** | — | **median −2.84e-5 ppm, MAD 1.34e-4** | the honest metric |
| paired-epoch diff, uncorrected | — | median −2.09e-3 ppm, MAD 1.39e-4 | the honest baseline |
| improvement | ~9000× | **~74×** (2.09e-3 / 2.84e-5) | retracted and corrected |

The unpaired medians (−2.43865e-5 vs −2.41510e-5) coincide because the
two chains sampled different epoch distributions (3408 vs 3138 rows,
only 2644 common epochs) against a drifting common clock. Same-epoch
differencing is the only valid cross-GEO closure test.

## Paired-epoch results (2644 common epochs, acceptance slice)

- Corrected diff is stable across the slice, not zero:
  t+00–15 min +2.8e-5; t+15–30 −9.3e-5; t+30–46 −4.5e-5; t+60–81 −3.4e-5.
- MAD unchanged by the correction (1.34e-4 vs 1.39e-4) — expected: the
  correction is quasi-static; it does not reduce per-epoch fit noise.
- Whole-file paired check (15905 rows incl. post-acceptance era):
  median −3.18e-5 ppm — the ~3e-5 closure level is holding, not
  degrading, but it is 3e-5, not 1e-7.

## Conclusion

- P0b removes the 2e-3 ppm diurnal GEO-motion term down to a residual
  inter-GEO bias of order **3e-5 ppm** (wandering ±1e-4 over tens of
  minutes). That is the demonstrated accuracy level of the correction.
- Promotion gate (<2e-4 ppm cross-GEO separation) still passes by ~7×
  on the honest metric. The promotion (commit 0000c8f) stands.
- The "9000× / 2.36e-7 ppm" figure is **retracted**: it is an
  unpaired-median artifact, not cross-GEO closure.
- Inter-GEO disagreement (~3e-5 ppm) is the same order as the receiver
  clock signal itself (−2.4e-5 ppm). Consequences:
  - P0b output stays **observe-only**. No discipline use.
  - Consensus across GEOs must reflect the inter-GEO bias, not just
    per-chain scatter sigmas.
  - Tier-1 vote promotion remains blocked pending a second,
    independently validated GEO and MT9-oracle validation.

## Provenance (self-reproducing)

- Slice: first 6546 rows of `gnss/observations/phase_drift_p0b_shadow.jsonl`
  (append-only; slice is prefix-stable as the file grows).
- Slice SHA256 (first 6546 lines, LF):
  `b99b21ea787af895f34c289449e14eef9bdf0dcf886b1d682391c6e8acc7c47e`
- Epochs: 1788027979.2 → 1788032840.4 (2026-08-29 14:26:19 → 15:47:20 EDT).
- Site anchor SHA256 (`gnss/observations/site.json`):
  `455ed826fc5bd5a7161b43c11b59bbb6253bb0544b2cabd8405506eb35ae9f9f`
- The slice spans the 15:24 EDT tracker restart (787 s outage,
  1788030785 → 1788031572) and 3 unique-epoch gaps > 5 s total. The
  phase producer ran continuously across it (fit_hist persists in
  producer memory); disclosed, not hidden.
- Filters: none beyond the producer's own fail-closed emission; rows as
  written. PRN 131 n=3408, PRN 135 n=3138, paired epochs 2644.
- Calculation (run from `/Volumes/Radiator 8TB/gnss`):

```python
import json, statistics
from collections import defaultdict
rows=[json.loads(l) for l in open("observations/phase_drift_p0b_shadow.jsonl")]
s=rows[:6546]
by=defaultdict(dict)
for r in s: by[r["epoch"]][r["prn"]]=r
paired={e:d for e,d in by.items() if 131 in d and 135 in d}
eps=sorted(paired)
dc=[paired[e][131]["p0b_ppm"]-paired[e][135]["p0b_ppm"] for e in eps]
du=[paired[e][131]["uncorr_ppm"]-paired[e][135]["uncorr_ppm"] for e in eps]
# median/MAD of dc and du are the acceptance statistics
```
