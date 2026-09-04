# Photon Debugger design QA

## Inputs

- Approved source: `/Users/ivo/.codex/generated_images/01a0369e-0ee8-7511-a749-e24aae104ff0/exec-e11a1b73-af9a-4fb6-b43b-127cc0864626.png`
- Implementation: `/Volumes/Radiator 8TB/gnss/hackrf_gnss/web/story.html`
- Deterministic implementation capture: `/tmp/photon-qa-v4.png`
- Live implementation capture: `/tmp/photon-live-v4.png`
- Exact mobile-emulation capture: `/tmp/photon-mobile-v4.png`
- Full side-by-side comparison: `/tmp/photon-compare-v4-full.png`
- Focused top comparison: `/tmp/photon-compare-v4-top.png`
- Focused lower comparison: `/tmp/photon-compare-v4-bottom.png`

## Viewports and state

| Capture | CSS viewport | Pixel dimensions | State |
|---|---:|---:|---|
| Approved source | 1487 × 1058 | 1487 × 1058 | Selected GNSS path with populated inspector, replay trace, provenance, and comparison tray |
| Deterministic implementation | 1487 × 1058 | 1487 × 1058 | `?qa=1`; GPS 12 selected, BeiDou 28 pinned, current sample, 30 qualified browser-session carrier points, sampled slip marker |
| Live implementation | 1487 × 1058 | 1487 × 1058 | Same-origin `/api/sync`; processor rows arriving while the RF sample stream is backlogged, so current observations and cross-stage model joins are withheld |
| Mobile implementation | 390 × 844 | 390 × 844 | CDP device metrics, `innerWidth === 390`, `max-width:430px` matched, deterministic QA state, scroll width 390 |

Core desktop dimensions: 66 px status header, 205 px chapter rail, 338 px inspector, 124 px hero, nine 46 px signal rows, and an 82 px carrier plot. Mobile controls use a minimum 44 px touch height.

## Comparison history

1. Initial comparison exposed an over-tall hero, seven-stage compression, a sparse replay fixture, an empty comparison tray, truncated status values, and lower panels extending past the reference viewport.
2. The second pass separated antenna/RF and ADC/FPGA concerns, restored the full nine-row source rhythm, added a dense deterministic trace and qualified comparison pair, shortened the evidence table, and reserved the mobile Replay action.
3. The third pass compacted the hero but clipped the selector and cropped away the satellite-to-Earth context.
4. The final pass anchors the selector, uses the generated satellite/Earth raster across the hero, keeps all evidence panels within the desktop frame, and preserves a collision-free mobile hero. Full and focused side-by-side inspections show no remaining P0 or P1 visual mismatch.

## Final review

- Layout and hierarchy: passed. Status strip, narrative rail, photon-path workbench, inspector, carrier window, uncertainty ledger, provenance ledger, and compare tray follow the approved composition.
- Visual language: passed. Navy/black field, cyan signal traces, amber warnings, compact technical typography, borders, radii, and density match the selected direction.
- Asset fidelity: passed. The hero uses the generated satellite/Earth raster; no placeholder, emoji, handcrafted SVG, or CSS illustration substitutes for it.
- Core interactions: passed. Satellite selection, stage navigation, evidence tabs, comparison selection/toggle, Replay, offline recovery, and keyboard tab movement are exercised by the deterministic browser test.
- Responsive behavior: passed. Exact 390 × 844 emulation has no horizontal overflow; Replay remains visible; the selector and evidence card do not overlap; controls meet the 44 px mobile target.
- Scientific integrity: passed. Unsupported mock values were intentionally replaced with explicit unavailable, diagnostic, model, or experimental states. Processor-output freshness is distinct from RF sample freshness; stale samples and non-co-temporal model joins are withheld.
- Runtime boundary: passed. The page polls only same-origin `/api/sync`, pauses when hidden, allows one request in flight, clears active evidence on failure, and does not touch radio or firmware processes.
- Verification: `node web/smoke_story.js` passed; `git diff --check` passed; `/story`, `/story.html`, and `/api/sync` returned HTTP 200; the served story matched the workspace file.

final result: passed
