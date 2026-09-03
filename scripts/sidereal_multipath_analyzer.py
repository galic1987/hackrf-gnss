#!/usr/bin/env python3
"""Sidereal Multipath Stacking (HMS) & Spatial Reflection Analyzer.

Constructs a Hemispherical Multipath Stacking (HMS) grid over 10+ sidereal days
of multi-constellation carrier tracking from observations/sky_history.jsonl:
  1. Derives the non-parametric smooth antenna elevation gain profile C/N0(el).
  2. Isolates multipath interference fringes: ΔC/N0(az, el) = C/N0_obs - C/N0_baseline(el).
  3. Grids the upper hemisphere into 10° Azimuth × 5° Elevation spatial bins.
  4. Stacks observations across sidereal repeat passes to compute Multipath Index (MPI).
  5. Maps azimuthal reflection sectors to identify physical building/roof bounce surfaces.
  6. Evaluates real-time multipath risk for each currently tracked satellite.
  7. Publishes observations/state.multipath.json atomically for live fusion into /api/sync.
"""
import argparse
import json
import math
import os
import sys
import time
import numpy as np

SKY_HISTORY_PATH = "/Volumes/Radiator 8TB/gnss/observations/sky_history.jsonl"
STATE_SKY_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.sky.json"
CACHE_GRID_PATH = "/Volumes/Radiator 8TB/gnss/observations/multipath_grid_cache.json"
STATE_MULTIPATH_PATH = "/Volumes/Radiator 8TB/gnss/observations/state.multipath.json"

OCTANTS = [
    ("North (N)", 337.5, 22.5),
    ("North-East (NE)", 22.5, 67.5),
    ("East (E)", 67.5, 112.5),
    ("South-East (SE)", 112.5, 157.5),
    ("South (S)", 157.5, 202.5),
    ("South-West (SW)", 202.5, 247.5),
    ("West (W)", 247.5, 292.5),
    ("North-West (NW)", 292.5, 337.5)
]


def load_sky_observations(history_path=SKY_HISTORY_PATH, subsample=2, max_frames=None):
    """Load and filter valid locked satellite observations from sky history."""
    if not os.path.exists(history_path):
        return None, 0.0, 0

    obs = []
    t_first = None
    t_last = None
    frame_count = 0

    with open(history_path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            if max_frames and frame_count >= max_frames:
                break
            if subsample > 1 and i % subsample != 0:
                continue
            if not line.strip():
                continue
            try:
                frame = json.loads(line)
                t = frame.get("t")
                if t_first is None and t is not None:
                    t_first = t
                if t is not None:
                    t_last = t
                frame_count += 1

                for s in frame.get("sats", []):
                    # Gate on locked channels with valid C/N0
                    if s.get("cls") == "tracked" and s.get("lock_s", 0) >= 10.0:
                        cn0 = s.get("cn0")
                        el = s.get("el_deg")
                        az = s.get("az_deg")
                        if cn0 is not None and 15.0 <= cn0 <= 60.0 and el is not None and el >= 5.0 and az is not None:
                            obs.append((float(az), float(el), float(cn0), s.get("sys", "gnss"), s.get("prn", 0)))
            except Exception:
                continue

    span_hours = (t_last - t_first) / 3600.0 if (t_first and t_last) else 0.0
    return obs, span_hours, frame_count


def compute_elevation_baseline(obs):
    """Derive smooth non-parametric elevation gain baseline."""
    if not obs:
        return {}

    els = np.array([o[1] for o in obs])
    cn0s = np.array([o[2] for o in obs])

    el_bins = np.arange(0, 92, 2)
    baseline = {}
    valid_mids = []
    valid_medians = []
    for k in range(len(el_bins) - 1):
        el_mid = (el_bins[k] + el_bins[k+1]) / 2.0
        m = (els >= el_bins[k]) & (els < el_bins[k+1])
        if np.any(m):
            med = float(np.median(cn0s[m]))
            valid_mids.append(el_mid)
            valid_medians.append(med)
            baseline[int(el_mid)] = round(med, 2)

    # Smoothly interpolate missing boundary bins from valid data
    if valid_mids:
        for k in range(len(el_bins) - 1):
            el_mid = int((el_bins[k] + el_bins[k+1]) / 2.0)
            if el_mid not in baseline:
                baseline[el_mid] = round(float(np.interp(el_mid, valid_mids, valid_medians)), 2)

    return baseline


def lookup_baseline(el_deg, baseline):
    """Lookup expected baseline C/N0 for a given elevation."""
    if not baseline:
        return 38.0
    # Find closest bin
    closest_el = min(baseline.keys(), key=lambda k: abs(k - el_deg))
    return baseline[closest_el]


def build_multipath_grid(obs, baseline, az_step=10, el_step=5):
    """Grid the hemisphere into az x el spatial bins and stack anomalies."""
    if not obs:
        return {}, {}, 0.0

    azs = np.array([o[0] for o in obs])
    els = np.array([o[1] for o in obs])
    cn0s = np.array([o[2] for o in obs])

    # Compute expected C/N0 for every point
    expected_cn0s = np.array([lookup_baseline(el, baseline) for el in els])
    delta_cn0 = cn0s - expected_cn0s

    overall_mpi = float(np.sqrt(np.mean(delta_cn0**2)))

    # Spatial Grid
    grid = {}
    az_bins = np.arange(0, 360, az_step)
    el_bins = np.arange(5, 90, el_step)

    for az_start in az_bins:
        az_end = az_start + az_step
        m_az = (azs >= az_start) & (azs < az_end)
        for el_start in el_bins:
            el_end = el_start + el_step
            m_cell = m_az & (els >= el_start) & (els < el_end)
            cnt = int(np.sum(m_cell))
            if cnt >= 5:
                cell_deltas = delta_cn0[m_cell]
                mpi = float(np.sqrt(np.mean(cell_deltas**2)))
                mean_anom = float(np.mean(cell_deltas))
                ptp = float(np.ptp(cell_deltas))
                status = "CLEAN" if mpi < 2.5 else ("MODERATE" if mpi < 4.0 else "SEVERE")

                key = f"{int(az_start)}_{int(el_start)}"
                grid[key] = {
                    "az_deg": int(az_start + az_step / 2),
                    "el_deg": int(el_start + el_step / 2),
                    "hits": cnt,
                    "mpi_db": round(mpi, 2),
                    "mean_anomaly_db": round(mean_anom, 2),
                    "fringe_ptp_db": round(ptp, 2),
                    "status": status
                }

    # Octants
    sectors = {}
    for name, a0, a1 in OCTANTS:
        if a0 > a1:
            m = (azs >= a0) | (azs < a1)
        else:
            m = (azs >= a0) & (azs < a1)

        cnt = int(np.sum(m))
        if cnt > 0:
            sec_deltas = delta_cn0[m]
            mpi = float(np.sqrt(np.mean(sec_deltas**2)))
            ptp = float(np.ptp(sec_deltas))
            status = "CLEAN" if mpi < 3.3 else ("MODERATE" if mpi < 3.7 else "SEVERE")
            sectors[name] = {
                "hits": cnt,
                "mpi_db": round(mpi, 2),
                "fringe_ptp_db": round(ptp, 1),
                "status": status
            }

    return grid, sectors, overall_mpi


class SiderealMultipathAnalyzer:
    def __init__(self, history_path=SKY_HISTORY_PATH, state_sky_path=STATE_SKY_PATH,
                 cache_path=CACHE_GRID_PATH, out_path=STATE_MULTIPATH_PATH):
        self.history_path = history_path
        self.state_sky_path = state_sky_path
        self.cache_path = cache_path
        self.out_path = out_path

        self.grid = {}
        self.sectors = {}
        self.baseline = {}
        self.overall_mpi = 0.0
        self.total_samples = 0
        self.span_hours = 0.0

    def load_or_rebuild_cache(self, force_rebuild=False):
        """Load stacked grid from cache or build from sky history."""
        cache_valid = False
        if not force_rebuild and os.path.exists(self.cache_path):
            try:
                cache_mtime = os.path.getmtime(self.cache_path)
                hist_mtime = os.path.getmtime(self.history_path) if os.path.exists(self.history_path) else 0
                if cache_mtime >= hist_mtime:
                    with open(self.cache_path, "r", encoding="utf-8") as f:
                        cache_data = json.load(f)
                    self.grid = cache_data.get("grid", {})
                    self.sectors = cache_data.get("sectors", {})
                    self.baseline = {int(k): v for k, v in cache_data.get("baseline", {}).items()}
                    self.overall_mpi = cache_data.get("overall_mpi", 0.0)
                    self.total_samples = cache_data.get("total_samples", 0)
                    self.span_hours = cache_data.get("span_hours", 0.0)
                    cache_valid = True
            except Exception:
                cache_valid = False

        if not cache_valid:
            obs, span_hours, _ = load_sky_observations(self.history_path, subsample=2)
            if not obs:
                return False
            self.total_samples = len(obs)
            self.span_hours = span_hours
            self.baseline = compute_elevation_baseline(obs)
            self.grid, self.sectors, self.overall_mpi = build_multipath_grid(obs, self.baseline)

            # Write cache
            cache_data = {
                "updated_at": time.time(),
                "total_samples": self.total_samples,
                "span_hours": round(self.span_hours, 2),
                "overall_mpi": round(self.overall_mpi, 2),
                "baseline": self.baseline,
                "sectors": self.sectors,
                "grid": self.grid
            }
            tmp_cache = self.cache_path + ".tmp"
            with open(tmp_cache, "w", encoding="utf-8") as f:
                json.dump(cache_data, f)
            os.replace(tmp_cache, self.cache_path)

        return True

    def evaluate_live(self):
        """Read state.sky.json, cross-reference with HMS grid, publish state.multipath.json."""
        if not self.grid and not self.load_or_rebuild_cache():
            return None

        # Cleanest and most reflective sectors
        cleanest_name = min(self.sectors.keys(), key=lambda k: self.sectors[k]["mpi_db"]) if self.sectors else "—"
        reflective_name = max(self.sectors.keys(), key=lambda k: self.sectors[k]["mpi_db"]) if self.sectors else "—"

        active_sats = {}
        if os.path.exists(self.state_sky_path):
            try:
                with open(self.state_sky_path, "r", encoding="utf-8") as f:
                    sky_data = json.load(f)
                live_sats = sky_data.get("sky", {}).get("sats", [])
                for s in live_sats:
                    if s.get("cls") == "tracked":
                        az = s.get("az_deg")
                        el = s.get("el_deg")
                        cn0 = s.get("cn0")
                        sat_id = f"{s.get('sys', 'GNSS').upper()}_{s.get('prn')}"
                        if az is not None and el is not None and el >= 5.0:
                            # Map to grid bin
                            az_bin = int(az // 10) * 10
                            el_bin = int(el // 5) * 5
                            key = f"{az_bin}_{el_bin}"
                            cell = self.grid.get(key, {})
                            mpi = cell.get("mpi_db", self.overall_mpi)

                            expected_cn0 = lookup_baseline(el, self.baseline)
                            delta_cn0 = round(cn0 - expected_cn0, 1) if cn0 is not None else None

                            risk = "LOW" if mpi < 3.2 else ("ELEVATED" if mpi < 3.8 else "HIGH")
                            active_sats[sat_id] = {
                                "sys": s.get("sys"),
                                "prn": s.get("prn"),
                                "az_deg": round(az, 1),
                                "el_deg": round(el, 1),
                                "cn0": cn0,
                                "expected_cn0": expected_cn0,
                                "delta_cn0_db": delta_cn0,
                                "grid_mpi_db": round(mpi, 2),
                                "grid_hits": cell.get("hits", 0),
                                "reflection_risk": risk
                            }
            except Exception:
                pass

        output = {
            "epoch": round(time.time(), 2),
            "observation_span_days": round(self.span_hours / 24.0, 1),
            "total_samples_stacked": self.total_samples,
            "overall_mpi_db": round(self.overall_mpi, 2),
            "cleanest_sector": cleanest_name,
            "cleanest_mpi_db": self.sectors.get(cleanest_name, {}).get("mpi_db", 0.0),
            "most_reflective_sector": reflective_name,
            "most_reflective_mpi_db": self.sectors.get(reflective_name, {}).get("mpi_db", 0.0),
            "sectors": self.sectors,
            "grid_cells_count": len(self.grid),
            "grid": self.grid,
            "active_satellites": active_sats
        }

        # Atomically write state file
        tmp_out = self.out_path + ".tmp"
        with open(tmp_out, "w", encoding="utf-8") as f:
            json.dump(output, f, indent=2)
        os.replace(tmp_out, self.out_path)

        return output


def main():
    parser = argparse.ArgumentParser(description="Sidereal Multipath Stacking (HMS) Analyzer")
    parser.add_argument("--once", action="store_true", help="Run once and exit")
    parser.add_argument("--loop", action="store_true", help="Run continuously in a loop")
    parser.add_argument("--interval", type=float, default=10.0, help="Loop interval in seconds")
    parser.add_argument("--rebuild-cache", action="store_true", help="Force rebuild of HMS grid cache")
    args = parser.parse_args()

    analyzer = SiderealMultipathAnalyzer()
    analyzer.load_or_rebuild_cache(force_rebuild=args.rebuild_cache)

    while True:
        res = analyzer.evaluate_live()
        if not args.loop:
            break
        time.sleep(args.interval)

    print("=================================================================")
    print("      SIDEREAL MULTIPATH STACKING (HMS) SKY MAP ANALYZER         ")
    print("=================================================================")
    if not res:
        print("Failed to compute multipath grid.")
        return

    print(f"Observation Span:        {res['observation_span_days']} days ({res['total_samples_stacked']} stacked passes)")
    print(f"Overall Multipath Index: {res['overall_mpi_db']} dB")
    print(f"Cleanest Sector:         {res['cleanest_sector']} (MPI: {res['cleanest_mpi_db']:.2f} dB)")
    print(f"Most Reflective Sector:  {res['most_reflective_sector']} (MPI: {res['most_reflective_mpi_db']:.2f} dB)")
    print(f"HMS Sky Cells Mapped:    {res['grid_cells_count']}")
    print("-----------------------------------------------------------------")
    print("ACTIVE SATELLITE MULTIPATH RISK:")
    for sat_id, sat in sorted(res["active_satellites"].items()):
        d_str = f"{sat['delta_cn0_db']:+.1f} dB" if sat['delta_cn0_db'] is not None else "—"
        print(f"  {sat_id:12s} Az:{sat['az_deg']:5.1f}° El:{sat['el_deg']:4.1f}° | Grid MPI:{sat['grid_mpi_db']:4.2f} dB (ΔC/N0: {d_str}) -> {sat['reflection_risk']}")
    print("=================================================================")
    print(f"State File:              {STATE_MULTIPATH_PATH}")


if __name__ == "__main__":
    main()
