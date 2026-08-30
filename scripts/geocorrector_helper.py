import math
import json

_CACHED_SITE_ECEF = None

def llh_to_ecef(lat_deg, lon_deg, h_m):
    a = 6378137.0
    e2 = 0.00669437999014
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    n = a / math.sqrt(1.0 - e2 * math.sin(lat)**2)
    x = (n + h_m) * math.cos(lat) * math.cos(lon)
    y = (n + h_m) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - e2) + h_m) * math.sin(lat)
    return x, y, z

def get_site_ecef():
    global _CACHED_SITE_ECEF
    if _CACHED_SITE_ECEF is not None:
        return _CACHED_SITE_ECEF
    try:
        with open("/Volumes/Radiator 8TB/gnss/observations/site.json") as f:
            d = json.load(f)
        _CACHED_SITE_ECEF = llh_to_ecef(d["lat"], d["lon"], d["h_m"])
        return _CACHED_SITE_ECEF
    except Exception:
        return None

def calc_geo_doppler_hz(geonav, site_ecef, f_carrier, t_unix=None):
    """Compute line-of-sight Doppler (Hz) + clock drift for an SBAS GEO from MT9 GeoNav."""
    if not geonav or not site_ecef:
        return 0.0
    try:
        sx, sy, sz = site_ecef
        dt = 0.0
        if t_unix is not None and "t0_s" in geonav:
            # GPS time-of-day rollover propagation
            t_sow = (t_unix - 315964800 + 18) % 604800
            t_tod = t_sow % 86400
            dt = t_tod - geonav["t0_s"]
            if dt > 43200:
                dt -= 86400
            elif dt < -43200:
                dt += 86400

        dt2 = dt * dt
        acc = geonav.get("acc_mps2", [0.0, 0.0, 0.0])
        px = geonav["pos_m"][0] + geonav["vel_mps"][0] * dt + 0.5 * acc[0] * dt2
        py = geonav["pos_m"][1] + geonav["vel_mps"][1] * dt + 0.5 * acc[1] * dt2
        pz = geonav["pos_m"][2] + geonav["vel_mps"][2] * dt + 0.5 * acc[2] * dt2

        vx = geonav["vel_mps"][0] + acc[0] * dt
        vy = geonav["vel_mps"][1] + acc[1] * dt
        vz = geonav["vel_mps"][2] + acc[2] * dt

        dx, dy, dz = px - sx, py - sy, pz - sz
        rho = math.sqrt(dx * dx + dy * dy + dz * dz)
        range_rate = (dx * vx + dy * vy + dz * vz) / rho
        agf1 = geonav.get("agf1_sps", geonav.get("agf1", 0.0))
        c = 299792458.0
        # Doppler: receding satellite (range_rate > 0) lowers received frequency (-range_rate/c).
        # Satellite clock drift (agf1 > 0) increases transmitted frequency (+agf1).
        return (-range_rate / c + agf1) * f_carrier
    except Exception:
        return 0.0
