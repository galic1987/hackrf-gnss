import math

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
    import json
    try:
        with open("/Volumes/Radiator 8TB/gnss/observations/site.json") as f:
            d = json.load(f)
        return llh_to_ecef(d["lat"], d["lon"], d["h_m"])
    except Exception:
        return None

def calc_geo_doppler_hz(geonav, site_ecef, f_carrier):
    if not geonav or not site_ecef:
        return 0.0
    try:
        sx, sy, sz = site_ecef
        px, py, pz = geonav["pos_m"]
        vx, vy, vz = geonav["vel_mps"]
        dx, dy, dz = px - sx, py - sy, pz - sz
        rho = math.sqrt(dx**2 + dy**2 + dz**2)
        range_rate = (dx*vx + dy*vy + dz*vz) / rho
        agf1 = geonav["agf1"]
        c = 299792458.0
        return (-range_rate / c + agf1) * f_carrier
    except Exception:
        return 0.0
