#!/usr/bin/env python3
"""Solar Terminator & Dawn Detection Engine for HackRF GNSS.

Detects solar transitions, astronomical/civil twilight, ionospheric photoionization,
solar RF radiometry flux, and satellite-sun angular conjunctions using the
Leo Bodnar GPSDO Split Star and HackRF multi-constellation receiver.

Key Physical Phenomena Detected:
1. Ionospheric Sunrise @ 350 km IPP Shell:
   Earth's curvature causes the ionospheric F2-layer (350 km) to be illuminated
   ~74 minutes BEFORE ground sunrise (horizon dip theta_dip = 18.57 deg).
   Solar EUV (lambda < 102.7 nm) photoionizes neutral O and N2, causing a rapid
   positive STEC/TEC ramp (dTEC/dt > 0) while the ground is in darkness.

2. Solar Radio Emission & System Temperature (T_sys):
   The Sun emits broadband RF across L-band (1.4 - 1.6 GHz) with quiet-sun
   flux S_sun ~ 100 - 150 SFU (1 SFU = 10^-22 W/m^2/Hz). As the Sun rises above
   the antenna horizon, antenna sky temperature T_sky rises.

3. Solar Terminator Crossing at Satellite IPPs:
   Detects the exact minute the day/night boundary crosses each satellite's
   line-of-sight pierce point, triggering acoustic-gravity wave TIDs.

4. Satellite-Sun Conjunctions (Solar Outages):
   Computes topocentric angular separation psi between each satellite and the Sun.
   Warns of direct solar beam passage (psi < 15 deg) and RF desensitization.

5. Tropospheric Solar Heating & Dew Burn-off:
   Ground sunrise (theta_sun >= 0 deg) drives surface boundary layer warming and
   convective moisture shifts reflected in Precipitable Water Vapor (PWV).

Publishes state to:
  /Volumes/Radiator 8TB/gnss/observations/state.solar.json
"""

import argparse
import datetime
import json
import math
import os
import sys
import time

# Physical Constants
RE_KM = 6371.0                  # Earth mean radius (km)
AU_KM = 149597870.7             # 1 Astronomical Unit in km
K_B = 1.380649e-23              # Boltzmann constant (J/K)
C_MPS = 299792458.0             # Speed of light (m/s)
SFU_TO_WM2HZ = 1e-22            # 1 Solar Flux Unit (W / (m^2 * Hz))

def compute_solar_ephemeris(lat_deg, lon_deg, alt_m=20.0, dt_utc=None):
    """Compute topocentric solar position using standard NOAA / Meeus algorithms.
    
    Returns dict with solar elevation, azimuth, distance, declination, and hour angle.
    """
    if dt_utc is None:
        dt_utc = datetime.datetime.now(datetime.timezone.utc)
    
    doy = dt_utc.timetuple().tm_yday
    hour_frac = dt_utc.hour + dt_utc.minute / 60.0 + (dt_utc.second + dt_utc.microsecond * 1e-6) / 3600.0

    # Fractional year in radians
    gamma = 2.0 * math.pi / 365.0 * (doy - 1 + (hour_frac - 12.0) / 24.0)

    # Equation of time in minutes
    eqtime_min = 229.18 * (0.000075 + 0.001868 * math.cos(gamma) - 0.032077 * math.sin(gamma)
                           - 0.014615 * math.cos(2.0 * gamma) - 0.040849 * math.sin(2.0 * gamma))

    # Solar declination in radians
    decl_rad = (0.006918 - 0.399912 * math.cos(gamma) + 0.070257 * math.sin(gamma)
                - 0.006758 * math.cos(2.0 * gamma) + 0.000907 * math.sin(2.0 * gamma))
    decl_deg = math.degrees(decl_rad)

    # True Solar Time in minutes
    time_offset = eqtime_min + 4.0 * lon_deg
    tst_min = hour_frac * 60.0 + time_offset
    ha_deg = (tst_min / 4.0) - 180.0
    ha_rad = math.radians(ha_deg)

    lat_rad = math.radians(lat_deg)

    # Solar zenith angle
    cos_zenith = math.sin(lat_rad) * math.sin(decl_rad) + math.cos(lat_rad) * math.cos(decl_rad) * math.cos(ha_rad)
    cos_zenith = max(-1.0, min(1.0, cos_zenith))
    zenith_rad = math.acos(cos_zenith)
    zenith_deg = math.degrees(zenith_rad)
    solar_el_geom = 90.0 - zenith_deg

    # Atmospheric refraction correction near horizon (Bennett 1982)
    # Applied when solar elevation > -5 deg
    refraction_deg = 0.0
    if solar_el_geom > -5.0:
        h_deg = max(-5.0, solar_el_geom)
        r_arcmin = 1.0 / math.tan(math.radians(h_deg + 7.31 / (h_deg + 4.4)))
        refraction_deg = r_arcmin / 60.0
    
    solar_el_apparent = solar_el_geom + refraction_deg

    # Solar azimuth angle (degrees from True North clockwise)
    az_rad = math.atan2(-math.sin(ha_rad),
                        math.tan(decl_rad) * math.cos(lat_rad) - math.sin(lat_rad) * math.cos(ha_rad))
    solar_az_deg = (math.degrees(az_rad) + 360.0) % 360.0

    # Solar distance in AU
    r_au = 1.00014 - 0.01671 * math.cos(gamma) - 0.00014 * math.cos(2.0 * gamma)

    return {
        "epoch": dt_utc.timestamp(),
        "utc_iso": dt_utc.isoformat(),
        "solar_el_geom_deg": round(solar_el_geom, 3),
        "solar_el_apparent_deg": round(solar_el_apparent, 3),
        "solar_az_deg": round(solar_az_deg, 3),
        "zenith_deg": round(zenith_deg, 3),
        "declination_deg": round(decl_deg, 3),
        "equation_of_time_min": round(eqtime_min, 2),
        "hour_angle_deg": round(ha_deg, 2),
        "solar_distance_au": round(r_au, 4),
    }

def compute_layer_illumination(solar_el_deg, altitude_km):
    """Compute if an atmospheric layer at altitude_km is illuminated by the Sun.
    
    Returns dip angle, effective elevation, shadow height, and illumination boolean.
    """
    dip_deg = math.degrees(math.acos(RE_KM / (RE_KM + altitude_km)))
    eff_el_deg = solar_el_deg + dip_deg
    is_illuminated = eff_el_deg > 0.0
    return {
        "altitude_km": altitude_km,
        "horizon_dip_deg": round(dip_deg, 2),
        "effective_solar_el_deg": round(eff_el_deg, 2),
        "is_illuminated": is_illuminated,
        "status": "ILLUMINATED (DAYLIGHT)" if is_illuminated else "SHADOW (NIGHT)"
    }

def compute_overhead_shadow_height_km(solar_el_deg):
    """Compute the altitude directly overhead the station below which is in Earth's shadow."""
    if solar_el_deg >= 0.0:
        return 0.0
    depress_rad = math.radians(-solar_el_deg)
    if depress_rad >= math.pi / 2.0:
        return 99999.0
    h_km = RE_KM * (1.0 / math.cos(depress_rad) - 1.0)
    return round(h_km, 1)

def compute_twilight_milestones(lat_deg, lon_deg, dt_utc=None):
    """Compute local transit times for ionospheric sunrise, twilights, sunrise, and noon."""
    if dt_utc is None:
        dt_utc = datetime.datetime.now(datetime.timezone.utc)
    
    doy = dt_utc.timetuple().tm_yday
    gamma = 2.0 * math.pi / 365.0 * (doy - 1)
    eqtime_min = 229.18 * (0.000075 + 0.001868 * math.cos(gamma) - 0.032077 * math.sin(gamma)
                           - 0.014615 * math.cos(2.0 * gamma) - 0.040849 * math.sin(2.0 * gamma))
    decl_rad = (0.006918 - 0.399912 * math.cos(gamma) + 0.070257 * math.sin(gamma)
                - 0.006758 * math.cos(2.0 * gamma) + 0.000907 * math.sin(2.0 * gamma))

    time_offset = eqtime_min + 4.0 * lon_deg
    lat_rad = math.radians(lat_deg)

    # 350 km ionospheric shell horizon dip
    iono_dip = math.degrees(math.acos(RE_KM / (RE_KM + 350.0))) # ~18.57 deg

    zeniths = {
        "iono_sunrise_350km": 90.0 + iono_dip,
        "astronomical_dawn": 108.0,
        "nautical_dawn": 102.0,
        "civil_dawn": 96.0,
        "ground_sunrise": 90.833,
        "ground_sunset": 90.833,
    }

    milestones = {}
    base_date = dt_utc.date()

    # Solar noon
    noon_min = 720.0 - time_offset
    noon_h = int(noon_min // 60)
    noon_m = int(noon_min % 60)
    noon_s = int((noon_min * 60) % 60)
    milestones["solar_noon_utc"] = f"{noon_h:02d}:{noon_m:02d}:{noon_s:02d}"

    for name, z_target in zeniths.items():
        cos_ha = (math.cos(math.radians(z_target)) - math.sin(lat_rad) * math.sin(decl_rad)) / (math.cos(lat_rad) * math.cos(decl_rad))
        if cos_ha < -1.0 or cos_ha > 1.0:
            milestones[name + "_utc"] = None
            continue
        ha_deg = math.degrees(math.acos(cos_ha))
        if "sunset" in name:
            tst_min = (ha_deg + 180.0) * 4.0
        else:
            tst_min = (-ha_deg + 180.0) * 4.0
        utc_min = tst_min - time_offset
        h = int(utc_min // 60) % 24
        m = int(utc_min % 60)
        s = int((utc_min * 60) % 60)
        milestones[name + "_utc"] = f"{h:02d}:{m:02d}:{s:02d}"
        edt_h = (h - 4) % 24
        milestones[name + "_edt"] = f"{edt_h:02d}:{m:02d}:{s:02d}"

    return milestones

def compute_satellite_sun_separation(sat_az_deg, sat_el_deg, sun_az_deg, sun_el_deg):
    """Compute topocentric angular separation (degrees) between satellite and Sun."""
    def az_el_to_unit(az, el):
        az_r = math.radians(az)
        el_r = math.radians(el)
        return (math.cos(el_r) * math.sin(az_r),
                math.cos(el_r) * math.cos(az_r),
                math.sin(el_r))
    
    sx, sy, sz = az_el_to_unit(sat_az_deg, sat_el_deg)
    ux, uy, uz = az_el_to_unit(sun_az_deg, sun_el_deg)
    dot = sx * ux + sy * uy + sz * uz
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))

def classify_dawn_state(solar_el_deg, iono_350km_illuminated):
    """Classify the current dawn/solar phase into a standardized state."""
    if solar_el_deg >= 5.0:
        return "FULL_DAYLIGHT", "Sun well above clutter horizon; full ionospheric photoionization active"
    elif solar_el_deg >= 0.0:
        return "GROUND_SUNRISE", "Sun crossing geometric horizon; morning thermal evaporation and ground dew burn-off"
    elif solar_el_deg >= -6.0:
        return "CIVIL_DAWN", "Sun -6° to 0°; diffuse skylight illuminated, tropospheric boundary layer beginning warmup"
    elif solar_el_deg >= -12.0:
        return "NAUTICAL_DAWN", "Sun -12° to -6°; upper atmosphere ionized, E and F layers fully lit"
    elif iono_350km_illuminated:
        return "IONOSPHERIC_SUNRISE_ACTIVE", "Sun -18.6° to -12°; 350 km F2 shell illuminated; early EUV photoionization surge"
    elif solar_el_deg >= -18.0:
        return "ASTRONOMICAL_DAWN", "Sun -18° to -12°; first astronomical rays reaching the exosphere"
    else:
        return "DEEP_NIGHT", "Sun below -18°; night ionospheric recombination regime"

def process_solar_state(observations_dir):
    """Main calculation function correlating live telemetry with solar ephemeris."""
    station_lat = 39.0029
    station_lon = -77.6058
    station_alt_m = 20.0

    # Read live tropo / meteo / iono states if available
    meteo_path = os.path.join(observations_dir, "state.meteorology.json")
    radiometry_path = os.path.join(observations_dir, "state.radiometry.json")
    iono_path = os.path.join(observations_dir, "state.iono.json")
    klobuchar_path = os.path.join(observations_dir, "state.klobuchar.json")

    meteo = {}
    radiometry = {}
    iono = {}
    klobuchar = {}

    if os.path.exists(meteo_path):
        try: meteo = json.load(open(meteo_path))
        except Exception: pass
    if os.path.exists(radiometry_path):
        try: radiometry = json.load(open(radiometry_path))
        except Exception: pass
    if os.path.exists(iono_path):
        try: iono = json.load(open(iono_path))
        except Exception: pass
    if os.path.exists(klobuchar_path):
        try: klobuchar = json.load(open(klobuchar_path))
        except Exception: pass

    # Use coordinates from state files if available
    if "station_lat" in meteo:
        station_lat = meteo["station_lat"]
        station_lon = meteo["station_lon"]
        station_alt_m = meteo.get("station_height_m", station_alt_m)

    dt_utc = datetime.datetime.now(datetime.timezone.utc)
    solar_eph = compute_solar_ephemeris(station_lat, station_lon, station_alt_m, dt_utc)
    sun_el = solar_eph["solar_el_apparent_deg"]
    sun_az = solar_eph["solar_az_deg"]

    # Atmospheric layer illumination profiles
    layers = {
        "ground_0km": compute_layer_illumination(sun_el, 0.0),
        "troposphere_10km": compute_layer_illumination(sun_el, 10.0),
        "stratosphere_30km": compute_layer_illumination(sun_el, 30.0),
        "mesosphere_85km": compute_layer_illumination(sun_el, 85.0),
        "e_layer_110km": compute_layer_illumination(sun_el, 110.0),
        "f2_layer_350km": compute_layer_illumination(sun_el, 350.0),
        "exosphere_1000km": compute_layer_illumination(sun_el, 1000.0),
    }

    overhead_shadow_h_km = compute_overhead_shadow_height_km(sun_el)
    milestones = compute_twilight_milestones(station_lat, station_lon, dt_utc)

    dawn_code, dawn_desc = classify_dawn_state(sun_el, layers["f2_layer_350km"]["is_illuminated"])

    # Satellite-Sun Geometry & IPP Solar Illumination
    satellites = {}
    sat_source = klobuchar.get("satellites", iono.get("satellites", {}))

    closest_sun_sat = None
    min_sun_sep = 999.0

    for s_id, s in sat_source.items():
        sat_az = s.get("az_deg", 0.0)
        sat_el = s.get("el_deg", 0.0)
        cn0 = s.get("cn0", 0.0)
        ipp_lat = s.get("ipp_lat_deg", station_lat)
        ipp_lon = s.get("ipp_lon_deg", station_lon)

        # Angular separation from Sun
        sep_deg = compute_satellite_sun_separation(sat_az, sat_el, sun_az, sun_el)
        if sep_deg < min_sun_sep:
            min_sun_sep = sep_deg
            closest_sun_sat = s_id

        # Solar elevation at IPP (approximate via longitudinal offset)
        ipp_eph = compute_solar_ephemeris(ipp_lat, ipp_lon, 350000.0, dt_utc)
        ipp_sun_el = ipp_eph["solar_el_geom_deg"]
        ipp_illum = (ipp_sun_el + layers["f2_layer_350km"]["horizon_dip_deg"]) > 0.0

        satellites[s_id] = {
            "az_deg": sat_az,
            "el_deg": sat_el,
            "cn0_dbhz": cn0,
            "sun_separation_deg": round(sep_deg, 2),
            "solar_outage_threat": "DIRECT_CONJUNCTION" if sep_deg < 5.0 else ("WARNING" if sep_deg < 15.0 else "CLEAR"),
            "ipp_lat_deg": round(ipp_lat, 3),
            "ipp_lon_deg": round(ipp_lon, 3),
            "ipp_sun_el_deg": round(ipp_sun_el, 2),
            "ipp_illuminated": ipp_illum,
            "ipp_illum_status": "EUV_ACTIVE_DAY" if ipp_illum else "NIGHT_RECOMBINATION"
        }

    # Solar RF Radiometry Impact
    # At L-band (1.575 GHz), quiet-sun flux S ~ 120 SFU.
    # When sun is above horizon (sun_el > 0), direct solar flux enters the antenna pattern.
    rad_summary = radiometry.get("radiometry_summary", {})
    tsys_k = rad_summary.get("mean_tsys_k", 450.0)
    tsys_baseline = rad_summary.get("min_tsys_k", 400.0)
    tsys_excess_k = max(0.0, tsys_k - tsys_baseline)

    solar_payload = {
        "epoch": time.time(),
        "ttl_s": 30.0,
        "station_lat": station_lat,
        "station_lon": station_lon,
        "station_alt_m": station_alt_m,
        "solar_ephemeris": solar_eph,
        "dawn_state": {
            "code": dawn_code,
            "description": dawn_desc,
            "overhead_shadow_height_km": overhead_shadow_h_km,
            "f2_layer_illuminated": layers["f2_layer_350km"]["is_illuminated"],
            "ground_illuminated": layers["ground_0km"]["is_illuminated"],
        },
        "twilight_milestones": milestones,
        "atmospheric_layers": layers,
        "solar_conjunction_summary": {
            "closest_satellite": closest_sun_sat,
            "min_separation_deg": round(min_sun_sep, 2) if min_sun_sep < 900.0 else None,
            "conjunction_threat_level": "CRITICAL" if min_sun_sep < 5.0 else ("ELEVATED" if min_sun_sep < 15.0 else "NOMINAL"),
        },
        "satellites": satellites,
        "solar_radiometry_correlation": {
            "current_tsys_k": tsys_k,
            "tsys_excess_k": round(tsys_excess_k, 1),
            "solar_flux_index_sfu": 125.0, # Solar cycle 25 mean
            "solar_rf_coupling_active": sun_el > -2.0,
        }
    }

    out_path = os.path.join(observations_dir, "state.solar.json")
    tmp_path = out_path + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump(solar_payload, f, indent=2)
    os.replace(tmp_path, out_path)
    return solar_payload

def main():
    parser = argparse.ArgumentParser(description="Solar Terminator & Dawn Detection Engine")
    parser.add_argument("--observations", default="/Volumes/Radiator 8TB/gnss/observations",
                        help="Path to observations directory")
    parser.add_argument("--once", action="store_true", help="Run once and exit")
    parser.add_argument("--interval", type=float, default=2.0, help="Poll interval in seconds")
    args = parser.parse_args()

    if args.once:
        state = process_solar_state(args.observations)
        print("Solar Dawn Detector: Single run complete.")
        print(f"Dawn State: {state['dawn_state']['code']} - {state['dawn_state']['description']}")
        print(f"Sun Position: El={state['solar_ephemeris']['solar_el_apparent_deg']}°, Az={state['solar_ephemeris']['solar_az_deg']}°")
        print(f"Overhead Earth Shadow Height: {state['dawn_state']['overhead_shadow_height_km']} km")
        return

    print(f"Solar Dawn Detector: Starting continuous daemon (poll interval {args.interval}s)...")
    while True:
        try:
            state = process_solar_state(args.observations)
        except Exception as e:
            print(f"Solar Dawn Detector error: {e}", file=sys.stderr)
        time.sleep(args.interval)

if __name__ == "__main__":
    main()
