//! The canonical site anchor: `observations/site.json`.
//!
//! Consumers must NOT carry their own hardcoded site coordinates — by
//! 2026-08-25 five hardcoded copies had drifted ~250 m apart (the Iridium
//! one was never re-sited). There is deliberately NO silent fallback
//! coordinate anywhere: a missing or invalid anchor is an error the caller
//! must surface, never a guessed location. (Mobile stations override per-run
//! via SITE_LL env or the caller's own CLI flags.)

/// Load `[lat_deg, lon_deg, h_m]` from a site.json file. `h_m` defaults to
/// 20 m (antenna-mast height) when absent — a schema default, not a site.
/// Returns None on any missing/invalid coordinate.
pub fn load_site(path: &std::path::Path) -> Option<[f64; 3]> {
    let t = std::fs::read_to_string(path).ok()?;
    let v: serde_json::Value = serde_json::from_str(&t).ok()?;
    let lat = v["lat"].as_f64()?;
    let lon = v["lon"].as_f64()?;
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return None;
    }
    Some([lat, lon, v["h_m"].as_f64().unwrap_or(20.0)])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp_site(body: &str) -> std::path::PathBuf {
        let p = std::env::temp_dir()
            .join(format!("hackrf_gnss_site_test_{}_{}.json", std::process::id(), body.len()));
        std::fs::write(&p, body).unwrap();
        p
    }

    #[test]
    fn loads_a_valid_site() {
        let p = tmp_site(r#"{"lat": 39.0032, "lon": -77.6058, "h_m": 20.0}"#);
        let s = load_site(&p).expect("valid site");
        assert!((s[0] - 39.0032).abs() < 1e-9);
        assert!((s[1] + 77.6058).abs() < 1e-9);
        assert!((s[2] - 20.0).abs() < 1e-9);
        std::fs::remove_file(&p).ok();
    }

    #[test]
    fn h_m_defaults_to_mast_height() {
        let p = tmp_site(r#"{"lat": 39.0, "lon": -77.6}"#);
        assert_eq!(load_site(&p).unwrap()[2], 20.0);
        std::fs::remove_file(&p).ok();
    }

    /// The whole point of the module: never invent coordinates.
    #[test]
    fn missing_or_invalid_is_none_not_a_guess() {
        let missing = std::path::PathBuf::from("/nonexistent/site.json");
        assert_eq!(load_site(&missing), None);
        let p = tmp_site(r#"{"lat": "north", "lon": -77.6}"#);
        assert_eq!(load_site(&p), None);
        std::fs::remove_file(&p).ok();
        let p = tmp_site(r#"{"lat": 123.0, "lon": -77.6}"#);
        assert_eq!(load_site(&p), None); // off the planet
        std::fs::remove_file(&p).ok();
        let p = tmp_site(r#"{"lat": 39.0}"#);
        assert_eq!(load_site(&p), None);
        std::fs::remove_file(&p).ok();
    }
}
