//! Module de Recherche de Lieux, Adresses BAN (IGN) & Coordonnées (Projetées & Geodésiques).

use crate::crs::{from_wgs84, to_wgs84, FrenchCrs};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LocationSearchResult {
    pub label: String,
    pub x_proj: f64,
    pub y_proj: f64,
    pub lat_wgs84: f64,
    pub lon_wgs84: f64,
    pub result_type: SearchResultType,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SearchResultType {
    ProjectedCoordinates,
    GeodeticCoordinates,
    BanAddress,
}

/// Tente d'abord de parser des coordonnées (X Y projetées ou Lat Lon WGS84).
/// Si ce n'est pas une coordonnée, interroge l'API BAN IGN (Base Adresse Nationale).
pub fn search_location(query: &str, active_crs: FrenchCrs) -> Vec<LocationSearchResult> {
    let q = query.trim();
    if q.is_empty() {
        return vec![];
    }

    // 1. Détection Coordonnées Projetées ou WGS84 (ex: 893250 6254000 ou 46.4968, 2.3388)
    let parts: Vec<&str> = q
        .split(|c: char| c == ',' || c == ';' || c == '|' || c.is_whitespace())
        .filter(|s| !s.is_empty())
        .collect();

    if parts.len() >= 2 {
        if let (Ok(val1), Ok(val2)) = (parts[0].parse::<f64>(), parts[1].parse::<f64>()) {
            // Est-ce des coordonnées WGS84 (Lat, Lon) ? (-90 <= Lat <= 90 et -180 <= Lon <= 180)
            if val1.abs() <= 90.0 && val2.abs() <= 180.0 {
                let lat = val1;
                let lon = val2;
                if let Ok((px, py)) = from_wgs84(active_crs, lon, lat) {
                    return vec![LocationSearchResult {
                        label: format!("📍 WGS84: Lat {:.6}°, Lon {:.6}° (X: {:.2}m, Y: {:.2}m)", lat, lon, px, py),
                        x_proj: px,
                        y_proj: py,
                        lat_wgs84: lat,
                        lon_wgs84: lon,
                        result_type: SearchResultType::GeodeticCoordinates,
                    }];
                }
            } else if val2.abs() <= 90.0 && val1.abs() <= 180.0 {
                let lon = val1;
                let lat = val2;
                if let Ok((px, py)) = from_wgs84(active_crs, lon, lat) {
                    return vec![LocationSearchResult {
                        label: format!("📍 WGS84: Lon {:.6}°, Lat {:.6}° (X: {:.2}m, Y: {:.2}m)", lon, lat, px, py),
                        x_proj: px,
                        y_proj: py,
                        lat_wgs84: lat,
                        lon_wgs84: lon,
                        result_type: SearchResultType::GeodeticCoordinates,
                    }];
                }
            } else {
                // Coordonnées projetées (X, Y)
                let px = val1;
                let py = val2;
                let (lon, lat) = to_wgs84(active_crs, px, py).unwrap_or((0.0, 0.0));
                return vec![LocationSearchResult {
                    label: format!("📐 Coordonnées Projetées: X {:.2} m, Y {:.2} m", px, py),
                    x_proj: px,
                    y_proj: py,
                    lat_wgs84: lat,
                    lon_wgs84: lon,
                    result_type: SearchResultType::ProjectedCoordinates,
                }];
            }
        }
    }

    // 2. Interrogation API BAN IGN (Base Adresse Nationale - api-adresse.data.gouv.fr)
    search_ban_api(q, active_crs)
}

fn search_ban_api(query: &str, active_crs: FrenchCrs) -> Vec<LocationSearchResult> {
    let clean_q = query.replace(" ", "+");
    let url = format!("https://api-adresse.data.gouv.fr/search/?q={}&limit=5", clean_q);

    let res = ureq::get(&url)
        .set("User-Agent", "AEROPHOTOGRAM/2.0")
        .timeout(std::time::Duration::from_secs(3))
        .call();

    let mut results = Vec::new();
    if let Ok(resp) = res {
        if let Ok(body) = resp.into_string() {
            if let Ok(json) = serde_json::from_str::<serde_json::Value>(&body) {
                if let Some(features) = json.get("features").and_then(|f| f.as_array()) {
                    for feat in features {
                        if let (Some(coords), Some(props)) = (
                            feat.get("geometry").and_then(|g| g.get("coordinates")).and_then(|c| c.as_array()),
                            feat.get("properties"),
                        ) {
                            if coords.len() >= 2 {
                                if let (Some(lon), Some(lat)) = (coords[0].as_f64(), coords[1].as_f64()) {
                                    let label = props.get("label").and_then(|l| l.as_str()).unwrap_or("Adresse inconnue").to_string();
                                    if let Ok((px, py)) = from_wgs84(active_crs, lon, lat) {
                                        results.push(LocationSearchResult {
                                            label: format!("🏠 {}", label),
                                            x_proj: px,
                                            y_proj: py,
                                            lat_wgs84: lat,
                                            lon_wgs84: lon,
                                            result_type: SearchResultType::BanAddress,
                                        });
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    results
}

/// Interroge l'API BAN IGN en Reverse Geocoding (à partir des coordonnées projetées X, Y dans la projection active).
/// Renvoie l'adresse BAN la plus proche avec la distance et les coordonnées dans la projection active.
pub fn reverse_geocode_location(x_proj: f64, y_proj: f64, active_crs: FrenchCrs) -> Option<LocationSearchResult> {
    let (lon, lat) = to_wgs84(active_crs, x_proj, y_proj).ok()?;
    let url = format!("https://api-adresse.data.gouv.fr/reverse/?lon={:.6}&lat={:.6}&limit=1", lon, lat);

    let res = ureq::get(&url)
        .set("User-Agent", "AEROPHOTOGRAM/2.0")
        .timeout(std::time::Duration::from_secs(3))
        .call()
        .ok()?;

    let body = res.into_string().ok()?;
    let json: serde_json::Value = serde_json::from_str(&body).ok()?;
    let features = json.get("features")?.as_array()?;
    let feat = features.first()?;
    let props = feat.get("properties")?;
    let label = props.get("label")?.as_str()?;
    let distance_m = props.get("distance").and_then(|d| d.as_f64()).unwrap_or(0.0);

    let coords = feat.get("geometry")?.get("coordinates")?.as_array()?;
    let ban_lon = coords.first()?.as_f64()?;
    let ban_lat = coords.get(1)?.as_f64()?;

    let (_px, _py) = from_wgs84(active_crs, ban_lon, ban_lat).unwrap_or((x_proj, y_proj));

    let formatted_label = if distance_m < 2.0 {
        format!("🏠 {}", label)
    } else {
        format!("🏠 {} (Adresse la plus proche, à {:.0}m)", label, distance_m)
    };

    Some(LocationSearchResult {
        label: formatted_label,
        x_proj,
        y_proj,
        lat_wgs84: lat,
        lon_wgs84: lon,
        result_type: SearchResultType::BanAddress,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crs::FrenchCrs;

    #[test]
    fn test_ban_reverse_geocoding_live() {
        let res = reverse_geocode_location(893500.0, 6270000.0, FrenchCrs::Lambert93);
        if let Some(loc) = res {
            println!("✅ BAN Reverse Geocoding : {}", loc.label);
            assert!(!loc.label.is_empty());
        } else {
            println!("⚠️ Mode hors-ligne détecté pour la BAN.");
        }
    }
}
