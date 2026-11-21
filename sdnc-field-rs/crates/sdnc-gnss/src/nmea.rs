//! Parseur et générateur NMEA-0183 (`GGA`, `GST`, `RMC`) pour récepteurs GNSS & NTRIP VRS.

use crate::types::{FixQuality, GnssSolution, GTime};
use sdnc_geo::{geodetic_to_ecef, WGS84};

pub fn verify_nmea_checksum(sentence: &str) -> bool {
    let s = sentence.trim();
    if !s.starts_with('$') {
        return false;
    }
    let Some(star) = s.rfind('*') else {
        return true; // Tolère les trames sans checksum
    };
    let body = &s[1..star];
    let expected = u8::from_str_radix(s[star + 1..].trim(), 16).ok();
    let calc = body.bytes().fold(0u8, |acc, b| acc ^ b);
    expected == Some(calc)
}

fn parse_nmea_deg(raw: &str, hemi: &str) -> Option<f64> {
    let val: f64 = raw.parse().ok()?;
    let deg = (val / 100.0).floor();
    let min = val - deg * 100.0;
    let mut res = deg + min / 60.0;
    if hemi == "S" || hemi == "W" {
        res = -res;
    }
    Some(res)
}

/// Met à jour une `GnssSolution` à partir d'une trame NMEA `$GxGGA` ou `$GxGST`.
pub fn parse_nmea_line(line: &str, sol: &mut GnssSolution) -> bool {
    let s = line.trim();
    if !s.starts_with('$') || !verify_nmea_checksum(s) {
        return false;
    }
    let body = s.split('*').next().unwrap_or(s);
    let fields: Vec<&str> = body.split(',').collect();
    if fields.len() < 6 {
        return false;
    }
    let msg_type = &fields[0][fields[0].len().saturating_sub(3)..];

    match msg_type {
        "GGA" if fields.len() >= 15 => {
            let Some(lat) = parse_nmea_deg(fields[2], fields[3]) else {
                return false;
            };
            let Some(lon) = parse_nmea_deg(fields[4], fields[5]) else {
                return false;
            };
            let q: u8 = fields[6].parse().unwrap_or(0);
            let nsats: u8 = fields[7].parse().unwrap_or(0);
            let hdop: f32 = fields[8].parse().unwrap_or(99.0);
            let alt_msl: f64 = fields[9].parse().unwrap_or(0.0);
            let geoid_sep: f64 = fields[11].parse().unwrap_or(0.0);
            let age: f32 = fields[13].parse().unwrap_or(0.0);

            sol.quality = match q {
                1 => FixQuality::Single,
                2 => FixQuality::Dgnss,
                4 => FixQuality::RtkFix,
                5 => FixQuality::RtkFloat,
                _ => FixQuality::NoFix,
            };
            sol.lat_deg = lat;
            sol.lon_deg = lon;
            sol.alt_orthometric_m = alt_msl;
            sol.alt_ellipsoid_m = alt_msl + geoid_sep;
            let (ex, ey, ez) = geodetic_to_ecef(
                lat.to_radians(),
                lon.to_radians(),
                sol.alt_ellipsoid_m,
                &WGS84,
            );
            sol.ecef = [ex, ey, ez];
            sol.num_sats = nsats;
            sol.hdop = hdop;
            sol.age_diff_s = age;
            sol.time = GTime::from_unix_f64(chrono::Utc::now().timestamp() as f64);
            true
        }
        "GST" if fields.len() >= 9 => {
            let std_lat: f32 = fields[6].parse().unwrap_or(sol.std_enu_m[1]);
            let std_lon: f32 = fields[7].parse().unwrap_or(sol.std_enu_m[0]);
            let std_alt: f32 = fields[8].parse().unwrap_or(sol.std_enu_m[2]);
            sol.std_enu_m = [std_lon, std_lat, std_alt];
            true
        }
        _ => false,
    }
}

/// Génère une trame `$GPGGA` valide pour les casters NTRIP VRS (ex. Centipède NEAR / Orphéon).
pub fn format_gga_sentence(lat_deg: f64, lon_deg: f64, alt_msl: f64) -> String {
    let now = chrono::Utc::now();
    let time_str = now.format("%H%M%S.00");

    let lat_abs = lat_deg.abs();
    let lat_d = lat_abs.floor() as u32;
    let lat_m = (lat_abs - lat_d as f64) * 60.0;
    let ns = if lat_deg >= 0.0 { 'N' } else { 'S' };

    let lon_abs = lon_deg.abs();
    let lon_d = lon_abs.floor() as u32;
    let lon_m = (lon_abs - lon_d as f64) * 60.0;
    let ew = if lon_deg >= 0.0 { 'E' } else { 'W' };

    let body = format!(
        "GPGGA,{},{:02}{:07.4},{},{:03}{:07.4},{},1,10,1.0,{:.2},M,48.0,M,,",
        time_str, lat_d, lat_m, ns, lon_d, lon_m, ew, alt_msl
    );
    let cks = body.bytes().fold(0u8, |acc, b| acc ^ b);
    format!("${}*{:02X}\r\n", body, cks)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nmea_gga_roundtrip() {
        let sentence = format_gga_sentence(43.5297, 5.4474, 210.5);
        assert!(verify_nmea_checksum(&sentence));
        let mut sol = GnssSolution::default();
        assert!(parse_nmea_line(&sentence, &mut sol));
        assert!((sol.lat_deg - 43.5297).abs() < 1e-4);
        assert!((sol.lon_deg - 5.4474).abs() < 1e-4);
        assert_eq!(sol.quality, FixQuality::Single);
    }
}
