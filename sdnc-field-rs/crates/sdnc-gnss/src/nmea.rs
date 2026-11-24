//! Parseur et générateur NMEA-0183 + trames propriétaires constructeurs :
//! - Standard NMEA-0183 : `$GxGGA`, `$GxGST`, `$GxGSA`, `$GxGSV` (Skyplot temps réel)
//! - **Trimble** : `$PTNL,GGK` (position RTK + qualité Fix/Float + PDOP + hauteur ellipsoïdale EHT)
//! - **Leica** : `$GPLLQ` (qualité RTK + nombre de satellites + CQ 3D)

use crate::types::{Constellation, FixQuality, GnssSolution, GTime, SatId, SatSkyInfo};
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

fn talker_constellation(talker: &str) -> Constellation {
    match talker {
        "GL" => Constellation::Glonass,
        "GA" => Constellation::Galileo,
        "GB" | "BD" => Constellation::BeiDou,
        "GQ" | "QZ" => Constellation::Qzss,
        _ => Constellation::Gps,
    }
}

/// Met à jour une `GnssSolution` à partir d'une trame NMEA standard (`GGA`, `GST`, `GSA`, `GSV`)
/// ou propriétaire constructeur (**Trimble `$PTNL,GGK`**, **Leica `$GPLLQ`**).
pub fn parse_nmea_line(line: &str, sol: &mut GnssSolution) -> bool {
    let s = line.trim();
    if !s.starts_with('$') || !verify_nmea_checksum(s) {
        return false;
    }
    let body = s.split('*').next().unwrap_or(s);
    let fields: Vec<&str> = body.split(',').collect();
    if fields.len() < 5 {
        return false;
    }

    // 1. Trame propriétaire Trimble : $PTNL,GGK,time,date,lat,N,lon,E,Q,sats,DOP,EHTxxx.xxx,M*cs
    if fields[0].eq_ignore_ascii_case("$PTNL") && fields.get(1).copied() == Some("GGK") && fields.len() >= 12 {
        let Some(lat) = parse_nmea_deg(fields[4], fields[5]) else {
            return false;
        };
        let Some(lon) = parse_nmea_deg(fields[6], fields[7]) else {
            return false;
        };
        let q: u8 = fields[8].parse().unwrap_or(0);
        let nsats: u8 = fields[9].parse().unwrap_or(0);
        let dop: f32 = fields[10].parse().unwrap_or(sol.pdop);
        let eht: f64 = fields[11]
            .trim_start_matches("EHT")
            .parse()
            .unwrap_or(sol.alt_ellipsoid_m);

        sol.quality = match q {
            1 => FixQuality::Single,
            2 => FixQuality::RtkFloat,
            3 => FixQuality::RtkFix,
            4 => FixQuality::Dgnss,
            _ => FixQuality::NoFix,
        };
        sol.lat_deg = lat;
        sol.lon_deg = lon;
        sol.alt_ellipsoid_m = eht;
        sol.alt_orthometric_m = eht - 48.0;
        let (ex, ey, ez) = geodetic_to_ecef(lat.to_radians(), lon.to_radians(), eht, &WGS84);
        sol.ecef = [ex, ey, ez];
        sol.num_sats = nsats;
        sol.pdop = dop;
        sol.time = GTime::from_unix_f64(chrono::Utc::now().timestamp() as f64);
        return true;
    }

    // 2. Trame propriétaire Leica : $GPLLQ,time,date,E,M,N,M,Q,sats,CQ,H,M*cs
    if fields[0].len() >= 6 && fields[0].ends_with("LLQ") && fields.len() >= 10 {
        let q: u8 = fields[7].parse().unwrap_or(0);
        let nsats: u8 = fields[8].parse().unwrap_or(sol.num_sats);
        let cq_m: f32 = fields[9].parse().unwrap_or(0.05);
        sol.quality = match q {
            1 => FixQuality::Single,
            2 => FixQuality::Dgnss,
            3 => FixQuality::RtkFix,
            4 => FixQuality::RtkFloat,
            _ => sol.quality,
        };
        sol.num_sats = nsats;
        if cq_m > 0.0 && cq_m < 50.0 {
            sol.std_enu_m = [cq_m * 0.6, cq_m * 0.6, cq_m * 0.8];
        }
        return true;
    }

    let header = fields[0].trim_start_matches('$');
    if header.len() < 5 {
        return false;
    }
    let talker = &header[0..2];
    let msg_type = &header[header.len() - 3..];

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
        "GSA" if fields.len() >= 18 => {
            if let Ok(pdop) = fields[15].parse::<f32>() {
                sol.pdop = pdop;
            }
            if let Ok(hdop) = fields[16].parse::<f32>() {
                sol.hdop = hdop;
            }
            true
        }
        "GSV" if fields.len() >= 8 => {
            let constel = talker_constellation(talker);
            let msg_num: u8 = fields[2].parse().unwrap_or(1);
            if msg_num == 1 {
                sol.satellites.retain(|s| s.sat.constellation != constel);
            }
            let mut idx = 4;
            while idx + 3 < fields.len() {
                if let Ok(prn) = fields[idx].parse::<u8>() {
                    let el: f32 = fields[idx + 1].parse().unwrap_or(0.0);
                    let az: f32 = fields[idx + 2].parse().unwrap_or(0.0);
                    let snr: f32 = fields[idx + 3].parse().unwrap_or(0.0);
                    let sat = SatId::new(constel, prn);
                    if let Some(existing) = sol.satellites.iter_mut().find(|s| s.sat == sat) {
                        existing.elevation_deg = el;
                        existing.azimuth_deg = az;
                        existing.snr_dbhz = snr;
                        existing.used_in_fix = snr >= 20.0;
                    } else {
                        sol.satellites.push(SatSkyInfo {
                            sat,
                            azimuth_deg: az,
                            elevation_deg: el,
                            snr_dbhz: snr,
                            used_in_fix: snr >= 20.0,
                        });
                    }
                }
                idx += 4;
            }
            false
        }
        _ => false,
    }
}

/// Génère une trame `$GPGGA` valide pour les casters NTRIP VRS (ex. Centipède NEAR / Orphéon / Teria).
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

    #[test]
    fn test_trimble_ptnl_ggk_and_leica_gpllq_and_gsv() {
        let mut sol = GnssSolution::default();
        let trimble = "$PTNL,GGK,114215.00,112426,4331.78200,N,00526.84400,E,3,18,1.1,EHT258.420,M";
        assert!(parse_nmea_line(trimble, &mut sol));
        assert_eq!(sol.quality, FixQuality::RtkFix);
        assert_eq!(sol.num_sats, 18);
        assert!((sol.alt_ellipsoid_m - 258.42).abs() < 1e-3);

        let leica = "$GPLLQ,114216.00,241126,894510.12,M,6269520.34,M,3,19,0.012,242.50,M";
        assert!(parse_nmea_line(leica, &mut sol));
        assert_eq!(sol.quality, FixQuality::RtkFix);
        assert!(sol.std_enu_m[0] < 0.01);

        let gsv = "$GAGSV,1,1,02,04,52,140,45,11,31,260,41";
        parse_nmea_line(gsv, &mut sol);
        assert_eq!(sol.satellites.len(), 2);
        assert_eq!(sol.satellites[0].sat, SatId::new(Constellation::Galileo, 4));
    }
}
