//! Calcul d'éphémérides satellites & modèles atmosphériques (portage RTKLIB `ephemeris.c` / `rtkcmn.c`).
//!
//! Inclut :
//! - Position & biais d'horloge satellite Kepler (`satpos_kepler`) avec correction relativiste
//! - Distance géométrique avec effet Sagnac (`geodist`)
//! - Azimut / Élévation ENU (`satazel`)
//! - Modèle troposphérique de Saastamoinen (`trop_saastamoinen`)
//! - Modèle ionosphérique de Klobuchar (`iono_klobuchar`)

use crate::types::{BroadcastEphemeris, GTime, CLIGHT, MU_GPS, OMGE};
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy)]
pub struct SatState {
    pub pos_ecef: [f64; 3],
    pub clock_bias_s: f64,
}

/// Calcule la position ECEF et le biais d'horloge d'un satellite à l'instant `t`
/// depuis son éphéméride radiodiffusée (équivalent `eph2pos` de RTKLIB).
pub fn satpos_kepler(t: GTime, eph: &BroadcastEphemeris) -> SatState {
    let a = eph.sqrt_a * eph.sqrt_a;
    let tk = t.diff(eph.toe);

    let n0 = (MU_GPS / (a * a * a)).sqrt();
    let n = n0 + eph.deln;
    let m = eph.m0 + n * tk;

    // Résolution itérative de l'équation de Kepler : E = M + e * sin(E)
    let mut ek = m;
    for _ in 0..15 {
        let ek_next = m + eph.e * ek.sin();
        if (ek_next - ek).abs() < 1e-13 {
            ek = ek_next;
            break;
        }
        ek = ek_next;
    }

    let sin_e = ek.sin();
    let cos_e = ek.cos();
    let one_m_e2 = (1.0 - eph.e * eph.e).max(1e-12);
    let nu = (one_m_e2.sqrt() * sin_e).atan2(cos_e - eph.e);
    let phi = nu + eph.omg;

    let sin2p = (2.0 * phi).sin();
    let cos2p = (2.0 * phi).cos();

    let u = phi + eph.cus * sin2p + eph.cuc * cos2p;
    let r = a * (1.0 - eph.e * cos_e) + eph.crs * sin2p + eph.crc * cos2p;
    let inc = eph.i0 + eph.idot * tk + eph.cis * sin2p + eph.cic * cos2p;

    let x_orb = r * u.cos();
    let y_orb = r * u.sin();

    let omega = eph.omg0 + (eph.omgd - OMGE) * tk - OMGE * eph.toe_s;
    let sin_o = omega.sin();
    let cos_o = omega.cos();
    let sin_i = inc.sin();
    let cos_i = inc.cos();

    let x = x_orb * cos_o - y_orb * cos_i * sin_o;
    let y = x_orb * sin_o + y_orb * cos_i * cos_o;
    let z = y_orb * sin_i;

    // Correction d'horloge satellite + correction relativiste
    let tc = t.diff(eph.toc);
    let dtr = -2.0 * (MU_GPS * a).sqrt() * eph.e * sin_e / (CLIGHT * CLIGHT);
    let clock_bias_s = eph.f0 + eph.f1 * tc + eph.f2 * tc * tc + dtr - eph.tgd;

    SatState {
        pos_ecef: [x, y, z],
        clock_bias_s,
    }
}

/// Distance géométrique satellite-récepteur avec correction Sagnac + vecteur unitaire LOS.
pub fn geodist(rs: [f64; 3], rr: [f64; 3]) -> Option<(f64, [f64; 3])> {
    let dx = rs[0] - rr[0];
    let dy = rs[1] - rr[1];
    let dz = rs[2] - rr[2];
    let r = (dx * dx + dy * dy + dz * dz).sqrt();
    if r < 1e3 {
        return None;
    }
    let los = [dx / r, dy / r, dz / r];
    let sagnac = OMGE * (rs[0] * rr[1] - rs[1] * rr[0]) / CLIGHT;
    Some((r + sagnac, los))
}

/// Calcule `(azimuth_rad, elevation_rad)` d'un satellite depuis la position géodésique `(lat_rad, lon_rad)`.
pub fn satazel(lat_rad: f64, lon_rad: f64, los_ecef: [f64; 3]) -> (f64, f64) {
    let sinp = lat_rad.sin();
    let cosp = lat_rad.cos();
    let sinl = lon_rad.sin();
    let cosl = lon_rad.cos();

    let e = -sinl * los_ecef[0] + cosl * los_ecef[1];
    let n = -sinp * cosl * los_ecef[0] - sinp * sinl * los_ecef[1] + cosp * los_ecef[2];
    let u = cosp * cosl * los_ecef[0] + cosp * sinl * los_ecef[1] + sinp * los_ecef[2];

    let az = e.atan2(n).rem_euclid(2.0 * PI);
    let el = u.clamp(-1.0, 1.0).asin();
    (az, el)
}

/// Modèle troposphérique de Saastamoinen (équivalent `tropmodel` RTKLIB, retour en mètres).
pub fn trop_saastamoinen(lat_rad: f64, h_m: f64, el_rad: f64, humi: f64) -> f64 {
    if h_m < -100.0 || h_m > 1e4 || el_rad <= 0.02 {
        return 0.0;
    }
    let h = h_m.max(0.0);
    let pres = 1013.25 * (1.0 - 2.2557e-5 * h).powf(5.2568);
    let temp = 15.0 - 6.5e-3 * h + 273.16;
    let e = 6.108 * humi * ((17.15 * temp - 4684.0) / (temp - 38.45)).exp();

    let z = PI / 2.0 - el_rad;
    let trph = 0.002_276_8 * pres / (1.0 - 0.00266 * (2.0 * lat_rad).cos() - 0.00028 * h * 1e-3) / z.cos();
    let trpw = 0.002_277 * (1255.0 / temp + 0.05) * e / z.cos();
    trph + trpw
}

/// Modèle ionosphérique de Klobuchar (équivalent `ionmodel` RTKLIB, retour en mètres L1).
pub fn iono_klobuchar(
    t: GTime,
    lat_rad: f64,
    lon_rad: f64,
    az_rad: f64,
    el_rad: f64,
    ion: &[f64; 8],
) -> f64 {
    if ion.iter().all(|&v| v == 0.0) || el_rad <= 0.0 {
        return 0.0;
    }
    let (_, tow) = t.to_gps_week_tow();
    let psi = 0.0137 / (el_rad / PI + 0.11) - 0.022;
    let mut phi = lat_rad / PI + psi * az_rad.cos();
    phi = phi.clamp(-0.416, 0.416);
    let lam = lon_rad / PI + psi * az_rad.sin() / (phi * PI).cos();
    let phi_m = phi + 0.064 * ((lam - 1.617) * PI).cos();

    let mut tt = 43_200.0 * lam + tow;
    tt = tt.rem_euclid(86_400.0);

    let f = 1.0 + 16.0 * (0.53 - el_rad / PI).powi(3);
    let amp = (ion[0] + phi_m * (ion[1] + phi_m * (ion[2] + phi_m * ion[3]))).max(0.0);
    let per = (ion[4] + phi_m * (ion[5] + phi_m * (ion[6] + phi_m * ion[7]))).max(72_000.0);
    let x = 2.0 * PI * (tt - 50_400.0) / per;

    let delay_s = if x.abs() < 1.57 {
        f * (5e-9 + amp * (1.0 - x * x / 2.0 + x.powi(4) / 24.0))
    } else {
        f * 5e-9
    };
    CLIGHT * delay_s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Constellation, SatId};

    #[test]
    fn test_kepler_orbit_altitude_gps() {
        let toe = GTime::from_gps_week_tow(2300, 100_000.0);
        let eph = BroadcastEphemeris {
            sat: SatId::new(Constellation::Gps, 1),
            iode: 1,
            week: 2300,
            toe_s: 100_000.0,
            toc: toe,
            toe,
            sqrt_a: 5153.6, // ~26560 km rayon orbital GPS
            e: 0.005,
            i0: 0.96,
            omg0: 1.2,
            omg: 0.5,
            m0: 0.3,
            deln: 4.5e-9,
            idot: 0.0,
            omgd: -8.0e-9,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            f0: 1e-5,
            f1: 1e-12,
            f2: 0.0,
            tgd: 0.0,
            sv_health: 0,
        };
        let state = satpos_kepler(toe.add(60.0), &eph);
        let r = (state.pos_ecef[0].powi(2) + state.pos_ecef[1].powi(2) + state.pos_ecef[2].powi(2)).sqrt();
        assert!(r > 26_000_000.0 && r < 27_100_000.0, "Rayon orbital GPS inattendu: {}", r);
    }

    #[test]
    fn test_saastamoinen_zenith_delay() {
        let d = trop_saastamoinen(45.0f64.to_radians(), 100.0, 90.0f64.to_radians(), 0.5);
        assert!(d > 2.2 && d < 2.6, "Retard tropo zénithal inattendu: {}", d);
    }
}
