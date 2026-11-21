//! Solveur PPK (Post-Processed Kinematic) Forward / Backward / Combined (portage `postpos.c` RTKLIB).
//!
//! Prend en entrée :
//! - Observations Rover RINEX 3 OBS (`.obs`)
//! - Observations Base RINEX 3 OBS (`.obs`, ex. station permanente RGP/IGN ou base locale)
//! - Éphémérides RINEX 3 NAV (`.nav`/`.rnx`)
//!
//! En mode `PpkCombined`, exécute :
//! 1. Une passe **Forward** (t0 -> tN)
//! 2. Une passe **Backward** (tN -> t0)
//! 3. Une combinaison lissée pondérée par les variances ($P_c^{-1} = P_f^{-1} + P_b^{-1}$),
//!    augmentant drastiquement le taux de `PPK FIX` après les masques (arbres/bâtiments).

use crate::rinex::{read_rinex3_nav, read_rinex3_obs};
use crate::rtk_engine::RtkEngine;
use crate::types::{FixQuality, GnssConfig, GnssMode, GnssSolution, NavData, ObsEpoch};
use anyhow::Result;
use sdnc_geo::{ecef_to_geodetic_h, WGS84};
use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PpkJobConfig {
    pub rover_obs_path: String,
    pub base_obs_path: String,
    pub nav_path: String,
    pub base_ecef_override: Option<[f64; 3]>,
    pub combined_forward_backward: bool,
    pub gnss_config: GnssConfig,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PpkReport {
    pub total_epochs: usize,
    pub fix_epochs: usize,
    pub float_epochs: usize,
    pub single_epochs: usize,
    pub fix_rate_pct: f32,
    pub mean_std_h_cm: f32,
    pub mean_std_v_cm: f32,
    pub solutions: Vec<GnssSolution>,
}

/// Exécute un post-traitement PPK complet depuis fichiers RINEX 3 sur disque.
pub fn run_ppk_files(job: &PpkJobConfig) -> Result<PpkReport> {
    let rover = read_rinex3_obs(Path::new(&job.rover_obs_path))?;
    let base = read_rinex3_obs(Path::new(&job.base_obs_path))?;
    let nav = read_rinex3_nav(Path::new(&job.nav_path))?;
    let base_ecef = job.base_ecef_override.or(base.approx_position_ecef);
    Ok(run_ppk_in_memory(
        &rover.epochs,
        &base.epochs,
        base_ecef,
        &nav,
        &job.gnss_config,
        job.combined_forward_backward,
    ))
}

/// Exécute le solveur PPK directement en mémoire (utilisable aussi pour les tests ou rejeu terrain).
pub fn run_ppk_in_memory(
    rover_epochs: &[ObsEpoch],
    base_epochs: &[ObsEpoch],
    base_ecef: Option<[f64; 3]>,
    nav: &NavData,
    cfg: &GnssConfig,
    combined: bool,
) -> PpkReport {
    if rover_epochs.is_empty() {
        return PpkReport::default();
    }

    // Helper : trouve l'époque base la plus proche dans un rayon de 15s
    let find_nearest_base = |t_unix: f64| -> Option<&ObsEpoch> {
        let mut best: Option<(&ObsEpoch, f64)> = None;
        for b in base_epochs {
            let dt = (b.time.to_unix_f64() - t_unix).abs();
            if dt <= 15.0 && best.map_or(true, |(_, bd)| dt < bd) {
                best = Some((b, dt));
            }
        }
        best.map(|(b, _)| b)
    };

    // 1. Passe Forward
    let mut fwd_cfg = cfg.clone();
    if matches!(fwd_cfg.mode, GnssMode::Single | GnssMode::PpkForward | GnssMode::PpkCombined) {
        fwd_cfg.mode = GnssMode::RtkKinematic;
    }
    let mut eng_fwd = RtkEngine::new(fwd_cfg.clone());
    let mut fwd_sols = Vec::with_capacity(rover_epochs.len());
    for rov in rover_epochs {
        let base_opt = find_nearest_base(rov.time.to_unix_f64());
        let mut s = eng_fwd.process_rtk_epoch(rov, base_opt, base_ecef, nav);
        s.quality = upgrade_to_ppk_quality(s.quality);
        fwd_sols.push(s);
    }

    // 2. Passe Backward + Combinaison lissée (équivalent `combres()` de RTKLIB)
    let final_sols = if combined && rover_epochs.len() > 1 {
        let mut eng_bwd = RtkEngine::new(fwd_cfg);
        let mut bwd_sols = vec![GnssSolution::default(); rover_epochs.len()];
        for idx in (0..rover_epochs.len()).rev() {
            let rov = &rover_epochs[idx];
            let base_opt = find_nearest_base(rov.time.to_unix_f64());
            let mut s = eng_bwd.process_rtk_epoch(rov, base_opt, base_ecef, nav);
            s.quality = upgrade_to_ppk_quality(s.quality);
            bwd_sols[idx] = s;
        }
        fwd_sols
            .into_iter()
            .zip(bwd_sols)
            .map(|(sf, sb)| combine_forward_backward(sf, sb))
            .collect()
    } else {
        fwd_sols
    };

    build_report(final_sols)
}

fn upgrade_to_ppk_quality(q: FixQuality) -> FixQuality {
    match q {
        FixQuality::RtkFix => FixQuality::PpkFix,
        FixQuality::RtkFloat => FixQuality::PpkFloat,
        other => other,
    }
}

fn quality_rank(q: FixQuality) -> u8 {
    match q {
        FixQuality::PpkFix | FixQuality::RtkFix => 4,
        FixQuality::PpkFloat | FixQuality::RtkFloat => 3,
        FixQuality::Dgnss => 2,
        FixQuality::Single => 1,
        FixQuality::NoFix => 0,
    }
}

/// Combinaison optimale Forward + Backward (RTKLIB `combres`).
fn combine_forward_backward(sf: GnssSolution, sb: GnssSolution) -> GnssSolution {
    let rf = quality_rank(sf.quality);
    let rb = quality_rank(sb.quality);
    if rf > rb {
        return sf;
    }
    if rb > rf {
        return sb;
    }
    if rf == 0 {
        return sf;
    }

    // Fusion pondérée inverse-variance composante par composante
    let mut ecef = [0.0; 3];
    let mut std_enu = [0.0f32; 3];
    for i in 0..3 {
        let vf = (sf.std_enu_m[i].max(0.002) as f64).powi(2);
        let vb = (sb.std_enu_m[i].max(0.002) as f64).powi(2);
        let wf = 1.0 / vf;
        let wb = 1.0 / vb;
        ecef[i] = (wf * sf.ecef[i] + wb * sb.ecef[i]) / (wf + wb);
        std_enu[i] = (1.0 / (wf + wb)).sqrt() as f32;
    }

    let (lat, lon, h) = ecef_to_geodetic_h(ecef[0], ecef[1], ecef[2], &WGS84);
    GnssSolution {
        ecef,
        lat_deg: lat.to_degrees(),
        lon_deg: lon.to_degrees(),
        alt_ellipsoid_m: h,
        alt_orthometric_m: h - 48.0,
        std_enu_m: std_enu,
        lambda_ratio: sf.lambda_ratio.max(sb.lambda_ratio),
        ..sf
    }
}

fn build_report(solutions: Vec<GnssSolution>) -> PpkReport {
    let total = solutions.len();
    let mut fix_epochs = 0usize;
    let mut float_epochs = 0usize;
    let mut single_epochs = 0usize;
    let mut sum_h = 0.0f64;
    let mut sum_v = 0.0f64;

    for s in &solutions {
        match s.quality {
            FixQuality::PpkFix | FixQuality::RtkFix => fix_epochs += 1,
            FixQuality::PpkFloat | FixQuality::RtkFloat => float_epochs += 1,
            FixQuality::Single | FixQuality::Dgnss => single_epochs += 1,
            FixQuality::NoFix => {}
        }
        let sh = ((s.std_enu_m[0].powi(2) + s.std_enu_m[1].powi(2)).sqrt()) as f64;
        sum_h += sh * 100.0;
        sum_v += (s.std_enu_m[2] as f64) * 100.0;
    }

    let denom = total.max(1) as f64;
    PpkReport {
        total_epochs: total,
        fix_epochs,
        float_epochs,
        single_epochs,
        fix_rate_pct: (fix_epochs as f32 / total.max(1) as f32) * 100.0,
        mean_std_h_cm: (sum_h / denom) as f32,
        mean_std_v_cm: (sum_v / denom) as f32,
        solutions,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ephemeris::{geodist, satpos_kepler};
    use crate::types::{BroadcastEphemeris, Constellation, GTime, ObsRecord, SatId, CLIGHT, LAMBDA1};
    use sdnc_geo::geodetic_to_ecef;

    #[test]
    fn test_ppk_combined_forward_backward_fix() {
        let t0 = GTime::from_gps_week_tow(2300, 150_000.0);
        let mut nav = NavData::default();
        for (prn, omg0, m0) in [
            (1, 0.2, 0.8),
            (3, 1.1, 1.4),
            (7, 2.0, 2.1),
            (11, 2.9, 0.5),
            (15, 3.8, 1.9),
            (19, 4.7, 2.8),
            (24, 5.5, 1.0),
        ] {
            nav.add_ephemeris(BroadcastEphemeris {
                sat: SatId::new(Constellation::Gps, prn),
                iode: 1,
                week: 2300,
                toe_s: 150_000.0,
                toc: t0,
                toe: t0,
                sqrt_a: 5153.6,
                e: 0.004,
                i0: 0.96,
                omg0,
                omg: 0.4,
                m0,
                deln: 4.5e-9,
                idot: 0.0,
                omgd: -8.0e-9,
                cuc: 0.0,
                cus: 0.0,
                crc: 0.0,
                crs: 0.0,
                cic: 0.0,
                cis: 0.0,
                f0: 2e-5,
                f1: 0.0,
                f2: 0.0,
                tgd: 0.0,
                sv_health: 0,
            });
        }

        let base = geodetic_to_ecef(43.500f64.to_radians(), 5.500f64.to_radians(), 250.0, &WGS84);
        let base_arr = [base.0, base.1, base.2];

        let mut rov_epochs = Vec::new();
        let mut bas_epochs = Vec::new();
        let mut true_positions = Vec::new();

        for step in 0..12 {
            let t = t0.add(step as f64);
            let rov_true = [
                base_arr[0] + 100.0 + step as f64 * 2.5,
                base_arr[1] + 75.0 + step as f64 * 1.5,
                base_arr[2] + 8.0,
            ];
            true_positions.push(rov_true);

            let mut rov_ep = ObsEpoch { time: t, records: Vec::new() };
            let mut bas_ep = ObsEpoch { time: t, records: Vec::new() };

            for (&sat, list) in &nav.ephemerides {
                let eph = &list[0];
                let mut t_tx = t.add(-0.075);
                for _ in 0..3 {
                    let sv_tmp = satpos_kepler(t_tx, eph);
                    let (r_tmp, _) = geodist(sv_tmp.pos_ecef, rov_true).unwrap();
                    t_tx = t.add(-(r_tmp - CLIGHT * sv_tmp.clock_bias_s + 10.0) / CLIGHT);
                }
                let sv = satpos_kepler(t_tx, eph);
                let (r_r, _) = geodist(sv.pos_ecef, rov_true).unwrap();
                let (r_b, _) = geodist(sv.pos_ecef, base_arr).unwrap();
                let n_int = (sat.prn as f64) * 19.0 - 30.0;

                let mut rr = ObsRecord::new(sat);
                rr.pseudorange[0] = r_r - CLIGHT * sv.clock_bias_s + 10.0;
                rr.carrier_phase[0] = (r_r - CLIGHT * sv.clock_bias_s + 10.0) / LAMBDA1 + n_int;
                rr.snr[0] = 46.0;
                rov_ep.records.push(rr);

                let mut bb = ObsRecord::new(sat);
                bb.pseudorange[0] = r_b - CLIGHT * sv.clock_bias_s - 5.0;
                bb.carrier_phase[0] = (r_b - CLIGHT * sv.clock_bias_s - 5.0) / LAMBDA1;
                bb.snr[0] = 48.0;
                bas_ep.records.push(bb);
            }
            rov_epochs.push(rov_ep);
            bas_epochs.push(bas_ep);
        }

        let mut cfg = GnssConfig::default();
        cfg.elevation_mask_deg = -90.0;
        cfg.lambda_ratio_thresh = 2.0;

        let report = run_ppk_in_memory(&rov_epochs, &bas_epochs, Some(base_arr), &nav, &cfg, true);
        assert_eq!(report.total_epochs, 12);
        assert!(report.fix_rate_pct >= 90.0, "Taux PPK FIX trop faible: {}%", report.fix_rate_pct);
        let mid_err = ((report.solutions[6].ecef[0] - true_positions[6][0]).powi(2)
            + (report.solutions[6].ecef[1] - true_positions[6][1]).powi(2)
            + (report.solutions[6].ecef[2] - true_positions[6][2]).powi(2))
        .sqrt();
        assert!(mid_err < 0.02, "Erreur PPK Combined trop grande : {:.4} m", mid_err);
    }
}
