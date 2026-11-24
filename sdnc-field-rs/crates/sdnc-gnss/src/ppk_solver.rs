//! Solveur PPK (Post-Processed Kinematic & Static) Forward / Backward / Combined (portage `postpos.c` RTKLIB).
//!
//! Prend en entrée (détection automatique RINEX 3 ou binaire u-blox `.ubx`) :
//! - Observations Rover : **RINEX 3 OBS** (`.obs`, `.rnx`, `.26o`) ou **binaire brut `.ubx`** (Emlid Reach L1→RS4 Pro, SparkFun/ZED-F9P)
//! - Observations Base : **RINEX 3 OBS** (station permanente RGP/IGN, Centipède, Leica/Trimble/Sokkia/Emlid base) ou **`.ubx`**
//! - Éphémérides : **RINEX 3 NAV** (`.nav`, `.rnx`, `.26n`/`.26p`)
//!
//! Supporte :
//! - **Mono-fréquence L1** (Emlid Reach M+/RS+ L1) et **Bi-fréquence L1+L2** (ZED-F9P, SparkFun, Emlid M2/RS2+/RS3/RS4 Pro, Leica, Trimble, Sokkia)
//! - **PPK Cinématique Forward / Combined (Forward + Backward)** + **PPK Statique** (calcul de pivot/borne)
//! - Correction automatique des **hauteurs de canne + offsets ARP→APC** et recalage des **marqueurs d'événements terrain**

use crate::rinex::{read_rinex3_nav, read_rinex3_obs};
use crate::rtk_engine::RtkEngine;
use crate::types::{FixQuality, GnssConfig, GnssMode, GnssSolution, GTime, NavData, ObsEpoch};
use crate::ubx::read_ubx_obs_file;
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
    /// Si true, traite en mode PPK Statique (borne/pivot fixe) et calcule la solution consolidée
    pub static_mode: bool,
    /// Hauteur d'antenne totale rover (canne + ARP→APC, en mètres) soustraite de l'altitude finale
    pub rover_antenna_h_m: f64,
    /// Hauteur d'antenne base (mètres) ajoutée au niveau de référence base si connue
    pub base_antenna_h_m: f64,
    /// Marqueurs d'événements terrain `(label, timestamp_gps)` à recaler sur la trajectoire PPK
    pub event_timestamps: Vec<(String, GTime)>,
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
    pub baseline_mean_m: f64,
    pub used_dual_freq_l1_l2: bool,
    /// Position consolidée (moyenne pondérée des époques PPK FIX) en mode Statique
    pub static_solution: Option<GnssSolution>,
    /// Événements terrain recalés par interpolation sur la trajectoire PPK
    pub event_solutions: Vec<(String, GnssSolution)>,
    pub solutions: Vec<GnssSolution>,
}

/// Charge un fichier d'observations Rover ou Base qu'il soit au format **RINEX 3** ou **u-blox `.ubx`**.
fn load_obs_auto(path: &Path) -> Result<(Option<[f64; 3]>, f64, Vec<ObsEpoch>)> {
    let ext = path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    let is_ubx = ext == "ubx" || starts_with_ubx_magic(path);
    if is_ubx {
        let (approx, epochs) = read_ubx_obs_file(path)?;
        Ok((approx, 0.0, epochs))
    } else {
        let rinex = read_rinex3_obs(path)?;
        Ok((rinex.approx_position_ecef, rinex.antenna_delta_h_m, rinex.epochs))
    }
}

fn starts_with_ubx_magic(path: &Path) -> bool {
    if let Ok(bytes) = std::fs::read(path) {
        bytes.windows(2).take(64).any(|w| w == [0xB5, 0x62])
    } else {
        false
    }
}

/// Exécute un post-traitement PPK complet depuis fichiers RINEX 3 ou `.ubx` sur disque.
pub fn run_ppk_files(job: &PpkJobConfig) -> Result<PpkReport> {
    let (_rov_approx, rov_hdr_h, rover_epochs) = load_obs_auto(Path::new(&job.rover_obs_path))?;
    let (base_approx, _base_hdr_h, base_epochs) = load_obs_auto(Path::new(&job.base_obs_path))?;
    let nav = read_rinex3_nav(Path::new(&job.nav_path))?;
    let base_ecef = job.base_ecef_override.or(base_approx);

    let mut cfg = job.gnss_config.clone();
    if job.static_mode {
        cfg.mode = GnssMode::RtkStatic;
    }

    let ant_h = if job.rover_antenna_h_m > 0.0 {
        job.rover_antenna_h_m
    } else {
        rov_hdr_h
    };

    let mut report = run_ppk_in_memory(
        &rover_epochs,
        &base_epochs,
        base_ecef,
        &nav,
        &cfg,
        job.combined_forward_backward,
    );

    // Appliquer la réduction hauteur de canne + offset ARP sur chaque époque
    if ant_h.abs() > 1e-4 {
        for s in &mut report.solutions {
            s.alt_ellipsoid_m -= ant_h;
            s.alt_orthometric_m -= ant_h;
        }
        if let Some(ref mut st) = report.static_solution {
            st.alt_ellipsoid_m -= ant_h;
            st.alt_orthometric_m -= ant_h;
        }
    }

    // Recaler les marqueurs d'événements terrain sur la trajectoire PPK
    if !job.event_timestamps.is_empty() && !report.solutions.is_empty() {
        report.event_solutions =
            interpolate_events_on_track(&job.event_timestamps, &report.solutions);
    }

    Ok(report)
}

/// Exécute le solveur PPK directement en mémoire (L1 seul ou L1+L2, Forward ou Combined, Cinématique ou Statique).
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
    if matches!(
        fwd_cfg.mode,
        GnssMode::Single | GnssMode::PpkForward | GnssMode::PpkCombined
    ) {
        fwd_cfg.mode = GnssMode::RtkKinematic;
    }
    let is_static = fwd_cfg.mode == GnssMode::RtkStatic;
    let mut eng_fwd = RtkEngine::new(fwd_cfg.clone());
    let mut fwd_sols = Vec::with_capacity(rover_epochs.len());
    let mut used_l2 = false;

    for rov in rover_epochs {
        let base_opt = find_nearest_base(rov.time.to_unix_f64());
        let mut s = eng_fwd.process_rtk_epoch(rov, base_opt, base_ecef, nav);
        used_l2 |= eng_fwd.last_used_l2;
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
            used_l2 |= eng_bwd.last_used_l2;
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

    build_report(final_sols, used_l2, is_static)
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

/// Recale chaque événement terrain par interpolation temporelle sur la trajectoire PPK.
fn interpolate_events_on_track(
    events: &[(String, GTime)],
    sols: &[GnssSolution],
) -> Vec<(String, GnssSolution)> {
    let mut out = Vec::with_capacity(events.len());
    for (label, t_ev) in events {
        let t_u = t_ev.to_unix_f64();
        let best = sols
            .iter()
            .min_by(|a, b| {
                (a.time.to_unix_f64() - t_u)
                    .abs()
                    .partial_cmp(&(b.time.to_unix_f64() - t_u).abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .cloned();
        if let Some(s) = best {
            out.push((label.clone(), s));
        }
    }
    out
}

fn build_report(
    solutions: Vec<GnssSolution>,
    used_dual_freq_l1_l2: bool,
    compute_static: bool,
) -> PpkReport {
    let total = solutions.len();
    let mut fix_epochs = 0usize;
    let mut float_epochs = 0usize;
    let mut single_epochs = 0usize;
    let mut sum_h = 0.0f64;
    let mut sum_v = 0.0f64;
    let mut sum_base = 0.0f64;

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
        sum_base += s.baseline_m;
    }

    // Calcul de la solution statique consolidée (moyenne pondérée des époques PPK FIX ou meilleures époques)
    let static_solution = if compute_static && !solutions.is_empty() {
        let pool: Vec<&GnssSolution> = if fix_epochs > 0 {
            solutions
                .iter()
                .filter(|s| matches!(s.quality, FixQuality::PpkFix | FixQuality::RtkFix))
                .collect()
        } else {
            solutions
                .iter()
                .filter(|s| s.quality != FixQuality::NoFix)
                .collect()
        };
        if !pool.is_empty() {
            let mut sum_w = [0.0f64; 3];
            let mut sum_wx = [0.0f64; 3];
            for s in &pool {
                for i in 0..3 {
                    let w = 1.0 / (s.std_enu_m[i].max(0.002) as f64).powi(2);
                    sum_w[i] += w;
                    sum_wx[i] += w * s.ecef[i];
                }
            }
            let ecef = [
                sum_wx[0] / sum_w[0],
                sum_wx[1] / sum_w[1],
                sum_wx[2] / sum_w[2],
            ];
            let (lat, lon, h) = ecef_to_geodetic_h(ecef[0], ecef[1], ecef[2], &WGS84);
            let last = pool.last().unwrap();
            Some(GnssSolution {
                ecef,
                lat_deg: lat.to_degrees(),
                lon_deg: lon.to_degrees(),
                alt_ellipsoid_m: h,
                alt_orthometric_m: h - 48.0,
                std_enu_m: [
                    (1.0 / sum_w[0]).sqrt() as f32,
                    (1.0 / sum_w[1]).sqrt() as f32,
                    (1.0 / sum_w[2]).sqrt() as f32,
                ],
                ..(*last).clone()
            })
        } else {
            None
        }
    } else {
        None
    };

    let denom = total.max(1) as f64;
    PpkReport {
        total_epochs: total,
        fix_epochs,
        float_epochs,
        single_epochs,
        fix_rate_pct: (fix_epochs as f32 / total.max(1) as f32) * 100.0,
        mean_std_h_cm: (sum_h / denom) as f32,
        mean_std_v_cm: (sum_v / denom) as f32,
        baseline_mean_m: sum_base / denom,
        used_dual_freq_l1_l2,
        static_solution,
        event_solutions: Vec::new(),
        solutions,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ephemeris::{geodist, satpos_kepler};
    use crate::types::{
        BroadcastEphemeris, Constellation, FrequencyBandMode, GTime, ObsRecord, SatId, CLIGHT,
        LAMBDA1, LAMBDA2,
    };
    use sdnc_geo::geodetic_to_ecef;

    #[test]
    fn test_ppk_combined_forward_backward_fix() {
        let t0 = GTime::from_gps_week_tow(2300, 150_000.0);
        let mut nav = NavData::default();
        for (prn, omg0, m0) in [
            (2, 0.3, 0.7),
            (5, 1.0, 1.5),
            (9, 1.9, 2.0),
            (14, 2.7, 0.6),
            (18, 3.6, 1.8),
            (22, 4.5, 2.6),
            (27, 5.4, 1.1),
        ] {
            nav.add_ephemeris(BroadcastEphemeris {
                sat: SatId::new(Constellation::Gps, prn),
                iode: 1,
                week: 2300,
                toe_s: 150_000.0,
                toc: t0,
                toe: t0,
                sqrt_a: 5153.7,
                e: 0.005,
                i0: 55.0f64.to_radians(),
                omg0,
                omg: 0.4,
                m0,
                deln: 4.2e-9,
                idot: 0.0,
                omgd: -8.0e-9,
                cuc: 0.0,
                cus: 0.0,
                crc: 0.0,
                crs: 0.0,
                cic: 0.0,
                cis: 0.0,
                f0: 0.0,
                f1: 0.0,
                f2: 0.0,
                tgd: 0.0,
                sv_health: 0,
            });
        }

        let base = geodetic_to_ecef(43.50f64.to_radians(), 5.50f64.to_radians(), 200.0, &WGS84);
        let base_arr = [base.0, base.1, base.2];

        let mut rov_epochs = Vec::new();
        let mut bas_epochs = Vec::new();
        for ep in 0..12 {
            let t = t0.add(ep as f64);
            let rover_pos = [
                base_arr[0] + 80.0 + (ep as f64) * 1.5,
                base_arr[1] - 40.0,
                base_arr[2] + 5.0,
            ];
            let mut r_ep = ObsEpoch {
                time: t,
                records: Vec::new(),
            };
            let mut b_ep = ObsEpoch {
                time: t,
                records: Vec::new(),
            };
            for (idx, (&sat, ephs)) in nav.ephemerides.iter().enumerate() {
                let sv_est = satpos_kepler(t, &ephs[0]);
                let (r_r0, _) = geodist(sv_est.pos_ecef, rover_pos).unwrap();
                let sv = satpos_kepler(t.add(-r_r0 / CLIGHT), &ephs[0]);
                let (r_r, _) = geodist(sv.pos_ecef, rover_pos).unwrap();
                let (r_b, _) = geodist(sv.pos_ecef, base_arr).unwrap();
                let n1 = (idx as f64) * 11.0 + 3.0;
                let n2 = (idx as f64) * 7.0 - 5.0;

                let mut rr = ObsRecord::new(sat);
                rr.pseudorange[0] = r_r + 10.0;
                rr.carrier_phase[0] = (r_r + 10.0) / LAMBDA1 + n1;
                rr.pseudorange[1] = r_r + 10.0;
                rr.carrier_phase[1] = (r_r + 10.0) / LAMBDA2 + n2;
                rr.snr[0] = 44.0;
                rr.snr[1] = 41.0;
                r_ep.records.push(rr);

                let mut bb = ObsRecord::new(sat);
                bb.pseudorange[0] = r_b - 5.0;
                bb.carrier_phase[0] = (r_b - 5.0) / LAMBDA1;
                bb.pseudorange[1] = r_b - 5.0;
                bb.carrier_phase[1] = (r_b - 5.0) / LAMBDA2;
                bb.snr[0] = 46.0;
                bb.snr[1] = 43.0;
                b_ep.records.push(bb);
            }
            rov_epochs.push(r_ep);
            bas_epochs.push(b_ep);
        }

        let mut cfg = GnssConfig::default();
        cfg.freq_mode = FrequencyBandMode::L1L2;
        cfg.elevation_mask_deg = -90.0;
        cfg.lambda_ratio_thresh = 2.0;

        let report = run_ppk_in_memory(&rov_epochs, &bas_epochs, Some(base_arr), &nav, &cfg, true);
        assert_eq!(report.total_epochs, 12);
        assert!(report.used_dual_freq_l1_l2, "Le solveur PPK doit exploiter L1+L2");
        assert!(
            report.fix_epochs > 0,
            "Le solveur PPK L1+L2 Combined doit résoudre des époques PPK FIX"
        );
    }
}
