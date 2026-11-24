//! Moteur de positionnement SPP & RTK Double-Différences (portage RTKLIB `pntpos.c` & `rtkpos.c`).
//!
//! Chaîne complète :
//! 1. **SPP (Single Point Positioning)** : moindres carrés pondérés itératifs sur pseudoranges L1 + Saastamoinen + Klobuchar
//! 2. **RTK Double-Différences (EKF)** :
//!    - Élimination des biais d'horloge satellite et récepteur par double-différences (Rover - Base, Sat i - Sat ref)
//!    - Filtre de Kalman étendu `[rx, ry, rz, B_1 .. B_m]` avec détection de sauts de cycle (`LLI`)
//!    - Solution **RTK Float** -> résolution des ambiguïtés entières par **LAMBDA** -> bascule **RTK Fix** si `ratio >= thresh`

use crate::ephemeris::{geodist, iono_klobuchar, satazel, satpos_kepler, trop_saastamoinen};
use crate::lambda::lambda_resolve;
use crate::types::{
    FixQuality, GnssConfig, GnssMode, GnssSolution, GTime, NavData, ObsEpoch, ObsRecord, SatId,
    SatSkyInfo, CLIGHT, LAMBDA1, LAMBDA2,
};
use nalgebra::{DMatrix, DVector};
use sdnc_geo::{ecef_to_geodetic_h, WGS84};
use std::collections::HashMap;

/// État interne du filtre de Kalman RTK L1 / L1+L2 (équivalent `rtk_t` RTKLIB).
pub struct RtkEngine {
    pub config: GnssConfig,
    /// Position ECEF estimée (m)
    pub pos_ecef: [f64; 3],
    /// Covariance 3x3 de la position ECEF
    pub pos_cov: [[f64; 3]; 3],
    /// Ambiguïtés double-différence par `(satellite, bande [0=L1, 1=L2])` : `(valeur cycles, variance)`
    pub sd_biases: HashMap<(SatId, u8), (f64, f64)>,
    pub last_ref_sat: Option<SatId>,
    pub last_time: Option<GTime>,
    pub initialized: bool,
    pub fix_count: u32,
    pub last_used_l2: bool,
}

impl RtkEngine {
    pub fn new(config: GnssConfig) -> Self {
        Self {
            config,
            pos_ecef: [0.0; 3],
            pos_cov: [[100.0, 0.0, 0.0], [0.0, 100.0, 0.0], [0.0, 0.0, 100.0]],
            sd_biases: HashMap::new(),
            last_ref_sat: None,
            last_time: None,
            initialized: false,
            fix_count: 0,
            last_used_l2: false,
        }
    }

    pub fn reset(&mut self) {
        self.sd_biases.clear();
        self.last_ref_sat = None;
        self.last_time = None;
        self.initialized = false;
        self.fix_count = 0;
    }

    /// Positionnement autonome SPP (équivalent `pntpos` RTKLIB).
    pub fn solve_spp(&self, rover: &ObsEpoch, nav: &NavData) -> Option<GnssSolution> {
        let mut x = if self.initialized {
            DVector::from_vec(vec![self.pos_ecef[0], self.pos_ecef[1], self.pos_ecef[2], 0.0])
        } else {
            DVector::from_vec(vec![4_200_000.0, 400_000.0, 4_700_000.0, 0.0])
        };

        let mut sat_infos = Vec::new();
        let mut q_pos = DMatrix::<f64>::identity(4, 4) * 100.0;
        let mut used_count = 0u8;

        for iter in 0..10 {
            let rr = [x[0], x[1], x[2]];
            let (lat, lon, h) = ecef_to_geodetic_h(rr[0], rr[1], rr[2], &WGS84);

            let mut h_rows = Vec::new();
            let mut v_vec = Vec::new();
            let mut weights = Vec::new();
            sat_infos.clear();

            for rec in &rover.records {
                let p1 = rec.pseudorange[0];
                if p1 < 1e6 {
                    continue;
                }
                let Some(eph) = nav.select_eph(rec.sat, rover.time) else {
                    continue;
                };
                // Temps d'émission approximatif
                let t_tx = rover.time.add(-p1 / CLIGHT);
                let sv = satpos_kepler(t_tx, eph);
                let Some((r, los)) = geodist(sv.pos_ecef, rr) else {
                    continue;
                };
                let (az, el) = satazel(lat, lon, los);
                let el_deg = el.to_degrees();
                let usable = iter < 2 || el_deg >= self.config.elevation_mask_deg;

                sat_infos.push(SatSkyInfo {
                    sat: rec.sat,
                    azimuth_deg: az.to_degrees() as f32,
                    elevation_deg: el_deg as f32,
                    snr_dbhz: rec.snr[0],
                    used_in_fix: usable,
                });

                if !usable {
                    continue;
                }

                let d_trop = if iter >= 1 {
                    trop_saastamoinen(lat, h, el, 0.6)
                } else {
                    0.0
                };
                let d_iono = if iter >= 1 {
                    iono_klobuchar(rover.time, lat, lon, az, el, &nav.iono_gps)
                } else {
                    0.0
                };

                let predicted = r + x[3] - CLIGHT * sv.clock_bias_s + d_trop + d_iono;
                let resid = p1 - predicted;
                let sin_el = el.sin().max(0.1);
                let var = (self.config.code_sigma_m / sin_el).powi(2) + 0.5;

                h_rows.push([-los[0], -los[1], -los[2], 1.0]);
                v_vec.push(resid);
                weights.push(1.0 / var);
            }

            if h_rows.len() < 4 {
                return None;
            }
            used_count = h_rows.len().min(255) as u8;

            let m = h_rows.len();
            let mut h_mat = DMatrix::<f64>::zeros(m, 4);
            let mut v = DVector::<f64>::zeros(m);
            let mut w_h = DMatrix::<f64>::zeros(4, m);
            for i in 0..m {
                v[i] = v_vec[i];
                for j in 0..4 {
                    h_mat[(i, j)] = h_rows[i][j];
                    w_h[(j, i)] = h_rows[i][j] * weights[i];
                }
            }

            let normal = &w_h * &h_mat;
            let rhs = &w_h * &v;
            let Some(inv_n) = normal.try_inverse() else {
                return None;
            };
            let dx = &inv_n * &rhs;
            x += &dx;
            q_pos = inv_n;

            if dx.norm() < 1e-4 {
                break;
            }
        }

        let rr = [x[0], x[1], x[2]];
        let (lat, lon, h) = ecef_to_geodetic_h(rr[0], rr[1], rr[2], &WGS84);
        let std_e = q_pos[(0, 0)].abs().sqrt() as f32;
        let std_n = q_pos[(1, 1)].abs().sqrt() as f32;
        let std_u = q_pos[(2, 2)].abs().sqrt() as f32;
        let pdop = (q_pos[(0, 0)] + q_pos[(1, 1)] + q_pos[(2, 2)]).abs().sqrt() as f32;
        let hdop = (q_pos[(0, 0)] + q_pos[(1, 1)]).abs().sqrt() as f32;

        Some(GnssSolution {
            time: rover.time,
            quality: FixQuality::Single,
            ecef: rr,
            lat_deg: lat.to_degrees(),
            lon_deg: lon.to_degrees(),
            alt_ellipsoid_m: h,
            alt_orthometric_m: h - 48.0, // RAF20 moyen métropole si grille non chargée
            std_enu_m: [std_e, std_n, std_u],
            hdop,
            pdop,
            num_sats: used_count,
            age_diff_s: 0.0,
            lambda_ratio: 0.0,
            baseline_m: 0.0,
            satellites: sat_infos,
        })
    }

    /// Traite une époque Rover + Base en mode RTK Double-Différences (équivalent `rtkpos` RTKLIB).
    pub fn process_rtk_epoch(
        &mut self,
        rover: &ObsEpoch,
        base: Option<&ObsEpoch>,
        base_ecef: Option<[f64; 3]>,
        nav: &NavData,
    ) -> GnssSolution {
        // 1. Solution SPP initiale (ou fallback si mode Single / pas de base)
        let spp = match self.solve_spp(rover, nav) {
            Some(s) => s,
            None => return GnssSolution::default(),
        };

        if !self.initialized {
            self.pos_ecef = spp.ecef;
            self.pos_cov = [[900.0, 0.0, 0.0], [0.0, 900.0, 0.0], [0.0, 0.0, 900.0]];
            self.initialized = true;
        }

        if self.config.mode == GnssMode::Single {
            self.pos_ecef = spp.ecef;
            self.last_time = Some(rover.time);
            return spp;
        }

        let (Some(base_obs), Some(rb)) = (base, base_ecef) else {
            self.last_time = Some(rover.time);
            return spp;
        };

        let age_diff_s = rover.time.diff(base_obs.time).abs() as f32;
        if age_diff_s > 30.0 {
            let mut sol = spp;
            sol.age_diff_s = age_diff_s;
            return sol;
        }

        // 2. Prédiction temporelle (EKF Time Update)
        let dt = self
            .last_time
            .map(|t0| rover.time.diff(t0).abs().clamp(0.05, 60.0))
            .unwrap_or(1.0);
        self.last_time = Some(rover.time);

        let is_static = matches!(self.config.mode, GnssMode::RtkStatic);
        if !is_static {
            // Cinématique : réinitialisation lâche de la position avec le SPP tout en conservant les biais de phase
            let q_kin = (30.0 * dt).powi(2).min(900.0);
            for k in 0..3 {
                self.pos_cov[k][k] += q_kin;
            }
        } else {
            // Statique : faible bruit de processus
            for k in 0..3 {
                self.pos_cov[k][k] += 1e-6 * dt;
            }
        }

        // 3. Appariement satellites communs Rover <-> Base + géométrie
        let base_map: HashMap<SatId, &ObsRecord> =
            base_obs.records.iter().map(|r| (r.sat, r)).collect();

        struct CommonSat<'a> {
            sat: SatId,
            rov: &'a ObsRecord,
            bas: &'a ObsRecord,
            los: [f64; 3],
            r_rov: f64,
            r_bas: f64,
            el: f64,
        }

        let rr = self.pos_ecef;
        let (lat, lon, _) = ecef_to_geodetic_h(rr[0], rr[1], rr[2], &WGS84);
        let mut common = Vec::new();
        let mut sky_infos = Vec::new();

        for rov in &rover.records {
            if rov.pseudorange[0] < 1e6 || rov.carrier_phase[0].abs() < 1.0 {
                continue;
            }
            let Some(&bas) = base_map.get(&rov.sat) else {
                continue;
            };
            if bas.pseudorange[0] < 1e6 || bas.carrier_phase[0].abs() < 1.0 {
                continue;
            }
            let Some(eph) = nav.select_eph(rov.sat, rover.time) else {
                continue;
            };
            let t_tx = rover.time.add(-rov.pseudorange[0] / CLIGHT);
            let sv = satpos_kepler(t_tx, eph);
            let (Some((r_rov, los_r)), Some((r_bas, _))) =
                (geodist(sv.pos_ecef, rr), geodist(sv.pos_ecef, rb))
            else {
                continue;
            };
            let (az, el) = satazel(lat, lon, los_r);
            let el_deg = el.to_degrees();
            let usable = el_deg >= self.config.elevation_mask_deg;

            sky_infos.push(SatSkyInfo {
                sat: rov.sat,
                azimuth_deg: az.to_degrees() as f32,
                elevation_deg: el_deg as f32,
                snr_dbhz: rov.snr[0],
                used_in_fix: usable,
            });

            if usable {
                common.push(CommonSat {
                    sat: rov.sat,
                    rov,
                    bas,
                    los: los_r,
                    r_rov,
                    r_bas,
                    el,
                });
            }
        }

        if common.len() < 4 {
            let mut sol = spp;
            sol.age_diff_s = age_diff_s;
            return sol;
        }

        // Sélection du satellite de référence d'élévation maximale
        common.sort_by(|a, b| b.el.partial_cmp(&a.el).unwrap_or(std::cmp::Ordering::Equal));
        let ref_sat = &common[0];
        if self.last_ref_sat != Some(ref_sat.sat) {
            self.sd_biases.clear();
            self.last_ref_sat = Some(ref_sat.sat);
        }
        let other_sats = &common[1..];
        let use_l2_band = self.config.freq_mode.use_l2()
            && ref_sat.rov.pseudorange[1] > 1e6
            && ref_sat.bas.pseudorange[1] > 1e6
            && ref_sat.rov.carrier_phase[1].abs() >= 1.0
            && ref_sat.bas.carrier_phase[1].abs() >= 1.0;

        // Liste des paires double-différences actives : (index dans other_sats, band [0=L1, 1=L2], lambda)
        let mut dd_pairs: Vec<(usize, u8, f64)> = Vec::with_capacity(other_sats.len() * 2);
        for k in 0..other_sats.len() {
            dd_pairs.push((k, 0, LAMBDA1));
        }
        let mut l2_count = 0usize;
        if use_l2_band {
            for (k, s) in other_sats.iter().enumerate() {
                if s.rov.pseudorange[1] > 1e6
                    && s.bas.pseudorange[1] > 1e6
                    && s.rov.carrier_phase[1].abs() >= 1.0
                    && s.bas.carrier_phase[1].abs() >= 1.0
                {
                    dd_pairs.push((k, 1, LAMBDA2));
                    l2_count += 1;
                }
            }
        }
        self.last_used_l2 = l2_count >= 3;
        let nb = dd_pairs.len(); // Nombre total d'ambiguïtés double-différences (L1 + éventuel L2)

        // 4. Construction du vecteur d'état EKF : x = [rx, ry, rz, dd_N_0, ..., dd_N_{nb-1}]
        let nx = 3 + nb;
        let mut x = DVector::<f64>::zeros(nx);
        let mut p = DMatrix::<f64>::zeros(nx, nx);
        for i in 0..3 {
            x[i] = self.pos_ecef[i];
            for j in 0..3 {
                p[(i, j)] = self.pos_cov[i][j];
            }
        }

        // Initialisation / maintien des ambiguïtés double-différences (détection cycle slip LLI par bande)
        let ref_sd_p = [
            ref_sat.rov.pseudorange[0] - ref_sat.bas.pseudorange[0],
            ref_sat.rov.pseudorange[1] - ref_sat.bas.pseudorange[1],
        ];
        let ref_sd_l = [
            ref_sat.rov.carrier_phase[0] - ref_sat.bas.carrier_phase[0],
            ref_sat.rov.carrier_phase[1] - ref_sat.bas.carrier_phase[1],
        ];
        let ref_slip = [
            (ref_sat.rov.lli[0] & 1) != 0 || (ref_sat.bas.lli[0] & 1) != 0,
            (ref_sat.rov.lli[1] & 1) != 0 || (ref_sat.bas.lli[1] & 1) != 0,
        ];

        for (idx, &(k, band, lam)) in dd_pairs.iter().enumerate() {
            let s = &other_sats[k];
            let b = band as usize;
            let slip = ref_slip[b] || (s.rov.lli[b] & 1) != 0 || (s.bas.lli[b] & 1) != 0;
            let sd_p = s.rov.pseudorange[b] - s.bas.pseudorange[b];
            let sd_l = s.rov.carrier_phase[b] - s.bas.carrier_phase[b];
            let dd_n_init = (sd_l - ref_sd_l[b]) - (sd_p - ref_sd_p[b]) / lam;

            let (val, var) = if !slip {
                if let Some(&(prev_v, prev_var)) = self.sd_biases.get(&(s.sat, band)) {
                    (prev_v, prev_var + 1e-8 * dt)
                } else {
                    (dd_n_init, 900.0)
                }
            } else {
                (dd_n_init, 900.0)
            };
            x[3 + idx] = val;
            p[(3 + idx, 3 + idx)] = var;
        }

        // 5. Formation des mesures Double-Différences : [nb phases, nb codes] (L1 et L2)
        let nv = 2 * nb;
        let mut h_mat = DMatrix::<f64>::zeros(nv, nx);
        let mut v_vec = DVector::<f64>::zeros(nv);
        let mut r_mat = DMatrix::<f64>::zeros(nv, nv);

        let ref_geo_sd = ref_sat.r_rov - ref_sat.r_bas;
        let sig_p_ref = self.config.code_sigma_m / ref_sat.el.sin().max(0.15);
        let sig_l_ref = self.config.phase_sigma_m / ref_sat.el.sin().max(0.15);

        for (idx, &(k, band, lam)) in dd_pairs.iter().enumerate() {
            let s = &other_sats[k];
            let b = band as usize;
            let sat_geo_sd = s.r_rov - s.r_bas;
            let dd_geo = sat_geo_sd - ref_geo_sd;

            let dd_l_m = ((s.rov.carrier_phase[b] - s.bas.carrier_phase[b]) - ref_sd_l[b]) * lam;
            let dd_p_m = (s.rov.pseudorange[b] - s.bas.pseudorange[b]) - ref_sd_p[b];

            // Dérivée partielle par rapport à rr : -(los_sat - los_ref)
            let dlos = [
                -(s.los[0] - ref_sat.los[0]),
                -(s.los[1] - ref_sat.los[1]),
                -(s.los[2] - ref_sat.los[2]),
            ];

            // Ligne phase porteuse (indice idx)
            for j in 0..3 {
                h_mat[(idx, j)] = dlos[j];
            }
            h_mat[(idx, 3 + idx)] = lam;
            v_vec[idx] = dd_l_m - (dd_geo + lam * x[3 + idx]);

            // Ligne code pseudorange (indice nb + idx)
            for j in 0..3 {
                h_mat[(nb + idx, j)] = dlos[j];
            }
            v_vec[nb + idx] = dd_p_m - dd_geo;

            // Bruit de mesure corrélé par le satellite de référence au sein de la même bande
            let sig_l_k = self.config.phase_sigma_m / s.el.sin().max(0.15);
            let sig_p_k = self.config.code_sigma_m / s.el.sin().max(0.15);
            for (j_idx, &(_, j_band, _)) in dd_pairs.iter().enumerate() {
                if j_band == band {
                    r_mat[(idx, j_idx)] = 2.0 * sig_l_ref * sig_l_ref;
                    r_mat[(nb + idx, nb + j_idx)] = 2.0 * sig_p_ref * sig_p_ref;
                }
            }
            r_mat[(idx, idx)] += 2.0 * sig_l_k * sig_l_k;
            r_mat[(nb + idx, nb + idx)] += 2.0 * sig_p_k * sig_p_k;
        }

        // 6. Mise à jour Kalman (Measurement Update) : K = P H^T (H P H^T + R)^-1
        let ht = h_mat.transpose();
        let s_inn = &h_mat * &p * &ht + &r_mat;
        let Some(s_inv) = s_inn.try_inverse() else {
            return spp;
        };
        let k_gain = &p * &ht * &s_inv;
        let x_float = &x + &k_gain * &v_vec;
        let eye = DMatrix::<f64>::identity(nx, nx);
        let ikh = &eye - &k_gain * &h_mat;
        let p_float = &ikh * &p * ikh.transpose() + &k_gain * &r_mat * k_gain.transpose();

        // Sauvegarde de l'état Float pour l'époque suivante
        for i in 0..3 {
            self.pos_ecef[i] = x_float[i];
            for j in 0..3 {
                self.pos_cov[i][j] = p_float[(i, j)];
            }
        }
        self.sd_biases.clear();
        for (idx, &(k, band, _)) in dd_pairs.iter().enumerate() {
            let s = &other_sats[k];
            self.sd_biases.insert(
                (s.sat, band),
                (x_float[3 + idx], p_float[(3 + idx, 3 + idx)].max(1e-6)),
            );
        }

        // 7. Tentative de résolution entière LAMBDA (Float -> Fix)
        let a_float = x_float.rows(3, nb).into_owned();
        let q_aa = p_float.view((3, 3), (nb, nb)).into_owned();
        let q_xa = p_float.view((0, 3), (3, nb)).into_owned();
        let q_xx = p_float.view((0, 0), (3, 3)).into_owned();

        let mut final_pos = [x_float[0], x_float[1], x_float[2]];
        let mut final_cov = q_xx;
        let mut quality = FixQuality::RtkFloat;
        let mut ratio = 0.0f32;

        if nb >= 3 {
            if let Ok(lam) = lambda_resolve(&a_float, &q_aa) {
                ratio = lam.ratio as f32;
                if lam.ratio >= self.config.lambda_ratio_thresh {
                    if let Some(q_aa_inv) = q_aa.try_inverse() {
                        let da = &a_float - &lam.best_int;
                        let dx_fix = &q_xa * &q_aa_inv * &da;
                        final_pos = [
                            x_float[0] - dx_fix[0],
                            x_float[1] - dx_fix[1],
                            x_float[2] - dx_fix[2],
                        ];
                        final_cov = &final_cov - &q_xa * &q_aa_inv * q_xa.transpose();
                        quality = FixQuality::RtkFix;
                        self.fix_count += 1;
                    }
                }
            }
        }

        let (lat_f, lon_f, h_f) = ecef_to_geodetic_h(final_pos[0], final_pos[1], final_pos[2], &WGS84);
        let baseline_m = ((final_pos[0] - rb[0]).powi(2)
            + (final_pos[1] - rb[1]).powi(2)
            + (final_pos[2] - rb[2]).powi(2))
        .sqrt();

        let std_e = final_cov[(0, 0)].abs().sqrt().max(0.004) as f32;
        let std_n = final_cov[(1, 1)].abs().sqrt().max(0.004) as f32;
        let std_u = final_cov[(2, 2)].abs().sqrt().max(0.008) as f32;

        GnssSolution {
            time: rover.time,
            quality,
            ecef: final_pos,
            lat_deg: lat_f.to_degrees(),
            lon_deg: lon_f.to_degrees(),
            alt_ellipsoid_m: h_f,
            alt_orthometric_m: h_f - 48.0,
            std_enu_m: [std_e, std_n, std_u],
            hdop: spp.hdop,
            pdop: spp.pdop,
            num_sats: common.len().min(255) as u8,
            age_diff_s,
            lambda_ratio: ratio,
            baseline_m,
            satellites: sky_infos,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{BroadcastEphemeris, Constellation};
    use sdnc_geo::geodetic_to_ecef;

    fn make_synthetic_nav(t0: GTime) -> NavData {
        let mut nav = NavData::default();
        // 7 satellites répartis sur le ciel au-dessus de la France
        let specs = [
            (1, 0.2, 0.8),
            (3, 1.1, 1.4),
            (7, 2.0, 2.1),
            (11, 2.9, 0.5),
            (15, 3.8, 1.9),
            (19, 4.7, 2.8),
            (24, 5.5, 1.0),
        ];
        for (prn, omg0, m0) in specs {
            nav.add_ephemeris(BroadcastEphemeris {
                sat: SatId::new(Constellation::Gps, prn),
                iode: 1,
                week: 2300,
                toe_s: 100_000.0,
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
        nav
    }

    #[test]
    fn test_rtk_double_difference_reaches_centimetric_fix() {
        let t0 = GTime::from_gps_week_tow(2300, 100_000.0);
        let nav = make_synthetic_nav(t0);

        // Base à Aix/Meyreuil, Rover à +120m Est, +85m Nord, +12m Haut
        let base_ecef = geodetic_to_ecef(43.500f64.to_radians(), 5.500f64.to_radians(), 250.0, &WGS84);
        let base_arr = [base_ecef.0, base_ecef.1, base_ecef.2];
        let rover_true = [base_arr[0] + 120.0, base_arr[1] + 85.0, base_arr[2] + 12.0];

        let mut cfg = GnssConfig::default();
        cfg.mode = GnssMode::RtkStatic;
        cfg.elevation_mask_deg = -90.0; // Accepter tous les satellites synthétiques du test
        cfg.lambda_ratio_thresh = 2.0;

        let mut engine = RtkEngine::new(cfg);

        // Ambiguïtés entières constantes par satellite avec vrai temps de vol signal t_tx = t - r/c
        let mut last_sol = GnssSolution::default();
        for step in 0..10 {
            let t = t0.add(step as f64);
            let mut rov_ep = ObsEpoch { time: t, records: Vec::new() };
            let mut bas_ep = ObsEpoch { time: t, records: Vec::new() };

            for (&sat, list) in &nav.ephemerides {
                let eph = &list[0];
                // Calcul itératif du temps d'émission t_tx comme sur un vrai signal GNSS
                let mut t_tx = t.add(-0.075);
                for _ in 0..3 {
                    let sv_tmp = satpos_kepler(t_tx, eph);
                    let (r_tmp, _) = geodist(sv_tmp.pos_ecef, rover_true).unwrap();
                    t_tx = t.add(-(r_tmp - CLIGHT * sv_tmp.clock_bias_s + 15.0) / CLIGHT);
                }
                let sv = satpos_kepler(t_tx, eph);
                let (r_rov, _) = geodist(sv.pos_ecef, rover_true).unwrap();
                let (r_bas, _) = geodist(sv.pos_ecef, base_arr).unwrap();
                let n_int = (sat.prn as f64) * 17.0 - 42.0;

                let mut r_rec = ObsRecord::new(sat);
                r_rec.pseudorange[0] = r_rov - CLIGHT * sv.clock_bias_s + 15.0;
                r_rec.carrier_phase[0] = (r_rov - CLIGHT * sv.clock_bias_s + 15.0) / LAMBDA1 + n_int;
                r_rec.snr[0] = 45.0;
                rov_ep.records.push(r_rec);

                let mut b_rec = ObsRecord::new(sat);
                b_rec.pseudorange[0] = r_bas - CLIGHT * sv.clock_bias_s - 8.0;
                b_rec.carrier_phase[0] = (r_bas - CLIGHT * sv.clock_bias_s - 8.0) / LAMBDA1;
                b_rec.snr[0] = 47.0;
                bas_ep.records.push(b_rec);
            }

            last_sol = engine.process_rtk_epoch(&rov_ep, Some(&bas_ep), Some(base_arr), &nav);
        }

        let err_m = ((last_sol.ecef[0] - rover_true[0]).powi(2)
            + (last_sol.ecef[1] - rover_true[1]).powi(2)
            + (last_sol.ecef[2] - rover_true[2]).powi(2))
        .sqrt();
        assert_eq!(last_sol.quality, FixQuality::RtkFix, "Devrait atteindre RTK FIX");
        assert!(err_m < 0.02, "Erreur RTK Fix trop grande : {:.4} m", err_m);
    }
}
