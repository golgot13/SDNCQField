//! Contrôleur GNSS temps réel multi-sources (RTK + PPK + Simulateur + TCP/UDP/NTRIP + Android JNI).
//!
//! Pilote l'ensemble de la chaîne RTKLIB Rust pour l'interface utilisateur :
//! - Connexion / déconnexion non-bloquante
//! - Réception NMEA/UBX Rover + Corrections RTCM3 NTRIP Base
//! - Enregistrement brut RINEX 3.04 (`.obs`) + marqueurs événements terrain pour PPK
//! - Moyennage multi-époques de points RTK haute précision
//! - Exécution asynchrone du post-traitement PPK (`ppk_solver`)

use crate::ephemeris::{geodist, satpos_kepler};
use crate::nmea::{format_gga_sentence, parse_nmea_line};
use crate::ntrip::connect_ntrip_stream;
use crate::ppk_solver::{run_ppk_files, run_ppk_in_memory, PpkJobConfig, PpkReport};
use crate::rinex::Rinex3ObsWriter;
use crate::rtcm3::{Rtcm3Message, Rtcm3StreamDecoder};
use crate::rtk_engine::RtkEngine;
use crate::types::{
    BroadcastEphemeris, Constellation, FixQuality, GnssConfig, GnssMode, GnssSolution, GTime,
    NavData, ObsEpoch, ObsRecord, ReceiverSource, SatId, CLIGHT, LAMBDA1,
};
use crate::ubx::{UbxMessage, UbxStreamDecoder};
use sdnc_geo::{geodetic_to_ecef, WGS84};
use std::io::{Read, Write};
use std::net::{TcpStream, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Default)]
pub struct GnssLiveTelemetry {
    pub connected: bool,
    pub status_message: String,
    pub solution: GnssSolution,
    pub ntrip_connected: bool,
    pub rtcm_frames_rx: u64,
    pub rtcm_msm_epochs_rx: u64,
    pub base_station_id: Option<u16>,
    pub base_ecef: Option<[f64; 3]>,
    pub ppk_recording: bool,
    pub ppk_epochs_logged: usize,
    pub raw_bytes_rx: u64,
    pub track_history_wgs84: Vec<(f64, f64, f64, FixQuality)>,
    pub event_markers: Vec<(String, GnssSolution)>,
    pub last_ppk_report: Option<PpkReport>,
    pub ppk_running: bool,
}

struct ExternalBytePipeline {
    ubx: UbxStreamDecoder,
    rtcm: Rtcm3StreamDecoder,
    rtk: RtkEngine,
    nmea_buf: String,
    rinex_writer: Option<Rinex3ObsWriter>,
}

impl Default for ExternalBytePipeline {
    fn default() -> Self {
        Self {
            ubx: UbxStreamDecoder::default(),
            rtcm: Rtcm3StreamDecoder::default(),
            rtk: RtkEngine::new(GnssConfig::default()),
            nmea_buf: String::new(),
            rinex_writer: None,
        }
    }
}

pub struct GnssController {
    pub config: Arc<Mutex<GnssConfig>>,
    pub telemetry: Arc<Mutex<GnssLiveTelemetry>>,
    external_pipe: Arc<Mutex<ExternalBytePipeline>>,
    stop_flag: Arc<AtomicBool>,
    ppk_record_flag: Arc<AtomicBool>,
}

impl Default for GnssController {
    fn default() -> Self {
        Self::new(GnssConfig::default())
    }
}

impl GnssController {
    pub fn new(config: GnssConfig) -> Self {
        let pipe = ExternalBytePipeline {
            rtk: RtkEngine::new(config.clone()),
            ..Default::default()
        };
        Self {
            config: Arc::new(Mutex::new(config)),
            telemetry: Arc::new(Mutex::new(GnssLiveTelemetry {
                status_message: "Déconnecté — prêt à démarrer RTK/PPK".to_string(),
                ..Default::default()
            })),
            external_pipe: Arc::new(Mutex::new(pipe)),
            stop_flag: Arc::new(AtomicBool::new(true)),
            ppk_record_flag: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn is_running(&self) -> bool {
        !self.stop_flag.load(Ordering::SeqCst)
    }

    pub fn snapshot(&self) -> GnssLiveTelemetry {
        self.telemetry
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }

    /// Injecte un flux brut d'octets provenant d'un récepteur externe u-blox / Emlid Reach
    /// (Bluetooth SPP/BLE Android via JNI, USB-OTG Série, ou port série desktop).
    /// Décode simultanément UBX (`NAV-PVT`, `RXM-RAWX`), NMEA (`GGA`, `GST`), exécute
    /// le moteur logiciel `RtkEngine` si les corrections RTCM3 MSM4/7 + éphémérides 1019/1046
    /// sont présentes, et enregistre en RINEX 3.04 si PPK actif.
    pub fn inject_external_raw_bytes(&self, bytes: &[u8], source_label: &str) {
        if bytes.is_empty() {
            return;
        }
        let cfg = self.config.lock().unwrap_or_else(|e| e.into_inner()).clone();
        let ppk_rec = self.ppk_record_flag.load(Ordering::SeqCst);

        let mut pipe = self
            .external_pipe
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        pipe.rtk.config = cfg.clone();
        let mut tel = self.telemetry.lock().unwrap_or_else(|e| e.into_inner());
        tel.connected = true;
        tel.raw_bytes_rx += bytes.len() as u64;

        let mut updated_pos = false;

        // 1. Décodage binaire UBX (u-blox ZED-F9P / M8T)
        for msg in pipe.ubx.push_bytes(bytes) {
            match msg {
                UbxMessage::NavPvt {
                    lat_deg,
                    lon_deg,
                    alt_ellipsoid_m,
                    alt_msl_m,
                    h_acc_m,
                    v_acc_m,
                    num_sv,
                    quality,
                } => {
                    let (ex, ey, ez) = geodetic_to_ecef(
                        lat_deg.to_radians(),
                        lon_deg.to_radians(),
                        alt_ellipsoid_m,
                        &WGS84,
                    );
                    tel.solution.ecef = [ex, ey, ez];
                    tel.solution.lat_deg = lat_deg;
                    tel.solution.lon_deg = lon_deg;
                    tel.solution.alt_ellipsoid_m = alt_ellipsoid_m;
                    tel.solution.alt_orthometric_m = alt_msl_m;
                    tel.solution.std_enu_m = [h_acc_m, h_acc_m, v_acc_m];
                    tel.solution.num_sats = num_sv;
                    tel.solution.quality = quality;
                    tel.solution.time = GTime::from_unix_f64(chrono::Utc::now().timestamp() as f64);
                    updated_pos = true;
                }
                UbxMessage::RxmRawx(epoch) => {
                    // Enregistrement brut RINEX 3.04 pour post-traitement PPK
                    if ppk_rec {
                        if pipe.rinex_writer.is_none() {
                            pipe.rinex_writer = Rinex3ObsWriter::create(
                                &cfg.raw_ppk_output_path,
                                tel.solution.ecef,
                                "EXT_ROVER",
                            )
                            .ok();
                        }
                        if let Some(w) = pipe.rinex_writer.as_mut() {
                            let _ = w.write_epoch(&epoch);
                            let _ = w.flush();
                            tel.ppk_epochs_logged = w.epochs_written;
                        }
                    } else {
                        pipe.rinex_writer = None;
                    }

                    // Fusion logicielle RTKLIB-Rust en direct si RTCM3 MSM + éphémérides 1019/1046 disponibles
                    if !pipe.rtcm.nav_data.ephemerides.is_empty() {
                        let base_ep = pipe.rtcm.latest_base_epoch.clone();
                        let base_xyz = cfg.base_ecef_override.or(pipe.rtcm.last_base_ecef);
                        let nav_snapshot = pipe.rtcm.nav_data.clone();
                        let rtk_sol = pipe.rtk.process_rtk_epoch(
                            &epoch,
                            base_ep.as_ref(),
                            base_xyz,
                            &nav_snapshot,
                        );
                        if rtk_sol.quality != FixQuality::NoFix {
                            tel.solution = rtk_sol;
                            updated_pos = true;
                        }
                    }
                }
            }
        }

        // 2. Décodage ASCII NMEA-0183 (Emlid Reach / u-blox / GNSS générique)
        pipe.nmea_buf.push_str(&String::from_utf8_lossy(bytes));
        if pipe.nmea_buf.len() > 8192 {
            let drain_to = pipe.nmea_buf.len() - 4096;
            pipe.nmea_buf.drain(0..drain_to);
        }
        while let Some(pos) = pipe.nmea_buf.find('\n') {
            let line: String = pipe.nmea_buf.drain(..=pos).collect();
            if parse_nmea_line(&line, &mut tel.solution) {
                updated_pos = true;
            }
        }

        if updated_pos && tel.solution.quality != FixQuality::NoFix {
            let quality = tel.solution.quality;
            let num_sats = tel.solution.num_sats;
            let sh_cm = ((tel.solution.std_enu_m[0].powi(2) + tel.solution.std_enu_m[1].powi(2))
                .sqrt())
                * 100.0;
            let (lon, lat, alt) = (
                tel.solution.lon_deg,
                tel.solution.lat_deg,
                tel.solution.alt_ellipsoid_m,
            );
            tel.status_message = format!(
                "{} — {} | {} sats | σH {:.1} cm",
                source_label,
                quality.label(),
                num_sats,
                sh_cm
            );
            tel.track_history_wgs84.push((lon, lat, alt, quality));
            if tel.track_history_wgs84.len() > 2000 {
                tel.track_history_wgs84.remove(0);
            }
        }
    }

    /// Injecte des octets RTCM 3.3 (NTRIP / radio / Bluetooth Base) dans le décodeur MSM4/MSM7.
    pub fn inject_rtcm_bytes(&self, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }
        let mut pipe = self
            .external_pipe
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let msgs = pipe.rtcm.push_bytes(bytes);
        if !msgs.is_empty() {
            let mut tel = self.telemetry.lock().unwrap_or_else(|e| e.into_inner());
            tel.ntrip_connected = true;
            tel.rtcm_frames_rx = pipe.rtcm.valid_frames;
            tel.rtcm_msm_epochs_rx = pipe.rtcm.msm_frames_decoded;
            if let Some(id) = pipe.rtcm.last_station_id {
                tel.base_station_id = Some(id);
            }
            if let Some(ecef) = pipe.rtcm.last_base_ecef {
                tel.base_ecef = Some(ecef);
            }
        }
    }

    /// Injecte une position provenant du GPS interne Android (via JNI).
    pub fn inject_android_location(
        &self,
        lat_deg: f64,
        lon_deg: f64,
        alt_ellipsoid_m: f64,
        accuracy_m: f32,
        num_sats: u8,
    ) {
        let (ex, ey, ez) = geodetic_to_ecef(
            lat_deg.to_radians(),
            lon_deg.to_radians(),
            alt_ellipsoid_m,
            &WGS84,
        );
        let sol = GnssSolution {
            time: GTime::from_unix_f64(chrono::Utc::now().timestamp() as f64),
            quality: FixQuality::Single,
            ecef: [ex, ey, ez],
            lat_deg,
            lon_deg,
            alt_ellipsoid_m,
            alt_orthometric_m: alt_ellipsoid_m - 48.0,
            std_enu_m: [accuracy_m, accuracy_m, accuracy_m * 1.5],
            hdop: (accuracy_m / 2.5).max(0.7),
            pdop: (accuracy_m / 1.8).max(1.1),
            num_sats,
            ..Default::default()
        };
        let mut tel = self.telemetry.lock().unwrap_or_else(|e| e.into_inner());
        tel.connected = true;
        tel.status_message = "GPS interne Android actif".to_string();
        tel.track_history_wgs84
            .push((lon_deg, lat_deg, alt_ellipsoid_m, sol.quality));
        if tel.track_history_wgs84.len() > 2000 {
            tel.track_history_wgs84.remove(0);
        }
        tel.solution = sol;
    }

    pub fn start(&self) {
        if self.is_running() {
            return;
        }
        self.stop_flag.store(false, Ordering::SeqCst);

        let cfg_arc = Arc::clone(&self.config);
        let tel_arc = Arc::clone(&self.telemetry);
        let stop = Arc::clone(&self.stop_flag);
        let ppk_rec = Arc::clone(&self.ppk_record_flag);

        std::thread::spawn(move || {
            let cfg = cfg_arc.lock().unwrap_or_else(|e| e.into_inner()).clone();
            {
                let mut t = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
                t.connected = true;
                t.status_message = format!("Connexion {:?} ({:?})...", cfg.source, cfg.mode);
            }

            match cfg.source {
                ReceiverSource::SimulatedRtk => {
                    run_simulated_rtk_loop(cfg, tel_arc, stop, ppk_rec);
                }
                ReceiverSource::TcpStream => {
                    run_tcp_stream_loop(cfg, tel_arc, stop);
                }
                ReceiverSource::UdpStream => {
                    run_udp_stream_loop(cfg, tel_arc, stop);
                }
                ReceiverSource::SerialPort => {
                    run_serial_port_loop(cfg, tel_arc, stop, ppk_rec);
                }
                ReceiverSource::BluetoothExternal => {
                    let mut t = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
                    t.status_message = format!(
                        "Bluetooth SPP actif ({}) — en attente flux JNI u-blox/Emlid",
                        cfg.bluetooth_device_name
                    );
                }
                ReceiverSource::AndroidInternal => {
                    let mut t = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
                    t.status_message = "En attente du flux GPS interne Android (JNI)".to_string();
                }
            }
        });
    }

    pub fn stop(&self) {
        self.stop_flag.store(true, Ordering::SeqCst);
        self.ppk_record_flag.store(false, Ordering::SeqCst);
        let mut tel = self.telemetry.lock().unwrap_or_else(|e| e.into_inner());
        tel.connected = false;
        tel.ntrip_connected = false;
        tel.ppk_recording = false;
        tel.status_message = "GNSS arrêté".to_string();
    }

    pub fn set_ppk_recording(&self, enable: bool) {
        self.ppk_record_flag.store(enable, Ordering::SeqCst);
        let mut tel = self.telemetry.lock().unwrap_or_else(|e| e.into_inner());
        tel.ppk_recording = enable;
    }

    pub fn clear_track(&self) {
        let mut tel = self.telemetry.lock().unwrap_or_else(|e| e.into_inner());
        tel.track_history_wgs84.clear();
        tel.event_markers.clear();
    }

    pub fn mark_event(&self, label: String) {
        let mut tel = self.telemetry.lock().unwrap_or_else(|e| e.into_inner());
        let sol = tel.solution.clone();
        tel.event_markers.push((label, sol));
    }

    /// Lance un post-traitement PPK réel sur fichiers RINEX 3 en arrière-plan.
    pub fn run_ppk_files_async(&self, job: PpkJobConfig) {
        let tel_arc = Arc::clone(&self.telemetry);
        {
            let mut t = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
            t.ppk_running = true;
            t.status_message = "Calcul PPK Forward/Backward en cours...".to_string();
        }
        std::thread::spawn(move || {
            let res = run_ppk_files(&job);
            let mut t = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
            t.ppk_running = false;
            match res {
                Ok(report) => {
                    t.status_message = format!(
                        "PPK terminé : {} époques ({:.1}% PPK FIX, RMS H={:.1} cm)",
                        report.total_epochs, report.fix_rate_pct, report.mean_std_h_cm
                    );
                    t.last_ppk_report = Some(report);
                }
                Err(e) => {
                    t.status_message = format!("Erreur PPK : {}", e);
                }
            }
        });
    }

    /// Exécute une démonstration complète du vrai solveur PPK Forward+Backward RTKLIB-Rust
    /// sur une trajectoire terrain synthétique (utile pour valider le moteur sans fichiers externes).
    pub fn run_ppk_demo_async(&self) {
        let tel_arc = Arc::clone(&self.telemetry);
        let cfg = self.config.lock().unwrap_or_else(|e| e.into_inner()).clone();
        {
            let mut t = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
            t.ppk_running = true;
            t.status_message = "Exécution PPK Forward+Backward (trajectoire terrain 40 époques)...".to_string();
        }
        std::thread::spawn(move || {
            let t0 = GTime::from_gps_week_tow(2300, 200_000.0);
            let nav = build_france_nav(t0);
            let base = geodetic_to_ecef(43.500f64.to_radians(), 5.500f64.to_radians(), 240.0, &WGS84);
            let base_arr = [base.0, base.1, base.2];

            let mut rov_epochs = Vec::with_capacity(40);
            let mut bas_epochs = Vec::with_capacity(40);
            for k in 0..40 {
                let t = t0.add(k as f64);
                let angle = (k as f64) * 0.16;
                let rover_pos = [
                    base_arr[0] + 150.0 + 45.0 * angle.cos(),
                    base_arr[1] + 90.0 + 45.0 * angle.sin(),
                    base_arr[2] + 6.0 + 1.5 * (angle * 0.5).sin(),
                ];
                let (r_ep, b_ep) = synth_epoch_pair(t, &nav, rover_pos, base_arr);
                rov_epochs.push(r_ep);
                bas_epochs.push(b_ep);
            }

            let mut ppk_cfg = cfg;
            ppk_cfg.elevation_mask_deg = -90.0;
            ppk_cfg.lambda_ratio_thresh = 2.0;
            let report = run_ppk_in_memory(&rov_epochs, &bas_epochs, Some(base_arr), &nav, &ppk_cfg, true);

            let mut t = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
            t.ppk_running = false;
            t.status_message = format!(
                "PPK Combined OK : {} époques | {:.1}% PPK FIX | σH={:.2} cm",
                report.total_epochs, report.fix_rate_pct, report.mean_std_h_cm
            );
            t.track_history_wgs84.clear();
            for s in &report.solutions {
                t.track_history_wgs84
                    .push((s.lon_deg, s.lat_deg, s.alt_ellipsoid_m, s.quality));
            }
            if let Some(last) = report.solutions.last() {
                t.solution = last.clone();
            }
            t.last_ppk_report = Some(report);
        });
    }
}

fn build_france_nav(t0: GTime) -> NavData {
    let mut nav = NavData::default();
    let specs = [
        (Constellation::Gps, 2, 0.25, 0.7),
        (Constellation::Gps, 5, 1.05, 1.3),
        (Constellation::Gps, 9, 1.85, 2.0),
        (Constellation::Gps, 14, 2.75, 0.4),
        (Constellation::Galileo, 3, 3.60, 1.7),
        (Constellation::Galileo, 11, 4.45, 2.6),
        (Constellation::Galileo, 21, 5.30, 0.9),
        (Constellation::Gps, 28, 5.95, 1.5),
    ];
    for (c, prn, omg0, m0) in specs {
        nav.add_ephemeris(BroadcastEphemeris {
            sat: SatId::new(c, prn),
            iode: 1,
            week: 2300,
            toe_s: 200_000.0,
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
            f0: 1.5e-5,
            f1: 0.0,
            f2: 0.0,
            tgd: 0.0,
            sv_health: 0,
        });
    }
    nav
}

fn synth_epoch_pair(
    t: GTime,
    nav: &NavData,
    rover_ecef: [f64; 3],
    base_ecef: [f64; 3],
) -> (ObsEpoch, ObsEpoch) {
    let mut rov_ep = ObsEpoch {
        time: t,
        records: Vec::new(),
    };
    let mut bas_ep = ObsEpoch {
        time: t,
        records: Vec::new(),
    };
    for (&sat, list) in &nav.ephemerides {
        let eph = &list[0];
        let mut t_tx = t.add(-0.075);
        for _ in 0..3 {
            let sv_tmp = satpos_kepler(t_tx, eph);
            let Some((r_tmp, _)) = geodist(sv_tmp.pos_ecef, rover_ecef) else {
                break;
            };
            t_tx = t.add(-(r_tmp - CLIGHT * sv_tmp.clock_bias_s + 12.0) / CLIGHT);
        }
        let sv = satpos_kepler(t_tx, eph);
        let (Some((r_r, _)), Some((r_b, _))) =
            (geodist(sv.pos_ecef, rover_ecef), geodist(sv.pos_ecef, base_ecef))
        else {
            continue;
        };
        let n_int = (sat.prn as f64) * 23.0 - 51.0;
        let mut rr = ObsRecord::new(sat);
        rr.pseudorange[0] = r_r - CLIGHT * sv.clock_bias_s + 12.0;
        rr.carrier_phase[0] = (r_r - CLIGHT * sv.clock_bias_s + 12.0) / LAMBDA1 + n_int;
        rr.snr[0] = 43.0 + (sat.prn % 6) as f32;
        rov_ep.records.push(rr);

        let mut bb = ObsRecord::new(sat);
        bb.pseudorange[0] = r_b - CLIGHT * sv.clock_bias_s - 6.0;
        bb.carrier_phase[0] = (r_b - CLIGHT * sv.clock_bias_s - 6.0) / LAMBDA1;
        bb.snr[0] = 46.0;
        bas_ep.records.push(bb);
    }
    (rov_ep, bas_ep)
}

/// Boucle RTK temps réel pilotée par le vrai moteur `RtkEngine` + `lambda_resolve` sur observations synthétiques.
fn run_simulated_rtk_loop(
    mut cfg: GnssConfig,
    tel_arc: Arc<Mutex<GnssLiveTelemetry>>,
    stop: Arc<AtomicBool>,
    ppk_rec: Arc<AtomicBool>,
) {
    let t0 = GTime::from_gps_week_tow(2300, 200_000.0);
    let nav = build_france_nav(t0);
    let base = geodetic_to_ecef(43.500f64.to_radians(), 5.500f64.to_radians(), 240.0, &WGS84);
    let base_arr = [base.0, base.1, base.2];

    cfg.elevation_mask_deg = -90.0;
    cfg.lambda_ratio_thresh = 2.0;
    let mut engine = RtkEngine::new(cfg.clone());
    let mut rinex_writer: Option<Rinex3ObsWriter> = None;
    let mut step = 0u64;

    while !stop.load(Ordering::SeqCst) {
        let t = t0.add(step as f64);
        let theta = (step as f64) * 0.08;
        let rover_ecef = [
            base_arr[0] + 180.0 + 35.0 * theta.cos(),
            base_arr[1] + 110.0 + 35.0 * theta.sin(),
            base_arr[2] + 8.0 + 1.2 * (theta * 0.5).sin(),
        ];
        let (rov_ep, bas_ep) = synth_epoch_pair(t, &nav, rover_ecef, base_arr);

        // Enregistrement brut RINEX 3.04 PPK si activé dans l'interface
        if ppk_rec.load(Ordering::SeqCst) {
            if rinex_writer.is_none() {
                rinex_writer =
                    Rinex3ObsWriter::create(&cfg.raw_ppk_output_path, rover_ecef, "SDNC_ROVER").ok();
            }
            if let Some(w) = rinex_writer.as_mut() {
                let _ = w.write_epoch(&rov_ep);
                let _ = w.flush();
            }
        } else {
            rinex_writer = None;
        }

        let use_base = cfg.mode != GnssMode::Single;
        let sol = engine.process_rtk_epoch(
            &rov_ep,
            if use_base { Some(&bas_ep) } else { None },
            if use_base { Some(base_arr) } else { None },
            &nav,
        );

        {
            let mut tel = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
            tel.connected = true;
            tel.ntrip_connected = use_base;
            tel.base_ecef = Some(base_arr);
            tel.base_station_id = Some(1042);
            tel.rtcm_frames_rx += if use_base { 3 } else { 0 };
            tel.ppk_epochs_logged = rinex_writer.as_ref().map(|w| w.epochs_written).unwrap_or(0);
            tel.status_message = format!(
                "{} | {} sats | Ratio {:.1} | σH {:.1} cm",
                sol.quality.label(),
                sol.num_sats,
                sol.lambda_ratio,
                ((sol.std_enu_m[0].powi(2) + sol.std_enu_m[1].powi(2)).sqrt()) * 100.0
            );
            tel.track_history_wgs84.push((
                sol.lon_deg,
                sol.lat_deg,
                sol.alt_ellipsoid_m,
                sol.quality,
            ));
            if tel.track_history_wgs84.len() > 2000 {
                tel.track_history_wgs84.remove(0);
            }
            tel.solution = sol;
        }

        step += 1;
        std::thread::sleep(Duration::from_millis(500));
    }
}

fn run_tcp_stream_loop(
    cfg: GnssConfig,
    tel_arc: Arc<Mutex<GnssLiveTelemetry>>,
    stop: Arc<AtomicBool>,
) {
    let addr = format!("{}:{}", cfg.tcp_host, cfg.tcp_port);
    let mut stream = match TcpStream::connect(&addr) {
        Ok(s) => s,
        Err(e) => {
            let mut t = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
            t.connected = false;
            t.status_message = format!("Erreur TCP {}: {}", addr, e);
            return;
        }
    };
    let _ = stream.set_read_timeout(Some(Duration::from_millis(300)));

    // Connexion NTRIP optionnelle pour corrections RTCM3
    let mut ntrip_stream = if cfg.ntrip.enabled {
        connect_ntrip_stream(&cfg.ntrip).ok()
    } else {
        None
    };
    let mut rtcm_dec = Rtcm3StreamDecoder::default();
    let mut ubx_dec = UbxStreamDecoder::default();
    let mut nmea_buf = String::new();
    let mut cur_sol = GnssSolution::default();
    let mut last_gga_tx = Instant::now();
    let mut tmp = [0u8; 2048];

    while !stop.load(Ordering::SeqCst) {
        if let Ok(n) = stream.read(&mut tmp) {
            if n > 0 {
                let slice = &tmp[..n];
                // Décodage UBX binaire
                for msg in ubx_dec.push_bytes(slice) {
                    if let UbxMessage::NavPvt {
                        lat_deg,
                        lon_deg,
                        alt_ellipsoid_m,
                        alt_msl_m,
                        h_acc_m,
                        v_acc_m,
                        num_sv,
                        quality,
                    } = msg
                    {
                        cur_sol.lat_deg = lat_deg;
                        cur_sol.lon_deg = lon_deg;
                        cur_sol.alt_ellipsoid_m = alt_ellipsoid_m;
                        cur_sol.alt_orthometric_m = alt_msl_m;
                        cur_sol.std_enu_m = [h_acc_m, h_acc_m, v_acc_m];
                        cur_sol.num_sats = num_sv;
                        cur_sol.quality = quality;
                    }
                }
                // Décodage NMEA ASCII
                nmea_buf.push_str(&String::from_utf8_lossy(slice));
                while let Some(pos) = nmea_buf.find('\n') {
                    let line: String = nmea_buf.drain(..=pos).collect();
                    parse_nmea_line(&line, &mut cur_sol);
                }
                let mut tel = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
                tel.solution = cur_sol.clone();
                tel.status_message = format!("TCP {} — {}", addr, cur_sol.quality.label());
                if cur_sol.quality != FixQuality::NoFix {
                    tel.track_history_wgs84.push((
                        cur_sol.lon_deg,
                        cur_sol.lat_deg,
                        cur_sol.alt_ellipsoid_m,
                        cur_sol.quality,
                    ));
                }
            }
        }

        // Lecture RTCM3 NTRIP + renvoi vers récepteur & décodage ARP Base
        if let Some(ns) = ntrip_stream.as_mut() {
            if cfg.ntrip.send_gga
                && last_gga_tx.elapsed() >= Duration::from_secs(5)
                && cur_sol.quality != FixQuality::NoFix
            {
                let gga = format_gga_sentence(
                    cur_sol.lat_deg,
                    cur_sol.lon_deg,
                    cur_sol.alt_orthometric_m,
                );
                let _ = ns.write_all(gga.as_bytes());
                last_gga_tx = Instant::now();
            }
            if let Ok(rn) = ns.read(&mut tmp) {
                if rn > 0 {
                    let _ = stream.write_all(&tmp[..rn]);
                    for rmsg in rtcm_dec.push_bytes(&tmp[..rn]) {
                        if let Rtcm3Message::StationArp {
                            station_id, ecef, ..
                        } = rmsg
                        {
                            let mut tel = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
                            tel.base_station_id = Some(station_id);
                            tel.base_ecef = Some(ecef);
                        }
                    }
                    let mut tel = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
                    tel.ntrip_connected = true;
                    tel.rtcm_frames_rx = rtcm_dec.valid_frames;
                    tel.rtcm_msm_epochs_rx = rtcm_dec.msm_frames_decoded;
                }
            }
        }
    }
}

/// Boucle port série USB/COM (Linux `/dev/ttyACM0`, `/dev/ttyUSB0`, Windows `COM3`)
/// pour récepteurs externes u-blox ZED-F9P / Emlid Reach avec renvoi bidirectionnel RTCM3 NTRIP
/// et enregistrement RINEX 3.04 PPK des trames `UBX-RXM-RAWX`.
fn run_serial_port_loop(
    cfg: GnssConfig,
    tel_arc: Arc<Mutex<GnssLiveTelemetry>>,
    stop: Arc<AtomicBool>,
    ppk_rec: Arc<AtomicBool>,
) {
    // Sur Linux, configure le baudrate via `stty` sans dépendance libudev native
    #[cfg(target_os = "linux")]
    {
        let _ = std::process::Command::new("stty")
            .args([
                "-F",
                &cfg.serial_port_path,
                &cfg.serial_baud_rate.to_string(),
                "raw",
                "-echo",
                "-echoe",
                "-echok",
            ])
            .status();
    }

    let mut serial = match std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(&cfg.serial_port_path)
    {
        Ok(f) => f,
        Err(e) => {
            let mut t = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
            t.connected = false;
            t.status_message = format!(
                "Port série '{}' ({} bauds) inaccessible : {}",
                cfg.serial_port_path, cfg.serial_baud_rate, e
            );
            return;
        }
    };

    let mut ntrip_stream = if cfg.ntrip.enabled {
        connect_ntrip_stream(&cfg.ntrip).ok()
    } else {
        None
    };

    let mut ubx_dec = UbxStreamDecoder::default();
    let mut rtcm_dec = Rtcm3StreamDecoder::default();
    let mut nmea_buf = String::new();
    let mut cur_sol = GnssSolution::default();
    let mut rinex_writer: Option<Rinex3ObsWriter> = None;
    let mut last_gga_tx = Instant::now();
    let mut tmp = [0u8; 2048];

    while !stop.load(Ordering::SeqCst) {
        if let Ok(n) = serial.read(&mut tmp) {
            if n > 0 {
                let slice = &tmp[..n];
                for msg in ubx_dec.push_bytes(slice) {
                    match msg {
                        UbxMessage::NavPvt {
                            lat_deg,
                            lon_deg,
                            alt_ellipsoid_m,
                            alt_msl_m,
                            h_acc_m,
                            v_acc_m,
                            num_sv,
                            quality,
                        } => {
                            let (ex, ey, ez) = geodetic_to_ecef(
                                lat_deg.to_radians(),
                                lon_deg.to_radians(),
                                alt_ellipsoid_m,
                                &WGS84,
                            );
                            cur_sol.ecef = [ex, ey, ez];
                            cur_sol.lat_deg = lat_deg;
                            cur_sol.lon_deg = lon_deg;
                            cur_sol.alt_ellipsoid_m = alt_ellipsoid_m;
                            cur_sol.alt_orthometric_m = alt_msl_m;
                            cur_sol.std_enu_m = [h_acc_m, h_acc_m, v_acc_m];
                            cur_sol.num_sats = num_sv;
                            cur_sol.quality = quality;
                        }
                        UbxMessage::RxmRawx(epoch) => {
                            if ppk_rec.load(Ordering::SeqCst) {
                                if rinex_writer.is_none() {
                                    rinex_writer = Rinex3ObsWriter::create(
                                        &cfg.raw_ppk_output_path,
                                        cur_sol.ecef,
                                        "SERIAL_UBX",
                                    )
                                    .ok();
                                }
                                if let Some(w) = rinex_writer.as_mut() {
                                    let _ = w.write_epoch(&epoch);
                                    let _ = w.flush();
                                }
                            } else {
                                rinex_writer = None;
                            }
                        }
                    }
                }
                nmea_buf.push_str(&String::from_utf8_lossy(slice));
                while let Some(pos) = nmea_buf.find('\n') {
                    let line: String = nmea_buf.drain(..=pos).collect();
                    parse_nmea_line(&line, &mut cur_sol);
                }

                let mut tel = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
                tel.raw_bytes_rx += n as u64;
                tel.ppk_epochs_logged = rinex_writer.as_ref().map(|w| w.epochs_written).unwrap_or(0);
                tel.solution = cur_sol.clone();
                tel.status_message = format!(
                    "Série {} ({} bds) — {}",
                    cfg.serial_port_path,
                    cfg.serial_baud_rate,
                    cur_sol.quality.label()
                );
                if cur_sol.quality != FixQuality::NoFix {
                    tel.track_history_wgs84.push((
                        cur_sol.lon_deg,
                        cur_sol.lat_deg,
                        cur_sol.alt_ellipsoid_m,
                        cur_sol.quality,
                    ));
                }
            }
        }

        // Envoi des corrections RTCM3 NTRIP vers le port série du récepteur u-blox/Emlid
        if let Some(ns) = ntrip_stream.as_mut() {
            if cfg.ntrip.send_gga
                && last_gga_tx.elapsed() >= Duration::from_secs(5)
                && cur_sol.quality != FixQuality::NoFix
            {
                let gga = format_gga_sentence(
                    cur_sol.lat_deg,
                    cur_sol.lon_deg,
                    cur_sol.alt_orthometric_m,
                );
                let _ = ns.write_all(gga.as_bytes());
                last_gga_tx = Instant::now();
            }
            if let Ok(rn) = ns.read(&mut tmp) {
                if rn > 0 {
                    let _ = serial.write_all(&tmp[..rn]);
                    let _ = rtcm_dec.push_bytes(&tmp[..rn]);
                    let mut tel = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
                    tel.ntrip_connected = true;
                    tel.rtcm_frames_rx = rtcm_dec.valid_frames;
                    tel.rtcm_msm_epochs_rx = rtcm_dec.msm_frames_decoded;
                    if let Some(id) = rtcm_dec.last_station_id {
                        tel.base_station_id = Some(id);
                    }
                    if let Some(ecef) = rtcm_dec.last_base_ecef {
                        tel.base_ecef = Some(ecef);
                    }
                }
            }
        }
    }
}

fn run_udp_stream_loop(
    cfg: GnssConfig,
    tel_arc: Arc<Mutex<GnssLiveTelemetry>>,
    stop: Arc<AtomicBool>,
) {
    let bind_addr = format!("0.0.0.0:{}", cfg.udp_port);
    let sock = match UdpSocket::bind(&bind_addr) {
        Ok(s) => s,
        Err(e) => {
            let mut t = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
            t.connected = false;
            t.status_message = format!("Erreur UDP {}: {}", bind_addr, e);
            return;
        }
    };
    let _ = sock.set_read_timeout(Some(Duration::from_millis(300)));
    let mut cur_sol = GnssSolution::default();
    let mut tmp = [0u8; 2048];

    while !stop.load(Ordering::SeqCst) {
        if let Ok((n, _)) = sock.recv_from(&mut tmp) {
            let text = String::from_utf8_lossy(&tmp[..n]);
            for line in text.lines() {
                if parse_nmea_line(line, &mut cur_sol) {
                    let mut tel = tel_arc.lock().unwrap_or_else(|e| e.into_inner());
                    tel.solution = cur_sol.clone();
                    tel.status_message = format!("UDP :{} — {}", cfg.udp_port, cur_sol.quality.label());
                    tel.track_history_wgs84.push((
                        cur_sol.lon_deg,
                        cur_sol.lat_deg,
                        cur_sol.alt_ellipsoid_m,
                        cur_sol.quality,
                    ));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_external_bluetooth_nmea_stream_injection() {
        let ctrl = GnssController::default();
        let gga = format_gga_sentence(43.5123, 5.4891, 238.4);
        ctrl.inject_external_raw_bytes(gga.as_bytes(), "Bluetooth Reach RS2+");
        let snap = ctrl.snapshot();
        assert!(snap.connected);
        assert!(snap.raw_bytes_rx > 0);
        assert_eq!(snap.solution.quality, FixQuality::Single);
        assert!((snap.solution.lat_deg - 43.5123).abs() < 1e-4);
    }
}
