//! Panneau de pilotage GNSS RTK (L1 / L1+L2 / L1+L2+L5) & Post-Traitement PPK (RTKLIB-Rust).
//!
//! Pilote les récepteurs :
//! - **Leica** (GS18 T / GS16 / GS15 / GS07 / FLX100)
//! - **Trimble** (R12i / R12 / R10 / R8s / DA2)
//! - **Sokkia / Topcon** (GCX3 / GRX3 / HiPer VR)
//! - **SparkFun & modules u-blox ZED-F9P** (RTK Facet / Express / Torch, ArduSimple simpleRTK2B)
//! - **Emlid Reach** (gamme complète : Reach M+/RS+ L1 → M2 → RS2/RS2+ → RS3 → **RS4 Pro**)

use crate::state::SdncFieldApp;
use egui::{Color32, Pos2, RichText, Stroke, Vec2};
use sdnc_gnss::{
    FixQuality, FrequencyBandMode, GnssLiveTelemetry, GnssMode, ReceiverProfile, ReceiverSource,
};

pub fn render_gnss_panel(ui: &mut egui::Ui, app: &mut SdncFieldApp, tel: &GnssLiveTelemetry) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.heading("GNSS RTK L1/L2 & PPK");
        ui.separator();

        // 1. Carte d'état temps réel (FixQuality, bandes L1/L2, précision, LAMBDA Ratio)
        let q = tel.solution.quality;
        let rgb = q.color_rgb();
        let badge_col = Color32::from_rgb(rgb[0], rgb[1], rgb[2]);

        egui::Frame::group(ui.style())
            .fill(Color32::from_rgb(18, 25, 38))
            .stroke(Stroke::new(1.5_f32, badge_col))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(q.label()).strong().size(15.5).color(badge_col));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(
                            RichText::new(format!("{} sats", tel.solution.num_sats))
                                .strong()
                                .color(Color32::WHITE),
                        );
                        let band_txt = if tel.dual_freq_active { "L1+L2" } else { "L1" };
                        let band_col = if tel.dual_freq_active {
                            Color32::from_rgb(90, 235, 255)
                        } else {
                            Color32::from_rgb(240, 200, 90)
                        };
                        ui.label(RichText::new(band_txt).strong().size(11.0).color(band_col));
                    });
                });

                let sh_cm = ((tel.solution.std_enu_m[0].powi(2) + tel.solution.std_enu_m[1].powi(2))
                    .sqrt())
                    * 100.0;
                let sv_cm = tel.solution.std_enu_m[2] * 100.0;

                ui.label(
                    RichText::new(format!(
                        "σH: {:.1} cm | σV: {:.1} cm | Ratio: {:.1}",
                        sh_cm, sv_cm, tel.solution.lambda_ratio
                    ))
                    .monospace()
                    .color(Color32::LIGHT_GRAY),
                );
                ui.label(
                    RichText::new(format!(
                        "Lat: {:.7}°  Lon: {:.7}°",
                        tel.solution.lat_deg, tel.solution.lon_deg
                    ))
                    .monospace()
                    .size(11.0),
                );
                ui.label(
                    RichText::new(format!(
                        "Alt NGF: {:.3} m (Ellip: {:.3} m) | Base: {:.1} m",
                        tel.solution.alt_orthometric_m,
                        tel.solution.alt_ellipsoid_m,
                        tel.solution.baseline_m
                    ))
                    .monospace()
                    .size(11.0),
                );
                ui.label(
                    RichText::new(&tel.status_message)
                        .size(10.5)
                        .color(Color32::from_rgb(160, 210, 255)),
                );
            });

        ui.add_space(6.0);

        // 2. Boutons principaux de pilotage & capture terrain
        ui.horizontal(|ui| {
            let running = app.gnss.is_running();
            if !running {
                if ui
                    .button(RichText::new("▶ Démarrer GNSS").strong().color(Color32::LIGHT_GREEN))
                    .clicked()
                {
                    app.gnss.start();
                }
            } else if ui
                .button(RichText::new("⏹ Arrêter GNSS").strong().color(Color32::LIGHT_RED))
                .clicked()
            {
                app.gnss.stop();
            }

            ui.checkbox(&mut app.follow_gnss, "Suivre");
        });

        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    q != FixQuality::NoFix,
                    egui::Button::new("📍 Capturer Point RTK"),
                )
                .clicked()
            {
                app.capture_gnss_point(tel);
            }
            if ui.button("🚩 Event PPK").clicked() {
                let idx = tel.event_markers.len() + 1;
                app.gnss.mark_event(format!("EVT_{:03}", idx));
            }
            if ui.button("Effacer").clicked() {
                app.gnss.clear_track();
            }
        });

        // 2.b Moyennage multi-époques haute précision (levé sur canne)
        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Moyennage :");
                ui.add(egui::DragValue::new(&mut app.avg_target_epochs).range(3..=60));
                ui.label("ép.");
                ui.checkbox(&mut app.avg_require_fix, "FIX seul");
            });
            if let Some(sess) = &app.averaging {
                let frac = (sess.samples.len() as f32 / sess.target_epochs.max(1) as f32).clamp(0.0, 1.0);
                ui.add(
                    egui::ProgressBar::new(frac)
                        .text(format!("{}/{} époques", sess.samples.len(), sess.target_epochs)),
                );
                if let Some((_, [sx, sy, sz])) = sess.mean_and_std() {
                    ui.label(
                        RichText::new(format!(
                            "Dispersion : σH={:.1} mm | σV={:.1} mm",
                            (sx * sx + sy * sy).sqrt() * 1000.0,
                            sz * 1000.0
                        ))
                        .size(10.5)
                        .color(Color32::LIGHT_GREEN),
                    );
                }
                if ui.button("Annuler moyennage").clicked() {
                    app.averaging = None;
                }
            } else if ui
                .add_enabled(
                    q != FixQuality::NoFix,
                    egui::Button::new("🎯 Démarrer Moyennage RTK"),
                )
                .clicked()
            {
                app.averaging = Some(crate::state::GnssAveragingSession::start(
                    app.avg_target_epochs,
                    app.avg_require_fix,
                ));
            }
        });

        ui.add_space(6.0);

        // 3. Pilote Matériel Récepteur RTK (Leica, Trimble, Sokkia, SparkFun F9P, Emlid Reach L1 → RS4 Pro)
        egui::CollapsingHeader::new("📡 Pilote Récepteur RTK (Marques & L1/L2)")
            .default_open(true)
            .show(ui, |ui| {
                let mut cfg = app
                    .gnss
                    .config
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .clone();
                let mut changed = false;

                let current_profile = cfg.receiver_profile;
                let mut selected_profile = current_profile;
                egui::ComboBox::from_label("Modèle")
                    .selected_text(current_profile.label())
                    .show_ui(ui, |ui| {
                        for &prof in ReceiverProfile::ALL {
                            ui.selectable_value(&mut selected_profile, prof, prof.label());
                        }
                    });
                if selected_profile != current_profile {
                    cfg.apply_receiver_profile(selected_profile);
                    changed = true;
                }

                let cap = cfg.receiver_profile.capabilities();
                ui.label(
                    RichText::new(format!(
                        "Bandes : {} {}",
                        cap.bands_label,
                        if cap.supports_imu_tilt { "· IMU Tilt" } else { "" }
                    ))
                    .size(10.5)
                    .color(Color32::from_rgb(130, 230, 255)),
                );
                ui.label(
                    RichText::new(cap.protocols_summary)
                        .size(10.0)
                        .color(Color32::GRAY),
                );

                // Bandes de fréquences actives (L1 seul / L1+L2 / L1+L2+L5)
                egui::ComboBox::from_label("Fréquences")
                    .selected_text(cfg.freq_mode.label())
                    .show_ui(ui, |ui| {
                        for fm in [
                            FrequencyBandMode::L1Only,
                            FrequencyBandMode::L1L2,
                            FrequencyBandMode::L1L2L5,
                        ] {
                            changed |= ui
                                .selectable_value(&mut cfg.freq_mode, fm, fm.label())
                                .changed();
                        }
                    });

                ui.horizontal(|ui| {
                    ui.label("Cadence :");
                    for hz in [1u8, 2, 5, 10] {
                        changed |= ui
                            .selectable_value(&mut cfg.measurement_rate_hz, hz, format!("{} Hz", hz))
                            .changed();
                    }
                });

                changed |= ui
                    .checkbox(
                        &mut cfg.auto_init_receiver,
                        "Auto-init (UBX-CFG-VALSET F9P / M8T / GREIS)",
                    )
                    .changed();

                if changed {
                    *app.gnss.config.lock().unwrap_or_else(|e| e.into_inner()) = cfg.clone();
                }

                if ui
                    .button("⚡ Envoyer commandes Init au récepteur")
                    .on_hover_text(
                        "Configure automatiquement NAV-PVT, RXM-RAWX L1/L2 (PPK), NMEA GGA/GST et entrée RTCM3",
                    )
                    .clicked()
                {
                    app.gnss.queue_receiver_init_commands();
                }
            });

        // 4. Liaison & Moteur RTKLIB (Source, Mode, Hauteur canne + ARP)
        egui::CollapsingHeader::new("⚙ Liaison & Moteur RTKLIB-Rust")
            .default_open(true)
            .show(ui, |ui| {
                let mut cfg = app
                    .gnss
                    .config
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .clone();
                let mut changed = false;

                egui::ComboBox::from_label("Liaison")
                    .selected_text(format!("{:?}", cfg.source))
                    .show_ui(ui, |ui| {
                        changed |= ui
                            .selectable_value(
                                &mut cfg.source,
                                ReceiverSource::SimulatedRtk,
                                "Simulateur RTK L1/L2 (EKF+LAMBDA)",
                            )
                            .changed();
                        changed |= ui
                            .selectable_value(
                                &mut cfg.source,
                                ReceiverSource::BluetoothExternal,
                                "Bluetooth SPP (Leica/Trimble/Sokkia/Reach/F9P)",
                            )
                            .changed();
                        changed |= ui
                            .selectable_value(
                                &mut cfg.source,
                                ReceiverSource::SerialPort,
                                "Port Série USB/COM (ZED-F9P / SparkFun / Sokkia)",
                            )
                            .changed();
                        changed |= ui
                            .selectable_value(
                                &mut cfg.source,
                                ReceiverSource::TcpStream,
                                "TCP Wi-Fi (Emlid Reach L1→RS4 Pro / Base)",
                            )
                            .changed();
                        changed |= ui
                            .selectable_value(
                                &mut cfg.source,
                                ReceiverSource::UdpStream,
                                "UDP Récepteur (NMEA)",
                            )
                            .changed();
                        changed |= ui
                            .selectable_value(
                                &mut cfg.source,
                                ReceiverSource::AndroidInternal,
                                "Android GPS Interne (JNI)",
                            )
                            .changed();
                    });

                egui::ComboBox::from_label("Mode")
                    .selected_text(format!("{:?}", cfg.mode))
                    .show_ui(ui, |ui| {
                        changed |= ui
                            .selectable_value(&mut cfg.mode, GnssMode::Single, "Single (SPP)")
                            .changed();
                        changed |= ui
                            .selectable_value(
                                &mut cfg.mode,
                                GnssMode::RtkKinematic,
                                "RTK Cinématique",
                            )
                            .changed();
                        changed |= ui
                            .selectable_value(&mut cfg.mode, GnssMode::RtkStatic, "RTK Statique")
                            .changed();
                        changed |= ui
                            .selectable_value(
                                &mut cfg.mode,
                                GnssMode::PpkCombined,
                                "PPK Combined (Fwd+Bwd)",
                            )
                            .changed();
                    });

                if cfg.source == ReceiverSource::TcpStream {
                    ui.horizontal(|ui| {
                        ui.label("Host:");
                        changed |= ui.text_edit_singleline(&mut cfg.tcp_host).changed();
                    });
                    ui.horizontal(|ui| {
                        ui.label("Port:");
                        changed |= ui.add(egui::DragValue::new(&mut cfg.tcp_port)).changed();
                    });
                } else if cfg.source == ReceiverSource::UdpStream {
                    ui.horizontal(|ui| {
                        ui.label("Port UDP:");
                        changed |= ui.add(egui::DragValue::new(&mut cfg.udp_port)).changed();
                    });
                } else if cfg.source == ReceiverSource::SerialPort {
                    ui.horizontal(|ui| {
                        ui.label("Port:");
                        changed |= ui.text_edit_singleline(&mut cfg.serial_port_path).changed();
                    });
                    ui.horizontal(|ui| {
                        ui.label("Bauds:");
                        changed |= ui
                            .add(egui::DragValue::new(&mut cfg.serial_baud_rate).speed(1200))
                            .changed();
                    });
                } else if cfg.source == ReceiverSource::BluetoothExternal {
                    ui.horizontal(|ui| {
                        ui.label("Récepteur BT:");
                        changed |= ui
                            .text_edit_singleline(&mut cfg.bluetooth_device_name)
                            .changed();
                    });
                    ui.label(
                        RichText::new(format!("Flux brut reçu : {} octets", tel.raw_bytes_rx))
                            .size(10.5)
                            .color(Color32::LIGHT_BLUE),
                    );
                }

                changed |= ui
                    .add(
                        egui::Slider::new(&mut cfg.elevation_mask_deg, 5.0..=35.0)
                            .text("Masque élév. (°)"),
                    )
                    .changed();
                changed |= ui
                    .add(
                        egui::Slider::new(&mut cfg.lambda_ratio_thresh, 1.5..=5.0)
                            .text("Seuil Ratio LAMBDA"),
                    )
                    .changed();
                changed |= ui
                    .add(
                        egui::Slider::new(&mut cfg.antenna_height_m, 0.0..=4.0)
                            .text("Hauteur canne (m)"),
                    )
                    .changed();
                changed |= ui
                    .add(
                        egui::Slider::new(&mut cfg.antenna_arp_offset_m, 0.0..=0.35)
                            .text("Offset ARP→APC (m)"),
                    )
                    .changed();
                ui.label(
                    RichText::new(format!(
                        "Hauteur totale antenne : {:.3} m",
                        cfg.total_antenna_height_m()
                    ))
                    .size(10.5)
                    .color(Color32::GRAY),
                );

                if changed {
                    *app.gnss.config.lock().unwrap_or_else(|e| e.into_inner()) = cfg;
                }
            });

        // 5. Skyplot Polaire & Barres C/N0 (SNR)
        egui::CollapsingHeader::new("🛰 Skyplot Constellation & SNR")
            .default_open(false)
            .show(ui, |ui| {
                render_skyplot(ui, tel);
            });

        // 6. Client NTRIP v1/v2 (Corrections RTCM3 MSM4/MSM7)
        egui::CollapsingHeader::new("🌐 Client NTRIP (RTCM 3.3 MSM4/MSM7)")
            .default_open(false)
            .show(ui, |ui| {
                let mut cfg = app
                    .gnss
                    .config
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .clone();
                let mut changed = false;

                changed |= ui
                    .checkbox(&mut cfg.ntrip.enabled, "Activer NTRIP (Centipède / Orphéon / Teria)")
                    .changed();
                ui.horizontal(|ui| {
                    ui.label("Caster:");
                    changed |= ui.text_edit_singleline(&mut cfg.ntrip.host).changed();
                });
                ui.horizontal(|ui| {
                    ui.label("Port:");
                    changed |= ui.add(egui::DragValue::new(&mut cfg.ntrip.port)).changed();
                    ui.label("Mount:");
                    changed |= ui.text_edit_singleline(&mut cfg.ntrip.mountpoint).changed();
                });
                ui.horizontal(|ui| {
                    ui.label("User:");
                    changed |= ui.text_edit_singleline(&mut cfg.ntrip.username).changed();
                });
                ui.horizontal(|ui| {
                    ui.label("Pass:");
                    changed |= ui
                        .add(egui::TextEdit::singleline(&mut cfg.ntrip.password).password(true))
                        .changed();
                });
                changed |= ui
                    .checkbox(&mut cfg.ntrip.send_gga, "Envoyer GGA (VRS / NEAR)")
                    .changed();
                ui.label(format!(
                    "RTCM3 : {} trames ({} MSM4/7) | Base ID : {:?}",
                    tel.rtcm_frames_rx, tel.rtcm_msm_epochs_rx, tel.base_station_id
                ));

                if changed {
                    *app.gnss.config.lock().unwrap_or_else(|e| e.into_inner()) = cfg;
                }
            });

        // 7. Post-Traitement PPK complet (RINEX 3.04 L1/L2 & fichiers bruts .UBX directs)
        egui::CollapsingHeader::new("🧮 Post-Traitement PPK (RINEX 3 & .UBX L1/L2)")
            .default_open(true)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    let rec = tel.ppk_recording;
                    if !rec {
                        if ui
                            .button(
                                RichText::new("⏺ Enregistrer RINEX L1/L2 (.obs)")
                                    .color(Color32::from_rgb(255, 110, 110)),
                            )
                            .clicked()
                        {
                            app.gnss.set_ppk_recording(true);
                        }
                    } else if ui.button("⏹ Stopper RINEX").clicked() {
                        app.gnss.set_ppk_recording(false);
                    }
                    ui.label(format!("{} ép.", tel.ppk_epochs_logged));
                });

                ui.separator();
                ui.label(
                    RichText::new("Entrées PPK (RINEX 3 .obs/.rnx ou binaire u-blox .ubx) :")
                        .strong()
                        .size(11.0),
                );

                ui.horizontal(|ui| {
                    ui.label("Rover:");
                    ui.text_edit_singleline(&mut app.ppk_rover_path);
                    #[cfg(not(target_os = "android"))]
                    if ui.small_button("📂").on_hover_text("Choisir .obs/.rnx/.ubx Rover").clicked() {
                        if let Some(p) = rfd::FileDialog::new()
                            .add_filter("RINEX / UBX", &["obs", "rnx", "ubx", "24o", "25o", "26o"])
                            .pick_file()
                        {
                            app.ppk_rover_path = p.to_string_lossy().to_string();
                        }
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Base :");
                    ui.text_edit_singleline(&mut app.ppk_base_path);
                    #[cfg(not(target_os = "android"))]
                    if ui.small_button("📂").on_hover_text("Choisir .obs/.rnx/.ubx Base RGP").clicked() {
                        if let Some(p) = rfd::FileDialog::new()
                            .add_filter("RINEX / UBX", &["obs", "rnx", "ubx", "24o", "25o", "26o"])
                            .pick_file()
                        {
                            app.ppk_base_path = p.to_string_lossy().to_string();
                        }
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Nav  :");
                    ui.text_edit_singleline(&mut app.ppk_nav_path);
                    #[cfg(not(target_os = "android"))]
                    if ui.small_button("📂").on_hover_text("Choisir .nav/.rnx Éphémérides").clicked() {
                        if let Some(p) = rfd::FileDialog::new()
                            .add_filter("RINEX NAV", &["nav", "rnx", "24n", "25n", "26n", "26p"])
                            .pick_file()
                        {
                            app.ppk_nav_path = p.to_string_lossy().to_string();
                        }
                    }
                });

                ui.horizontal(|ui| {
                    ui.checkbox(&mut app.ppk_combined, "Combined (Fwd+Bwd)");
                    ui.checkbox(&mut app.ppk_static_mode, "Statique (Pivot)");
                });
                ui.horizontal(|ui| {
                    ui.label("H. antenne base (m) :");
                    ui.add(
                        egui::DragValue::new(&mut app.ppk_base_antenna_h_m)
                            .speed(0.01)
                            .range(0.0..=10.0),
                    );
                });

                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(!tel.ppk_running, egui::Button::new("⚙ Lancer Calcul PPK"))
                        .clicked()
                    {
                        app.launch_ppk_from_files();
                    }
                    if ui
                        .add_enabled(!tel.ppk_running, egui::Button::new("🧪 Démo PPK L1+L2"))
                        .clicked()
                    {
                        app.gnss.run_ppk_demo_async();
                    }
                });

                if let Some(rep) = &tel.last_ppk_report {
                    ui.separator();
                    let band_str = if rep.used_dual_freq_l1_l2 { "L1+L2" } else { "L1" };
                    ui.label(
                        RichText::new(format!(
                            "Rapport PPK ({}) : {}/{} FIX ({:.1}%)",
                            band_str, rep.fix_epochs, rep.total_epochs, rep.fix_rate_pct
                        ))
                        .strong()
                        .color(Color32::LIGHT_GREEN),
                    );
                    ui.label(format!(
                        "Float: {} | Single: {} | σH: {:.2} cm | σV: {:.2} cm | Base: {:.0} m",
                        rep.float_epochs,
                        rep.single_epochs,
                        rep.mean_std_h_cm,
                        rep.mean_std_v_cm,
                        rep.baseline_mean_m
                    ));
                    if let Some(ref st) = rep.static_solution {
                        ui.label(
                            RichText::new(format!(
                                "Pivot Statique : {:.7}°, {:.7}°, Z={:.3} m",
                                st.lat_deg, st.lon_deg, st.alt_orthometric_m
                            ))
                            .size(10.5)
                            .color(Color32::from_rgb(130, 235, 255)),
                        );
                    }
                    if !rep.event_solutions.is_empty() {
                        ui.label(
                            RichText::new(format!(
                                "Événements recalés PPK : {}",
                                rep.event_solutions.len()
                            ))
                            .size(10.5)
                            .color(Color32::GOLD),
                        );
                    }
                    ui.horizontal(|ui| {
                        if ui.button("💾 Carte (.gpkg)").clicked() {
                            app.import_ppk_report_as_layer(tel);
                        }
                        if ui.button("📄 Exporter CSV").clicked() {
                            app.export_ppk_report_csv(tel);
                        }
                    });
                }
            });
    });
}

fn render_skyplot(ui: &mut egui::Ui, tel: &GnssLiveTelemetry) {
    let size = Vec2::new(ui.available_width().min(220.0), 185.0);
    let (resp, painter) = ui.allocate_painter(size, egui::Sense::hover());
    let rect = resp.rect;
    let center = rect.center();
    let radius = (rect.width().min(rect.height()) * 0.44).max(40.0);

    painter.rect_filled(rect, 6.0, Color32::from_rgb(12, 16, 26));
    let ring_stroke = Stroke::new(0.8_f32, Color32::from_rgb(55, 75, 105));
    for frac in [1.0, 0.66, 0.33] {
        painter.circle_stroke(center, radius * frac, ring_stroke);
    }
    painter.line_segment(
        [Pos2::new(center.x - radius, center.y), Pos2::new(center.x + radius, center.y)],
        ring_stroke,
    );
    painter.line_segment(
        [Pos2::new(center.x, center.y - radius), Pos2::new(center.x, center.y + radius)],
        ring_stroke,
    );
    painter.text(
        Pos2::new(center.x, center.y - radius - 7.0),
        egui::Align2::CENTER_CENTER,
        "N",
        egui::FontId::proportional(10.0),
        Color32::LIGHT_GRAY,
    );

    for sat in &tel.solution.satellites {
        let el_clamped = sat.elevation_deg.clamp(0.0, 90.0);
        let r = radius * (1.0 - el_clamped / 90.0);
        let az_rad = sat.azimuth_deg.to_radians();
        let pos = Pos2::new(center.x + r * az_rad.sin(), center.y - r * az_rad.cos());
        let col = if sat.used_in_fix {
            Color32::from_rgb(50, 235, 120)
        } else {
            Color32::from_rgb(235, 160, 50)
        };
        painter.circle_filled(pos, 5.5, col);
        painter.text(
            pos + Vec2::new(7.0, 0.0),
            egui::Align2::LEFT_CENTER,
            sat.sat.code(),
            egui::FontId::proportional(9.0),
            Color32::WHITE,
        );
    }
}
