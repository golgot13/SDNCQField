//! `sdnc-ui` — Interface graphique 2D multiplateforme (Android, Windows, Linux) pour SDNCQField.
//!
//! Remplace Qt/QML + QGIS par `egui` + `wgpu` (Vulkan 2D) pour afficher à haute performance :
//! - Plans cadastraux **DXF, DXF-PCI et EDIGÉO** (triangulation GPU + segments instanciés)
//! - Flux **IGN Géoplateforme Raster (WMTS)** et **Vecteurs (WFS 2.0 GeoJSON)**
//! - Couches terrain **GeoPackage (`.gpkg`)** & pilotage GNSS **RTK / PPK** (`sdnc-gnss`)

pub mod gnss_panel;
pub mod layers_panel;
pub mod state;
pub mod viewport_2d;

pub use state::{MapTool, SdncFieldApp};

use egui::{Color32, RichText};
use sdnc_data::FieldWidgetType;
use std::sync::atomic::Ordering;

impl SdncFieldApp {
    /// Point d'entrée de rendu unique partagé entre Desktop (`eframe`) et Android (`eframe / wgpu`).
    pub fn render_ui(&mut self, ctx: &egui::Context) {
        let screen_w = ctx.input(|i| i.viewport_rect().width());
        let is_phone = screen_w < 780.0;
        let is_tablet = (780.0..1150.0).contains(&screen_w);

        // Sur téléphone portable, démarre en plein écran carte et agrandit les zones tactiles
        if !self.mobile_layout_initialized && screen_w > 0.0 {
            if is_phone {
                self.show_layers_panel = false;
                self.show_gnss_panel = false;
            } else if is_tablet {
                self.show_layers_panel = false;
                self.show_gnss_panel = true;
            }
            self.mobile_layout_initialized = true;
        }
        if is_phone || is_tablet {
            let mut style = (*ctx.style()).clone();
            style.spacing.interact_size = egui::vec2(42.0, 34.0);
            style.spacing.button_padding = egui::vec2(10.0, 6.0);
            ctx.set_style(style);
        }

        // Intégrer les couches IGN WFS éventuellement reçues en arrière-plan
        self.poll_wfs_inbox();

        let gnss_tel = self.gnss.snapshot();
        self.tick_gnss_averaging(&gnss_tel);
        if self.gnss.is_running()
            || gnss_tel.ppk_running
            || self.averaging.is_some()
            || self.wfs_loading.load(Ordering::SeqCst)
        {
            ctx.request_repaint_after(std::time::Duration::from_millis(120));
        }

        // 1. Barre d'outils supérieure (adaptée téléphone, tablette & PC)
        egui::TopBottomPanel::top("sdnc_top_toolbar").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(
                    RichText::new(if is_phone { "SDNC 2D" } else { "SDNCQField-RS 2D" })
                        .strong()
                        .size(15.0)
                        .color(Color32::from_rgb(80, 220, 255)),
                );
                ui.separator();

                if ui.toggle_value(&mut self.show_layers_panel, "📚 Carto & Cadastre").clicked()
                    && is_phone
                    && self.show_layers_panel
                {
                    self.show_gnss_panel = false;
                }
                if ui.toggle_value(&mut self.show_gnss_panel, "🛰 GNSS").clicked()
                    && is_phone
                    && self.show_gnss_panel
                {
                    self.show_layers_panel = false;
                }
                ui.separator();

                for tool in [
                    MapTool::Navigate,
                    MapTool::Identify,
                    MapTool::MeasureDistance,
                    MapTool::MeasureArea,
                    MapTool::DigitizePoint,
                    MapTool::DigitizeLine,
                    MapTool::DigitizePolygon,
                ] {
                    if ui
                        .selectable_label(self.active_tool == tool, tool.label())
                        .clicked()
                    {
                        self.active_tool = tool;
                        self.draft_vertices.clear();
                    }
                }

                ui.separator();
                if ui
                    .add_enabled(!self.undo_stack.is_empty(), egui::Button::new("↩ Undo"))
                    .clicked()
                {
                    self.undo_last_edit();
                }

                // Pastille statut GNSS toujours visible dans la barre supérieure
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let q = gnss_tel.solution.quality;
                    let rgb = q.color_rgb();
                    ui.label(
                        RichText::new(format!("● {}", q.label()))
                            .strong()
                            .color(Color32::from_rgb(rgb[0], rgb[1], rgb[2])),
                    );
                });
            });
        });

        // 2. Barre d'état inférieure
        egui::TopBottomPanel::bottom("sdnc_status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new(&self.status_banner)
                        .size(11.0)
                        .color(Color32::LIGHT_GRAY),
                );
                if let Some(info) = &self.identified_info {
                    ui.separator();
                    ui.label(RichText::new(info).size(11.0).color(Color32::GOLD));
                }
            });
        });

        // 3. Panneau gauche : Carto 2D, Cadastre DXF/EDIGÉO & Flux IGN
        let panel_w = if is_phone {
            (screen_w * 0.86).clamp(240.0, 340.0)
        } else {
            295.0
        };
        if self.show_layers_panel {
            egui::SidePanel::left("sdnc_layers_panel")
                .default_width(panel_w)
                .resizable(!is_phone)
                .show(ctx, |ui| {
                    layers_panel::render_layers_panel(ui, self);
                });
        }

        // 4. Panneau droit : Pilotage GNSS RTK & PPK (RTKLIB-Rust)
        if self.show_gnss_panel {
            egui::SidePanel::right("sdnc_gnss_panel")
                .default_width(if is_phone { panel_w } else { 305.0 })
                .resizable(!is_phone)
                .show(ctx, |ui| {
                    gnss_panel::render_gnss_panel(ui, self, &gnss_tel);
                });
        }

        // 5. Zone centrale : Cartographie 2D Haute Performance (Vulkan GPU)
        egui::CentralPanel::default().show(ctx, |ui| {
            viewport_2d::render_viewport_2d(ui, self, &gnss_tel);
        });

        // 6. Fenêtre modale de formulaire attributaire terrain
        let mut commit_form = false;
        let mut cancel_form = false;
        if let Some(form) = &mut self.pending_form {
            let fields_spec = self
                .layers
                .iter()
                .find(|l| l.spec.id == form.layer_id)
                .map(|l| l.spec.form_fields.clone())
                .unwrap_or_default();

            egui::Window::new("Formulaire Attributaire Terrain")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
                .show(ctx, |ui| {
                    ui.label(
                        RichText::new(format!("Qualité acquisition : {}", form.gnss_summary))
                            .color(Color32::LIGHT_GREEN),
                    );
                    ui.separator();

                    for spec in &fields_spec {
                        let entry = form
                            .values
                            .entry(spec.name.clone())
                            .or_insert_with(|| spec.default_value.clone().unwrap_or_default());
                        ui.horizontal(|ui| {
                            ui.label(&spec.label);
                            match &spec.widget {
                                FieldWidgetType::Choice(opts) => {
                                    egui::ComboBox::from_id_salt(&spec.name)
                                        .selected_text(entry.as_str())
                                        .show_ui(ui, |ui| {
                                            for o in opts {
                                                ui.selectable_value(entry, o.clone(), o);
                                            }
                                        });
                                }
                                _ => {
                                    ui.text_edit_singleline(entry);
                                }
                            }
                        });
                    }

                    ui.separator();
                    ui.horizontal(|ui| {
                        if ui
                            .button(
                                RichText::new("✔ Enregistrer")
                                    .strong()
                                    .color(Color32::LIGHT_GREEN),
                            )
                            .clicked()
                        {
                            commit_form = true;
                        }
                        if ui.button("Annuler").clicked() {
                            cancel_form = true;
                        }
                    });
                });
        }
        if commit_form {
            self.commit_pending_form();
        } else if cancel_form {
            self.pending_form = None;
        }
    }
}

impl eframe::App for SdncFieldApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.render_ui(ctx);
    }
}
