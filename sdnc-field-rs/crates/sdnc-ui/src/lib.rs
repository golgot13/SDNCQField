//! `sdnc-ui` — Interface graphique multiplateforme (Android, Windows, Linux) pour SDNCQField.
//!
//! Remplace Qt/QML + QGIS par `egui` + `wgpu` (Vulkan) en réutilisant les modules
//! cartographiques 2D et 3D adaptés d'`AEROPHOTOGRAM` et le moteur RTK/PPK `sdnc-gnss`.

pub mod gnss_panel;
pub mod layers_panel;
pub mod state;
pub mod viewport_2d;
pub mod viewport_3d;

pub use state::{MapTool, SdncFieldApp, ViewMode};

use egui::{Color32, RichText};
use sdnc_data::FieldWidgetType;

impl SdncFieldApp {
    /// Point d'entrée de rendu unique partagé entre Desktop (`eframe`) et Android (`winit + egui-wgpu`).
    pub fn render_ui(&mut self, ctx: &egui::Context) {
        let gnss_tel = self.gnss.snapshot();
        if self.gnss.is_running() || gnss_tel.ppk_running {
            ctx.request_repaint_after(std::time::Duration::from_millis(120));
        }

        // 1. Barre d'outils supérieure (adaptée tactile & desktop)
        egui::TopBottomPanel::top("sdnc_top_toolbar").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(
                    RichText::new("SDNCQField-RS")
                        .strong()
                        .size(15.0)
                        .color(Color32::from_rgb(80, 220, 255)),
                );
                ui.separator();

                ui.toggle_value(&mut self.show_layers_panel, "📚 Couches");
                ui.toggle_value(&mut self.show_gnss_panel, "🛰 GNSS RTK/PPK");
                ui.separator();

                ui.selectable_value(&mut self.view_mode, ViewMode::Map2D, "🗺 2D");
                ui.selectable_value(&mut self.view_mode, ViewMode::Surface3D, "🏔 3D Vulkan");
                ui.selectable_value(&mut self.view_mode, ViewMode::Split2D3D, "◫ 2D+3D");
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

        // 3. Panneau gauche : Projet & Couches
        if self.show_layers_panel {
            egui::SidePanel::left("sdnc_layers_panel")
                .default_width(275.0)
                .resizable(true)
                .show(ctx, |ui| {
                    layers_panel::render_layers_panel(ui, self);
                });
        }

        // 4. Panneau droit : Pilotage GNSS RTK & PPK (RTKLIB-Rust)
        if self.show_gnss_panel {
            egui::SidePanel::right("sdnc_gnss_panel")
                .default_width(305.0)
                .resizable(true)
                .show(ctx, |ui| {
                    gnss_panel::render_gnss_panel(ui, self, &gnss_tel);
                });
        }

        // 5. Zone centrale : Cartographie 2D / 3D Vulkan Compute
        egui::CentralPanel::default().show(ctx, |ui| match self.view_mode {
            ViewMode::Map2D => viewport_2d::render_viewport_2d(ui, self, &gnss_tel),
            ViewMode::Surface3D => viewport_3d::render_viewport_3d(ui, self, &gnss_tel),
            ViewMode::Split2D3D => {
                ui.columns(2, |cols| {
                    viewport_2d::render_viewport_2d(&mut cols[0], self, &gnss_tel);
                    viewport_3d::render_viewport_3d(&mut cols[1], self, &gnss_tel);
                });
            }
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
                            .button(RichText::new("✔ Enregistrer").strong().color(Color32::LIGHT_GREEN))
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
