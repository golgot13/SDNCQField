//! Panneau latéral Projet, Couches (.gpkg/.json/DXF/GeoJSON), Fonds IGN/OSM et Géocodage.

use crate::state::SdncFieldApp;
use egui::{Color32, RichText};
use sdnc_geo::{search_location, BasemapLayerType, FrenchCrs};

pub fn render_layers_panel(ui: &mut egui::Ui, app: &mut SdncFieldApp) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.heading("Projet & Couches SIG");
        ui.separator();

        // 1. Projet natif léger (.sdnc.json + .gpkg)
        ui.horizontal(|ui| {
            ui.label("Projet :");
            ui.text_edit_singleline(&mut app.project.name);
        });

        ui.horizontal(|ui| {
            if ui.button("💾 Sauvegarder (.gpkg + .json)").clicked() {
                app.save_project();
            }
        });

        #[cfg(not(target_os = "android"))]
        ui.horizontal(|ui| {
            if ui.button("📂 Ouvrir Projet").clicked() {
                if let Some(p) = rfd::FileDialog::new()
                    .add_filter("Projet SDNC", &["json"])
                    .pick_file()
                {
                    app.load_project_file(&p);
                }
            }
            if ui.button("➕ Importer Vecteur").clicked() {
                if let Some(p) = rfd::FileDialog::new()
                    .add_filter("SIG / CAO", &["gpkg", "geojson", "json", "dxf"])
                    .pick_file()
                {
                    app.import_vector_file(&p);
                }
            }
        });

        ui.separator();

        // 2. Projection active (28 CRS Français + UTM)
        egui::ComboBox::from_label("CRS")
            .selected_text(format!("EPSG:{} ({:?})", app.project.active_crs.epsg(), app.project.active_crs))
            .show_ui(ui, |ui| {
                for &crs in FrenchCrs::ALL {
                    if ui
                        .selectable_value(
                            &mut app.project.active_crs,
                            crs,
                            format!("EPSG:{} — {:?}", crs.epsg(), crs),
                        )
                        .clicked()
                    {
                        app.vp3d.invalidate_terrain();
                    }
                }
            });

        // 3. Recherche rapide coordonnées / BAN IGN
        ui.horizontal(|ui| {
            ui.text_edit_singleline(&mut app.search_query);
            if ui.button("🔍").on_hover_text("Aller à une coordonnée ou adresse BAN").clicked() {
                let results = search_location(&app.search_query, app.project.active_crs);
                if let Some(first) = results.first() {
                    app.project.center_xy = Some((first.x_proj, first.y_proj));
                    app.vp2d.pan_offset = egui::Vec2::ZERO;
                    app.vp2d.zoom_level = 2.0;
                    app.status_banner = format!("Centré sur : {}", first.label);
                } else {
                    app.status_banner = "Aucun résultat de géocodage".to_string();
                }
            }
        });

        ui.separator();

        // 4. Fonds cartographiques (IGN Géoplateforme & OSM + Hors-Ligne SQLite)
        egui::CollapsingHeader::new("Fonds de Carte (WMTS / Hors-Ligne)")
            .default_open(true)
            .show(ui, |ui| {
                let mut offline = app.tile_client.is_offline_mode();
                if ui
                    .checkbox(&mut offline, "Mode Terrain Hors-Ligne (Cache SQLite)")
                    .changed()
                {
                    app.tile_client.set_offline_mode(offline);
                    app.project.basemap.offline_mode = offline;
                }

                for bm in [
                    BasemapLayerType::IgnOrtho,
                    BasemapLayerType::IgnPlan,
                    BasemapLayerType::IgnCadastre,
                    BasemapLayerType::IgnAltimetrieMnt,
                    BasemapLayerType::OpenStreetMap,
                ] {
                    let mut active = app.project.basemap.active_layers.contains(&bm);
                    if ui.checkbox(&mut active, bm.label()).changed() {
                        if active {
                            app.project.basemap.active_layers.push(bm);
                        } else {
                            app.project.basemap.active_layers.retain(|&x| x != bm);
                        }
                    }
                }
                ui.add(
                    egui::Slider::new(&mut app.project.basemap.opacity, 0.1..=1.0).text("Opacité"),
                );
            });

        ui.separator();

        // 5. Liste des couches vectorielles
        ui.label(RichText::new("Couches Vectorielles :").strong());
        let mut terrain_changed = false;
        for (idx, layer) in app.layers.iter_mut().enumerate() {
            let is_active = idx == app.active_layer_idx;
            egui::Frame::group(ui.style())
                .fill(if is_active {
                    Color32::from_rgb(24, 38, 56)
                } else {
                    Color32::from_rgb(18, 24, 34)
                })
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        if ui.checkbox(&mut layer.spec.visible, "").changed() {
                            terrain_changed = true;
                        }
                        let mut label_text = format!(
                            "{} ({})",
                            layer.spec.name,
                            layer.data.features.len()
                        );
                        if layer.dirty {
                            label_text.push_str(" *");
                        }
                        if ui
                            .selectable_label(is_active, RichText::new(label_text).strong())
                            .clicked()
                        {
                            app.active_layer_idx = idx;
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(format!("EPSG:{}", layer.spec.crs.epsg()))
                                .size(10.0)
                                .color(Color32::GRAY),
                        );
                        ui.checkbox(&mut layer.spec.editable, "Éditable");
                    });
                });
        }
        if terrain_changed {
            app.vp3d.invalidate_terrain();
        }

        ui.separator();
        ui.checkbox(&mut app.vp2d.show_grid, "Afficher carroyage métrique");
        ui.checkbox(&mut app.snapping_enabled, "Accrochage sommets (Snapping)");
        if ui
            .add(egui::Slider::new(&mut app.vp3d.z_exaggeration, 0.5..=8.0).text("Exagération Z 3D"))
            .changed()
        {
            app.vp3d.invalidate_terrain();
        }
    });
}
