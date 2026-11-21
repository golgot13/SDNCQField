//! Panneau latéral Projet, Plan Cadastral (DXF / DXF-PCI / EDIGÉO), Flux IGN Raster (WMTS) & Vecteurs (WFS).

use crate::state::SdncFieldApp;
use egui::{Color32, RichText};
use sdnc_geo::{search_location, BasemapLayerType, FrenchCrs, IgnWfsLayer};
use std::sync::atomic::Ordering;

pub fn render_layers_panel(ui: &mut egui::Ui, app: &mut SdncFieldApp) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.heading("Carto 2D, Cadastre & IGN");
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
            if ui.button("➕ GPKG / GeoJSON").clicked() {
                if let Some(p) = rfd::FileDialog::new()
                    .add_filter("SIG Vecteur", &["gpkg", "geojson", "json"])
                    .pick_file()
                {
                    app.import_vector_file(&p);
                }
            }
        });

        ui.separator();

        // 2. Plan Cadastral (DXF, DXF-PCI, EDIGÉO)
        egui::CollapsingHeader::new("🏛 Plan Cadastral (DXF / DXF-PCI / EDIGÉO)")
            .default_open(true)
            .show(ui, |ui| {
                #[cfg(not(target_os = "android"))]
                {
                    ui.horizontal(|ui| {
                        if ui
                            .button("📐 Importer DXF / DXF-PCI")
                            .on_hover_text("AutoCAD DXF ou DGFiP DXF-PCI avec détection CRS")
                            .clicked()
                        {
                            if let Some(p) = rfd::FileDialog::new()
                                .add_filter("Plan DXF / DXF-PCI", &["dxf", "DXF"])
                                .pick_file()
                            {
                                app.import_cadastral_dxf(&p);
                            }
                        }
                    });
                    ui.horizontal(|ui| {
                        if ui
                            .button("📄 EDIGÉO (.THF)")
                            .on_hover_text("Fichier index .THF d'une feuille cadastrale DGFiP")
                            .clicked()
                        {
                            if let Some(p) = rfd::FileDialog::new()
                                .add_filter("Cadastre EDIGÉO (THF)", &["thf", "THF"])
                                .pick_file()
                            {
                                app.import_cadastral_edigeo(&p);
                            }
                        }
                        if ui
                            .button("📁 Dossier EDIGÉO")
                            .on_hover_text("Dossier contenant un lot EDIGÉO (.THF/.GEO/.VEC)")
                            .clicked()
                        {
                            if let Some(dir) = rfd::FileDialog::new().pick_folder() {
                                app.import_cadastral_edigeo(&dir);
                            }
                        }
                    });
                }

                if ui
                    .button("✨ Charger Îlot Cadastral Démo (PCI)")
                    .on_hover_text(
                        "Charge une section cadastrale exemple (parcelles, bâtiments concaves L/U, bornes)",
                    )
                    .clicked()
                {
                    app.load_demo_cadastre();
                }

                let tris = app.vector_gpu_batch.fill_vertices.len() / 3;
                let segs = app.vector_gpu_batch.segments.len();
                ui.label(
                    RichText::new(format!(
                        "GPU Vulkan 2D : {} tris · {} segments VRAM",
                        tris, segs
                    ))
                    .size(10.5)
                    .color(Color32::from_rgb(130, 225, 255)),
                );
            });

        ui.separator();

        // 3. Projection active (28 CRS Français + UTM) & Recherche BAN
        egui::ComboBox::from_label("CRS")
            .selected_text(format!(
                "EPSG:{} ({:?})",
                app.project.active_crs.epsg(),
                app.project.active_crs
            ))
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
                        app.mark_vector_gpu_dirty();
                    }
                }
            });

        ui.horizontal(|ui| {
            ui.text_edit_singleline(&mut app.search_query);
            if ui
                .button("🔍")
                .on_hover_text("Aller à une coordonnée ou adresse BAN IGN")
                .clicked()
            {
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

        // 4. Flux IGN Raster (WMTS Géoplateforme + Cache Hors-Ligne SQLite)
        egui::CollapsingHeader::new("🗺 Flux IGN Raster (WMTS / Hors-Ligne)")
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
                    BasemapLayerType::IgnParcellaireExpress,
                    BasemapLayerType::IgnEstompage,
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

        // 5. Flux IGN Vecteurs (WFS 2.0 Géoplateforme GeoJSON)
        egui::CollapsingHeader::new("⚡ Flux IGN Vecteurs (WFS Géoplateforme)")
            .default_open(true)
            .show(ui, |ui| {
                for &wfs_layer in IgnWfsLayer::ALL {
                    let mut checked = app.wfs_selected_layers.contains(&wfs_layer);
                    if ui.checkbox(&mut checked, wfs_layer.label()).changed() {
                        if checked {
                            app.wfs_selected_layers.push(wfs_layer);
                        } else {
                            app.wfs_selected_layers.retain(|&l| l != wfs_layer);
                        }
                    }
                }
                let is_loading = app.wfs_loading.load(Ordering::SeqCst);
                let btn_label = if is_loading {
                    "⏳ Téléchargement IGN WFS..."
                } else {
                    "⬇ Charger Vecteurs IGN sur la vue"
                };
                if ui
                    .add_enabled(!is_loading, egui::Button::new(btn_label))
                    .clicked()
                {
                    app.request_ign_wfs_for_current_view(ui.ctx().clone());
                }
                ui.label(
                    RichText::new("Astuce : sauvegardez en .gpkg pour conserver les vecteurs IGN hors-ligne.")
                        .size(10.0)
                        .color(Color32::GRAY),
                );
            });

        ui.separator();

        // 6. Liste des couches vectorielles actives
        ui.label(RichText::new("Couches Vectorielles :").strong());
        let mut gpu_dirty = false;
        let mut remove_idx: Option<usize> = None;

        for (idx, layer) in app.layers.iter_mut().enumerate() {
            let is_active = idx == app.active_layer_idx;
            let [sr, sg, sb, _] = layer.spec.style.stroke_rgba;
            egui::Frame::group(ui.style())
                .fill(if is_active {
                    Color32::from_rgb(24, 38, 56)
                } else {
                    Color32::from_rgb(18, 24, 34)
                })
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        if ui.checkbox(&mut layer.spec.visible, "").changed() {
                            gpu_dirty = true;
                        }
                        // Pastille couleur de la couche
                        ui.label(
                            RichText::new("■")
                                .size(13.0)
                                .color(Color32::from_rgb(sr, sg, sb)),
                        );
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
                        if idx >= 3
                            && ui
                                .small_button("🗑")
                                .on_hover_text("Retirer cette couche de la vue")
                                .clicked()
                        {
                            remove_idx = Some(idx);
                        }
                    });
                });
        }

        if let Some(ridx) = remove_idx {
            app.layers.remove(ridx);
            if app.active_layer_idx >= app.layers.len() && !app.layers.is_empty() {
                app.active_layer_idx = app.layers.len() - 1;
            }
            gpu_dirty = true;
        }
        if gpu_dirty {
            app.mark_vector_gpu_dirty();
        }

        ui.separator();
        ui.checkbox(&mut app.vp2d.show_grid, "Afficher carroyage métrique");
        ui.checkbox(&mut app.snapping_enabled, "Accrochage sommets (Snapping)");
    });
}
