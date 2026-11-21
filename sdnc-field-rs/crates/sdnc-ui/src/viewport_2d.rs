//! Module Cartographique 2D adapté d'`AEROPHOTOGRAM` (`viewport_2d.rs`) pour SDNCQField.
//!
//! Fonctionnalités :
//! - Reprojection géodésique des tuiles WMTS IGN / OSM par maillage texturé 4x4 (`render_warped_tile`)
//! - Multi-touch Android (pinch-to-zoom + pan) & souris desktop (zoom ancré sur curseur)
//! - Rendu des couches vectorielles GeoPackage / GeoJSON / DXF / EDIGÉO (reprojection à la volée)
//! - Surimpression GNSS RTK/PPK en direct (position, cercle d'erreur, trace colorée par FixQuality, base)
//! - Saisie terrain (Point / Ligne / Polygone) avec accrochage (snapping) & mesures Distance / Surface

use crate::state::{MapTool, SdncFieldApp};
use egui::{Color32, Pos2, Rect, RichText, Sense, Stroke, TextureHandle, Vec2};
use sdnc_data::{Coordinate, Geometry};
use sdnc_geo::{
    from_wgs84, meridian_convergence, reproject_point_3d, to_wgs84_robust, FrenchCrs,
    TileStreamClient, WmtsTileKey, CONCURRENT_TILE_REQUESTS,
};
use sdnc_gnss::{FixQuality, GnssLiveTelemetry};
use std::collections::HashMap;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};

pub struct Viewport2dState {
    pub pan_offset: Vec2,
    pub zoom_level: f32,
    pub hovered_world: Option<(f64, f64)>,
    pub measure_points: Vec<(f64, f64)>,
    pub texture_cache: HashMap<WmtsTileKey, TextureHandle>,
    pub pending_requests: Arc<Mutex<HashMap<WmtsTileKey, bool>>>,
    pub show_grid: bool,
}

impl Default for Viewport2dState {
    fn default() -> Self {
        Self {
            pan_offset: Vec2::ZERO,
            zoom_level: 1.0,
            hovered_world: None,
            measure_points: Vec::new(),
            texture_cache: HashMap::new(),
            pending_requests: Arc::new(Mutex::new(HashMap::new())),
            show_grid: true,
        }
    }
}

pub fn render_viewport_2d(
    ui: &mut egui::Ui,
    app: &mut SdncFieldApp,
    gnss_tel: &GnssLiveTelemetry,
) {
    let active_crs = app.project.active_crs;
    let available = ui.available_size();
    let (response, painter) = ui.allocate_painter(available, Sense::click_and_drag());
    let rect = response.rect;

    painter.rect_filled(rect, 0.0, Color32::from_rgb(14, 19, 30));

    // 1. Calcul de l'emprise de référence (couches ou centre projet par défaut)
    let (base_cx, base_cy, base_span) = compute_reference_extent(app, gnss_tel);
    let base_scale = (rect.width().min(rect.height()) as f64 * 0.85) / base_span.max(50.0);
    let scale = base_scale * (app.vp2d.zoom_level as f64);

    // Suivi automatique GNSS si activé
    if app.follow_gnss && gnss_tel.solution.quality != FixQuality::NoFix {
        if let Ok((gx, gy)) = from_wgs84(
            active_crs,
            gnss_tel.solution.lon_deg,
            gnss_tel.solution.lat_deg,
        ) {
            app.vp2d.pan_offset = Vec2::new(
                -((gx - base_cx) * scale) as f32,
                ((gy - base_cy) * scale) as f32,
            );
        }
    }

    // 2. Interaction Tactile (Android Pinch-to-Zoom) + Souris (Zero-Drift Zoom)
    if let Some(mt) = ui.input(|i| i.multi_touch()) {
        if (mt.zoom_delta - 1.0).abs() > 1e-4 {
            let old_z = app.vp2d.zoom_level;
            let new_z = (old_z * mt.zoom_delta).clamp(0.002, 5000.0);
            let ratio = new_z / old_z;
            let anchor = mt.center_pos - rect.center();
            app.vp2d.pan_offset = anchor - (anchor - app.vp2d.pan_offset) * ratio;
            app.vp2d.zoom_level = new_z;
        }
        if mt.translation_delta.length_sq() > 0.0 {
            app.vp2d.pan_offset += mt.translation_delta;
            app.follow_gnss = false;
        }
    } else if response.hovered() {
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll.abs() > 0.01 {
            let factor = 1.15f32.powf(scroll * 0.04);
            let old_z = app.vp2d.zoom_level;
            let new_z = (old_z * factor).clamp(0.002, 5000.0);
            let ratio = new_z / old_z;
            if let Some(hover_pos) = response.hover_pos() {
                let anchor = hover_pos - rect.center();
                app.vp2d.pan_offset = anchor - (anchor - app.vp2d.pan_offset) * ratio;
            }
            app.vp2d.zoom_level = new_z;
        }
    }

    if response.dragged() && app.active_tool == MapTool::Navigate {
        app.vp2d.pan_offset += response.drag_delta();
        app.follow_gnss = false;
    }

    let scale = base_scale * (app.vp2d.zoom_level as f64);
    let pan = app.vp2d.pan_offset;
    let world_to_screen = |wx: f64, wy: f64| -> Pos2 {
        let sx = rect.center().x as f64 + pan.x as f64 + (wx - base_cx) * scale;
        let sy = rect.center().y as f64 + pan.y as f64 - (wy - base_cy) * scale;
        Pos2::new(sx as f32, sy as f32)
    };
    let screen_to_world = |sp: Pos2| -> (f64, f64) {
        let wx = base_cx + (sp.x as f64 - rect.center().x as f64 - pan.x as f64) / scale;
        let wy = base_cy - (sp.y as f64 - rect.center().y as f64 - pan.y as f64) / scale;
        (wx, wy)
    };

    // 3. Fond de carte WMTS IGN / OSM avec reprojection par maillage 4x4 (d'AEROPHOTOGRAM)
    render_basemap_tiles(
        ui,
        &painter,
        rect,
        active_crs,
        scale,
        &screen_to_world,
        &world_to_screen,
        app,
    );

    // 4. Carroyage métrique adaptatif
    if app.vp2d.show_grid {
        render_metric_grid(&painter, rect, &screen_to_world, &world_to_screen, scale);
    }

    // 5. Rendu de toutes les couches vectorielles (GeoPackage / GeoJSON / DXF / EDIGÉO)
    let mut snap_candidates: Vec<(Pos2, Coordinate)> = Vec::new();
    for layer in &app.layers {
        if !layer.spec.visible {
            continue;
        }
        let s = &layer.spec.style;
        let alpha_mul = layer.spec.opacity.clamp(0.0, 1.0);
        let stroke_col = Color32::from_rgba_unmultiplied(
            s.stroke_rgba[0],
            s.stroke_rgba[1],
            s.stroke_rgba[2],
            (s.stroke_rgba[3] as f32 * alpha_mul) as u8,
        );
        let fill_col = Color32::from_rgba_unmultiplied(
            s.fill_rgba[0],
            s.fill_rgba[1],
            s.fill_rgba[2],
            (s.fill_rgba[3] as f32 * alpha_mul) as u8,
        );
        let stroke = Stroke::new(s.stroke_width, stroke_col);

        for feat in &layer.data.features {
            let Some(geom) = &feat.geometry else {
                continue;
            };
            let label = s
                .label_field
                .as_ref()
                .and_then(|fld| feat.get_string(fld));
            draw_geometry_2d(
                &painter,
                geom,
                layer.spec.crs,
                active_crs,
                &world_to_screen,
                stroke,
                fill_col,
                s.point_radius,
                label,
                &mut snap_candidates,
            );
        }
    }

    // 6. Surimpression GNSS RTK / PPK (Base + Ligne de base + Trace colorée + Rover + Cercle σ)
    render_gnss_overlay_2d(&painter, active_crs, scale, &world_to_screen, gnss_tel);

    // 7. Curseur, Accrochage (Snapping) et Gestion des clics (Mesure / Saisie / Identification)
    let mut snapped_world: Option<Coordinate> = None;
    if let Some(hover) = response.hover_pos() {
        let (wx, wy) = screen_to_world(hover);
        app.vp2d.hovered_world = Some((wx, wy));

        if app.snapping_enabled
            && matches!(
                app.active_tool,
                MapTool::DigitizePoint | MapTool::DigitizeLine | MapTool::DigitizePolygon
            )
        {
            let mut best: Option<(f32, Pos2, Coordinate)> = None;
            for &(sp, c) in &snap_candidates {
                let d = sp.distance(hover);
                if d <= app.snapping_radius_px && best.map_or(true, |(bd, _, _)| d < bd) {
                    best = Some((d, sp, c));
                }
            }
            if let Some((_, sp, c)) = best {
                snapped_world = Some(c);
                painter.circle_stroke(sp, 8.0, Stroke::new(2.0_f32, Color32::from_rgb(255, 50, 220)));
            }
        }

        // Réticule viseur discret
        let cross_col = Color32::from_rgba_unmultiplied(0, 240, 255, 110);
        painter.line_segment(
            [Pos2::new(hover.x - 12.0, hover.y), Pos2::new(hover.x + 12.0, hover.y)],
            (1.0, cross_col),
        );
        painter.line_segment(
            [Pos2::new(hover.x, hover.y - 12.0), Pos2::new(hover.x, hover.y + 12.0)],
            (1.0, cross_col),
        );
    }

    if response.clicked() {
        if let Some(click_pos) = response.interact_pointer_pos() {
            let coord = snapped_world.unwrap_or_else(|| {
                let (wx, wy) = screen_to_world(click_pos);
                Coordinate::new_3d(wx, wy, 240.0)
            });

            match app.active_tool {
                MapTool::Navigate => {}
                MapTool::Identify => {
                    app.identified_info = identify_nearest_feature(app, active_crs, click_pos, &world_to_screen);
                }
                MapTool::MeasureDistance | MapTool::MeasureArea => {
                    app.vp2d.measure_points.push((coord.x, coord.y));
                }
                MapTool::DigitizePoint => {
                    app.open_attribute_form(Geometry::Point { coordinates: coord }, gnss_tel);
                }
                MapTool::DigitizeLine | MapTool::DigitizePolygon => {
                    app.draft_vertices.push(coord);
                }
            }
        }
    }

    // Double-clic ou clic droit termine une saisie Ligne / Polygone ou réinitialise la mesure
    if response.double_clicked() || response.secondary_clicked() {
        match app.active_tool {
            MapTool::DigitizeLine if app.draft_vertices.len() >= 2 => {
                let coords = std::mem::take(&mut app.draft_vertices);
                app.open_attribute_form(Geometry::LineString { coordinates: coords }, gnss_tel);
            }
            MapTool::DigitizePolygon if app.draft_vertices.len() >= 3 => {
                let mut ring = std::mem::take(&mut app.draft_vertices);
                ring.push(ring[0]);
                app.open_attribute_form(Geometry::Polygon { coordinates: vec![ring] }, gnss_tel);
            }
            MapTool::MeasureDistance | MapTool::MeasureArea => {
                app.vp2d.measure_points.clear();
            }
            _ => {}
        }
    }

    // 8. Rendu du brouillon de numérisation (Rubberband) & des mesures
    render_draft_and_measures(&painter, app, &world_to_screen);

    // 9. HUD Cartographique (Flèche Nord convergence méridien, Échelle dynamique, Coordonnées)
    render_hud_2d(&painter, rect, active_crs, scale, app, gnss_tel);

    // Boutons flottants tactiles (+ / - / Recentrer / Finir saisie)
    egui::Area::new(ui.id().with("map2d_touch_controls"))
        .fixed_pos(Pos2::new(rect.max.x - 132.0, rect.min.y + 12.0))
        .show(ui.ctx(), |ui| {
            egui::Frame::popup(ui.style())
                .fill(Color32::from_rgba_unmultiplied(15, 22, 36, 220))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        if ui.button(" + ").on_hover_text("Zoom avant").clicked() {
                            app.vp2d.zoom_level = (app.vp2d.zoom_level * 1.4).clamp(0.002, 5000.0);
                        }
                        if ui.button(" - ").on_hover_text("Zoom arrière").clicked() {
                            app.vp2d.zoom_level = (app.vp2d.zoom_level / 1.4).clamp(0.002, 5000.0);
                        }
                        if ui.button("Centrer").clicked() {
                            app.vp2d.pan_offset = Vec2::ZERO;
                            app.vp2d.zoom_level = 1.0;
                        }
                    });
                    if !app.draft_vertices.is_empty() {
                        ui.separator();
                        if ui
                            .button(RichText::new("Valider géométrie").color(Color32::LIGHT_GREEN))
                            .clicked()
                        {
                            if app.active_tool == MapTool::DigitizeLine
                                && app.draft_vertices.len() >= 2
                            {
                                let coords = std::mem::take(&mut app.draft_vertices);
                                app.open_attribute_form(
                                    Geometry::LineString { coordinates: coords },
                                    gnss_tel,
                                );
                            } else if app.active_tool == MapTool::DigitizePolygon
                                && app.draft_vertices.len() >= 3
                            {
                                let mut ring = std::mem::take(&mut app.draft_vertices);
                                ring.push(ring[0]);
                                app.open_attribute_form(
                                    Geometry::Polygon {
                                        coordinates: vec![ring],
                                    },
                                    gnss_tel,
                                );
                            }
                        }
                    }
                });
        });
}

fn compute_reference_extent(app: &SdncFieldApp, gnss_tel: &GnssLiveTelemetry) -> (f64, f64, f64) {
    let active_crs = app.project.active_crs;
    let mut min_x = f64::MAX;
    let mut max_x = f64::MIN;
    let mut min_y = f64::MAX;
    let mut max_y = f64::MIN;

    for (x, y, _, _) in app.collect_elevation_samples() {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }

    if min_x < max_x && min_y < max_y {
        let cx = (min_x + max_x) * 0.5;
        let cy = (min_y + max_y) * 0.5;
        let span = (max_x - min_x).max(max_y - min_y).max(200.0) * 1.4;
        return (cx, cy, span);
    }

    if gnss_tel.solution.quality != FixQuality::NoFix {
        if let Ok((gx, gy)) = from_wgs84(
            active_crs,
            gnss_tel.solution.lon_deg,
            gnss_tel.solution.lat_deg,
        ) {
            return (gx, gy, 500.0);
        }
    }

    if let Some((cx, cy)) = app.project.center_xy {
        return (cx, cy, 800.0);
    }
    (700_000.0, 6_600_000.0, 2000.0)
}

#[allow(clippy::too_many_arguments)]
fn render_basemap_tiles(
    ui: &egui::Ui,
    painter: &egui::Painter,
    rect: Rect,
    crs: FrenchCrs,
    scale: f64,
    screen_to_world: &impl Fn(Pos2) -> (f64, f64),
    world_to_screen: &impl Fn(f64, f64) -> Pos2,
    app: &mut SdncFieldApp,
) {
    if app.project.basemap.active_layers.is_empty() {
        return;
    }

    let (w_min_x, w_max_y) = screen_to_world(rect.min);
    let (w_max_x, w_min_y) = screen_to_world(rect.max);
    let meters_per_px = 1.0 / scale.max(1e-6);
    let zoom_z = ((156_543.0 / meters_per_px).log2().round() as i32).clamp(1, 19) as u8;

    let (lon0, lat0) = to_wgs84_robust(crs, w_min_x.min(w_max_x), w_min_y.min(w_max_y));
    let (lon1, lat1) = to_wgs84_robust(crs, w_min_x.max(w_max_x), w_min_y.max(w_max_y));
    if (lon0 == 0.0 && lat0 == 0.0) && (lon1 == 0.0 && lat1 == 0.0) {
        return;
    }

    let (tx0, ty1) = TileStreamClient::lon_lat_to_tile(lon0, lat0, zoom_z);
    let (tx1, ty0) = TileStreamClient::lon_lat_to_tile(lon1, lat1, zoom_z);
    let t_min_x = tx0.min(tx1).saturating_sub(1);
    let t_max_x = (tx0.max(tx1) + 1).min(t_min_x + 16);
    let t_min_y = ty0.min(ty1).saturating_sub(1);
    let t_max_y = (ty0.max(ty1) + 1).min(t_min_y + 16);

    let active_basemaps = app.project.basemap.active_layers.clone();
    let opacity = app.project.basemap.opacity;

    for layer in active_basemaps {
        for tx in t_min_x..=t_max_x {
            for ty in t_min_y..=t_max_y {
                let key = WmtsTileKey {
                    layer,
                    z: zoom_z,
                    x: tx,
                    y: ty,
                };
                let raw = TileStreamClient::tile_to_lon_lat_bounds(tx, ty, zoom_z);
                let pad_lon = (raw.2 - raw.0) * 0.002;
                let pad_lat = (raw.3 - raw.1) * 0.002;
                let bounds = (
                    raw.0 - pad_lon,
                    raw.1 - pad_lat,
                    raw.2 + pad_lon,
                    raw.3 + pad_lat,
                );

                let (tex_opt, uv_rect) = if let Some(h) = app.vp2d.texture_cache.get(&key) {
                    (Some(h.id()), Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)))
                } else if let Some((tw, th, rgba)) = app.tile_client.get_cached_tile_rgba(key) {
                    let img = egui::ColorImage::from_rgba_unmultiplied(
                        [tw as usize, th as usize],
                        &rgba,
                    );
                    let handle = ui.ctx().load_texture(
                        format!("tile_{:?}_{}_{}_{}", layer, zoom_z, tx, ty),
                        img,
                        egui::TextureOptions::LINEAR,
                    );
                    let tid = handle.id();
                    app.vp2d.texture_cache.insert(key, handle);
                    (Some(tid), Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)))
                } else {
                    let parent = WmtsTileKey {
                        layer,
                        z: zoom_z.saturating_sub(1),
                        x: tx / 2,
                        y: ty / 2,
                    };
                    let u0 = (tx % 2) as f32 * 0.5;
                    let v0 = (ty % 2) as f32 * 0.5;
                    let p_uv = Rect::from_min_max(Pos2::new(u0, v0), Pos2::new(u0 + 0.5, v0 + 0.5));
                    (app.vp2d.texture_cache.get(&parent).map(|t| t.id()), p_uv)
                };

                if let Some(tid) = tex_opt {
                    render_warped_tile(painter, tid, crs, bounds, uv_rect, world_to_screen, opacity);
                }

                // Déclenche le téléchargement asynchrone si la tuile exacte n'est pas en RAM
                if !app.vp2d.texture_cache.contains_key(&key) {
                    let mut pending = app
                        .vp2d
                        .pending_requests
                        .lock()
                        .unwrap_or_else(|e| e.into_inner());
                    if !pending.contains_key(&key)
                        && CONCURRENT_TILE_REQUESTS.load(Ordering::SeqCst) < 8
                    {
                        pending.insert(key, true);
                        let p_map = Arc::clone(&app.vp2d.pending_requests);
                        let client = app.tile_client.clone();
                        let ctx = ui.ctx().clone();
                        CONCURRENT_TILE_REQUESTS.fetch_add(1, Ordering::SeqCst);
                        std::thread::spawn(move || {
                            let _ = client.fetch_tile(key);
                            p_map.lock().unwrap_or_else(|e| e.into_inner()).remove(&key);
                            CONCURRENT_TILE_REQUESTS.fetch_sub(1, Ordering::SeqCst);
                            ctx.request_repaint();
                        });
                    }
                }
            }
        }
    }
}

/// Maillage géodésique subdivisé 4x4 (issu d'`AEROPHOTOGRAM`) pour reprojeter sans distorsion
/// les tuiles Web Mercator (EPSG:3857) vers Lambert-93 / CC42–CC50 / UTM.
pub fn render_warped_tile(
    painter: &egui::Painter,
    texture_id: egui::TextureId,
    crs: FrenchCrs,
    bounds: (f64, f64, f64, f64),
    uv_rect: Rect,
    world_to_screen: &impl Fn(f64, f64) -> Pos2,
    opacity: f32,
) {
    const SUBDIV: usize = 4;
    let (min_lon, min_lat, max_lon, max_lat) = bounds;
    let tile_color =
        Color32::from_rgba_unmultiplied(255, 255, 255, (opacity.clamp(0.0, 1.0) * 255.0) as u8);

    let mut mesh = egui::Mesh::with_texture(texture_id);
    let mut indices_grid = Vec::with_capacity((SUBDIV + 1) * (SUBDIV + 1));

    for j in 0..=SUBDIV {
        let sv = j as f32 / SUBDIV as f32;
        let lat = (max_lat - sv as f64 * (max_lat - min_lat)).clamp(-82.0, 84.0);
        let v = uv_rect.min.y + sv * uv_rect.height();
        for i in 0..=SUBDIV {
            let su = i as f32 / SUBDIV as f32;
            let lon = min_lon + su as f64 * (max_lon - min_lon);
            let u = uv_rect.min.x + su * uv_rect.width();
            if let Ok((wx, wy)) = from_wgs84(crs, lon, lat) {
                let idx = mesh.vertices.len() as u32;
                mesh.vertices.push(egui::epaint::Vertex {
                    pos: world_to_screen(wx, wy),
                    uv: Pos2::new(u, v),
                    color: tile_color,
                });
                indices_grid.push(Some(idx));
            } else {
                indices_grid.push(None);
            }
        }
    }

    let stride = SUBDIV + 1;
    for j in 0..SUBDIV {
        for i in 0..SUBDIV {
            if let (Some(tl), Some(tr), Some(bl), Some(br)) = (
                indices_grid[j * stride + i],
                indices_grid[j * stride + i + 1],
                indices_grid[(j + 1) * stride + i],
                indices_grid[(j + 1) * stride + i + 1],
            ) {
                mesh.indices.extend_from_slice(&[tl, tr, bl, tr, br, bl]);
            }
        }
    }

    if !mesh.indices.is_empty() {
        painter.add(mesh);
    }
}

fn render_metric_grid(
    painter: &egui::Painter,
    rect: Rect,
    screen_to_world: &impl Fn(Pos2) -> (f64, f64),
    world_to_screen: &impl Fn(f64, f64) -> Pos2,
    scale: f64,
) {
    let raw_step = 110.0 / scale.max(1e-6);
    let pow10 = 10.0f64.powf(raw_step.log10().floor());
    let step = if raw_step / pow10 < 2.0 {
        pow10
    } else if raw_step / pow10 < 5.0 {
        pow10 * 2.0
    } else {
        pow10 * 5.0
    };

    let (x0, y1) = screen_to_world(rect.min);
    let (x1, y0) = screen_to_world(rect.max);
    let stroke = Stroke::new(0.8_f32, Color32::from_rgba_unmultiplied(120, 170, 220, 42));

    let mut gx = (x0.min(x1) / step).floor() * step;
    let x_end = x0.max(x1);
    while gx <= x_end {
        let sx = world_to_screen(gx, y0).x;
        painter.line_segment([Pos2::new(sx, rect.min.y), Pos2::new(sx, rect.max.y)], stroke);
        gx += step;
    }

    let mut gy = (y0.min(y1) / step).floor() * step;
    let y_end = y0.max(y1);
    while gy <= y_end {
        let sy = world_to_screen(x0, gy).y;
        painter.line_segment([Pos2::new(rect.min.x, sy), Pos2::new(rect.max.x, sy)], stroke);
        gy += step;
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_geometry_2d(
    painter: &egui::Painter,
    geom: &Geometry,
    src_crs: FrenchCrs,
    dst_crs: FrenchCrs,
    world_to_screen: &impl Fn(f64, f64) -> Pos2,
    stroke: Stroke,
    fill: Color32,
    point_radius: f32,
    label: Option<&str>,
    snap_out: &mut Vec<(Pos2, Coordinate)>,
) {
    let proj = |c: &Coordinate| -> (Pos2, Coordinate) {
        let (x, y) = if src_crs != dst_crs {
            reproject_point_3d(src_crs, dst_crs, c.x, c.y, c.z.unwrap_or(0.0))
                .map(|(rx, ry, _)| (rx, ry))
                .unwrap_or((c.x, c.y))
        } else {
            (c.x, c.y)
        };
        (world_to_screen(x, y), Coordinate { x, y, z: c.z })
    };

    match geom {
        Geometry::Point { coordinates } => {
            let (sp, c2) = proj(coordinates);
            snap_out.push((sp, c2));
            painter.circle_filled(sp, point_radius, fill);
            painter.circle_stroke(sp, point_radius, stroke);
            if let Some(txt) = label {
                painter.text(
                    sp + Vec2::new(point_radius + 4.0, -6.0),
                    egui::Align2::LEFT_CENTER,
                    txt,
                    egui::FontId::proportional(11.0),
                    Color32::WHITE,
                );
            }
        }
        Geometry::LineString { coordinates } => {
            let pts: Vec<Pos2> = coordinates
                .iter()
                .map(|c| {
                    let (sp, c2) = proj(c);
                    snap_out.push((sp, c2));
                    sp
                })
                .collect();
            if pts.len() >= 2 {
                painter.add(egui::Shape::line(pts, stroke));
            }
        }
        Geometry::Polygon { coordinates } => {
            if let Some(ext) = coordinates.first() {
                let pts: Vec<Pos2> = ext
                    .iter()
                    .map(|c| {
                        let (sp, c2) = proj(c);
                        snap_out.push((sp, c2));
                        sp
                    })
                    .collect();
                if pts.len() >= 3 {
                    painter.add(egui::Shape::convex_polygon(pts.clone(), fill, stroke));
                    if let Some(txt) = label {
                        let cx = pts.iter().map(|p| p.x).sum::<f32>() / pts.len() as f32;
                        let cy = pts.iter().map(|p| p.y).sum::<f32>() / pts.len() as f32;
                        painter.text(
                            Pos2::new(cx, cy),
                            egui::Align2::CENTER_CENTER,
                            txt,
                            egui::FontId::proportional(11.5),
                            Color32::WHITE,
                        );
                    }
                }
            }
        }
        Geometry::MultiPoint { coordinates } => {
            for c in coordinates {
                draw_geometry_2d(
                    painter,
                    &Geometry::Point { coordinates: *c },
                    src_crs,
                    dst_crs,
                    world_to_screen,
                    stroke,
                    fill,
                    point_radius,
                    None,
                    snap_out,
                );
            }
        }
        Geometry::MultiLineString { coordinates } => {
            for l in coordinates {
                draw_geometry_2d(
                    painter,
                    &Geometry::LineString {
                        coordinates: l.clone(),
                    },
                    src_crs,
                    dst_crs,
                    world_to_screen,
                    stroke,
                    fill,
                    point_radius,
                    None,
                    snap_out,
                );
            }
        }
        Geometry::MultiPolygon { coordinates } => {
            for p in coordinates {
                draw_geometry_2d(
                    painter,
                    &Geometry::Polygon {
                        coordinates: p.clone(),
                    },
                    src_crs,
                    dst_crs,
                    world_to_screen,
                    stroke,
                    fill,
                    point_radius,
                    None,
                    snap_out,
                );
            }
        }
        Geometry::GeometryCollection { geometries } => {
            for g in geometries {
                draw_geometry_2d(
                    painter,
                    g,
                    src_crs,
                    dst_crs,
                    world_to_screen,
                    stroke,
                    fill,
                    point_radius,
                    None,
                    snap_out,
                );
            }
        }
    }
}

fn render_gnss_overlay_2d(
    painter: &egui::Painter,
    crs: FrenchCrs,
    scale: f64,
    world_to_screen: &impl Fn(f64, f64) -> Pos2,
    tel: &GnssLiveTelemetry,
) {
    // 1. Trace GNSS colorée selon la qualité Fix/Float/Single
    if tel.track_history_wgs84.len() >= 2 {
        for win in tel.track_history_wgs84.windows(2) {
            if let (Ok((x0, y0)), Ok((x1, y1))) = (
                from_wgs84(crs, win[0].0, win[0].1),
                from_wgs84(crs, win[1].0, win[1].1),
            ) {
                let rgb = win[1].3.color_rgb();
                painter.line_segment(
                    [world_to_screen(x0, y0), world_to_screen(x1, y1)],
                    Stroke::new(2.5_f32, Color32::from_rgb(rgb[0], rgb[1], rgb[2])),
                );
            }
        }
    }

    // 2. Station de Base RTK (ligne de base en pointillés)
    let mut base_screen = None;
    if let Some(b_ecef) = tel.base_ecef {
        let (blat, blon, _) =
            sdnc_geo::ecef_to_geodetic_h(b_ecef[0], b_ecef[1], b_ecef[2], &sdnc_geo::WGS84);
        if let Ok((bx, by)) = from_wgs84(crs, blon.to_degrees(), blat.to_degrees()) {
            let bs = world_to_screen(bx, by);
            base_screen = Some(bs);
            painter.rect_filled(
                Rect::from_center_size(bs, Vec2::splat(10.0)),
                2.0,
                Color32::from_rgb(255, 110, 40),
            );
            painter.text(
                bs + Vec2::new(8.0, -8.0),
                egui::Align2::LEFT_CENTER,
                format!("BASE #{}", tel.base_station_id.unwrap_or(0)),
                egui::FontId::proportional(10.5),
                Color32::from_rgb(255, 170, 90),
            );
        }
    }

    // 3. Position Rover courante + cercle d'incertitude horizontal σH
    if tel.solution.quality != FixQuality::NoFix {
        if let Ok((rx, ry)) = from_wgs84(crs, tel.solution.lon_deg, tel.solution.lat_deg) {
            let rs = world_to_screen(rx, ry);
            if let Some(bs) = base_screen {
                painter.line_segment(
                    [bs, rs],
                    Stroke::new(1.2_f32, Color32::from_rgba_unmultiplied(255, 160, 60, 120)),
                );
            }
            let rgb = tel.solution.quality.color_rgb();
            let col = Color32::from_rgb(rgb[0], rgb[1], rgb[2]);
            let sigma_h = (tel.solution.std_enu_m[0].powi(2) + tel.solution.std_enu_m[1].powi(2))
                .sqrt() as f64;
            let err_radius_px = (sigma_h * scale).clamp(8.0, 90.0) as f32;
            painter.circle_filled(
                rs,
                err_radius_px,
                Color32::from_rgba_unmultiplied(rgb[0], rgb[1], rgb[2], 45),
            );
            painter.circle_stroke(rs, err_radius_px, Stroke::new(1.5_f32, col));
            painter.circle_filled(rs, 6.0, col);
            painter.circle_stroke(rs, 6.0, Stroke::new(2.0_f32, Color32::WHITE));
            painter.text(
                rs + Vec2::new(10.0, -10.0),
                egui::Align2::LEFT_CENTER,
                format!("{} (±{:.1} cm)", tel.solution.quality.label(), sigma_h * 100.0),
                egui::FontId::proportional(11.0),
                col,
            );
        }
    }
}

fn render_draft_and_measures(
    painter: &egui::Painter,
    app: &SdncFieldApp,
    world_to_screen: &impl Fn(f64, f64) -> Pos2,
) {
    // Brouillon de numérisation
    if !app.draft_vertices.is_empty() {
        let pts: Vec<Pos2> = app
            .draft_vertices
            .iter()
            .map(|c| world_to_screen(c.x, c.y))
            .collect();
        let stroke = Stroke::new(2.5_f32, Color32::from_rgb(255, 80, 200));
        if pts.len() >= 2 {
            painter.add(egui::Shape::line(pts.clone(), stroke));
        }
        for &p in &pts {
            painter.circle_filled(p, 4.5, Color32::WHITE);
            painter.circle_stroke(p, 4.5, stroke);
        }
    }

    // Mesures Distance / Surface
    if app.vp2d.measure_points.len() >= 2 {
        let pts: Vec<Pos2> = app
            .vp2d
            .measure_points
            .iter()
            .map(|&(x, y)| world_to_screen(x, y))
            .collect();
        let stroke = Stroke::new(2.2_f32, Color32::YELLOW);
        painter.add(egui::Shape::line(pts.clone(), stroke));

        let mut total_dist = 0.0;
        for w in app.vp2d.measure_points.windows(2) {
            total_dist += ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt();
        }
        let mut msg = format!("Dist: {:.2} m", total_dist);
        if app.active_tool == MapTool::MeasureArea && app.vp2d.measure_points.len() >= 3 {
            let mut area2 = 0.0;
            let n = app.vp2d.measure_points.len();
            for i in 0..n {
                let j = (i + 1) % n;
                area2 += app.vp2d.measure_points[i].0 * app.vp2d.measure_points[j].1
                    - app.vp2d.measure_points[j].0 * app.vp2d.measure_points[i].1;
            }
            msg = format!("{} | Surf: {:.2} m²", msg, area2.abs() * 0.5);
        }
        if let Some(&last) = pts.last() {
            painter.text(
                last + Vec2::new(8.0, -10.0),
                egui::Align2::LEFT_CENTER,
                msg,
                egui::FontId::proportional(12.0),
                Color32::YELLOW,
            );
        }
    }
}

fn identify_nearest_feature(
    app: &SdncFieldApp,
    dst_crs: FrenchCrs,
    click: Pos2,
    world_to_screen: &impl Fn(f64, f64) -> Pos2,
) -> Option<String> {
    let mut best: Option<(f32, String)> = None;
    for layer in &app.layers {
        if !layer.spec.visible {
            continue;
        }
        for feat in &layer.data.features {
            if let Some(g) = &feat.geometry {
                if let Some((x0, y0, x1, y1)) = g.bbox() {
                    let cx = (x0 + x1) * 0.5;
                    let cy = (y0 + y1) * 0.5;
                    let (px, py) = if layer.spec.crs != dst_crs {
                        reproject_point_3d(layer.spec.crs, dst_crs, cx, cy, 0.0)
                            .map(|(a, b, _)| (a, b))
                            .unwrap_or((cx, cy))
                    } else {
                        (cx, cy)
                    };
                    let d = world_to_screen(px, py).distance(click);
                    if d < 28.0 && best.as_ref().map_or(true, |(bd, _)| d < *bd) {
                        let props: Vec<String> = feat
                            .properties
                            .iter()
                            .map(|(k, v)| format!("{} = {:?}", k, v))
                            .collect();
                        best = Some((
                            d,
                            format!("[{}] {}", layer.spec.name, props.join(" | ")),
                        ));
                    }
                }
            }
        }
    }
    best.map(|(_, s)| s)
}

fn render_hud_2d(
    painter: &egui::Painter,
    rect: Rect,
    crs: FrenchCrs,
    scale: f64,
    app: &SdncFieldApp,
    gnss_tel: &GnssLiveTelemetry,
) {
    // Flèche Nord géographique avec convergence des méridiens
    let north_center = Pos2::new(rect.min.x + 34.0, rect.min.y + 38.0);
    let conv_rad = app
        .vp2d
        .hovered_world
        .map(|(wx, wy)| {
            let (lon, lat) = to_wgs84_robust(crs, wx, wy);
            meridian_convergence(crs, lon, lat).to_radians() as f32
        })
        .unwrap_or(0.0);
    painter.circle_filled(
        north_center,
        20.0,
        Color32::from_rgba_unmultiplied(12, 18, 30, 200),
    );
    painter.circle_stroke(north_center, 20.0, Stroke::new(1.2_f32, Color32::LIGHT_BLUE));
    let tip = north_center + Vec2::new(conv_rad.sin() * 14.0, -conv_rad.cos() * 14.0);
    painter.line_segment([north_center, tip], Stroke::new(2.5_f32, Color32::from_rgb(255, 80, 80)));
    painter.text(
        north_center + Vec2::new(0.0, 9.0),
        egui::Align2::CENTER_CENTER,
        "N",
        egui::FontId::proportional(11.0),
        Color32::WHITE,
    );

    // Barre d'échelle dynamique (bas-gauche)
    let raw_m = 100.0 / scale.max(1e-6);
    let pow10 = 10.0f64.powf(raw_m.log10().floor());
    let bar_m = if raw_m / pow10 < 2.0 {
        pow10
    } else if raw_m / pow10 < 5.0 {
        pow10 * 2.0
    } else {
        pow10 * 5.0
    };
    let bar_px = (bar_m * scale) as f32;
    let bx = rect.min.x + 16.0;
    let by = rect.max.y - 22.0;
    painter.line_segment(
        [Pos2::new(bx, by), Pos2::new(bx + bar_px, by)],
        Stroke::new(3.0_f32, Color32::WHITE),
    );
    let bar_label = if bar_m >= 1000.0 {
        format!("{:.0} km", bar_m / 1000.0)
    } else {
        format!("{:.0} m", bar_m)
    };
    painter.text(
        Pos2::new(bx + bar_px * 0.5, by - 10.0),
        egui::Align2::CENTER_CENTER,
        bar_label,
        egui::FontId::proportional(11.0),
        Color32::WHITE,
    );

    // Bandeau coordonnées + GNSS (bas-droite)
    let hud_text = if let Some((wx, wy)) = app.vp2d.hovered_world {
        let (lon, lat) = to_wgs84_robust(crs, wx, wy);
        format!(
            "EPSG:{} | X: {:.2} m  Y: {:.2} m | WGS84: {:.6}°, {:.6}° | GNSS: {}",
            crs.epsg(),
            wx,
            wy,
            lat,
            lon,
            gnss_tel.solution.quality.label()
        )
    } else {
        format!(
            "EPSG:{} | GNSS: {}",
            crs.epsg(),
            gnss_tel.solution.quality.label()
        )
    };
    let hud_pos = Pos2::new(rect.max.x - 14.0, rect.max.y - 16.0);
    painter.text(
        hud_pos,
        egui::Align2::RIGHT_CENTER,
        hud_text,
        egui::FontId::monospace(11.0),
        Color32::from_rgb(200, 235, 255),
    );
}
