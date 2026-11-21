//! Module Cartographique 3D adapté d'`AEROPHOTOGRAM` (`viewport_3d.rs`) + Vulkan Compute (`sdnc-gpu`).
//!
//! Amélioration clé par rapport à `AEROPHOTOGRAM` :
//! - L'interpolation MNT IDW et l'ombrage (Hillshade / Pentes) sont exécutés via le shader
//!   Vulkan Compute (`terrain_idw.wgsl`) et mis en cache tant que les données ne changent pas.
//! - Affiche le terrain drapé (ombrage GPU + hypsométrie), les vecteurs 3D (GeoPackage/DXF/GeoJSON)
//!   et la trajectoire GNSS RTK/PPK en 3D avec balise rover et station de base.

use crate::state::SdncFieldApp;
use egui::{Color32, Pos2, Stroke, Vec2};
use sdnc_data::{Coordinate, Geometry};
use sdnc_geo::{from_wgs84, reproject_point_3d};
use sdnc_gnss::{FixQuality, GnssLiveTelemetry};
use sdnc_gpu::TerrainGridResult;

pub struct Viewport3dState {
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    pub distance_factor: f32,
    pub pan_offset: Vec2,
    pub target_pitch_deg: f32,
    pub target_yaw_deg: f32,
    pub target_distance_factor: f32,
    pub target_pan_offset: Vec2,
    pub z_exaggeration: f32,
    pub cached_terrain: Option<TerrainGridResult>,
    pub terrain_dirty: bool,
}

impl Default for Viewport3dState {
    fn default() -> Self {
        Self {
            pitch_deg: 42.0,
            yaw_deg: 32.0,
            distance_factor: 1.0,
            pan_offset: Vec2::ZERO,
            target_pitch_deg: 42.0,
            target_yaw_deg: 32.0,
            target_distance_factor: 1.0,
            target_pan_offset: Vec2::ZERO,
            z_exaggeration: 2.0,
            cached_terrain: None,
            terrain_dirty: true,
        }
    }
}

impl Viewport3dState {
    pub fn invalidate_terrain(&mut self) {
        self.terrain_dirty = true;
    }
}

pub fn render_viewport_3d(
    ui: &mut egui::Ui,
    app: &mut SdncFieldApp,
    gnss_tel: &GnssLiveTelemetry,
) {
    let dt = ui.input(|i| i.stable_dt).min(0.1);
    let lerp = (dt * 14.0).clamp(0.06, 1.0);
    app.vp3d.distance_factor += (app.vp3d.target_distance_factor - app.vp3d.distance_factor) * lerp;
    app.vp3d.yaw_deg += (app.vp3d.target_yaw_deg - app.vp3d.yaw_deg) * lerp;
    app.vp3d.pitch_deg += (app.vp3d.target_pitch_deg - app.vp3d.pitch_deg) * lerp;
    app.vp3d.pan_offset += (app.vp3d.target_pan_offset - app.vp3d.pan_offset) * lerp;

    if (app.vp3d.target_distance_factor - app.vp3d.distance_factor).abs() > 1e-4
        || (app.vp3d.target_yaw_deg - app.vp3d.yaw_deg).abs() > 1e-3
        || (app.vp3d.target_pitch_deg - app.vp3d.pitch_deg).abs() > 1e-3
    {
        ui.ctx().request_repaint();
    }

    let available = ui.available_size();
    let (response, painter) = ui.allocate_painter(available, egui::Sense::click_and_drag());
    let rect = response.rect;
    painter.rect_filled(rect, 0.0, Color32::from_rgb(10, 14, 24));

    // Interactions Orbite / Pan / Pinch-to-Zoom
    if let Some(mt) = ui.input(|i| i.multi_touch()) {
        if (mt.zoom_delta - 1.0).abs() > 1e-4 {
            app.vp3d.target_distance_factor =
                (app.vp3d.target_distance_factor / mt.zoom_delta).clamp(0.05, 25.0);
        }
        app.vp3d.target_pan_offset += mt.translation_delta;
    } else if response.dragged() {
        let d = response.drag_delta();
        let is_pan = ui.input(|i| {
            i.modifiers.shift
                || i.pointer.middle_down()
                || i.pointer.button_down(egui::PointerButton::Secondary)
        });
        if is_pan {
            app.vp3d.target_pan_offset += d;
        } else {
            app.vp3d.target_yaw_deg += d.x * 0.45;
            app.vp3d.target_pitch_deg = (app.vp3d.target_pitch_deg + d.y * 0.45).clamp(8.0, 85.0);
        }
    }
    if response.hovered() {
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll.abs() > 0.01 {
            let factor = 1.15f32.powf(-scroll * 0.04);
            app.vp3d.target_distance_factor =
                (app.vp3d.target_distance_factor * factor).clamp(0.05, 25.0);
        }
    }

    // 1. Collecte des échantillons altimétriques + emprise 3D
    let active_crs = app.project.active_crs;
    let mut samples = app.collect_elevation_samples();
    for &(lon, lat, alt, _) in &gnss_tel.track_history_wgs84 {
        if let Ok((gx, gy)) = from_wgs84(active_crs, lon, lat) {
            samples.push((gx, gy, alt - 48.0, 3.0));
        }
    }

    let (mut min_x, mut max_x, mut min_y, mut max_y, mut min_z, mut max_z) =
        (f64::MAX, f64::MIN, f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for &(x, y, z, _) in &samples {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
        min_z = min_z.min(z);
        max_z = max_z.max(z);
    }
    if min_x >= max_x || min_y >= max_y {
        let (cx, cy) = app.project.center_xy.unwrap_or((894500.0, 6269500.0));
        min_x = cx - 300.0;
        max_x = cx + 300.0;
        min_y = cy - 300.0;
        max_y = cy + 300.0;
        min_z = 220.0;
        max_z = 270.0;
    }

    let center_x = (min_x + max_x) * 0.5;
    let center_y = (min_y + max_y) * 0.5;
    let avg_z = (min_z + max_z) * 0.5;
    let extent_x = (max_x - min_x).max(120.0);
    let extent_y = (max_y - min_y).max(120.0);
    let max_extent = extent_x.max(extent_y).max((max_z - min_z).max(20.0));

    // 2. Calcul (ou réutilisation du cache) du MNT + Hillshade via Vulkan Compute (`sdnc-gpu`)
    if app.vp3d.terrain_dirty || app.vp3d.cached_terrain.is_none() {
        let grid_n = 36usize;
        let ox = center_x - extent_x * 0.75;
        let oy = center_y - extent_y * 0.75;
        let dx = (extent_x * 1.5) / (grid_n - 1) as f64;
        let dy = (extent_y * 1.5) / (grid_n - 1) as f64;
        let grid = app.terrain_engine.compute_terrain_grid(
            ox,
            oy,
            dx,
            dy,
            grid_n,
            grid_n,
            avg_z,
            app.vp3d.z_exaggeration,
            &samples,
        );
        app.vp3d.cached_terrain = Some(grid);
        app.vp3d.terrain_dirty = false;
    }

    // 3. Projection orbitale 3D -> Écran
    let pitch = (app.vp3d.pitch_deg as f64).to_radians();
    let yaw = (app.vp3d.yaw_deg as f64).to_radians();
    let (cos_p, sin_p) = (pitch.cos(), pitch.sin());
    let (cos_y, sin_y) = (yaw.cos(), yaw.sin());
    let screen_center = rect.center() + app.vp3d.pan_offset;
    let view_scale =
        (rect.height() as f64 * 0.65 / max_extent) / (app.vp3d.distance_factor as f64);
    let z_exag = app.vp3d.z_exaggeration as f64;

    let project_3d = |wx: f64, wy: f64, wz: f64| -> Pos2 {
        let dx = wx - center_x;
        let dy = wy - center_y;
        let dz = (wz - avg_z) * z_exag;
        let rx = dx * cos_y - dy * sin_y;
        let ry = dx * sin_y + dy * cos_y;
        let rz = ry * sin_p + dz * cos_p;
        let ry_proj = ry * cos_p - dz * sin_p;
        let f = 1000.0;
        let pf = f / (f + ry_proj + max_extent * 2.0).max(50.0);
        Pos2::new(
            (screen_center.x as f64 + rx * view_scale * pf) as f32,
            (screen_center.y as f64 - rz * view_scale * pf) as f32,
        )
    };

    // 4. Rendu de la surface MNT calculée par Vulkan Compute (colorisation hypsométrique * hillshade)
    let mut computed_on_gpu = false;
    if let Some(grid) = &app.vp3d.cached_terrain {
        computed_on_gpu = grid.computed_on_gpu;
        let z_span = (max_z - min_z).max(5.0);
        let mut mesh = egui::Mesh::default();

        for gy in 0..(grid.rows - 1) {
            for gx in 0..(grid.cols - 1) {
                let corners = [(gx, gy), (gx + 1, gy), (gx + 1, gy + 1), (gx, gy + 1)];
                let base_idx = mesh.vertices.len() as u32;
                for (cx_i, cy_i) in corners {
                    let cell = grid.cells[cy_i * grid.cols + cx_i];
                    let wx = grid.origin_x + cx_i as f64 * grid.dx;
                    let wy = grid.origin_y + cy_i as f64 * grid.dy;
                    let wz = cell.elevation as f64;
                    let sp = project_3d(wx, wy, wz);
                    let norm_z = ((wz - min_z) / z_span).clamp(0.0, 1.0) as f32;
                    let shade = cell.hillshade.clamp(0.15, 1.0);
                    let (r, g, b) = hypsometric_rgb(norm_z);
                    let col = Color32::from_rgb(
                        (r as f32 * shade) as u8,
                        (g as f32 * shade) as u8,
                        (b as f32 * shade) as u8,
                    );
                    mesh.vertices.push(egui::epaint::Vertex {
                        pos: sp,
                        uv: Pos2::ZERO,
                        color: col,
                    });
                }
                mesh.indices.extend_from_slice(&[
                    base_idx,
                    base_idx + 1,
                    base_idx + 2,
                    base_idx,
                    base_idx + 2,
                    base_idx + 3,
                ]);
            }
        }
        painter.add(mesh);

        // Maillage filaire discret sur un pas sur 3
        let wire_stroke = Stroke::new(0.6_f32, Color32::from_rgba_unmultiplied(200, 230, 255, 35));
        for gy in (0..grid.rows).step_by(3) {
            let mut row_pts = Vec::with_capacity(grid.cols);
            for gx in 0..grid.cols {
                let c = grid.cells[gy * grid.cols + gx];
                row_pts.push(project_3d(
                    grid.origin_x + gx as f64 * grid.dx,
                    grid.origin_y + gy as f64 * grid.dy,
                    c.elevation as f64,
                ));
            }
            painter.add(egui::Shape::line(row_pts, wire_stroke));
        }
    }

    // 5. Rendu 3D des couches vectorielles drapées / Z natif
    for layer in &app.layers {
        if !layer.spec.visible {
            continue;
        }
        let s = &layer.spec.style;
        let stroke = Stroke::new(
            s.stroke_width,
            Color32::from_rgb(s.stroke_rgba[0], s.stroke_rgba[1], s.stroke_rgba[2]),
        );
        for feat in &layer.data.features {
            let Some(g) = &feat.geometry else {
                continue;
            };
            let proj_coord = |c: &Coordinate| -> Pos2 {
                let (wx, wy, wz_raw) = if layer.spec.crs != active_crs {
                    reproject_point_3d(layer.spec.crs, active_crs, c.x, c.y, c.z.unwrap_or(avg_z))
                        .unwrap_or((c.x, c.y, c.z.unwrap_or(avg_z)))
                } else {
                    (c.x, c.y, c.z.unwrap_or(avg_z))
                };
                let wz = if c.z.is_some() {
                    wz_raw
                } else if let Some(grid) = &app.vp3d.cached_terrain {
                    grid.sample_bilinear(wx, wy).0
                } else {
                    avg_z
                };
                project_3d(wx, wy, wz)
            };

            match g {
                Geometry::Point { coordinates } => {
                    let sp = proj_coord(coordinates);
                    let ground_z = app
                        .vp3d
                        .cached_terrain
                        .as_ref()
                        .map(|t| t.sample_bilinear(coordinates.x, coordinates.y).0)
                        .unwrap_or(avg_z);
                    let gp = project_3d(coordinates.x, coordinates.y, ground_z);
                    painter.line_segment([gp, sp], Stroke::new(1.5_f32, Color32::WHITE));
                    painter.circle_filled(sp, 5.0, stroke.color);
                }
                Geometry::LineString { coordinates } => {
                    let pts: Vec<Pos2> = coordinates.iter().map(proj_coord).collect();
                    if pts.len() >= 2 {
                        painter.add(egui::Shape::line(pts, stroke));
                    }
                }
                Geometry::Polygon { coordinates } => {
                    if let Some(ext) = coordinates.first() {
                        let pts: Vec<Pos2> = ext.iter().map(proj_coord).collect();
                        if pts.len() >= 2 {
                            painter.add(egui::Shape::line(pts, stroke));
                        }
                    }
                }
                _ => {}
            }
        }
    }

    // 6. Trajectoire et Balise GNSS RTK/PPK en 3D
    if gnss_tel.track_history_wgs84.len() >= 2 {
        for win in gnss_tel.track_history_wgs84.windows(2) {
            if let (Ok((x0, y0)), Ok((x1, y1))) = (
                from_wgs84(active_crs, win[0].0, win[0].1),
                from_wgs84(active_crs, win[1].0, win[1].1),
            ) {
                let rgb = win[1].3.color_rgb();
                painter.line_segment(
                    [
                        project_3d(x0, y0, win[0].2 - 48.0),
                        project_3d(x1, y1, win[1].2 - 48.0),
                    ],
                    Stroke::new(2.5_f32, Color32::from_rgb(rgb[0], rgb[1], rgb[2])),
                );
            }
        }
    }
    if gnss_tel.solution.quality != FixQuality::NoFix {
        if let Ok((rx, ry)) = from_wgs84(
            active_crs,
            gnss_tel.solution.lon_deg,
            gnss_tel.solution.lat_deg,
        ) {
            let rz = gnss_tel.solution.alt_orthometric_m;
            let sp = project_3d(rx, ry, rz);
            let rgb = gnss_tel.solution.quality.color_rgb();
            let col = Color32::from_rgb(rgb[0], rgb[1], rgb[2]);
            painter.circle_filled(sp, 6.5, col);
            painter.circle_stroke(sp, 6.5, Stroke::new(2.0_f32, Color32::WHITE));
        }
    }

    // 7. HUD 3D + Trièdre d'orientation XYZ
    let triad_o = Pos2::new(rect.min.x + 42.0, rect.max.y - 42.0);
    let draw_axis = |dx: f64, dy: f64, dz: f64, col: Color32, name: &str| {
        let rx = dx * cos_y - dy * sin_y;
        let ry = dx * sin_y + dy * cos_y;
        let rz = ry * sin_p + dz * cos_p;
        let end = triad_o + Vec2::new((rx * 26.0) as f32, -(rz * 26.0) as f32);
        painter.line_segment([triad_o, end], Stroke::new(2.0_f32, col));
        painter.text(
            end + Vec2::new(4.0, 0.0),
            egui::Align2::LEFT_CENTER,
            name,
            egui::FontId::proportional(10.0),
            col,
        );
    };
    draw_axis(1.0, 0.0, 0.0, Color32::from_rgb(255, 80, 80), "E");
    draw_axis(0.0, 1.0, 0.0, Color32::from_rgb(80, 255, 110), "N");
    draw_axis(0.0, 0.0, 1.0, Color32::from_rgb(80, 170, 255), "Z");

    painter.text(
        Pos2::new(rect.min.x + 12.0, rect.min.y + 14.0),
        egui::Align2::LEFT_TOP,
        format!(
            "Vue 3D MNT [{}] | LiDAR GPU: {} pts | Pitch {:.0}° Yaw {:.0}° | Exag Z x{:.1}",
            if computed_on_gpu {
                "Vulkan Compute"
            } else {
                "CPU Fallback"
            },
            app.lidar_cloud.points.len(),
            app.vp3d.pitch_deg,
            app.vp3d.yaw_deg,
            app.vp3d.z_exaggeration
        ),
        egui::FontId::monospace(11.0),
        Color32::from_rgb(190, 230, 255),
    );
}

fn hypsometric_rgb(t: f32) -> (u8, u8, u8) {
    if t < 0.25 {
        let u = t / 0.25;
        ((40.0 + 30.0 * u) as u8, (110.0 + 80.0 * u) as u8, (70.0 + 20.0 * u) as u8)
    } else if t < 0.55 {
        let u = (t - 0.25) / 0.30;
        ((70.0 + 130.0 * u) as u8, (190.0 - 20.0 * u) as u8, (90.0 - 40.0 * u) as u8)
    } else if t < 0.80 {
        let u = (t - 0.55) / 0.25;
        ((200.0 - 30.0 * u) as u8, (170.0 - 65.0 * u) as u8, (50.0 + 20.0 * u) as u8)
    } else {
        let u = (t - 0.80) / 0.20;
        ((170.0 + 75.0 * u) as u8, (105.0 + 135.0 * u) as u8, (70.0 + 175.0 * u) as u8)
    }
}
