//! État global de l'application terrain SDNCQField (Rust + Vulkan 2D + RTKLIB-Rust).
//!
//! Gère :
//! - Le projet natif léger (`.sdnc.json` + `.gpkg`) et les couches vectorielles éditables
//! - L'import cadastral multi-couches **DXF, DXF-PCI et EDIGÉO** avec détection CRS & symbologie DGFiP
//! - Les flux **IGN Géoplateforme Raster (WMTS)** et **Vecteur (WFS 2.0 GeoJSON)** sauvegardables hors-ligne
//! - Le batch VRAM GPU 2D (`sdnc_gpu::Vector2dGpuBatch`) pour le rendu Vulkan natif à 60–120 FPS
//! - Le pilotage GNSS RTK & PPK + moyennage multi-époques

use crate::viewport_2d::Viewport2dState;
use sdnc_data::{
    generate_demo_cadastral_layers, load_dxf_cadastral_layers, load_edigeo_cadastral_layers,
    Coordinate, Feature, FeatureCollection, GeoPackage, Geometry, GeometryType, LayerSource,
    LayerStyle, ProjectLayerSpec, PropertyValue, SdncProject,
};
use sdnc_geo::{
    fetch_ign_wfs_geojson, from_wgs84, reproject_point_3d, FrenchCrs, IgnWfsLayer, TileStreamClient,
};
use sdnc_gnss::{FixQuality, GnssController, GnssLiveTelemetry, PpkJobConfig};
use sdnc_gpu::{
    pack_rgba8, triangulate_polygon_earclip, FillVertex2d, Segment2dGpu, Vector2dGpuBatch,
    VulkanComputeContext,
};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapTool {
    Navigate,
    Identify,
    MeasureDistance,
    MeasureArea,
    DigitizePoint,
    DigitizeLine,
    DigitizePolygon,
}

impl MapTool {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Navigate => "Naviguer",
            Self::Identify => "Identifier",
            Self::MeasureDistance => "Distance",
            Self::MeasureArea => "Surface",
            Self::DigitizePoint => "+ Point",
            Self::DigitizeLine => "+ Ligne",
            Self::DigitizePolygon => "+ Polygone",
        }
    }
}

#[derive(Debug, Clone)]
pub struct RuntimeLayer {
    pub spec: ProjectLayerSpec,
    pub data: FeatureCollection,
    pub dirty: bool,
}

#[derive(Debug, Clone)]
pub struct PendingAttributeForm {
    pub layer_id: String,
    pub geometry: Geometry,
    pub values: HashMap<String, String>,
    pub gnss_summary: String,
}

/// Session de moyennage multi-époques GNSS RTK haute précision (levé topographique sur canne).
#[derive(Debug, Clone)]
pub struct GnssAveragingSession {
    pub target_epochs: usize,
    pub require_fix_only: bool,
    pub samples: Vec<(f64, f64, f64)>, // (X_proj, Y_proj, Z_ortho_sol)
    pub last_epoch_unix: f64,
}

impl GnssAveragingSession {
    pub fn start(target_epochs: usize, require_fix_only: bool) -> Self {
        Self {
            target_epochs: target_epochs.max(2),
            require_fix_only,
            samples: Vec::with_capacity(target_epochs),
            last_epoch_unix: -1.0,
        }
    }

    pub fn mean_and_std(&self) -> Option<([f64; 3], [f64; 3])> {
        let n = self.samples.len();
        if n == 0 {
            return None;
        }
        let nf = n as f64;
        let mx = self.samples.iter().map(|s| s.0).sum::<f64>() / nf;
        let my = self.samples.iter().map(|s| s.1).sum::<f64>() / nf;
        let mz = self.samples.iter().map(|s| s.2).sum::<f64>() / nf;

        let vx = self.samples.iter().map(|s| (s.0 - mx).powi(2)).sum::<f64>() / nf;
        let vy = self.samples.iter().map(|s| (s.1 - my).powi(2)).sum::<f64>() / nf;
        let vz = self.samples.iter().map(|s| (s.2 - mz).powi(2)).sum::<f64>() / nf;
        Some(([mx, my, mz], [vx.sqrt(), vy.sqrt(), vz.sqrt()]))
    }
}

type WfsInboxItem = (IgnWfsLayer, Result<FeatureCollection, String>);

pub struct SdncFieldApp {
    pub project: SdncProject,
    pub project_dir: PathBuf,
    pub project_file: PathBuf,
    pub layers: Vec<RuntimeLayer>,
    pub active_layer_idx: usize,
    pub undo_stack: Vec<(usize, FeatureCollection)>,

    pub active_tool: MapTool,
    pub show_layers_panel: bool,
    pub show_gnss_panel: bool,
    pub follow_gnss: bool,
    pub snapping_enabled: bool,
    pub snapping_radius_px: f32,

    pub tile_client: TileStreamClient,
    pub gpu_backend_label: String,
    pub wgpu_target_format: Option<egui_wgpu::wgpu::TextureFormat>,
    pub vector_gpu_batch: Vector2dGpuBatch,
    pub vector_gpu_dirty: bool,
    pub vp2d: Viewport2dState,

    /// Emprise WGS84 de la vue 2D courante `(min_lon, min_lat, max_lon, max_lat)` pour les requêtes WFS IGN.
    pub last_view_wgs84_bbox: Option<(f64, f64, f64, f64)>,
    pub wfs_selected_layers: Vec<IgnWfsLayer>,
    pub wfs_loading: Arc<AtomicBool>,
    pub wfs_inbox: Arc<Mutex<Vec<WfsInboxItem>>>,

    pub gnss: Arc<GnssController>,
    pub averaging: Option<GnssAveragingSession>,
    pub avg_target_epochs: usize,
    pub avg_require_fix: bool,
    pub ppk_rover_path: String,
    pub ppk_base_path: String,
    pub ppk_nav_path: String,
    pub ppk_combined: bool,

    pub draft_vertices: Vec<Coordinate>,
    pub pending_form: Option<PendingAttributeForm>,
    pub identified_info: Option<String>,
    pub search_query: String,
    pub status_banner: String,
    pub mobile_layout_initialized: bool,
}

impl SdncFieldApp {
    pub fn new(base_dir: PathBuf) -> Self {
        Self::new_with_wgpu(base_dir, None)
    }

    pub fn new_with_wgpu(
        base_dir: PathBuf,
        wgpu_target_format: Option<egui_wgpu::wgpu::TextureFormat>,
    ) -> Self {
        Self::new_with_wgpu_and_gnss(
            base_dir,
            wgpu_target_format,
            Arc::new(GnssController::default()),
        )
    }

    pub fn new_with_wgpu_and_gnss(
        base_dir: PathBuf,
        wgpu_target_format: Option<egui_wgpu::wgpu::TextureFormat>,
        gnss: Arc<GnssController>,
    ) -> Self {
        let project = SdncProject::default();
        let project_file = base_dir.join("projet_terrain.sdnc.json");

        let tile_client = TileStreamClient::new();
        let cache_db = base_dir.join(&project.basemap.tile_cache_rel_path);
        let _ = tile_client.enable_disk_cache(&cache_db);

        let gpu_backend_label = VulkanComputeContext::try_new()
            .map(|c| format!("{} ({})", c.info.adapter_name, c.info.backend_name))
            .unwrap_or_else(|_| {
                if wgpu_target_format.is_some() {
                    "WGPU / Vulkan 2D".to_string()
                } else {
                    "CPU 2D Fallback".to_string()
                }
            });

        let mut app = Self {
            project,
            project_dir: base_dir.clone(),
            project_file,
            layers: Vec::new(),
            active_layer_idx: 0,
            undo_stack: Vec::new(),
            active_tool: MapTool::Navigate,
            show_layers_panel: true,
            show_gnss_panel: true,
            follow_gnss: false,
            snapping_enabled: true,
            snapping_radius_px: 14.0,
            tile_client,
            gpu_backend_label: gpu_backend_label.clone(),
            wgpu_target_format,
            vector_gpu_batch: Vector2dGpuBatch::default(),
            vector_gpu_dirty: true,
            vp2d: Viewport2dState::default(),
            last_view_wgs84_bbox: None,
            wfs_selected_layers: vec![
                IgnWfsLayer::ParcellaireParcelles,
                IgnWfsLayer::ParcellaireBatiments,
            ],
            wfs_loading: Arc::new(AtomicBool::new(false)),
            wfs_inbox: Arc::new(Mutex::new(Vec::new())),
            gnss,
            averaging: None,
            avg_target_epochs: 10,
            avg_require_fix: true,
            ppk_rover_path: base_dir
                .join("gnss_logs/rover_raw.obs")
                .to_string_lossy()
                .to_string(),
            ppk_base_path: base_dir
                .join("gnss_logs/base_rgp.obs")
                .to_string_lossy()
                .to_string(),
            ppk_nav_path: base_dir
                .join("gnss_logs/brdc.nav")
                .to_string_lossy()
                .to_string(),
            ppk_combined: true,
            draft_vertices: Vec::new(),
            pending_form: None,
            identified_info: None,
            search_query: String::new(),
            status_banner: format!("SDNCQField-RS 2D prêt | GPU : {}", gpu_backend_label),
            mobile_layout_initialized: false,
        };

        app.init_default_layers();
        app
    }

    pub fn mark_vector_gpu_dirty(&mut self) {
        self.vector_gpu_dirty = true;
    }

    /// Reconstruit le batch VRAM 2D (`FillVertex2d` triangulés + `Segment2dGpu` instanciés)
    /// uniquement lorsque les couches, la visibilité ou le CRS changent.
    pub fn rebuild_vector_gpu_batch(&mut self) {
        let active_crs = self.project.active_crs;
        let origin_xy = if let Some((x0, y0, x1, y1)) = self.compute_layers_bbox() {
            [(x0 + x1) * 0.5, (y0 + y1) * 0.5]
        } else if let Some((cx, cy)) = self.project.center_xy {
            [cx, cy]
        } else {
            [700_000.0, 6_600_000.0]
        };

        let mut fill_vertices: Vec<FillVertex2d> = Vec::new();
        let mut segments: Vec<Segment2dGpu> = Vec::new();

        for layer in &self.layers {
            if !layer.spec.visible {
                continue;
            }
            let s = &layer.spec.style;
            let alpha_mul = layer.spec.opacity.clamp(0.0, 1.0);
            let stroke_rgba = [
                s.stroke_rgba[0],
                s.stroke_rgba[1],
                s.stroke_rgba[2],
                (s.stroke_rgba[3] as f32 * alpha_mul) as u8,
            ];
            let fill_rgba = [
                s.fill_rgba[0],
                s.fill_rgba[1],
                s.fill_rgba[2],
                (s.fill_rgba[3] as f32 * alpha_mul) as u8,
            ];
            let stroke_u32 = pack_rgba8(stroke_rgba);
            let fill_u32 = pack_rgba8(fill_rgba);
            let draw_fill = fill_rgba[3] > 0;
            let draw_stroke = stroke_rgba[3] > 0 && s.stroke_width > 0.0;
            let width_px = s.stroke_width.max(1.0);

            for feat in &layer.data.features {
                let Some(geom) = &feat.geometry else {
                    continue;
                };
                append_geometry_to_gpu_batch(
                    geom,
                    layer.spec.crs,
                    active_crs,
                    origin_xy,
                    fill_u32,
                    stroke_u32,
                    width_px,
                    draw_fill,
                    draw_stroke,
                    &mut fill_vertices,
                    &mut segments,
                );
            }
        }

        let next_gen = self.vector_gpu_batch.generation.wrapping_add(1).max(1);
        self.vector_gpu_batch = Vector2dGpuBatch {
            origin_xy,
            fill_vertices: Arc::new(fill_vertices),
            segments: Arc::new(segments),
            generation: next_gen,
        };
        self.vector_gpu_dirty = false;
    }

    /// Charge un îlot cadastral français de démonstration (Section AB, Parcelles, Bâtiments concaves L/U, Bornes).
    pub fn load_demo_cadastre(&mut self) {
        let (cx, cy) = self.project.center_xy.unwrap_or((894500.0, 6269500.0));
        let crs = self.project.active_crs;
        self.layers.retain(|l| !l.spec.id.starts_with("demo_cad_"));

        let demo_layers = generate_demo_cadastral_layers(cx, cy, crs);
        let mut total_feats = 0usize;
        for (spec, fc) in demo_layers {
            total_feats += fc.features.len();
            self.layers.push(RuntimeLayer {
                spec,
                data: fc,
                dirty: false,
            });
        }
        self.project.center_xy = Some((cx, cy));
        self.vp2d.pan_offset = egui::Vec2::ZERO;
        self.vp2d.zoom_level = 1.0;
        self.mark_vector_gpu_dirty();
        self.status_banner = format!(
            "Plan cadastral démo chargé ({} entités PCI : Section AB, Parcelles, Bâtis L/U, Bornes)",
            total_feats
        );
    }

    /// Importe un plan cadastral **DXF ou DXF-PCI** découpé par calques avec détection CRS et couleurs DGFiP/ACI.
    pub fn import_cadastral_dxf(&mut self, path: &Path) {
        match load_dxf_cadastral_layers(path, self.project.active_crs) {
            Ok((detected_crs, cad_layers)) => {
                let n_layers = cad_layers.len();
                let mut total_feats = 0usize;
                for (spec, fc) in cad_layers {
                    total_feats += fc.features.len();
                    self.layers.push(RuntimeLayer {
                        spec,
                        data: fc,
                        dirty: false,
                    });
                }
                self.project.active_crs = detected_crs;
                if let Some((x0, y0, x1, y1)) = self.compute_layers_bbox() {
                    self.project.center_xy = Some(((x0 + x1) * 0.5, (y0 + y1) * 0.5));
                }
                self.vp2d.pan_offset = egui::Vec2::ZERO;
                self.vp2d.zoom_level = 1.0;
                self.mark_vector_gpu_dirty();
                self.status_banner = format!(
                    "Cadastre DXF/DXF-PCI importé : {} calques, {} entités (EPSG:{})",
                    n_layers,
                    total_feats,
                    detected_crs.epsg()
                );
            }
            Err(e) => {
                self.status_banner = format!("Erreur import DXF/DXF-PCI : {}", e);
            }
        }
    }

    /// Importe un lot cadastral **EDIGÉO** (fichier `.THF` ou dossier) avec topologie NF Z 52-000 & symbologie DGFiP.
    pub fn import_cadastral_edigeo(&mut self, path: &Path) {
        match load_edigeo_cadastral_layers(path, self.project.active_crs) {
            Ok((detected_crs, cad_layers)) => {
                let n_layers = cad_layers.len();
                let mut total_feats = 0usize;
                for (spec, fc) in cad_layers {
                    total_feats += fc.features.len();
                    self.layers.push(RuntimeLayer {
                        spec,
                        data: fc,
                        dirty: false,
                    });
                }
                self.project.active_crs = detected_crs;
                if let Some((x0, y0, x1, y1)) = self.compute_layers_bbox() {
                    self.project.center_xy = Some(((x0 + x1) * 0.5, (y0 + y1) * 0.5));
                }
                self.vp2d.pan_offset = egui::Vec2::ZERO;
                self.vp2d.zoom_level = 1.0;
                self.mark_vector_gpu_dirty();
                self.status_banner = format!(
                    "Cadastre EDIGÉO importé : {} couches DGFiP, {} objets (EPSG:{})",
                    n_layers,
                    total_feats,
                    detected_crs.epsg()
                );
            }
            Err(e) => {
                self.status_banner = format!("Erreur import EDIGÉO : {}", e);
            }
        }
    }

    /// Lance le téléchargement asynchrone des flux **IGN Géoplateforme Vecteur (WFS 2.0 GeoJSON)**
    /// sur l'emprise 2D actuellement affichée à l'écran.
    pub fn request_ign_wfs_for_current_view(&mut self, ctx: egui::Context) {
        if self.wfs_selected_layers.is_empty() {
            self.status_banner = "Sélectionnez au moins une couche IGN WFS à charger".to_string();
            return;
        }
        if self.wfs_loading.swap(true, Ordering::SeqCst) {
            return;
        }
        let Some((mut min_lon, mut min_lat, mut max_lon, mut max_lat)) = self.last_view_wgs84_bbox
        else {
            self.wfs_loading.store(false, Ordering::SeqCst);
            self.status_banner = "Emprise 2D indisponible pour la requête IGN WFS".to_string();
            return;
        };

        // Limiter l'emprise WFS à ~4 km autour du centre si l'utilisateur est très dézoomé
        let max_span_deg = 0.04;
        if (max_lon - min_lon) > max_span_deg {
            let cx = 0.5 * (min_lon + max_lon);
            min_lon = cx - max_span_deg * 0.5;
            max_lon = cx + max_span_deg * 0.5;
        }
        if (max_lat - min_lat) > max_span_deg {
            let cy = 0.5 * (min_lat + max_lat);
            min_lat = cy - max_span_deg * 0.5;
            max_lat = cy + max_span_deg * 0.5;
        }

        let layers_to_fetch = self.wfs_selected_layers.clone();
        let inbox = Arc::clone(&self.wfs_inbox);
        let loading = Arc::clone(&self.wfs_loading);
        self.status_banner = format!(
            "Chargement IGN WFS ({} couche(s)) sur la vue courante...",
            layers_to_fetch.len()
        );

        std::thread::spawn(move || {
            for layer in layers_to_fetch {
                let res = fetch_ign_wfs_geojson(layer, min_lon, min_lat, max_lon, max_lat, 1500)
                    .map_err(|e| e.to_string())
                    .and_then(|body| {
                        FeatureCollection::from_geojson(&body).map_err(|e| e.to_string())
                    });
                inbox
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .push((layer, res));
                ctx.request_repaint();
            }
            loading.store(false, Ordering::SeqCst);
            ctx.request_repaint();
        });
    }

    /// Intègre les couches vectorielles IGN WFS reçues en arrière-plan (reprojection WGS84 → CRS actif).
    pub fn poll_wfs_inbox(&mut self) {
        let items: Vec<WfsInboxItem> = {
            let mut guard = self.wfs_inbox.lock().unwrap_or_else(|e| e.into_inner());
            if guard.is_empty() {
                return;
            }
            std::mem::take(&mut *guard)
        };

        let active_crs = self.project.active_crs;
        let mut total_added = 0usize;
        let mut last_err: Option<String> = None;

        for (wfs_layer, res) in items {
            match res {
                Ok(mut fc) => {
                    // Les flux IGN WFS CRS:84 arrivent en (lon, lat) WGS84 -> reprojeter dans active_crs
                    for feat in &mut fc.features {
                        if let Some(ref mut g) = feat.geometry {
                            reproject_geometry_from_wgs84(g, active_crs);
                        }
                    }
                    let count = fc.features.len();
                    total_added += count;

                    let (stroke_rgba, fill_rgba, stroke_width, label_field) =
                        wfs_layer.default_style();
                    let table_name = format!("ign_wfs_{:?}", wfs_layer).to_lowercase();
                    let spec = ProjectLayerSpec {
                        id: table_name.clone(),
                        name: wfs_layer.label().to_string(),
                        source: LayerSource::GeoPackageTable {
                            gpkg_rel_path: self.project.gpkg_file.clone(),
                            table_name,
                        },
                        crs: active_crs,
                        visible: true,
                        editable: false,
                        opacity: 1.0,
                        style: LayerStyle {
                            stroke_rgba,
                            fill_rgba,
                            stroke_width,
                            point_radius: 4.0,
                            label_field: label_field.map(|s| s.to_string()),
                        },
                        form_fields: sdnc_data::default_field_form(),
                    };

                    if let Some(existing) = self.layers.iter_mut().find(|l| l.spec.id == spec.id) {
                        existing.spec.crs = active_crs;
                        existing.data = fc;
                        existing.dirty = true;
                    } else {
                        self.layers.push(RuntimeLayer {
                            spec,
                            data: fc,
                            dirty: true,
                        });
                    }
                    self.mark_vector_gpu_dirty();
                }
                Err(e) => {
                    last_err = Some(format!("{} : {}", wfs_layer.label(), e));
                }
            }
        }

        if let Some(err) = last_err {
            self.status_banner = format!("IGN WFS partiel/erreur — {}", err);
        } else {
            self.status_banner = format!(
                "Flux Vecteurs IGN WFS chargés ({} entités en EPSG:{}, sauvegardables en .gpkg)",
                total_added,
                active_crs.epsg()
            );
        }
    }

    /// Met à jour une session de moyennage RTK active à chaque nouvelle époque GNSS.
    pub fn tick_gnss_averaging(&mut self, tel: &GnssLiveTelemetry) {
        let Some(sess) = &mut self.averaging else {
            return;
        };
        let q = tel.solution.quality;
        if q == FixQuality::NoFix {
            return;
        }
        if sess.require_fix_only && !matches!(q, FixQuality::RtkFix | FixQuality::PpkFix) {
            return;
        }
        let t_unix = tel.solution.time.to_unix_f64();
        if (t_unix - sess.last_epoch_unix).abs() < 1e-3 {
            return;
        }
        if let Ok((wx, wy)) = from_wgs84(
            self.project.active_crs,
            tel.solution.lon_deg,
            tel.solution.lat_deg,
        ) {
            let ant_h = self
                .gnss
                .config
                .lock()
                .map(|c| c.antenna_height_m)
                .unwrap_or(0.0);
            let z = tel.solution.alt_orthometric_m - ant_h;
            sess.samples.push((wx, wy, z));
            sess.last_epoch_unix = t_unix;
        }

        if sess.samples.len() >= sess.target_epochs {
            let done = self.averaging.take().unwrap();
            if let Some(([mx, my, mz], [sx, sy, sz])) = done.mean_and_std() {
                let geom = Geometry::Point {
                    coordinates: Coordinate::new_3d(mx, my, mz),
                };
                self.open_attribute_form(geom, tel);
                if let Some(form) = &mut self.pending_form {
                    let sh_mm = (sx * sx + sy * sy).sqrt() * 1000.0;
                    let sv_mm = sz * 1000.0;
                    let summary = format!(
                        "Moyennage {} ép. ({}) | σH={:.1} mm σV={:.1} mm",
                        done.samples.len(),
                        q.label(),
                        sh_mm,
                        sv_mm
                    );
                    form.gnss_summary = summary.clone();
                    form.values.insert("gnss_statut".to_string(), summary);
                }
                self.status_banner = format!(
                    "Moyennage GNSS terminé ({} époques) — prêt à enregistrer",
                    done.samples.len()
                );
            }
        }
    }

    /// Exporte le dernier rapport PPK en CSV topographique (EPSG actif + WGS84 + qualité + σ).
    pub fn export_ppk_report_csv(&mut self, tel: &GnssLiveTelemetry) {
        let Some(report) = &tel.last_ppk_report else {
            return;
        };
        let out_path = self.project_dir.join("ppk_export.csv");
        if let Some(parent) = out_path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let crs = self.project.active_crs;
        let mut csv = String::from(
            "epoch_idx,gps_unix_s,quality,epsg,x_m,y_m,z_ngf_m,lat_deg,lon_deg,h_ellip_m,std_e_m,std_n_m,std_u_m,lambda_ratio,num_sats\n",
        );
        for (i, s) in report.solutions.iter().enumerate() {
            let (wx, wy) = from_wgs84(crs, s.lon_deg, s.lat_deg).unwrap_or((0.0, 0.0));
            csv.push_str(&format!(
                "{},{:.3},{},{},{:.4},{:.4},{:.4},{:.8},{:.8},{:.4},{:.4},{:.4},{:.4},{:.2},{}\n",
                i + 1,
                s.time.to_unix_f64(),
                s.quality.label(),
                crs.epsg(),
                wx,
                wy,
                s.alt_orthometric_m,
                s.lat_deg,
                s.lon_deg,
                s.alt_ellipsoid_m,
                s.std_enu_m[0],
                s.std_enu_m[1],
                s.std_enu_m[2],
                s.lambda_ratio,
                s.num_sats
            ));
        }
        match std::fs::write(&out_path, csv) {
            Ok(()) => {
                self.status_banner = format!(
                    "Rapport PPK exporté en CSV ({} époques -> {:?})",
                    report.total_epochs,
                    out_path.file_name().unwrap_or_default()
                );
            }
            Err(e) => self.status_banner = format!("Erreur export CSV PPK : {}", e),
        }
    }

    fn init_default_layers(&mut self) {
        self.layers.clear();
        let specs = self.project.layers.clone();
        for (idx, spec) in specs.into_iter().enumerate() {
            let mut fc = FeatureCollection::empty();
            if idx == 0 {
                let mut f = Feature::with_geometry(Geometry::Point {
                    coordinates: Coordinate::new_3d(894510.0, 6269520.0, 242.5),
                });
                f.set_property("nom", "Borne_IGN_01");
                f.set_property("categorie", "Borne");
                f.set_property("gnss_statut", "RTK FIX (±1.2 cm)");
                fc.add_feature(f);

                let mut f2 = Feature::with_geometry(Geometry::Point {
                    coordinates: Coordinate::new_3d(894690.0, 6269610.0, 256.8),
                });
                f2.set_property("nom", "GCP_Terrain_02");
                f2.set_property("categorie", "GCP");
                f2.set_property("gnss_statut", "PPK FIX (±0.9 cm)");
                fc.add_feature(f2);
            } else if idx == 2 {
                let mut poly = Feature::with_geometry(Geometry::Polygon {
                    coordinates: vec![vec![
                        Coordinate::new_3d(894440.0, 6269460.0, 239.0),
                        Coordinate::new_3d(894760.0, 6269460.0, 246.0),
                        Coordinate::new_3d(894760.0, 6269700.0, 261.0),
                        Coordinate::new_3d(894440.0, 6269700.0, 250.0),
                        Coordinate::new_3d(894440.0, 6269460.0, 239.0),
                    ]],
                });
                poly.set_property("nom", "Zone_Etude_Sud");
                poly.set_property("categorie", "Limite");
                fc.add_feature(poly);
            }

            self.layers.push(RuntimeLayer {
                spec,
                data: fc,
                dirty: false,
            });
        }
        self.mark_vector_gpu_dirty();
    }

    /// Sauvegarde le projet natif (`.sdnc.json`) + toutes les couches GeoPackage (y compris IGN WFS).
    pub fn save_project(&mut self) {
        let gpkg_path = self.project.resolve_path(&self.project_dir, &self.project.gpkg_file);
        match GeoPackage::open(&gpkg_path) {
            Ok(mut gpkg) => {
                let mut saved_count = 0usize;
                for layer in &mut self.layers {
                    if let LayerSource::GeoPackageTable { table_name, .. } = &layer.spec.source {
                        let srs = layer.spec.crs.epsg() as i32;
                        if let Err(e) = gpkg.save_layer(
                            table_name,
                            &layer.data,
                            srs,
                            GeometryType::GeometryCollection,
                        ) {
                            self.status_banner =
                                format!("Erreur sauvegarde table {}: {}", table_name, e);
                            return;
                        }
                        layer.dirty = false;
                        saved_count += 1;
                    }
                }
                self.project.layers = self.layers.iter().map(|l| l.spec.clone()).collect();
                match self.project.save(&self.project_file) {
                    Ok(()) => {
                        self.status_banner = format!(
                            "Projet sauvegardé ({} tables dans {:?} + {:?})",
                            saved_count,
                            gpkg_path.file_name().unwrap_or_default(),
                            self.project_file.file_name().unwrap_or_default()
                        );
                    }
                    Err(e) => self.status_banner = format!("Erreur JSON projet: {}", e),
                }
            }
            Err(e) => self.status_banner = format!("Erreur ouverture GeoPackage: {}", e),
        }
    }

    /// Charge un projet `.sdnc.json` et ses couches `.gpkg` / `.geojson` / `.dxf` / EDIGÉO.
    pub fn load_project_file(&mut self, json_path: &Path) {
        match SdncProject::load(json_path) {
            Ok(proj) => {
                let dir = json_path
                    .parent()
                    .map(|p| p.to_path_buf())
                    .unwrap_or_else(|| self.project_dir.clone());
                self.project = proj;
                self.project_dir = dir;
                self.project_file = json_path.to_path_buf();
                self.layers.clear();

                let specs = self.project.layers.clone();
                for spec in specs {
                    let fc = match &spec.source {
                        LayerSource::GeoPackageTable {
                            gpkg_rel_path,
                            table_name,
                        } => {
                            let p = self.project.resolve_path(&self.project_dir, gpkg_rel_path);
                            GeoPackage::open(p)
                                .and_then(|g| g.load_layer(table_name))
                                .map(|(c, _)| c)
                                .unwrap_or_else(|_| FeatureCollection::empty())
                        }
                        LayerSource::GeoJsonFile { rel_path } => {
                            let p = self.project.resolve_path(&self.project_dir, rel_path);
                            std::fs::read_to_string(p)
                                .ok()
                                .and_then(|s| FeatureCollection::from_geojson(&s).ok())
                                .unwrap_or_else(FeatureCollection::empty)
                        }
                        LayerSource::DxfFile { rel_path } => {
                            let p = self.project.resolve_path(&self.project_dir, rel_path);
                            sdnc_data::DxfParser::parse(p)
                                .map(|d| d.to_feature_collection(None))
                                .unwrap_or_else(|_| FeatureCollection::empty())
                        }
                        LayerSource::EdigeoDir { rel_path } => {
                            let p = self.project.resolve_path(&self.project_dir, rel_path);
                            sdnc_data::EdigeoParser::parse(p)
                                .map(|d| d.to_feature_collection(None))
                                .unwrap_or_else(|_| FeatureCollection::empty())
                        }
                        LayerSource::DemRaster { .. } => FeatureCollection::empty(),
                    };
                    self.layers.push(RuntimeLayer {
                        spec,
                        data: fc,
                        dirty: false,
                    });
                }
                self.mark_vector_gpu_dirty();
                self.status_banner = format!(
                    "Projet '{}' chargé ({} couches)",
                    self.project.name,
                    self.layers.len()
                );
            }
            Err(e) => self.status_banner = format!("Erreur chargement projet: {}", e),
        }
    }

    /// Importe un fichier GeoPackage, GeoJSON, DXF/DXF-PCI ou EDIGÉO (`.thf` ou dossier).
    pub fn import_vector_file(&mut self, path: &Path) {
        if path.is_dir() {
            self.import_cadastral_edigeo(path);
            return;
        }
        let ext = path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("")
            .to_lowercase();
        let stem = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("couche")
            .to_string();

        match ext.as_str() {
            "dxf" => self.import_cadastral_dxf(path),
            "thf" | "vec" | "geo" => self.import_cadastral_edigeo(path),
            "gpkg" => match GeoPackage::open(path) {
                Ok(gpkg) => {
                    if let Ok(metas) = gpkg.list_layers() {
                        let mut added = 0;
                        for m in metas {
                            if let Ok((fc, srs)) = gpkg.load_layer(&m.table_name) {
                                let crs = FrenchCrs::from_epsg(srs as u32)
                                    .unwrap_or(self.project.active_crs);
                                self.layers.push(RuntimeLayer {
                                    spec: ProjectLayerSpec {
                                        id: format!("gpkg_{}", m.table_name),
                                        name: m.table_name.clone(),
                                        source: LayerSource::GeoPackageTable {
                                            gpkg_rel_path: path.to_string_lossy().to_string(),
                                            table_name: m.table_name,
                                        },
                                        crs,
                                        visible: true,
                                        editable: true,
                                        opacity: 1.0,
                                        style: Default::default(),
                                        form_fields: sdnc_data::default_field_form(),
                                    },
                                    data: fc,
                                    dirty: false,
                                });
                                added += 1;
                            }
                        }
                        self.mark_vector_gpu_dirty();
                        self.status_banner =
                            format!("GeoPackage importé : {} couche(s) depuis {}", added, stem);
                    }
                }
                Err(e) => self.status_banner = format!("Erreur GPKG: {}", e),
            },
            "geojson" | "json" => match std::fs::read_to_string(path)
                .ok()
                .and_then(|s| FeatureCollection::from_geojson(&s).ok())
            {
                Some(fc) => {
                    let count = fc.features.len();
                    self.layers.push(RuntimeLayer {
                        spec: ProjectLayerSpec {
                            id: format!("geojson_{}", stem),
                            name: stem.clone(),
                            source: LayerSource::GeoJsonFile {
                                rel_path: path.to_string_lossy().to_string(),
                            },
                            crs: self.project.active_crs,
                            visible: true,
                            editable: true,
                            opacity: 1.0,
                            style: Default::default(),
                            form_fields: sdnc_data::default_field_form(),
                        },
                        data: fc,
                        dirty: false,
                    });
                    self.mark_vector_gpu_dirty();
                    self.status_banner = format!("GeoJSON '{}' importé ({} entités)", stem, count);
                }
                None => self.status_banner = format!("GeoJSON invalide: {:?}", path),
            },
            _ => {
                self.status_banner = format!("Format non reconnu : .{}", ext);
            }
        }
    }

    /// Ouvre le formulaire attributaire pour valider une géométrie saisie sur la carte ou au GNSS.
    pub fn open_attribute_form(&mut self, geometry: Geometry, gnss_tel: &GnssLiveTelemetry) {
        let Some(layer) = self.layers.get(self.active_layer_idx) else {
            return;
        };
        let mut values = HashMap::new();
        let next_num = layer.data.features.len() + 1;
        for f in &layer.spec.form_fields {
            let def = if f.name == "nom" {
                format!("PT_{:03}", next_num)
            } else {
                f.default_value.clone().unwrap_or_default()
            };
            values.insert(f.name.clone(), def);
        }

        let gnss_summary = if gnss_tel.solution.quality != FixQuality::NoFix {
            let sh_cm = ((gnss_tel.solution.std_enu_m[0].powi(2)
                + gnss_tel.solution.std_enu_m[1].powi(2))
            .sqrt())
                * 100.0;
            format!(
                "{} (±{:.1} cm, Ratio {:.1}, {} sats)",
                gnss_tel.solution.quality.label(),
                sh_cm,
                gnss_tel.solution.lambda_ratio,
                gnss_tel.solution.num_sats
            )
        } else {
            "Saisie manuelle carte".to_string()
        };
        values.insert("gnss_statut".to_string(), gnss_summary.clone());

        self.pending_form = Some(PendingAttributeForm {
            layer_id: layer.spec.id.clone(),
            geometry,
            values,
            gnss_summary,
        });
    }

    /// Enregistre l'entité validée dans la couche active (avec pile Undo).
    pub fn commit_pending_form(&mut self) {
        let Some(form) = self.pending_form.take() else {
            return;
        };
        if let Some((idx, layer)) = self
            .layers
            .iter_mut()
            .enumerate()
            .find(|(_, l)| l.spec.id == form.layer_id)
        {
            self.undo_stack.push((idx, layer.data.clone()));
            if self.undo_stack.len() > 30 {
                self.undo_stack.remove(0);
            }
            let mut feat = Feature::with_geometry(form.geometry);
            for (k, v) in form.values {
                feat.properties.insert(k, PropertyValue::String(v));
            }
            feat.set_property("horodatage", chrono::Utc::now().to_rfc3339());
            layer.data.add_feature(feat);
            layer.dirty = true;
            let name = layer.spec.name.clone();
            self.mark_vector_gpu_dirty();
            self.status_banner = format!("Entité ajoutée dans '{}'", name);
        }
    }

    pub fn undo_last_edit(&mut self) {
        if let Some((idx, prev_fc)) = self.undo_stack.pop() {
            if let Some(layer) = self.layers.get_mut(idx) {
                layer.data = prev_fc;
                layer.dirty = true;
                let name = layer.spec.name.clone();
                self.mark_vector_gpu_dirty();
                self.status_banner = format!("Dernière modification annulée ({})", name);
            }
        }
    }

    /// Capture un point directement aux coordonnées GNSS courantes (RTK/PPK) dans la couche active.
    pub fn capture_gnss_point(&mut self, tel: &GnssLiveTelemetry) {
        if tel.solution.quality == FixQuality::NoFix {
            self.status_banner = "Aucun fix GNSS disponible pour la capture".to_string();
            return;
        }
        let crs = self.project.active_crs;
        if let Ok((wx, wy)) = from_wgs84(crs, tel.solution.lon_deg, tel.solution.lat_deg) {
            let ant_h = self
                .gnss
                .config
                .lock()
                .map(|c| c.antenna_height_m)
                .unwrap_or(0.0);
            let z = tel.solution.alt_orthometric_m - ant_h;
            let geom = Geometry::Point {
                coordinates: Coordinate::new_3d(wx, wy, z),
            };
            self.open_attribute_form(geom, tel);
        }
    }

    /// Importe la dernière trajectoire PPK post-traitée comme couche vectorielle dans le projet.
    pub fn import_ppk_report_as_layer(&mut self, tel: &GnssLiveTelemetry) {
        let Some(report) = &tel.last_ppk_report else {
            return;
        };
        if report.solutions.is_empty() {
            return;
        }
        let crs = self.project.active_crs;
        let mut fc = FeatureCollection::new(Vec::new());
        let mut line_coords = Vec::with_capacity(report.solutions.len());

        for (i, s) in report.solutions.iter().enumerate() {
            if let Ok((wx, wy)) = from_wgs84(crs, s.lon_deg, s.lat_deg) {
                let c = Coordinate::new_3d(wx, wy, s.alt_orthometric_m);
                line_coords.push(c);
                let mut pt = Feature::with_geometry(Geometry::Point { coordinates: c });
                pt.set_property("nom", format!("PPK_{:04}", i + 1));
                pt.set_property("gnss_statut", s.quality.label());
                pt.set_property("ratio_lambda", s.lambda_ratio as f64);
                fc.add_feature(pt);
            }
        }
        if line_coords.len() >= 2 {
            let mut trk = Feature::with_geometry(Geometry::LineString {
                coordinates: line_coords,
            });
            trk.set_property("nom", "Trajectoire_PPK_Combined");
            fc.add_feature(trk);
        }

        let table_name = format!("ppk_track_{}", self.layers.len() + 1);
        self.layers.push(RuntimeLayer {
            spec: ProjectLayerSpec {
                id: table_name.clone(),
                name: format!("Trajectoire PPK ({:.0}% FIX)", report.fix_rate_pct),
                source: LayerSource::GeoPackageTable {
                    gpkg_rel_path: self.project.gpkg_file.clone(),
                    table_name,
                },
                crs,
                visible: true,
                editable: true,
                opacity: 1.0,
                style: LayerStyle {
                    stroke_rgba: [40, 240, 120, 255],
                    fill_rgba: [40, 240, 120, 180],
                    stroke_width: 2.5,
                    point_radius: 4.5,
                    label_field: None,
                },
                form_fields: sdnc_data::default_field_form(),
            },
            data: fc,
            dirty: true,
        });
        self.mark_vector_gpu_dirty();
        self.status_banner = format!(
            "Trajectoire PPK ({} époques) ajoutée au projet (prête à sauvegarder en .gpkg)",
            report.total_epochs
        );
    }

    pub fn launch_ppk_from_files(&mut self) {
        let cfg = self
            .gnss
            .config
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone();
        let job = PpkJobConfig {
            rover_obs_path: self.ppk_rover_path.clone(),
            base_obs_path: self.ppk_base_path.clone(),
            nav_path: self.ppk_nav_path.clone(),
            base_ecef_override: cfg.base_ecef_override,
            combined_forward_backward: self.ppk_combined,
            gnss_config: cfg,
        };
        self.gnss.run_ppk_files_async(job);
    }

    /// Calcule la boîte englobante 2D `(min_x, min_y, max_x, max_y)` de toutes les couches visibles dans le CRS actif.
    pub fn compute_layers_bbox(&self) -> Option<(f64, f64, f64, f64)> {
        let active_crs = self.project.active_crs;
        let mut min_x = f64::MAX;
        let mut max_x = f64::MIN;
        let mut min_y = f64::MAX;
        let mut max_y = f64::MIN;

        for layer in &self.layers {
            if !layer.spec.visible {
                continue;
            }
            for f in &layer.data.features {
                if let Some(g) = &f.geometry {
                    if let Some((x0, y0, x1, y1)) = g.bbox() {
                        for (cx, cy) in [(x0, y0), (x1, y1)] {
                            let (px, py) = if layer.spec.crs != active_crs {
                                reproject_point_3d(layer.spec.crs, active_crs, cx, cy, 0.0)
                                    .map(|(a, b, _)| (a, b))
                                    .unwrap_or((cx, cy))
                            } else {
                                (cx, cy)
                            };
                            if px.is_finite() && py.is_finite() {
                                min_x = min_x.min(px);
                                max_x = max_x.max(px);
                                min_y = min_y.min(py);
                                max_y = max_y.max(py);
                            }
                        }
                    }
                }
            }
        }

        if min_x <= max_x && min_y <= max_y {
            Some((min_x, min_y, max_x, max_y))
        } else {
            None
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn append_geometry_to_gpu_batch(
    geom: &Geometry,
    src_crs: FrenchCrs,
    dst_crs: FrenchCrs,
    origin_xy: [f64; 2],
    fill_rgba: u32,
    stroke_rgba: u32,
    width_px: f32,
    draw_fill: bool,
    draw_stroke: bool,
    fill_out: &mut Vec<FillVertex2d>,
    seg_out: &mut Vec<Segment2dGpu>,
) {
    let to_rel = |c: &Coordinate| -> (f32, f32) {
        let (x, y) = if src_crs != dst_crs {
            reproject_point_3d(src_crs, dst_crs, c.x, c.y, c.z.unwrap_or(0.0))
                .map(|(rx, ry, _)| (rx, ry))
                .unwrap_or((c.x, c.y))
        } else {
            (c.x, c.y)
        };
        ((x - origin_xy[0]) as f32, (y - origin_xy[1]) as f32)
    };

    let push_polyline = |coords: &[Coordinate], segs: &mut Vec<Segment2dGpu>| {
        if coords.len() < 2 || !draw_stroke {
            return;
        }
        let mut prev = to_rel(&coords[0]);
        for c in coords.iter().skip(1) {
            let next = to_rel(c);
            if (next.0 - prev.0).hypot(next.1 - prev.1) > 1e-5 {
                segs.push(Segment2dGpu {
                    x0: prev.0,
                    y0: prev.1,
                    x1: next.0,
                    y1: next.1,
                    width_px,
                    rgba: stroke_rgba,
                    pad0: 0,
                    pad1: 0,
                });
            }
            prev = next;
        }
    };

    match geom {
        Geometry::Point { .. } | Geometry::MultiPoint { .. } => {
            // Les points et textes cadastraux sont rendus avec culling viewport dans viewport_2d.rs
        }
        Geometry::LineString { coordinates } => {
            push_polyline(coordinates, seg_out);
        }
        Geometry::MultiLineString { coordinates } => {
            for line in coordinates {
                push_polyline(line, seg_out);
            }
        }
        Geometry::Polygon { coordinates } => {
            if let Some(ext) = coordinates.first() {
                if draw_fill && ext.len() >= 3 {
                    let rel_ring: Vec<(f32, f32)> = ext.iter().map(to_rel).collect();
                    triangulate_polygon_earclip(&rel_ring, fill_rgba, fill_out);
                }
            }
            for ring in coordinates {
                push_polyline(ring, seg_out);
            }
        }
        Geometry::MultiPolygon { coordinates } => {
            for poly in coordinates {
                if let Some(ext) = poly.first() {
                    if draw_fill && ext.len() >= 3 {
                        let rel_ring: Vec<(f32, f32)> = ext.iter().map(to_rel).collect();
                        triangulate_polygon_earclip(&rel_ring, fill_rgba, fill_out);
                    }
                }
                for ring in poly {
                    push_polyline(ring, seg_out);
                }
            }
        }
        Geometry::GeometryCollection { geometries } => {
            for sub in geometries {
                append_geometry_to_gpu_batch(
                    sub,
                    src_crs,
                    dst_crs,
                    origin_xy,
                    fill_rgba,
                    stroke_rgba,
                    width_px,
                    draw_fill,
                    draw_stroke,
                    fill_out,
                    seg_out,
                );
            }
        }
    }
}

fn reproject_geometry_from_wgs84(geom: &mut Geometry, dst_crs: FrenchCrs) {
    let proj_c = |c: &mut Coordinate| {
        if let Ok((x, y)) = from_wgs84(dst_crs, c.x, c.y) {
            c.x = x;
            c.y = y;
        }
    };
    match geom {
        Geometry::Point { coordinates } => proj_c(coordinates),
        Geometry::LineString { coordinates } | Geometry::MultiPoint { coordinates } => {
            for c in coordinates {
                proj_c(c);
            }
        }
        Geometry::Polygon { coordinates } | Geometry::MultiLineString { coordinates } => {
            for ring in coordinates {
                for c in ring {
                    proj_c(c);
                }
            }
        }
        Geometry::MultiPolygon { coordinates } => {
            for poly in coordinates {
                for ring in poly {
                    for c in ring {
                        proj_c(c);
                    }
                }
            }
        }
        Geometry::GeometryCollection { geometries } => {
            for sub in geometries {
                reproject_geometry_from_wgs84(sub, dst_crs);
            }
        }
    }
}
