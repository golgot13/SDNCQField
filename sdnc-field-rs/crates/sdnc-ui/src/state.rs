//! État global de l'application terrain SDNCQField (Rust + Vulkan Compute).

use crate::viewport_2d::Viewport2dState;
use crate::viewport_3d::Viewport3dState;
use sdnc_data::{
    Coordinate, Feature, FeatureCollection, GeoPackage, Geometry, GeometryType, LayerSource,
    ProjectLayerSpec, PropertyValue, SdncProject,
};
use sdnc_geo::{from_wgs84, reproject_point_3d, FrenchCrs, TileStreamClient};
use sdnc_gnss::{FixQuality, GnssController, GnssLiveTelemetry, PpkJobConfig};
use sdnc_gpu::VulkanTerrainEngine;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewMode {
    Map2D,
    Surface3D,
    Split2D3D,
}

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

pub struct SdncFieldApp {
    pub project: SdncProject,
    pub project_dir: PathBuf,
    pub project_file: PathBuf,
    pub layers: Vec<RuntimeLayer>,
    pub active_layer_idx: usize,
    pub undo_stack: Vec<(usize, FeatureCollection)>,

    pub view_mode: ViewMode,
    pub active_tool: MapTool,
    pub show_layers_panel: bool,
    pub show_gnss_panel: bool,
    pub follow_gnss: bool,
    pub snapping_enabled: bool,
    pub snapping_radius_px: f32,

    pub tile_client: TileStreamClient,
    pub terrain_engine: VulkanTerrainEngine,
    pub vp2d: Viewport2dState,
    pub vp3d: Viewport3dState,

    pub gnss: GnssController,
    pub ppk_rover_path: String,
    pub ppk_base_path: String,
    pub ppk_nav_path: String,
    pub ppk_combined: bool,

    pub draft_vertices: Vec<Coordinate>,
    pub pending_form: Option<PendingAttributeForm>,
    pub identified_info: Option<String>,
    pub search_query: String,
    pub status_banner: String,
}

impl SdncFieldApp {
    pub fn new(base_dir: PathBuf) -> Self {
        let project = SdncProject::default();
        let project_file = base_dir.join("projet_terrain.sdnc.json");

        let tile_client = TileStreamClient::new();
        let cache_db = base_dir.join(&project.basemap.tile_cache_rel_path);
        let _ = tile_client.enable_disk_cache(&cache_db);

        let terrain_engine = VulkanTerrainEngine::new();
        let gpu_label = format!(
            "{} ({})",
            terrain_engine.backend_info.adapter_name, terrain_engine.backend_info.backend_name
        );

        let mut app = Self {
            project,
            project_dir: base_dir.clone(),
            project_file,
            layers: Vec::new(),
            active_layer_idx: 0,
            undo_stack: Vec::new(),
            view_mode: ViewMode::Map2D,
            active_tool: MapTool::Navigate,
            show_layers_panel: true,
            show_gnss_panel: true,
            follow_gnss: false,
            snapping_enabled: true,
            snapping_radius_px: 14.0,
            tile_client,
            terrain_engine,
            vp2d: Viewport2dState::default(),
            vp3d: Viewport3dState::default(),
            gnss: GnssController::default(),
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
            status_banner: format!("SDNCQField Rust prêt | Compute : {}", gpu_label),
        };

        app.init_default_layers();
        app
    }

    fn init_default_layers(&mut self) {
        self.layers.clear();
        let specs = self.project.layers.clone();
        for (idx, spec) in specs.into_iter().enumerate() {
            let mut fc = FeatureCollection::empty();
            // Quelques entités exemples autour de Meyreuil / Aix-en-Provence (Lambert-93)
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
        self.vp3d.invalidate_terrain();
    }

    /// Sauvegarde le projet natif (`.sdnc.json`) + toutes les couches éditables dans le `.gpkg`.
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

    /// Charge un projet `.sdnc.json` et ses couches `.gpkg` / `.geojson` / `.dxf`.
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
                self.vp3d.invalidate_terrain();
                self.status_banner = format!(
                    "Projet '{}' chargé ({} couches)",
                    self.project.name,
                    self.layers.len()
                );
            }
            Err(e) => self.status_banner = format!("Erreur chargement projet: {}", e),
        }
    }

    /// Importe un fichier GeoPackage, GeoJSON ou DXF directement en nouvelle couche.
    pub fn import_vector_file(&mut self, path: &Path) {
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
                        self.vp3d.invalidate_terrain();
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
                    self.vp3d.invalidate_terrain();
                    self.status_banner = format!("GeoJSON '{}' importé ({} entités)", stem, count);
                }
                None => self.status_banner = format!("GeoJSON invalide: {:?}", path),
            },
            "dxf" => match sdnc_data::DxfParser::parse(path) {
                Ok(doc) => {
                    let fc = doc.to_feature_collection(None);
                    let count = fc.features.len();
                    self.layers.push(RuntimeLayer {
                        spec: ProjectLayerSpec {
                            id: format!("dxf_{}", stem),
                            name: format!("{} (DXF)", stem),
                            source: LayerSource::DxfFile {
                                rel_path: path.to_string_lossy().to_string(),
                            },
                            crs: self.project.active_crs,
                            visible: true,
                            editable: false,
                            opacity: 1.0,
                            style: Default::default(),
                            form_fields: sdnc_data::default_field_form(),
                        },
                        data: fc,
                        dirty: false,
                    });
                    self.vp3d.invalidate_terrain();
                    self.status_banner = format!("DXF '{}' importé ({} entités)", stem, count);
                }
                Err(e) => self.status_banner = format!("Erreur DXF: {}", e),
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
            self.vp3d.invalidate_terrain();
            self.status_banner = format!("Entité ajoutée dans '{}'", layer.spec.name);
        }
    }

    pub fn undo_last_edit(&mut self) {
        if let Some((idx, prev_fc)) = self.undo_stack.pop() {
            if let Some(layer) = self.layers.get_mut(idx) {
                layer.data = prev_fc;
                layer.dirty = true;
                self.vp3d.invalidate_terrain();
                self.status_banner = format!("Dernière modification annulée ({})", layer.spec.name);
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
                style: sdnc_data::LayerStyle {
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
        self.vp3d.invalidate_terrain();
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

    /// Collecte tous les points 3D projetés dans le CRS actif (pour l'emprise et le MNT Vulkan Compute).
    pub fn collect_elevation_samples(&self) -> Vec<(f64, f64, f64, f32)> {
        let active_crs = self.project.active_crs;
        let mut pts = Vec::new();
        for layer in &self.layers {
            if !layer.spec.visible {
                continue;
            }
            for f in &layer.data.features {
                if let Some(g) = &f.geometry {
                    collect_geom_coords(g, layer.spec.crs, active_crs, &mut pts);
                }
            }
        }
        pts
    }
}

fn collect_geom_coords(
    g: &Geometry,
    src_crs: FrenchCrs,
    dst_crs: FrenchCrs,
    out: &mut Vec<(f64, f64, f64, f32)>,
) {
    let mut push_c = |c: &Coordinate| {
        let z = c.z.unwrap_or(240.0);
        let (x, y, z2) = if src_crs != dst_crs {
            reproject_point_3d(src_crs, dst_crs, c.x, c.y, z).unwrap_or((c.x, c.y, z))
        } else {
            (c.x, c.y, z)
        };
        if x.is_finite() && y.is_finite() && z2.is_finite() {
            out.push((x, y, z2, 5.0));
        }
    };
    match g {
        Geometry::Point { coordinates } => push_c(coordinates),
        Geometry::LineString { coordinates } | Geometry::MultiPoint { coordinates } => {
            for c in coordinates {
                push_c(c);
            }
        }
        Geometry::Polygon { coordinates } | Geometry::MultiLineString { coordinates } => {
            for ring in coordinates {
                for c in ring {
                    push_c(c);
                }
            }
        }
        Geometry::MultiPolygon { coordinates } => {
            for poly in coordinates {
                for ring in poly {
                    for c in ring {
                        push_c(c);
                    }
                }
            }
        }
        Geometry::GeometryCollection { geometries } => {
            for sub in geometries {
                collect_geom_coords(sub, src_crs, dst_crs, out);
            }
        }
    }
}
