//! Format de projet terrain natif léger SDNC (`.sdnc.json` + `.gpkg`).
//!
//! Remplace les projets XML `.qgs`/`.qgz` de QGIS par un manifeste JSON concis
//! associé à une base GeoPackage (`.gpkg`) pour les couches vectorielles éditables.

use anyhow::{Context, Result};
use sdnc_geo::{BasemapLayerType, FrenchCrs};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayerStyle {
    pub stroke_rgba: [u8; 4],
    pub fill_rgba: [u8; 4],
    pub stroke_width: f32,
    pub point_radius: f32,
    pub label_field: Option<String>,
}

impl Default for LayerStyle {
    fn default() -> Self {
        Self {
            stroke_rgba: [0, 220, 255, 255],
            fill_rgba: [0, 220, 255, 64],
            stroke_width: 2.0,
            point_radius: 5.5,
            label_field: Some("nom".to_string()),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FieldWidgetType {
    Text,
    Number,
    Choice(Vec<String>),
    Checkbox,
    DateTime,
    GnssAccuracy,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AttributeFieldSpec {
    pub name: String,
    pub label: String,
    pub widget: FieldWidgetType,
    pub required: bool,
    pub default_value: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LayerSource {
    GeoPackageTable { gpkg_rel_path: String, table_name: String },
    GeoJsonFile { rel_path: String },
    DxfFile { rel_path: String },
    EdigeoDir { rel_path: String },
    DemRaster { rel_path: String },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProjectLayerSpec {
    pub id: String,
    pub name: String,
    pub source: LayerSource,
    pub crs: FrenchCrs,
    pub visible: bool,
    pub editable: bool,
    pub opacity: f32,
    pub style: LayerStyle,
    pub form_fields: Vec<AttributeFieldSpec>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BasemapConfig {
    pub active_layers: Vec<BasemapLayerType>,
    pub opacity: f32,
    pub offline_mode: bool,
    pub tile_cache_rel_path: String,
}

impl Default for BasemapConfig {
    fn default() -> Self {
        Self {
            active_layers: vec![BasemapLayerType::IgnOrtho],
            opacity: 1.0,
            offline_mode: false,
            tile_cache_rel_path: "cache/tiles_cache.sqlite".to_string(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SdncProject {
    pub schema_version: u32,
    pub name: String,
    pub active_crs: FrenchCrs,
    pub gpkg_file: String,
    pub basemap: BasemapConfig,
    pub layers: Vec<ProjectLayerSpec>,
    pub center_xy: Option<(f64, f64)>,
    pub default_zoom: f32,
}

impl Default for SdncProject {
    fn default() -> Self {
        Self {
            schema_version: 1,
            name: "Chantier Terrain SDNC".to_string(),
            active_crs: FrenchCrs::Lambert93,
            gpkg_file: "terrain_data.gpkg".to_string(),
            basemap: BasemapConfig::default(),
            layers: vec![
                ProjectLayerSpec {
                    id: "points_rtk".to_string(),
                    name: "Points Relevés GNSS / RTK".to_string(),
                    source: LayerSource::GeoPackageTable {
                        gpkg_rel_path: "terrain_data.gpkg".to_string(),
                        table_name: "points_rtk".to_string(),
                    },
                    crs: FrenchCrs::Lambert93,
                    visible: true,
                    editable: true,
                    opacity: 1.0,
                    style: LayerStyle {
                        stroke_rgba: [0, 255, 140, 255],
                        fill_rgba: [0, 255, 140, 180],
                        stroke_width: 2.0,
                        point_radius: 6.0,
                        label_field: Some("nom".to_string()),
                    },
                    form_fields: default_field_form(),
                },
                ProjectLayerSpec {
                    id: "traces_lignes".to_string(),
                    name: "Lignes & Réseaux".to_string(),
                    source: LayerSource::GeoPackageTable {
                        gpkg_rel_path: "terrain_data.gpkg".to_string(),
                        table_name: "traces_lignes".to_string(),
                    },
                    crs: FrenchCrs::Lambert93,
                    visible: true,
                    editable: true,
                    opacity: 1.0,
                    style: LayerStyle {
                        stroke_rgba: [255, 190, 0, 255],
                        fill_rgba: [255, 190, 0, 80],
                        stroke_width: 2.5,
                        point_radius: 4.5,
                        label_field: Some("nom".to_string()),
                    },
                    form_fields: default_field_form(),
                },
                ProjectLayerSpec {
                    id: "zones_polygones".to_string(),
                    name: "Parcelles & Zones".to_string(),
                    source: LayerSource::GeoPackageTable {
                        gpkg_rel_path: "terrain_data.gpkg".to_string(),
                        table_name: "zones_polygones".to_string(),
                    },
                    crs: FrenchCrs::Lambert93,
                    visible: true,
                    editable: true,
                    opacity: 0.9,
                    style: LayerStyle {
                        stroke_rgba: [0, 200, 255, 255],
                        fill_rgba: [0, 180, 255, 55],
                        stroke_width: 2.0,
                        point_radius: 4.0,
                        label_field: Some("nom".to_string()),
                    },
                    form_fields: default_field_form(),
                },
            ],
            center_xy: Some((894500.0, 6269500.0)), // Provence / Aix-Meyreuil en Lambert-93
            default_zoom: 1.0,
        }
    }
}

pub fn default_field_form() -> Vec<AttributeFieldSpec> {
    vec![
        AttributeFieldSpec {
            name: "nom".to_string(),
            label: "Identifiant / Nom".to_string(),
            widget: FieldWidgetType::Text,
            required: true,
            default_value: None,
        },
        AttributeFieldSpec {
            name: "categorie".to_string(),
            label: "Catégorie".to_string(),
            widget: FieldWidgetType::Choice(vec![
                "Borne".to_string(),
                "GCP".to_string(),
                "Réseau".to_string(),
                "Limite".to_string(),
                "Observation".to_string(),
            ]),
            required: false,
            default_value: Some("Observation".to_string()),
        },
        AttributeFieldSpec {
            name: "commentaire".to_string(),
            label: "Commentaire terrain".to_string(),
            widget: FieldWidgetType::Text,
            required: false,
            default_value: None,
        },
        AttributeFieldSpec {
            name: "gnss_statut".to_string(),
            label: "Qualité GNSS".to_string(),
            widget: FieldWidgetType::GnssAccuracy,
            required: false,
            default_value: None,
        },
    ]
}

impl SdncProject {
    pub fn load<P: AsRef<Path>>(json_path: P) -> Result<Self> {
        let content = std::fs::read_to_string(json_path.as_ref())
            .with_context(|| format!("Lecture impossible du projet {:?}", json_path.as_ref()))?;
        let proj: Self = serde_json::from_str(&content)?;
        Ok(proj)
    }

    pub fn save<P: AsRef<Path>>(&self, json_path: P) -> Result<()> {
        if let Some(parent) = json_path.as_ref().parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent)?;
            }
        }
        let content = serde_json::to_string_pretty(self)?;
        std::fs::write(json_path.as_ref(), content)?;
        Ok(())
    }

    pub fn resolve_path(&self, project_dir: &Path, rel: &str) -> PathBuf {
        let p = Path::new(rel);
        if p.is_absolute() {
            p.to_path_buf()
        } else {
            project_dir.join(p)
        }
    }
}
