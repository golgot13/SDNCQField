//! `sdnc-data` — Couche de données SIG terrain & format de projet natif (`.sdnc.json` + `.gpkg`).
//!
//! Remplace GDAL/OGR et les projets `.qgs`/`.qgz` de QGIS par :
//! - Un provider GeoPackage OGC lecture/écriture complet (`rusqlite` bundled)
//! - Un format de projet JSON léger (couches, styles, formulaires terrain, CRS, fond de carte)
//! - Les parsers GeoJSON, DXF, EDIGÉO et MNT (ASC / GeoTIFF) adaptés d'`AEROPHOTOGRAM`

pub mod cadastre;
pub mod dem;
pub mod dxf;
pub mod edigeo;
pub mod geopackage;
pub mod project;
pub mod vector;

pub use cadastre::{
    cadastral_style_for_layer, detect_dxf_crs, detect_edigeo_crs, generate_demo_cadastral_layers,
    load_dxf_cadastral_layers, load_edigeo_cadastral_layers, parse_crs_hint,
};
pub use dem::DemReader;
pub use dxf::{aci_to_rgb, DxfDocument, DxfParser, DxfWriter};
pub use edigeo::{EdigeoDocument, EdigeoParser, EdigeoWriter};
pub use geopackage::{decode_gpkg_binary, encode_gpkg_binary, GeoPackage, GpkgLayerMeta};
pub use project::{
    default_field_form, AttributeFieldSpec, BasemapConfig, FieldWidgetType, LayerSource,
    LayerStyle, ProjectLayerSpec, SdncProject,
};
pub use vector::{Coordinate, Feature, FeatureCollection, Geometry, GeometryType, PropertyValue};
