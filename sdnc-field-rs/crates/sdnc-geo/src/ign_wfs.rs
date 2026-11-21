//! Client de flux **IGN Géoplateforme Vecteur (WFS 2.0.0 GeoJSON)**.
//!
//! Permet de charger en direct (et d'enregistrer hors-ligne en GeoPackage) les couches
//! vectorielles officielles de la Géoplateforme IGN (`data.geopf.fr/wfs/ows`) :
//! - Cadastre PCI Vecteur : Parcelles, Bâtiments, Feuilles/Sections
//! - BD TOPO v3 : Bâtiments, Tronçons de route, Cours d'eau, Communes

use anyhow::Result;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum IgnWfsLayer {
    ParcellaireParcelles,
    ParcellaireBatiments,
    ParcellaireFeuilles,
    BdTopoBatiments,
    BdTopoRoutes,
    BdTopoHydro,
    BdTopoCommunes,
}

impl IgnWfsLayer {
    pub const ALL: &'static [IgnWfsLayer] = &[
        IgnWfsLayer::ParcellaireParcelles,
        IgnWfsLayer::ParcellaireBatiments,
        IgnWfsLayer::ParcellaireFeuilles,
        IgnWfsLayer::BdTopoBatiments,
        IgnWfsLayer::BdTopoRoutes,
        IgnWfsLayer::BdTopoHydro,
        IgnWfsLayer::BdTopoCommunes,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            Self::ParcellaireParcelles => "IGN WFS — Parcelles (PCI Express)",
            Self::ParcellaireBatiments => "IGN WFS — Bâtiments (PCI Express)",
            Self::ParcellaireFeuilles => "IGN WFS — Feuilles / Sections PCI",
            Self::BdTopoBatiments => "IGN WFS — BD TOPO Bâtiments",
            Self::BdTopoRoutes => "IGN WFS — BD TOPO Routes",
            Self::BdTopoHydro => "IGN WFS — BD TOPO Cours d'eau",
            Self::BdTopoCommunes => "IGN WFS — Limites Communes",
        }
    }

    pub fn type_name(&self) -> &'static str {
        match self {
            Self::ParcellaireParcelles => "CADASTRALPARCELS.PARCELLAIRE_EXPRESS:parcelle",
            Self::ParcellaireBatiments => "CADASTRALPARCELS.PARCELLAIRE_EXPRESS:batiment",
            Self::ParcellaireFeuilles => "CADASTRALPARCELS.PARCELLAIRE_EXPRESS:feuille",
            Self::BdTopoBatiments => "BDTOPO_V3:batiment",
            Self::BdTopoRoutes => "BDTOPO_V3:troncon_de_route",
            Self::BdTopoHydro => "BDTOPO_V3:cours_d_eau",
            Self::BdTopoCommunes => "BDTOPO_V3:commune",
        }
    }

    /// Style cartographique 2D DGFiP / IGN recommandé : `(stroke_rgba, fill_rgba, stroke_width, label_field)`.
    pub fn default_style(&self) -> ([u8; 4], [u8; 4], f32, Option<&'static str>) {
        match self {
            Self::ParcellaireParcelles => ([255, 215, 70, 235], [255, 215, 70, 22], 1.4, Some("numero")),
            Self::ParcellaireBatiments => ([215, 130, 55, 255], [235, 175, 85, 155], 1.5, None),
            Self::ParcellaireFeuilles => ([255, 80, 120, 240], [255, 80, 120, 14], 2.4, Some("section")),
            Self::BdTopoBatiments => ([235, 110, 70, 255], [235, 110, 70, 110], 1.3, Some("nature")),
            Self::BdTopoRoutes => ([230, 235, 245, 220], [0, 0, 0, 0], 2.2, Some("nom_1_gauche")),
            Self::BdTopoHydro => ([60, 175, 255, 240], [60, 175, 255, 65], 2.0, Some("toponyme")),
            Self::BdTopoCommunes => ([220, 100, 255, 230], [220, 100, 255, 12], 2.6, Some("nom_officiel")),
        }
    }
}

/// Construit une requête WFS 2.0.0 GeoJSON conforme Géoplateforme IGN pour une emprise WGS84
/// `(min_lon, min_lat, max_lon, max_lat)` (l'axe CRS:84 / EPSG:4326 dans WFS 2.0 IGN attend `lat,lon`
/// si `EPSG:4326` ou `lon,lat` avec `CRS:84` — on utilise explicitement `CRS:84`).
pub fn build_ign_wfs_url(
    layer: IgnWfsLayer,
    min_lon: f64,
    min_lat: f64,
    max_lon: f64,
    max_lat: f64,
    max_features: usize,
) -> String {
    format!(
        "https://data.geopf.fr/wfs/ows?SERVICE=WFS&VERSION=2.0.0&REQUEST=GetFeature&TYPENAMES={}&OUTPUTFORMAT=application/json&SRSNAME=CRS:84&BBOX={:.6},{:.6},{:.6},{:.6},CRS:84&COUNT={}",
        layer.type_name(),
        min_lon,
        min_lat,
        max_lon,
        max_lat,
        max_features.clamp(10, 5000)
    )
}

/// Télécharge les entités GeoJSON d'une couche vecteur IGN WFS sur l'emprise WGS84 courante.
pub fn fetch_ign_wfs_geojson(
    layer: IgnWfsLayer,
    min_lon: f64,
    min_lat: f64,
    max_lon: f64,
    max_lat: f64,
    max_features: usize,
) -> Result<String> {
    let url = build_ign_wfs_url(layer, min_lon, min_lat, max_lon, max_lat, max_features);
    let resp = ureq::get(&url)
        .set("User-Agent", "SDNCQField-RS/0.1 (IGN WFS Client)")
        .timeout(std::time::Duration::from_secs(12))
        .call()?;
    let body = resp.into_string()?;
    if !body.contains("FeatureCollection") {
        anyhow::bail!("Réponse WFS IGN inattendue (pas de FeatureCollection)");
    }
    Ok(body)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ign_wfs_url_construction() {
        let url = build_ign_wfs_url(
            IgnWfsLayer::ParcellaireParcelles,
            5.44,
            43.52,
            5.46,
            43.53,
            1000,
        );
        assert!(url.contains("data.geopf.fr/wfs/ows"));
        assert!(url.contains("CADASTRALPARCELS.PARCELLAIRE_EXPRESS:parcelle"));
        assert!(url.contains("CRS:84"));
        assert!(url.contains("COUNT=1000"));
    }
}
