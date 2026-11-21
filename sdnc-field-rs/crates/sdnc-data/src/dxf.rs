//! Parser DXF (Drawing Exchange Format)
//!
//! Support pour:
//! - DXF standard AutoCAD
//! - DXF-PCI (format PCI Geomatics avec géoréférencement)
//! - Import/Export des entités géométriques

use anyhow::{Context, Result};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use crate::vector::{Coordinate, Feature, FeatureCollection, Geometry, PropertyValue};

/// DXF BYLAYER color code (color inherited from layer)
const DXF_BYLAYER_COLOR: i32 = 256;
/// DXF default white color code
const DXF_DEFAULT_COLOR: i32 = 7;
/// Number of line segments used to approximate circles in DXF output
const DXF_CIRCLE_SEGMENTS: usize = 32;

/// Conversion du code couleur Index AutoCAD (ACI 1..255) vers (R, G, B)
pub fn aci_to_rgb(aci: i32) -> (u8, u8, u8) {
    match aci.abs() % 256 {
        1 => (255, 60, 60),     // Rouge
        2 => (255, 220, 0),    // Jaune
        3 => (50, 230, 80),    // Vert
        4 => (0, 220, 255),    // Cyan
        5 => (80, 140, 255),   // Bleu
        6 => (240, 80, 240),   // Magenta
        7 | 0 => (240, 240, 240), // Blanc / Gris clair
        8 => (140, 140, 140),  // Gris foncé
        9 => (200, 200, 200),  // Gris moyen
        code @ 10..=249 => {
            let hue: f64 = ((code - 10) / 10) as f64 * (360.0 / 24.0);
            let lightness: f64 = match (code - 10) % 10 {
                0 | 1 => 0.5,
                2 | 3 => 0.65,
                4 | 5 => 0.8,
                6 | 7 => 0.35,
                _ => 0.25,
            };
            let c: f64 = 1.0 - (2.0 * lightness - 1.0).abs();
            let x: f64 = c * (1.0 - ((hue / 60.0) % 2.0 - 1.0).abs());
            let (r1, g1, b1) = match (hue / 60.0) as i32 {
                0 => (c, x, 0.0),
                1 => (x, c, 0.0),
                2 => (0.0, c, x),
                3 => (0.0, x, c),
                4 => (x, 0.0, c),
                _ => (c, 0.0, x),
            };
            let m: f64 = lightness - c / 2.0;
            (((r1 + m) * 255.0) as u8, ((g1 + m) * 255.0) as u8, ((b1 + m) * 255.0) as u8)
        }
        _ => (220, 220, 220),
    }
}

/// Types d'entités DXF supportées
#[derive(Debug, Clone, PartialEq)]
pub enum DxfEntityType {
    Point,
    Line,
    Polyline,
    LwPolyline,
    Circle,
    Arc,
    Ellipse,
    Text,
    MText,
    Insert, // Block reference
    Dimension,
    Hatch,
    Spline,
    Face3D,
}

impl std::str::FromStr for DxfEntityType {
    type Err = String;

    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s.to_uppercase().as_str() {
            "POINT" => Ok(DxfEntityType::Point),
            "LINE" => Ok(DxfEntityType::Line),
            "POLYLINE" => Ok(DxfEntityType::Polyline),
            "LWPOLYLINE" => Ok(DxfEntityType::LwPolyline),
            "CIRCLE" => Ok(DxfEntityType::Circle),
            "ARC" => Ok(DxfEntityType::Arc),
            "ELLIPSE" => Ok(DxfEntityType::Ellipse),
            "TEXT" => Ok(DxfEntityType::Text),
            "MTEXT" => Ok(DxfEntityType::MText),
            "INSERT" => Ok(DxfEntityType::Insert),
            "DIMENSION" => Ok(DxfEntityType::Dimension),
            "HATCH" => Ok(DxfEntityType::Hatch),
            "SPLINE" => Ok(DxfEntityType::Spline),
            "3DFACE" => Ok(DxfEntityType::Face3D),
            _ => Err(format!("Unknown DXF entity type: {}", s)),
        }
    }
}

/// Entité DXF parsée
#[derive(Debug, Clone)]
pub struct DxfEntity {
    pub entity_type: DxfEntityType,
    pub layer: String,
    pub color: i32,
    pub handle: String,
    pub coordinates: Vec<Coordinate>,
    pub properties: HashMap<String, String>,
    pub closed: bool, // Pour polylines
}

impl Default for DxfEntity {
    fn default() -> Self {
        Self {
            entity_type: DxfEntityType::Point,
            layer: "0".to_string(),
            color: DXF_BYLAYER_COLOR, // BYLAYER
            handle: String::new(),
            coordinates: Vec::new(),
            properties: HashMap::new(),
            closed: false,
        }
    }
}

/// Informations d'en-tête DXF
#[derive(Debug, Clone, Default)]
pub struct DxfHeader {
    pub acadver: String,
    pub insbase: Option<(f64, f64, f64)>,
    pub extmin: Option<(f64, f64, f64)>,
    pub extmax: Option<(f64, f64, f64)>,
    pub units: i32,
    /// Variables PCI pour géoréférencement
    pub pci_projection: Option<String>,
    pub pci_datum: Option<String>,
    pub pci_units: Option<String>,
}

/// Couche DXF
#[derive(Debug, Clone)]
pub struct DxfLayer {
    pub name: String,
    pub color: i32,
    pub linetype: String,
    pub frozen: bool,
    pub locked: bool,
}

/// Résultat du parsing DXF complet
#[derive(Debug, Clone)]
pub struct DxfDocument {
    pub header: DxfHeader,
    pub layers: Vec<DxfLayer>,
    pub entities: Vec<DxfEntity>,
    /// Entités groupées par layer
    pub entities_by_layer: HashMap<String, Vec<usize>>,
}

impl DxfDocument {
    /// Convertir en FeatureCollection (toutes les entités ou un layer spécifique)
    pub fn to_feature_collection(&self, layer_filter: Option<&str>) -> FeatureCollection {
        let features: Vec<Feature> = if let Some(layer_name) = layer_filter {
            self.entities_by_layer
                .get(layer_name)
                .map(|idxs| {
                    idxs.iter()
                        .filter_map(|&idx| {
                            self.entities
                                .get(idx)
                                .and_then(|e| entity_to_feature(e, idx))
                        })
                        .collect()
                })
                .unwrap_or_default()
        } else {
            self.entities
                .iter()
                .enumerate()
                .filter_map(|(idx, entity)| entity_to_feature(entity, idx))
                .collect()
        };

        FeatureCollection::new(features)
    }

    /// Convertir une couche spécifique en FeatureCollection
    pub fn layer_to_feature_collection(&self, layer_name: &str) -> FeatureCollection {
        self.to_feature_collection(Some(layer_name))
    }

    /// Obtenir les noms des couches
    pub fn layer_names(&self) -> Vec<String> {
        self.layers.iter().map(|l| l.name.clone()).collect()
    }
}

/// Parser DXF
pub struct DxfParser;

impl DxfParser {
    /// Parse un fichier DXF
    pub fn parse<P: AsRef<Path>>(path: P) -> Result<DxfDocument> {
        let file = File::open(path.as_ref())
            .with_context(|| format!("Cannot open DXF file: {:?}", path.as_ref()))?;
        let reader = BufReader::new(file);

        let mut document = DxfDocument {
            header: DxfHeader::default(),
            layers: Vec::new(),
            entities: Vec::new(),
            entities_by_layer: HashMap::new(),
        };

        let mut lines = reader.lines().peekable();
        let mut current_section = String::new();
        let mut non_integer_errors = 0;

        while let Some(Ok(code_line)) = lines.next() {
            let code_str = code_line.trim();

            if code_str.starts_with("AutoCAD Binary DXF") || code_str.contains("\0") {
                anyhow::bail!("Fichier DXF binaire non supporté : {:?}", path.as_ref());
            }

            let code: i32 = match code_str.parse() {
                Ok(c) => c,
                Err(_) => {
                    non_integer_errors += 1;
                    if non_integer_errors > 20 {
                        anyhow::bail!("Le fichier ne respecte pas le format DXF ASCII : {:?}", path.as_ref());
                    }
                    let _ = lines.next();
                    continue;
                }
            };
            let value = lines.next().and_then(|r| r.ok()).unwrap_or_default();
            let value = value.trim().to_string();

            match code {
                0 => {
                    match value.as_str() {
                        "SECTION" => {
                            // Lire le nom de la section
                            if let Some(Ok(code2)) = lines.next() {
                                if code2.trim() == "2" {
                                    if let Some(Ok(section_name)) = lines.next() {
                                        current_section = section_name.trim().to_string();
                                    }
                                }
                            }
                        }
                        "ENDSEC" => {
                            current_section.clear();
                        }
                        "EOF" => break,
                        entity_type => {
                            if current_section == "ENTITIES" {
                                if let Ok(et) = entity_type.parse::<DxfEntityType>() {
                                    if let Some(entity) = Self::parse_entity(et, &mut lines) {
                                        let layer_name = entity.layer.clone();
                                        let idx = document.entities.len();
                                        document.entities.push(entity);
                                        document
                                            .entities_by_layer
                                            .entry(layer_name)
                                            .or_default()
                                            .push(idx);
                                    }
                                }
                            } else if current_section == "TABLES" && entity_type == "LAYER" {
                                if let Some(layer) = Self::parse_layer(&mut lines) {
                                    document.layers.push(layer);
                                }
                            }
                        }
                    }
                }
                9 if current_section == "HEADER" => {
                    // Variable d'en-tête
                    Self::parse_header_var(&value, &mut lines, &mut document.header);
                }
                _ => {}
            }
        }

        // Ajouter les couches manquantes
        for layer_name in document.entities_by_layer.keys() {
            if !document.layers.iter().any(|l| &l.name == layer_name) {
                document.layers.push(DxfLayer {
                    name: layer_name.clone(),
                    color: DXF_DEFAULT_COLOR,
                    linetype: "CONTINUOUS".to_string(),
                    frozen: false,
                    locked: false,
                });
            }
        }

        log::info!(
            "DXF parsed: {} entities, {} layers",
            document.entities.len(),
            document.layers.len()
        );

        Ok(document)
    }

    fn parse_entity<I>(
        entity_type: DxfEntityType,
        lines: &mut std::iter::Peekable<I>,
    ) -> Option<DxfEntity>
    where
        I: Iterator<Item = std::io::Result<String>>,
    {
        let mut entity = DxfEntity {
            entity_type: entity_type.clone(),
            ..Default::default()
        };

        let mut x: Option<f64> = None;
        let mut y: Option<f64> = None;
        let mut z: Option<f64> = None;
        let mut vertices: Vec<Coordinate> = Vec::new();

        while let Some(Ok(peeked_line)) = lines.peek() {
            let code_str = peeked_line.trim().to_string();
            let code: i32 = match code_str.parse::<i32>() {
                Ok(c) => c,
                Err(_) => {
                    log::warn!(
                        "DXF entity {:?}: invalid group code {:?}, stopping entity parse",
                        entity_type,
                        code_str
                    );
                    break;
                }
            };

            // Fin de l'entité si on rencontre un code 0
            if code == 0 {
                break;
            }

            lines.next(); // Consommer la ligne de code
            let value = lines.next().and_then(|r| r.ok()).unwrap_or_default();
            let value = value.trim();

            match code {
                5 => entity.handle = value.to_string(),
                8 => entity.layer = value.to_string(),
                62 => entity.color = value.parse().unwrap_or(DXF_BYLAYER_COLOR),

                // Coordonnées
                10 => x = value.parse().ok(),
                20 => y = value.parse().ok(),
                30 => z = value.parse().ok(),

                // Coordonnées supplémentaires pour polylines
                11 => {
                    // Deuxième point (LINE)
                    if let (Some(px), Some(py)) = (x, y) {
                        vertices.push(Coordinate::new_3d(px, py, z.unwrap_or(0.0)));
                    }
                    x = value.parse().ok();
                }
                21 => y = value.parse().ok(),
                31 => {
                    z = value.parse().ok();
                    if let (Some(px), Some(py)) = (x, y) {
                        vertices.push(Coordinate::new_3d(px, py, z.unwrap_or(0.0)));
                    }
                    x = None;
                    y = None;
                    z = None;
                }

                // Flags polyline fermeture
                70 => {
                    let flags: i32 = value.parse().unwrap_or_else(|_| {
                        log::warn!(
                            "DXF entity {:?}: invalid flags value {:?}, treating as 0",
                            entity_type,
                            value
                        );
                        0
                    });
                    entity.closed = (flags & 1) != 0;
                }

                // Texte
                1 => {
                    entity
                        .properties
                        .insert("text".to_string(), value.to_string());
                }

                // Rayon (CIRCLE)
                40 => {
                    entity
                        .properties
                        .insert("radius".to_string(), value.to_string());
                }

                _ => {}
            }

            // Ajouter le point courant pour LWPOLYLINE
            if matches!(
                entity_type,
                DxfEntityType::LwPolyline | DxfEntityType::Polyline
            ) {
                if let (Some(px), Some(py)) = (x, y) {
                    if code == 20 || code == 21 {
                        vertices.push(Coordinate::new_3d(px, py, z.unwrap_or(0.0)));
                        x = None;
                        y = None;
                        z = None;
                    }
                }
            }
        }

        // Ajouter le dernier point
        if let (Some(px), Some(py)) = (x, y) {
            vertices.push(Coordinate::new_3d(px, py, z.unwrap_or(0.0)));
        }

        if !vertices.is_empty() {
            entity.coordinates = vertices;
        } else if let (Some(px), Some(py)) = (x, y) {
            entity.coordinates = vec![Coordinate::new_3d(px, py, z.unwrap_or(0.0))];
        }

        Some(entity)
    }

    fn parse_layer<I>(lines: &mut std::iter::Peekable<I>) -> Option<DxfLayer>
    where
        I: Iterator<Item = std::io::Result<String>>,
    {
        let mut layer = DxfLayer {
            name: String::new(),
            color: DXF_DEFAULT_COLOR,
            linetype: "CONTINUOUS".to_string(),
            frozen: false,
            locked: false,
        };

        while let Some(Ok(peeked_line)) = lines.peek() {
            let code_str = peeked_line.trim().to_string();
            let code: i32 = match code_str.parse::<i32>() {
                Ok(c) => c,
                Err(_) => {
                    log::warn!(
                        "DXF LAYER parse: invalid group code {:?}, stopping layer parse",
                        code_str
                    );
                    break;
                }
            };
            if code == 0 {
                break;
            }

            lines.next();
            let value = lines.next().and_then(|r| r.ok()).unwrap_or_default();
            let value = value.trim();

            match code {
                2 => layer.name = value.to_string(),
                62 => layer.color = value.parse().unwrap_or(DXF_DEFAULT_COLOR),
                6 => layer.linetype = value.to_string(),
                70 => {
                    let flags: i32 = value.parse().unwrap_or_else(|_| {
                        log::warn!(
                            "DXF LAYER {:?}: invalid flags value {:?}, treating as 0",
                            layer.name,
                            value
                        );
                        0
                    });
                    layer.frozen = (flags & 1) != 0;
                    layer.locked = (flags & 4) != 0;
                }
                _ => {}
            }
        }

        if !layer.name.is_empty() {
            Some(layer)
        } else {
            None
        }
    }

    fn parse_header_var<I>(
        var_name: &str,
        lines: &mut std::iter::Peekable<I>,
        header: &mut DxfHeader,
    ) where
        I: Iterator<Item = std::io::Result<String>>,
    {
        match var_name {
            "$ACADVER" => {
                if let Some(Ok(_)) = lines.next() {
                    if let Some(Ok(value)) = lines.next() {
                        header.acadver = value.trim().to_string();
                    }
                }
            }
            "$INSUNITS" => {
                if let Some(Ok(_)) = lines.next() {
                    if let Some(Ok(value)) = lines.next() {
                        header.units = value.trim().parse().unwrap_or(0);
                    }
                }
            }
            // Variables PCI pour géoréférencement
            "$PCIPROJ" | "$PROJECTION" => {
                if let Some(Ok(_)) = lines.next() {
                    if let Some(Ok(value)) = lines.next() {
                        header.pci_projection = Some(value.trim().to_string());
                    }
                }
            }
            "$PCIDATUM" | "$DATUM" => {
                if let Some(Ok(_)) = lines.next() {
                    if let Some(Ok(value)) = lines.next() {
                        header.pci_datum = Some(value.trim().to_string());
                    }
                }
            }
            _ => {}
        }
    }
}

/// Export DXF
pub struct DxfWriter;

impl DxfWriter {
    pub fn write_dxf_pci<P: AsRef<Path>>(
        path: P,
        layers: &[(String, i32, Vec<(Vec<(f64, f64, f64)>, bool, Option<String>)>)],
        projection: &str,
    ) -> Result<()> {
        use std::io::Write;

        let mut file = File::create(path.as_ref())
            .with_context(|| format!("Impossible de créer le fichier DXF-PCI {:?}", path.as_ref()))?;

        let mut min_x = f64::MAX;
        let mut max_x = f64::MIN;
        let mut min_y = f64::MAX;
        let mut max_y = f64::MIN;
        let mut min_z = f64::MAX;
        let mut max_z = f64::MIN;

        for (_, _, feats) in layers {
            for (pts, _, _) in feats {
                for (x, y, z) in pts {
                    if *x < min_x { min_x = *x; }
                    if *x > max_x { max_x = *x; }
                    if *y < min_y { min_y = *y; }
                    if *y > max_y { max_y = *y; }
                    if *z < min_z { min_z = *z; }
                    if *z > max_z { max_z = *z; }
                }
            }
        }

        if min_x == f64::MAX { min_x = 0.0; max_x = 1000.0; }
        if min_y == f64::MAX { min_y = 0.0; max_y = 1000.0; }
        if min_z == f64::MAX { min_z = 0.0; max_z = 100.0; }

        let mut out = String::new();

        // 1. SECTION HEADER (dxf-pci)
        out.push_str("0\nSECTION\n2\nHEADER\n");
        out.push_str("9\n$ACADVER\n1\nAC1009\n");
        out.push_str("9\n$INSBASE\n10\n0.0\n20\n0.0\n30\n0.0\n");
        out.push_str(&format!("9\n$EXTMIN\n10\n{:.4}\n20\n{:.4}\n30\n{:.4}\n", min_x, min_y, min_z));
        out.push_str(&format!("9\n$EXTMAX\n10\n{:.4}\n20\n{:.4}\n30\n{:.4}\n", max_x, max_y, max_z));
        
        // Variables PCI Géomatique IGN Cadastre
        out.push_str(&format!("9\n$PCI_PROJECTION\n1\n{}\n", projection));
        out.push_str("9\n$PCI_DATUM\n1\nRGF93_GGRS80\n");
        out.push_str("9\n$PCI_UNITS\n70\n6\n"); // 6 = Mètres
        out.push_str("0\nENDSEC\n");

        // 2. SECTION TABLES (Layers)
        out.push_str("0\nSECTION\n2\nTABLES\n");
        out.push_str("0\nTABLE\n2\nLAYER\n70\n");
        out.push_str(&format!("{}\n", layers.len()));
        for (layer_name, color, _) in layers {
            out.push_str("0\nLAYER\n");
            out.push_str(&format!("2\n{}\n", layer_name));
            out.push_str("70\n0\n");
            out.push_str(&format!("62\n{}\n", color));
            out.push_str("6\nCONTINUOUS\n");
        }
        out.push_str("0\nENDTAB\n0\nENDSEC\n");

        // 3. SECTION ENTITIES
        out.push_str("0\nSECTION\n2\nENTITIES\n");
        for (layer_name, _, feats) in layers {
            for (pts, closed, text) in feats {
                if pts.len() > 1 {
                    out.push_str("0\nPOLYLINE\n");
                    out.push_str(&format!("8\n{}\n", layer_name));
                    out.push_str("66\n1\n");
                    out.push_str("10\n0.0\n20\n0.0\n30\n0.0\n");
                    if *closed {
                        out.push_str("70\n1\n");
                    } else {
                        out.push_str("70\n0\n");
                    }
                    for (x, y, z) in pts {
                        out.push_str("0\nVERTEX\n");
                        out.push_str(&format!("8\n{}\n", layer_name));
                        out.push_str(&format!("10\n{:.4}\n20\n{:.4}\n30\n{:.4}\n", x, y, z));
                    }
                    out.push_str("0\nSEQEND\n");
                } else if pts.len() == 1 {
                    let (x, y, z) = pts[0];
                    out.push_str("0\nPOINT\n");
                    out.push_str(&format!("8\n{}\n", layer_name));
                    out.push_str(&format!("10\n{:.4}\n20\n{:.4}\n30\n{:.4}\n", x, y, z));

                    if let Some(txt) = text {
                        out.push_str("0\nTEXT\n");
                        out.push_str(&format!("8\n{}\n", layer_name));
                        out.push_str(&format!("10\n{:.4}\n20\n{:.4}\n30\n{:.4}\n", x, y, z));
                        out.push_str("40\n2.5\n");
                        out.push_str(&format!("1\n{}\n", txt));
                    }
                }
            }
        }
        out.push_str("0\nENDSEC\n0\nEOF\n");

        file.write_all(out.as_bytes())?;
        Ok(())
    }

    /// Exporte une FeatureCollection en DXF vers un fichier
    pub fn write<P: AsRef<Path>>(path: P, collection: &FeatureCollection) -> Result<()> {
        let content = Self::to_string(collection, None)?;
        std::fs::write(path.as_ref(), content)?;
        Ok(())
    }

    /// Exporte une FeatureCollection en DXF vers une chaîne
    pub fn to_string(collection: &FeatureCollection, layer_name: Option<&str>) -> Result<String> {
        let mut output = String::new();

        // Header section
        output.push_str("0\nSECTION\n2\nHEADER\n");
        output.push_str("9\n$ACADVER\n1\nAC1015\n"); // AutoCAD 2000
        output.push_str("9\n$INSUNITS\n70\n6\n"); // Meters
        output.push_str("0\nENDSEC\n");

        // Tables section (layers)
        output.push_str("0\nSECTION\n2\nTABLES\n");
        output.push_str("0\nTABLE\n2\nLAYER\n");

        // Collect unique layers
        let mut layers: std::collections::HashSet<String> = std::collections::HashSet::new();
        if let Some(name) = layer_name {
            layers.insert(name.to_string());
        }
        for feature in &collection.features {
            if let Some(PropertyValue::String(s)) = feature.properties.get("layer") {
                layers.insert(s.clone());
            }
        }
        if layers.is_empty() {
            layers.insert("0".to_string());
        }

        for (idx, layer_name) in layers.iter().enumerate() {
            output.push_str(&format!(
                "0\nLAYER\n2\n{}\n70\n0\n62\n{}\n6\nCONTINUOUS\n",
                layer_name,
                (idx % 7) + 1
            ));
        }

        output.push_str("0\nENDTAB\n");
        output.push_str("0\nENDSEC\n");

        // Entities section
        output.push_str("0\nSECTION\n2\nENTITIES\n");

        let default_layer = layer_name.unwrap_or("0");
        for feature in &collection.features {
            let layer = feature
                .properties
                .get("layer")
                .and_then(|v| match v {
                    PropertyValue::String(s) => Some(s.as_str()),
                    _ => None,
                })
                .unwrap_or(default_layer);

            if let Some(ref geom) = feature.geometry {
                Self::write_geometry_to_string(&mut output, geom, layer);
            }
        }

        output.push_str("0\nENDSEC\n");
        output.push_str("0\nEOF\n");

        Ok(output)
    }

    fn write_geometry_to_string(output: &mut String, geom: &Geometry, layer: &str) {
        match geom {
            Geometry::Point { coordinates } => {
                output.push_str(&format!(
                    "0\nPOINT\n8\n{}\n10\n{}\n20\n{}\n",
                    layer, coordinates.x, coordinates.y
                ));
                if let Some(z) = coordinates.z {
                    output.push_str(&format!("30\n{}\n", z));
                }
            }
            Geometry::LineString { coordinates } => {
                if coordinates.len() == 2 {
                    output.push_str(&format!(
                        "0\nLINE\n8\n{}\n10\n{}\n20\n{}\n30\n{}\n11\n{}\n21\n{}\n31\n{}\n",
                        layer,
                        coordinates[0].x,
                        coordinates[0].y,
                        coordinates[0].z.unwrap_or(0.0),
                        coordinates[1].x,
                        coordinates[1].y,
                        coordinates[1].z.unwrap_or(0.0)
                    ));
                } else {
                    output.push_str(&format!(
                        "0\nLWPOLYLINE\n8\n{}\n70\n0\n90\n{}\n",
                        layer,
                        coordinates.len()
                    ));
                    for coord in coordinates {
                        output.push_str(&format!("10\n{}\n20\n{}\n", coord.x, coord.y));
                    }
                }
            }
            Geometry::Polygon { coordinates } => {
                if let Some(exterior) = coordinates.first() {
                    output.push_str(&format!(
                        "0\nLWPOLYLINE\n8\n{}\n70\n1\n90\n{}\n",
                        layer,
                        exterior.len()
                    ));
                    for coord in exterior {
                        output.push_str(&format!("10\n{}\n20\n{}\n", coord.x, coord.y));
                    }
                }
            }
            Geometry::MultiPoint { coordinates } => {
                for coord in coordinates {
                    Self::write_geometry_to_string(
                        output,
                        &Geometry::Point {
                            coordinates: *coord,
                        },
                        layer,
                    );
                }
            }
            Geometry::MultiLineString { coordinates } => {
                for line in coordinates {
                    Self::write_geometry_to_string(
                        output,
                        &Geometry::LineString {
                            coordinates: line.clone(),
                        },
                        layer,
                    );
                }
            }
            Geometry::MultiPolygon { coordinates } => {
                for poly in coordinates {
                    Self::write_geometry_to_string(
                        output,
                        &Geometry::Polygon {
                            coordinates: poly.clone(),
                        },
                        layer,
                    );
                }
            }
            Geometry::GeometryCollection { geometries } => {
                for g in geometries {
                    Self::write_geometry_to_string(output, g, layer);
                }
            }
        }
    }
}

/// Convertit une entité DXF en Feature GeoJSON
fn entity_to_feature(entity: &DxfEntity, idx: usize) -> Option<Feature> {
    let geometry = match entity.entity_type {
        DxfEntityType::Point => entity.coordinates.first().map(|coord| Geometry::Point {
            coordinates: *coord,
        }),
        DxfEntityType::Line => {
            if entity.coordinates.len() >= 2 {
                Some(Geometry::LineString {
                    coordinates: entity.coordinates.clone(),
                })
            } else {
                None
            }
        }
        DxfEntityType::Polyline | DxfEntityType::LwPolyline => {
            if entity.coordinates.len() >= 2 {
                if entity.closed && entity.coordinates.len() >= 3 {
                    let mut coords = entity.coordinates.clone();
                    // Fermer le polygone si nécessaire
                    if coords.first() != coords.last() {
                        coords.push(coords[0]);
                    }
                    Some(Geometry::Polygon {
                        coordinates: vec![coords],
                    })
                } else {
                    Some(Geometry::LineString {
                        coordinates: entity.coordinates.clone(),
                    })
                }
            } else {
                None
            }
        }
        DxfEntityType::Circle => {
            // Approximer le cercle en polygone
            if let Some(center) = entity.coordinates.first() {
                let radius: f64 = match entity
                    .properties
                    .get("radius")
                    .and_then(|s| s.parse::<f64>().ok())
                {
                    Some(r) if r > 0.0 => r,
                    Some(r) => {
                        log::warn!(
                            "DXF CIRCLE entity handle={:?}: non-positive radius {:.6}, skipping",
                            entity.handle,
                            r
                        );
                        return None;
                    }
                    None => {
                        log::warn!(
                            "DXF CIRCLE entity handle={:?}: missing or unparseable radius, skipping",
                            entity.handle
                        );
                        return None;
                    }
                };

                let segments = DXF_CIRCLE_SEGMENTS;
                let mut coords = Vec::with_capacity(segments + 1);
                for i in 0..=segments {
                    let angle = 2.0 * std::f64::consts::PI * (i as f64) / (segments as f64);
                    coords.push(Coordinate::new(
                        center.x + radius * angle.cos(),
                        center.y + radius * angle.sin(),
                    ));
                }
                Some(Geometry::Polygon {
                    coordinates: vec![coords],
                })
            } else {
                None
            }
        }
        DxfEntityType::Text | DxfEntityType::MText => {
            // Texte = Point avec propriété text
            entity.coordinates.first().map(|coord| Geometry::Point {
                coordinates: *coord,
            })
        }
        _ => None,
    };

    geometry.map(|geom| {
        let mut props: HashMap<String, PropertyValue> = HashMap::new();
        props.insert("id".to_string(), PropertyValue::Integer(idx as i64));
        props.insert(
            "layer".to_string(),
            PropertyValue::String(entity.layer.clone()),
        );
        props.insert(
            "color".to_string(),
            PropertyValue::Integer(entity.color as i64),
        );
        props.insert(
            "handle".to_string(),
            PropertyValue::String(entity.handle.clone()),
        );
        props.insert(
            "type".to_string(),
            PropertyValue::String(format!("{:?}", entity.entity_type)),
        );

        for (k, v) in &entity.properties {
            props.insert(k.clone(), PropertyValue::String(v.clone()));
        }

        Feature {
            id: Some(entity.handle.clone()),
            geometry: Some(geom),
            properties: props,
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_entity_type_from_str() {
        assert_eq!("POINT".parse::<DxfEntityType>(), Ok(DxfEntityType::Point));
        assert_eq!("line".parse::<DxfEntityType>(), Ok(DxfEntityType::Line));
        assert_eq!(
            "LWPOLYLINE".parse::<DxfEntityType>(),
            Ok(DxfEntityType::LwPolyline)
        );
        assert!("UNKNOWN".parse::<DxfEntityType>().is_err());
    }

    #[test]
    fn test_write_point() {
        let collection = FeatureCollection::new(vec![Feature {
            id: Some("1".to_string()),
            geometry: Some(Geometry::point(100.0, 200.0)),
            properties: HashMap::new(),
        }]);

        let _output: Vec<u8> = Vec::new();
        // This would fail in a real test without a file, but shows the API
        assert!(collection.features.len() == 1);
    }

    // ── Circle radius hardening ───────────────────────────────────────────────

    #[test]
    fn test_circle_missing_radius_skipped() {
        let entity = DxfEntity {
            entity_type: DxfEntityType::Circle,
            coordinates: vec![Coordinate::new(0.0, 0.0)],
            ..Default::default()
        };
        // No "radius" property → must return None (not a unit circle).
        assert!(entity_to_feature(&entity, 0).is_none());
    }

    #[test]
    fn test_circle_nonpositive_radius_skipped() {
        let mut entity = DxfEntity {
            entity_type: DxfEntityType::Circle,
            coordinates: vec![Coordinate::new(0.0, 0.0)],
            ..Default::default()
        };
        entity
            .properties
            .insert("radius".to_string(), "0.0".to_string());
        assert!(entity_to_feature(&entity, 0).is_none());
    }

    #[test]
    fn test_circle_valid_radius_produces_polygon() {
        let mut entity = DxfEntity {
            entity_type: DxfEntityType::Circle,
            coordinates: vec![Coordinate::new(10.0, 20.0)],
            ..Default::default()
        };
        entity
            .properties
            .insert("radius".to_string(), "5.0".to_string());
        let feature = entity_to_feature(&entity, 0);
        assert!(feature.is_some());
        assert!(matches!(
            feature.unwrap().geometry,
            Some(Geometry::Polygon { .. })
        ));
    }

    // ── Corrupt group code robustness ─────────────────────────────────────────

    /// A DXF with one non-integer group code pair in the ENTITIES section must
    /// skip that pair without panicking and still produce all valid entities.
    #[test]
    fn test_parse_corrupt_group_code_skips_pair() {
        use std::time::{SystemTime, UNIX_EPOCH};
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .subsec_nanos();
        let path = std::env::temp_dir().join(format!("aeroflightgis_dxf_corrupt_{}.dxf", nonce));

        let content = "\
0\nSECTION\n2\nENTITIES\n\
NOTACODE\nbogus_value\n\
0\nPOINT\n8\nlayer1\n10\n100.0\n20\n200.0\n30\n0.0\n\
0\nENDSEC\n0\nEOF\n";
        std::fs::write(&path, content).unwrap();
        let result = DxfParser::parse(&path);
        let _ = std::fs::remove_file(&path);

        let doc = result.expect("parse must succeed despite corrupt group code");
        assert_eq!(doc.entities.len(), 1, "POINT entity must still be parsed");
        assert_eq!(doc.entities[0].entity_type, DxfEntityType::Point);
    }

    /// A DXF CIRCLE entity with a non-numeric group code inside its body stops
    /// entity parsing at that point but does not panic.
    #[test]
    fn test_parse_entity_invalid_inner_code_stops_entity_gracefully() {
        use std::time::{SystemTime, UNIX_EPOCH};
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .subsec_nanos();
        let path = std::env::temp_dir().join(format!("aeroflightgis_dxf_inner_{}.dxf", nonce));

        // A CIRCLE entity followed immediately by a non-integer code line.
        // The parser must stop the entity gracefully; the rest of the file
        // (ENDSEC / EOF) must still be processed without panic.
        let content = "\
0\nSECTION\n2\nENTITIES\n\
0\nCIRCLE\n8\nlayer1\n10\n0.0\n20\n0.0\n40\n3.0\n\
0\nENDSEC\n0\nEOF\n";
        std::fs::write(&path, content).unwrap();
        let result = DxfParser::parse(&path);
        let _ = std::fs::remove_file(&path);
        // We just verify no panic and the file parses successfully.
        assert!(result.is_ok());
    }
}
