//! Parser EDIGEO (Format cadastral français)
//!
//! EDIGEO (EDIteur GEOgraphique) est le format standard français
//! utilisé par la DGFiP pour les données cadastrales.
//!
//! Structure des fichiers :
//! - THF (Table d'Habillage Format) : fichier index
//! - GEO : géométrie
//! - DIC : dictionnaire
//! - SCD : schéma conceptuel
//! - QAL : qualité
//! - VEC : données vectorielles

use anyhow::{anyhow, Result};
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use crate::vector::{Coordinate, Feature, FeatureCollection, Geometry, PropertyValue};

/// Types d'objets EDIGEO cadastraux
#[derive(Debug, Clone, PartialEq)]
pub enum EdigeoObjectType {
    /// Parcelle cadastrale
    Parcelle,
    /// Section cadastrale
    Section,
    /// Subdivision fiscale
    SubdivisionFiscale,
    /// Lieu-dit
    LieuDit,
    /// Bâtiment
    Batiment,
    /// Tronçon de route
    Route,
    /// Cours d'eau
    Hydro,
    /// Limite communale
    Commune,
    /// Point géodésique
    PointGeo,
    /// Texte/Annotation
    Texte,
    /// Autre
    Autre(String),
}

impl EdigeoObjectType {
    pub fn from_code(code: &str) -> Self {
        match code {
            "PARCELLE" | "Parcelle" => EdigeoObjectType::Parcelle,
            "SECTION" | "Section" => EdigeoObjectType::Section,
            "SUBDFISCAL" | "SUBDFISC" => EdigeoObjectType::SubdivisionFiscale,
            "LIEUDIT" => EdigeoObjectType::LieuDit,
            "BATIMENT" | "BATIMENT_DUR" => EdigeoObjectType::Batiment,
            "TRONROUTE" | "ROUTE" => EdigeoObjectType::Route,
            "TRONEAU" | "HYDRO" => EdigeoObjectType::Hydro,
            "COMMUNE" | "COMMUNE_id" => EdigeoObjectType::Commune,
            "PTCANV" | "BORNE" => EdigeoObjectType::PointGeo,
            "NUMVOIE" | "ZONCOMMUNI" | "LABEL" => EdigeoObjectType::Texte,
            _ => EdigeoObjectType::Autre(code.to_string()),
        }
    }

    pub fn as_str(&self) -> &str {
        match self {
            EdigeoObjectType::Parcelle => "Parcelle",
            EdigeoObjectType::Section => "Section",
            EdigeoObjectType::SubdivisionFiscale => "Subdivision fiscale",
            EdigeoObjectType::LieuDit => "Lieu-dit",
            EdigeoObjectType::Batiment => "Bâtiment",
            EdigeoObjectType::Route => "Route",
            EdigeoObjectType::Hydro => "Hydrographie",
            EdigeoObjectType::Commune => "Commune",
            EdigeoObjectType::PointGeo => "Point géodésique",
            EdigeoObjectType::Texte => "Texte",
            EdigeoObjectType::Autre(s) => s,
        }
    }
}

/// Entité EDIGEO
#[derive(Debug, Clone)]
pub struct EdigeoObject {
    pub id: String,
    pub object_type: EdigeoObjectType,
    pub geometry: Option<Geometry>,
    pub attributes: HashMap<String, String>,
}

/// Document EDIGEO complet (une commune)
#[derive(Debug, Clone)]
pub struct EdigeoDocument {
    /// Code INSEE de la commune
    pub code_insee: String,
    /// Code département
    pub code_dep: String,
    /// Nom de la commune
    pub commune_name: String,
    /// Date de mise à jour
    pub date_maj: String,
    /// Projection (généralement Lambert 93 ou CC zone)
    pub projection: String,
    /// Objets par type
    pub objects: HashMap<String, Vec<EdigeoObject>>,
    /// Tous les objets
    pub all_objects: Vec<EdigeoObject>,
}

impl Default for EdigeoDocument {
    fn default() -> Self {
        Self::new()
    }
}

impl EdigeoDocument {
    pub fn new() -> Self {
        Self {
            code_insee: String::new(),
            code_dep: String::new(),
            commune_name: String::new(),
            date_maj: String::new(),
            projection: "EPSG:2154".to_string(), // Lambert 93 par défaut
            objects: HashMap::new(),
            all_objects: Vec::new(),
        }
    }

    /// Convertir toutes les parcelles en FeatureCollection
    pub fn parcelles_to_feature_collection(&self) -> FeatureCollection {
        self.type_to_feature_collection(&EdigeoObjectType::Parcelle)
    }

    /// Convertir les bâtiments en FeatureCollection
    pub fn batiments_to_feature_collection(&self) -> FeatureCollection {
        self.type_to_feature_collection(&EdigeoObjectType::Batiment)
    }

    /// Convertir un type spécifique en FeatureCollection
    pub fn type_to_feature_collection(&self, obj_type: &EdigeoObjectType) -> FeatureCollection {
        let type_key = obj_type.as_str();
        let objects = self.objects.get(type_key);

        let features: Vec<Feature> = objects
            .map(|objs| objs.iter().filter_map(obj_to_feature).collect())
            .unwrap_or_default();

        FeatureCollection::new(features)
    }

    /// Convertir tout en FeatureCollection (avec filtre optionnel par type)
    pub fn to_feature_collection(&self, type_filter: Option<&str>) -> FeatureCollection {
        if let Some(type_name) = type_filter {
            // Convertir le nom de type en EdigeoObjectType
            let obj_type = match type_name.to_lowercase().as_str() {
                "parcelle" => EdigeoObjectType::Parcelle,
                "batiment" | "bâtiment" => EdigeoObjectType::Batiment,
                "section" => EdigeoObjectType::Section,
                "commune" => EdigeoObjectType::Commune,
                "lieu_dit" | "lieudit" => EdigeoObjectType::LieuDit,
                "route" | "voirie" => EdigeoObjectType::Route,
                "hydro" | "hydrographie" => EdigeoObjectType::Hydro,
                "point_geo" | "pointgeo" => EdigeoObjectType::PointGeo,
                "texte" | "annotation" => EdigeoObjectType::Texte,
                "subdivision" | "subdivision_fiscale" => EdigeoObjectType::SubdivisionFiscale,
                _ => {
                    // Si le type n'est pas reconnu, retourner une collection vide
                    return FeatureCollection::empty();
                }
            };
            self.type_to_feature_collection(&obj_type)
        } else {
            let features: Vec<Feature> =
                self.all_objects.iter().filter_map(obj_to_feature).collect();
            FeatureCollection::new(features)
        }
    }

    /// Obtenir les types d'objets disponibles
    pub fn available_types(&self) -> Vec<String> {
        self.objects.keys().cloned().collect()
    }
}

/// Parser EDIGEO
pub struct EdigeoParser;

impl EdigeoParser {
    /// Parse un fichier EDIGEO (THF ou dossier)
    pub fn parse<P: AsRef<Path>>(path: P) -> Result<EdigeoDocument> {
        let path = path.as_ref();

        // Si c'est un dossier, chercher le fichier THF
        let thf_path = if path.is_dir() {
            Self::find_thf_file(path)?
        } else if path.extension().map(|e| e.to_ascii_lowercase()) == Some("thf".into()) {
            path.to_path_buf()
        } else {
            // Chercher le THF dans le même dossier
            let parent = path.parent().unwrap_or(path);
            Self::find_thf_file(parent)?
        };

        let parent_dir = thf_path.parent().unwrap_or(Path::new("."));
        let mut document = EdigeoDocument::new();

        // Parser le fichier THF pour obtenir la liste des fichiers
        let file_list = Self::parse_thf(&thf_path)?;

        // Dictionnaire pour les attributs
        let mut dictionnaire: HashMap<String, String> = HashMap::new();

        // Géométries indexées par ID
        let mut geometries: HashMap<String, Vec<Coordinate>> = HashMap::new();
        let mut geometry_types: HashMap<String, String> = HashMap::new();

        // Parser chaque fichier référencé
        for filename in &file_list {
            let file_path = parent_dir.join(filename);
            if !file_path.exists() {
                log::warn!("EDIGEO file not found: {:?}", file_path);
                continue;
            }

            let ext = file_path
                .extension()
                .map(|e| e.to_string_lossy().to_uppercase())
                .unwrap_or_default();

            match ext.as_str() {
                "GEO" => {
                    Self::parse_geo(&file_path, &mut geometries, &mut geometry_types)?;
                }
                "DIC" => {
                    Self::parse_dic(&file_path, &mut dictionnaire)?;
                }
                "VEC" => {
                    Self::parse_vec(&file_path, &mut document, &geometries, &geometry_types)?;
                }
                "SCD" => {
                    // Schéma conceptuel - contient les métadonnées
                    Self::parse_scd(&file_path, &mut document)?;
                }
                _ => {}
            }
        }

        // Organiser les objets par type
        for obj in &document.all_objects {
            let type_key = obj.object_type.as_str().to_string();
            document
                .objects
                .entry(type_key)
                .or_default()
                .push(obj.clone());
        }

        log::info!(
            "EDIGEO parsed: {} objects, {} types",
            document.all_objects.len(),
            document.objects.len()
        );

        Ok(document)
    }

    fn find_thf_file(dir: &Path) -> Result<PathBuf> {
        for entry in fs::read_dir(dir)? {
            let entry = entry?;
            let path = entry.path();
            if let Some(ext) = path.extension() {
                if ext.eq_ignore_ascii_case("thf") {
                    return Ok(path);
                }
            }
        }
        Err(anyhow!("No THF file found in {:?}", dir))
    }

    fn parse_thf(path: &Path) -> Result<Vec<String>> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let mut files = Vec::new();

        for line in reader.lines() {
            let line = line?;
            // Format: LOT:xxx ou LOTL:xxx
            if line.starts_with("LOT") || line.starts_with("EOF") {
                continue;
            }
            // Chercher les références de fichiers
            if let Some(filename) = Self::extract_filename(&line) {
                files.push(filename);
            }
        }

        // Si aucun fichier trouvé, chercher les fichiers par convention de nommage
        if files.is_empty() {
            let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("");
            let parent = path.parent().unwrap_or(Path::new("."));

            for ext in &["GEO", "DIC", "SCD", "QAL", "VEC"] {
                let candidate = parent.join(format!("{}.{}", stem, ext));
                if candidate.exists() {
                    let name = candidate
                        .file_name()
                        .map(|n| n.to_string_lossy().to_string())
                        .unwrap_or_else(|| candidate.to_string_lossy().to_string());
                    files.push(name);
                }
            }
        }

        Ok(files)
    }

    fn extract_filename(line: &str) -> Option<String> {
        // Chercher les patterns de fichiers EDIGEO
        let parts: Vec<&str> = line.split(':').collect();
        if parts.len() >= 2 {
            let value = parts[1].trim();
            if value.contains('.') && !value.contains(' ') {
                return Some(value.to_string());
            }
        }
        None
    }

    fn parse_geo(
        path: &Path,
        geometries: &mut HashMap<String, Vec<Coordinate>>,
        geometry_types: &mut HashMap<String, String>,
    ) -> Result<()> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        let mut current_id = String::new();
        let mut current_coords: Vec<Coordinate> = Vec::new();
        let mut current_type = String::new();

        for line in reader.lines() {
            let line = line?;
            let trimmed = line.trim();

            // Nouvel objet géométrique
            if trimmed.starts_with("RIDSA") {
                // Sauvegarder l'objet précédent
                if !current_id.is_empty() && !current_coords.is_empty() {
                    geometries.insert(current_id.clone(), current_coords.clone());
                    geometry_types.insert(current_id.clone(), current_type.clone());
                }

                // Extraire l'ID
                if let Some(id) = Self::extract_value(trimmed, "RIDSA") {
                    current_id = id;
                    current_coords.clear();
                }
            }
            // Type de primitive (PAR, ARC, PNO)
            else if trimmed.starts_with("TYPSA") {
                if let Some(t) = Self::extract_value(trimmed, "TYPSA") {
                    current_type = t;
                }
            }
            // Coordonnées
            else if trimmed.starts_with("CORCC") {
                // Format: CORCC;x;y ou CORCC:x:y
                let parts: Vec<&str> = trimmed.split([';', ':']).collect();
                if parts.len() >= 3 {
                    if let (Ok(x), Ok(y)) = (
                        parts[1].trim().parse::<f64>(),
                        parts[2].trim().parse::<f64>(),
                    ) {
                        current_coords.push(Coordinate::new(x, y));
                    }
                }
            }
            // Coordonnées alternatives
            else if trimmed.starts_with("PTCCP") || trimmed.starts_with("SATYP") {
                let parts: Vec<&str> = trimmed.split([';', ':']).collect();
                if parts.len() >= 3 {
                    if let (Ok(x), Ok(y)) = (
                        parts[1].trim().parse::<f64>(),
                        parts[2].trim().parse::<f64>(),
                    ) {
                        current_coords.push(Coordinate::new(x, y));
                    }
                }
            }
        }

        // Sauvegarder le dernier objet
        if !current_id.is_empty() && !current_coords.is_empty() {
            geometries.insert(current_id.clone(), current_coords);
            geometry_types.insert(current_id, current_type);
        }

        Ok(())
    }

    fn parse_dic(path: &Path, dictionnaire: &mut HashMap<String, String>) -> Result<()> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        let mut current_key = String::new();

        for line in reader.lines() {
            let line = line?;
            let trimmed = line.trim();

            if trimmed.starts_with("RIDDI") {
                if let Some(key) = Self::extract_value(trimmed, "RIDDI") {
                    current_key = key;
                }
            } else if trimmed.starts_with("DINM") {
                if let Some(name) = Self::extract_value(trimmed, "DINM") {
                    dictionnaire.insert(current_key.clone(), name);
                }
            }
        }

        Ok(())
    }

    fn parse_vec(
        path: &Path,
        document: &mut EdigeoDocument,
        geometries: &HashMap<String, Vec<Coordinate>>,
        geometry_types: &HashMap<String, String>,
    ) -> Result<()> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        let mut current_obj: Option<EdigeoObject> = None;
        let mut current_geo_refs: Vec<String> = Vec::new();

        for line in reader.lines() {
            let line = line?;
            let trimmed = line.trim();

            // Nouvel objet
            if trimmed.starts_with("RIDOB") {
                // Sauvegarder l'objet précédent
                if let Some(mut obj) = current_obj.take() {
                    obj.geometry =
                        Self::build_geometry(&current_geo_refs, geometries, geometry_types);
                    document.all_objects.push(obj);
                }

                if let Some(id) = Self::extract_value(trimmed, "RIDOB") {
                    current_obj = Some(EdigeoObject {
                        id,
                        object_type: EdigeoObjectType::Autre(String::new()),
                        geometry: None,
                        attributes: HashMap::new(),
                    });
                    current_geo_refs.clear();
                }
            }
            // Type d'objet
            else if trimmed.starts_with("TYOB") || trimmed.starts_with("SEESSION") {
                if let Some(ref mut obj) = current_obj {
                    if let Some(type_code) = Self::extract_value(
                        trimmed,
                        if trimmed.starts_with("TYOB") {
                            "TYOB"
                        } else {
                            "SEESSION"
                        },
                    ) {
                        obj.object_type = EdigeoObjectType::from_code(&type_code);
                    }
                }
            }
            // Référence géométrique
            else if trimmed.starts_with("RIDO") || trimmed.starts_with("RIDG") {
                let prefix = if trimmed.starts_with("RIDO") {
                    "RIDO"
                } else {
                    "RIDG"
                };
                if let Some(geo_ref) = Self::extract_value(trimmed, prefix) {
                    current_geo_refs.push(geo_ref);
                }
            }
            // Attribut
            else if trimmed.starts_with("APTS") || trimmed.starts_with("ATPV") {
                if let Some(ref mut obj) = current_obj {
                    let parts: Vec<&str> = trimmed.splitn(3, [';', ':']).collect();
                    if parts.len() >= 3 {
                        let attr_name = parts[1].trim().to_string();
                        let attr_value = parts[2].trim().to_string();
                        obj.attributes.insert(attr_name, attr_value);
                    }
                }
            }
            // Identifiant parcelle
            else if trimmed.contains("IDU") || trimmed.contains("TEX") {
                if let Some(ref mut obj) = current_obj {
                    let parts: Vec<&str> = trimmed.split([';', ':']).collect();
                    if parts.len() >= 2 {
                        obj.attributes
                            .insert("IDU".to_string(), parts[1].trim().to_string());
                    }
                }
            }
        }

        // Sauvegarder le dernier objet
        if let Some(mut obj) = current_obj {
            obj.geometry = Self::build_geometry(&current_geo_refs, geometries, geometry_types);
            document.all_objects.push(obj);
        }

        Ok(())
    }

    fn parse_scd(path: &Path, document: &mut EdigeoDocument) -> Result<()> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        for line in reader.lines() {
            let line = line?;
            let trimmed = line.trim();

            if trimmed.starts_with("SCRC") {
                // Système de coordonnées
                if let Some(srs) = Self::extract_value(trimmed, "SCRC") {
                    if srs.contains("93") || srs.contains("2154") {
                        document.projection = "EPSG:2154".to_string();
                    }
                }
            } else if trimmed.starts_with("DTEFIN") || trimmed.starts_with("DATE") {
                if let Some(date) = Self::extract_value(
                    trimmed,
                    if trimmed.starts_with("DTEFIN") {
                        "DTEFIN"
                    } else {
                        "DATE"
                    },
                ) {
                    document.date_maj = date;
                }
            }
        }

        Ok(())
    }

    fn extract_value(line: &str, prefix: &str) -> Option<String> {
        if let Some(rest) = line.strip_prefix(prefix) {
            let parts: Vec<&str> = rest.split([';', ':']).collect();
            if !parts.is_empty() && !parts[0].is_empty() {
                return Some(parts[0].trim().to_string());
            }
            if parts.len() > 1 {
                return Some(parts[1].trim().to_string());
            }
        }
        None
    }

    fn build_geometry(
        geo_refs: &[String],
        geometries: &HashMap<String, Vec<Coordinate>>,
        geometry_types: &HashMap<String, String>,
    ) -> Option<Geometry> {
        if geo_refs.is_empty() {
            return None;
        }

        let mut all_coords: Vec<Coordinate> = Vec::new();
        let mut is_polygon = false;

        for geo_ref in geo_refs {
            if let Some(coords) = geometries.get(geo_ref) {
                all_coords.extend(coords.iter().cloned());
            }
            if let Some(gtype) = geometry_types.get(geo_ref) {
                if gtype.contains("PAR") || gtype.contains("polygon") {
                    is_polygon = true;
                }
            }
        }

        if all_coords.is_empty() {
            return None;
        }

        if all_coords.len() == 1 {
            Some(Geometry::Point {
                coordinates: all_coords[0],
            })
        } else if is_polygon || (all_coords.len() >= 3 && all_coords.first() == all_coords.last()) {
            // Polygone
            let mut coords = all_coords;
            if coords.first() != coords.last() {
                coords.push(coords[0]);
            }
            Some(Geometry::Polygon {
                coordinates: vec![coords],
            })
        } else {
            // Ligne
            Some(Geometry::LineString {
                coordinates: all_coords,
            })
        }
    }
}

/// Convertit un objet EDIGEO en Feature
fn obj_to_feature(obj: &EdigeoObject) -> Option<Feature> {
    let geom = obj.geometry.clone()?;

    let mut props: HashMap<String, PropertyValue> = HashMap::new();
    props.insert("id".to_string(), PropertyValue::String(obj.id.clone()));
    props.insert(
        "type".to_string(),
        PropertyValue::String(obj.object_type.as_str().to_string()),
    );

    for (k, v) in &obj.attributes {
        props.insert(k.clone(), PropertyValue::String(v.clone()));
    }

    Some(Feature {
        id: Some(obj.id.clone()),
        geometry: Some(geom),
        properties: props,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_object_type_from_code() {
        assert_eq!(EdigeoObjectType::from_code("PARCELLE").as_str(), "Parcelle");
        assert_eq!(EdigeoObjectType::from_code("BATIMENT").as_str(), "Bâtiment");
    }
}

/// Exportateur EDIGEO Standard (NF Z 52-000 DGFiP Cadastre)
pub struct EdigeoWriter;

impl EdigeoWriter {
    pub fn write_edigeo_bundle<P: AsRef<Path>>(
        output_dir: P,
        commune_code: &str,
        projection_crs: &str,
        layers: &[(String, Vec<Vec<(f64, f64, f64)>>)],
    ) -> Result<()> {
        let dir = output_dir.as_ref();
        fs::create_dir_all(dir)?;

        let lot_prefix = format!("E{}", commune_code);
        let thf_path = dir.join(format!("{}.THF", lot_prefix));
        let geo_path = dir.join(format!("{}.GEO", lot_prefix));
        let vec_path = dir.join(format!("{}.VEC", lot_prefix));
        let scd_path = dir.join(format!("{}.SCD", lot_prefix));
        let dic_path = dir.join(format!("{}.DIC", lot_prefix));

        // 1. Écriture du fichier THF (Table d'Habillage Format Header)
        let thf_content = format!(
            "RTY:THF\nRID:LOT_{commune_code}\nVER:1999-01-01\nPRO:{projection_crs}\nCOM:{commune_code}\n\
             FIC:{lot_prefix}.GEO\nFIC:{lot_prefix}.VEC\nFIC:{lot_prefix}.SCD\nFIC:{lot_prefix}.DIC\nEOF:\n"
        );
        fs::write(&thf_path, thf_content)?;

        // 2. Écriture du fichier GEO (Coordonnées des sommets géométriques)
        let mut geo_content = String::from("RTY:GEO\n");
        let mut pt_id = 1;
        for (_, polylines) in layers {
            for polyline in polylines {
                for (x, y, z) in polyline {
                    geo_content.push_str(&format!("PNT:P{pt_id};X={x:.4};Y={y:.4};Z={z:.4}\n"));
                    pt_id += 1;
                }
            }
        }
        geo_content.push_str("EOF:\n");
        fs::write(&geo_path, geo_content)?;

        // 3. Écriture du fichier VEC (Entités vectorielles cadastrales)
        let mut vec_content = String::from("RTY:VEC\n");
        let mut obj_id = 1;
        for (layer_name, polylines) in layers {
            for _polyline in polylines {
                vec_content.push_str(&format!("OBJ:O{obj_id};TYP={layer_name};GEO=LINE\n"));
                obj_id += 1;
            }
        }
        vec_content.push_str("EOF:\n");
        fs::write(&vec_path, vec_content)?;

        // 4. Fichiers de dictionnaire et de schéma (DIC et SCD)
        fs::write(&dic_path, "RTY:DIC\nNOM:DGFiP_EDIGEO\nEOF:\n")?;
        fs::write(&scd_path, "RTY:SCD\nSCH:CADASTRE_FRANCE\nEOF:\n")?;

        Ok(())
    }
}
