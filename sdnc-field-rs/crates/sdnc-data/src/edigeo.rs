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
        let u = code.trim().to_uppercase();
        if u.contains("PARCELLE") {
            EdigeoObjectType::Parcelle
        } else if u.contains("BATIMENT") {
            EdigeoObjectType::Batiment
        } else if u.contains("SECTION") || u.contains("SUBDSECT") {
            EdigeoObjectType::Section
        } else if u.contains("SUBDFISC") {
            EdigeoObjectType::SubdivisionFiscale
        } else if u.contains("LIEUDIT") {
            EdigeoObjectType::LieuDit
        } else if u.contains("TRONROUTE") || u.contains("VOIEP") || u == "ROUTE" {
            EdigeoObjectType::Route
        } else if u.contains("TRONEAU") || u.contains("TRONFLUV") || u.contains("HYDRO") {
            EdigeoObjectType::Hydro
        } else if u.contains("COMMUNE") {
            EdigeoObjectType::Commune
        } else if u.contains("PTCANV") || u.contains("BORNE") || u.contains("TPOINT") {
            EdigeoObjectType::PointGeo
        } else if u.contains("NUMVOIE")
            || u.contains("ZONCOMMUNI")
            || u.contains("LABEL")
            || u.contains("TEX")
            || u.contains("TLINE")
            || u.contains("TSURF")
            || u.contains("SYMBLIM")
        {
            EdigeoObjectType::Texte
        } else {
            EdigeoObjectType::Autre(code.to_string())
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

        // Parser le fichier THF + découvrir automatiquement tous les fichiers .GEO/.DIC/.SCD/.VEC du lot
        let mut file_list = Self::parse_thf(&thf_path)?;
        Self::discover_sibling_files(parent_dir, &mut file_list);

        // Garantir l'ordre de parsing : GEO / DIC / SCD d'abord, puis VEC
        file_list.sort_by_key(|f| {
            let ext = Path::new(f)
                .extension()
                .map(|e| e.to_string_lossy().to_uppercase())
                .unwrap_or_default();
            match ext.as_str() {
                "GEO" => 0,
                "DIC" => 1,
                "SCD" => 2,
                "VEC" => 3,
                _ => 4,
            }
        });

        // Dictionnaire pour les attributs
        let mut dictionnaire: HashMap<String, String> = HashMap::new();

        // Géométries indexées par ID (utilisées par le format simplifié)
        let mut geometries: HashMap<String, Vec<Coordinate>> = HashMap::new();
        let mut geometry_types: HashMap<String, String> = HashMap::new();

        // Parser chaque fichier référencé
        for filename in &file_list {
            let Some(file_path) = Self::resolve_case_insensitive(parent_dir, filename) else {
                log::warn!("EDIGEO file not found: {:?}", parent_dir.join(filename));
                continue;
            };

            let ext = file_path
                .extension()
                .map(|e| e.to_string_lossy().to_uppercase())
                .unwrap_or_default();

            match ext.as_str() {
                "GEO" => {
                    Self::parse_geo(
                        &file_path,
                        &mut document,
                        &mut geometries,
                        &mut geometry_types,
                    )?;
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
            "EDIGEO parsed: {} objects, {} types, crs={}",
            document.all_objects.len(),
            document.objects.len(),
            document.projection
        );

        Ok(document)
    }

    fn resolve_case_insensitive(dir: &Path, filename: &str) -> Option<PathBuf> {
        let direct = dir.join(filename);
        if direct.exists() {
            return Some(direct);
        }
        let target_lower = filename.to_lowercase();
        if let Ok(entries) = fs::read_dir(dir) {
            for entry in entries.flatten() {
                let p = entry.path();
                if let Some(name) = p.file_name().and_then(|n| n.to_str()) {
                    if name.to_lowercase() == target_lower {
                        return Some(p);
                    }
                }
            }
        }
        None
    }

    fn discover_sibling_files(dir: &Path, files: &mut Vec<String>) {
        let mut seen: std::collections::HashSet<String> =
            files.iter().map(|s| s.to_lowercase()).collect();
        if let Ok(entries) = fs::read_dir(dir) {
            for entry in entries.flatten() {
                let p = entry.path();
                if let Some(ext) = p.extension().and_then(|e| e.to_str()) {
                    let ext_u = ext.to_uppercase();
                    if matches!(ext_u.as_str(), "GEO" | "DIC" | "SCD" | "VEC") {
                        if let Some(name) = p.file_name().and_then(|n| n.to_str()) {
                            if seen.insert(name.to_lowercase()) {
                                files.push(name.to_string());
                            }
                        }
                    }
                }
            }
        }
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
            let trimmed = line.trim();
            if trimmed.starts_with("LOT") || trimmed.starts_with("EOF") {
                continue;
            }
            // Format DGFiP officiel : GEOSA..:<stem>, LONSA..:<stem>, GNSSA..:<stem>, GDCSA..:<stem>
            if let Some(val) = trimmed.split_once(':').map(|(_, v)| v.trim()) {
                if !val.is_empty() && !val.contains(' ') {
                    if val.contains('.') {
                        files.push(val.to_string());
                    } else if trimmed.starts_with("GEOSA") {
                        files.push(format!("{}.GEO", val));
                    } else if trimmed.starts_with("LONSA") || trimmed.starts_with("GDLSA") {
                        files.push(format!("{}.VEC", val));
                    } else if trimmed.starts_with("GDCSA") {
                        files.push(format!("{}.DIC", val));
                    } else if trimmed.starts_with("GNSSA") {
                        files.push(format!("{}.SCD", val));
                    }
                }
            }
        }

        Ok(files)
    }

    fn parse_geo(
        path: &Path,
        document: &mut EdigeoDocument,
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

            // Projection officielle DGFiP dans .GEO : RELSA..:LAMB93, RGF93CC44, LAMB2E, etc.
            if trimmed.starts_with("RELSA") {
                if let Some((_, val)) = trimmed.split_once(':') {
                    let s = val.trim();
                    if !s.is_empty() {
                        document.projection = s.to_string();
                    }
                }
            }
            // Nouvel objet géométrique (format simplifié)
            else if trimmed.starts_with("RIDSA") {
                if !current_id.is_empty() && !current_coords.is_empty() {
                    geometries.insert(current_id.clone(), current_coords.clone());
                    geometry_types.insert(current_id.clone(), current_type.clone());
                }
                if let Some(id) = Self::nf_value(trimmed).or_else(|| Self::extract_value(trimmed, "RIDSA")) {
                    current_id = id;
                    current_coords.clear();
                }
            } else if trimmed.starts_with("TYPSA") {
                if let Some(t) = Self::nf_value(trimmed).or_else(|| Self::extract_value(trimmed, "TYPSA")) {
                    current_type = t;
                }
            } else if trimmed.starts_with("CORCC")
                || trimmed.starts_with("PTCCP")
                || trimmed.starts_with("SATYP")
            {
                if let Some(coord) = Self::parse_corcc(trimmed) {
                    current_coords.push(coord);
                }
            }
        }

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

            if trimmed.starts_with("RIDDI") || trimmed.starts_with("RIDSA") {
                if let Some(key) = Self::nf_value(trimmed).or_else(|| Self::extract_value(trimmed, "RIDDI")) {
                    current_key = key;
                }
            } else if trimmed.starts_with("DINM") || trimmed.starts_with("LABSA") {
                if let Some(name) = Self::nf_value(trimmed).or_else(|| Self::extract_value(trimmed, "DINM")) {
                    if !current_key.is_empty() {
                        dictionnaire.insert(current_key.clone(), name);
                    }
                }
            }
        }

        Ok(())
    }

    /// Extrait la valeur après `:` pour une ligne NF Z 52-000 (`TAGnn:valeur`).
    fn nf_value(line: &str) -> Option<String> {
        line.split_once(':').map(|(_, v)| v.trim().to_string())
    }

    /// Extrait le dernier identifiant d'une référence composée NF Z 52-000 (`a;b;c;ID`).
    fn nf_ref_id(line: &str) -> Option<String> {
        let val = Self::nf_value(line)?;
        val.split(';')
            .rev()
            .find(|s| !s.trim().is_empty())
            .map(|s| s.trim().to_string())
    }

    /// Parse une ligne de coordonnées `CORCC..:+894500.12;+6269500.34;` ou `CORCC;x;y`.
    fn parse_corcc(trimmed: &str) -> Option<Coordinate> {
        let payload = trimmed.split_once(':').map(|(_, v)| v).unwrap_or(trimmed);
        let nums: Vec<f64> = payload
            .split([';', ':'])
            .filter_map(|p| {
                let s = p.trim();
                if s.is_empty() {
                    None
                } else {
                    s.parse::<f64>().ok()
                }
            })
            .collect();
        if nums.len() >= 2 {
            Some(Coordinate::new(nums[0], nums[1]))
        } else {
            None
        }
    }

    fn parse_vec(
        path: &Path,
        document: &mut EdigeoDocument,
        geometries: &HashMap<String, Vec<Coordinate>>,
        geometry_types: &HashMap<String, String>,
    ) -> Result<()> {
        let content = fs::read_to_string(path)?;
        // Détection automatique : format officiel NF Z 52-000 (`RTYSA..:`) vs format simplifié (`RIDOB`)
        if content.contains("RTYSA") {
            return Self::parse_vec_nfz52000(&content, document);
        }

        let mut current_obj: Option<EdigeoObject> = None;
        let mut current_geo_refs: Vec<String> = Vec::new();

        for line in content.lines() {
            let trimmed = line.trim();

            if trimmed.starts_with("RIDOB") {
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
            } else if trimmed.starts_with("TYOB") || trimmed.starts_with("SEESSION") {
                if let Some(ref mut obj) = current_obj {
                    let prefix = if trimmed.starts_with("TYOB") {
                        "TYOB"
                    } else {
                        "SEESSION"
                    };
                    if let Some(type_code) = Self::extract_value(trimmed, prefix) {
                        obj.object_type = EdigeoObjectType::from_code(&type_code);
                    }
                }
            } else if trimmed.starts_with("RIDO") || trimmed.starts_with("RIDG") {
                let prefix = if trimmed.starts_with("RIDO") {
                    "RIDO"
                } else {
                    "RIDG"
                };
                if let Some(geo_ref) = Self::extract_value(trimmed, prefix) {
                    current_geo_refs.push(geo_ref);
                }
            } else if trimmed.starts_with("APTS") || trimmed.starts_with("ATPV") {
                if let Some(ref mut obj) = current_obj {
                    let parts: Vec<&str> = trimmed.splitn(3, [';', ':']).collect();
                    if parts.len() >= 3 {
                        obj.attributes
                            .insert(parts[1].trim().to_string(), parts[2].trim().to_string());
                    }
                }
            } else if trimmed.contains("IDU") || trimmed.contains("TEX") {
                if let Some(ref mut obj) = current_obj {
                    let parts: Vec<&str> = trimmed.split([';', ':']).collect();
                    if parts.len() >= 2 {
                        obj.attributes
                            .insert("IDU".to_string(), parts[1].trim().to_string());
                    }
                }
            }
        }

        if let Some(mut obj) = current_obj {
            obj.geometry = Self::build_geometry(&current_geo_refs, geometries, geometry_types);
            document.all_objects.push(obj);
        }

        Ok(())
    }

    /// Parseur complet du format officiel DGFiP NF Z 52-000 `.VEC` :
    /// reconstruit la topologie `PNO` (nœuds), `ARC` (arcs `CORCC`), `PAR` (faces polygonales
    /// par chaînage d'arcs), `FEA` (objets cadastraux `PARCELLE`, `BATIMENT`, `SECTION`, etc.)
    /// et `LNK` (liens `FTPCP`).
    fn parse_vec_nfz52000(content: &str, document: &mut EdigeoDocument) -> Result<()> {
        let mut nodes: HashMap<String, Coordinate> = HashMap::new();
        let mut arcs: HashMap<String, Vec<Coordinate>> = HashMap::new();
        let mut face_arcs: HashMap<String, Vec<String>> = HashMap::new();
        let mut features: HashMap<String, EdigeoObject> = HashMap::new();
        let mut feature_order: Vec<String> = Vec::new();
        let mut fea_links: HashMap<String, Vec<String>> = HashMap::new();

        #[derive(Clone, Copy, PartialEq)]
        enum RecKind {
            None,
            Pno,
            Arc,
            Par,
            Fea,
            Lnk,
        }

        let mut kind = RecKind::None;
        let mut cur_id = String::new();
        let mut cur_coords: Vec<Coordinate> = Vec::new();
        let mut cur_type = String::new();
        let mut cur_attrs: HashMap<String, String> = HashMap::new();
        let mut cur_attr_key = String::new();
        let mut cur_links: Vec<String> = Vec::new();

        let flush = |kind: RecKind,
                     cur_id: &str,
                     cur_coords: &[Coordinate],
                     cur_type: &str,
                     cur_attrs: &HashMap<String, String>,
                     cur_links: &[String],
                     nodes: &mut HashMap<String, Coordinate>,
                     arcs: &mut HashMap<String, Vec<Coordinate>>,
                     face_arcs: &mut HashMap<String, Vec<String>>,
                     features: &mut HashMap<String, EdigeoObject>,
                     feature_order: &mut Vec<String>,
                     fea_links: &mut HashMap<String, Vec<String>>| {
            match kind {
                RecKind::Pno => {
                    if !cur_id.is_empty() {
                        if let Some(&c) = cur_coords.first() {
                            nodes.insert(cur_id.to_string(), c);
                        }
                    }
                }
                RecKind::Arc => {
                    if !cur_id.is_empty() && !cur_coords.is_empty() {
                        arcs.insert(cur_id.to_string(), cur_coords.to_vec());
                    }
                }
                RecKind::Par => {
                    if !cur_id.is_empty() {
                        face_arcs.entry(cur_id.to_string()).or_default();
                    }
                }
                RecKind::Fea => {
                    if !cur_id.is_empty() {
                        let obj = EdigeoObject {
                            id: cur_id.to_string(),
                            object_type: EdigeoObjectType::from_code(cur_type),
                            geometry: None,
                            attributes: cur_attrs.clone(),
                        };
                        if !features.contains_key(cur_id) {
                            feature_order.push(cur_id.to_string());
                        }
                        features.insert(cur_id.to_string(), obj);
                    }
                }
                RecKind::Lnk => {
                    if cur_links.len() >= 2 {
                        // Identifier les liens PAR <-> ARC ou FEA <-> (PAR/ARC/PNO)
                        let mut pars = Vec::new();
                        let mut arc_ids = Vec::new();
                        let mut fea_ids = Vec::new();
                        let mut pno_ids = Vec::new();

                        for r in cur_links {
                            if features.contains_key(r) {
                                fea_ids.push(r.clone());
                            } else if face_arcs.contains_key(r) || r.starts_with("Face_") || r.starts_with("PAR_") {
                                pars.push(r.clone());
                            } else if arcs.contains_key(r) || r.starts_with("Arc_") || r.starts_with("ARC_") {
                                arc_ids.push(r.clone());
                            } else if nodes.contains_key(r) || r.starts_with("Noeud_") || r.starts_with("PNO_") {
                                pno_ids.push(r.clone());
                            }
                        }

                        if !pars.is_empty() && !arc_ids.is_empty() {
                            for p in &pars {
                                face_arcs.entry(p.clone()).or_default().extend(arc_ids.iter().cloned());
                            }
                        }
                        if !fea_ids.is_empty() {
                            for f in &fea_ids {
                                let entry = fea_links.entry(f.clone()).or_default();
                                entry.extend(pars.iter().cloned());
                                entry.extend(arc_ids.iter().cloned());
                                entry.extend(pno_ids.iter().cloned());
                            }
                        }
                    }
                }
                RecKind::None => {}
            }
        };

        for line in content.lines() {
            let trimmed = line.trim();
            if trimmed.starts_with("RTYSA") {
                flush(
                    kind,
                    &cur_id,
                    &cur_coords,
                    &cur_type,
                    &cur_attrs,
                    &cur_links,
                    &mut nodes,
                    &mut arcs,
                    &mut face_arcs,
                    &mut features,
                    &mut feature_order,
                    &mut fea_links,
                );
                cur_id.clear();
                cur_coords.clear();
                cur_type.clear();
                cur_attrs.clear();
                cur_attr_key.clear();
                cur_links.clear();

                kind = match Self::nf_value(trimmed).as_deref() {
                    Some("PNO") => RecKind::Pno,
                    Some("ARC") => RecKind::Arc,
                    Some("PAR") => RecKind::Par,
                    Some("FEA") => RecKind::Fea,
                    Some("LNK") => RecKind::Lnk,
                    _ => RecKind::None,
                };
            } else if trimmed.starts_with("RIDSA") {
                if let Some(id) = Self::nf_value(trimmed) {
                    cur_id = id;
                }
            } else if trimmed.starts_with("CORCC") {
                if let Some(c) = Self::parse_corcc(trimmed) {
                    cur_coords.push(c);
                }
            } else if trimmed.starts_with("SCPCP") {
                if let Some(t) = Self::nf_ref_id(trimmed) {
                    cur_type = t;
                }
            } else if trimmed.starts_with("ATPCP") {
                if let Some(k) = Self::nf_ref_id(trimmed) {
                    // Normaliser les noms DGFiP courants (TEX_id -> TEX, IDU_id -> IDU, DUR_id -> DUR)
                    cur_attr_key = k.trim_end_matches("_id").to_string();
                }
            } else if trimmed.starts_with("ATVCV") || trimmed.starts_with("ATVST") {
                if let Some(v) = Self::nf_value(trimmed) {
                    let key = if cur_attr_key.is_empty() {
                        "TEX".to_string()
                    } else {
                        cur_attr_key.clone()
                    };
                    cur_attrs.insert(key, v);
                }
            } else if trimmed.starts_with("FTPCP") {
                if let Some(r) = Self::nf_ref_id(trimmed) {
                    cur_links.push(r);
                }
            }
        }

        flush(
            kind,
            &cur_id,
            &cur_coords,
            &cur_type,
            &cur_attrs,
            &cur_links,
            &mut nodes,
            &mut arcs,
            &mut face_arcs,
            &mut features,
            &mut feature_order,
            &mut fea_links,
        );

        // Assembler la géométrie de chaque FEA à partir de ses liens topologiques
        for fid in feature_order {
            if let Some(mut obj) = features.remove(&fid) {
                if let Some(links) = fea_links.get(&fid) {
                    let mut polygons: Vec<Vec<Vec<Coordinate>>> = Vec::new();
                    let mut lines: Vec<Vec<Coordinate>> = Vec::new();
                    let mut pts: Vec<Coordinate> = Vec::new();

                    for r in links {
                        if let Some(arc_list) = face_arcs.get(r) {
                            let ring = Self::stitch_arcs_into_ring(arc_list, &arcs);
                            if ring.len() >= 4 {
                                polygons.push(vec![ring]);
                            }
                        } else if let Some(ac) = arcs.get(r) {
                            if ac.len() >= 2 {
                                lines.push(ac.clone());
                            }
                        } else if let Some(&pt) = nodes.get(r) {
                            pts.push(pt);
                        }
                    }

                    obj.geometry = if polygons.len() == 1 {
                        Some(Geometry::Polygon {
                            coordinates: polygons.remove(0),
                        })
                    } else if polygons.len() > 1 {
                        Some(Geometry::MultiPolygon {
                            coordinates: polygons,
                        })
                    } else if lines.len() == 1 {
                        Some(Geometry::LineString {
                            coordinates: lines.remove(0),
                        })
                    } else if lines.len() > 1 {
                        Some(Geometry::MultiLineString { coordinates: lines })
                    } else if pts.len() == 1 {
                        Some(Geometry::Point {
                            coordinates: pts[0],
                        })
                    } else if pts.len() > 1 {
                        Some(Geometry::MultiPoint { coordinates: pts })
                    } else {
                        None
                    };
                }
                if obj.geometry.is_some() {
                    document.all_objects.push(obj);
                }
            }
        }

        Ok(())
    }

    /// Chaîne les arcs topologiques d'une face `PAR` en un anneau polygonal fermé ordonné.
    fn stitch_arcs_into_ring(
        arc_ids: &[String],
        arcs: &HashMap<String, Vec<Coordinate>>,
    ) -> Vec<Coordinate> {
        let mut segments: Vec<Vec<Coordinate>> = arc_ids
            .iter()
            .filter_map(|id| arcs.get(id))
            .filter(|pts| pts.len() >= 2)
            .cloned()
            .collect();
        if segments.is_empty() {
            return Vec::new();
        }
        let mut ring = segments.remove(0);
        let dist2 = |a: Coordinate, b: Coordinate| (a.x - b.x).powi(2) + (a.y - b.y).powi(2);

        while !segments.is_empty() {
            let tail = *ring.last().unwrap();
            let mut best_idx = 0usize;
            let mut best_rev = false;
            let mut best_d = f64::MAX;

            for (i, seg) in segments.iter().enumerate() {
                let d_start = dist2(tail, seg[0]);
                if d_start < best_d {
                    best_d = d_start;
                    best_idx = i;
                    best_rev = false;
                }
                let d_end = dist2(tail, *seg.last().unwrap());
                if d_end < best_d {
                    best_d = d_end;
                    best_idx = i;
                    best_rev = true;
                }
            }

            let mut next_seg = segments.remove(best_idx);
            if best_rev {
                next_seg.reverse();
            }
            if best_d < 1e-4 {
                ring.extend(next_seg.into_iter().skip(1));
            } else {
                ring.extend(next_seg);
            }
        }

        if ring.len() >= 3 {
            let first = ring[0];
            let last = *ring.last().unwrap();
            if dist2(first, last) > 1e-6 {
                ring.push(first);
            }
        }
        ring
    }

    fn parse_scd(path: &Path, document: &mut EdigeoDocument) -> Result<()> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        for line in reader.lines() {
            let line = line?;
            let trimmed = line.trim();

            if trimmed.starts_with("SCRC") {
                if let Some(srs) = Self::extract_value(trimmed, "SCRC") {
                    if !srs.is_empty() {
                        document.projection = srs;
                    }
                }
            } else if trimmed.starts_with("DTEFIN") || trimmed.starts_with("DATE") {
                let prefix = if trimmed.starts_with("DTEFIN") {
                    "DTEFIN"
                } else {
                    "DATE"
                };
                if let Some(date) = Self::extract_value(trimmed, prefix) {
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
