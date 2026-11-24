//! Gestion des couches vecteur
//!
//! Support pour :
//! - GeoJSON (lecture/écriture)
//! - WFS (Web Feature Service)
//! - Géométries (Point, LineString, Polygon, Multi*)

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Types de géométrie supportés
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum GeometryType {
    Point,
    LineString,
    Polygon,
    MultiPoint,
    MultiLineString,
    MultiPolygon,
    GeometryCollection,
}

impl std::str::FromStr for GeometryType {
    type Err = String;

    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "point" => Ok(GeometryType::Point),
            "linestring" => Ok(GeometryType::LineString),
            "polygon" => Ok(GeometryType::Polygon),
            "multipoint" => Ok(GeometryType::MultiPoint),
            "multilinestring" => Ok(GeometryType::MultiLineString),
            "multipolygon" => Ok(GeometryType::MultiPolygon),
            "geometrycollection" => Ok(GeometryType::GeometryCollection),
            _ => Err(format!("Unknown geometry type: {}", s)),
        }
    }
}

/// Coordonnée 2D ou 3D
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Coordinate {
    pub x: f64,
    pub y: f64,
    pub z: Option<f64>,
}

impl Coordinate {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y, z: None }
    }

    pub fn new_3d(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z: Some(z) }
    }

    pub fn from_array(arr: &[f64]) -> Option<Self> {
        match arr.len() {
            2 => Some(Self::new(arr[0], arr[1])),
            3 => Some(Self::new_3d(arr[0], arr[1], arr[2])),
            _ => None,
        }
    }

    pub fn to_array(&self) -> Vec<f64> {
        if let Some(z) = self.z {
            vec![self.x, self.y, z]
        } else {
            vec![self.x, self.y]
        }
    }

    pub fn distance_2d(&self, other: &Coordinate) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

/// Géométrie GeoJSON-like
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum Geometry {
    Point {
        coordinates: Coordinate,
    },
    LineString {
        coordinates: Vec<Coordinate>,
    },
    Polygon {
        coordinates: Vec<Vec<Coordinate>>, // exterior ring + holes
    },
    MultiPoint {
        coordinates: Vec<Coordinate>,
    },
    MultiLineString {
        coordinates: Vec<Vec<Coordinate>>,
    },
    MultiPolygon {
        coordinates: Vec<Vec<Vec<Coordinate>>>,
    },
    GeometryCollection {
        geometries: Vec<Geometry>,
    },
}

impl Geometry {
    /// Type de la géométrie
    pub fn geometry_type(&self) -> GeometryType {
        match self {
            Geometry::Point { .. } => GeometryType::Point,
            Geometry::LineString { .. } => GeometryType::LineString,
            Geometry::Polygon { .. } => GeometryType::Polygon,
            Geometry::MultiPoint { .. } => GeometryType::MultiPoint,
            Geometry::MultiLineString { .. } => GeometryType::MultiLineString,
            Geometry::MultiPolygon { .. } => GeometryType::MultiPolygon,
            Geometry::GeometryCollection { .. } => GeometryType::GeometryCollection,
        }
    }

    /// Crée un point
    pub fn point(x: f64, y: f64) -> Self {
        Geometry::Point {
            coordinates: Coordinate::new(x, y),
        }
    }

    /// Crée une ligne
    pub fn line_string(coords: Vec<(f64, f64)>) -> Self {
        Geometry::LineString {
            coordinates: coords
                .into_iter()
                .map(|(x, y)| Coordinate::new(x, y))
                .collect(),
        }
    }

    /// Crée un polygone (sans trous)
    pub fn polygon(exterior: Vec<(f64, f64)>) -> Self {
        let ring: Vec<Coordinate> = exterior
            .into_iter()
            .map(|(x, y)| Coordinate::new(x, y))
            .collect();
        Geometry::Polygon {
            coordinates: vec![ring],
        }
    }

    /// Calcule le bounding box
    pub fn bbox(&self) -> Option<(f64, f64, f64, f64)> {
        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;

        fn update_bounds(
            coord: &Coordinate,
            min_x: &mut f64,
            min_y: &mut f64,
            max_x: &mut f64,
            max_y: &mut f64,
        ) {
            *min_x = min_x.min(coord.x);
            *min_y = min_y.min(coord.y);
            *max_x = max_x.max(coord.x);
            *max_y = max_y.max(coord.y);
        }

        fn process_coords(
            coords: &[Coordinate],
            min_x: &mut f64,
            min_y: &mut f64,
            max_x: &mut f64,
            max_y: &mut f64,
        ) {
            for c in coords {
                update_bounds(c, min_x, min_y, max_x, max_y);
            }
        }

        match self {
            Geometry::Point { coordinates } => {
                update_bounds(coordinates, &mut min_x, &mut min_y, &mut max_x, &mut max_y);
            }
            Geometry::LineString { coordinates } => {
                process_coords(coordinates, &mut min_x, &mut min_y, &mut max_x, &mut max_y);
            }
            Geometry::Polygon { coordinates } => {
                for ring in coordinates {
                    process_coords(ring, &mut min_x, &mut min_y, &mut max_x, &mut max_y);
                }
            }
            Geometry::MultiPoint { coordinates } => {
                process_coords(coordinates, &mut min_x, &mut min_y, &mut max_x, &mut max_y);
            }
            Geometry::MultiLineString { coordinates } => {
                for line in coordinates {
                    process_coords(line, &mut min_x, &mut min_y, &mut max_x, &mut max_y);
                }
            }
            Geometry::MultiPolygon { coordinates } => {
                for poly in coordinates {
                    for ring in poly {
                        process_coords(ring, &mut min_x, &mut min_y, &mut max_x, &mut max_y);
                    }
                }
            }
            Geometry::GeometryCollection { geometries } => {
                for geom in geometries {
                    if let Some((gmin_x, gmin_y, gmax_x, gmax_y)) = geom.bbox() {
                        min_x = min_x.min(gmin_x);
                        min_y = min_y.min(gmin_y);
                        max_x = max_x.max(gmax_x);
                        max_y = max_y.max(gmax_y);
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

    /// Calcule l'aire (pour polygones)
    pub fn area(&self) -> f64 {
        fn ring_area(coords: &[Coordinate]) -> f64 {
            if coords.len() < 3 {
                return 0.0;
            }
            let mut area = 0.0;
            for i in 0..coords.len() {
                let j = (i + 1) % coords.len();
                area += coords[i].x * coords[j].y;
                area -= coords[j].x * coords[i].y;
            }
            area.abs() / 2.0
        }

        match self {
            Geometry::Polygon { coordinates } => {
                let mut area = 0.0;
                for (i, ring) in coordinates.iter().enumerate() {
                    let ring_a = ring_area(ring);
                    if i == 0 {
                        area += ring_a; // exterior
                    } else {
                        area -= ring_a; // holes
                    }
                }
                area
            }
            Geometry::MultiPolygon { coordinates } => coordinates
                .iter()
                .map(|poly| {
                    let mut area = 0.0;
                    for (i, ring) in poly.iter().enumerate() {
                        let ring_a = ring_area(ring);
                        if i == 0 {
                            area += ring_a;
                        } else {
                            area -= ring_a;
                        }
                    }
                    area
                })
                .sum(),
            _ => 0.0,
        }
    }

    /// Calcule le centroïde
    pub fn centroid(&self) -> Option<Coordinate> {
        match self {
            Geometry::Point { coordinates } => Some(*coordinates),
            Geometry::LineString { coordinates } => {
                if coordinates.is_empty() {
                    return None;
                }
                let sum_x: f64 = coordinates.iter().map(|c| c.x).sum();
                let sum_y: f64 = coordinates.iter().map(|c| c.y).sum();
                let n = coordinates.len() as f64;
                Some(Coordinate::new(sum_x / n, sum_y / n))
            }
            Geometry::Polygon { coordinates } => {
                if coordinates.is_empty() || coordinates[0].is_empty() {
                    return None;
                }
                let exterior = &coordinates[0];
                let sum_x: f64 = exterior.iter().map(|c| c.x).sum();
                let sum_y: f64 = exterior.iter().map(|c| c.y).sum();
                let n = exterior.len() as f64;
                Some(Coordinate::new(sum_x / n, sum_y / n))
            }
            _ => {
                // Pour les multi-géométries, on prend le centre du bbox
                self.bbox().map(|(min_x, min_y, max_x, max_y)| {
                    Coordinate::new((min_x + max_x) / 2.0, (min_y + max_y) / 2.0)
                })
            }
        }
    }

    /// Parse depuis GeoJSON
    pub fn from_geojson(json: &serde_json::Value) -> Option<Self> {
        let geom_type = json.get("type")?.as_str()?;

        match geom_type {
            "Point" => {
                let coords = json.get("coordinates")?.as_array()?;
                let x = coords.first()?.as_f64()?;
                let y = coords.get(1)?.as_f64()?;
                let z = coords.get(2).and_then(|v| v.as_f64());
                Some(Geometry::Point {
                    coordinates: Coordinate { x, y, z },
                })
            }
            "LineString" => {
                let coords_array = json.get("coordinates")?.as_array()?;
                let coordinates: Vec<Coordinate> = coords_array
                    .iter()
                    .filter_map(|c| {
                        let arr = c.as_array()?;
                        let x = arr.first()?.as_f64()?;
                        let y = arr.get(1)?.as_f64()?;
                        let z = arr.get(2).and_then(|v| v.as_f64());
                        Some(Coordinate { x, y, z })
                    })
                    .collect();
                Some(Geometry::LineString { coordinates })
            }
            "Polygon" => {
                let rings = json.get("coordinates")?.as_array()?;
                let coordinates: Vec<Vec<Coordinate>> = rings
                    .iter()
                    .filter_map(|ring| {
                        let ring_arr = ring.as_array()?;
                        let coords: Vec<Coordinate> = ring_arr
                            .iter()
                            .filter_map(|c| {
                                let arr = c.as_array()?;
                                let x = arr.first()?.as_f64()?;
                                let y = arr.get(1)?.as_f64()?;
                                let z = arr.get(2).and_then(|v| v.as_f64());
                                Some(Coordinate { x, y, z })
                            })
                            .collect();
                        Some(coords)
                    })
                    .collect();
                Some(Geometry::Polygon { coordinates })
            }
            "MultiPoint" => {
                let coords_array = json.get("coordinates")?.as_array()?;
                let coordinates: Vec<Coordinate> = coords_array
                    .iter()
                    .filter_map(|c| {
                        let arr = c.as_array()?;
                        let x = arr.first()?.as_f64()?;
                        let y = arr.get(1)?.as_f64()?;
                        let z = arr.get(2).and_then(|v| v.as_f64());
                        Some(Coordinate { x, y, z })
                    })
                    .collect();
                Some(Geometry::MultiPoint { coordinates })
            }
            "MultiLineString" => {
                let lines = json.get("coordinates")?.as_array()?;
                let coordinates: Vec<Vec<Coordinate>> = lines
                    .iter()
                    .filter_map(|line| {
                        let line_arr = line.as_array()?;
                        Some(
                            line_arr
                                .iter()
                                .filter_map(|c| {
                                    let arr = c.as_array()?;
                                    let x = arr.first()?.as_f64()?;
                                    let y = arr.get(1)?.as_f64()?;
                                    let z = arr.get(2).and_then(|v| v.as_f64());
                                    Some(Coordinate { x, y, z })
                                })
                                .collect(),
                        )
                    })
                    .collect();
                Some(Geometry::MultiLineString { coordinates })
            }
            "MultiPolygon" => {
                let polys = json.get("coordinates")?.as_array()?;
                let coordinates: Vec<Vec<Vec<Coordinate>>> = polys
                    .iter()
                    .filter_map(|poly| {
                        let poly_arr = poly.as_array()?;
                        Some(
                            poly_arr
                                .iter()
                                .filter_map(|ring| {
                                    let ring_arr = ring.as_array()?;
                                    Some(
                                        ring_arr
                                            .iter()
                                            .filter_map(|c| {
                                                let arr = c.as_array()?;
                                                let x = arr.first()?.as_f64()?;
                                                let y = arr.get(1)?.as_f64()?;
                                                let z = arr.get(2).and_then(|v| v.as_f64());
                                                Some(Coordinate { x, y, z })
                                            })
                                            .collect(),
                                    )
                                })
                                .collect(),
                        )
                    })
                    .collect();
                Some(Geometry::MultiPolygon { coordinates })
            }
            "GeometryCollection" => {
                let geoms = json.get("geometries")?.as_array()?;
                let geometries: Vec<Geometry> =
                    geoms.iter().filter_map(Geometry::from_geojson).collect();
                Some(Geometry::GeometryCollection { geometries })
            }
            _ => None,
        }
    }

    /// Convertit la géométrie en WKT (Well-Known Text)
    pub fn to_wkt(&self) -> String {
        fn coord_to_wkt(c: &Coordinate) -> String {
            if let Some(z) = c.z {
                format!("{} {} {}", c.x, c.y, z)
            } else {
                format!("{} {}", c.x, c.y)
            }
        }

        fn coords_to_wkt(coords: &[Coordinate]) -> String {
            coords
                .iter()
                .map(coord_to_wkt)
                .collect::<Vec<_>>()
                .join(", ")
        }

        fn ring_to_wkt(ring: &[Coordinate]) -> String {
            format!("({})", coords_to_wkt(ring))
        }

        match self {
            Geometry::Point { coordinates } => {
                format!("POINT ({})", coord_to_wkt(coordinates))
            }
            Geometry::LineString { coordinates } => {
                format!("LINESTRING ({})", coords_to_wkt(coordinates))
            }
            Geometry::Polygon { coordinates } => {
                let rings: Vec<String> = coordinates.iter().map(|r| ring_to_wkt(r)).collect();
                format!("POLYGON ({})", rings.join(", "))
            }
            Geometry::MultiPoint { coordinates } => {
                let points: Vec<String> = coordinates
                    .iter()
                    .map(|c| format!("({})", coord_to_wkt(c)))
                    .collect();
                format!("MULTIPOINT ({})", points.join(", "))
            }
            Geometry::MultiLineString { coordinates } => {
                let lines: Vec<String> = coordinates
                    .iter()
                    .map(|line| format!("({})", coords_to_wkt(line)))
                    .collect();
                format!("MULTILINESTRING ({})", lines.join(", "))
            }
            Geometry::MultiPolygon { coordinates } => {
                let polys: Vec<String> = coordinates
                    .iter()
                    .map(|poly| {
                        let rings: Vec<String> = poly.iter().map(|r| ring_to_wkt(r)).collect();
                        format!("({})", rings.join(", "))
                    })
                    .collect();
                format!("MULTIPOLYGON ({})", polys.join(", "))
            }
            Geometry::GeometryCollection { geometries } => {
                let geoms: Vec<String> = geometries.iter().map(|g| g.to_wkt()).collect();
                format!("GEOMETRYCOLLECTION ({})", geoms.join(", "))
            }
        }
    }

    /// Convertit la géométrie en GeoJSON
    pub fn to_geojson(&self) -> serde_json::Value {
        fn coord_to_json(c: &Coordinate) -> serde_json::Value {
            if let Some(z) = c.z {
                serde_json::json!([c.x, c.y, z])
            } else {
                serde_json::json!([c.x, c.y])
            }
        }

        match self {
            Geometry::Point { coordinates } => {
                serde_json::json!({
                    "type": "Point",
                    "coordinates": coord_to_json(coordinates)
                })
            }
            Geometry::LineString { coordinates } => {
                serde_json::json!({
                    "type": "LineString",
                    "coordinates": coordinates.iter().map(coord_to_json).collect::<Vec<_>>()
                })
            }
            Geometry::Polygon { coordinates } => {
                serde_json::json!({
                    "type": "Polygon",
                    "coordinates": coordinates.iter()
                        .map(|ring| ring.iter().map(coord_to_json).collect::<Vec<_>>())
                        .collect::<Vec<_>>()
                })
            }
            Geometry::MultiPoint { coordinates } => {
                serde_json::json!({
                    "type": "MultiPoint",
                    "coordinates": coordinates.iter().map(coord_to_json).collect::<Vec<_>>()
                })
            }
            Geometry::MultiLineString { coordinates } => {
                serde_json::json!({
                    "type": "MultiLineString",
                    "coordinates": coordinates.iter()
                        .map(|line| line.iter().map(coord_to_json).collect::<Vec<_>>())
                        .collect::<Vec<_>>()
                })
            }
            Geometry::MultiPolygon { coordinates } => {
                serde_json::json!({
                    "type": "MultiPolygon",
                    "coordinates": coordinates.iter()
                        .map(|poly| poly.iter()
                            .map(|ring| ring.iter().map(coord_to_json).collect::<Vec<_>>())
                            .collect::<Vec<_>>())
                        .collect::<Vec<_>>()
                })
            }
            Geometry::GeometryCollection { geometries } => {
                serde_json::json!({
                    "type": "GeometryCollection",
                    "geometries": geometries.iter().map(|g| g.to_geojson()).collect::<Vec<_>>()
                })
            }
        }
    }
}

/// Valeur d'attribut
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum PropertyValue {
    Null,
    Bool(bool),
    Integer(i64),
    Float(f64),
    String(String),
    Array(Vec<PropertyValue>),
    Object(HashMap<String, PropertyValue>),
}

impl PropertyValue {
    pub fn as_str(&self) -> Option<&str> {
        match self {
            PropertyValue::String(s) => Some(s),
            _ => None,
        }
    }

    pub fn as_i64(&self) -> Option<i64> {
        match self {
            PropertyValue::Integer(i) => Some(*i),
            PropertyValue::Float(f) => Some(*f as i64),
            _ => None,
        }
    }

    pub fn as_f64(&self) -> Option<f64> {
        match self {
            PropertyValue::Float(f) => Some(*f),
            PropertyValue::Integer(i) => Some(*i as f64),
            _ => None,
        }
    }

    pub fn as_bool(&self) -> Option<bool> {
        match self {
            PropertyValue::Bool(b) => Some(*b),
            _ => None,
        }
    }
}

impl From<&str> for PropertyValue {
    fn from(s: &str) -> Self {
        PropertyValue::String(s.to_string())
    }
}

impl From<String> for PropertyValue {
    fn from(s: String) -> Self {
        PropertyValue::String(s)
    }
}

impl From<f64> for PropertyValue {
    fn from(v: f64) -> Self {
        PropertyValue::Float(v)
    }
}

impl From<i64> for PropertyValue {
    fn from(v: i64) -> Self {
        PropertyValue::Integer(v)
    }
}

impl From<bool> for PropertyValue {
    fn from(v: bool) -> Self {
        PropertyValue::Bool(v)
    }
}

impl PropertyValue {

    pub fn from_json(value: &serde_json::Value) -> Self {
        match value {
            serde_json::Value::Null => PropertyValue::Null,
            serde_json::Value::Bool(b) => PropertyValue::Bool(*b),
            serde_json::Value::Number(n) => {
                if let Some(i) = n.as_i64() {
                    PropertyValue::Integer(i)
                } else if let Some(f) = n.as_f64() {
                    PropertyValue::Float(f)
                } else {
                    PropertyValue::Null
                }
            }
            serde_json::Value::String(s) => PropertyValue::String(s.clone()),
            serde_json::Value::Array(arr) => {
                PropertyValue::Array(arr.iter().map(PropertyValue::from_json).collect())
            }
            serde_json::Value::Object(obj) => {
                let mut map = HashMap::new();
                for (k, v) in obj {
                    map.insert(k.clone(), PropertyValue::from_json(v));
                }
                PropertyValue::Object(map)
            }
        }
    }
}

/// Feature GeoJSON
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Feature {
    /// Identifiant unique (optionnel)
    pub id: Option<String>,
    /// Géométrie
    pub geometry: Option<Geometry>,
    /// Propriétés (attributs)
    pub properties: HashMap<String, PropertyValue>,
}

impl Feature {
    pub fn new(geometry: Geometry) -> Self {
        Self {
            id: None,
            geometry: Some(geometry),
            properties: HashMap::new(),
        }
    }

    pub fn with_id(mut self, id: &str) -> Self {
        self.id = Some(id.to_string());
        self
    }

    pub fn with_property(mut self, key: &str, value: PropertyValue) -> Self {
        self.properties.insert(key.to_string(), value);
        self
    }

    pub fn get_property(&self, key: &str) -> Option<&PropertyValue> {
        self.properties.get(key)
    }

    pub fn with_geometry(geometry: Geometry) -> Self {
        Self::new(geometry)
    }

    pub fn set_property<V: Into<PropertyValue>>(&mut self, key: &str, value: V) {
        self.properties.insert(key.to_string(), value.into());
    }

    pub fn get_string(&self, key: &str) -> Option<&str> {
        self.properties.get(key).and_then(|v| v.as_str())
    }

    /// Parse depuis GeoJSON
    pub fn from_geojson(json: &serde_json::Value) -> Option<Self> {
        let feature_type = json.get("type")?.as_str()?;
        if feature_type != "Feature" {
            return None;
        }

        let id = json.get("id").and_then(|v| match v {
            serde_json::Value::String(s) => Some(s.clone()),
            serde_json::Value::Number(n) => Some(n.to_string()),
            _ => None,
        });

        let geometry = json.get("geometry").and_then(Geometry::from_geojson);

        let properties = json
            .get("properties")
            .and_then(|p| p.as_object())
            .map(|obj| {
                obj.iter()
                    .map(|(k, v)| (k.clone(), PropertyValue::from_json(v)))
                    .collect()
            })
            .unwrap_or_default();

        Some(Feature {
            id,
            geometry,
            properties,
        })
    }
}

/// Collection de features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureCollection {
    pub features: Vec<Feature>,
    /// Système de coordonnées
    pub crs: Option<String>,
    /// Bounding box global
    pub bbox: Option<(f64, f64, f64, f64)>,
}

impl FeatureCollection {
    pub fn new(features: Vec<Feature>) -> Self {
        Self {
            features,
            crs: None,
            bbox: None,
        }
    }

    pub fn empty() -> Self {
        Self {
            features: Vec::new(),
            crs: None,
            bbox: None,
        }
    }

    pub fn with_crs(mut self, crs: &str) -> Self {
        self.crs = Some(crs.to_string());
        self
    }

    pub fn add(&mut self, feature: Feature) {
        self.features.push(feature);
    }

    pub fn add_feature(&mut self, feature: Feature) {
        self.features.push(feature);
    }

    pub fn len(&self) -> usize {
        self.features.len()
    }

    pub fn is_empty(&self) -> bool {
        self.features.is_empty()
    }

    /// Calcule le bbox global
    pub fn compute_bbox(&mut self) {
        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;

        for feature in &self.features {
            if let Some(geom) = &feature.geometry {
                if let Some((gmin_x, gmin_y, gmax_x, gmax_y)) = geom.bbox() {
                    min_x = min_x.min(gmin_x);
                    min_y = min_y.min(gmin_y);
                    max_x = max_x.max(gmax_x);
                    max_y = max_y.max(gmax_y);
                }
            }
        }

        if min_x <= max_x && min_y <= max_y {
            self.bbox = Some((min_x, min_y, max_x, max_y));
        }
    }

    /// Parse depuis GeoJSON string
    pub fn from_geojson(json_str: &str) -> Result<Self, serde_json::Error> {
        let json: serde_json::Value = serde_json::from_str(json_str)?;
        Ok(Self::from_geojson_value(&json))
    }

    /// Parse depuis GeoJSON value
    pub fn from_geojson_value(json: &serde_json::Value) -> Self {
        let collection_type = json.get("type").and_then(|t| t.as_str());

        if collection_type == Some("FeatureCollection") {
            let features: Vec<Feature> = json
                .get("features")
                .and_then(|f| f.as_array())
                .map(|arr| arr.iter().filter_map(Feature::from_geojson).collect())
                .unwrap_or_default();

            let crs = json
                .get("crs")
                .and_then(|c| c.get("properties"))
                .and_then(|p| p.get("name"))
                .and_then(|n| n.as_str())
                .map(|s| s.to_string());

            let bbox = json.get("bbox").and_then(|b| {
                let arr = b.as_array()?;
                if arr.len() >= 4 {
                    Some((
                        arr[0].as_f64()?,
                        arr[1].as_f64()?,
                        arr[2].as_f64()?,
                        arr[3].as_f64()?,
                    ))
                } else {
                    None
                }
            });

            FeatureCollection {
                features,
                crs,
                bbox,
            }
        } else if collection_type == Some("Feature") {
            // Single feature
            let mut collection = FeatureCollection::empty();
            if let Some(feature) = Feature::from_geojson(json) {
                collection.add(feature);
            }
            collection
        } else {
            FeatureCollection::empty()
        }
    }

    /// Convertit en GeoJSON string
    pub fn to_geojson(&self) -> String {
        let features_json: Vec<serde_json::Value> = self
            .features
            .iter()
            .map(|f| serde_json::to_value(f).unwrap_or(serde_json::Value::Null))
            .collect();

        let mut obj = serde_json::json!({
            "type": "FeatureCollection",
            "features": features_json
        });

        if let Some(crs) = &self.crs {
            obj["crs"] = serde_json::json!({
                "type": "name",
                "properties": {
                    "name": crs
                }
            });
        }

        if let Some((min_x, min_y, max_x, max_y)) = self.bbox {
            obj["bbox"] = serde_json::json!([min_x, min_y, max_x, max_y]);
        }

        serde_json::to_string_pretty(&obj).unwrap_or_default()
    }
}

impl Default for FeatureCollection {
    fn default() -> Self {
        Self::empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_geometry_bbox() {
        let poly = Geometry::polygon(vec![
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0),
        ]);
        let bbox = poly.bbox().unwrap();
        assert_eq!(bbox, (0.0, 0.0, 10.0, 10.0));
    }

    #[test]
    fn test_geometry_area() {
        let poly = Geometry::polygon(vec![
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0),
        ]);
        let area = poly.area();
        assert!((area - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_geojson_parse() {
        let geojson = r#"{
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "id": "1",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [2.5, 48.8]
                    },
                    "properties": {
                        "name": "Paris",
                        "population": 2161000
                    }
                }
            ]
        }"#;

        let collection = FeatureCollection::from_geojson(geojson).unwrap();
        assert_eq!(collection.len(), 1);

        let feature = &collection.features[0];
        assert_eq!(feature.id, Some("1".to_string()));

        let name = feature.get_property("name").unwrap().as_str().unwrap();
        assert_eq!(name, "Paris");
    }

    #[test]
    fn test_coordinate_distance() {
        let c1 = Coordinate::new(0.0, 0.0);
        let c2 = Coordinate::new(3.0, 4.0);
        assert!((c1.distance_2d(&c2) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_coordinate_from_array_2d() {
        let c = Coordinate::from_array(&[1.0, 2.0]).unwrap();
        assert!((c.x - 1.0).abs() < 1e-10);
        assert!((c.y - 2.0).abs() < 1e-10);
        assert!(c.z.is_none());
    }

    #[test]
    fn test_coordinate_from_array_3d() {
        let c = Coordinate::from_array(&[1.0, 2.0, 3.0]).unwrap();
        assert!((c.x - 1.0).abs() < 1e-10);
        assert!((c.z.unwrap() - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_coordinate_from_array_invalid() {
        assert!(Coordinate::from_array(&[1.0]).is_none());
        assert!(Coordinate::from_array(&[1.0, 2.0, 3.0, 4.0]).is_none());
        assert!(Coordinate::from_array(&[]).is_none());
    }

    #[test]
    fn test_coordinate_to_array_2d() {
        let c = Coordinate::new(5.0, 10.0);
        let arr = c.to_array();
        assert_eq!(arr.len(), 2);
        assert!((arr[0] - 5.0).abs() < 1e-10);
        assert!((arr[1] - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_coordinate_to_array_3d() {
        let c = Coordinate::new_3d(5.0, 10.0, 15.0);
        let arr = c.to_array();
        assert_eq!(arr.len(), 3);
        assert!((arr[2] - 15.0).abs() < 1e-10);
    }

    #[test]
    fn test_coordinate_roundtrip_2d() {
        let original = Coordinate::new(42.5, -73.1);
        let arr = original.to_array();
        let restored = Coordinate::from_array(&arr).unwrap();
        assert_eq!(original, restored);
    }

    #[test]
    fn test_coordinate_roundtrip_3d() {
        let original = Coordinate::new_3d(42.5, -73.1, 125.0);
        let arr = original.to_array();
        let restored = Coordinate::from_array(&arr).unwrap();
        assert_eq!(original, restored);
    }

    #[test]
    fn test_geometry_line_string() {
        let line = Geometry::line_string(vec![(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)]);
        assert_eq!(line.geometry_type(), GeometryType::LineString);
        let bbox = line.bbox().unwrap();
        assert!((bbox.0 - 0.0).abs() < 1e-10); // min_x
        assert!((bbox.2 - 2.0).abs() < 1e-10); // max_x
    }

    #[test]
    fn test_geometry_point() {
        let p = Geometry::point(3.5, 48.8);
        assert_eq!(p.geometry_type(), GeometryType::Point);
        let bbox = p.bbox().unwrap();
        assert!((bbox.0 - 3.5).abs() < 1e-10);
        assert!((bbox.1 - 48.8).abs() < 1e-10);
    }

    #[test]
    fn test_centroid_point() {
        let p = Geometry::point(3.5, 48.8);
        let c = p.centroid().unwrap();
        assert!((c.x - 3.5).abs() < 1e-10);
        assert!((c.y - 48.8).abs() < 1e-10);
    }

    #[test]
    fn test_centroid_line_string() {
        let line = Geometry::line_string(vec![(0.0, 0.0), (10.0, 0.0)]);
        let c = line.centroid().unwrap();
        assert!((c.x - 5.0).abs() < 1e-10);
        assert!((c.y - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_centroid_polygon() {
        let poly = Geometry::polygon(vec![
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0),
        ]);
        let c = poly.centroid().unwrap();
        // Centroid is mean of vertices (including closing vertex)
        // (0+10+10+0+0)/5, (0+0+10+10+0)/5 = 4.0, 4.0
        assert!((c.x - 4.0).abs() < 1e-10);
        assert!((c.y - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_to_wkt_point() {
        let p = Geometry::point(2.3, 48.8);
        let wkt = p.to_wkt();
        assert!(wkt.starts_with("POINT ("));
        assert!(wkt.contains("2.3"));
        assert!(wkt.contains("48.8"));
    }

    #[test]
    fn test_to_wkt_linestring() {
        let line = Geometry::line_string(vec![(0.0, 0.0), (1.0, 1.0)]);
        let wkt = line.to_wkt();
        assert!(wkt.starts_with("LINESTRING ("));
    }

    #[test]
    fn test_to_wkt_polygon() {
        let poly = Geometry::polygon(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]);
        let wkt = poly.to_wkt();
        assert!(wkt.starts_with("POLYGON ("));
    }

    #[test]
    fn test_geojson_point_roundtrip() {
        let p = Geometry::point(2.3, 48.8);
        let json = p.to_geojson();
        let restored = Geometry::from_geojson(&json).unwrap();
        assert_eq!(p, restored);
    }

    #[test]
    fn test_geojson_polygon_roundtrip() {
        let poly = Geometry::polygon(vec![
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0),
        ]);
        let json = poly.to_geojson();
        let restored = Geometry::from_geojson(&json).unwrap();
        assert_eq!(poly, restored);
    }

    #[test]
    fn test_geometry_type_from_str() {
        assert_eq!("point".parse::<GeometryType>(), Ok(GeometryType::Point));
        assert_eq!("POLYGON".parse::<GeometryType>(), Ok(GeometryType::Polygon));
        assert_eq!(
            "LineString".parse::<GeometryType>(),
            Ok(GeometryType::LineString)
        );
        assert!("invalid".parse::<GeometryType>().is_err());
    }

    #[test]
    fn test_area_non_polygon_is_zero() {
        let p = Geometry::point(1.0, 2.0);
        assert!((p.area() - 0.0).abs() < 1e-10);
        let line = Geometry::line_string(vec![(0.0, 0.0), (1.0, 1.0)]);
        assert!((line.area() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_area_triangle() {
        let tri = Geometry::polygon(vec![(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (0.0, 0.0)]);
        let area = tri.area();
        assert!((area - 6.0).abs() < 1e-10); // 4*3/2 = 6
    }

    #[test]
    fn test_feature_collection_empty() {
        let fc = FeatureCollection::empty();
        assert_eq!(fc.len(), 0);
        assert!(fc.is_empty());
    }

    #[test]
    fn test_feature_collection_compute_bbox() {
        let mut fc = FeatureCollection::empty();
        fc.add(Feature {
            id: Some("1".to_string()),
            geometry: Some(Geometry::point(5.0, 45.0)),
            properties: HashMap::new(),
        });
        fc.add(Feature {
            id: Some("2".to_string()),
            geometry: Some(Geometry::point(10.0, 50.0)),
            properties: HashMap::new(),
        });
        fc.compute_bbox();
        let bbox = fc.bbox.unwrap();
        assert!((bbox.0 - 5.0).abs() < 1e-10);
        assert!((bbox.1 - 45.0).abs() < 1e-10);
        assert!((bbox.2 - 10.0).abs() < 1e-10);
        assert!((bbox.3 - 50.0).abs() < 1e-10);
    }

    #[test]
    fn test_feature_collection_to_geojson() {
        let mut fc = FeatureCollection::empty();
        fc.add(Feature {
            id: Some("test".to_string()),
            geometry: Some(Geometry::point(2.3, 48.8)),
            properties: HashMap::new(),
        });
        let geojson_str = fc.to_geojson();
        assert!(geojson_str.contains("FeatureCollection"));
        assert!(geojson_str.contains("2.3"));
    }

    #[test]
    fn test_property_value_types() {
        let s = PropertyValue::String("hello".to_string());
        let n = PropertyValue::Float(42.0);
        let b = PropertyValue::Bool(true);
        let null = PropertyValue::Null;

        assert_eq!(s.as_str(), Some("hello"));
        assert!(s.as_f64().is_none());
        assert!((n.as_f64().unwrap() - 42.0).abs() < 1e-10);
        assert!(n.as_str().is_none());
        assert_eq!(b.as_bool(), Some(true));
        assert!(null.as_str().is_none());
    }
}
