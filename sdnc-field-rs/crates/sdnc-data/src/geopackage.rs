//! Provider OGC GeoPackage (`.gpkg`) complet en Rust pur (`rusqlite` bundled).
//!
//! Implémente le standard OGC GeoPackage 1.3 :
//! - Tables système `gpkg_spatial_ref_sys`, `gpkg_contents`, `gpkg_geometry_columns`
//! - Encodage et décodage GeoPackage Binary (GPB) + ISO WKB 2D & 3D (Point, LineString, Polygon, Multi*)
//! - Lecture / écriture complète des géométries et attributs

use crate::vector::{Coordinate, Feature, FeatureCollection, Geometry, GeometryType, PropertyValue};
use anyhow::{anyhow, Context, Result};
use rusqlite::{params, types::Value, Connection};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone)]
pub struct GpkgLayerMeta {
    pub table_name: String,
    pub geometry_column: String,
    pub geometry_type: String,
    pub srs_id: i32,
    pub feature_count: usize,
}

pub struct GeoPackage {
    path: PathBuf,
    conn: Connection,
}

impl GeoPackage {
    /// Ouvre ou crée un fichier GeoPackage conforme OGC.
    pub fn open<P: AsRef<Path>>(path: P) -> Result<Self> {
        let path_buf = path.as_ref().to_path_buf();
        if let Some(parent) = path_buf.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent)?;
            }
        }
        let conn = Connection::open(&path_buf)
            .with_context(|| format!("Impossible d'ouvrir le GeoPackage {:?}", path_buf))?;
        let gpkg = Self {
            path: path_buf,
            conn,
        };
        gpkg.ensure_core_tables()?;
        Ok(gpkg)
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    fn ensure_core_tables(&self) -> Result<()> {
        self.conn.execute_batch(
            "PRAGMA application_id = 1196444487;
             PRAGMA user_version = 10300;
             CREATE TABLE IF NOT EXISTS gpkg_spatial_ref_sys (
                 srs_name TEXT NOT NULL,
                 srs_id INTEGER NOT NULL PRIMARY KEY,
                 organization TEXT NOT NULL,
                 organization_coordsys_id INTEGER NOT NULL,
                 definition TEXT NOT NULL,
                 description TEXT
             );
             CREATE TABLE IF NOT EXISTS gpkg_contents (
                 table_name TEXT NOT NULL PRIMARY KEY,
                 data_type TEXT NOT NULL,
                 identifier TEXT UNIQUE,
                 description TEXT DEFAULT '',
                 last_change DATETIME NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ','now')),
                 min_x DOUBLE,
                 min_y DOUBLE,
                 max_x DOUBLE,
                 max_y DOUBLE,
                 srs_id INTEGER,
                 CONSTRAINT fk_gc_r_srs_id FOREIGN KEY (srs_id) REFERENCES gpkg_spatial_ref_sys(srs_id)
             );
             CREATE TABLE IF NOT EXISTS gpkg_geometry_columns (
                 table_name TEXT NOT NULL,
                 column_name TEXT NOT NULL,
                 geometry_type_name TEXT NOT NULL,
                 srs_id INTEGER NOT NULL,
                 z TINYINT NOT NULL,
                 m TINYINT NOT NULL,
                 CONSTRAINT pk_geom_cols PRIMARY KEY (table_name, column_name)
             );
             INSERT OR IGNORE INTO gpkg_spatial_ref_sys VALUES
                 ('Undefined cartesian SRS', -1, 'NONE', -1, 'undefined', 'undefined cartesian'),
                 ('Undefined geographic SRS', 0, 'NONE', 0, 'undefined', 'undefined geographic'),
                 ('WGS 84', 4326, 'EPSG', 4326, 'GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433]]', 'WGS 84'),
                 ('RGF93 / Lambert-93', 2154, 'EPSG', 2154, 'PROJCS[\"RGF93 / Lambert-93\",GEOGCS[\"RGF93\",DATUM[\"Reseau_Geodesique_Francais_1993\",SPHEROID[\"GRS 1980\",6378137,298.257222101]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433]],PROJECTION[\"Lambert_Conformal_Conic_2SP\"],PARAMETER[\"standard_parallel_1\",49],PARAMETER[\"standard_parallel_2\",44],PARAMETER[\"latitude_of_origin\",46.5],PARAMETER[\"central_meridian\",3],PARAMETER[\"false_easting\",700000],PARAMETER[\"false_northing\",6600000],UNIT[\"metre\",1]]', 'Lambert-93');",
        )?;
        Ok(())
    }

    pub fn ensure_srs(&self, epsg: i32, name: &str) -> Result<()> {
        self.conn.execute(
            "INSERT OR IGNORE INTO gpkg_spatial_ref_sys
             (srs_name, srs_id, organization, organization_coordsys_id, definition, description)
             VALUES (?1, ?2, 'EPSG', ?2, 'undefined', ?1)",
            params![name, epsg],
        )?;
        Ok(())
    }

    /// Liste les couches vectorielles présentes dans le GeoPackage.
    pub fn list_layers(&self) -> Result<Vec<GpkgLayerMeta>> {
        let mut stmt = self.conn.prepare(
            "SELECT gc.table_name, gc.column_name, gc.geometry_type_name, gc.srs_id
             FROM gpkg_geometry_columns gc
             JOIN gpkg_contents c ON c.table_name = gc.table_name
             WHERE c.data_type = 'features'",
        )?;

        let rows: Vec<(String, String, String, i32)> = stmt
            .query_map([], |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?, row.get(3)?)))?
            .collect::<std::result::Result<Vec<_>, _>>()?;

        let mut layers = Vec::with_capacity(rows.len());
        for (table_name, geometry_column, geometry_type, srs_id) in rows {
            let count_sql = format!("SELECT COUNT(*) FROM \"{}\"", sanitize_ident(&table_name));
            let feature_count: i64 = self
                .conn
                .query_row(&count_sql, [], |r| r.get(0))
                .unwrap_or(0);
            layers.push(GpkgLayerMeta {
                table_name,
                geometry_column,
                geometry_type,
                srs_id,
                feature_count: feature_count.max(0) as usize,
            });
        }
        Ok(layers)
    }

    /// Charge une couche complète sous forme de `FeatureCollection`.
    pub fn load_layer(&self, table_name: &str) -> Result<(FeatureCollection, i32)> {
        let safe_table = sanitize_ident(table_name);
        let (geom_col, srs_id): (String, i32) = self.conn.query_row(
            "SELECT column_name, srs_id FROM gpkg_geometry_columns WHERE table_name = ?1",
            params![table_name],
            |r| Ok((r.get(0)?, r.get(1)?)),
        )?;

        let mut stmt = self
            .conn
            .prepare(&format!("SELECT * FROM \"{}\"", safe_table))?;
        let col_names: Vec<String> = stmt.column_names().iter().map(|s| s.to_string()).collect();
        let geom_idx = col_names
            .iter()
            .position(|c| c.eq_ignore_ascii_case(&geom_col))
            .ok_or_else(|| anyhow!("Colonne géométrique '{}' absente de {}", geom_col, table_name))?;

        let mut features = Vec::new();
        let mut rows = stmt.query([])?;
        while let Some(row) = rows.next()? {
            let mut feat_id: Option<String> = None;
            let mut geometry: Option<Geometry> = None;
            let mut properties = HashMap::new();

            for (i, col_name) in col_names.iter().enumerate() {
                if i == geom_idx {
                    if let Ok(blob) = row.get::<_, Vec<u8>>(i) {
                        if let Ok((g, _)) = decode_gpkg_binary(&blob) {
                            geometry = Some(g);
                        }
                    }
                    continue;
                }
                let val_ref = row.get_ref(i)?;
                if col_name.eq_ignore_ascii_case("fid") || col_name.eq_ignore_ascii_case("id") {
                    if let rusqlite::types::ValueRef::Integer(id_num) = val_ref {
                        feat_id = Some(id_num.to_string());
                        continue;
                    }
                }
                let prop = match val_ref {
                    rusqlite::types::ValueRef::Null => PropertyValue::Null,
                    rusqlite::types::ValueRef::Integer(v) => PropertyValue::Integer(v),
                    rusqlite::types::ValueRef::Real(v) => PropertyValue::Float(v),
                    rusqlite::types::ValueRef::Text(bytes) => {
                        PropertyValue::String(String::from_utf8_lossy(bytes).to_string())
                    }
                    rusqlite::types::ValueRef::Blob(bytes) => {
                        PropertyValue::String(format!("<blob {} bytes>", bytes.len()))
                    }
                };
                properties.insert(col_name.clone(), prop);
            }

            let mut f = Feature {
                id: feat_id,
                geometry,
                properties,
            };
            if f.id.is_none() {
                f.id = Some((features.len() + 1).to_string());
            }
            features.push(f);
        }

        let mut fc = FeatureCollection::new(features);
        fc.crs = Some(format!("EPSG:{}", srs_id));
        Ok((fc, srs_id))
    }

    /// Sauvegarde (crée ou remplace) une couche complète dans le GeoPackage.
    pub fn save_layer(
        &mut self,
        table_name: &str,
        collection: &FeatureCollection,
        srs_id: i32,
        geometry_type: GeometryType,
    ) -> Result<()> {
        let safe_table = sanitize_ident(table_name);
        self.ensure_srs(srs_id, &format!("EPSG:{}", srs_id))?;

        // Collecte de tous les attributs présents
        let mut col_types: HashMap<String, &'static str> = HashMap::new();
        for f in &collection.features {
            for (k, v) in &f.properties {
                let safe_k = sanitize_ident(k);
                if safe_k.eq_ignore_ascii_case("fid") || safe_k.eq_ignore_ascii_case("geom") {
                    continue;
                }
                let sql_ty = match v {
                    PropertyValue::Integer(_) | PropertyValue::Bool(_) => "INTEGER",
                    PropertyValue::Float(_) => "REAL",
                    _ => "TEXT",
                };
                col_types
                    .entry(safe_k)
                    .and_modify(|existing| {
                        if *existing != sql_ty {
                            *existing = "TEXT";
                        }
                    })
                    .or_insert(sql_ty);
            }
        }
        let mut attr_cols: Vec<(String, &'static str)> = col_types.into_iter().collect();
        attr_cols.sort_by(|a, b| a.0.cmp(&b.0));

        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;
        let mut has_z = false;

        for f in &collection.features {
            if let Some(g) = &f.geometry {
                if let Some((bx0, by0, bx1, by1)) = g.bbox() {
                    min_x = min_x.min(bx0);
                    min_y = min_y.min(by0);
                    max_x = max_x.max(bx1);
                    max_y = max_y.max(by1);
                }
                if geometry_has_z(g) {
                    has_z = true;
                }
            }
        }
        if min_x > max_x {
            min_x = 0.0;
            min_y = 0.0;
            max_x = 0.0;
            max_y = 0.0;
        }

        let tx = self.conn.transaction()?;
        tx.execute(&format!("DROP TABLE IF EXISTS \"{}\"", safe_table), [])?;

        let mut create_sql = format!(
            "CREATE TABLE \"{}\" (fid INTEGER PRIMARY KEY AUTOINCREMENT, geom BLOB",
            safe_table
        );
        for (col, ty) in &attr_cols {
            create_sql.push_str(&format!(", \"{}\" {}", col, ty));
        }
        create_sql.push(')');
        tx.execute(&create_sql, [])?;

        let geom_type_name = match geometry_type {
            GeometryType::Point => "POINT",
            GeometryType::LineString => "LINESTRING",
            GeometryType::Polygon => "POLYGON",
            GeometryType::MultiPoint => "MULTIPOINT",
            GeometryType::MultiLineString => "MULTILINESTRING",
            GeometryType::MultiPolygon => "MULTIPOLYGON",
            GeometryType::GeometryCollection => "GEOMETRY",
        };

        tx.execute(
            "INSERT OR REPLACE INTO gpkg_contents
             (table_name, data_type, identifier, description, min_x, min_y, max_x, max_y, srs_id)
             VALUES (?1, 'features', ?1, '', ?2, ?3, ?4, ?5, ?6)",
            params![table_name, min_x, min_y, max_x, max_y, srs_id],
        )?;
        tx.execute(
            "INSERT OR REPLACE INTO gpkg_geometry_columns
             (table_name, column_name, geometry_type_name, srs_id, z, m)
             VALUES (?1, 'geom', ?2, ?3, ?4, 0)",
            params![table_name, geom_type_name, srs_id, if has_z { 1 } else { 0 }],
        )?;

        let mut cols_sql = String::from("geom");
        let mut placeholders = String::from("?1");
        for (idx, (col, _)) in attr_cols.iter().enumerate() {
            cols_sql.push_str(&format!(", \"{}\"", col));
            placeholders.push_str(&format!(", ?{}", idx + 2));
        }
        let insert_sql = format!(
            "INSERT INTO \"{}\" ({}) VALUES ({})",
            safe_table, cols_sql, placeholders
        );

        {
            let mut stmt = tx.prepare(&insert_sql)?;
            for f in &collection.features {
                let mut values: Vec<Value> = Vec::with_capacity(attr_cols.len() + 1);
                if let Some(g) = &f.geometry {
                    values.push(Value::Blob(encode_gpkg_binary(g, srs_id)));
                } else {
                    values.push(Value::Null);
                }
                for (col, _) in &attr_cols {
                    let v = match f.properties.get(col) {
                        Some(PropertyValue::Integer(i)) => Value::Integer(*i),
                        Some(PropertyValue::Float(fl)) => Value::Real(*fl),
                        Some(PropertyValue::Bool(b)) => Value::Integer(if *b { 1 } else { 0 }),
                        Some(PropertyValue::String(s)) => Value::Text(s.clone()),
                        Some(PropertyValue::Null) | None => Value::Null,
                        Some(other) => Value::Text(serde_json::to_string(other).unwrap_or_default()),
                    };
                    values.push(v);
                }
                stmt.execute(rusqlite::params_from_iter(values))?;
            }
        }

        tx.commit()?;
        Ok(())
    }
}

fn sanitize_ident(name: &str) -> String {
    name.chars()
        .map(|c| if c.is_alphanumeric() || c == '_' { c } else { '_' })
        .collect()
}

fn geometry_has_z(geom: &Geometry) -> bool {
    match geom {
        Geometry::Point { coordinates } => coordinates.z.is_some(),
        Geometry::LineString { coordinates } | Geometry::MultiPoint { coordinates } => {
            coordinates.iter().any(|c| c.z.is_some())
        }
        Geometry::Polygon { coordinates } | Geometry::MultiLineString { coordinates } => {
            coordinates.iter().flatten().any(|c| c.z.is_some())
        }
        Geometry::MultiPolygon { coordinates } => coordinates
            .iter()
            .flatten()
            .flatten()
            .any(|c| c.z.is_some()),
        Geometry::GeometryCollection { geometries } => geometries.iter().any(geometry_has_z),
    }
}

/// Encode une géométrie au format OGC GeoPackage Binary (GPB + ISO WKB little-endian).
pub fn encode_gpkg_binary(geom: &Geometry, srs_id: i32) -> Vec<u8> {
    let mut out = Vec::with_capacity(64);
    // GPB Header: Magic "GP", Version 0, Flags = 0x01 (little-endian, no envelope)
    out.extend_from_slice(&[b'G', b'P', 0x00, 0x01]);
    out.extend_from_slice(&srs_id.to_le_bytes());
    encode_wkb(geom, &mut out);
    out
}

fn encode_wkb(geom: &Geometry, out: &mut Vec<u8>) {
    let has_z = geometry_has_z(geom);
    let z_off: u32 = if has_z { 1000 } else { 0 };
    out.push(1); // NDR little-endian

    let write_coord = |c: &Coordinate, buf: &mut Vec<u8>| {
        buf.extend_from_slice(&c.x.to_le_bytes());
        buf.extend_from_slice(&c.y.to_le_bytes());
        if has_z {
            buf.extend_from_slice(&c.z.unwrap_or(0.0).to_le_bytes());
        }
    };

    match geom {
        Geometry::Point { coordinates } => {
            out.extend_from_slice(&(1u32 + z_off).to_le_bytes());
            write_coord(coordinates, out);
        }
        Geometry::LineString { coordinates } => {
            out.extend_from_slice(&(2u32 + z_off).to_le_bytes());
            out.extend_from_slice(&(coordinates.len() as u32).to_le_bytes());
            for c in coordinates {
                write_coord(c, out);
            }
        }
        Geometry::Polygon { coordinates } => {
            out.extend_from_slice(&(3u32 + z_off).to_le_bytes());
            out.extend_from_slice(&(coordinates.len() as u32).to_le_bytes());
            for ring in coordinates {
                out.extend_from_slice(&(ring.len() as u32).to_le_bytes());
                for c in ring {
                    write_coord(c, out);
                }
            }
        }
        Geometry::MultiPoint { coordinates } => {
            out.extend_from_slice(&(4u32 + z_off).to_le_bytes());
            out.extend_from_slice(&(coordinates.len() as u32).to_le_bytes());
            for c in coordinates {
                encode_wkb(&Geometry::Point { coordinates: *c }, out);
            }
        }
        Geometry::MultiLineString { coordinates } => {
            out.extend_from_slice(&(5u32 + z_off).to_le_bytes());
            out.extend_from_slice(&(coordinates.len() as u32).to_le_bytes());
            for line in coordinates {
                encode_wkb(&Geometry::LineString { coordinates: line.clone() }, out);
            }
        }
        Geometry::MultiPolygon { coordinates } => {
            out.extend_from_slice(&(6u32 + z_off).to_le_bytes());
            out.extend_from_slice(&(coordinates.len() as u32).to_le_bytes());
            for poly in coordinates {
                encode_wkb(&Geometry::Polygon { coordinates: poly.clone() }, out);
            }
        }
        Geometry::GeometryCollection { geometries } => {
            out.extend_from_slice(&(7u32 + z_off).to_le_bytes());
            out.extend_from_slice(&(geometries.len() as u32).to_le_bytes());
            for g in geometries {
                encode_wkb(g, out);
            }
        }
    }
}

/// Décode un BLOB OGC GeoPackage Binary (GPB) -> `(Geometry, srs_id)`.
pub fn decode_gpkg_binary(blob: &[u8]) -> Result<(Geometry, i32)> {
    if blob.len() < 8 || blob[0] != b'G' || blob[1] != b'P' {
        anyhow::bail!("En-tête GeoPackage Binary (GP) invalide");
    }
    let flags = blob[3];
    let little_endian = (flags & 0x01) != 0;
    let env_code = (flags >> 1) & 0x07;
    let env_bytes = match env_code {
        0 => 0,
        1 => 32,
        2 | 3 => 48,
        4 => 64,
        _ => anyhow::bail!("Code enveloppe GPB inconnu: {}", env_code),
    };
    let srs_id = if little_endian {
        i32::from_le_bytes([blob[4], blob[5], blob[6], blob[7]])
    } else {
        i32::from_be_bytes([blob[4], blob[5], blob[6], blob[7]])
    };
    let mut offset = 8 + env_bytes;
    let geom = decode_wkb(blob, &mut offset)?;
    Ok((geom, srs_id))
}

fn read_u32(data: &[u8], offset: &mut usize, le: bool) -> Result<u32> {
    if *offset + 4 > data.len() {
        anyhow::bail!("WKB tronqué (u32)");
    }
    let arr = [
        data[*offset],
        data[*offset + 1],
        data[*offset + 2],
        data[*offset + 3],
    ];
    *offset += 4;
    Ok(if le {
        u32::from_le_bytes(arr)
    } else {
        u32::from_be_bytes(arr)
    })
}

fn read_f64(data: &[u8], offset: &mut usize, le: bool) -> Result<f64> {
    if *offset + 8 > data.len() {
        anyhow::bail!("WKB tronqué (f64)");
    }
    let arr: [u8; 8] = data[*offset..*offset + 8].try_into()?;
    *offset += 8;
    Ok(if le {
        f64::from_le_bytes(arr)
    } else {
        f64::from_be_bytes(arr)
    })
}

fn read_coord(data: &[u8], offset: &mut usize, le: bool, has_z: bool, has_m: bool) -> Result<Coordinate> {
    let x = read_f64(data, offset, le)?;
    let y = read_f64(data, offset, le)?;
    let z = if has_z {
        Some(read_f64(data, offset, le)?)
    } else {
        None
    };
    if has_m {
        let _ = read_f64(data, offset, le)?;
    }
    Ok(Coordinate { x, y, z })
}

fn decode_wkb(data: &[u8], offset: &mut usize) -> Result<Geometry> {
    if *offset >= data.len() {
        anyhow::bail!("WKB vide");
    }
    let le = data[*offset] == 1;
    *offset += 1;
    let raw_type = read_u32(data, offset, le)?;
    // Supporte ISO WKB (1000/2000/3000) et EWKB (0x80000000 / 0x40000000)
    let ewkb_z = (raw_type & 0x8000_0000) != 0;
    let ewkb_m = (raw_type & 0x4000_0000) != 0;
    let clean_type = raw_type & 0x0FFF_FFFF;
    let iso_band = clean_type / 1000;
    let base_type = clean_type % 1000;
    let has_z = ewkb_z || iso_band == 1 || iso_band == 3;
    let has_m = ewkb_m || iso_band == 2 || iso_band == 3;

    match base_type {
        1 => Ok(Geometry::Point {
            coordinates: read_coord(data, offset, le, has_z, has_m)?,
        }),
        2 => {
            let n = read_u32(data, offset, le)? as usize;
            let mut coords = Vec::with_capacity(n);
            for _ in 0..n {
                coords.push(read_coord(data, offset, le, has_z, has_m)?);
            }
            Ok(Geometry::LineString { coordinates: coords })
        }
        3 => {
            let n_rings = read_u32(data, offset, le)? as usize;
            let mut rings = Vec::with_capacity(n_rings);
            for _ in 0..n_rings {
                let n = read_u32(data, offset, le)? as usize;
                let mut ring = Vec::with_capacity(n);
                for _ in 0..n {
                    ring.push(read_coord(data, offset, le, has_z, has_m)?);
                }
                rings.push(ring);
            }
            Ok(Geometry::Polygon { coordinates: rings })
        }
        4 => {
            let n = read_u32(data, offset, le)? as usize;
            let mut pts = Vec::with_capacity(n);
            for _ in 0..n {
                if let Geometry::Point { coordinates } = decode_wkb(data, offset)? {
                    pts.push(coordinates);
                }
            }
            Ok(Geometry::MultiPoint { coordinates: pts })
        }
        5 => {
            let n = read_u32(data, offset, le)? as usize;
            let mut lines = Vec::with_capacity(n);
            for _ in 0..n {
                if let Geometry::LineString { coordinates } = decode_wkb(data, offset)? {
                    lines.push(coordinates);
                }
            }
            Ok(Geometry::MultiLineString { coordinates: lines })
        }
        6 => {
            let n = read_u32(data, offset, le)? as usize;
            let mut polys = Vec::with_capacity(n);
            for _ in 0..n {
                if let Geometry::Polygon { coordinates } = decode_wkb(data, offset)? {
                    polys.push(coordinates);
                }
            }
            Ok(Geometry::MultiPolygon { coordinates: polys })
        }
        7 => {
            let n = read_u32(data, offset, le)? as usize;
            let mut geoms = Vec::with_capacity(n);
            for _ in 0..n {
                geoms.push(decode_wkb(data, offset)?);
            }
            Ok(Geometry::GeometryCollection { geometries: geoms })
        }
        _ => anyhow::bail!("Type géométrie WKB non supporté: {}", base_type),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_geopackage_roundtrip_3d_features() {
        let tmp = std::env::temp_dir().join("sdnc_test_roundtrip.gpkg");
        let _ = std::fs::remove_file(&tmp);

        let mut gpkg = GeoPackage::open(&tmp).unwrap();
        let mut fc = FeatureCollection::new(Vec::new());
        let mut f1 = Feature::with_geometry(Geometry::Point {
            coordinates: Coordinate::new_3d(902500.25, 6269100.75, 245.12),
        });
        f1.set_property("nom", "Borne_RTK_01");
        f1.set_property("precision_cm", 1.4f64);
        fc.add_feature(f1);

        let mut f2 = Feature::with_geometry(Geometry::Polygon {
            coordinates: vec![vec![
                Coordinate::new_3d(902500.0, 6269100.0, 245.0),
                Coordinate::new_3d(902520.0, 6269100.0, 246.0),
                Coordinate::new_3d(902520.0, 6269120.0, 247.0),
                Coordinate::new_3d(902500.0, 6269100.0, 245.0),
            ]],
        });
        f2.set_property("nom", "Parcelle_A");
        fc.add_feature(f2);

        gpkg.save_layer("releves_terrain", &fc, 2154, GeometryType::GeometryCollection)
            .unwrap();

        let layers = gpkg.list_layers().unwrap();
        assert_eq!(layers.len(), 1);
        assert_eq!(layers[0].table_name, "releves_terrain");
        assert_eq!(layers[0].feature_count, 2);

        let (loaded, srs) = gpkg.load_layer("releves_terrain").unwrap();
        assert_eq!(srs, 2154);
        assert_eq!(loaded.features.len(), 2);
        assert_eq!(loaded.features[0].get_string("nom"), Some("Borne_RTK_01"));
        if let Some(Geometry::Point { coordinates }) = &loaded.features[0].geometry {
            assert!((coordinates.x - 902500.25).abs() < 1e-6);
            assert!((coordinates.z.unwrap() - 245.12).abs() < 1e-6);
        } else {
            panic!("Point attendu");
        }

        let _ = std::fs::remove_file(&tmp);
    }
}
