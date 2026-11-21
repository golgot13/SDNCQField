//! Client de flux cartographiques IGN Géoplateforme & OSM avec cache disque SQLite hors-ligne.
//!
//! Adapté de `aerophotogram-core::geo::ign_streams` pour un usage terrain (SDNCQField) :
//! - Orthophotos HD IGN (`ORTHOIMAGERY.ORTHOPHOTOS`)
//! - Plan IGN v2 (`GEOGRAPHICALGRIDSYSTEMS.PLANIGNV2`)
//! - Cadastre (`CADASTRALPARCELS.PARCELS`)
//! - Altimétrie MNT BIL 32-bit (`ELEVATION.ELEVATIONGRIDCOVERAGE`)
//! - OpenStreetMap Standard (XYZ)
//! - Cache disque persistant SQLite (`tiles_cache.sqlite`) compatible mode hors-ligne terrain

use anyhow::Result;
use rusqlite::{params, Connection};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicUsize;
use std::sync::{Arc, Mutex};

pub static CONCURRENT_TILE_REQUESTS: AtomicUsize = AtomicUsize::new(0);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum BasemapLayerType {
    IgnOrtho,
    IgnPlan,
    IgnCadastre,
    IgnParcellaireExpress,
    IgnEstompage,
    IgnAltimetrieMnt,
    OpenStreetMap,
}

impl BasemapLayerType {
    pub fn label(&self) -> &'static str {
        match self {
            Self::IgnOrtho => "IGN BD ORTHO HD",
            Self::IgnPlan => "Plan IGN v2",
            Self::IgnCadastre => "Cadastre PCI Image (IGN)",
            Self::IgnParcellaireExpress => "Parcellaire Express PCI (IGN)",
            Self::IgnEstompage => "Estompage Relief (IGN)",
            Self::IgnAltimetrieMnt => "MNT Altimétrie IGN",
            Self::OpenStreetMap => "OpenStreetMap",
        }
    }

    pub fn layer_key(&self) -> &'static str {
        match self {
            Self::IgnOrtho => "ORTHOIMAGERY.ORTHOPHOTOS",
            Self::IgnPlan => "GEOGRAPHICALGRIDSYSTEMS.PLANIGNV2",
            Self::IgnCadastre => "CADASTRALPARCELS.PARCELS",
            Self::IgnParcellaireExpress => "CADASTRALPARCELS.PARCELLAIRE_EXPRESS",
            Self::IgnEstompage => "ELEVATION.ELEVATIONGRIDCOVERAGE.SHADOW",
            Self::IgnAltimetrieMnt => "ELEVATION.ELEVATIONGRIDCOVERAGE",
            Self::OpenStreetMap => "OSM.STANDARD",
        }
    }

    pub fn format_ext(&self) -> &'static str {
        match self {
            Self::IgnOrtho => "jpeg",
            Self::IgnPlan
            | Self::IgnCadastre
            | Self::IgnParcellaireExpress
            | Self::IgnEstompage
            | Self::OpenStreetMap => "png",
            Self::IgnAltimetrieMnt => "x-bil;bits=32",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct WmtsTileKey {
    pub layer: BasemapLayerType,
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

#[derive(Clone)]
pub struct TileStreamClient {
    pub tile_cache: Arc<Mutex<HashMap<WmtsTileKey, Vec<u8>>>>,
    pub disk_cache_path: Arc<Mutex<Option<PathBuf>>>,
    pub offline_mode: Arc<Mutex<bool>>,
}

impl Default for TileStreamClient {
    fn default() -> Self {
        Self {
            tile_cache: Arc::new(Mutex::new(HashMap::new())),
            disk_cache_path: Arc::new(Mutex::new(None)),
            offline_mode: Arc::new(Mutex::new(false)),
        }
    }
}

impl TileStreamClient {
    pub fn new() -> Self {
        Self::default()
    }

    /// Active un cache disque SQLite persistant pour le mode terrain hors-ligne.
    pub fn enable_disk_cache<P: AsRef<Path>>(&self, db_path: P) -> Result<()> {
        let p = db_path.as_ref().to_path_buf();
        if let Some(parent) = p.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent)?;
            }
        }
        let conn = Connection::open(&p)?;
        conn.execute_batch(
            "CREATE TABLE IF NOT EXISTS tiles (
                layer TEXT NOT NULL,
                z INTEGER NOT NULL,
                x INTEGER NOT NULL,
                y INTEGER NOT NULL,
                data BLOB NOT NULL,
                PRIMARY KEY (layer, z, x, y)
            );",
        )?;
        *self.disk_cache_path.lock().unwrap_or_else(|e| e.into_inner()) = Some(p);
        Ok(())
    }

    pub fn set_offline_mode(&self, offline: bool) {
        *self.offline_mode.lock().unwrap_or_else(|e| e.into_inner()) = offline;
    }

    pub fn is_offline_mode(&self) -> bool {
        *self.offline_mode.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Conversion Longitude / Latitude (WGS84) -> indices tuile Web Mercator (EPSG:3857)
    pub fn lon_lat_to_tile(lon_deg: f64, lat_deg: f64, zoom: u8) -> (u32, u32) {
        let n = 2.0f64.powi(zoom as i32);
        let x = ((lon_deg + 180.0) / 360.0 * n).floor().clamp(0.0, n - 1.0) as u32;
        let lat_clamped = lat_deg.clamp(-85.0511, 85.0511);
        let lat_rad = lat_clamped.to_radians();
        let y = ((1.0 - (lat_rad.tan() + 1.0 / lat_rad.cos()).ln() / std::f64::consts::PI) / 2.0 * n)
            .floor()
            .clamp(0.0, n - 1.0) as u32;
        (x, y)
    }

    /// Enveloppe WGS84 `(min_lon, min_lat, max_lon, max_lat)` d'une tuile Web Mercator
    pub fn tile_to_lon_lat_bounds(tile_x: u32, tile_y: u32, zoom: u8) -> (f64, f64, f64, f64) {
        let n = 2.0f64.powi(zoom as i32);
        let min_lon = (tile_x as f64 / n) * 360.0 - 180.0;
        let max_lon = ((tile_x + 1) as f64 / n) * 360.0 - 180.0;
        let sinh_min = (std::f64::consts::PI * (1.0 - 2.0 * (tile_y + 1) as f64 / n)).sinh();
        let min_lat = sinh_min.atan().to_degrees();
        let sinh_max = (std::f64::consts::PI * (1.0 - 2.0 * tile_y as f64 / n)).sinh();
        let max_lat = sinh_max.atan().to_degrees();
        (min_lon, min_lat, max_lon, max_lat)
    }

    pub fn build_tile_url(key: WmtsTileKey) -> String {
        match key.layer {
            BasemapLayerType::OpenStreetMap => {
                format!("https://tile.openstreetmap.org/{}/{}/{}.png", key.z, key.x, key.y)
            }
            _ => format!(
                "https://data.geopf.fr/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER={}&STYLE=normal&FORMAT=image/{}&TILEMATRIXSET=PM&TILEMATRIX={}&TILEROW={}&TILECOL={}",
                key.layer.layer_key(),
                key.layer.format_ext(),
                key.z,
                key.y,
                key.x
            ),
        }
    }

    fn load_from_disk(&self, key: WmtsTileKey) -> Option<Vec<u8>> {
        let path_opt = self.disk_cache_path.lock().unwrap_or_else(|e| e.into_inner()).clone();
        let path = path_opt?;
        let conn = Connection::open(path).ok()?;
        let mut stmt = conn
            .prepare("SELECT data FROM tiles WHERE layer = ?1 AND z = ?2 AND x = ?3 AND y = ?4")
            .ok()?;
        stmt.query_row(
            params![key.layer.layer_key(), key.z as i64, key.x as i64, key.y as i64],
            |row| row.get::<_, Vec<u8>>(0),
        )
        .ok()
    }

    fn save_to_disk(&self, key: WmtsTileKey, bytes: &[u8]) {
        let path_opt = self.disk_cache_path.lock().unwrap_or_else(|e| e.into_inner()).clone();
        if let Some(path) = path_opt {
            if let Ok(conn) = Connection::open(path) {
                let _ = conn.execute(
                    "INSERT OR REPLACE INTO tiles (layer, z, x, y, data) VALUES (?1, ?2, ?3, ?4, ?5)",
                    params![key.layer.layer_key(), key.z as i64, key.x as i64, key.y as i64, bytes],
                );
            }
        }
    }

    /// Récupère une tuile depuis la RAM, le cache disque SQLite, ou le réseau (si non hors-ligne).
    pub fn fetch_tile(&self, key: WmtsTileKey) -> Result<Vec<u8>> {
        {
            let cache = self.tile_cache.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(bytes) = cache.get(&key) {
                return Ok(bytes.clone());
            }
        }

        if let Some(disk_bytes) = self.load_from_disk(key) {
            let mut cache = self.tile_cache.lock().unwrap_or_else(|e| e.into_inner());
            cache.insert(key, disk_bytes.clone());
            return Ok(disk_bytes);
        }

        if self.is_offline_mode() {
            anyhow::bail!("Mode hors-ligne actif et tuile absente du cache disque");
        }

        let url = Self::build_tile_url(key);
        let resp = ureq::get(&url)
            .set("User-Agent", "SDNCQField-Rust/0.1 (Terrain GIS)")
            .timeout(std::time::Duration::from_secs(6))
            .call()?;

        let mut bytes = Vec::new();
        resp.into_reader().read_to_end(&mut bytes)?;

        let is_valid = if key.layer == BasemapLayerType::IgnAltimetrieMnt {
            bytes.len() >= 256 * 256 * 4
        } else {
            image::load_from_memory(&bytes).is_ok()
        };

        if !is_valid {
            anyhow::bail!("Tuile invalide ou tronquée : {}", url);
        }

        self.save_to_disk(key, &bytes);
        {
            let mut cache = self.tile_cache.lock().unwrap_or_else(|e| e.into_inner());
            cache.insert(key, bytes.clone());
        }
        Ok(bytes)
    }

    /// Décodeur de matrice d'altitude binaire BIL 32-bit float (256x256)
    pub fn parse_bil32_elevation_tile(bytes: &[u8]) -> Option<(u32, u32, Vec<f32>, Vec<u8>)> {
        if bytes.len() < 256 * 256 * 4 {
            return None;
        }
        let w = 256u32;
        let h = 256u32;
        let mut heights = Vec::with_capacity((w * h) as usize);
        let mut min_z = f32::MAX;
        let mut max_z = f32::MIN;

        for chunk in bytes.chunks_exact(4).take((w * h) as usize) {
            let val = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            let z = if val > -100.0 && val < 9000.0 { val } else { 0.0 };
            min_z = min_z.min(z);
            max_z = max_z.max(z);
            heights.push(z);
        }

        let mut rgba = Vec::with_capacity((w * h * 4) as usize);
        for &z in &heights {
            let norm_z = ((z - min_z) / (max_z - min_z).max(1.0)).clamp(0.0, 1.0);
            let (r, g, b) = if norm_z < 0.25 {
                let t = norm_z / 0.25;
                (0u8, (50.0 * (1.0 - t) + 220.0 * t) as u8, (255.0 * (1.0 - t) + 150.0 * t) as u8)
            } else if norm_z < 0.50 {
                let t = (norm_z - 0.25) / 0.25;
                ((255.0 * t) as u8, (220.0 * (1.0 - t) + 240.0 * t) as u8, (150.0 * (1.0 - t)) as u8)
            } else if norm_z < 0.75 {
                let t = (norm_z - 0.50) / 0.25;
                ((255.0 * (1.0 - t) + 250.0 * t) as u8, (240.0 * (1.0 - t) + 120.0 * t) as u8, 0u8)
            } else {
                let t = (norm_z - 0.75) / 0.25;
                ((250.0 * (1.0 - t) + 255.0 * t) as u8, (120.0 * (1.0 - t) + 255.0 * t) as u8, (255.0 * t) as u8)
            };
            rgba.extend_from_slice(&[r, g, b, 255]);
        }

        Some((w, h, heights, rgba))
    }

    pub fn get_cached_tile_rgba(&self, key: WmtsTileKey) -> Option<(u32, u32, Vec<u8>)> {
        let cache = self.tile_cache.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(bytes) = cache.get(&key) {
            if key.layer == BasemapLayerType::IgnAltimetrieMnt && bytes.len() >= 256 * 256 * 4 {
                if let Some((w, h, _, rgba)) = Self::parse_bil32_elevation_tile(bytes) {
                    return Some((w, h, rgba));
                }
            }
            if let Ok(dyn_img) = image::load_from_memory(bytes) {
                let rgba = dyn_img.to_rgba8();
                let (w, h) = rgba.dimensions();
                return Some((w, h, rgba.into_raw()));
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tile_math_paris() {
        let (tx, ty) = TileStreamClient::lon_lat_to_tile(2.3522, 48.8566, 14);
        let bounds = TileStreamClient::tile_to_lon_lat_bounds(tx, ty, 14);
        assert!(bounds.0 <= 2.3522 && bounds.2 >= 2.3522);
        assert!(bounds.1 <= 48.8566 && bounds.3 >= 48.8566);
    }

    #[test]
    fn test_disk_cache_roundtrip() {
        let tmp = std::env::temp_dir().join("sdnc_test_tiles_cache.sqlite");
        let _ = std::fs::remove_file(&tmp);
        let client = TileStreamClient::new();
        client.enable_disk_cache(&tmp).unwrap();
        let key = WmtsTileKey {
            layer: BasemapLayerType::IgnAltimetrieMnt,
            z: 14,
            x: 8300,
            y: 5600,
        };
        let payload = vec![0u8; 256 * 256 * 4];
        client.save_to_disk(key, &payload);
        client.set_offline_mode(true);
        let fetched = client.fetch_tile(key).unwrap();
        assert_eq!(fetched.len(), payload.len());
        let _ = std::fs::remove_file(&tmp);
    }
}
