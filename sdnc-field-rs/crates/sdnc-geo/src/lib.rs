//! `sdnc-geo` — Moteur géodésique 100 % Rust (adapté de `AEROPHOTOGRAM`)
//!
//! Fournit sans dépendance PROJ/GDAL C++ :
//! - Les 28 CRS français légaux (Lambert 93, CC42–CC50, NTF, UTM métropole/DOM-TOM) + WGS84
//! - Transformations ECEF <-> Géodésique <-> ENU local
//! - Ondulations de géoïde & altimétrie légale (NGF-IGN69, IGN78, DOM-TOM)
//! - Indexation spatiale Octree 3D
//! - Flux cartographiques WMTS IGN / OSM avec cache disque SQLite hors-ligne
//! - Géocodage BAN IGN

pub mod crs;
pub mod geocoding;
pub mod geoid;
pub mod ign_streams;
pub mod spatial_index;

pub use crs::*;
pub use geocoding::*;
pub use geoid::*;
pub use ign_streams::*;
pub use spatial_index::*;
