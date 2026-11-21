# SDNCQField-RS (Rust + Vulkan Compute + Carto 2D/3D AEROPHOTOGRAM + RTKLIB-Rust)

Reconstruction modulaire 100 % Rust de **SDNCQField** remplaçant QGIS, GDAL, PROJ et Qt par :

- **Cartographie 2D & 3D** extraites et découplées d'`AEROPHOTOGRAM` (`sdnc-geo` + `sdnc-ui`)
- **Accélération GPU Vulkan Compute (`wgpu` / WGSL)** (`sdnc-gpu`) : interpolation MNT IDW, pente et ombrage (hillshade) en origine flottante `f32` (compatible GPU mobiles Android Mali/Adreno, Linux et Windows)
- **Format de projet natif léger (`.sdnc.json` + `.gpkg`)** (`sdnc-data`) : provider OGC GeoPackage lecture/écriture complet (`rusqlite` bundled), GeoJSON, DXF, EDIGÉO et cache disque SQLite hors-ligne pour les tuiles IGN / OSM
- **Moteur GNSS RTK & PPK adapté de RTKLIB en Rust pur** (`sdnc-gnss`) :
  - Éphémérides Kepler, Sagnac, Saastamoinen, Klobuchar (`ephemeris.rs`)
  - Résolution d'ambiguïtés entières **LAMBDA / MLAMBDA** + Ratio Test (`lambda.rs`)
  - Filtre de Kalman Double-Différences **RTK Float / Fix** + SPP (`rtk_engine.rs`)
  - Post-traitement **PPK Forward / Backward / Combined** sur RINEX 3 OBS & NAV (`ppk_solver.rs`, `rinex.rs`)
  - Flux temps réel **NTRIP v1/v2**, **RTCM 3.x (CRC-24Q)**, **UBX (NAV-PVT / RXM-RAWX)**, **NMEA-0183** et pont **Android JNI**

## Architecture du workspace

```text
sdnc-field-rs/
├── crates/
│   ├── sdnc-geo/          # 28 CRS FR + WGS84, géoïde, Octree 3D, flux WMTS IGN/OSM + cache SQLite offline
│   ├── sdnc-data/         # GeoPackage OGC R/W (GPB/WKB 2D/3D), projet .sdnc.json, GeoJSON, DXF, EDIGÉO, DEM
│   ├── sdnc-gpu/          # Contexte Vulkan Compute (wgpu) + shaders WGSL (terrain_idw.wgsl, point_project.wgsl)
│   ├── sdnc-gnss/         # RTKLIB en Rust pur (SPP, RTK EKF, LAMBDA, PPK Fwd/Bwd, RINEX 3, RTCM3, UBX, NTRIP, NMEA)
│   ├── sdnc-ui/           # Interface tactile & desktop (egui + wgpu) : 2D, 3D Vulkan, Saisie/Snapping, Panneau RTK/PPK
│   ├── sdnc-app-desktop/  # Exécutable Windows & Linux (`sdnc-field`)
│   └── sdnc-app-android/  # Bibliothèque native Android (`libsdnc_field_android.so` + pont JNI GNSS)
└── android/               # Gabarit AndroidManifest.xml + MainActivity.kt (GameActivity + Vulkan 1.1 + LocationManager)
```

## Compilation & Tests

### Linux / Windows (Desktop)

```bash
cd sdnc-field-rs
cargo test --workspace
cargo run --release -p sdnc-app-desktop
```

Pour compiler un exécutable Windows depuis Linux (cross-compilation MinGW ou nativement sous Windows MSVC) :

```bash
# Sous Windows natif (MSVC + Vulkan/DX12) :
cargo build --release -p sdnc-app-desktop
```

### Android (`arm64-v8a`, Vulkan 1.1+)

Prérequis : Android NDK + `cargo-ndk` + cible Rust `aarch64-linux-android`.

```bash
rustup target add aarch64-linux-android
cargo install cargo-ndk
cargo ndk -t arm64-v8a -o android/app/src/main/jniLibs build --release -p sdnc-app-android
```
