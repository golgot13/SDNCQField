# SDNCQField-RS (Rust + Vulkan 2D Cadastre + Flux IGN Raster/Vecteurs + RTKLIB-Rust)

Reconstruction modulaire 100 % Rust de **SDNCQField** (PC Windows, PC Linux et Android tablettes & téléphones) remplaçant QGIS, GDAL, PROJ et Qt par une application **strictement 2D haute performance** :

- **Moteur Cartographique 2D GPU natif Vulkan (`wgpu` / `egui_wgpu::CallbackTrait`)** (`sdnc-gpu` + `sdnc-ui`) :
  - Triangulation **ear-clipping 2D** des polygones concaves (bâtiments en L/U, parcelles complexes)
  - Buffers VRAM persistants en origine flottante `f32` (upload uniquement sur modification de couche)
  - **2 draw calls GPU par frame** (remplissage polygones + segments cadastraux instanciés anti-aliasés avec culling frustum dans le vertex shader)
  - Reprojection géodésique des tuiles raster par maillage 4×4 (adaptée d'`AEROPHOTOGRAM`)
- **Support natif du Plan Cadastral Français** (`sdnc-data::cadastre`, `dxf.rs`, `edigeo.rs`) :
  - **DXF & DXF-PCI** (entités `POLYLINE`+`VERTEX`+`SEQEND`, `LWPOLYLINE`, `LINE`, `CIRCLE`, `ARC`, `TEXT`/`MTEXT`, détection automatique `$PCI_PROJECTION` et couleurs ACI)
  - **EDIGÉO DGFiP (NF Z 52-000)** (`.THF` / dossier complet, reconstruction topologique `PAR`/`ARC`/`PNO`/`FEA`/`LNK` par chaînage d'arcs, détection CRS `.GEO` `RELSA:LAMB93` / `RGF93CC42..50`)
  - Symbologie cadastrale DGFiP automatique par calques (Sections, Parcelles, Bâtiments durs & légers, Voirie, Hydrographie, Bornes, Habillage/Textes)
- **Flux IGN Géoplateforme Raster & Vecteurs** (`sdnc-geo`) :
  - **Raster WMTS** avec cache disque SQLite hors-ligne : BD ORTHO, Plan IGN v2, Cadastre PCI Image, Parcellaire Express, Estompage, Altimétrie MNT, OpenStreetMap
  - **Vecteurs WFS 2.0.0 GeoJSON** (`data.geopf.fr/wfs/ows`, `SRSNAME=CRS:84`) : Parcelles PCI Express, Bâtiments PCI Express, Feuilles/Sections PCI, BD TOPO v3 (Bâtiments, Routes, Cours d'eau, Communes) — sauvegardables hors-ligne dans le `.gpkg`
- **Format de projet natif léger (`.sdnc.json` + `.gpkg`)** (`sdnc-data`) : provider OGC GeoPackage lecture/écriture complet (`rusqlite` bundled) + GeoJSON
- **Pilotage GNSS RTK L1/L2/L5 multi-marques & Post-Traitement PPK (RTKLIB-Rust)** (`sdnc-gnss`) :
  - **Pilotes & profils matériels dédiés** (`receivers.rs`) : **Leica** (GS18 T / GS16 / GS15 / GS07 / FLX100, `$GPLLQ`), **Trimble** (R12i / R12 / R10 / R8s / DA2, `$PTNL,GGK`), **Sokkia / Topcon** (GCX3 / GRX3 / HiPer VR, commandes GREIS/TPS), **SparkFun & modules u-blox ZED-F9P** (RTK Facet / Express / Torch, ArduSimple, initialisation `UBX-CFG-VALSET` L1+L2), **Emlid Reach complet** (Reach M+/RS+ **L1** M8T → M2 → RS2/RS2+ → RS3 → **RS4 Pro L1/L2/L5 + Tilt**)
  - **Filtre de Kalman Double-Différences bi-fréquence L1+L2** + résolution entière **LAMBDA / MLAMBDA** + Ratio Test (`rtk_engine.rs`, `lambda.rs`) + moyennage multi-époques + compensation hauteur canne & offset ARP→APC
  - **Post-traitement PPK complet** (`ppk_solver.rs`, `rinex.rs`, `ubx.rs`) : entrée directe **RINEX 3.04 L1/L2/L5** ou **binaire `.ubx` brut** (Emlid/F9P), modes **Cinématique Combined (Forward+Backward)** et **Statique (Pivot/Borne)**, recalage des événements terrain, import carte `.gpkg` & export CSV
  - Flux temps réel **NTRIP v1/v2**, **RTCM 3.3 (CRC-24Q, 1005/1006, 1019/1046, MSM4/MSM7 bit-à-bit)**, **UBX (`NAV-PVT` / `RXM-RAWX` L1+L2+L5)**, **NMEA (`GGA`/`GST`/`GSA`/`GSV` Skyplot)**, **Bluetooth SPP bidirectionnel / USB-Série / TCP Wi-Fi** et pont JNI Android

## Architecture du workspace

```text
sdnc-field-rs/
├── crates/
│   ├── sdnc-geo/          # 28 CRS FR + WGS84, géoïde, BAN, flux IGN WMTS (cache SQLite) & IGN WFS 2.0 GeoJSON
│   ├── sdnc-data/         # GeoPackage OGC R/W (GPB/WKB 2D/3D), projet .sdnc.json, DXF/DXF-PCI, EDIGÉO NF Z 52-000, cadastre
│   ├── sdnc-gpu/          # Contexte Vulkan (wgpu) + shader vector_2d.wgsl + Vector2dWgpuCallback (ear-clipping & segments instanciés)
│   ├── sdnc-gnss/         # RTKLIB en Rust pur (SPP, RTK EKF, LAMBDA, PPK Fwd/Bwd, RINEX 3, RTCM3 MSM7, UBX, NTRIP, NMEA)
│   ├── sdnc-ui/           # Interface 2D tactile & desktop (egui + wgpu) : Cadastre, IGN Raster/Vecteurs, Saisie/Snapping, RTK/PPK
│   ├── sdnc-app-desktop/  # Exécutable PC Windows (.exe) & PC Linux (`sdnc-field`)
│   └── sdnc-app-android/  # Bibliothèque native Android (`arm64-v8a` + `armeabi-v7a`, pont JNI GNSS & Bluetooth SPP)
└── android/               # Projet Gradle Kotlin DSL (AndroidManifest.xml + MainActivity.kt + APK universel)
```

## Compilation & Tests

### PC Linux & Windows

```bash
cd sdnc-field-rs
cargo test --workspace
cargo run --release -p sdnc-app-desktop
```

### Android Tablettes & Téléphones (`arm64-v8a` + `armeabi-v7a`, Vulkan 1.1+)

```bash
rustup target add aarch64-linux-android armv7-linux-androideabi
cargo install cargo-ndk
cargo ndk -t arm64-v8a -t armeabi-v7a -o android/app/src/main/jniLibs build --release -p sdnc-app-android
cd android && ./gradlew assembleDebug
```
