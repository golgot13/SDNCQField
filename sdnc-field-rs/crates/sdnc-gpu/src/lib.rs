//! `sdnc-gpu` — Moteur Vulkan Compute & Rendu GPU natif (`wgpu` + `egui_wgpu` + WGSL) pour SDNCQField.
//!
//! Fournit :
//! - Sélection automatique Vulkan (Android, Linux) et Vulkan/DX12 (Windows)
//! - Interpolation MNT IDW + calcul Pente & Ombrage (Hillshade) en compute shader WGSL
//! - Shader de projection 3D & frustum culling parallèle (`point_project.wgsl`)
//! - Rendu GPU natif `egui_wgpu::CallbackTrait` pour nuages LiDAR de plusieurs millions de points (`lidar_renderer`)

pub mod context;
pub mod lidar_renderer;
pub mod terrain;

pub use context::{GpuBackendInfo, VulkanComputeContext};
pub use lidar_renderer::{
    generate_synthetic_lidar_cloud, load_las_point_cloud, load_xyz_point_cloud, LidarCameraUniform,
    LidarCloudData, LidarGpuPipeline, LidarPointGpu, LidarWgpuCallback,
};
pub use terrain::{TerrainCellOutput, TerrainGridResult, TerrainSampleGpu, VulkanTerrainEngine};
