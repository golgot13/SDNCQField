//! `sdnc-gpu` — Moteur Vulkan Compute (`wgpu` + WGSL) pour SDNCQField.
//!
//! Fournit :
//! - Sélection automatique Vulkan (Android, Linux) et Vulkan/DX12 (Windows)
//! - Interpolation MNT IDW + calcul Pente & Ombrage (Hillshade) en compute shader WGSL
//! - Shader de projection 3D & frustum culling parallèle (`point_project.wgsl`)

pub mod context;
pub mod terrain;

pub use context::{GpuBackendInfo, VulkanComputeContext};
pub use terrain::{TerrainCellOutput, TerrainGridResult, TerrainSampleGpu, VulkanTerrainEngine};
