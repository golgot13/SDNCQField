//! `sdnc-gpu` — Moteur GPU 2D Vulkan / WGPU (`egui_wgpu` + WGSL) pour SDNCQField.
//!
//! Dédié à l'affichage cartographique 2D ultra-fluide sur Android, Windows et Linux :
//! - Sélection Vulkan prioritaire (`VulkanComputeContext`)
//! - Rendu natif `egui_wgpu::CallbackTrait` (`Vector2dWgpuCallback`) des plans cadastraux denses
//!   (DXF, DXF-PCI, EDIGÉO, IGN WFS, GeoPackage) en 2 draw calls GPU par frame
//! - Triangulation ear-clipping 2D des polygones concaves (parcelles & bâtiments)

pub mod context;
pub mod vector2d_renderer;

pub use context::{GpuBackendInfo, VulkanComputeContext};
pub use vector2d_renderer::{
    pack_rgba8, triangulate_polygon_earclip, Camera2dUniform, FillVertex2d, Segment2dGpu,
    Vector2dGpuBatch, Vector2dGpuPipeline, Vector2dWgpuCallback,
};
