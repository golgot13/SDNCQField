//! Contexte GPU Vulkan / WGPU multiplateforme (Android, Linux, Windows).
//!
//! Inspiré de `aerophotogram-compute::compute::wgpu_context` :
//! - Sélectionne Vulkan en priorité sur Linux et Android (et Vulkan/DX12 sur Windows)
//! - Expose un mode fallback CPU transparent si aucun GPU Vulkan n'est disponible

use anyhow::Result;
use std::sync::Arc;
use wgpu::{
    Backends, Device, DeviceDescriptor, Features, Instance, InstanceDescriptor, InstanceFlags,
    PowerPreference, Queue, RequestAdapterOptions,
};

#[derive(Debug, Clone)]
pub struct GpuBackendInfo {
    pub adapter_name: String,
    pub backend_name: String,
    pub max_buffer_size_mb: u64,
    pub is_gpu_active: bool,
}

#[derive(Debug, Clone)]
pub struct VulkanComputeContext {
    pub device: Arc<Device>,
    pub queue: Arc<Queue>,
    pub info: GpuBackendInfo,
}

impl VulkanComputeContext {
    /// Tente d'initialiser un contexte Vulkan Compute (`wgpu`).
    pub fn try_new() -> Result<Self> {
        pollster::block_on(Self::new_async())
    }

    pub async fn new_async() -> Result<Self> {
        let flags = InstanceFlags::default()
            .difference(InstanceFlags::VALIDATION)
            .difference(InstanceFlags::DEBUG);

        // Vulkan prioritaire sur Android et Linux ; Vulkan puis DX12 sur Windows
        let backend_order: &[Backends] = if cfg!(target_os = "windows") {
            &[Backends::VULKAN, Backends::DX12, Backends::PRIMARY]
        } else {
            &[Backends::VULKAN, Backends::PRIMARY]
        };

        let mut chosen = None;
        for &backends in backend_order {
            let instance = Instance::new(&InstanceDescriptor {
                backends,
                flags,
                ..Default::default()
            });
            if let Ok(adapter) = instance
                .request_adapter(&RequestAdapterOptions {
                    power_preference: PowerPreference::HighPerformance,
                    compatible_surface: None,
                    force_fallback_adapter: false,
                })
                .await
            {
                chosen = Some(adapter);
                break;
            }
        }

        let adapter = chosen.ok_or_else(|| {
            anyhow::anyhow!("Aucun adaptateur GPU Vulkan/WGPU disponible sur cette machine")
        })?;

        let adapter_info = adapter.get_info();
        let limits = adapter.limits();
        let (device, queue) = adapter
            .request_device(&DeviceDescriptor {
                label: Some("SDNCQField Vulkan Compute Device"),
                required_features: Features::empty(),
                required_limits: limits.clone(),
                ..Default::default()
            })
            .await?;

        let info = GpuBackendInfo {
            adapter_name: adapter_info.name,
            backend_name: format!("{:?}", adapter_info.backend),
            max_buffer_size_mb: limits.max_buffer_size / (1024 * 1024),
            is_gpu_active: true,
        };

        log::info!(
            "SDNC GPU initialisé : {} ({:?}, max_buffer={} Mo)",
            info.adapter_name,
            info.backend_name,
            info.max_buffer_size_mb
        );

        Ok(Self {
            device: Arc::new(device),
            queue: Arc::new(queue),
            info,
        })
    }
}
