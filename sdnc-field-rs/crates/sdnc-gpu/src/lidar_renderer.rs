//! Rendu GPU natif `egui_wgpu::CallbackTrait` pour nuages de points LiDAR massifs (millions de points).
//!
//! Fonctionnement :
//! - Un unique buffer `STORAGE` persistant en VRAM (16 octets/point, soit 16 Mo pour 1 million de points)
//! - Upload GPU incrémental déclenché uniquement lorsque `generation` change (zéro copie CPU->GPU pendant l'orbite)
//! - Rendu en un seul `draw(0..6, 0..point_count)` dans le RenderPass Vulkan/DX12 d'`egui`

use bytemuck::{Pod, Zeroable};
use egui_wgpu::wgpu;
use std::sync::Arc;
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct LidarPointGpu {
    pub rx: f32,
    pub ry: f32,
    pub rz: f32,
    pub rgba: u32,
}

impl LidarPointGpu {
    pub fn pack_rgba(r: u8, g: u8, b: u8, a: u8) -> u32 {
        (r as u32) | ((g as u32) << 8) | ((b as u32) << 16) | ((a as u32) << 24)
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct LidarCameraUniform {
    pub cos_y: f32,
    pub sin_y: f32,
    pub cos_p: f32,
    pub sin_p: f32,
    pub z_exag: f32,
    pub view_scale: f32,
    pub focal: f32,
    pub depth_bias: f32,
    pub viewport_w: f32,
    pub viewport_h: f32,
    pub pan_x: f32,
    pub pan_y: f32,
    pub point_size_px: f32,
    pub origin_shift_x: f32,
    pub origin_shift_y: f32,
    pub origin_shift_z: f32,
}

#[derive(Debug, Clone)]
pub struct LidarCloudData {
    pub origin_xyz: [f64; 3],
    pub points: Arc<Vec<LidarPointGpu>>,
    pub generation: u64,
}

impl Default for LidarCloudData {
    fn default() -> Self {
        Self {
            origin_xyz: [0.0; 3],
            points: Arc::new(Vec::new()),
            generation: 0,
        }
    }
}

/// Ressources Vulkan/WGPU persistantes stockées dans `egui_wgpu::CallbackResources`.
pub struct LidarGpuPipeline {
    pipeline: wgpu::RenderPipeline,
    bind_group_layout: wgpu::BindGroupLayout,
    uniform_buffer: wgpu::Buffer,
    point_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    uploaded_generation: u64,
    uploaded_count: u32,
}

impl LidarGpuPipeline {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("SDNC LiDAR GPU Render Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/lidar_render.wgsl").into()),
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("SDNC LiDAR BGL"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("SDNC LiDAR Pipeline Layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("SDNC LiDAR Render Pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let dummy_pt = LidarPointGpu {
            rx: 0.0,
            ry: 0.0,
            rz: 0.0,
            rgba: 0,
        };
        let point_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("SDNC LiDAR Points VRAM"),
            contents: bytemuck::bytes_of(&dummy_pt),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("SDNC LiDAR Camera Uniform"),
            size: std::mem::size_of::<LidarCameraUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group = Self::create_bind_group(
            device,
            &bind_group_layout,
            &point_buffer,
            &uniform_buffer,
        );

        Self {
            pipeline,
            bind_group_layout,
            uniform_buffer,
            point_buffer,
            bind_group,
            uploaded_generation: 0,
            uploaded_count: 0,
        }
    }

    fn create_bind_group(
        device: &wgpu::Device,
        layout: &wgpu::BindGroupLayout,
        point_buffer: &wgpu::Buffer,
        uniform_buffer: &wgpu::Buffer,
    ) -> wgpu::BindGroup {
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("SDNC LiDAR BG"),
            layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: point_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: uniform_buffer.as_entire_binding(),
                },
            ],
        })
    }

    fn ensure_cloud_uploaded(&mut self, device: &wgpu::Device, cloud: &LidarCloudData) {
        if self.uploaded_generation == cloud.generation {
            return;
        }
        if cloud.points.is_empty() {
            self.uploaded_count = 0;
            self.uploaded_generation = cloud.generation;
            return;
        }
        self.point_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("SDNC LiDAR Points VRAM"),
            contents: bytemuck::cast_slice(&cloud.points),
            usage: wgpu::BufferUsages::STORAGE,
        });
        self.bind_group = Self::create_bind_group(
            device,
            &self.bind_group_layout,
            &self.point_buffer,
            &self.uniform_buffer,
        );
        self.uploaded_count = cloud.points.len().min(u32::MAX as usize) as u32;
        self.uploaded_generation = cloud.generation;
    }
}

/// Callback natif `egui_wgpu` exécuté directement dans le RenderPass Vulkan d'`egui`.
pub struct LidarWgpuCallback {
    pub cloud: LidarCloudData,
    pub camera: LidarCameraUniform,
    pub target_format: wgpu::TextureFormat,
}

impl egui_wgpu::CallbackTrait for LidarWgpuCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen_descriptor: &egui_wgpu::ScreenDescriptor,
        _egui_encoder: &mut wgpu::CommandEncoder,
        callback_resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let res = callback_resources
            .entry::<LidarGpuPipeline>()
            .or_insert_with(|| LidarGpuPipeline::new(device, self.target_format));

        res.ensure_cloud_uploaded(device, &self.cloud);
        queue.write_buffer(&res.uniform_buffer, 0, bytemuck::bytes_of(&self.camera));
        Vec::new()
    }

    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        callback_resources: &egui_wgpu::CallbackResources,
    ) {
        if let Some(res) = callback_resources.get::<LidarGpuPipeline>() {
            if res.uploaded_count > 0 {
                render_pass.set_pipeline(&res.pipeline);
                render_pass.set_bind_group(0, &res.bind_group, &[]);
                render_pass.draw(0..6, 0..res.uploaded_count);
            }
        }
    }
}

/// Génère un nuage de points LiDAR synthétique haute densité (colline + talus + végétation)
/// pour tester le rendu GPU Vulkan natif jusqu'à plusieurs millions de points.
pub fn generate_synthetic_lidar_cloud(
    origin_x: f64,
    origin_y: f64,
    base_z: f64,
    span_m: f32,
    num_points: usize,
    generation: u64,
) -> LidarCloudData {
    let mut pts = Vec::with_capacity(num_points);
    let side = (num_points as f64).sqrt().ceil() as usize;
    let step = span_m / (side.max(2) - 1) as f32;
    let half = span_m * 0.5;

    for idx in 0..num_points {
        let gx = idx % side;
        let gy = idx / side;
        // Hash déterministe rapide pour le jitter sub-métrique
        let h = (idx as u64).wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let jx = (((h >> 16) & 0xFFFF) as f32 / 65535.0 - 0.5) * step;
        let jy = (((h >> 32) & 0xFFFF) as f32 / 65535.0 - 0.5) * step;

        let rx = -half + gx as f32 * step + jx;
        let ry = -half + gy as f32 * step + jy;
        let ridge = 18.0 * (rx * 0.018).sin() * (ry * 0.015).cos()
            + 8.0 * ((rx * 0.04 + ry * 0.03).cos());
        let is_canopy = ((h >> 48) & 0xFF) > 225;
        let rz = ridge + if is_canopy { 6.5 } else { 0.0 };

        let t = ((rz + 25.0) / 55.0).clamp(0.0, 1.0);
        let (r, g, b) = if is_canopy {
            (40u8, 195u8, 85u8)
        } else if t < 0.5 {
            let u = t * 2.0;
            ((50.0 + 160.0 * u) as u8, (130.0 + 70.0 * u) as u8, (180.0 - 110.0 * u) as u8)
        } else {
            let u = (t - 0.5) * 2.0;
            ((210.0 + 40.0 * u) as u8, (200.0 - 80.0 * u) as u8, (70.0 + 120.0 * u) as u8)
        };

        pts.push(LidarPointGpu {
            rx,
            ry,
            rz,
            rgba: LidarPointGpu::pack_rgba(r, g, b, 235),
        });
    }

    LidarCloudData {
        origin_xyz: [origin_x, origin_y, base_z],
        points: Arc::new(pts),
        generation,
    }
}

/// Charge un nuage de points texte `.xyz` / `.txt` / `.csv` (`X Y Z [R G B]`) en origine flottante `f32`.
pub fn load_xyz_point_cloud<P: AsRef<std::path::Path>>(
    path: P,
    generation: u64,
) -> anyhow::Result<LidarCloudData> {
    let content = std::fs::read_to_string(path)?;
    let mut raw = Vec::new();
    for line in content.lines() {
        let parts: Vec<f64> = line
            .split(|c: char| c.is_whitespace() || c == ',' || c == ';')
            .filter(|s| !s.is_empty())
            .filter_map(|s| s.parse().ok())
            .collect();
        if parts.len() >= 3 {
            let (r, g, b) = if parts.len() >= 6 {
                (parts[3] as u8, parts[4] as u8, parts[5] as u8)
            } else {
                (90, 210, 255)
            };
            raw.push((parts[0], parts[1], parts[2], r, g, b));
        }
    }
    if raw.is_empty() {
        return Ok(LidarCloudData::default());
    }
    let ox = raw.iter().map(|p| p.0).sum::<f64>() / raw.len() as f64;
    let oy = raw.iter().map(|p| p.1).sum::<f64>() / raw.len() as f64;
    let oz = raw.iter().map(|p| p.2).sum::<f64>() / raw.len() as f64;

    let points: Vec<LidarPointGpu> = raw
        .into_iter()
        .map(|(x, y, z, r, g, b)| LidarPointGpu {
            rx: (x - ox) as f32,
            ry: (y - oy) as f32,
            rz: (z - oz) as f32,
            rgba: LidarPointGpu::pack_rgba(r, g, b, 240),
        })
        .collect();

    Ok(LidarCloudData {
        origin_xyz: [ox, oy, oz],
        points: Arc::new(points),
        generation,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_massive_lidar_cloud_memory_layout() {
        assert_eq!(std::mem::size_of::<LidarPointGpu>(), 16);
        assert_eq!(std::mem::size_of::<LidarCameraUniform>(), 64);
        let cloud = generate_synthetic_lidar_cloud(894500.0, 6269500.0, 245.0, 400.0, 250_000, 1);
        assert_eq!(cloud.points.len(), 250_000);
        assert_eq!(cloud.generation, 1);
    }
}
