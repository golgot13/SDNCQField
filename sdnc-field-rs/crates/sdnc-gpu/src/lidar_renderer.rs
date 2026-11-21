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

/// Couleur standard ASPRS pour la classification LiDAR HD IGN (Sol, Végétation, Bâti, Eau, Ponts).
fn asprs_class_rgb(class_code: u8, norm_z: f32) -> (u8, u8, u8) {
    match class_code {
        2 => (165, 130, 90),  // Sol (Ground)
        3 => (110, 185, 80),  // Végétation basse
        4 => (60, 165, 65),   // Végétation moyenne
        5 => (30, 130, 50),   // Végétation haute
        6 => (225, 95, 65),   // Bâtiment
        9 => (50, 145, 235),  // Eau
        17 => (205, 195, 175), // Pont
        _ => {
            let t = norm_z.clamp(0.0, 1.0);
            (
                (60.0 + 185.0 * t) as u8,
                (140.0 + 80.0 * (1.0 - (t - 0.5).abs() * 2.0)) as u8,
                (220.0 - 150.0 * t) as u8,
            )
        }
    }
}

/// Lecteur binaire pur Rust **ASPRS LAS 1.2 / 1.3 / 1.4** (formats 0, 1, 2, 3, 6, 7, 8)
/// conçu pour charger directement les dalles **LiDAR HD IGN** en VRAM (`LidarCloudData`).
pub fn load_las_point_cloud<P: AsRef<std::path::Path>>(
    path: P,
    max_points: usize,
    generation: u64,
) -> anyhow::Result<LidarCloudData> {
    let bytes = std::fs::read(path.as_ref())?;
    if bytes.len() < 227 || &bytes[0..4] != b"LASF" {
        anyhow::bail!("En-tête binaire ASPRS LASF invalide");
    }

    let ver_major = bytes[24];
    let ver_minor = bytes[25];
    let offset_to_points = u32::from_le_bytes(bytes[96..100].try_into()?) as usize;
    let raw_format = bytes[104];
    if (raw_format & 0x80) != 0 {
        anyhow::bail!("Fichier compressé LAZ détecté : décompressez-le en .las non compressé");
    }
    let fmt = raw_format & 0x3F;
    let rec_len = u16::from_le_bytes(bytes[105..107].try_into()?) as usize;
    if rec_len < 20 {
        anyhow::bail!("Longueur d'enregistrement LAS invalide: {}", rec_len);
    }

    let legacy_count = u32::from_le_bytes(bytes[107..111].try_into()?) as u64;
    let total_points = if ver_major == 1 && ver_minor >= 4 && bytes.len() >= 255 {
        let ext = u64::from_le_bytes(bytes[247..255].try_into()?);
        if ext > 0 { ext } else { legacy_count }
    } else {
        legacy_count
    } as usize;

    let read_f64 = |off: usize| -> anyhow::Result<f64> {
        Ok(f64::from_le_bytes(bytes[off..off + 8].try_into()?))
    };
    let scale_x = read_f64(131)?;
    let scale_y = read_f64(139)?;
    let scale_z = read_f64(147)?;
    let off_x = read_f64(155)?;
    let off_y = read_f64(163)?;
    let off_z = read_f64(171)?;
    let max_x = read_f64(179)?;
    let min_x = read_f64(187)?;
    let max_y = read_f64(195)?;
    let min_y = read_f64(203)?;
    let max_z = read_f64(211)?;
    let min_z = read_f64(219)?;

    let ox = (min_x + max_x) * 0.5;
    let oy = (min_y + max_y) * 0.5;
    let oz = (min_z + max_z) * 0.5;
    let z_span = (max_z - min_z).max(1.0);

    let avail_records = bytes.len().saturating_sub(offset_to_points) / rec_len;
    let count = total_points.min(avail_records);
    let stride = if max_points > 0 && count > max_points {
        count / max_points
    } else {
        1
    }
    .max(1);

    let rgb_offset: Option<usize> = match fmt {
        2 => Some(20),
        3 => Some(28),
        7 | 8 => Some(30),
        _ => None,
    };
    let class_offset: usize = if fmt >= 6 { 16 } else { 15 };

    let mut points = Vec::with_capacity(count / stride + 1);
    for i in (0..count).step_by(stride) {
        let base = offset_to_points + i * rec_len;
        let ix = i32::from_le_bytes(bytes[base..base + 4].try_into()?);
        let iy = i32::from_le_bytes(bytes[base + 4..base + 8].try_into()?);
        let iz = i32::from_le_bytes(bytes[base + 8..base + 12].try_into()?);

        let wx = ix as f64 * scale_x + off_x;
        let wy = iy as f64 * scale_y + off_y;
        let wz = iz as f64 * scale_z + off_z;

        let mut rgb_opt = None;
        if let Some(roff) = rgb_offset {
            if roff + 6 <= rec_len {
                let r16 = u16::from_le_bytes(bytes[base + roff..base + roff + 2].try_into()?);
                let g16 = u16::from_le_bytes(bytes[base + roff + 2..base + roff + 4].try_into()?);
                let b16 = u16::from_le_bytes(bytes[base + roff + 4..base + roff + 6].try_into()?);
                if (r16 | g16 | b16) != 0 {
                    let r8 = if r16 > 255 { (r16 >> 8) as u8 } else { r16 as u8 };
                    let g8 = if g16 > 255 { (g16 >> 8) as u8 } else { g16 as u8 };
                    let b8 = if b16 > 255 { (b16 >> 8) as u8 } else { b16 as u8 };
                    rgb_opt = Some((r8, g8, b8));
                }
            }
        }

        let (r, g, b) = rgb_opt.unwrap_or_else(|| {
            let raw_cls = bytes.get(base + class_offset).copied().unwrap_or(0);
            let cls = if fmt >= 6 { raw_cls } else { raw_cls & 0x1F };
            let norm_z = ((wz - min_z) / z_span) as f32;
            asprs_class_rgb(cls, norm_z)
        });

        points.push(LidarPointGpu {
            rx: (wx - ox) as f32,
            ry: (wy - oy) as f32,
            rz: (wz - oz) as f32,
            rgba: LidarPointGpu::pack_rgba(r, g, b, 245),
        });
    }

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

    #[test]
    fn test_las_12_binary_reader_roundtrip() {
        let tmp = std::env::temp_dir().join("sdnc_test_lidar_hd.las");
        // Construit un fichier binaire LAS 1.2 Format 2 (XYZ + Classification + RGB16)
        let mut buf = vec![0u8; 227];
        buf[0..4].copy_from_slice(b"LASF");
        buf[24] = 1; // Major 1
        buf[25] = 2; // Minor 2
        buf[94..96].copy_from_slice(&227u16.to_le_bytes()); // Header size
        buf[96..100].copy_from_slice(&227u32.to_le_bytes()); // Offset to point data
        buf[104] = 2; // Point Data Format 2 (26 bytes with RGB)
        buf[105..107].copy_from_slice(&26u16.to_le_bytes());
        buf[107..111].copy_from_slice(&2u32.to_le_bytes()); // 2 points

        // Scale = 0.001 m, Offset = Lambert-93 Aix/Meyreuil
        buf[131..139].copy_from_slice(&0.001f64.to_le_bytes());
        buf[139..147].copy_from_slice(&0.001f64.to_le_bytes());
        buf[147..155].copy_from_slice(&0.001f64.to_le_bytes());
        buf[155..163].copy_from_slice(&894_500.0f64.to_le_bytes());
        buf[163..171].copy_from_slice(&6_269_500.0f64.to_le_bytes());
        buf[171..179].copy_from_slice(&240.0f64.to_le_bytes());

        // Bounds Max/Min
        buf[179..187].copy_from_slice(&894_510.0f64.to_le_bytes());
        buf[187..195].copy_from_slice(&894_500.0f64.to_le_bytes());
        buf[195..203].copy_from_slice(&6_269_520.0f64.to_le_bytes());
        buf[203..211].copy_from_slice(&6_269_500.0f64.to_le_bytes());
        buf[211..219].copy_from_slice(&250.0f64.to_le_bytes());
        buf[219..227].copy_from_slice(&240.0f64.to_le_bytes());

        // Point 1: +0m, +0m, +0m, class=6 (Building), RGB=(0,0,0 -> uses class color)
        let mut p1 = vec![0u8; 26];
        p1[15] = 6;
        buf.extend_from_slice(&p1);

        // Point 2: +10m, +20m, +10m, RGB16=(255<<8, 128<<8, 64<<8)
        let mut p2 = vec![0u8; 26];
        p2[0..4].copy_from_slice(&10_000i32.to_le_bytes());
        p2[4..8].copy_from_slice(&20_000i32.to_le_bytes());
        p2[8..12].copy_from_slice(&10_000i32.to_le_bytes());
        p2[20..22].copy_from_slice(&(255u16 << 8).to_le_bytes());
        p2[22..24].copy_from_slice(&(128u16 << 8).to_le_bytes());
        p2[24..26].copy_from_slice(&(64u16 << 8).to_le_bytes());
        buf.extend_from_slice(&p2);

        std::fs::write(&tmp, &buf).unwrap();
        let cloud = load_las_point_cloud(&tmp, 1_000_000, 7).unwrap();
        assert_eq!(cloud.points.len(), 2);
        assert_eq!(cloud.generation, 7);
        assert!((cloud.origin_xyz[0] - 894_505.0).abs() < 1e-6);
        assert!((cloud.points[0].rx - (-5.0)).abs() < 1e-3);
        assert!((cloud.points[1].rx - 5.0).abs() < 1e-3);
        assert_eq!(cloud.points[1].rgba & 0xFF, 255);
        assert_eq!((cloud.points[1].rgba >> 8) & 0xFF, 128);
        let _ = std::fs::remove_file(&tmp);
    }
}
