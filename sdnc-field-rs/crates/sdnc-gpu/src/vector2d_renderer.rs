//! Moteur de rendu vectoriel 2D natif GPU (`egui_wgpu::CallbackTrait`) pour plans cadastraux denses.
//!
//! Conçu pour afficher à 60–120 FPS des plans **DXF, DXF-PCI, EDIGÉO, IGN WFS et GeoPackage**
//! contenant des centaines de milliers de parcelles, bâtiments et limites :
//! - **Triangulation ear-clipping 2D** des polygones concaves (bâtiments en L/U, parcelles complexes)
//! - **Buffers VRAM persistants** en origine flottante `f32` (upload uniquement sur changement de `generation`)
//! - **2 draw calls GPU par frame** (1 draw call polygones remplis + 1 draw call instancié pour tous les segments)

use bytemuck::{Pod, Zeroable};
use egui_wgpu::wgpu;
use std::sync::Arc;
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct Camera2dUniform {
    pub shift_x: f32,
    pub shift_y: f32,
    pub scale: f32,
    pub pad0: f32,
    pub viewport_w: f32,
    pub viewport_h: f32,
    pub pan_x: f32,
    pub pan_y: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct FillVertex2d {
    pub x: f32,
    pub y: f32,
    pub rgba: u32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct Segment2dGpu {
    pub x0: f32,
    pub y0: f32,
    pub x1: f32,
    pub y1: f32,
    pub width_px: f32,
    pub rgba: u32,
    pub pad0: u32,
    pub pad1: u32,
}

#[inline]
pub fn pack_rgba8(rgba: [u8; 4]) -> u32 {
    (rgba[0] as u32)
        | ((rgba[1] as u32) << 8)
        | ((rgba[2] as u32) << 16)
        | ((rgba[3] as u32) << 24)
}

#[derive(Debug, Clone)]
pub struct Vector2dGpuBatch {
    pub origin_xy: [f64; 2],
    pub fill_vertices: Arc<Vec<FillVertex2d>>,
    pub segments: Arc<Vec<Segment2dGpu>>,
    pub generation: u64,
}

impl Default for Vector2dGpuBatch {
    fn default() -> Self {
        Self {
            origin_xy: [0.0, 0.0],
            fill_vertices: Arc::new(Vec::new()),
            segments: Arc::new(Vec::new()),
            generation: 0,
        }
    }
}

/// Ressources Vulkan/WGPU persistantes stockées dans `egui_wgpu::CallbackResources`.
pub struct Vector2dGpuPipeline {
    fill_pipeline: wgpu::RenderPipeline,
    seg_pipeline: wgpu::RenderPipeline,
    seg_bgl: wgpu::BindGroupLayout,
    cam_uniform_buf: wgpu::Buffer,
    cam_bind_group: wgpu::BindGroup,
    fill_vbo: wgpu::Buffer,
    seg_storage_buf: wgpu::Buffer,
    seg_bind_group: wgpu::BindGroup,
    uploaded_generation: u64,
    fill_vertex_count: u32,
    segment_count: u32,
}

impl Vector2dGpuPipeline {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("SDNC Vector 2D Cadastre WGSL"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/vector_2d.wgsl").into()),
        });

        let cam_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Vector2D Camera BGL"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let seg_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Vector2D Segments BGL"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let fill_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Vector2D Fill Layout"),
            bind_group_layouts: &[&cam_bgl],
            push_constant_ranges: &[],
        });

        let seg_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Vector2D Seg Layout"),
            bind_group_layouts: &[&cam_bgl, &seg_bgl],
            push_constant_ranges: &[],
        });

        let fill_vbo_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<FillVertex2d>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x2,
                },
                wgpu::VertexAttribute {
                    offset: 8,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Uint32,
                },
            ],
        };

        let fill_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Vector2D Fill Pipeline"),
            layout: Some(&fill_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_fill"),
                buffers: &[fill_vbo_layout],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_fill"),
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

        let seg_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Vector2D Segment Pipeline"),
            layout: Some(&seg_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_seg"),
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_seg"),
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

        let cam_uniform_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Vector2D Camera Uniform"),
            size: std::mem::size_of::<Camera2dUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let cam_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Vector2D Camera BG"),
            layout: &cam_bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: cam_uniform_buf.as_entire_binding(),
            }],
        });

        let dummy_fill = FillVertex2d {
            x: 0.0,
            y: 0.0,
            rgba: 0,
        };
        let fill_vbo = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Vector2D Fill VBO"),
            contents: bytemuck::bytes_of(&dummy_fill),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let dummy_seg = Segment2dGpu {
            x0: 0.0,
            y0: 0.0,
            x1: 0.0,
            y1: 0.0,
            width_px: 1.0,
            rgba: 0,
            pad0: 0,
            pad1: 0,
        };
        let seg_storage_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Vector2D Segments VRAM"),
            contents: bytemuck::bytes_of(&dummy_seg),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let seg_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Vector2D Segments BG"),
            layout: &seg_bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: seg_storage_buf.as_entire_binding(),
            }],
        });

        Self {
            fill_pipeline,
            seg_pipeline,
            seg_bgl,
            cam_uniform_buf,
            cam_bind_group,
            fill_vbo,
            seg_storage_buf,
            seg_bind_group,
            uploaded_generation: 0,
            fill_vertex_count: 0,
            segment_count: 0,
        }
    }

    fn ensure_uploaded(&mut self, device: &wgpu::Device, batch: &Vector2dGpuBatch) {
        if self.uploaded_generation == batch.generation {
            return;
        }
        if batch.fill_vertices.is_empty() {
            self.fill_vertex_count = 0;
        } else {
            self.fill_vbo = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Vector2D Fill VBO"),
                contents: bytemuck::cast_slice(&batch.fill_vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });
            self.fill_vertex_count = batch.fill_vertices.len().min(u32::MAX as usize) as u32;
        }

        if batch.segments.is_empty() {
            self.segment_count = 0;
        } else {
            self.seg_storage_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Vector2D Segments VRAM"),
                contents: bytemuck::cast_slice(&batch.segments),
                usage: wgpu::BufferUsages::STORAGE,
            });
            self.seg_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("Vector2D Segments BG"),
                layout: &self.seg_bgl,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: self.seg_storage_buf.as_entire_binding(),
                }],
            });
            self.segment_count = batch.segments.len().min(u32::MAX as usize) as u32;
        }
        self.uploaded_generation = batch.generation;
    }
}

/// Callback natif `egui_wgpu` rendant l'ensemble des couches vectorielles et cadastrales 2D sur Vulkan.
pub struct Vector2dWgpuCallback {
    pub batch: Vector2dGpuBatch,
    pub camera: Camera2dUniform,
    pub target_format: wgpu::TextureFormat,
}

impl egui_wgpu::CallbackTrait for Vector2dWgpuCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen_descriptor: &egui_wgpu::ScreenDescriptor,
        _egui_encoder: &mut wgpu::CommandEncoder,
        callback_resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let pipe = callback_resources
            .entry::<Vector2dGpuPipeline>()
            .or_insert_with(|| Vector2dGpuPipeline::new(device, self.target_format));
        pipe.ensure_uploaded(device, &self.batch);
        queue.write_buffer(&pipe.cam_uniform_buf, 0, bytemuck::bytes_of(&self.camera));
        Vec::new()
    }

    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        callback_resources: &egui_wgpu::CallbackResources,
    ) {
        if let Some(pipe) = callback_resources.get::<Vector2dGpuPipeline>() {
            if pipe.fill_vertex_count >= 3 {
                render_pass.set_pipeline(&pipe.fill_pipeline);
                render_pass.set_bind_group(0, &pipe.cam_bind_group, &[]);
                render_pass.set_vertex_buffer(0, pipe.fill_vbo.slice(..));
                render_pass.draw(0..pipe.fill_vertex_count, 0..1);
            }
            if pipe.segment_count > 0 {
                render_pass.set_pipeline(&pipe.seg_pipeline);
                render_pass.set_bind_group(0, &pipe.cam_bind_group, &[]);
                render_pass.set_bind_group(1, &pipe.seg_bind_group, &[]);
                render_pass.draw(0..6, 0..pipe.segment_count);
            }
        }
    }
}

/// Triangulation ear-clipping 2D pour polygones simples (convexes ET concaves : bâtiments en L/U, parcelles).
/// Ajoute les sommets triangulés dans `out` au format `FillVertex2d`.
pub fn triangulate_polygon_earclip(ring: &[(f32, f32)], rgba: u32, out: &mut Vec<FillVertex2d>) {
    let mut pts: Vec<(f32, f32)> = ring.to_vec();
    // Supprimer le dernier point s'il referme le premier
    if pts.len() >= 2 {
        let first = pts[0];
        let last = pts[pts.len() - 1];
        if (first.0 - last.0).abs() < 1e-4 && (first.1 - last.1).abs() < 1e-4 {
            pts.pop();
        }
    }
    if pts.len() < 3 {
        return;
    }

    // Assurer l'orientation anti-horaire (CCW)
    let mut signed_area2 = 0.0f64;
    for i in 0..pts.len() {
        let j = (i + 1) % pts.len();
        signed_area2 +=
            (pts[i].0 as f64) * (pts[j].1 as f64) - (pts[j].0 as f64) * (pts[i].1 as f64);
    }
    if signed_area2 < 0.0 {
        pts.reverse();
    }

    let mut idx: Vec<usize> = (0..pts.len()).collect();
    let mut guard = 0usize;
    let max_iter = pts.len() * pts.len() + 16;

    while idx.len() > 3 && guard < max_iter {
        guard += 1;
        let m = idx.len();
        let mut ear_found = false;

        for i in 0..m {
            let prev = pts[idx[(i + m - 1) % m]];
            let curr = pts[idx[i]];
            let next = pts[idx[(i + 1) % m]];

            let cross = (curr.0 - prev.0) * (next.1 - prev.1) - (curr.1 - prev.1) * (next.0 - prev.0);
            if cross <= 1e-6 {
                continue;
            }

            // Vérifier qu'aucun autre sommet ne se trouve dans le triangle (prev, curr, next)
            let mut contains_other = false;
            for &other_i in &idx {
                if other_i == idx[(i + m - 1) % m]
                    || other_i == idx[i]
                    || other_i == idx[(i + 1) % m]
                {
                    continue;
                }
                if point_in_triangle(pts[other_i], prev, curr, next) {
                    contains_other = true;
                    break;
                }
            }

            if !contains_other {
                out.push(FillVertex2d { x: prev.0, y: prev.1, rgba });
                out.push(FillVertex2d { x: curr.0, y: curr.1, rgba });
                out.push(FillVertex2d { x: next.0, y: next.1, rgba });
                idx.remove(i);
                ear_found = true;
                break;
            }
        }

        if !ear_found {
            // Repli fan si polygone dégénéré / auto-intersectant
            for k in 1..(idx.len() - 1) {
                let p0 = pts[idx[0]];
                let p1 = pts[idx[k]];
                let p2 = pts[idx[k + 1]];
                out.push(FillVertex2d { x: p0.0, y: p0.1, rgba });
                out.push(FillVertex2d { x: p1.0, y: p1.1, rgba });
                out.push(FillVertex2d { x: p2.0, y: p2.1, rgba });
            }
            return;
        }
    }

    if idx.len() == 3 {
        for &k in &idx {
            let p = pts[k];
            out.push(FillVertex2d { x: p.0, y: p.1, rgba });
        }
    }
}

#[inline]
fn point_in_triangle(p: (f32, f32), a: (f32, f32), b: (f32, f32), c: (f32, f32)) -> bool {
    let d1 = (p.0 - b.0) * (a.1 - b.1) - (a.0 - b.0) * (p.1 - b.1);
    let d2 = (p.0 - c.0) * (b.1 - c.1) - (b.0 - c.0) * (p.1 - c.1);
    let d3 = (p.0 - a.0) * (c.1 - a.1) - (c.0 - a.0) * (p.1 - a.1);
    let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);
    !(has_neg && has_pos)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gpu_struct_alignments() {
        assert_eq!(std::mem::size_of::<Camera2dUniform>(), 32);
        assert_eq!(std::mem::size_of::<FillVertex2d>(), 12);
        assert_eq!(std::mem::size_of::<Segment2dGpu>(), 32);
    }

    #[test]
    fn test_earclip_concave_l_shaped_building() {
        // Bâtiment cadastral concave en forme de "L" (6 sommets -> 4 triangles = 12 sommets)
        let l_shape = vec![
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 4.0),
            (4.0, 4.0),
            (4.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0),
        ];
        let mut verts = Vec::new();
        triangulate_polygon_earclip(&l_shape, pack_rgba8([230, 180, 90, 200]), &mut verts);
        assert_eq!(verts.len(), 12);
        // Vérifie que la somme des aires des triangles vaut exactement l'aire du L (10*4 + 4*6 = 64)
        let mut area = 0.0f32;
        for tri in verts.chunks_exact(3) {
            let (a, b, c) = (tri[0], tri[1], tri[2]);
            area += 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs();
        }
        assert!((area - 64.0).abs() < 1e-3, "Aire attendue 64.0, obtenue {}", area);
    }
}
