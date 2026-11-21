//! Moteur de calcul MNT & Hillshade sur Vulkan Compute (WGSL) avec repli CPU transparent.
//!
//! Remplace le calcul IDW CPU à chaque frame d'`AEROPHOTOGRAM` par un dispatch
//! Vulkan Compute (`terrain_idw.wgsl`) travaillant en origine flottante `f32`
//! pour la compatibilité complète Android (Mali/Adreno), Windows et Linux.

use crate::context::{GpuBackendInfo, VulkanComputeContext};
use bytemuck::{Pod, Zeroable};
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct TerrainSampleGpu {
    pub rx: f32,
    pub ry: f32,
    pub rz: f32,
    pub weight: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct TerrainUniformsGpu {
    cols: u32,
    rows: u32,
    num_samples: u32,
    pad0: u32,
    dx: f32,
    dy: f32,
    default_z: f32,
    power_bias: f32,
    sun_azimuth_rad: f32,
    sun_altitude_rad: f32,
    z_factor: f32,
    pad1: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct TerrainCellOutput {
    pub elevation: f32,
    pub slope_deg: f32,
    pub hillshade: f32,
    pub pad: f32,
}

#[derive(Debug, Clone)]
pub struct TerrainGridResult {
    pub cols: usize,
    pub rows: usize,
    pub origin_x: f64,
    pub origin_y: f64,
    pub dx: f64,
    pub dy: f64,
    pub cells: Vec<TerrainCellOutput>,
    pub computed_on_gpu: bool,
}

impl TerrainGridResult {
    pub fn sample_bilinear(&self, wx: f64, wy: f64) -> (f64, f32) {
        if self.cols < 2 || self.rows < 2 || self.cells.is_empty() {
            return (0.0, 0.8);
        }
        let gx = ((wx - self.origin_x) / self.dx).clamp(0.0, (self.cols - 2) as f64);
        let gy = ((wy - self.origin_y) / self.dy).clamp(0.0, (self.rows - 2) as f64);
        let ix = gx.floor() as usize;
        let iy = gy.floor() as usize;
        let fx = gx - ix as f64;
        let fy = gy - iy as f64;

        let c00 = self.cells[iy * self.cols + ix];
        let c10 = self.cells[iy * self.cols + ix + 1];
        let c01 = self.cells[(iy + 1) * self.cols + ix];
        let c11 = self.cells[(iy + 1) * self.cols + ix + 1];

        let z = c00.elevation as f64 * (1.0 - fx) * (1.0 - fy)
            + c10.elevation as f64 * fx * (1.0 - fy)
            + c01.elevation as f64 * (1.0 - fx) * fy
            + c11.elevation as f64 * fx * fy;
        let shade = (c00.hillshade as f64 * (1.0 - fx) * (1.0 - fy)
            + c10.hillshade as f64 * fx * (1.0 - fy)
            + c01.hillshade as f64 * (1.0 - fx) * fy
            + c11.hillshade as f64 * fx * fy) as f32;
        (z, shade)
    }
}

pub struct VulkanTerrainEngine {
    ctx: Option<VulkanComputeContext>,
    pipeline_idw: Option<wgpu::ComputePipeline>,
    pipeline_shade: Option<wgpu::ComputePipeline>,
    bind_group_layout: Option<wgpu::BindGroupLayout>,
    pub backend_info: GpuBackendInfo,
}

impl VulkanTerrainEngine {
    pub fn new() -> Self {
        match VulkanComputeContext::try_new() {
            Ok(ctx) => {
                let shader = ctx.device.create_shader_module(wgpu::ShaderModuleDescriptor {
                    label: Some("SDNC Terrain IDW + Hillshade WGSL"),
                    source: wgpu::ShaderSource::Wgsl(include_str!("shaders/terrain_idw.wgsl").into()),
                });

                let bgl = ctx
                    .device
                    .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                        label: Some("Terrain Compute BGL"),
                        entries: &[
                            wgpu::BindGroupLayoutEntry {
                                binding: 0,
                                visibility: wgpu::ShaderStages::COMPUTE,
                                ty: wgpu::BindingType::Buffer {
                                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                                    has_dynamic_offset: false,
                                    min_binding_size: None,
                                },
                                count: None,
                            },
                            wgpu::BindGroupLayoutEntry {
                                binding: 1,
                                visibility: wgpu::ShaderStages::COMPUTE,
                                ty: wgpu::BindingType::Buffer {
                                    ty: wgpu::BufferBindingType::Uniform,
                                    has_dynamic_offset: false,
                                    min_binding_size: None,
                                },
                                count: None,
                            },
                            wgpu::BindGroupLayoutEntry {
                                binding: 2,
                                visibility: wgpu::ShaderStages::COMPUTE,
                                ty: wgpu::BindingType::Buffer {
                                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                                    has_dynamic_offset: false,
                                    min_binding_size: None,
                                },
                                count: None,
                            },
                            wgpu::BindGroupLayoutEntry {
                                binding: 3,
                                visibility: wgpu::ShaderStages::COMPUTE,
                                ty: wgpu::BindingType::Buffer {
                                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                                    has_dynamic_offset: false,
                                    min_binding_size: None,
                                },
                                count: None,
                            },
                        ],
                    });

                let pl = ctx
                    .device
                    .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                        label: Some("Terrain Compute Pipeline Layout"),
                        bind_group_layouts: &[&bgl],
                        push_constant_ranges: &[],
                    });

                let p_idw = ctx
                    .device
                    .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                        label: Some("Terrain IDW Pipeline"),
                        layout: Some(&pl),
                        module: &shader,
                        entry_point: Some("interpolate_idw"),
                        compilation_options: Default::default(),
                        cache: None,
                    });

                let p_shade = ctx
                    .device
                    .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                        label: Some("Terrain Hillshade Pipeline"),
                        layout: Some(&pl),
                        module: &shader,
                        entry_point: Some("compute_hillshade"),
                        compilation_options: Default::default(),
                        cache: None,
                    });

                let info = ctx.info.clone();
                Self {
                    ctx: Some(ctx),
                    pipeline_idw: Some(p_idw),
                    pipeline_shade: Some(p_shade),
                    bind_group_layout: Some(bgl),
                    backend_info: info,
                }
            }
            Err(e) => {
                log::warn!("Vulkan Compute indisponible ({}), bascule CPU automatique.", e);
                Self {
                    ctx: None,
                    pipeline_idw: None,
                    pipeline_shade: None,
                    bind_group_layout: None,
                    backend_info: GpuBackendInfo {
                        adapter_name: "CPU Fallback".to_string(),
                        backend_name: "CPU".to_string(),
                        max_buffer_size_mb: 0,
                        is_gpu_active: false,
                    },
                }
            }
        }
    }

    /// Calcule la grille MNT + Pente + Ombrage (Hillshade) sur Vulkan Compute (ou fallback CPU).
    pub fn compute_terrain_grid(
        &self,
        origin_x: f64,
        origin_y: f64,
        dx: f64,
        dy: f64,
        cols: usize,
        rows: usize,
        default_z: f64,
        z_factor: f32,
        samples_world: &[(f64, f64, f64, f32)],
    ) -> TerrainGridResult {
        let mut gpu_samples: Vec<TerrainSampleGpu> = samples_world
            .iter()
            .map(|&(wx, wy, wz, w)| TerrainSampleGpu {
                rx: (wx - origin_x) as f32,
                ry: (wy - origin_y) as f32,
                rz: wz as f32,
                weight: w,
            })
            .collect();
        if gpu_samples.is_empty() {
            gpu_samples.push(TerrainSampleGpu {
                rx: 0.0,
                ry: 0.0,
                rz: default_z as f32,
                weight: 1.0,
            });
        }

        let uniforms = TerrainUniformsGpu {
            cols: cols as u32,
            rows: rows as u32,
            num_samples: if samples_world.is_empty() {
                0
            } else {
                gpu_samples.len() as u32
            },
            pad0: 0,
            dx: dx as f32,
            dy: dy as f32,
            default_z: default_z as f32,
            power_bias: 2.0,
            sun_azimuth_rad: 315.0f32.to_radians(),
            sun_altitude_rad: 45.0f32.to_radians(),
            z_factor,
            pad1: 0.0,
        };

        if let (Some(ctx), Some(p_idw), Some(p_shade), Some(bgl)) = (
            &self.ctx,
            &self.pipeline_idw,
            &self.pipeline_shade,
            &self.bind_group_layout,
        ) {
            if let Some(cells) =
                self.dispatch_gpu(ctx, p_idw, p_shade, bgl, &gpu_samples, uniforms)
            {
                return TerrainGridResult {
                    cols,
                    rows,
                    origin_x,
                    origin_y,
                    dx,
                    dy,
                    cells,
                    computed_on_gpu: true,
                };
            }
        }

        let cells = compute_terrain_cpu(&gpu_samples, uniforms);
        TerrainGridResult {
            cols,
            rows,
            origin_x,
            origin_y,
            dx,
            dy,
            cells,
            computed_on_gpu: false,
        }
    }

    fn dispatch_gpu(
        &self,
        ctx: &VulkanComputeContext,
        p_idw: &wgpu::ComputePipeline,
        p_shade: &wgpu::ComputePipeline,
        bgl: &wgpu::BindGroupLayout,
        samples: &[TerrainSampleGpu],
        uniforms: TerrainUniformsGpu,
    ) -> Option<Vec<TerrainCellOutput>> {
        let cell_count = (uniforms.cols * uniforms.rows) as usize;
        let heights_size = (cell_count * std::mem::size_of::<f32>()) as u64;
        let out_size = (cell_count * std::mem::size_of::<TerrainCellOutput>()) as u64;

        let sample_buf = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Terrain Samples"),
                contents: bytemuck::cast_slice(samples),
                usage: wgpu::BufferUsages::STORAGE,
            });
        let uniform_buf = ctx
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Terrain Uniforms"),
                contents: bytemuck::bytes_of(&uniforms),
                usage: wgpu::BufferUsages::UNIFORM,
            });
        let heights_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Terrain Heights"),
            size: heights_size.max(4),
            usage: wgpu::BufferUsages::STORAGE,
            mapped_at_creation: false,
        });
        let out_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Terrain Cells Out"),
            size: out_size.max(16),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let staging_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Terrain Staging"),
            size: out_size.max(16),
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bg = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Terrain Compute BG"),
            layout: bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: sample_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: uniform_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: heights_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: out_buf.as_entire_binding(),
                },
            ],
        });

        let wg_x = uniforms.cols.div_ceil(8);
        let wg_y = uniforms.rows.div_ceil(8);

        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Terrain Compute Encoder"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Terrain IDW Pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(p_idw);
            pass.set_bind_group(0, &bg, &[]);
            pass.dispatch_workgroups(wg_x, wg_y, 1);
        }
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Terrain Hillshade Pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(p_shade);
            pass.set_bind_group(0, &bg, &[]);
            pass.dispatch_workgroups(wg_x, wg_y, 1);
        }
        encoder.copy_buffer_to_buffer(&out_buf, 0, &staging_buf, 0, out_size);
        ctx.queue.submit(Some(encoder.finish()));

        let slice = staging_buf.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |res| {
            let _ = tx.send(res);
        });
        let _ = ctx.device.poll(wgpu::PollType::Wait {
            submission_index: None,
            timeout: None,
        });
        rx.recv().ok()?.ok()?;

        let data = slice.get_mapped_range();
        let cells: Vec<TerrainCellOutput> = bytemuck::cast_slice(&data).to_vec();
        drop(data);
        staging_buf.unmap();
        Some(cells)
    }
}

impl Default for VulkanTerrainEngine {
    fn default() -> Self {
        Self::new()
    }
}

fn compute_terrain_cpu(
    samples: &[TerrainSampleGpu],
    cfg: TerrainUniformsGpu,
) -> Vec<TerrainCellOutput> {
    let cols = cfg.cols as usize;
    let rows = cfg.rows as usize;
    let mut heights = vec![cfg.default_z; cols * rows];

    for gy in 0..rows {
        let wy = gy as f32 * cfg.dy;
        for gx in 0..cols {
            let wx = gx as f32 * cfg.dx;
            let mut num = 0.0f32;
            let mut den = 0.0f32;
            let mut exact = None;
            for s in samples.iter().take(cfg.num_samples as usize) {
                let ddx = s.rx - wx;
                let ddy = s.ry - wy;
                let d2 = ddx * ddx + ddy * ddy;
                if d2 < 1.0 {
                    exact = Some(s.rz);
                    break;
                }
                let w = s.weight / (d2 + cfg.power_bias);
                num += w * s.rz;
                den += w;
            }
            heights[gy * cols + gx] = exact.unwrap_or_else(|| {
                if den > 1e-6 {
                    num / den
                } else {
                    cfg.default_z
                }
            });
        }
    }

    let cos_alt = cfg.sun_altitude_rad.cos();
    let sun = [
        cfg.sun_azimuth_rad.sin() * cos_alt,
        cfg.sun_azimuth_rad.cos() * cos_alt,
        cfg.sun_altitude_rad.sin(),
    ];

    let mut out = vec![
        TerrainCellOutput {
            elevation: cfg.default_z,
            slope_deg: 0.0,
            hillshade: 0.8,
            pad: 0.0,
        };
        cols * rows
    ];

    for gy in 0..rows {
        for gx in 0..cols {
            let x_prev = gx.saturating_sub(1);
            let x_next = (gx + 1).min(cols - 1);
            let y_prev = gy.saturating_sub(1);
            let y_next = (gy + 1).min(rows - 1);

            let dz_dx = (heights[gy * cols + x_next] - heights[gy * cols + x_prev])
                / ((x_next - x_prev) as f32 * cfg.dx).max(1e-3);
            let dz_dy = (heights[y_next * cols + gx] - heights[y_prev * cols + gx])
                / ((y_next - y_prev) as f32 * cfg.dy).max(1e-3);

            let nx = -dz_dx * cfg.z_factor;
            let ny = -dz_dy * cfg.z_factor;
            let nz = 1.0f32;
            let n_len = (nx * nx + ny * ny + nz * nz).sqrt();
            let shade = ((nx / n_len) * sun[0] + (ny / n_len) * sun[1] + (nz / n_len) * sun[2])
                .clamp(0.08, 1.0);
            let slope_deg = (dz_dx * dz_dx + dz_dy * dz_dy).sqrt().atan().to_degrees();

            out[gy * cols + gx] = TerrainCellOutput {
                elevation: heights[gy * cols + gx],
                slope_deg,
                hillshade: shade,
                pad: 0.0,
            };
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_terrain_idw_and_hillshade() {
        let engine = VulkanTerrainEngine::new();
        let samples = vec![
            (0.0, 0.0, 100.0, 1.0),
            (100.0, 0.0, 120.0, 1.0),
            (0.0, 100.0, 110.0, 1.0),
            (100.0, 100.0, 150.0, 1.0),
        ];
        let grid = engine.compute_terrain_grid(0.0, 0.0, 10.0, 10.0, 11, 11, 100.0, 1.5, &samples);
        assert_eq!(grid.cells.len(), 121);
        let (z_mid, shade_mid) = grid.sample_bilinear(50.0, 50.0);
        assert!(z_mid > 110.0 && z_mid < 135.0, "z_mid = {}", z_mid);
        assert!(shade_mid >= 0.08 && shade_mid <= 1.0);
    }
}
