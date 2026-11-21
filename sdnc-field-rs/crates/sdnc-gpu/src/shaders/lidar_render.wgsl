// Shader GPU natif Vulkan/WGPU — Rendu instancié de nuages LiDAR massifs (millions de points)
// Intégré directement dans le RenderPass egui via `egui_wgpu::CallbackTrait`.
// Compatible Android Vulkan 1.1+, Linux Vulkan et Windows Vulkan/DX12 (origine flottante f32).

struct LidarPoint {
    rx: f32,
    ry: f32,
    rz: f32,
    rgba: u32,
};

struct LidarCameraUniform {
    cos_y: f32,
    sin_y: f32,
    cos_p: f32,
    sin_p: f32,
    z_exag: f32,
    view_scale: f32,
    focal: f32,
    depth_bias: f32,
    viewport_w: f32,
    viewport_h: f32,
    pan_x: f32,
    pan_y: f32,
    point_size_px: f32,
    origin_shift_x: f32,
    origin_shift_y: f32,
    origin_shift_z: f32,
};

@group(0) @binding(0) var<storage, read> points: array<LidarPoint>;
@group(0) @binding(1) var<uniform> cam: LidarCameraUniform;

struct VsOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) quad_uv: vec2<f32>,
};

@vertex
fn vs_main(
    @builtin(vertex_index) vid: u32,
    @builtin(instance_index) iid: u32,
) -> VsOut {
    var out: VsOut;
    let p = points[iid];

    let dx = p.rx + cam.origin_shift_x;
    let dy = p.ry + cam.origin_shift_y;
    let dz = (p.rz + cam.origin_shift_z) * cam.z_exag;

    // Rotation orbitale Yaw (Z) puis Pitch (X)
    let rx = dx * cam.cos_y - dy * cam.sin_y;
    let ry = dx * cam.sin_y + dy * cam.cos_y;
    let rz = ry * cam.sin_p + dz * cam.cos_p;
    let ry_proj = ry * cam.cos_p - dz * cam.sin_p;

    let denom = cam.focal + ry_proj + cam.depth_bias;
    if (denom <= 5.0) {
        out.clip_pos = vec4<f32>(2.0, 2.0, 2.0, 1.0);
        out.color = vec4<f32>(0.0);
        out.quad_uv = vec2<f32>(0.0);
        return out;
    }

    let pf = cam.focal / denom;
    let sx = cam.pan_x + rx * cam.view_scale * pf;
    let sy = -cam.pan_y + rz * cam.view_scale * pf;

    // 6 sommets pour 2 triangles formant un billboard centré sur le point
    var corner: vec2<f32>;
    switch (vid) {
        case 0u: { corner = vec2<f32>(-1.0, -1.0); }
        case 1u: { corner = vec2<f32>( 1.0, -1.0); }
        case 2u: { corner = vec2<f32>( 1.0,  1.0); }
        case 3u: { corner = vec2<f32>(-1.0, -1.0); }
        case 4u: { corner = vec2<f32>( 1.0,  1.0); }
        default: { corner = vec2<f32>(-1.0,  1.0); }
    }

    let half_w = max(cam.viewport_w * 0.5, 1.0);
    let half_h = max(cam.viewport_h * 0.5, 1.0);
    let ndc_x = (sx + corner.x * cam.point_size_px * 0.5) / half_w;
    let ndc_y = (sy + corner.y * cam.point_size_px * 0.5) / half_h;

    let r = f32(p.rgba & 0xFFu) / 255.0;
    let g = f32((p.rgba >> 8u) & 0xFFu) / 255.0;
    let b = f32((p.rgba >> 16u) & 0xFFu) / 255.0;
    let a = f32((p.rgba >> 24u) & 0xFFu) / 255.0;

    out.clip_pos = vec4<f32>(ndc_x, ndc_y, 0.5, 1.0);
    out.color = vec4<f32>(r, g, b, a);
    out.quad_uv = corner;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    // Splat circulaire lissé pour un rendu LiDAR net
    let d2 = dot(in.quad_uv, in.quad_uv);
    if (d2 > 1.0) {
        discard;
    }
    return in.color;
}
