// Shader Vulkan Compute (WGSL) — Projection 3D orbitale & Frustum Culling parallèle
// Compatible GPU mobiles Android (f32 relatif / origine flottante).

struct PointIn {
    rx: f32,
    ry: f32,
    rz: f32,
    rgba: u32,
};

struct CameraUniforms {
    cos_y: f32,
    sin_y: f32,
    cos_p: f32,
    sin_p: f32,
    z_exag: f32,
    view_scale: f32,
    focal: f32,
    depth_bias: f32,
    center_sx: f32,
    center_sy: f32,
    min_sx: f32,
    min_sy: f32,
    max_sx: f32,
    max_sy: f32,
    num_points: u32,
    pad0: u32,
};

struct ProjectedPoint {
    sx: f32,
    sy: f32,
    depth: f32,
    visible_rgba: u32, // 0 si hors frustum
};

@group(0) @binding(0) var<storage, read> points_in: array<PointIn>;
@group(0) @binding(1) var<uniform> cam: CameraUniforms;
@group(0) @binding(2) var<storage, read_write> points_out: array<ProjectedPoint>;

@compute @workgroup_size(64, 1, 1)
fn project_points(@builtin(global_invocation_id) gid: vec3<u32>) {
    let idx = gid.x;
    if (idx >= cam.num_points) {
        return;
    }

    let p = points_in[idx];
    let dz = p.rz * cam.z_exag;

    let rx = p.rx * cam.cos_y - p.ry * cam.sin_y;
    let ry = p.rx * cam.sin_y + p.ry * cam.cos_y;

    let rz = ry * cam.sin_p + dz * cam.cos_p;
    let ry_proj = ry * cam.cos_p - dz * cam.sin_p;

    let denom = cam.focal + ry_proj + cam.depth_bias;
    if (denom <= 1.0) {
        points_out[idx] = ProjectedPoint(0.0, 0.0, -1.0, 0u);
        return;
    }

    let p_factor = cam.focal / denom;
    let sx = cam.center_sx + rx * cam.view_scale * p_factor;
    let sy = cam.center_sy - rz * cam.view_scale * p_factor;

    let in_view = sx >= cam.min_sx && sx <= cam.max_sx && sy >= cam.min_sy && sy <= cam.max_sy;
    let vis = select(0u, p.rgba | 0xFF000000u, in_view);
    points_out[idx] = ProjectedPoint(sx, sy, denom, vis);
}
