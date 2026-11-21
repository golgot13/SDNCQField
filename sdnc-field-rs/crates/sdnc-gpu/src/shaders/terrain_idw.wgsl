// Shader Vulkan Compute (WGSL) — Interpolation MNT IDW + Ombrage (Hillshade) & Pente
// Conçu pour Android (Vulkan 1.1+ mobile sans f64), Linux et Windows en origine flottante f32.

struct SamplePoint {
    rx: f32, // X relatif à l'origine de grille (m)
    ry: f32, // Y relatif à l'origine de grille (m)
    rz: f32, // Altitude Z (m)
    weight: f32,
};

struct TerrainUniforms {
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
};

struct CellOutput {
    elevation: f32,
    slope_deg: f32,
    hillshade: f32,
    pad: f32,
};

@group(0) @binding(0) var<storage, read> samples: array<SamplePoint>;
@group(0) @binding(1) var<uniform> cfg: TerrainUniforms;
@group(0) @binding(2) var<storage, read_write> heights_out: array<f32>;
@group(0) @binding(3) var<storage, read_write> cells_out: array<CellOutput>;

@compute @workgroup_size(8, 8, 1)
fn interpolate_idw(@builtin(global_invocation_id) gid: vec3<u32>) {
    let gx = gid.x;
    let gy = gid.y;
    if (gx >= cfg.cols || gy >= cfg.rows) {
        return;
    }

    let wx = f32(gx) * cfg.dx;
    let wy = f32(gy) * cfg.dy;

    var num: f32 = 0.0;
    var den: f32 = 0.0;
    var exact_z: f32 = cfg.default_z;
    var has_exact: bool = false;

    for (var i: u32 = 0u; i < cfg.num_samples; i = i + 1u) {
        let s = samples[i];
        let ddx = s.rx - wx;
        let ddy = s.ry - wy;
        let d2 = ddx * ddx + ddy * ddy;
        if (d2 < 1.0) {
            exact_z = s.rz;
            has_exact = true;
            break;
        }
        let w = s.weight / (d2 + cfg.power_bias);
        num = num + w * s.rz;
        den = den + w;
    }

    let z = select(select(cfg.default_z, num / den, den > 1e-6), exact_z, has_exact);
    heights_out[gy * cfg.cols + gx] = z;
}

@compute @workgroup_size(8, 8, 1)
fn compute_hillshade(@builtin(global_invocation_id) gid: vec3<u32>) {
    let gx = gid.x;
    let gy = gid.y;
    if (gx >= cfg.cols || gy >= cfg.rows) {
        return;
    }

    let idx = gy * cfg.cols + gx;
    let zc = heights_out[idx];

    let x_prev = select(0u, gx - 1u, gx > 0u);
    let x_next = min(gx + 1u, cfg.cols - 1u);
    let y_prev = select(0u, gy - 1u, gy > 0u);
    let y_next = min(gy + 1u, cfg.rows - 1u);

    let dz_dx = (heights_out[gy * cfg.cols + x_next] - heights_out[gy * cfg.cols + x_prev])
        / max(f32(x_next - x_prev) * cfg.dx, 1e-3);
    let dz_dy = (heights_out[y_next * cfg.cols + gx] - heights_out[y_prev * cfg.cols + gx])
        / max(f32(y_next - y_prev) * cfg.dy, 1e-3);

    let nx = -dz_dx * cfg.z_factor;
    let ny = -dz_dy * cfg.z_factor;
    let nz = 1.0;
    let n_len = sqrt(nx * nx + ny * ny + nz * nz);
    let normal = vec3<f32>(nx / n_len, ny / n_len, nz / n_len);

    let slope_rad = atan(sqrt(dz_dx * dz_dx + dz_dy * dz_dy));
    let slope_deg = slope_rad * 57.2957795;

    let cos_alt = cos(cfg.sun_altitude_rad);
    let sun_dir = vec3<f32>(
        sin(cfg.sun_azimuth_rad) * cos_alt,
        cos(cfg.sun_azimuth_rad) * cos_alt,
        sin(cfg.sun_altitude_rad)
    );

    let shade = clamp(dot(normal, sun_dir), 0.08, 1.0);
    cells_out[idx] = CellOutput(zc, slope_deg, shade, 0.0);
}
