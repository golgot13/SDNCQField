// Shader GPU natif Vulkan/WGPU — Rendu cartographique 2D haute performance
// Conçu pour plans cadastraux denses (DXF, DXF-PCI, EDIGÉO, IGN WFS, GeoPackage)
// en origine flottante f32 (compatible Android Vulkan 1.1+, Linux Vulkan, Windows Vulkan/DX12).

struct Camera2dUniform {
    // Décalage entre l'origine du batch VRAM et le centre de vue (mètres)
    shift_x: f32,
    shift_y: f32,
    // Échelle pixels / mètre
    scale: f32,
    pad0: f32,
    // Dimensions du viewport en pixels logiques
    viewport_w: f32,
    viewport_h: f32,
    pan_x: f32,
    pan_y: f32,
};

@group(0) @binding(0) var<uniform> cam: Camera2dUniform;

// ─── Pipeline 1 : Polygones remplis 2D (parcelles, bâtiments durs/légers, hydro) ───

struct FillVertexIn {
    @location(0) pos: vec2<f32>,
    @location(1) rgba: u32,
};

struct FillVsOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) color: vec4<f32>,
};

fn unpack_rgba(c: u32) -> vec4<f32> {
    return vec4<f32>(
        f32(c & 0xFFu) / 255.0,
        f32((c >> 8u) & 0xFFu) / 255.0,
        f32((c >> 16u) & 0xFFu) / 255.0,
        f32((c >> 24u) & 0xFFu) / 255.0
    );
}

fn world_rel_to_px(p: vec2<f32>) -> vec2<f32> {
    let sx = cam.pan_x + (p.x + cam.shift_x) * cam.scale;
    let sy = -cam.pan_y + (p.y + cam.shift_y) * cam.scale;
    return vec2<f32>(sx, sy);
}

fn px_to_ndc(px: vec2<f32>) -> vec2<f32> {
    let hw = max(cam.viewport_w * 0.5, 1.0);
    let hh = max(cam.viewport_h * 0.5, 1.0);
    return vec2<f32>(px.x / hw, px.y / hh);
}

@vertex
fn vs_fill(in: FillVertexIn) -> FillVsOut {
    var out: FillVsOut;
    let ndc = px_to_ndc(world_rel_to_px(in.pos));
    out.clip_pos = vec4<f32>(ndc.x, ndc.y, 0.5, 1.0);
    out.color = unpack_rgba(in.rgba);
    return out;
}

@fragment
fn fs_fill(in: FillVsOut) -> @location(0) vec4<f32> {
    return in.color;
}

// ─── Pipeline 2 : Segments & polylignes 2D instanciés (contours parcelles/bâtis/voies) ───

struct Segment2d {
    x0: f32,
    y0: f32,
    x1: f32,
    y1: f32,
    width_px: f32,
    rgba: u32,
    pad0: u32,
    pad1: u32,
};

@group(1) @binding(0) var<storage, read> segments: array<Segment2d>;

struct SegVsOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) edge_dist: f32,
};

@vertex
fn vs_seg(
    @builtin(vertex_index) vid: u32,
    @builtin(instance_index) iid: u32,
) -> SegVsOut {
    var out: SegVsOut;
    let seg = segments[iid];
    let p0 = world_rel_to_px(vec2<f32>(seg.x0, seg.y0));
    let p1 = world_rel_to_px(vec2<f32>(seg.x1, seg.y1));

    // Culling frustum rapide dans le vertex shader
    let hw = cam.viewport_w * 0.5 + seg.width_px + 4.0;
    let hh = cam.viewport_h * 0.5 + seg.width_px + 4.0;
    if ((p0.x < -hw && p1.x < -hw) || (p0.x > hw && p1.x > hw)
        || (p0.y < -hh && p1.y < -hh) || (p0.y > hh && p1.y > hh)) {
        out.clip_pos = vec4<f32>(2.0, 2.0, 2.0, 1.0);
        out.color = vec4<f32>(0.0);
        out.edge_dist = 0.0;
        return out;
    }

    let dir = p1 - p0;
    let len = max(length(dir), 0.001);
    let tangent = dir / len;
    let normal = vec2<f32>(-tangent.y, tangent.x);
    let half_w = max(seg.width_px * 0.5, 0.6);

    var along: f32 = 0.0;
    var side: f32 = 0.0;
    switch (vid) {
        case 0u: { along = 0.0; side = -1.0; }
        case 1u: { along = 1.0; side = -1.0; }
        case 2u: { along = 1.0; side =  1.0; }
        case 3u: { along = 0.0; side = -1.0; }
        case 4u: { along = 1.0; side =  1.0; }
        default: { along = 0.0; side =  1.0; }
    }

    let pos_px = mix(p0, p1, along) + normal * (side * half_w);
    let ndc = px_to_ndc(pos_px);
    out.clip_pos = vec4<f32>(ndc.x, ndc.y, 0.5, 1.0);
    out.color = unpack_rgba(seg.rgba);
    out.edge_dist = side;
    return out;
}

@fragment
fn fs_seg(in: SegVsOut) -> @location(0) vec4<f32> {
    // Anti-aliasing doux sur les bords des lignes cadastrales
    let aa = 1.0 - smoothstep(0.75, 1.0, abs(in.edge_dist));
    return vec4<f32>(in.color.rgb, in.color.a * aa);
}
