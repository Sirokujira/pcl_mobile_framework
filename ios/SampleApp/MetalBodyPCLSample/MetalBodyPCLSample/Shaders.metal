// Shaders.metal — MetalBodyPCLSample
//
// Two pipeline pairs:
//
//   1. bodyPointVertex / circleFragment
//      Renders joint positions as large point sprites.
//      Color encodes joint height (y) via jet colormap:
//        foot (low) → blue,  head (high) → red.
//      Buffer layout:
//        buffer(0)  PointUniforms { float4x4 mvp; float pointSize; float yMin; float yMax; float _pad; }
//        buffer(1)  packed_float3[]  world-space joint positions
//
//   2. wireVertex / wireFragment
//      Renders line-list geometry for bounding box edges and normal arrows.
//      Uniform color passed per draw call via WireUniforms.color.
//      Buffer layout:
//        buffer(0)  WireUniforms { float4x4 mvp; float4 color; }
//        buffer(1)  packed_float3[]  vertex positions (two per line segment)

#include <metal_stdlib>
using namespace metal;

// ─── Shared types ─────────────────────────────────────────────────────────────

struct PointUniforms {
    float4x4 mvp;
    float    pointSize;
    float    yMin;
    float    yMax;
    float    _pad;
};

struct WireUniforms {
    float4x4 mvp;
    float4   color;
};

// ─── Jet colormap ─────────────────────────────────────────────────────────────
// t ∈ [0,1]: 0 → blue, 0.5 → green, 1 → red

float4 jet(float t) {
    t = saturate(t);
    return float4(
        saturate(1.5 - abs(4.0*t - 3.0)),
        saturate(1.5 - abs(4.0*t - 2.0)),
        saturate(1.5 - abs(4.0*t - 1.0)),
        1.0
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// MARK: 1. Body joint point cloud
// ═══════════════════════════════════════════════════════════════════════════════

struct PointOut {
    float4 position  [[position]];
    float  pointSize [[point_size]];
    float4 color;
};

vertex PointOut bodyPointVertex(
    uint                     vid  [[vertex_id]],
    constant packed_float3*  pts  [[buffer(1)]],
    constant PointUniforms&  u    [[buffer(0)]]
) {
    float3 p = float3(pts[vid]);
    float  t = (u.yMax > u.yMin)
             ? (p.y - u.yMin) / (u.yMax - u.yMin)
             : 0.5;
    PointOut out;
    out.position  = u.mvp * float4(p, 1.0);
    out.pointSize = u.pointSize;
    out.color     = jet(t);
    return out;
}

// Render circular point sprites by discarding corners.
fragment float4 circleFragment(
    PointOut in    [[stage_in]],
    float2   coord [[point_coord]]
) {
    if (length(coord - 0.5) > 0.5) discard_fragment();
    return in.color;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MARK: 2. Wireframe (bounding box edges + normal arrows)
// ═══════════════════════════════════════════════════════════════════════════════

struct WireOut {
    float4 position [[position]];
    float4 color;
};

vertex WireOut wireVertex(
    uint                   vid  [[vertex_id]],
    constant packed_float3* pts  [[buffer(1)]],
    constant WireUniforms&  u    [[buffer(0)]]
) {
    WireOut out;
    out.position = u.mvp * float4(float3(pts[vid]), 1.0);
    out.color    = u.color;
    return out;
}

fragment float4 wireFragment(WireOut in [[stage_in]]) {
    return in.color;
}
