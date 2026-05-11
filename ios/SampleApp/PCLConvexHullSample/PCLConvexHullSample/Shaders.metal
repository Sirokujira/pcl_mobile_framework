// Shaders.metal — PCLConvexHullSample
//
// Three shader pairs:
//   rawPointVertex / rawCircleFragment   — pre-computation raw cloud (dim white)
//   hullLineVertex / hullLineFragment    — convex hull boundary (line strip per plane)
//   hullFillVertex / hullFillFragment    — filled triangle fan from centroid (translucent)
//
// Buffer layout for hull shaders:
//   buffer(0) HullUniforms { float4x4 mvp; float4 color; float pointSize; float3 _pad; }
//   buffer(1) packed_float3[] vertices

#include <metal_stdlib>
using namespace metal;

// ─── Raw point cloud ───────────────────────────────────────────────────────

struct RawUniforms {
    float4x4 mvp;
    float    pointSize;
    float3   _pad;
};

struct RawOut {
    float4 position  [[position]];
    float  pointSize [[point_size]];
    float4 color;
};

vertex RawOut rawPointVertex(
    uint                   vid [[vertex_id]],
    constant packed_float3* pts [[buffer(1)]],
    constant RawUniforms&   u   [[buffer(0)]]
) {
    RawOut out;
    out.position  = u.mvp * float4(float3(pts[vid]), 1.0);
    out.pointSize = u.pointSize;
    out.color     = float4(0.55, 0.55, 0.55, 1.0);
    return out;
}

fragment float4 rawCircleFragment(
    RawOut in    [[stage_in]],
    float2 coord [[point_coord]]
) {
    if (length(coord - 0.5) > 0.5) discard_fragment();
    return in.color;
}

// ─── Hull line / fill ──────────────────────────────────────────────────────

struct HullUniforms {
    float4x4 mvp;
    float4   color;
    float    pointSize;
    float3   _pad;
};

struct HullOut {
    float4 position  [[position]];
    float4 color;
};

vertex HullOut hullLineVertex(
    uint                    vid [[vertex_id]],
    constant packed_float3* pts [[buffer(1)]],
    constant HullUniforms&  u   [[buffer(0)]]
) {
    HullOut out;
    out.position = u.mvp * float4(float3(pts[vid]), 1.0);
    out.color    = u.color;
    return out;
}

fragment float4 hullLineFragment(HullOut in [[stage_in]]) {
    return in.color;
}

// Triangle fan fill (slightly transparent version of hull color)
vertex HullOut hullFillVertex(
    uint                    vid [[vertex_id]],
    constant packed_float3* pts [[buffer(1)]],
    constant HullUniforms&  u   [[buffer(0)]]
) {
    HullOut out;
    out.position = u.mvp * float4(float3(pts[vid]), 1.0);
    out.color    = float4(u.color.rgb, 0.18);
    return out;
}

fragment float4 hullFillFragment(HullOut in [[stage_in]]) {
    return in.color;
}
