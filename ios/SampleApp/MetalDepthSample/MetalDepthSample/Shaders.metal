// Shaders.metal — MetalDepthSample
//
// Single render pass: world-space point cloud → screen.
//
// Buffer layout (matches PointCloudMetalRenderer.swift):
//   buffer(0)  ViewerUniforms  { float4x4 mvp; float pointSize; float[3] pad; }
//   buffer(1)  packed_float3[] world-space point positions

#include <metal_stdlib>
using namespace metal;

// ─── Uniform struct ──────────────────────────────────────────────────────────

struct ViewerUniforms {
    float4x4 mvp;
    float    pointSize;
    float    pad[3];
};

// ─── Vertex / fragment I/O ───────────────────────────────────────────────────

struct PointOut {
    float4 position  [[position]];
    float  pointSize [[point_size]];
    float4 color;
};

// ─── Jet colormap ────────────────────────────────────────────────────────────
// Maps t ∈ [0,1] → blue … cyan … green … yellow … red

float4 jet(float t) {
    t = saturate(t);
    return float4(
        saturate(1.5 - abs(4.0*t - 3.0)),   // R
        saturate(1.5 - abs(4.0*t - 2.0)),   // G
        saturate(1.5 - abs(4.0*t - 1.0)),   // B
        1.0
    );
}

// ─── Vertex shader ───────────────────────────────────────────────────────────
// Each vertex ID maps to one world-space point.
// The jet color encodes distance from the world origin (blue=near, red=far).

vertex PointOut pointCloudVertex(
    uint                     vid    [[vertex_id]],
    constant packed_float3*  points [[buffer(1)]],
    constant ViewerUniforms& u      [[buffer(0)]]
) {
    float3 p    = float3(points[vid]);
    float  dist = length(p);                // 0 → ~4 m typical indoor range

    PointOut out;
    out.position  = u.mvp * float4(p, 1.0);
    out.pointSize = u.pointSize;
    out.color     = jet(dist * 0.28);       // 0 m → blue,  ~3.5 m → red
    return out;
}

// ─── Fragment shader ─────────────────────────────────────────────────────────
// Discard corners to render circular point sprites.

fragment float4 circleFragment(
    PointOut in    [[stage_in]],
    float2   coord [[point_coord]]
) {
    if (length(coord - 0.5) > 0.5) discard_fragment();
    return in.color;
}
