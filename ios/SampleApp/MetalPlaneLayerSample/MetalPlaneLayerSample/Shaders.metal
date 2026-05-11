// Shaders.metal — MetalPlaneLayerSample
//
// Single pipeline pair for colored plane layers:
//
//   layerPointVertex — maps world-space point to clip space,
//                      applies per-draw-call uniform color.
//   layerCircleFragment — circular sprite discard.
//
// Buffer layout:
//   buffer(0) LayerUniforms { float4x4 mvp; float pointSize; float3 color; float pad; }
//   buffer(1) packed_float3[] point positions

#include <metal_stdlib>
using namespace metal;

struct LayerUniforms {
    float4x4 mvp;
    float    pointSize;
    float3   color;
    float    _pad;
};

struct PointOut {
    float4 position  [[position]];
    float  pointSize [[point_size]];
    float4 color;
};

vertex PointOut layerPointVertex(
    uint                    vid  [[vertex_id]],
    constant packed_float3* pts  [[buffer(1)]],
    constant LayerUniforms& u    [[buffer(0)]]
) {
    PointOut out;
    out.position  = u.mvp * float4(float3(pts[vid]), 1.0);
    out.pointSize = u.pointSize;
    out.color     = float4(u.color, 1.0);
    return out;
}

fragment float4 layerCircleFragment(
    PointOut in    [[stage_in]],
    float2   coord [[point_coord]]
) {
    if (length(coord - 0.5) > 0.5) discard_fragment();
    return in.color;
}
