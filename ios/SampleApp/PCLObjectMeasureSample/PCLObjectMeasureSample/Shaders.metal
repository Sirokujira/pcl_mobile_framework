// Shaders.metal — PCLObjectMeasureSample
//
// Two shader pairs:
//   objectPointVertex / objectCircleFragment — colored point sprites (floor=grey, clusters=vivid)
//   wireVertex / wireFragment               — solid-color line primitives for AABB edges
//
// Buffer layout (both pairs):
//   buffer(0) ObjectUniforms { float4x4 mvp; float pointSize; float3 color; float pad; }
//   buffer(1) packed_float3[] positions

#include <metal_stdlib>
using namespace metal;

struct ObjectUniforms {
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

struct WireOut {
    float4 position [[position]];
    float4 color;
};

vertex PointOut objectPointVertex(
    uint                    vid [[vertex_id]],
    constant packed_float3* pts [[buffer(1)]],
    constant ObjectUniforms& u  [[buffer(0)]]
) {
    PointOut out;
    out.position  = u.mvp * float4(float3(pts[vid]), 1.0);
    out.pointSize = u.pointSize;
    out.color     = float4(u.color, 1.0);
    return out;
}

fragment float4 objectCircleFragment(
    PointOut in    [[stage_in]],
    float2   coord [[point_coord]]
) {
    if (length(coord - 0.5) > 0.5) discard_fragment();
    return in.color;
}

vertex WireOut wireVertex(
    uint                    vid [[vertex_id]],
    constant packed_float3* pts [[buffer(1)]],
    constant ObjectUniforms& u  [[buffer(0)]]
) {
    WireOut out;
    out.position = u.mvp * float4(float3(pts[vid]), 1.0);
    out.color    = float4(u.color, 1.0);
    return out;
}

fragment float4 wireFragment(WireOut in [[stage_in]]) {
    return in.color;
}
