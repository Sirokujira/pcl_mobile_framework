// Shaders.metal — MetalNormalSample
//
// normalColorVertex: maps world-space position to clip space and encodes
//   the surface normal as an RGB color:  color = abs(normal)
//     |nx| → R  (left/right facing surfaces)
//     |ny| → G  (up/down facing — floor, ceiling)
//     |nz| → B  (front/back facing — walls)
//
// Buffer layout:
//   buffer(0) NormalUniforms { float4x4 mvp; float pointSize; float3 _pad; }
//   buffer(1) packed_float3[] positions
//   buffer(2) packed_float3[] normals

#include <metal_stdlib>
using namespace metal;

struct NormalUniforms {
    float4x4 mvp;
    float    pointSize;
    float3   _pad;
};

struct NormalOut {
    float4 position  [[position]];
    float  pointSize [[point_size]];
    float4 color;
};

vertex NormalOut normalColorVertex(
    uint                    vid      [[vertex_id]],
    constant packed_float3* positions [[buffer(1)]],
    constant packed_float3* normals   [[buffer(2)]],
    constant NormalUniforms& u        [[buffer(0)]]
) {
    NormalOut out;
    out.position  = u.mvp * float4(float3(positions[vid]), 1.0);
    out.pointSize = u.pointSize;
    float3 n = abs(float3(normals[vid]));
    // Normalize so that saturated colors pop even for near-45° surfaces
    float maxComp = max(n.x, max(n.y, n.z));
    if (maxComp > 0.0) n /= maxComp;
    out.color = float4(n, 1.0);
    return out;
}

fragment float4 normalCircleFragment(
    NormalOut in    [[stage_in]],
    float2    coord [[point_coord]]
) {
    if (length(coord - 0.5) > 0.5) discard_fragment();
    return in.color;
}
