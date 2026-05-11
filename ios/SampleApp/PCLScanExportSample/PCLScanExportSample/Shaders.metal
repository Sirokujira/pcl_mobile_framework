// Shaders.metal — PCLScanExportSample
//
// Points are colored by their distance from the camera origin (jet colormap).
// buffer(0) ScanUniforms { float4x4 mvp; float pointSize; float maxDist; float2 _pad; }
// buffer(1) packed_float3[] positions

#include <metal_stdlib>
using namespace metal;

struct ScanUniforms {
    float4x4 mvp;
    float    pointSize;
    float    maxDist;
    float2   _pad;
};

struct ScanOut {
    float4 position  [[position]];
    float  pointSize [[point_size]];
    float4 color;
};

static float4 jet(float t) {
    t = saturate(t);
    float r = saturate(1.5 - abs(4.0*t - 3.0));
    float g = saturate(1.5 - abs(4.0*t - 2.0));
    float b = saturate(1.5 - abs(4.0*t - 1.0));
    return float4(r, g, b, 1.0);
}

vertex ScanOut scanPointVertex(
    uint                   vid  [[vertex_id]],
    constant packed_float3* pts [[buffer(1)]],
    constant ScanUniforms&  u   [[buffer(0)]]
) {
    ScanOut out;
    float3 p     = float3(pts[vid]);
    out.position  = u.mvp * float4(p, 1.0);
    out.pointSize = u.pointSize;
    out.color     = jet(length(p) / u.maxDist);
    return out;
}

fragment float4 scanCircleFragment(
    ScanOut in    [[stage_in]],
    float2  coord [[point_coord]]
) {
    if (length(coord - 0.5) > 0.5) discard_fragment();
    return in.color;
}
