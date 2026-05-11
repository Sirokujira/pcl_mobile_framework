// Shaders.metal — MetalAROverlay
//
// Three shader pairs:
//
//   1. cameraBackgroundVertex / cameraBackgroundFragment
//      Full-screen quad rendering the ARKit YCbCr camera feed as RGB.
//      The displayTransform (passed as a float3x3) corrects for device
//      orientation and viewport aspect ratio.
//
//   2. depthOverlayVertex / overlayPointFragment
//      Per-depth-pixel vertex shader: samples the depth texture, unprojects
//      the pixel to camera space using scaled intrinsics, then transforms
//      to clip space with the ARCamera projection matrix.
//      Requires a LiDAR device (ARFrame.sceneDepth != nil).
//
//   3. featurePointVertex / overlayPointFragment
//      Non-LiDAR fallback: renders world-space packed_float3 feature points
//      using a combined view-projection matrix.
//
// Coordinate conventions:
//   ARKit camera space:  +X right, +Y up, −Z forward (right-handed).
//   depth values:        positive metres along −Z (i.e. z_c = −depth).
//   Image row 0:         top of frame → flip Y when converting to camera Y.

#include <metal_stdlib>
using namespace metal;

// ─── Shared helper: circular sprite discard ──────────────────────────────────

// Called from every point fragment shader.
inline float4 circleOrDiscard(float4 color, float2 pointCoord) {
    if (length(pointCoord - 0.5) > 0.5) discard_fragment();
    return color;
}

// ─── Depth-to-color helpers ──────────────────────────────────────────────────

// Jet colormap: t ∈ [0,1] → blue … cyan … green … yellow … red.
float4 jet(float t) {
    t = saturate(t);
    return float4(
        saturate(1.5 - abs(4.0*t - 3.0)),
        saturate(1.5 - abs(4.0*t - 2.0)),
        saturate(1.5 - abs(4.0*t - 1.0)),
        0.85
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// MARK: 1. Camera background
// ═══════════════════════════════════════════════════════════════════════════════

struct CameraUniforms {
    float3x3 displayTransform;
};

struct CameraVert {
    float4 position [[position]];
    float2 uv;
};

// Fullscreen triangle-strip quad.
// Positions: NDC corners  (−1,−1), (−1,1), (1,−1), (1,1).
// UVs before transform: image origin = top-left → (0,0) maps to bottom-left NDC.
constant float2 kPos[4] = { {-1,-1}, {-1,1}, {1,-1}, {1,1} };
constant float2 kUV[4]  = { { 0, 1}, { 0, 0}, {1, 1}, {1, 0} };

vertex CameraVert cameraBackgroundVertex(
    uint                    vid  [[vertex_id]],
    constant CameraUniforms& u   [[buffer(0)]]
) {
    CameraVert out;
    out.position = float4(kPos[vid], 0.0, 1.0);
    // Apply the CGAffineTransform (stored column-major as float3x3) to the UV
    // coordinates so the camera image maps correctly to the portrait viewport.
    float3 t = u.displayTransform * float3(kUV[vid], 1.0);
    out.uv = t.xy;
    return out;
}

// BT.709 full-range YCbCr → RGB conversion.
// capturedImage pixel format: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange.
// Plane 0 (Y):    r8Unorm   — luma  [0,1]
// Plane 1 (CbCr): rg8Unorm  — chroma centred at 0.5
fragment float4 cameraBackgroundFragment(
    CameraVert               in      [[stage_in]],
    texture2d<float>         yTex    [[texture(0)]],
    texture2d<float>         cbcrTex [[texture(1)]]
) {
    constexpr sampler s(filter::linear, address::clamp_to_edge);
    float  y    = yTex.sample(s, in.uv).r;
    float2 cbcr = cbcrTex.sample(s, in.uv).rg - 0.5;
    float3 rgb  = float3(
        y + 1.4028f * cbcr.y,
        y - 0.3456f * cbcr.x - 0.7145f * cbcr.y,
        y + 1.7710f * cbcr.x
    );
    return float4(saturate(rgb), 1.0);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MARK: 2. Depth overlay (LiDAR — vertex-shader unprojection)
// ═══════════════════════════════════════════════════════════════════════════════

struct OverlayUniforms {
    float4x4 projectionMatrix;   // ARCamera projection (camera-space → clip)
    float    fx, fy;             // depth-sensor focal lengths (scaled to depth res)
    float    cx, cy;             // depth-sensor principal point
    float    depthWidth;
    float    depthHeight;
    float    maxDepth;
    float    pointSize;
};

struct DepthPointOut {
    float4 position  [[position]];
    float  pointSize [[point_size]];
    float4 color;
};

// texture(1) is the optional confidence map (r8Uint, ARConfidenceLevel values).
// We discard low-confidence points (value < 2 = ARConfidenceLevelHigh) when the
// texture is bound.  If not bound, no discard.
vertex DepthPointOut depthOverlayVertex(
    uint                           vid       [[vertex_id]],
    constant OverlayUniforms&      u         [[buffer(0)]],
    texture2d<float, access::read> depthTex  [[texture(0)]],
    texture2d<uint,  access::read> confTex   [[texture(1)]]
) {
    uint w   = uint(u.depthWidth);
    uint h   = uint(u.depthHeight);
    uint col = vid % w;
    uint row = vid / w;

    float depth = depthTex.read(uint2(col, row)).r;

    DepthPointOut out;

    // Discard invalid or out-of-range depths (push off screen).
    if (depth < 0.05 || depth > u.maxDepth) {
        out.position  = float4(10, 10, 0, 1);
        out.pointSize = 0;
        out.color     = 0;
        return out;
    }

    // Skip low-confidence pixels when the confidence texture is available.
    // ARConfidenceLevelHigh == 2.  texture_type uint requires access::read.
    if (col < confTex.get_width() && row < confTex.get_height()) {
        uint conf = confTex.read(uint2(col, row)).r;
        if (conf < 2) {
            out.position  = float4(10, 10, 0, 1);
            out.pointSize = 0;
            out.color     = 0;
            return out;
        }
    }

    // Unproject depth pixel → camera space (ARKit: −Z forward, +Y up).
    //   x_c =  (col − cx) / fx · d
    //   y_c = −(row − cy) / fy · d   ← image Y increases downward; camera Y is up
    //   z_c = −d                      ← camera −Z is forward; depth is positive
    float xc =  (float(col) - u.cx) / u.fx * depth;
    float yc = -(float(row) - u.cy) / u.fy * depth;
    float zc = -depth;

    // Project camera-space point to clip space via ARCamera projection matrix.
    out.position  = u.projectionMatrix * float4(xc, yc, zc, 1.0);
    out.pointSize = u.pointSize;
    out.color     = jet(depth / u.maxDepth);
    return out;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MARK: 3. Feature-point overlay (non-LiDAR fallback)
// ═══════════════════════════════════════════════════════════════════════════════

struct FeatureUniforms {
    float4x4 viewProjection;
    float    pointSize;
    float    pad[3];
};

struct FeaturePointOut {
    float4 position  [[position]];
    float  pointSize [[point_size]];
    float4 color;
};

vertex FeaturePointOut featurePointVertex(
    uint                    vid    [[vertex_id]],
    constant packed_float3* points [[buffer(1)]],
    constant FeatureUniforms& u    [[buffer(0)]]
) {
    float3 p = float3(points[vid]);
    FeaturePointOut out;
    out.position  = u.viewProjection * float4(p, 1.0);
    out.pointSize = u.pointSize;
    // Bright cyan for feature points
    out.color = float4(0.0, 0.9, 1.0, 0.9);
    return out;
}

// ─── Shared fragment shader for both overlay passes ───────────────────────────

fragment float4 overlayPointFragment(
    DepthPointOut in    [[stage_in]],
    float2        coord [[point_coord]]
) {
    return circleOrDiscard(in.color, coord);
}
