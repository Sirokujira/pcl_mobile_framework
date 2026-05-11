// DepthCapturer.swift
//
// ARSession manager that extracts world-space points from each frame.
//
// On LiDAR devices (iPhone 12 Pro+, iPad Pro 2020+):
//   Uses ARSceneDepth — unprojection of the depth CVPixelBuffer via camera
//   intrinsics scaled to the depth-map resolution.  Every other pixel is
//   sampled to keep the per-frame budget below ~25 k points.
//
// On non-LiDAR devices:
//   Falls back to ARFrame.rawFeaturePoints (sparse, ~hundreds/frame).
//
// Accumulated points are kept in a [Float] packed buffer (x,y,z,…) that
// is mirrored into a shared MTLBuffer for zero-copy GPU access.

import ARKit
import Metal
import simd

@MainActor
final class DepthCapturer: NSObject, ObservableObject {

    @Published var statusMessage = "Starting…"
    @Published var pointCount    = 0
    @Published var frameCount    = 0
    @Published var hasLiDAR      = false

    // Shared Metal device — handed to PointCloudMetalRenderer.
    let device: MTLDevice = MTLCreateSystemDefaultDevice()!

    // Packed float buffer and its GPU mirror.
    private(set) var metalBuffer: MTLBuffer?
    private var floatData: [Float] = []

    private let maxFloats = 300_000 * 3   // ~300 k points
    private let session   = ARSession()

    override init() {
        super.init()
        startSession()
    }

    // MARK: - Public

    func clear() {
        floatData     = []
        metalBuffer   = nil
        pointCount    = 0
        frameCount    = 0
        statusMessage = "Cleared — move camera to collect"
    }

    /// Returns the current Metal buffer and point count, or nil if empty.
    func currentCloud() -> (MTLBuffer, Int)? {
        guard let buf = metalBuffer, pointCount > 0 else { return nil }
        return (buf, pointCount)
    }

    // MARK: - Setup

    private func startSession() {
        let config = ARWorldTrackingConfiguration()
        config.planeDetection = [.horizontal, .vertical]

        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics = .sceneDepth
            hasLiDAR      = true
            statusMessage = "LiDAR active — move camera slowly"
        } else {
            statusMessage = "No LiDAR — using feature points"
        }

        session.delegate = self
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    // MARK: - Accumulation

    private func append(_ newFloats: [Float]) {
        floatData.append(contentsOf: newFloats)
        if floatData.count > maxFloats {
            floatData = Array(floatData.suffix(maxFloats))
        }
        rebuildBuffer()
        pointCount = floatData.count / 3
        frameCount += 1
        updateStatus()
    }

    private func rebuildBuffer() {
        guard !floatData.isEmpty else { metalBuffer = nil; return }
        let byteLen = floatData.count * MemoryLayout<Float>.size
        if let existing = metalBuffer, existing.length >= byteLen {
            memcpy(existing.contents(), floatData, byteLen)
        } else {
            metalBuffer = device.makeBuffer(bytes:    floatData,
                                            length:   byteLen,
                                            options:  .storageModeShared)
        }
    }

    private func updateStatus() {
        let n   = pointCount
        let src = hasLiDAR ? "depth pts" : "feature pts"
        if n < 500 {
            statusMessage = "Collecting \(src)… (\(n))"
        } else if n < 20_000 {
            statusMessage = "\(n) \(src) — drag to orbit"
        } else {
            statusMessage = "\(n) pts — drag / pinch / 2-finger pan"
        }
    }
}

// MARK: - ARSessionDelegate

extension DepthCapturer: ARSessionDelegate {

    nonisolated func session(_ session: ARSession, didUpdate frame: ARFrame) {
        var pts = [Float]()

        if let sceneDepth = frame.sceneDepth {
            pts = Self.extractDepth(frame: frame, sceneDepth: sceneDepth)
        } else if let fp = frame.rawFeaturePoints {
            pts.reserveCapacity(fp.points.count * 3)
            for p in fp.points { pts.append(p.x); pts.append(p.y); pts.append(p.z) }
        }

        guard !pts.isEmpty else { return }
        Task { @MainActor [weak self] in self?.append(pts) }
    }

    nonisolated func session(_ session: ARSession, didFailWithError error: Error) {
        Task { @MainActor [weak self] in
            self?.statusMessage = "AR error: \(error.localizedDescription)"
        }
    }

    // Unproject depth map pixels to world-space packed floats.
    // Static + nonisolated so it runs entirely off the main actor.
    nonisolated private static func extractDepth(
        frame: ARFrame,
        sceneDepth: ARDepthData
    ) -> [Float] {
        let depthMap = sceneDepth.depthMap
        let dW = CVPixelBufferGetWidth(depthMap)
        let dH = CVPixelBufferGetHeight(depthMap)

        // Scale RGB intrinsics to depth-map resolution.
        let imgSize = frame.camera.imageResolution
        let sx = Float(dW) / Float(imgSize.width)
        let sy = Float(dH) / Float(imgSize.height)
        let K  = frame.camera.intrinsics        // column-major simd_float3x3
        // K[col][row]: K[0][0]=fx, K[1][1]=fy, K[2][0]=cx, K[2][1]=cy
        let fx = K[0][0] * sx,  fy = K[1][1] * sy
        let cx = K[2][0] * sx,  cy = K[2][1] * sy
        let camToWorld = frame.camera.transform // 4×4 camera-to-world

        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        let base = CVPixelBufferGetBaseAddress(depthMap)!
            .assumingMemoryBound(to: Float32.self)

        // Sample every 2nd pixel (row and col) — keeps ≤25 k pts/frame.
        var out = [Float]()
        out.reserveCapacity((dW / 2) * (dH / 2) * 3)

        for row in stride(from: 0, to: dH, by: 2) {
            for col in stride(from: 0, to: dW, by: 2) {
                let d = base[row * dW + col]
                guard d > 0.1 && d < 5.0 else { continue }

                // ARKit camera space: +X right, +Y up, -Z forward.
                // Depth is the distance along -Z, so z_c = -d.
                // Image row 0 is at the top → flip Y.
                let xc =  (Float(col) - cx) / fx * d
                let yc = -(Float(row) - cy) / fy * d
                let zc = -d

                // Camera space → world space via camera transform.
                let w = camToWorld * SIMD4<Float>(xc, yc, zc, 1)
                out.append(w.x); out.append(w.y); out.append(w.z)
            }
        }
        return out
    }
}
