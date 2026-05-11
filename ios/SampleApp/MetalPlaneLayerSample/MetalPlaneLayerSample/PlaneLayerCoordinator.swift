// PlaneLayerCoordinator.swift — MetalPlaneLayerSample
//
// Depth accumulation + multi-pass PCLMobile plane segmentation.
//
// Iterative plane extraction algorithm:
//   For each pass (up to maxLayers passes):
//     a. Run segmentPlane on the current remaining cloud.
//     b. For every remaining point, compute signed distance to the plane:
//          dist = (dot(normal, p) + d) / |normal|
//        Points with |dist| < threshold → inlier of this plane layer.
//        Points with |dist| ≥ threshold → outlier, becomes the new remaining cloud.
//     c. Store the inliers with a layer colour.
//   Final remaining points → "unclassified" grey layer.
//
// This approach uses PCLMobile only for the plane model coefficients (a,b,c,d)
// and performs the inlier/outlier split on the Swift side, which allows us
// to work with the raw [Float] buffer directly without extra PCLMobile API calls.

import ARKit
import PCLMobile
import simd
import Foundation

// One colour layer of classified points.
struct PointLayer {
    var floatData: [Float]         // packed x,y,z ...
    var color:     SIMD3<Float>    // RGB
    var label:     String
}

struct PlaneLayerResult {
    var layers:      [PointLayer]
    var durationMs:  Double
}

@MainActor
final class PlaneLayerCoordinator: NSObject, ObservableObject {

    // MARK: - Published

    @Published var statusMessage  = "Move camera to collect depth data"
    @Published var rawCount       = 0
    @Published var filteredCount  = 0
    @Published var layerCount     = 0
    @Published var durationLabel  = "—"
    @Published var isProcessing   = false

    // Rendering snapshot (read by renderer under lock)
    private let lock = NSLock()
    private var _layerResult:   PlaneLayerResult?
    private var _rawBuffer:     MTLBuffer?
    private var _renderRawCount: Int = 0

    func currentResult()    -> PlaneLayerResult? { lock.withLock { _layerResult } }
    func currentRawCloud()  -> (MTLBuffer, Int)?  {
        lock.withLock { _rawBuffer.map { ($0, _renderRawCount) } }
    }

    // MARK: - Depth accumulation

    let device = MTLCreateSystemDefaultDevice()!
    private var rawFloats: [Float] = []
    private let maxFloats = 300_000 * 3

    private let session = ARSession()

    // Plane layer colours (up to 3 planes + remainder)
    static let layerColors: [(SIMD3<Float>, String)] = [
        (SIMD3(0.2, 0.5, 1.0), "Plane 1"),   // blue
        (SIMD3(0.2, 0.9, 0.4), "Plane 2"),   // green
        (SIMD3(0.7, 0.3, 1.0), "Plane 3"),   // purple
        (SIMD3(0.8, 0.8, 0.8), "Other"),     // grey
    ]

    static let maxLayers = 3

    // MARK: - Init

    override init() {
        super.init()
        startSession()
    }

    // MARK: - Session

    private func startSession() {
        let config = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics = .sceneDepth
            statusMessage = "LiDAR active — move camera to scan room"
        } else {
            statusMessage = "No LiDAR — collecting feature points"
        }
        session.delegate = self
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    // MARK: - Actions

    func clear() {
        rawFloats = []
        lock.withLock { _rawBuffer = nil; _renderRawCount = 0; _layerResult = nil }
        rawCount = 0; filteredCount = 0; layerCount = 0
        durationLabel = "—"
        statusMessage = "Cleared — move camera to scan"
    }

    func segment() {
        guard !isProcessing, rawFloats.count >= 300 else { return }
        isProcessing  = true
        statusMessage = "Running PCL segmentation…"
        let snapshot  = rawFloats
        let t0        = Date()

        Task.detached(priority: .userInitiated) { [weak self] in
            guard let self else { return }
            let result = try? Self.runPipeline(rawFloats: snapshot, startedAt: t0)

            await MainActor.run { [weak self] in
                guard let self else { return }
                if let result {
                    lock.withLock { _layerResult = result }
                    layerCount    = result.layers.filter { $0.label != "Other" }.count
                    filteredCount = result.layers.reduce(0) { $0 + $1.floatData.count / 3 }
                    durationLabel = String(format: "%.0f ms", result.durationMs)
                    statusMessage = "\(layerCount) planes segmented"
                } else {
                    statusMessage = "Segmentation failed — collect more data"
                }
                isProcessing = false
            }
        }
    }

    // MARK: - PCL pipeline (background)

    nonisolated private static func runPipeline(rawFloats: [Float],
                                                startedAt t0: Date) throws -> PlaneLayerResult {
        let rawCount = rawFloats.count / 3

        // 1. Build → downsample → clean
        let rawCloud = try rawFloats.withUnsafeBufferPointer { buf in
            try PointCloud.make(packedXYZ: buf.baseAddress!, count: UInt(rawCount))
        }
        let downsampled = try rawCloud.voxelGridDownsampled(leaf: 0.04)
        let filtered    = try downsampled.statisticalOutlierRemoval(meanK: 20, stddevMulThresh: 1.5)

        // 2. Unpack filtered points
        var remaining = try unpackFloats(filtered)   // [SIMD3<Float>]

        // 3. Iterative plane extraction
        var layers = [PointLayer]()
        let threshold: Float = 0.05

        for i in 0..<maxLayers {
            guard remaining.count >= 20 else { break }

            let packed: [Float] = remaining.flatMap { [$0.x, $0.y, $0.z] }
            let cloud = try packed.withUnsafeBufferPointer { buf in
                try PointCloud.make(packedXYZ: buf.baseAddress!, count: UInt(remaining.count))
            }
            guard let plane = try? cloud.segmentPlane(distanceThreshold: 0.04, maxIterations: 200)
            else { break }

            // Split inliers / outliers using plane model coefficients (a,b,c,d).
            let normal  = SIMD3<Float>(plane.a, plane.b, plane.c)
            let nLen    = simd_length(normal)
            var inliers  = [SIMD3<Float>]()
            var outliers = [SIMD3<Float>]()

            for p in remaining {
                let dist = abs(simd_dot(normal, p) + plane.d) / nLen
                if dist < threshold { inliers.append(p) }
                else                { outliers.append(p) }
            }

            guard inliers.count > 10 else { break }

            let (color, label) = layerColors[i]
            var floatBuf = [Float]()
            floatBuf.reserveCapacity(inliers.count * 3)
            inliers.forEach { floatBuf.append($0.x); floatBuf.append($0.y); floatBuf.append($0.z) }
            layers.append(PointLayer(floatData: floatBuf, color: color, label: label))

            remaining = outliers
        }

        // Remaining unclassified points
        if !remaining.isEmpty {
            let (color, label) = layerColors[maxLayers]
            var floatBuf = [Float]()
            floatBuf.reserveCapacity(remaining.count * 3)
            remaining.forEach { floatBuf.append($0.x); floatBuf.append($0.y); floatBuf.append($0.z) }
            layers.append(PointLayer(floatData: floatBuf, color: color, label: label))
        }

        return PlaneLayerResult(layers: layers, durationMs: Date().timeIntervalSince(t0) * 1000)
    }

    /// Unpack a PCLMobile PointCloud into [SIMD3<Float>].
    nonisolated private static func unpackFloats(_ cloud: PointCloud) throws -> [SIMD3<Float>] {
        let n = Int(cloud.pointCount)
        var buf = [Float](repeating: 0, count: n * 3)
        var actual: UInt = 0
        _ = try buf.withUnsafeMutableBufferPointer { ptr in
            try cloud.copyPackedXYZ(into: ptr.baseAddress!, capacity: UInt(n), actualCount: &actual)
        }
        return (0..<Int(actual)).map { i in SIMD3<Float>(buf[i*3], buf[i*3+1], buf[i*3+2]) }
    }

    // MARK: - Depth accumulation helpers

    private func appendDepthPoints(_ newFloats: [Float]) {
        rawFloats.append(contentsOf: newFloats)
        if rawFloats.count > maxFloats { rawFloats = Array(rawFloats.suffix(maxFloats)) }
        rawCount = rawFloats.count / 3
        rebuildRawBuffer()
        if rawCount < 500 {
            statusMessage = "Scanning… (\(rawCount) pts)"
        } else {
            statusMessage = "\(rawCount) pts — tap Segment Planes"
        }
    }

    private func rebuildRawBuffer() {
        let byteLen = rawFloats.count * MemoryLayout<Float>.size
        guard byteLen > 0 else { lock.withLock { _rawBuffer = nil; _renderRawCount = 0 }; return }
        let buf = device.makeBuffer(bytes: rawFloats, length: byteLen, options: .storageModeShared)
        lock.withLock { _rawBuffer = buf; _renderRawCount = rawFloats.count / 3 }
    }
}

// MARK: - ARSessionDelegate

extension PlaneLayerCoordinator: ARSessionDelegate {

    nonisolated func session(_ session: ARSession, didUpdate frame: ARFrame) {
        var newFloats = [Float]()

        if let depth = frame.sceneDepth {
            let dm = depth.depthMap
            let dW = CVPixelBufferGetWidth(dm); let dH = CVPixelBufferGetHeight(dm)
            let imgSize = frame.camera.imageResolution
            let sx = Float(dW)/Float(imgSize.width); let sy = Float(dH)/Float(imgSize.height)
            let K  = frame.camera.intrinsics
            let fx = K[0][0]*sx; let fy = K[1][1]*sy; let cx = K[2][0]*sx; let cy = K[2][1]*sy
            let camXform = frame.camera.transform

            CVPixelBufferLockBaseAddress(dm, .readOnly)
            let base = CVPixelBufferGetBaseAddress(dm)!.assumingMemoryBound(to: Float32.self)
            for row in Swift.stride(from: 0, to: dH, by: 3) {
                for col in Swift.stride(from: 0, to: dW, by: 3) {
                    let d = base[row*dW+col]
                    guard d > 0.15 && d < 6.0 else { continue }
                    let xc = (Float(col)-cx)/fx*d; let yc = -(Float(row)-cy)/fy*d; let zc = -d
                    let w  = camXform * SIMD4<Float>(xc, yc, zc, 1)
                    newFloats.append(w.x); newFloats.append(w.y); newFloats.append(w.z)
                }
            }
            CVPixelBufferUnlockBaseAddress(dm, .readOnly)
        } else if let fp = frame.rawFeaturePoints {
            for p in fp.points { newFloats.append(p.x); newFloats.append(p.y); newFloats.append(p.z) }
        }

        guard !newFloats.isEmpty else { return }
        Task { @MainActor [weak self] in self?.appendDepthPoints(newFloats) }
    }

    nonisolated func session(_ session: ARSession, didFailWithError error: Error) {
        Task { @MainActor [weak self] in
            self?.statusMessage = "Error: \(error.localizedDescription)"
        }
    }
}

private extension NSLock {
    @discardableResult
    func withLock<T>(_ f: () -> T) -> T { lock(); defer { unlock() }; return f() }
}
