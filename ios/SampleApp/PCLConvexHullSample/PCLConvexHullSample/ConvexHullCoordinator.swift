// ConvexHullCoordinator.swift — PCLConvexHullSample
//
// For each detected plane (up to 4):
//   a. segmentPlane(RANSAC) → get plane model coefficients
//   b. Swift-side inlier split → build inlier PointCloud
//   c. projectInliersToPlane() → all inliers snapped onto the fitted plane
//   d. convexHull() → boundary polygon vertices
//   e. Store hull vertices + centroid + area estimate
//
// The convex hull outlines show the true 2D shape of each plane surface,
// which is more accurate than an AABB for irregular surfaces.

import ARKit
import PCLMobile
import simd
import Foundation
import Metal

struct PlaneHull {
    var hullVertices: [SIMD3<Float>]   // in hull order (line loop)
    var centroid:     SIMD3<Float>
    var area:         Float             // approximate (shoelace on projected 2D)
    var color:        SIMD3<Float>
    var label:        String
}

struct HullResult {
    var hulls:      [PlaneHull]
    var rawData:    [Float]            // pre-segmentation preview
    var durationMs: Double
}

@MainActor
final class ConvexHullCoordinator: NSObject, ObservableObject {

    @Published var statusMessage = "Move camera to collect depth data"
    @Published var rawCount      = 0
    @Published var hullCount     = 0
    @Published var durationLabel = "—"
    @Published var isProcessing  = false
    @Published var hullInfos:  [HullInfo] = []

    struct HullInfo: Identifiable {
        let id    = UUID()
        var label: String
        var pts:   Int
        var area:  Float
        var color: SIMD3<Float>
    }

    let device = MTLCreateSystemDefaultDevice()!

    private let lock = NSLock()
    private var _result: HullResult?

    func currentResult() -> HullResult? { lock.withLock { _result } }

    private var rawFloats: [Float] = []
    private let maxFloats = 300_000 * 3
    private let session   = ARSession()

    static let planeColors: [(SIMD3<Float>, String)] = [
        (SIMD3(0.2, 0.5, 1.0), "Plane 1"),
        (SIMD3(0.2, 0.9, 0.4), "Plane 2"),
        (SIMD3(1.0, 0.6, 0.1), "Plane 3"),
        (SIMD3(0.9, 0.2, 0.8), "Plane 4"),
    ]

    override init() {
        super.init()
        startSession()
    }

    private func startSession() {
        let config = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics = .sceneDepth
            statusMessage = "LiDAR active — scan flat surfaces"
        } else {
            statusMessage = "No LiDAR — collecting feature points"
        }
        session.delegate = self
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    func clear() {
        rawFloats = []
        lock.withLock { _result = nil }
        rawCount = 0; hullCount = 0; hullInfos = []; durationLabel = "—"
        statusMessage = "Cleared — move camera to scan"
    }

    func computeHulls() {
        guard !isProcessing, rawFloats.count >= 300 else { return }
        isProcessing  = true
        statusMessage = "Computing convex hulls…"
        let snapshot  = rawFloats
        let t0        = Date()

        Task.detached(priority: .userInitiated) { [weak self] in
            guard let self else { return }
            let result = try? Self.runPipeline(rawFloats: snapshot, startedAt: t0)
            await MainActor.run { [weak self] in
                guard let self else { return }
                if let result {
                    self.lock.withLock { self._result = result }
                    self.hullCount     = result.hulls.count
                    self.durationLabel = String(format: "%.0f ms", result.durationMs)
                    self.hullInfos     = result.hulls.map { h in
                        HullInfo(label: h.label, pts: h.hullVertices.count,
                                 area: h.area, color: h.color)
                    }
                    self.statusMessage = "\(self.hullCount) plane hulls computed"
                } else {
                    self.statusMessage = "Failed — collect more data and retry"
                }
                self.isProcessing = false
            }
        }
    }

    // MARK: - Background pipeline

    nonisolated private static func runPipeline(rawFloats: [Float],
                                                startedAt t0: Date) throws -> HullResult {
        let n   = rawFloats.count / 3
        let raw = try rawFloats.withUnsafeBufferPointer { buf in
            try PointCloud.make(packedXYZ: buf.baseAddress!, count: UInt(n))
        }
        let ds       = try raw.voxelGridDownsampled(leaf: 0.03)
        let filtered = try ds.statisticalOutlierRemoval(meanK: 20, stddevMulThresh: 1.5)

        // Store raw preview data
        let rawCount   = Int(filtered.pointCount)
        var rawBuf     = [Float](repeating: 0, count: rawCount * 3); var actual: UInt = 0
        _ = try rawBuf.withUnsafeMutableBufferPointer { ptr in
            try filtered.copyPackedXYZ(into: ptr.baseAddress!, capacity: UInt(rawCount),
                                        actualCount: &actual)
        }
        let rawData = Array(rawBuf.prefix(Int(actual) * 3))

        // Iterative plane extraction → convex hull per plane
        var remaining = try unpack(filtered)
        var hulls     = [PlaneHull]()
        let threshold: Float = 0.05

        for i in 0..<planeColors.count {
            guard remaining.count >= 50 else { break }

            let packed = pack(remaining)
            let cloud  = try packed.withUnsafeBufferPointer { buf in
                try PointCloud.make(packedXYZ: buf.baseAddress!, count: UInt(remaining.count))
            }
            guard let plane = try? cloud.segmentPlane(distanceThreshold: 0.04,
                                                      maxIterations: 300) else { break }

            let normal  = SIMD3<Float>(plane.a, plane.b, plane.c)
            let nLen    = simd_length(normal)
            var inliers  = [SIMD3<Float>]()
            var outliers = [SIMD3<Float>]()
            for p in remaining {
                if abs(simd_dot(normal, p) + plane.d) / nLen < threshold {
                    inliers.append(p)
                } else {
                    outliers.append(p)
                }
            }
            guard inliers.count >= 20 else { remaining = outliers; continue }

            // Project inliers onto the plane → convex hull
            let inPacked = pack(inliers)
            guard let inCloud = try? inPacked.withUnsafeBufferPointer({ buf in
                try PointCloud.make(packedXYZ: buf.baseAddress!, count: UInt(inliers.count))
            }),
            let projected = try? inCloud.projectInliersToPlane(distanceThreshold: 0.04,
                                                               maxIterations: 300),
            let hullCloud = try? projected.convexHull()
            else { remaining = outliers; continue }

            let hullPts  = try unpack(hullCloud)
            guard hullPts.count >= 3 else { remaining = outliers; continue }

            let centroid = sumSIMD3(hullPts) / Float(hullPts.count)
            let area     = approxPolygonArea(hullPts, normal: normal / nLen)
            let (color, label) = planeColors[i]
            hulls.append(PlaneHull(hullVertices: hullPts, centroid: centroid,
                                   area: area, color: color, label: label))
            remaining = outliers
        }

        return HullResult(hulls: hulls, rawData: rawData,
                          durationMs: Date().timeIntervalSince(t0) * 1000)
    }

    // Shoelace formula projected onto the plane's local 2D axes
    nonisolated private static func approxPolygonArea(_ pts: [SIMD3<Float>],
                                                      normal: SIMD3<Float>) -> Float {
        guard pts.count >= 3 else { return 0 }
        // Build local 2D frame on the plane
        let up  = abs(normal.y) < 0.9 ? SIMD3<Float>(0,1,0) : SIMD3<Float>(1,0,0)
        let u   = simd_normalize(simd_cross(normal, up))
        let v   = simd_cross(normal, u)
        let pts2 = pts.map { SIMD2<Float>(simd_dot($0, u), simd_dot($0, v)) }
        var area: Float = 0
        let n = pts2.count
        for i in 0..<n {
            let j = (i + 1) % n
            area += pts2[i].x * pts2[j].y - pts2[j].x * pts2[i].y
        }
        return abs(area) * 0.5
    }

    nonisolated private static func unpack(_ cloud: PointCloud) throws -> [SIMD3<Float>] {
        let n = Int(cloud.pointCount)
        var buf = [Float](repeating: 0, count: n * 3); var actual: UInt = 0
        _ = try buf.withUnsafeMutableBufferPointer { ptr in
            try cloud.copyPackedXYZ(into: ptr.baseAddress!, capacity: UInt(n), actualCount: &actual)
        }
        return (0..<Int(actual)).map { i in SIMD3(buf[i*3], buf[i*3+1], buf[i*3+2]) }
    }

    nonisolated private static func pack(_ pts: [SIMD3<Float>]) -> [Float] {
        var out = [Float](); out.reserveCapacity(pts.count * 3)
        for p in pts { out.append(p.x); out.append(p.y); out.append(p.z) }
        return out
    }

    private func appendDepth(_ pts: [Float]) {
        rawFloats.append(contentsOf: pts)
        if rawFloats.count > maxFloats { rawFloats = Array(rawFloats.suffix(maxFloats)) }
        rawCount = rawFloats.count / 3
        statusMessage = rawCount < 1000 ? "Scanning… (\(rawCount) pts)"
                                        : "\(rawCount) pts — tap Compute Hulls"
    }
}

extension ConvexHullCoordinator: ARSessionDelegate {

    nonisolated func session(_ session: ARSession, didUpdate frame: ARFrame) {
        var pts = [Float]()
        if let depth = frame.sceneDepth {
            let dm  = depth.depthMap
            let dW  = CVPixelBufferGetWidth(dm); let dH = CVPixelBufferGetHeight(dm)
            let img = frame.camera.imageResolution
            let sx  = Float(dW)/Float(img.width); let sy = Float(dH)/Float(img.height)
            let K   = frame.camera.intrinsics
            let fx  = K[0][0]*sx; let fy = K[1][1]*sy; let cx = K[2][0]*sx; let cy = K[2][1]*sy
            let T   = frame.camera.transform
            CVPixelBufferLockBaseAddress(dm, .readOnly)
            let base = CVPixelBufferGetBaseAddress(dm)!.assumingMemoryBound(to: Float32.self)
            for row in Swift.stride(from: 0, to: dH, by: 3) {
                for col in Swift.stride(from: 0, to: dW, by: 3) {
                    let d = base[row*dW+col]
                    guard d > 0.15 && d < 6.0 else { continue }
                    let xc = (Float(col)-cx)/fx*d; let yc = -(Float(row)-cy)/fy*d
                    let w  = T * SIMD4<Float>(xc, yc, -d, 1)
                    pts.append(w.x); pts.append(w.y); pts.append(w.z)
                }
            }
            CVPixelBufferUnlockBaseAddress(dm, .readOnly)
        } else if let fp = frame.rawFeaturePoints {
            for p in fp.points { pts.append(p.x); pts.append(p.y); pts.append(p.z) }
        }
        guard !pts.isEmpty else { return }
        Task { @MainActor [weak self] in self?.appendDepth(pts) }
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

private func sumSIMD3(_ pts: [SIMD3<Float>]) -> SIMD3<Float> {
    pts.reduce(SIMD3<Float>.zero) { SIMD3($0.x + $1.x, $0.y + $1.y, $0.z + $1.z) }
}
