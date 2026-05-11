// ObjectMeasureCoordinator.swift — PCLObjectMeasureSample
//
// Pipeline (triggered by "Measure" button):
//   1. voxelGrid(0.04) + SOR(20,1.5)
//   2. Detect floor: find the RANSAC plane whose normal is most vertical |ny|>0.75
//      and is below the camera — classify floor vs non-floor with Swift-side split
//   3. On non-floor cloud: voxel-BFS clustering (8cm voxels, 26-neighbour adjacency)
//   4. For each cluster ≥ minPts: PCLMobile boundsAndCentroid() → AABB dimensions

import ARKit
import PCLMobile
import simd
import Foundation
import Metal

struct ClusterMesh {
    var floatData: [Float]
    var color:     SIMD3<Float>
    var min:       SIMD3<Float>
    var max:       SIMD3<Float>
    var centroid:  SIMD3<Float>
    var label:     String
}

struct ObjectResult {
    var floorData:  [Float]
    var clusters:   [ClusterMesh]
    var durationMs: Double
}

@MainActor
final class ObjectMeasureCoordinator: NSObject, ObservableObject {

    @Published var statusMessage = "Scan the room with LiDAR"
    @Published var rawCount      = 0
    @Published var clusterCount  = 0
    @Published var floorCount    = 0
    @Published var durationLabel = "—"
    @Published var isProcessing  = false
    @Published var clusterInfos: [ClusterInfo] = []

    struct ClusterInfo: Identifiable {
        let id     = UUID()
        var label: String
        var w, h, d: Float
        var color: SIMD3<Float>
    }

    let device = MTLCreateSystemDefaultDevice()!

    private let lock = NSLock()
    private var _result: ObjectResult?

    func currentResult() -> ObjectResult? { lock.withLock { _result } }

    private var rawFloats: [Float] = []
    private let maxFloats = 300_000 * 3
    private let session   = ARSession()

    // Cluster colors (cycling)
    static let colors: [SIMD3<Float>] = [
        SIMD3(1.0, 0.35, 0.2),
        SIMD3(0.2, 0.85, 1.0),
        SIMD3(1.0, 0.9,  0.1),
        SIMD3(0.6, 1.0,  0.3),
        SIMD3(1.0, 0.4,  1.0),
        SIMD3(0.4, 0.6,  1.0),
    ]

    override init() {
        super.init()
        startSession()
    }

    private func startSession() {
        let config = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics = .sceneDepth
            statusMessage = "LiDAR active — scan floor + objects"
        } else {
            statusMessage = "No LiDAR — collecting feature points"
        }
        session.delegate = self
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    func clear() {
        rawFloats = []
        lock.withLock { _result = nil }
        rawCount = 0; clusterCount = 0; floorCount = 0
        clusterInfos = []; durationLabel = "—"
        statusMessage = "Cleared — move camera to scan"
    }

    func measure() {
        guard !isProcessing, rawFloats.count >= 600 else { return }
        isProcessing  = true
        statusMessage = "Analysing…"
        let snapshot  = rawFloats
        let t0        = Date()

        Task.detached(priority: .userInitiated) { [weak self] in
            guard let self else { return }
            let result = try? Self.runPipeline(rawFloats: snapshot, startedAt: t0)
            await MainActor.run { [weak self] in
                guard let self else { return }
                if let result {
                    self.lock.withLock { self._result = result }
                    self.clusterCount  = result.clusters.count
                    self.floorCount    = result.floorData.count / 3
                    self.durationLabel = String(format: "%.0f ms", result.durationMs)
                    self.clusterInfos  = result.clusters.enumerated().map { i, c in
                        let w = abs(c.max.x - c.min.x)
                        let h = abs(c.max.y - c.min.y)
                        let d = abs(c.max.z - c.min.z)
                        return ClusterInfo(label: c.label, w: w, h: h, d: d, color: c.color)
                    }
                    self.statusMessage = "\(self.clusterCount) objects found"
                } else {
                    self.statusMessage = "Analysis failed — collect more data"
                }
                self.isProcessing = false
            }
        }
    }

    // MARK: - Background PCL pipeline

    nonisolated private static func runPipeline(rawFloats: [Float],
                                                startedAt t0: Date) throws -> ObjectResult {
        let n   = rawFloats.count / 3
        let raw = try rawFloats.withUnsafeBufferPointer { buf in
            try PointCloud.make(packedXYZ: buf.baseAddress!, count: UInt(n))
        }
        let ds       = try raw.voxelGridDownsampled(leaf: 0.04)
        let filtered = try ds.statisticalOutlierRemoval(meanK: 20, stddevMulThresh: 1.5)
        let points   = try unpack(filtered)

        // Detect floor: run up to 3 RANSAC planes, pick the one most vertical (|ny| highest)
        var floorPoints:    [SIMD3<Float>] = []
        var nonFloorPoints: [SIMD3<Float>] = points

        for _ in 0..<3 {
            guard nonFloorPoints.count >= 50 else { break }
            let packed = pack(nonFloorPoints)
            let cloud  = try packed.withUnsafeBufferPointer { buf in
                try PointCloud.make(packedXYZ: buf.baseAddress!, count: UInt(nonFloorPoints.count))
            }
            guard let plane = try? cloud.segmentPlane(distanceThreshold: 0.04, maxIterations: 300)
            else { break }

            let normal = SIMD3<Float>(plane.a, plane.b, plane.c)
            let nLen   = simd_length(normal)
            let ny     = abs(normal.y) / nLen

            var inliers  = [SIMD3<Float>]()
            var outliers = [SIMD3<Float>]()
            for p in nonFloorPoints {
                let dist = abs(simd_dot(normal, p) + plane.d) / nLen
                if dist < 0.05 { inliers.append(p) } else { outliers.append(p) }
            }

            if ny > 0.75 {
                floorPoints.append(contentsOf: inliers)
                nonFloorPoints = outliers
                break
            } else {
                // Not the floor — remove it anyway to expose objects beneath
                nonFloorPoints = outliers
            }
        }

        // Swift voxel-BFS clustering on non-floor points
        let clusters = voxelBFSClusters(points: nonFloorPoints, voxelSize: 0.08, minPts: 80)

        // Build cluster meshes with per-cluster bounds
        var meshes = [ClusterMesh]()
        for (i, idxList) in clusters.enumerated() {
            let pts    = idxList.map { nonFloorPoints[$0] }
            let packed = pack(pts)
            let cloud  = try? packed.withUnsafeBufferPointer { buf in
                try PointCloud.make(packedXYZ: buf.baseAddress!, count: UInt(pts.count))
            }
            guard let cloud, let bounds = try? cloud.boundsAndCentroid() else { continue }
            let color  = colors[i % colors.count]
            meshes.append(ClusterMesh(
                floatData: packed,
                color:     color,
                min:       SIMD3(bounds.minX, bounds.minY, bounds.minZ),
                max:       SIMD3(bounds.maxX, bounds.maxY, bounds.maxZ),
                centroid:  SIMD3(bounds.centroidX, bounds.centroidY, bounds.centroidZ),
                label:     "Object \(i+1)"
            ))
        }

        let floorData = pack(floorPoints)
        return ObjectResult(floorData: floorData, clusters: meshes,
                            durationMs: Date().timeIntervalSince(t0) * 1000)
    }

    // MARK: - Swift voxel-BFS clustering

    private struct VoxelKey: Hashable { let i, j, k: Int }

    nonisolated private static func voxelBFSClusters(points: [SIMD3<Float>],
                                                     voxelSize: Float,
                                                     minPts: Int) -> [[Int]] {
        var voxelMap = [VoxelKey: [Int]]()
        for (idx, p) in points.enumerated() {
            let key = VoxelKey(i: Int(floor(p.x / voxelSize)),
                               j: Int(floor(p.y / voxelSize)),
                               k: Int(floor(p.z / voxelSize)))
            voxelMap[key, default: []].append(idx)
        }

        var visited  = Set<VoxelKey>()
        var clusters = [[Int]]()

        for startKey in voxelMap.keys {
            guard !visited.contains(startKey) else { continue }
            var cluster = [Int]()
            var queue   = [startKey]
            visited.insert(startKey)

            while !queue.isEmpty {
                let key = queue.removeFirst()
                cluster.append(contentsOf: voxelMap[key] ?? [])
                for di in -1...1 {
                    for dj in -1...1 {
                        for dk in -1...1 {
                            guard di != 0 || dj != 0 || dk != 0 else { continue }
                            let nb = VoxelKey(i: key.i+di, j: key.j+dj, k: key.k+dk)
                            if voxelMap[nb] != nil && !visited.contains(nb) {
                                visited.insert(nb); queue.append(nb)
                            }
                        }
                    }
                }
            }
            if cluster.count >= minPts { clusters.append(cluster) }
        }

        // Largest clusters first
        return clusters.sorted { $0.count > $1.count }
    }

    // MARK: - Utility

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

    private func appendDepth(_ newFloats: [Float]) {
        rawFloats.append(contentsOf: newFloats)
        if rawFloats.count > maxFloats { rawFloats = Array(rawFloats.suffix(maxFloats)) }
        rawCount = rawFloats.count / 3
        statusMessage = rawCount < 1000 ? "Scanning… (\(rawCount) pts)"
                                        : "\(rawCount) pts — tap Measure"
    }
}

extension ObjectMeasureCoordinator: ARSessionDelegate {

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
            for row in Swift.stride(from: 0, to: dH, by: 4) {
                for col in Swift.stride(from: 0, to: dW, by: 4) {
                    let d = base[row*dW+col]
                    guard d > 0.1 && d < 5.0 else { continue }
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
