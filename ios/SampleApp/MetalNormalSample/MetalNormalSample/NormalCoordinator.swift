import ARKit
import PCLMobile
import simd
import Foundation
import Metal

struct NormalPoint {
    var position: SIMD3<Float>
    var normal:   SIMD3<Float>
}

@MainActor
final class NormalCoordinator: NSObject, ObservableObject {

    @Published var statusMessage  = "Move camera to collect depth data"
    @Published var rawCount       = 0
    @Published var normalCount    = 0
    @Published var durationLabel  = "—"
    @Published var isProcessing   = false

    let device = MTLCreateSystemDefaultDevice()!

    private let lock = NSLock()
    private var _posBuf:    MTLBuffer?
    private var _normalBuf: MTLBuffer?
    private var _ptCount:   Int = 0

    func currentBuffers() -> (pos: MTLBuffer, normal: MTLBuffer, count: Int)? {
        lock.withLock {
            guard let p = _posBuf, let n = _normalBuf, _ptCount > 0 else { return nil }
            return (p, n, _ptCount)
        }
    }

    private var rawFloats: [Float] = []
    private let maxFloats = 300_000 * 3

    private let session = ARSession()

    override init() {
        super.init()
        startSession()
    }

    private func startSession() {
        let config = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics = .sceneDepth
            statusMessage = "LiDAR active — move camera to scan"
        } else {
            statusMessage = "No LiDAR — collecting feature points"
        }
        session.delegate = self
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    func clear() {
        rawFloats = []
        lock.withLock { _posBuf = nil; _normalBuf = nil; _ptCount = 0 }
        rawCount = 0; normalCount = 0
        durationLabel = "—"
        statusMessage = "Cleared — move camera to scan"
    }

    func estimateNormals() {
        guard !isProcessing, rawFloats.count >= 150 else { return }
        isProcessing  = true
        statusMessage = "Estimating normals…"
        let snapshot  = rawFloats
        let t0        = Date()

        Task.detached(priority: .userInitiated) { [weak self] in
            guard let self else { return }
            let result = try? Self.runNormalPipeline(rawFloats: snapshot, startedAt: t0)
            await MainActor.run { [weak self] in
                guard let self else { return }
                if let (positions, normals, ms) = result, !positions.isEmpty {
                    self.uploadBuffers(positions: positions, normals: normals)
                    self.normalCount  = positions.count
                    self.durationLabel = String(format: "%.0f ms", ms)
                    self.statusMessage = "\(positions.count) pts with normals"
                } else {
                    self.statusMessage = "Estimation failed — collect more data"
                }
                self.isProcessing = false
            }
        }
    }

    private func uploadBuffers(positions: [SIMD3<Float>], normals: [SIMD3<Float>]) {
        let n = positions.count
        // Pack as packed_float3 (12 bytes each)
        var posData   = [Float](); posData.reserveCapacity(n * 3)
        var normData  = [Float](); normData.reserveCapacity(n * 3)
        for i in 0..<n {
            posData.append(positions[i].x); posData.append(positions[i].y); posData.append(positions[i].z)
            normData.append(normals[i].x);  normData.append(normals[i].y);  normData.append(normals[i].z)
        }
        let posLen  = posData.count  * MemoryLayout<Float>.size
        let normLen = normData.count * MemoryLayout<Float>.size
        let pb = device.makeBuffer(bytes: posData,  length: posLen,  options: .storageModeShared)
        let nb = device.makeBuffer(bytes: normData, length: normLen, options: .storageModeShared)
        lock.withLock { _posBuf = pb; _normalBuf = nb; _ptCount = n }
    }

    nonisolated private static func runNormalPipeline(
        rawFloats: [Float], startedAt t0: Date
    ) throws -> ([SIMD3<Float>], [SIMD3<Float>], Double)? {
        let rawCount = rawFloats.count / 3
        guard rawCount >= 50 else { return nil }

        let rawCloud = try rawFloats.withUnsafeBufferPointer { buf in
            try PointCloud.make(packedXYZ: buf.baseAddress!, count: UInt(rawCount))
        }
        let downsampled = try rawCloud.voxelGridDownsampled(leaf: 0.05)
        let filtered    = try downsampled.statisticalOutlierRemoval(meanK: 20, stddevMulThresh: 1.5)
        let n = Int(filtered.pointCount)
        guard n >= 10 else { return nil }

        // Unpack positions
        var posBuf = [Float](repeating: 0, count: n * 3)
        var actual: UInt = 0
        _ = try posBuf.withUnsafeMutableBufferPointer { ptr in
            try filtered.copyPackedXYZ(into: ptr.baseAddress!, capacity: UInt(n), actualCount: &actual)
        }
        let positions: [SIMD3<Float>] = (0..<Int(actual)).map { i in
            SIMD3<Float>(posBuf[i*3], posBuf[i*3+1], posBuf[i*3+2])
        }

        // Estimate normals — returns (nx,ny,nz,curvature) × n floats
        let kSearch = min(15, n - 1)
        let normalData = try filtered.estimateNormals(kSearch: kSearch)
        let normals = parseNormals(normalData as NSData, count: Int(actual))

        let ms = Date().timeIntervalSince(t0) * 1000
        return (positions, normals, ms)
    }

    nonisolated private static func parseNormals(_ data: NSData, count: Int) -> [SIMD3<Float>] {
        let ptr = data.bytes.assumingMemoryBound(to: Float.self)
        return (0..<count).map { i in SIMD3<Float>(ptr[i*4], ptr[i*4+1], ptr[i*4+2]) }
    }

    private func appendDepthPoints(_ newFloats: [Float]) {
        rawFloats.append(contentsOf: newFloats)
        if rawFloats.count > maxFloats { rawFloats = Array(rawFloats.suffix(maxFloats)) }
        rawCount = rawFloats.count / 3
        if rawCount < 500 {
            statusMessage = "Scanning… (\(rawCount) pts)"
        } else {
            statusMessage = "\(rawCount) pts — tap Estimate Normals"
        }
    }
}

extension NormalCoordinator: ARSessionDelegate {

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
            for row in Swift.stride(from: 0, to: dH, by: 4) {
                for col in Swift.stride(from: 0, to: dW, by: 4) {
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
