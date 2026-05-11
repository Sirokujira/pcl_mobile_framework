// ScanExportCoordinator.swift — PCLScanExportSample
//
// Accumulates LiDAR depth as world-space points, then:
//   filter()  → voxelGrid(0.03) → SOR(20,1.5) → rebuild Metal buffer
//   export()  → PCLMobile write(pcdAt:) → sets exportURL for share sheet
//
// .pcd files (ASCII format) can be opened in MeshLab, CloudCompare, Open3D, etc.

import ARKit
import PCLMobile
import simd
import Foundation
import Metal

@MainActor
final class ScanExportCoordinator: NSObject, ObservableObject {

    @Published var statusMessage   = "Move camera to collect depth data"
    @Published var rawCount        = 0
    @Published var filteredCount   = 0
    @Published var durationLabel   = "—"
    @Published var exportSizeKB    = 0
    @Published var isProcessing    = false
    @Published var exportURL: URL? = nil

    let device = MTLCreateSystemDefaultDevice()!

    private let lock = NSLock()
    private var _renderBuf:   MTLBuffer?
    private var _renderCount: Int = 0

    func currentCloud() -> (MTLBuffer, Int)? {
        lock.withLock { _renderBuf.map { ($0, _renderCount) } }
    }

    private var rawFloats: [Float] = []
    private let maxFloats = 500_000 * 3
    private let session   = ARSession()

    override init() {
        super.init()
        startSession()
    }

    private func startSession() {
        let config = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics = .sceneDepth
            statusMessage = "LiDAR active — scan the room"
        } else {
            statusMessage = "No LiDAR — collecting feature points"
        }
        session.delegate = self
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    func clear() {
        rawFloats = []
        lock.withLock { _renderBuf = nil; _renderCount = 0 }
        rawCount = 0; filteredCount = 0; exportSizeKB = 0; exportURL = nil
        durationLabel = "—"
        statusMessage = "Cleared — move camera to scan"
    }

    // MARK: - PCL Filter

    func filter() {
        guard !isProcessing, rawFloats.count >= 300 else { return }
        isProcessing  = true
        statusMessage = "Filtering…"
        let snapshot  = rawFloats
        let t0        = Date()

        Task.detached(priority: .userInitiated) { [weak self] in
            guard let self else { return }
            let result = try? Self.runFilter(rawFloats: snapshot)
            await MainActor.run { [weak self] in
                guard let self else { return }
                if let (pts, ms) = result {
                    self.uploadBuffer(pts)
                    self.filteredCount = pts.count / 3
                    self.durationLabel = String(format: "%.0f ms", ms)
                    self.statusMessage = "\(self.filteredCount) pts filtered — tap Export"
                } else {
                    self.statusMessage = "Filter failed — collect more data"
                }
                self.isProcessing = false
            }
        }
    }

    // MARK: - Export PCD

    func export() {
        guard !isProcessing, filteredCount > 0 else { return }
        guard let (renderBuf, count) = lock.withLock({ _renderBuf.map { ($0, _renderCount) } })
        else { return }
        isProcessing  = true
        statusMessage = "Exporting…"

        // Read current filtered points from MTLBuffer
        let floatPtr = renderBuf.contents().assumingMemoryBound(to: Float.self)
        var floats = [Float](repeating: 0, count: count * 3)
        floats.withUnsafeMutableBufferPointer { buf in
            buf.baseAddress!.assign(from: floatPtr, count: count * 3)
        }

        Task.detached(priority: .userInitiated) { [weak self] in
            guard let self else { return }
            let url = try? Self.writePCD(floats: floats, count: count)
            await MainActor.run { [weak self] in
                guard let self else { return }
                if let url {
                    let kb = (try? url.resourceValues(forKeys: [.fileSizeKey]).fileSize) ?? 0
                    self.exportSizeKB  = kb / 1024
                    self.exportURL     = url
                    self.statusMessage = "Exported \(self.filteredCount) pts (\(self.exportSizeKB) KB)"
                } else {
                    self.statusMessage = "Export failed"
                }
                self.isProcessing = false
            }
        }
    }

    // MARK: - Background helpers

    nonisolated private static func runFilter(rawFloats: [Float]) throws -> ([Float], Double) {
        let t0  = Date()
        let n   = rawFloats.count / 3
        let raw = try rawFloats.withUnsafeBufferPointer { buf in
            try PointCloud.make(packedXYZ: buf.baseAddress!, count: UInt(n))
        }
        let ds = try raw.voxelGridDownsampled(leaf: 0.03)
        let cl = try ds.statisticalOutlierRemoval(meanK: 20, stddevMulThresh: 1.5)
        let cn = Int(cl.pointCount)
        var out = [Float](repeating: 0, count: cn * 3); var actual: UInt = 0
        _ = try out.withUnsafeMutableBufferPointer { ptr in
            try cl.copyPackedXYZ(into: ptr.baseAddress!, capacity: UInt(cn), actualCount: &actual)
        }
        return (Array(out.prefix(Int(actual) * 3)), Date().timeIntervalSince(t0) * 1000)
    }

    nonisolated private static func writePCD(floats: [Float], count: Int) throws -> URL {
        let cloud = try floats.withUnsafeBufferPointer { buf in
            try PointCloud.make(packedXYZ: buf.baseAddress!, count: UInt(count))
        }
        let url = URL.temporaryDirectory.appendingPathComponent("scan_\(Int(Date().timeIntervalSince1970)).pcd")
        try cloud.write(pcdAt: url.path)
        return url
    }

    private func uploadBuffer(_ floats: [Float]) {
        let len = floats.count * MemoryLayout<Float>.size
        guard len > 0 else { return }
        let buf = device.makeBuffer(bytes: floats, length: len, options: .storageModeShared)
        lock.withLock { _renderBuf = buf; _renderCount = floats.count / 3 }
    }

    private func appendDepth(_ newFloats: [Float]) {
        rawFloats.append(contentsOf: newFloats)
        if rawFloats.count > maxFloats { rawFloats = Array(rawFloats.suffix(maxFloats)) }
        rawCount = rawFloats.count / 3
        if rawCount < 1000 {
            statusMessage = "Scanning… (\(rawCount) pts)"
        } else {
            statusMessage = "\(rawCount) pts — tap Filter"
        }
    }
}

extension ScanExportCoordinator: ARSessionDelegate {

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
                    guard d > 0.15 && d < 8.0 else { continue }
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
