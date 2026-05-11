// ARPointCloudCoordinator.swift
//
// @MainActor ObservableObject that:
//   • acts as ARSessionDelegate to accumulate feature points across frames
//   • drives the PCLPipeline on a background Task
//   • pushes rendering commands back to ARViewController on the main actor
//
// Points are stored as a flat [x,y,z,...] Float buffer (12-byte stride) so
// they can be handed directly to PointCloud.make(packedXYZ:count:) without
// an intermediate conversion step.

import ARKit
import Foundation
import PCLMobile
import simd

@MainActor
final class ARPointCloudCoordinator: NSObject, ObservableObject {

    // MARK: - Published (read by ContentView)

    @Published var rawCount:      Int    = 0
    @Published var filteredCount: Int    = 0
    @Published var planeCount:    Int    = 0
    @Published var durationMs:    Double = 0
    @Published var isProcessing:  Bool   = false
    @Published var statusMessage: String = "Move camera to collect points"
    @Published var errorMessage:    String?
    @Published var planeInfo:       String?
    @Published var lastSavedURL:    URL?
    @Published var showResultsSheet: Bool = false

    // MARK: - Back-reference to the rendering VC

    /// Set by ARViewContainer before the session starts.
    weak var viewController: ARViewController?

    // MARK: - Internal

    private weak var session: ARSession?
    private(set) var lastResult: PCLPipelineResult?

    // Packed float buffer — avoids SIMD3<Float>'s 16-byte stride overhead.
    private var xyz: [Float] = []
    private let maxPoints    = 20_000   // cap total accumulated points

    // Skip rendering every other update to reduce GPU pressure.
    private var frameIndex = 0

    // MARK: - ARSession attachment

    func attach(session: ARSession) {
        self.session = session
    }

    // MARK: - Actions

    func captureAndProcess() {
        guard !isProcessing else { return }
        errorMessage  = nil
        lastSavedURL  = nil

        // Prefer accumulated points; fall back to a single current frame.
        var source = xyz
        if source.count < 9, let frame = session?.currentFrame {
            source = Self.extractXYZ(from: frame)
        }
        guard source.count >= 9 else {
            errorMessage = "Not enough points — move the camera."
            return
        }

        let snapshot = source
        isProcessing  = true
        statusMessage = "Processing \(snapshot.count / 3) points…"

        Task.detached(priority: .userInitiated) { [weak self] in
            guard let self else { return }
            do {
                let result = try PCLPipeline.run(rawXYZ: snapshot)
                await MainActor.run {
                    self.lastResult       = result
                    self.filteredCount    = result.filteredPoints.count
                    self.planeCount       = result.planePoints.count
                    self.durationMs       = result.durationMs
                    self.isProcessing     = false
                    self.showResultsSheet = true
                    self.updatePlaneInfo(result.planeModel)
                    self.viewController?.showProcessedResult(result)
                }
            } catch {
                await MainActor.run {
                    self.isProcessing  = false
                    self.errorMessage  = error.localizedDescription
                    self.statusMessage = "Processing failed"
                }
            }
        }
    }

    func savePCD() {
        guard let result = lastResult, !result.filteredPoints.isEmpty else {
            errorMessage = "No processed cloud — tap Capture & Analyse first."
            return
        }

        let packed: [Float] = result.filteredPoints.flatMap { [$0.x, $0.y, $0.z] }
        let cloud: PointCloud
        do {
            cloud = try packed.withUnsafeBufferPointer { buf in
                try PointCloud.make(packedXYZ: buf.baseAddress!,
                                    count: UInt(result.filteredPoints.count))
            }
        } catch {
            errorMessage = "Failed to build cloud: \(error.localizedDescription)"
            return
        }

        let docs = FileManager.default.urls(for: .documentDirectory,
                                             in: .userDomainMask).first!
        let stamp = ISO8601DateFormatter().string(from: Date())
            .replacingOccurrences(of: ":", with: "-")
        let url = docs.appendingPathComponent("pcl-ar-\(stamp).pcd")
        do {
            try cloud.write(pcdAt: url.path)
            lastSavedURL  = url
            statusMessage = "Saved \(url.lastPathComponent)"
        } catch {
            errorMessage = "Save failed: \(error.localizedDescription)"
        }
    }

    func clear() {
        xyz           = []
        lastResult    = nil
        rawCount      = 0
        filteredCount = 0
        planeCount    = 0
        durationMs    = 0
        planeInfo     = nil
        errorMessage  = nil
        lastSavedURL  = nil
        statusMessage = "Cleared — move camera to collect points"
        viewController?.clearAll()
    }

    // MARK: - Private helpers

    private func updatePlaneInfo(_ model: PlaneModel?) {
        guard let m = model else {
            planeInfo     = nil
            statusMessage = "Done — no dominant plane found"
            return
        }
        let ratio = Int(100 * Double(m.inlierCount) / Double(max(1, m.inputCount)))
        planeInfo     = String(format: "n=(%.2f, %.2f, %.2f)  d=%.2f  %d%% inliers",
                               m.a, m.b, m.c, m.d, ratio)
        statusMessage = "Done — plane detected"
    }

    /// Extract a packed [x,y,z,...] float buffer from an ARFrame.
    /// On LiDAR devices: uses ARMeshAnchor vertex buffers (dense).
    /// Otherwise:         uses rawFeaturePoints (sparse, ~hundreds/frame).
    ///
    /// This is a pure function with no actor-isolated state, declared as a
    /// static nonisolated func so it can be called from nonisolated context.
    nonisolated static func extractXYZ(from frame: ARFrame) -> [Float] {
        let meshAnchors = frame.anchors.compactMap { $0 as? ARMeshAnchor }
        if !meshAnchors.isEmpty {
            var out = [Float]()
            for anchor in meshAnchors {
                let v     = anchor.geometry.vertices
                let base  = v.buffer.contents().advanced(by: v.offset)
                let xform = anchor.transform
                for i in 0..<v.count {
                    let local = base.advanced(by: i * v.stride)
                        .assumingMemoryBound(to: SIMD3<Float>.self).pointee
                    let world = xform * SIMD4<Float>(local, 1)
                    out.append(world.x); out.append(world.y); out.append(world.z)
                }
            }
            return out
        }

        guard let raw = frame.rawFeaturePoints else { return [] }
        var out = [Float]()
        out.reserveCapacity(raw.points.count * 3)
        for p in raw.points { out.append(p.x); out.append(p.y); out.append(p.z) }
        return out
    }
}

// MARK: - ARSessionDelegate

extension ARPointCloudCoordinator: ARSessionDelegate {

    nonisolated func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let newXYZ = Self.extractXYZ(from: frame)
        guard !newXYZ.isEmpty else { return }

        Task { @MainActor [weak self] in
            guard let self else { return }
            self.xyz.append(contentsOf: newXYZ)
            let cap = self.maxPoints * 3
            if self.xyz.count > cap {
                self.xyz = Array(self.xyz.suffix(cap))
            }
            let count = self.xyz.count / 3
            self.rawCount = count
            if count < 200 {
                self.statusMessage = "Move camera slowly (\(count) pts)"
            } else if count < 500 {
                self.statusMessage = "Keep moving — \(count) pts collected"
            } else {
                self.statusMessage = "\(count) pts — tap Capture & Analyse"
            }

            // Refresh raw-point layer every 3rd frame.
            self.frameIndex += 1
            guard self.frameIndex % 3 == 0 else { return }
            let pts: [simd_float3] = stride(from: 0,
                                             to: self.xyz.count - 2,
                                             by: 3).map {
                simd_float3(self.xyz[$0], self.xyz[$0 + 1], self.xyz[$0 + 2])
            }
            self.viewController?.showRawCloud(pts)
        }
    }

    nonisolated func session(_ session: ARSession,
                             cameraDidChangeTrackingState camera: ARCamera) {
        let msg: String
        switch camera.trackingState {
        case .notAvailable:               msg = "Tracking unavailable"
        case .limited(.initializing):    msg = "Initialising AR…"
        case .limited(.relocalizing):    msg = "Relocalising…"
        case .limited(.excessiveMotion): msg = "Move camera more slowly"
        case .limited(.insufficientFeatures): msg = "Point at a textured surface"
        case .normal:                     msg = "Tracking normal"
        @unknown default:                 msg = "Unknown tracking state"
        }
        Task { @MainActor [weak self] in
            guard let self, self.rawCount == 0 else { return }
            self.statusMessage = msg
        }
    }

    nonisolated func session(_ session: ARSession, didFailWithError error: Error) {
        Task { @MainActor [weak self] in
            self?.errorMessage = "AR session error: \(error.localizedDescription)"
        }
    }
}
