// ARPointCloudCoordinator.swift
//
// Bridges ARKit feature-point output to PCLMobile.
//
// On every "Capture & process" tap, the coordinator pulls
// `ARFrame.rawFeaturePoints` (or the union of all `ARMeshAnchor`s on LiDAR
// devices), constructs a PCLMPointCloud, runs voxel grid downsample and a
// pass-through filter on the z-axis, and exposes the resulting counts on
// the published properties so the SwiftUI ContentView can show them live.

import ARKit
import Foundation
import PCLMobile
import simd

@MainActor
final class ARPointCloudCoordinator: NSObject, ObservableObject, ARSessionDelegate {

    // MARK: - Published state read by the view
    @Published var lastRawCount: Int = 0
    @Published var lastDownsampledCount: Int = 0
    @Published var lastFilteredCount: Int = 0
    @Published var lastError: String?
    @Published var lastSavedURL: URL?

    // Tunable processing parameters.
    let voxelLeaf: Float = 0.02   // 2 cm
    let zMin: Float = -1.5
    let zMax: Float = 1.5

    // MARK: - Internal
    private weak var session: ARSession?
    private var lastFrame: ARFrame?
    private var lastFiltered: PCLMPointCloud?

    func attach(session: ARSession) {
        self.session = session
    }

    // MARK: ARSessionDelegate
    nonisolated func session(_ session: ARSession, didUpdate frame: ARFrame) {
        Task { @MainActor in
            self.lastFrame = frame
        }
    }

    nonisolated func session(_ session: ARSession,
                             didFailWithError error: Error) {
        Task { @MainActor in
            self.lastError = error.localizedDescription
        }
    }

    // MARK: - Actions
    func captureNow() {
        lastError = nil
        lastSavedURL = nil

        guard let frame = lastFrame ?? session?.currentFrame else {
            lastError = "no ARFrame available yet"
            return
        }

        // 1. Pull a flat [x,y,z,...] buffer out of ARKit.
        let rawXYZ = collectXYZ(from: frame)
        let rawCount = rawXYZ.count / 3
        lastRawCount = rawCount
        guard rawCount > 0 else {
            lastDownsampledCount = 0
            lastFilteredCount = 0
            lastFiltered = nil
            return
        }

        // 2. Hand the buffer to PCLMobile.
        let cloud: PCLMPointCloud
        do {
            cloud = try rawXYZ.withUnsafeBufferPointer { buf -> PCLMPointCloud in
                guard let base = buf.baseAddress else {
                    throw NSError(domain: "PCLMobileARSample", code: -1)
                }
                return try PCLMPointCloud.make(packedXYZ: base,
                                                count: UInt(rawCount))
            }
        } catch {
            lastError = "make(packedXYZ:) failed: \(error.localizedDescription)"
            return
        }

        // 3. Voxel grid downsample.
        let down: PCLMPointCloud
        do {
            down = try cloud.voxelGridDownsampled(leaf: Double(voxelLeaf))
        } catch {
            lastError = "voxelGrid failed: \(error.localizedDescription)"
            return
        }
        lastDownsampledCount = Int(down.pointCount)

        // 4. Pass-through filter on z so we keep only points roughly at
        //    head-height in front of the device.
        let filtered: PCLMPointCloud
        do {
            filtered = try down.passThroughFiltered(axis: "z",
                                                     min: Double(zMin),
                                                     max: Double(zMax))
        } catch {
            lastError = "passThrough failed: \(error.localizedDescription)"
            return
        }
        lastFilteredCount = Int(filtered.pointCount)
        lastFiltered = filtered
    }

    func savePCD() {
        guard let cloud = lastFiltered, cloud.pointCount > 0 else {
            lastError = "no processed cloud to save — tap Capture first."
            return
        }

        let docs = FileManager.default.urls(for: .documentDirectory,
                                            in: .userDomainMask).first!
        let formatter = ISO8601DateFormatter()
        formatter.formatOptions = [.withFullDate, .withTime, .withColonSeparatorInTime]
        let stamp = formatter.string(from: Date())
            .replacingOccurrences(of: ":", with: "-")
        let url = docs.appendingPathComponent("pcl-mobile-\(stamp).pcd")

        do {
            try cloud.write(pcdAt: url.path)
            lastSavedURL = url
        } catch {
            lastError = "write(pcdAt:) failed: \(error.localizedDescription)"
        }
    }

    // MARK: - Private helpers

    /// Pull a flat `[x,y,z,...]` buffer out of an ARFrame.
    /// On LiDAR devices we prefer the union of all `ARMeshAnchor`s (much
    /// denser); otherwise fall back to `rawFeaturePoints` (sparse, ~hundreds
    /// of points per frame).
    private func collectXYZ(from frame: ARFrame) -> [Float] {
        // 1. LiDAR mesh anchors, if any.
        let meshAnchors = frame.anchors.compactMap { $0 as? ARMeshAnchor }
        if !meshAnchors.isEmpty {
            var out: [Float] = []
            out.reserveCapacity(
                meshAnchors.reduce(0) { $0 + $1.geometry.vertices.count } * 3)
            for anchor in meshAnchors {
                let vertices = anchor.geometry.vertices
                // `vertices.buffer` is a Metal buffer. Per anchor offset
                // (vertices.offset) and stride (vertices.stride) tell us
                // where each vertex sits inside the buffer.
                let basePtr = vertices.buffer.contents()
                    .advanced(by: vertices.offset)
                let stride = vertices.stride
                let xform  = anchor.transform
                for i in 0..<vertices.count {
                    let vertexPtr = basePtr.advanced(by: i * stride)
                        .assumingMemoryBound(to: SIMD3<Float>.self)
                    let local = vertexPtr.pointee
                    let world = xform * SIMD4<Float>(local, 1)
                    out.append(world.x); out.append(world.y); out.append(world.z)
                }
            }
            return out
        }

        // 2. Sparse raw feature points.
        guard let raw = frame.rawFeaturePoints else { return [] }
        var out = [Float]()
        out.reserveCapacity(raw.points.count * 3)
        for p in raw.points {
            out.append(p.x); out.append(p.y); out.append(p.z)
        }
        return out
    }
}
