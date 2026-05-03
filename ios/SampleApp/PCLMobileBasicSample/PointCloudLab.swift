// PointCloudLab.swift
//
// Drives PCLMobile from a synthetic point cloud so the demo works on the
// iOS Simulator with no camera / sensor dependency. Keeps every public
// PCLMobile API on display:
//
//   * `PCLMPointCloud.make(packedXYZ:count:)`     -- build from a buffer
//   * `voxelGridDownsampled(leaf:)`               -- decimate
//   * `passThroughFiltered(axis:min:max:)`        -- crop
//   * `write(pcdAt:)` + `load(pcdAt:)`            -- file round-trip
//   * `copyPackedXYZ(into:capacity:actualCount:)` -- read back to Swift

import Foundation
import PCLMobile

@MainActor
final class PointCloudLab: ObservableObject {

    // MARK: - User-tunable parameters
    @Published var sourcePointCount: Int = 50_000
    @Published var voxelLeaf: Float       = 0.05    // 5 cm
    @Published var passAxis: String       = "z"
    @Published var passMin: Float         = 0.25
    @Published var passMax: Float         = 0.75

    // MARK: - Outputs read by ContentView
    @Published private(set) var sourceCount       = 0
    @Published private(set) var downsampledCount  = 0
    @Published private(set) var filteredCount     = 0
    @Published private(set) var roundTripCount    = 0
    @Published private(set) var saveURL: URL?
    @Published private(set) var lastError: String?
    @Published private(set) var lastDurationMS: Double = 0

    // MARK: - Public actions
    func runPipeline() {
        lastError = nil
        let started = Date()
        do {
            // 1. Synthesise a point cloud filling the unit cube (with a
            //    twist that makes the pass-through and voxel results
            //    visibly different).
            let raw = synthesise(count: sourcePointCount)
            sourceCount = raw.count / 3

            // 2. Hand off to PCLMobile.
            let cloud = try raw.withUnsafeBufferPointer { buf in
                try PCLMPointCloud.make(packedXYZ: buf.baseAddress!,
                                        count: UInt(sourceCount))
            }

            // 3. Voxel grid downsample.
            let down = try cloud.voxelGridDownsampled(leaf: Double(voxelLeaf))
            downsampledCount = Int(down.pointCount)

            // 4. Pass-through filter on the chosen axis.
            let filtered = try down.passThroughFiltered(
                axis: passAxis,
                min: Double(passMin),
                max: Double(passMax))
            filteredCount = Int(filtered.pointCount)

            // 5. Save the filtered cloud as PCD, then reload to verify
            //    the round-trip works.
            let url = makeOutputURL()
            try filtered.write(pcdAt: url.path)
            saveURL = url

            let reloaded = try PCLMPointCloud.load(pcdAt: url.path)
            roundTripCount = Int(reloaded.pointCount)
        } catch {
            lastError = error.localizedDescription
        }
        lastDurationMS = Date().timeIntervalSince(started) * 1000
    }

    func deleteSavedFile() {
        guard let url = saveURL else { return }
        try? FileManager.default.removeItem(at: url)
        saveURL = nil
    }

    // MARK: - Internals

    private func synthesise(count: Int) -> [Float] {
        // Half the points sit on a noisy plane at z ≈ 0.5 (so the
        // passthrough filter on z[0.25..0.75] keeps the plane and
        // discards the rest); the other half are uniformly scattered
        // through the unit cube. Voxel grid then collapses both groups
        // by spatial bucket.
        var rng = SystemRandomNumberGenerator()
        var out = [Float]()
        out.reserveCapacity(count * 3)
        for i in 0..<count {
            let onPlane = (i % 2 == 0)
            let x = Float.random(in: 0...1, using: &rng)
            let y = Float.random(in: 0...1, using: &rng)
            let z: Float
            if onPlane {
                z = 0.5 + Float.random(in: -0.02...0.02, using: &rng)
            } else {
                z = Float.random(in: 0...1, using: &rng)
            }
            out.append(x); out.append(y); out.append(z)
        }
        return out
    }

    private func makeOutputURL() -> URL {
        let docs = FileManager.default.urls(for: .documentDirectory,
                                            in: .userDomainMask).first!
        let stamp = ISO8601DateFormatter()
            .string(from: Date())
            .replacingOccurrences(of: ":", with: "-")
        return docs.appendingPathComponent("pcl-mobile-demo-\(stamp).pcd")
    }
}
