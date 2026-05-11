// PCLPipeline.swift
//
// Stateless PCLMobile processing pipeline.
//
// Stage order:
//   raw XYZ → PointCloud → voxelGrid → SOR → (boundsAndCentroid) →
//   segmentPlane → extractPlaneInliers
//
// All stages are pure functions; call this from a background Task.

import Foundation
import simd
import PCLMobile

struct PCLPipelineResult {
    /// Points after voxel-grid downsample + statistical outlier removal.
    let filteredPoints: [simd_float3]
    /// Subset of filteredPoints classified as plane inliers (orange layer).
    let planePoints:    [simd_float3]
    /// RANSAC plane model, or nil when no dominant plane was found.
    let planeModel:     PlaneModel?
    /// Centroid of the filtered cloud (used to place the normal arrow).
    let centroid:       simd_float3?
    /// Wall-clock time for the whole pipeline.
    let durationMs:     Double
}

enum PCLPipeline {

    // MARK: - Public entry point

    /// Run the full pipeline on a packed `[x,y,z,x,y,z,…]` float buffer.
    ///
    /// This function is **not** isolated to any actor — call it from a
    /// detached/background Task.
    static func run(rawXYZ: [Float]) throws -> PCLPipelineResult {
        let t0 = Date()
        let rawCount = rawXYZ.count / 3
        guard rawCount >= 3 else {
            throw pipelineError("need at least 3 points, got \(rawCount)")
        }

        // 1. Build PointCloud from the packed buffer.
        let cloud: PointCloud = try rawXYZ.withUnsafeBufferPointer { buf in
            guard let base = buf.baseAddress else {
                throw pipelineError("null buffer")
            }
            return try PointCloud.make(packedXYZ: base, count: UInt(rawCount))
        }

        // 2. Voxel-grid downsample (2 cm leaf → manageable density).
        let downsampled = try cloud.voxelGridDownsampled(leaf: 0.02)

        // 3. Statistical outlier removal (k=20 neighbours, 1.5 σ).
        let filtered = try downsampled.statisticalOutlierRemoval(
            meanK: 20, stddevMulThresh: 1.5)

        // 4. Bounding-box centroid via PCLMobile (backed by Eigen).
        let bounds  = try? filtered.boundsAndCentroid()
        let centroid = bounds.map { simd_float3($0.centroidX, $0.centroidY, $0.centroidZ) }

        // 5. RANSAC plane segmentation.
        let planeModel = try? filtered.segmentPlane(
            distanceThreshold: 0.02, maxIterations: 200)

        // 6. Extract plane inliers when a plane was found.
        let planeCloud: PointCloud? = planeModel != nil
            ? try? filtered.extractPlaneInliers(
                distanceThreshold: 0.02, maxIterations: 200)
            : nil

        return PCLPipelineResult(
            filteredPoints: try unpack(filtered),
            planePoints:    try unpack(planeCloud),
            planeModel:     planeModel,
            centroid:       centroid,
            durationMs:     Date().timeIntervalSince(t0) * 1000
        )
    }

    // MARK: - Private helpers

    /// Copy packed float data from a PointCloud into [simd_float3].
    private static func unpack(_ cloud: PointCloud?) throws -> [simd_float3] {
        guard let cloud, cloud.pointCount > 0 else { return [] }
        let n   = Int(cloud.pointCount)
        var buf = [Float](repeating: 0, count: n * 3)
        var actual: UInt = 0
        _ = try buf.withUnsafeMutableBufferPointer { ptr in
            try cloud.copyPackedXYZ(into:        ptr.baseAddress!,
                                    capacity:    UInt(n),
                                    actualCount: &actual)
        }
        return (0..<Int(actual)).map { i in
            simd_float3(buf[i * 3], buf[i * 3 + 1], buf[i * 3 + 2])
        }
    }

    private static func pipelineError(_ msg: String) -> Error {
        NSError(domain: "PCLPipeline", code: 1,
                userInfo: [NSLocalizedDescriptionKey: msg])
    }
}
