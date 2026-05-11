// PointCloudNode.swift
//
// Builds SCNGeometry point clouds from [simd_float3] arrays.
//
// Swift's SIMD3<Float> (= simd_float3) has stride = 16 bytes on arm64 —
// the fourth float is unused padding.  We pass the full stride to
// SCNGeometrySource so SceneKit reads x,y,z at offset 0 and skips the
// padding, giving correct world-space positions.

import SceneKit
import simd

enum PointCloudNode {

    /// Return an SCNGeometry that renders `points` as coloured dot primitives.
    ///
    /// - Parameters:
    ///   - points:    World-space positions.
    ///   - color:     Uniform diffuse colour applied to every point.
    ///   - pointSize: Base screen-space point diameter in points.
    static func geometry(points: [simd_float3],
                         color: UIColor,
                         pointSize: CGFloat = 5) -> SCNGeometry {

        guard !points.isEmpty else {
            return SCNGeometry()
        }

        // Vertex buffer.
        // MemoryLayout<simd_float3>.stride == 16 on arm64 (not 12).
        let stride     = MemoryLayout<simd_float3>.stride
        let vertexData = Data(bytes: points, count: points.count * stride)

        let source = SCNGeometrySource(
            data:                vertexData,
            semantic:            .vertex,
            vectorCount:         points.count,
            usesFloatComponents: true,
            componentsPerVector: 3,
            bytesPerComponent:   MemoryLayout<Float>.size,
            dataOffset:          0,
            dataStride:          stride         // 16 — SceneKit reads x,y,z and skips pad
        )

        // Index buffer (one Int32 index per point).
        var indices = [Int32](repeating: 0, count: points.count)
        for i in 0..<points.count { indices[i] = Int32(i) }
        let indexData = Data(bytes: indices,
                             count: indices.count * MemoryLayout<Int32>.size)

        let element = SCNGeometryElement(
            data:           indexData,
            primitiveType:  .point,
            primitiveCount: points.count,
            bytesPerIndex:  MemoryLayout<Int32>.size
        )
        element.pointSize                     = pointSize
        element.minimumPointScreenSpaceRadius = 1
        element.maximumPointScreenSpaceRadius = pointSize * 2

        let geo = SCNGeometry(sources: [source], elements: [element])
        let mat = SCNMaterial()
        mat.diffuse.contents = color
        mat.lightingModel    = .constant
        mat.isDoubleSided    = true
        geo.materials        = [mat]
        return geo
    }
}
