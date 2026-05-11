// BodyPCLCoordinator.swift — MetalBodyPCLSample
//
// ARBodyTrackingConfiguration + PCLMobile analysis pipeline.
//
// Every tracked ARFrame with a body anchor:
//   1. Extracts all tracked joint world-space positions (up to 91 joints).
//   2. Feeds them into PCLMobile as a PointCloud.
//   3. Runs:
//        boundsAndCentroid() → body height / width / depth + center of mass
//        estimateNormals(kSearch:) → orientation vectors at each joint
//   4. Publishes rendering data to BodyMetalRenderer via the shared
//      BodyPCLResult snapshot.
//
// PCLMobile is called on a background Task so the main actor stays free.

import ARKit
import PCLMobile
import simd
import Foundation

// Immutable rendering snapshot handed to the Metal renderer each frame.
struct BodyPCLResult {
    let joints:    [SIMD3<Float>]     // world-space joint positions
    let normals:   [SIMD3<Float>]     // PCL-estimated normal at each joint
    let centroid:  SIMD3<Float>
    let minBounds: SIMD3<Float>
    let maxBounds: SIMD3<Float>
    let durationMs: Double
}

@MainActor
final class BodyPCLCoordinator: NSObject, ObservableObject {

    // MARK: - Published UI state

    @Published var statusMessage    = "Initialising body tracking…"
    @Published var jointCount       = 0
    @Published var heightLabel      = "—"
    @Published var widthLabel       = "—"
    @Published var pclDurationLabel = "—"
    @Published var orientationLabel = "n=(—,—,—)"
    @Published var isSupported      = ARBodyTrackingConfiguration.isSupported

    // Render toggles
    @Published var showBBox     = true
    @Published var showNormals  = true
    @Published var showCentroid = true

    // MARK: - Rendering snapshot (read by BodyMetalRenderer under lock)

    private let resultLock = NSLock()
    private var _result: BodyPCLResult?

    func currentResult() -> BodyPCLResult? { resultLock.withLock { _result } }

    // MARK: - ARKit

    private(set) var session = ARSession()
    private var isProcessing = false

    // MARK: - Init

    override init() {
        super.init()
        guard isSupported else { return }
        let config = ARBodyTrackingConfiguration()
        session.delegate = self
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    // MARK: - PCLMobile analysis (background Task)

    private func analyze(joints: [SIMD3<Float>]) {
        guard !isProcessing, joints.count >= 4 else { return }
        isProcessing = true
        let t0 = Date()

        Task.detached(priority: .userInitiated) { [weak self] in
            defer { Task { @MainActor [weak self] in self?.isProcessing = false } }

            guard let self else { return }
            let result = try? Self.runPCL(joints: joints, startedAt: t0)
            guard let result else { return }

            await MainActor.run { [weak self] in
                guard let self else { return }
                self.resultLock.withLock { self._result = result }
                let h = result.maxBounds.y - result.minBounds.y
                let w = result.maxBounds.x - result.minBounds.x
                heightLabel      = String(format: "%.2f m", h)
                widthLabel       = String(format: "%.2f m", w)
                pclDurationLabel = String(format: "%.0f ms", result.durationMs)
                jointCount       = result.joints.count
                if let torsoNormal = result.normals.first {
                    orientationLabel = String(format: "n=(%.2f,%.2f,%.2f)",
                                             torsoNormal.x, torsoNormal.y, torsoNormal.z)
                }
            }
        }
    }

    // MARK: - Pure PCL pipeline (nonisolated, runs on background executor)

    nonisolated private static func runPCL(joints: [SIMD3<Float>],
                                           startedAt t0: Date) throws -> BodyPCLResult {
        // Pack joints into a flat float buffer for PCLMobile.
        var packed = [Float]()
        packed.reserveCapacity(joints.count * 3)
        for j in joints { packed.append(j.x); packed.append(j.y); packed.append(j.z) }

        let cloud: PointCloud = try packed.withUnsafeBufferPointer { buf in
            guard let base = buf.baseAddress else { throw PCLError.nullBuffer }
            return try PointCloud.make(packedXYZ: base, count: UInt(joints.count))
        }

        // Bounding box + centroid.
        let bounds = try cloud.boundsAndCentroid()

        // Normal estimation: kSearch = min(5, N-1) to handle small joint counts.
        let k = max(1, min(5, joints.count - 1))
        let normalData = try cloud.estimateNormals(kSearch: k)
        let normals    = parseNormals(normalData as NSData, count: joints.count)

        return BodyPCLResult(
            joints:    joints,
            normals:   normals,
            centroid:  SIMD3<Float>(bounds.centroidX, bounds.centroidY, bounds.centroidZ),
            minBounds: SIMD3<Float>(bounds.minX, bounds.minY, bounds.minZ),
            maxBounds: SIMD3<Float>(bounds.maxX, bounds.maxY, bounds.maxZ),
            durationMs: Date().timeIntervalSince(t0) * 1000
        )
    }

    /// Decode PCLMobile normal NSData: each point has (nx, ny, nz, curvature).
    nonisolated private static func parseNormals(_ data: NSData, count: Int) -> [SIMD3<Float>] {
        let ptr = data.bytes.assumingMemoryBound(to: Float.self)
        return (0..<count).map { i in SIMD3<Float>(ptr[i*4], ptr[i*4+1], ptr[i*4+2]) }
    }

    enum PCLError: Error { case nullBuffer }
}

// MARK: - ARSessionDelegate

extension BodyPCLCoordinator: ARSessionDelegate {

    nonisolated func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard let bodyAnchor = frame.anchors.first(where: { $0 is ARBodyAnchor }) as? ARBodyAnchor
        else { return }

        // Extract all tracked joint world-space positions.
        let skeleton    = bodyAnchor.skeleton
        let worldXform  = bodyAnchor.transform
        let definition  = ARSkeletonDefinition.defaultBody3D

        var joints = [SIMD3<Float>]()
        joints.reserveCapacity(definition.jointCount)

        for i in 0..<definition.jointCount {
            guard skeleton.isJointTracked(i) else { continue }
            let localXform = skeleton.jointModelTransforms[i]
            let worldPos   = worldXform * localXform
            joints.append(SIMD3<Float>(worldPos.columns.3.x,
                                       worldPos.columns.3.y,
                                       worldPos.columns.3.z))
        }

        Task { @MainActor [weak self] in
            guard let self else { return }
            statusMessage = "Body tracked — \(joints.count) joints"
            analyze(joints: joints)
        }
    }

    nonisolated func session(_ session: ARSession,
                             cameraDidChangeTrackingState camera: ARCamera) {
        let msg: String
        switch camera.trackingState {
        case .limited(.initializing): msg = "Initialising…"
        case .normal:                  msg = "Point camera at a person (full body visible)"
        default:                       msg = "Adjusting…"
        }
        Task { @MainActor [weak self] in self?.statusMessage = msg }
    }

    nonisolated func session(_ session: ARSession, didFailWithError error: Error) {
        Task { @MainActor [weak self] in
            self?.statusMessage = "Session error: \(error.localizedDescription)"
        }
    }
}

private extension NSLock {
    @discardableResult
    func withLock<T>(_ f: () -> T) -> T { lock(); defer { unlock() }; return f() }
}
