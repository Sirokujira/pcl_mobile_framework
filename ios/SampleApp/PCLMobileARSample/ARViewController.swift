// ARViewController.swift
//
// UIViewController hosting an ARSCNView.  Owns three SceneKit nodes for
// layered point-cloud visualisation:
//
//   rawNode      — all accumulated feature points (white, shown live)
//   filteredNode — PCL-filtered non-plane points   (cyan,   post-process)
//   planeNode    — RANSAC plane inliers             (orange, post-process)
//   normalNode   — plane-normal arrow               (yellow, post-process)
//
// The plane-normal arrow is constructed with simd quaternion math —
// Apple's built-in SIMD library fills the same role as glm on other
// platforms (rotations, cross product, normalization, etc.).
//
// ARPointCloudCoordinator owns the AR session delegate and business logic;
// this VC just owns rendering.

import UIKit
import ARKit
import SceneKit
import simd

final class ARViewController: UIViewController {

    // MARK: - Interface

    weak var coordinator: ARPointCloudCoordinator?

    // MARK: - Private

    private var sceneView: ARSCNView!

    // Four persistent SCNNodes — geometry is swapped in/out.
    private var rawNode:      SCNNode!
    private var filteredNode: SCNNode!
    private var planeNode:    SCNNode!
    private var normalNode:   SCNNode!

    // MARK: - Lifecycle

    override func viewDidLoad() {
        super.viewDidLoad()
        setupSceneView()
    }

    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        startARSession()
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        sceneView.session.pause()
    }

    // MARK: - Setup

    private func setupSceneView() {
        sceneView = ARSCNView(frame: view.bounds)
        sceneView.autoresizingMask     = [.flexibleWidth, .flexibleHeight]
        sceneView.delegate             = self
        sceneView.automaticallyUpdatesLighting = true
        // ARKit's own yellow feature-point overlay for reference.
        sceneView.debugOptions         = [.showFeaturePoints]
        view.addSubview(sceneView)

        let scene = SCNScene()
        sceneView.scene = scene

        rawNode      = addNode(to: scene)
        filteredNode = addNode(to: scene)
        planeNode    = addNode(to: scene)
        normalNode   = addNode(to: scene)
    }

    private func startARSession() {
        guard let coordinator else { return }
        guard ARWorldTrackingConfiguration.isSupported else {
            coordinator.statusMessage = "ARWorldTracking not supported"
            return
        }
        let config = ARWorldTrackingConfiguration()
        config.planeDetection = [.horizontal, .vertical]
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh   // LiDAR devices only
        }
        sceneView.session.delegate = coordinator
        sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
        coordinator.attach(session: sceneView.session)
    }

    // MARK: - Rendering API (called from ARPointCloudCoordinator on main actor)

    /// Replace the raw-point layer with the current accumulated cloud.
    func showRawCloud(_ points: [simd_float3]) {
        rawNode.geometry = points.isEmpty ? nil :
            PointCloudNode.geometry(points: points,
                                    color: UIColor(white: 0.9, alpha: 0.45),
                                    pointSize: 3)
    }

    /// Update all processed layers from a pipeline result.
    func showProcessedResult(_ result: PCLPipelineResult) {
        // Raw layer hidden while processed layers are visible.
        rawNode.geometry = nil

        filteredNode.geometry = result.filteredPoints.isEmpty ? nil :
            PointCloudNode.geometry(
                points:    result.filteredPoints,
                color:     UIColor(red: 0, green: 0.85, blue: 1, alpha: 1),
                pointSize: 5)

        planeNode.geometry = result.planePoints.isEmpty ? nil :
            PointCloudNode.geometry(
                points:    result.planePoints,
                color:     UIColor(red: 1, green: 0.55, blue: 0, alpha: 1),
                pointSize: 6)

        // Plane normal arrow (uses simd quaternion math).
        clearNormalArrow()
        if let model = result.planeModel, let centroid = result.centroid {
            addNormalArrow(at: centroid,
                           normal: simd_float3(model.a, model.b, model.c))
        }
    }

    /// Remove all rendered geometry.
    func clearAll() {
        rawNode.geometry      = nil
        filteredNode.geometry = nil
        planeNode.geometry    = nil
        clearNormalArrow()
    }

    // MARK: - Plane normal visualisation

    /// Draw a yellow arrow from `origin` in the direction of `normal`.
    ///
    /// Uses simd to:
    ///   1. Normalise the normal vector.
    ///   2. Compute the rotation axis via cross product with SCNCylinder's
    ///      natural up-axis (Y).
    ///   3. Build a quaternion for the rotation.
    ///   4. Set `simdOrientation` / `simdPosition` on the SCNNodes.
    private func addNormalArrow(at origin: simd_float3, normal: simd_float3) {
        let arrowLength: Float = 0.30
        let n   = simd_normalize(normal)
        let tip = origin + n * arrowLength

        // ── Cylinder (shaft) ──────────────────────────────────────────────
        let shaftLength = arrowLength - 0.04
        let shaft = SCNCylinder(radius: 0.004, height: CGFloat(shaftLength))
        shaft.firstMaterial?.diffuse.contents = UIColor.yellow
        shaft.firstMaterial?.lightingModel    = .constant

        let shaftNode = SCNNode(geometry: shaft)
        // SCNCylinder is aligned along +Y; rotate to align with n.
        shaftNode.simdOrientation = rotationFrom(yAxis: simd_float3(0, 1, 0), to: n)
        // Position at mid-point of the shaft.
        shaftNode.simdPosition = origin + n * (shaftLength * 0.5)

        // ── Cone (arrowhead) ──────────────────────────────────────────────
        let cone = SCNCone(topRadius: 0, bottomRadius: 0.012, height: 0.04)
        cone.firstMaterial?.diffuse.contents = UIColor.yellow
        cone.firstMaterial?.lightingModel    = .constant

        // SCNCone is also Y-aligned with base at -Y; rotate the same way.
        let coneNode = SCNNode(geometry: cone)
        coneNode.simdOrientation = shaftNode.simdOrientation
        // Position so the base sits at tip and the point faces forward.
        coneNode.simdPosition = tip + n * 0.02

        normalNode.addChildNode(shaftNode)
        normalNode.addChildNode(coneNode)
    }

    private func clearNormalArrow() {
        normalNode.childNodes.forEach { $0.removeFromParentNode() }
    }

    /// Return the simd_quatf that rotates `from` onto `to`.
    /// Both vectors are assumed to be unit-length.
    private func rotationFrom(yAxis: simd_float3, to target: simd_float3) -> simd_quatf {
        let axis  = simd_cross(yAxis, target)
        let axLen = simd_length(axis)
        guard axLen > 1e-6 else {
            // Parallel or anti-parallel — no rotation or 180° flip.
            if simd_dot(yAxis, target) > 0 { return simd_quatf(ix: 0, iy: 0, iz: 0, r: 1) }
            return simd_quatf(angle: .pi, axis: simd_float3(1, 0, 0))
        }
        let angle = acos(simd_clamp(simd_dot(yAxis, target), -1, 1))
        return simd_quatf(angle: angle, axis: axis / axLen)
    }

    // MARK: - Helpers

    private func addNode(to scene: SCNScene) -> SCNNode {
        let node = SCNNode()
        scene.rootNode.addChildNode(node)
        return node
    }
}

// MARK: - ARSCNViewDelegate

extension ARViewController: ARSCNViewDelegate {}
