// BodyViewController.swift — ARBodySample
//
// ARSCNView with ARBodyTrackingConfiguration.
//
// Skeleton rendering:
//   • A sphere node (radius 0.025 m) is placed at each tracked joint.
//   • Cylinder nodes connect joints along the standard bone segments.
//   • Both sets are children of the ARBodyAnchor SCNNode so they follow
//     the body in world space automatically.
//
// ARSkeleton3D joint model transforms are in body-local space.
// Assigning them directly to child SCNNode.simdTransform works because
// the anchor node itself carries the body→world transform.
//
// Key joint names (from ARSkeletonDefinition.defaultBody3D.jointNames):
//   "root"                 hips / centre of mass
//   "spine_7_joint"        upper chest
//   "neck_1_joint"         base of neck
//   "head_joint"           head
//   "left_shoulder_1_joint" / "right_shoulder_1_joint"
//   "left_arm_joint"       / "right_arm_joint"     (upper arm)
//   "left_forearm_joint"   / "right_forearm_joint" (elbow)
//   "left_hand_joint"      / "right_hand_joint"
//   "left_upLeg_joint"     / "right_upLeg_joint"   (thigh)
//   "left_leg_joint"       / "right_leg_joint"     (shin)
//   "left_foot_joint"      / "right_foot_joint"

import UIKit
import ARKit
import SceneKit
import simd

final class BodyViewController: UIViewController {

    // MARK: - Interface

    weak var coordinator: BodyCoordinator?

    // MARK: - Skeleton data

    // Joints to render as spheres (subset of 91 total body joints)
    private static let keyJoints: [String] = [
        "root",
        "spine_7_joint",
        "neck_1_joint",
        "head_joint",
        "left_shoulder_1_joint",  "right_shoulder_1_joint",
        "left_arm_joint",          "right_arm_joint",
        "left_forearm_joint",      "right_forearm_joint",
        "left_hand_joint",         "right_hand_joint",
        "left_upLeg_joint",        "right_upLeg_joint",
        "left_leg_joint",          "right_leg_joint",
        "left_foot_joint",         "right_foot_joint",
    ]

    // Bone segments (pairs of joint names to connect with cylinders)
    private static let bones: [(String, String)] = [
        ("root",               "spine_7_joint"),
        ("spine_7_joint",      "neck_1_joint"),
        ("neck_1_joint",       "head_joint"),
        ("neck_1_joint",       "left_shoulder_1_joint"),
        ("neck_1_joint",       "right_shoulder_1_joint"),
        ("left_shoulder_1_joint",  "left_arm_joint"),
        ("left_arm_joint",         "left_forearm_joint"),
        ("left_forearm_joint",     "left_hand_joint"),
        ("right_shoulder_1_joint", "right_arm_joint"),
        ("right_arm_joint",        "right_forearm_joint"),
        ("right_forearm_joint",    "right_hand_joint"),
        ("root",               "left_upLeg_joint"),
        ("left_upLeg_joint",   "left_leg_joint"),
        ("left_leg_joint",     "left_foot_joint"),
        ("root",               "right_upLeg_joint"),
        ("right_upLeg_joint",  "right_leg_joint"),
        ("right_leg_joint",    "right_foot_joint"),
    ]

    // MARK: - SceneKit nodes per body anchor

    // Maps bodyAnchor.identifier → (joint-sphere nodes, bone-cylinder nodes)
    private var skeletonNodes:  [UUID: [String: SCNNode]] = [:]
    private var boneNodes:      [UUID: [SCNNode]]         = [:]

    private var sceneView: ARSCNView!

    // MARK: - Lifecycle

    override func viewDidLoad() {
        super.viewDidLoad()
        setupScene()
    }

    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        startSession()
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        sceneView.session.pause()
    }

    // MARK: - Setup

    private func setupScene() {
        sceneView = ARSCNView(frame: view.bounds)
        sceneView.autoresizingMask            = [.flexibleWidth, .flexibleHeight]
        sceneView.delegate                    = self
        sceneView.automaticallyUpdatesLighting = true
        view.addSubview(sceneView)
    }

    private func startSession() {
        guard ARBodyTrackingConfiguration.isSupported else { return }
        let config = ARBodyTrackingConfiguration()
        sceneView.session.delegate = coordinator
        sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    // MARK: - Skeleton node helpers

    /// Add a full set of joint spheres and bone cylinders as children of `anchorNode`.
    private func buildSkeletonNodes(for bodyAnchor: ARBodyAnchor, in anchorNode: SCNNode) {
        let definition = ARSkeletonDefinition.defaultBody3D
        let jointNames = definition.jointNames

        // Index lookup for fast access during updates.
        var indexMap: [String: Int] = [:]
        for (i, name) in jointNames.enumerated() { indexMap[name] = i }

        // Build joint spheres
        let jointMat = SCNMaterial()
        jointMat.diffuse.contents = UIColor(red: 0.1, green: 0.9, blue: 0.5, alpha: 1)
        jointMat.lightingModel    = .physicallyBased

        var jointNodes: [String: SCNNode] = [:]
        for name in Self.keyJoints {
            guard let idx = indexMap[name] else { continue }
            let transform = bodyAnchor.body.skeleton.jointModelTransforms[idx]
            let sphere     = SCNSphere(radius: 0.025)
            sphere.materials = [jointMat]
            let node = SCNNode(geometry: sphere)
            node.simdTransform = transform
            anchorNode.addChildNode(node)
            jointNodes[name] = node
        }
        skeletonNodes[bodyAnchor.identifier] = jointNodes

        // Build bone cylinders
        let boneMat = SCNMaterial()
        boneMat.diffuse.contents = UIColor(red: 1, green: 0.8, blue: 0.1, alpha: 0.9)
        boneMat.lightingModel    = .physicallyBased

        var boneNodeList: [SCNNode] = []
        for (nameA, nameB) in Self.bones {
            guard
                let nodeA = jointNodes[nameA],
                let nodeB = jointNodes[nameB]
            else { continue }

            let posA = SIMD3<Float>(nodeA.simdTransform.columns.3.x,
                                    nodeA.simdTransform.columns.3.y,
                                    nodeA.simdTransform.columns.3.z)
            let posB = SIMD3<Float>(nodeB.simdTransform.columns.3.x,
                                    nodeB.simdTransform.columns.3.y,
                                    nodeB.simdTransform.columns.3.z)

            let boneNode = makeCylinder(from: posA, to: posB, material: boneMat)
            anchorNode.addChildNode(boneNode)
            boneNodeList.append(boneNode)
        }
        boneNodes[bodyAnchor.identifier] = boneNodeList
    }

    /// Update existing skeleton nodes from the latest body anchor.
    private func updateSkeletonNodes(for bodyAnchor: ARBodyAnchor) {
        let definition = ARSkeletonDefinition.defaultBody3D
        let jointNames = definition.jointNames
        var indexMap: [String: Int] = [:]
        for (i, name) in jointNames.enumerated() { indexMap[name] = i }

        guard let jointNodes = skeletonNodes[bodyAnchor.identifier] else { return }
        let skeleton = bodyAnchor.body.skeleton

        // Move joint spheres
        for (name, node) in jointNodes {
            guard let idx = indexMap[name] else { continue }
            node.simdTransform = skeleton.jointModelTransforms[idx]
        }

        // Rebuild bone cylinders (remove old, add new)
        boneNodes[bodyAnchor.identifier]?.forEach { $0.removeFromParentNode() }
        boneNodes[bodyAnchor.identifier] = []

        let boneMat = SCNMaterial()
        boneMat.diffuse.contents = UIColor(red: 1, green: 0.8, blue: 0.1, alpha: 0.9)
        boneMat.lightingModel    = .physicallyBased

        for (nameA, nameB) in Self.bones {
            guard
                let nodeA = jointNodes[nameA],
                let nodeB = jointNodes[nameB]
            else { continue }

            let posA = SIMD3<Float>(nodeA.simdTransform.columns.3.x,
                                    nodeA.simdTransform.columns.3.y,
                                    nodeA.simdTransform.columns.3.z)
            let posB = SIMD3<Float>(nodeB.simdTransform.columns.3.x,
                                    nodeB.simdTransform.columns.3.y,
                                    nodeB.simdTransform.columns.3.z)

            let boneNode = makeCylinder(from: posA, to: posB, material: boneMat)
            nodeA.parent?.addChildNode(boneNode)
            boneNodes[bodyAnchor.identifier]?.append(boneNode)
        }

        // Count tracked joints
        let tracked = skeleton.definition.jointNames.indices.filter {
            skeleton.isJointTracked($0)
        }.count

        Task { @MainActor [weak self] in
            guard let self else { return }
            coordinator?.trackedJoints = tracked
            coordinator?.statusMessage = "Tracking body — \(tracked) joints"
        }
    }

    // MARK: - Cylinder between two points

    /// Creates a SCNNode containing a cylinder aligned from `a` to `b`.
    private func makeCylinder(from a: SIMD3<Float>, to b: SIMD3<Float>,
                               material: SCNMaterial) -> SCNNode {
        let diff   = b - a
        let length = simd_length(diff)
        guard length > 0.001 else { return SCNNode() }

        let cylinder = SCNCylinder(radius: 0.012, height: CGFloat(length))
        cylinder.materials = [material]

        let node = SCNNode(geometry: cylinder)
        node.simdPosition = (a + b) / 2

        // Rotate cylinder's natural up-axis (+Y) to align with bone direction.
        let up   = SIMD3<Float>(0, 1, 0)
        let dir  = simd_normalize(diff)
        let axis = simd_cross(up, dir)
        let len  = simd_length(axis)
        if len > 1e-6 {
            let angle = acos(simd_dot(up, dir))
            node.simdOrientation = simd_quatf(angle: angle, axis: axis / len)
        }
        return node
    }
}

// MARK: - ARSCNViewDelegate

extension BodyViewController: ARSCNViewDelegate {

    func renderer(_ renderer: SCNSceneRenderer,
                  didAdd node: SCNNode, for anchor: ARAnchor) {
        guard let bodyAnchor = anchor as? ARBodyAnchor else { return }
        buildSkeletonNodes(for: bodyAnchor, in: node)
        Task { @MainActor [weak self] in
            self?.coordinator?.bodyCount = 1
            self?.coordinator?.statusMessage = "Body detected!"
        }
    }

    func renderer(_ renderer: SCNSceneRenderer,
                  didUpdate node: SCNNode, for anchor: ARAnchor) {
        guard let bodyAnchor = anchor as? ARBodyAnchor else { return }
        updateSkeletonNodes(for: bodyAnchor)
    }

    func renderer(_ renderer: SCNSceneRenderer,
                  didRemove node: SCNNode, for anchor: ARAnchor) {
        guard let bodyAnchor = anchor as? ARBodyAnchor else { return }
        skeletonNodes.removeValue(forKey: bodyAnchor.identifier)
        boneNodes.removeValue(forKey: bodyAnchor.identifier)
        Task { @MainActor [weak self] in
            self?.coordinator?.bodyCount    = 0
            self?.coordinator?.trackedJoints = 0
            self?.coordinator?.statusMessage = "Body lost — step back into view"
        }
    }
}
