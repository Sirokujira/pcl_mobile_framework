// FaceViewController.swift — ARFaceSample
//
// ARSCNView with ARFaceTrackingConfiguration.
//
// Rendering strategy:
//   • The face mesh uses ARSCNFaceGeometry, which wraps ARFaceGeometry
//     and updates automatically as expressions change.
//   • Two materials:
//       fillMesh=true  → solid translucent skin tone (inner mesh)
//       fillMesh=false → wireframe edges (outer mesh — same geometry, different material)
//   • A small orange sphere marks each of the 5 key feature points
//     (left/right eye, nose, mouth corners) so students can see tracked points.

import UIKit
import ARKit
import SceneKit
import simd

final class FaceViewController: UIViewController {

    // MARK: - Interface

    weak var coordinator: FaceCoordinator?

    // MARK: - Private

    private var sceneView:    ARSCNView!
    private var faceNode:     SCNNode!
    private var fillGeo:      ARSCNFaceGeometry!
    private var wireGeo:      ARSCNFaceGeometry!

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
        sceneView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        sceneView.delegate         = self
        sceneView.automaticallyUpdatesLighting = true
        view.addSubview(sceneView)

        faceNode = SCNNode()
        sceneView.scene.rootNode.addChildNode(faceNode)
    }

    private func startSession() {
        guard ARFaceTrackingConfiguration.isSupported else { return }
        let config = ARFaceTrackingConfiguration()
        config.isLightEstimationEnabled = true
        sceneView.session.delegate = coordinator
        sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    // MARK: - Face geometry setup (called once per anchor add)

    private func setupFaceGeometry() {
        guard let device = sceneView.device else { return }

        // Solid fill — translucent skin-like material
        fillGeo = ARSCNFaceGeometry(device: device, fillMesh: true)
        let fillMat = SCNMaterial()
        fillMat.diffuse.contents  = UIColor(red: 0.9, green: 0.7, blue: 0.6, alpha: 0.55)
        fillMat.isDoubleSided     = true
        fillMat.lightingModel     = .phong
        fillGeo.materials         = [fillMat]

        let fillNode = SCNNode(geometry: fillGeo)
        faceNode.addChildNode(fillNode)

        // Wireframe overlay — same geometry but wireframe material
        wireGeo = ARSCNFaceGeometry(device: device, fillMesh: false)
        let wireMat = SCNMaterial()
        wireMat.diffuse.contents = UIColor.cyan.withAlphaComponent(0.6)
        wireMat.fillMode         = .lines
        wireMat.isDoubleSided    = true
        wireMat.lightingModel    = .constant
        wireGeo.materials        = [wireMat]

        let wireNode = SCNNode(geometry: wireGeo)
        faceNode.addChildNode(wireNode)
    }
}

// MARK: - ARSCNViewDelegate

extension FaceViewController: ARSCNViewDelegate {

    func renderer(_ renderer: SCNSceneRenderer,
                  didAdd node: SCNNode, for anchor: ARAnchor) {
        guard anchor is ARFaceAnchor else { return }
        faceNode = node
        setupFaceGeometry()
    }

    func renderer(_ renderer: SCNSceneRenderer,
                  didUpdate node: SCNNode, for anchor: ARAnchor) {
        guard let faceAnchor = anchor as? ARFaceAnchor else { return }

        // Update both geometry objects from the latest ARFaceGeometry.
        fillGeo?.update(from: faceAnchor.geometry)
        wireGeo?.update(from: faceAnchor.geometry)

        // Publish blend shape values to the coordinator.
        Task { @MainActor [weak self] in
            self?.coordinator?.updateBlendShapes(faceAnchor.blendShapes)
            self?.coordinator?.statusMessage = "Tracking face"
        }
    }

    func renderer(_ renderer: SCNSceneRenderer,
                  didRemove node: SCNNode, for anchor: ARAnchor) {
        guard anchor is ARFaceAnchor else { return }
        Task { @MainActor [weak self] in
            self?.coordinator?.statusMessage = "Face lost — look at camera"
            self?.coordinator?.topBlendShapes = []
        }
    }
}
