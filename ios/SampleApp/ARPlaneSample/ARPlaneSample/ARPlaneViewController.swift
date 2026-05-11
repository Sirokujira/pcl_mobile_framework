// ARPlaneViewController.swift — ARPlaneSample
//
// UIViewController hosting an ARSCNView.
//
// Plane visualisation:
//   Uses ARSCNPlaneGeometry to track the exact polygon outline of each
//   detected plane.  Colour encodes ARPlaneAnchor.Classification.
//
// Object placement:
//   UITapGestureRecognizer → ARRaycastQuery(allowing: .existingPlaneGeometry)
//   → SCNBox placed at the hit point, slightly above the plane surface.
//
// ARKit raycasting replaces the older hitTest(_:types:) API (deprecated iOS 14).

import UIKit
import ARKit
import SceneKit
import SwiftUI

final class ARPlaneViewController: UIViewController {

    // MARK: - Interface

    weak var coordinator: PlaneCoordinator?

    // MARK: - Private

    private var sceneView: ARSCNView!
    private var device:    MTLDevice!

    // Maps anchor.identifier → the SCNNode for that plane's geometry.
    private var planeGeometryNodes: [UUID: SCNNode] = [:]
    // Placed object nodes (kept for clearing).
    private var placedObjectNodes: [SCNNode] = []

    // Cycling hue for placed objects (0 → 1, wraps around).
    private var hue: CGFloat = 0

    // MARK: - Lifecycle

    override func viewDidLoad() {
        super.viewDidLoad()
        device    = MTLCreateSystemDefaultDevice()!
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
        sceneView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        sceneView.delegate         = self
        sceneView.automaticallyUpdatesLighting = true
        view.addSubview(sceneView)

        let tap = UITapGestureRecognizer(target: self, action: #selector(handleTap(_:)))
        sceneView.addGestureRecognizer(tap)
    }

    private func startARSession() {
        let config = ARWorldTrackingConfiguration()
        config.planeDetection             = [.horizontal, .vertical]
        config.sceneReconstruction        = ARWorldTrackingConfiguration
            .supportsSceneReconstruction(.mesh) ? .mesh : []
        config.environmentTexturing       = .automatic
        sceneView.session.delegate        = coordinator
        sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    // MARK: - Tap to place

    @objc private func handleTap(_ sender: UITapGestureRecognizer) {
        let pt    = sender.location(in: sceneView)
        guard let query  = sceneView.raycastQuery(from: pt,
                                                   allowing: .existingPlaneGeometry,
                                                   alignment: .any),
              let result = sceneView.session.raycast(query).first
        else { return }

        let size: CGFloat = 0.08
        let box  = SCNBox(width: size, height: size, length: size, chamferRadius: 0.006)
        let mat  = SCNMaterial()
        mat.diffuse.contents = UIColor(hue: hue, saturation: 0.8, brightness: 0.9, alpha: 1)
        mat.lightingModel    = .physicallyBased
        box.materials        = [mat]
        hue = (hue + 0.13).truncatingRemainder(dividingBy: 1)

        let node = SCNNode(geometry: box)
        node.simdTransform  = result.worldTransform
        node.simdPosition.y += Float(size) / 2   // sit on surface
        sceneView.scene.rootNode.addChildNode(node)
        placedObjectNodes.append(node)

        Task { @MainActor [weak self] in
            guard let self else { return }
            coordinator?.objectCount = placedObjectNodes.count
        }
    }

    // MARK: - API

    func clearPlacedObjects() {
        placedObjectNodes.forEach { $0.removeFromParentNode() }
        placedObjectNodes.removeAll()
    }
}

// MARK: - ARSCNViewDelegate

extension ARPlaneViewController: ARSCNViewDelegate {

    func renderer(_ renderer: SCNSceneRenderer,
                  didAdd node: SCNNode, for anchor: ARAnchor) {
        guard let planeAnchor = anchor as? ARPlaneAnchor else { return }

        // Use ARSCNPlaneGeometry for accurate polygon-outline rendering.
        let planeGeo = ARSCNPlaneGeometry(device: device)!
        planeGeo.update(from: planeAnchor.geometry)
        planeGeo.materials = [planeMaterial(for: planeAnchor.classification)]

        let planeNode = SCNNode(geometry: planeGeo)
        node.addChildNode(planeNode)

        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            planeGeometryNodes[anchor.identifier] = planeNode
            coordinator?.planeCount = planeGeometryNodes.count
            coordinator?.statusMessage = "Detected \(planeGeometryNodes.count) plane(s) — tap to place"
        }
    }

    func renderer(_ renderer: SCNSceneRenderer,
                  didUpdate node: SCNNode, for anchor: ARAnchor) {
        guard let planeAnchor = anchor as? ARPlaneAnchor,
              let planeNode   = planeGeometryNodes[anchor.identifier],
              let planeGeo    = planeNode.geometry as? ARSCNPlaneGeometry
        else { return }
        planeGeo.update(from: planeAnchor.geometry)
    }

    func renderer(_ renderer: SCNSceneRenderer,
                  didRemove node: SCNNode, for anchor: ARAnchor) {
        guard anchor is ARPlaneAnchor else { return }
        DispatchQueue.main.async { [weak self] in
            self?.planeGeometryNodes.removeValue(forKey: anchor.identifier)
            self?.coordinator?.planeCount = self?.planeGeometryNodes.count ?? 0
        }
    }

    // MARK: - Plane material

    private func planeMaterial(for classification: ARPlaneAnchor.Classification) -> SCNMaterial {
        let mat = SCNMaterial()
        mat.lightingModel = .constant
        mat.isDoubleSided = true

        // Grid texture overlaid on a classification colour.
        let baseColor: UIColor
        switch classification {
        case .floor:    baseColor = UIColor(red: 0.2, green: 0.5, blue: 1.0, alpha: 0.45)
        case .wall:     baseColor = UIColor(red: 0.2, green: 0.9, blue: 0.4, alpha: 0.45)
        case .ceiling:  baseColor = UIColor(red: 0.7, green: 0.3, blue: 1.0, alpha: 0.45)
        case .table:    baseColor = UIColor(red: 1.0, green: 0.6, blue: 0.1, alpha: 0.45)
        case .seat:     baseColor = UIColor(red: 1.0, green: 0.9, blue: 0.1, alpha: 0.45)
        case .door:     baseColor = UIColor(red: 1.0, green: 0.2, blue: 0.2, alpha: 0.45)
        case .window:   baseColor = UIColor(red: 0.1, green: 0.9, blue: 0.9, alpha: 0.45)
        default:        baseColor = UIColor(white: 0.8, alpha: 0.30)
        }
        mat.diffuse.contents = baseColor
        return mat
    }
}

// MARK: - ARSessionDelegate forwarding

extension PlaneCoordinator: ARSessionDelegate {

    nonisolated func session(_ session: ARSession,
                             cameraDidChangeTrackingState camera: ARCamera) {
        let msg: String
        switch camera.trackingState {
        case .notAvailable:                    msg = "Tracking unavailable"
        case .limited(.initializing):         msg = "Initialising…"
        case .limited(.relocalizing):         msg = "Relocalising…"
        case .limited(.excessiveMotion):      msg = "Slow down"
        case .limited(.insufficientFeatures): msg = "Point at a textured surface"
        case .normal:                          msg = "Move camera to detect surfaces"
        @unknown default:                      msg = "Unknown state"
        }
        Task { @MainActor [weak self] in
            guard let self, planeCount == 0 else { return }
            statusMessage = msg
        }
    }
}
