// ARViewContainer.swift
//
// SwiftUI wrapper around an ARKit ARSCNView. The ARSession's delegate is the
// ARPointCloudCoordinator instance owned by ContentView, which captures
// rawFeaturePoints and feeds them to PCLMobile.

import ARKit
import SwiftUI

struct ARViewContainer: UIViewRepresentable {
    let coordinator: ARPointCloudCoordinator

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView(frame: .zero)
        view.session.delegate = coordinator
        view.automaticallyUpdatesLighting = true
        view.debugOptions = [.showFeaturePoints]

        let configuration = ARWorldTrackingConfiguration()
        configuration.planeDetection = [.horizontal, .vertical]
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            // LiDAR devices give us much denser data via mesh anchors.
            configuration.sceneReconstruction = .mesh
        }
        view.session.run(configuration, options: [.resetTracking, .removeExistingAnchors])

        coordinator.attach(session: view.session)
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {
        // Nothing to refresh — coordinator owns the state.
    }

    static func dismantleUIView(_ uiView: ARSCNView, coordinator: ARPointCloudCoordinator) {
        uiView.session.pause()
    }
}
