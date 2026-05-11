// BodyCoordinator.swift — ARBodySample
//
// Manages the body-tracking session and published stats.

import ARKit
import SwiftUI

@MainActor
final class BodyCoordinator: NSObject, ObservableObject {

    @Published var statusMessage    = "Initialising…"
    @Published var bodyCount        = 0
    @Published var trackedJoints    = 0
    @Published var trackingQuality  = "—"
    @Published var isSupported      = ARBodyTrackingConfiguration.isSupported

    weak var viewController: BodyViewController?
}

// MARK: - ARSessionDelegate

extension BodyCoordinator: ARSessionDelegate {

    nonisolated func session(_ session: ARSession,
                             cameraDidChangeTrackingState camera: ARCamera) {
        let msg: String
        switch camera.trackingState {
        case .limited(.initializing): msg = "Initialising body tracking…"
        case .normal:                  msg = "Point camera at a full-body subject"
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
