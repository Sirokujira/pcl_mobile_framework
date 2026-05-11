// FaceCoordinator.swift — ARFaceSample
//
// Manages the face-tracking AR session and publishes blend-shape data.
// ARFaceTrackingConfiguration requires a device with a TrueDepth camera
// (iPhone X or later, iPad Pro 2020 or later).
//
// Blend shapes are Apple's coefficients for 52 facial action units:
//   jawOpen          — mouth opening
//   eyeBlinkLeft/Right — eye closure
//   mouthSmileLeft/Right — smile intensity
//   browInnerUp      — worried / surprised brow
//   etc.
// Each value is in [0, 1]; 0 = neutral, 1 = maximum deformation.

import ARKit
import SwiftUI

@MainActor
final class FaceCoordinator: NSObject, ObservableObject {

    struct BlendShape: Identifiable {
        let id   = UUID()
        let name: String
        let value: Float
    }

    @Published var statusMessage  = "Initialising…"
    @Published var topBlendShapes: [BlendShape] = []
    @Published var isSupported    = ARFaceTrackingConfiguration.isSupported

    weak var viewController: FaceViewController?

    // Friendly short names for display.
    private static let friendlyNames: [ARFaceAnchor.BlendShapeLocation: String] = [
        .jawOpen:            "Jaw Open",
        .eyeBlinkLeft:       "Eye Blink L",
        .eyeBlinkRight:      "Eye Blink R",
        .mouthSmileLeft:     "Smile L",
        .mouthSmileRight:    "Smile R",
        .browInnerUp:        "Brow Up",
        .browOuterUpLeft:    "Brow Out L",
        .browOuterUpRight:   "Brow Out R",
        .cheekPuff:          "Cheek Puff",
        .mouthFunnel:        "Mouth Funnel",
        .tongueOut:          "Tongue Out",
        .eyeSquintLeft:      "Eye Squint L",
        .eyeSquintRight:     "Eye Squint R",
    ]

    func updateBlendShapes(_ shapes: [ARFaceAnchor.BlendShapeLocation: NSNumber]) {
        let top = shapes
            .compactMap { (loc, val) -> BlendShape? in
                guard let name = Self.friendlyNames[loc] else { return nil }
                return BlendShape(name: name, value: val.floatValue)
            }
            .filter { $0.value > 0.02 }
            .sorted { $0.value > $1.value }
            .prefix(5)
        topBlendShapes = Array(top)
    }
}

// MARK: - ARSessionDelegate

extension FaceCoordinator: ARSessionDelegate {

    nonisolated func session(_ session: ARSession,
                             cameraDidChangeTrackingState camera: ARCamera) {
        let msg: String
        switch camera.trackingState {
        case .limited(.initializing): msg = "Initialising face tracking…"
        case .normal:                  msg = "Face tracking active"
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
