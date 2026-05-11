// PlaneCoordinator.swift — ARPlaneSample
//
// @MainActor ObservableObject that drives the plane detection session.
// The view controller is separate so UIKit lifecycle is handled correctly.

import ARKit
import SceneKit
import SwiftUI

@MainActor
final class PlaneCoordinator: NSObject, ObservableObject {

    @Published var statusMessage = "Move camera to detect surfaces"
    @Published var planeCount    = 0
    @Published var objectCount   = 0

    weak var viewController: ARPlaneViewController?

    // Classification model for legend display
    enum Classification: String, CaseIterable {
        case floor, wall, ceiling, table, seat, door, window, other

        var label: String { rawValue.capitalized }

        var color: Color {
            switch self {
            case .floor:   return .blue
            case .wall:    return .green
            case .ceiling: return .purple
            case .table:   return .orange
            case .seat:    return .yellow
            case .door:    return .red
            case .window:  return .cyan
            case .other:   return .white
            }
        }

        static func from(_ classification: ARPlaneAnchor.Classification) -> Classification {
            switch classification {
            case .floor:   return .floor
            case .wall:    return .wall
            case .ceiling: return .ceiling
            case .table:   return .table
            case .seat:    return .seat
            case .door:    return .door
            case .window:  return .window
            default:       return .other
            }
        }
    }

    func clearObjects() {
        viewController?.clearPlacedObjects()
        objectCount = 0
    }
}
