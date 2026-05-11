import SwiftUI
import UIKit

struct ARPlaneViewContainer: UIViewControllerRepresentable {

    @ObservedObject var coordinator: PlaneCoordinator

    func makeUIViewController(context: Context) -> ARPlaneViewController {
        let vc = ARPlaneViewController()
        vc.coordinator = coordinator
        coordinator.viewController = vc
        return vc
    }

    func updateUIViewController(_ vc: ARPlaneViewController, context: Context) {}
}
