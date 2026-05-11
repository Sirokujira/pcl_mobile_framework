import SwiftUI

struct ARFaceViewContainer: UIViewControllerRepresentable {
    @ObservedObject var coordinator: FaceCoordinator

    func makeUIViewController(context: Context) -> FaceViewController {
        let vc = FaceViewController()
        vc.coordinator = coordinator
        coordinator.viewController = vc
        return vc
    }

    func updateUIViewController(_ vc: FaceViewController, context: Context) {}
}
