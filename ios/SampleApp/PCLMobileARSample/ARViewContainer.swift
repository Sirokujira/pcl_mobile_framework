// ARViewContainer.swift
//
// UIViewControllerRepresentable wrapping ARViewController.
//
// Using UIViewControllerRepresentable (rather than UIViewRepresentable)
// lets ARViewController manage viewWillAppear / viewWillDisappear lifecycle
// hooks so the ARSession is paused and resumed automatically.

import SwiftUI

struct ARViewContainer: UIViewControllerRepresentable {

    @ObservedObject var coordinator: ARPointCloudCoordinator

    func makeUIViewController(context: Context) -> ARViewController {
        let vc        = ARViewController()
        vc.coordinator = coordinator
        coordinator.viewController = vc
        return vc
    }

    func updateUIViewController(_ vc: ARViewController, context: Context) {
        // All state changes go through coordinator — no imperative updates needed.
    }
}
