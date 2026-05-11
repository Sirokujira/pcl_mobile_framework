// MetalViewContainer.swift
//
// UIViewRepresentable that creates an MTKView and wires it to
// PointCloudMetalRenderer.  Gesture recognisers are attached here
// so SwiftUI coordinate transforms don't interfere.

import SwiftUI
import MetalKit

struct MetalViewContainer: UIViewRepresentable {

    @ObservedObject var capturer: DepthCapturer

    func makeCoordinator() -> Coordinator { Coordinator() }

    func makeUIView(context: Context) -> MTKView {
        let renderer = PointCloudMetalRenderer(capturer: capturer)
        context.coordinator.renderer = renderer   // retain

        let view = MTKView(frame: .zero, device: capturer.device)
        view.delegate                 = renderer
        view.colorPixelFormat         = .bgra8Unorm
        view.depthStencilPixelFormat  = .depth32Float
        view.clearColor               = MTLClearColor(red: 0.02, green: 0.02,
                                                       blue: 0.06, alpha: 1)
        view.preferredFramesPerSecond = 60

        let pan   = UIPanGestureRecognizer(
            target: renderer,
            action: #selector(PointCloudMetalRenderer.handlePan(_:)))
        let pinch = UIPinchGestureRecognizer(
            target: renderer,
            action: #selector(PointCloudMetalRenderer.handlePinch(_:)))
        // Allow simultaneous pan + pinch.
        pan.maximumNumberOfTouches = 2
        view.addGestureRecognizer(pan)
        view.addGestureRecognizer(pinch)
        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {}

    class Coordinator {
        var renderer: PointCloudMetalRenderer?
    }
}
