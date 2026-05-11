// ARMetalViewContainer.swift
//
// UIViewRepresentable wrapping an MTKView whose delegate is AROverlayRenderer.
// The MTKView is the single view that renders both the camera feed and the
// AR depth overlay — no UIKit/SceneKit layer sits behind it.

import SwiftUI
import MetalKit

struct ARMetalViewContainer: UIViewRepresentable {

    @ObservedObject var renderer: AROverlayRenderer

    func makeUIView(context: Context) -> MTKView {
        let view = MTKView(frame: .zero, device: renderer.device)
        view.delegate                 = renderer
        view.colorPixelFormat         = .bgra8Unorm
        view.depthStencilPixelFormat  = .depth32Float
        view.clearColor               = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 1)
        view.preferredFramesPerSecond = 60
        // Disable automatic clear — we paint the full frame every pass.
        view.clearDepth               = 1.0
        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {}
}
