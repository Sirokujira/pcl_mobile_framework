import SwiftUI
import MetalKit

struct BodyMetalViewContainer: UIViewRepresentable {
    @ObservedObject var coordinator: BodyPCLCoordinator

    func makeCoordinator() -> Coordinator { Coordinator() }

    func makeUIView(context: Context) -> MTKView {
        let renderer = BodyMetalRenderer(coordinator: coordinator)
        context.coordinator.renderer = renderer

        let view = MTKView(frame: .zero, device: MTLCreateSystemDefaultDevice())
        view.delegate                 = renderer
        view.colorPixelFormat         = .bgra8Unorm
        view.depthStencilPixelFormat  = .depth32Float
        view.clearColor               = MTLClearColor(red: 0.03, green: 0.03, blue: 0.08, alpha: 1)
        view.preferredFramesPerSecond = 60

        let pan   = UIPanGestureRecognizer(target: renderer,
                                           action: #selector(BodyMetalRenderer.handlePan(_:)))
        let pinch = UIPinchGestureRecognizer(target: renderer,
                                             action: #selector(BodyMetalRenderer.handlePinch(_:)))
        pan.maximumNumberOfTouches = 2
        view.addGestureRecognizer(pan)
        view.addGestureRecognizer(pinch)
        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {}

    class Coordinator { var renderer: BodyMetalRenderer? }
}
