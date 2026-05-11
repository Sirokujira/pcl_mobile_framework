import SwiftUI
import MetalKit

struct PlaneMetalViewContainer: UIViewRepresentable {
    @ObservedObject var coordinator: PlaneLayerCoordinator
    func makeCoordinator() -> Coordinator { Coordinator() }
    func makeUIView(context: Context) -> MTKView {
        let renderer = PlaneLayerRenderer(coordinator: coordinator)
        context.coordinator.renderer = renderer
        let view = MTKView(frame: .zero, device: coordinator.device)
        view.delegate                = renderer
        view.colorPixelFormat        = .bgra8Unorm
        view.depthStencilPixelFormat = .depth32Float
        view.clearColor              = MTLClearColor(red: 0.03, green: 0.03, blue: 0.06, alpha: 1)
        view.preferredFramesPerSecond = 60
        let pan   = UIPanGestureRecognizer(target: renderer, action: #selector(PlaneLayerRenderer.handlePan(_:)))
        let pinch = UIPinchGestureRecognizer(target: renderer, action: #selector(PlaneLayerRenderer.handlePinch(_:)))
        pan.maximumNumberOfTouches = 2
        view.addGestureRecognizer(pan); view.addGestureRecognizer(pinch)
        return view
    }
    func updateUIView(_ uiView: MTKView, context: Context) {}
    class Coordinator { var renderer: PlaneLayerRenderer? }
}
