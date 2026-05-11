import SwiftUI
import MetalKit

struct ObjectMetalViewContainer: UIViewRepresentable {
    @ObservedObject var coordinator: ObjectMeasureCoordinator
    func makeCoordinator() -> Coordinator { Coordinator() }
    func makeUIView(context: Context) -> MTKView {
        let renderer = ObjectMeasureRenderer(coordinator: coordinator)
        context.coordinator.renderer = renderer
        let view = MTKView(frame: .zero, device: coordinator.device)
        view.delegate                 = renderer
        view.colorPixelFormat         = .bgra8Unorm
        view.depthStencilPixelFormat  = .depth32Float
        view.clearColor               = MTLClearColor(red: 0.04, green: 0.04, blue: 0.07, alpha: 1)
        view.preferredFramesPerSecond = 60
        let pan   = UIPanGestureRecognizer(target: renderer,
                                           action: #selector(ObjectMeasureRenderer.handlePan(_:)))
        let pinch = UIPinchGestureRecognizer(target: renderer,
                                             action: #selector(ObjectMeasureRenderer.handlePinch(_:)))
        pan.maximumNumberOfTouches = 2
        view.addGestureRecognizer(pan); view.addGestureRecognizer(pinch)
        return view
    }
    func updateUIView(_ uiView: MTKView, context: Context) {}
    class Coordinator { var renderer: ObjectMeasureRenderer? }
}
