import SwiftUI
import MetalKit

struct ScanMetalViewContainer: UIViewRepresentable {
    @ObservedObject var coordinator: ScanExportCoordinator
    func makeCoordinator() -> Coordinator { Coordinator() }
    func makeUIView(context: Context) -> MTKView {
        let renderer = ScanRenderer(coordinator: coordinator)
        context.coordinator.renderer = renderer
        let view = MTKView(frame: .zero, device: coordinator.device)
        view.delegate                 = renderer
        view.colorPixelFormat         = .bgra8Unorm
        view.depthStencilPixelFormat  = .depth32Float
        view.clearColor               = MTLClearColor(red: 0.03, green: 0.03, blue: 0.06, alpha: 1)
        view.preferredFramesPerSecond = 60
        let pan   = UIPanGestureRecognizer(target: renderer, action: #selector(ScanRenderer.handlePan(_:)))
        let pinch = UIPinchGestureRecognizer(target: renderer, action: #selector(ScanRenderer.handlePinch(_:)))
        pan.maximumNumberOfTouches = 2
        view.addGestureRecognizer(pan); view.addGestureRecognizer(pinch)
        return view
    }
    func updateUIView(_ uiView: MTKView, context: Context) {}
    class Coordinator { var renderer: ScanRenderer? }
}
