// PlaneLayerRenderer.swift — MetalPlaneLayerSample
//
// Renders layered point clouds: each plane layer gets a distinct colour
// passed via LayerUniforms.color.  Before segmentation, the raw accumulated
// cloud is shown in a dim grey to confirm scanning is working.

import Metal
import MetalKit
import simd

private struct LayerUniforms {
    var mvp:       simd_float4x4
    var pointSize: Float
    var color:     SIMD3<Float>
    var _pad:      Float = 0
}

final class PlaneLayerRenderer: NSObject, MTKViewDelegate {

    private let device:       MTLDevice
    private let commandQueue: MTLCommandQueue
    private var pipeline:     MTLRenderPipelineState!
    private var rawPipeline:  MTLRenderPipelineState!
    private var depthState:   MTLDepthStencilState!
    private var uniBuf:       MTLBuffer!

    private unowned let coordinator: PlaneLayerCoordinator

    private var theta: Float = 0.3; private var phi: Float = 0.2
    private var radius: Float = 3.0; private var center = SIMD3<Float>(0, 0, 0)
    private var prevPan: CGPoint = .zero

    init(coordinator: PlaneLayerCoordinator) {
        self.coordinator = coordinator
        device       = coordinator.device
        commandQueue = device.makeCommandQueue()!
        super.init()
        buildPipeline()
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func draw(in view: MTKView) {
        guard let drawable = view.currentDrawable,
              let passDesc = view.currentRenderPassDescriptor,
              let cmd      = commandQueue.makeCommandBuffer(),
              let enc      = cmd.makeRenderCommandEncoder(descriptor: passDesc)
        else { return }

        let mvp = makeMVP(viewportSize: view.drawableSize)

        if let result = coordinator.currentResult() {
            // Draw each plane layer with its assigned colour.
            for layer in result.layers {
                guard !layer.floatData.isEmpty else { continue }
                let buf = layer.floatData.withUnsafeBytes { bytes in
                    device.makeBuffer(bytes: bytes.baseAddress!,
                                      length: bytes.count, options: .storageModeShared)!
                }
                var u = LayerUniforms(mvp: mvp, pointSize: 3.5,
                                      color: layer.color)
                memcpy(uniBuf.contents(), &u, MemoryLayout<LayerUniforms>.size)
                enc.setRenderPipelineState(pipeline)
                enc.setDepthStencilState(depthState)
                enc.setVertexBuffer(uniBuf, offset: 0, index: 0)
                enc.setVertexBuffer(buf,    offset: 0, index: 1)
                enc.drawPrimitives(type: .point, vertexStart: 0,
                                   vertexCount: layer.floatData.count / 3)
            }
        } else if let (rawBuf, rawCount) = coordinator.currentRawCloud() {
            // Pre-segmentation: show raw cloud in dim white.
            var u = LayerUniforms(mvp: mvp, pointSize: 2.5,
                                  color: SIMD3<Float>(0.6, 0.6, 0.6))
            memcpy(uniBuf.contents(), &u, MemoryLayout<LayerUniforms>.size)
            enc.setRenderPipelineState(pipeline)
            enc.setDepthStencilState(depthState)
            enc.setVertexBuffer(uniBuf, offset: 0, index: 0)
            enc.setVertexBuffer(rawBuf, offset: 0, index: 1)
            enc.drawPrimitives(type: .point, vertexStart: 0, vertexCount: rawCount)
        }

        enc.endEncoding(); cmd.present(drawable); cmd.commit()
    }

    @objc func handlePan(_ g: UIPanGestureRecognizer) {
        let t = g.translation(in: g.view)
        let dx = Float(t.x - prevPan.x); let dy = Float(t.y - prevPan.y)
        if g.numberOfTouches == 1 {
            theta -= dx*0.005; phi += dy*0.005
            phi = phi.clamped(to: -.pi/2+0.05 ... .pi/2-0.05)
        } else {
            let f = simd_normalize(center - eye())
            let r = simd_normalize(simd_cross(f, SIMD3(0,1,0)))
            let u = simd_normalize(simd_cross(r, f))
            center -= r * dx * radius * 0.0008; center += u * dy * radius * 0.0008
        }
        prevPan = t
        if g.state == .ended || g.state == .cancelled { prevPan = .zero }
    }

    @objc func handlePinch(_ g: UIPinchGestureRecognizer) {
        radius /= Float(g.scale); radius = radius.clamped(to: 0.1...20); g.scale = 1
    }

    private func eye() -> SIMD3<Float> {
        center + SIMD3(radius*cos(phi)*sin(theta), radius*sin(phi), radius*cos(phi)*cos(theta))
    }

    private func makeMVP(viewportSize: CGSize) -> simd_float4x4 {
        let aspect = Float(viewportSize.width / max(1, viewportSize.height))
        let proj = perspFov(fovY: 60 * .pi/180, aspect: aspect, near: 0.01, far: 100)
        let e = eye()
        let f = simd_normalize(center-e); let r = simd_normalize(simd_cross(f, SIMD3(0,1,0))); let u = simd_cross(r,f)
        let view = simd_float4x4(columns: (
            SIMD4(r.x,u.x,-f.x,0), SIMD4(r.y,u.y,-f.y,0), SIMD4(r.z,u.z,-f.z,0),
            SIMD4(-simd_dot(r,e),-simd_dot(u,e),simd_dot(f,e),1)))
        return proj * view
    }

    private func perspFov(fovY: Float, aspect: Float, near: Float, far: Float) -> simd_float4x4 {
        let y = 1/tan(fovY*0.5); let x = y/aspect; let z = far/(near-far)
        return simd_float4x4(columns: (SIMD4(x,0,0,0),SIMD4(0,y,0,0),SIMD4(0,0,z,-1),SIMD4(0,0,z*near,0)))
    }

    private func buildPipeline() {
        guard let lib = device.makeDefaultLibrary() else { fatalError("Shaders.metal missing") }
        let d = MTLRenderPipelineDescriptor()
        d.vertexFunction                  = lib.makeFunction(name: "layerPointVertex")
        d.fragmentFunction                = lib.makeFunction(name: "layerCircleFragment")
        d.colorAttachments[0].pixelFormat = .bgra8Unorm
        d.depthAttachmentPixelFormat      = .depth32Float
        pipeline = try! device.makeRenderPipelineState(descriptor: d)
        rawPipeline = pipeline
        let ds = MTLDepthStencilDescriptor()
        ds.depthCompareFunction = .less; ds.isDepthWriteEnabled = true
        depthState = device.makeDepthStencilState(descriptor: ds)!
        uniBuf = device.makeBuffer(length: MemoryLayout<LayerUniforms>.size, options: .storageModeShared)
    }
}

private extension Float {
    func clamped(to r: ClosedRange<Float>) -> Float { min(max(self, r.lowerBound), r.upperBound) }
}
