import Metal
import MetalKit
import simd

private struct ObjectUniforms {
    var mvp:       simd_float4x4
    var pointSize: Float
    var color:     SIMD3<Float>
    var _pad:      Float = 0
}

final class ObjectMeasureRenderer: NSObject, MTKViewDelegate {

    private let device:        MTLDevice
    private let commandQueue:  MTLCommandQueue
    private var pointPipeline: MTLRenderPipelineState!
    private var wirePipeline:  MTLRenderPipelineState!
    private var depthState:    MTLDepthStencilState!
    private var uniBuf:        MTLBuffer!

    private unowned let coordinator: ObjectMeasureCoordinator

    private var theta:  Float = 0.3
    private var phi:    Float = 0.15
    private var radius: Float = 4.0
    private var center  = SIMD3<Float>.zero
    private var prevPan: CGPoint = .zero

    init(coordinator: ObjectMeasureCoordinator) {
        self.coordinator = coordinator
        device           = coordinator.device
        commandQueue     = device.makeCommandQueue()!
        super.init()
        buildPipelines()
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func draw(in view: MTKView) {
        guard let drawable = view.currentDrawable,
              let passDesc = view.currentRenderPassDescriptor,
              let cmd      = commandQueue.makeCommandBuffer(),
              let enc      = cmd.makeRenderCommandEncoder(descriptor: passDesc)
        else { return }

        guard let result = coordinator.currentResult() else {
            enc.endEncoding(); cmd.present(drawable); cmd.commit(); return
        }

        let mvp = makeMVP(viewportSize: view.drawableSize)

        // Floor (grey points)
        if !result.floorData.isEmpty {
            drawPoints(enc: enc, mvp: mvp,
                       floats: result.floorData,
                       color: SIMD3(0.45, 0.45, 0.45), size: 2.0)
        }

        // Clusters: points + AABB wireframe
        for cluster in result.clusters {
            drawPoints(enc: enc, mvp: mvp,
                       floats: cluster.floatData,
                       color: cluster.color, size: 3.5)
            drawAABB(enc: enc, mvp: mvp,
                     min: cluster.min, max: cluster.max,
                     color: cluster.color * 1.3)
        }

        enc.endEncoding(); cmd.present(drawable); cmd.commit()
    }

    // MARK: - Gesture

    @objc func handlePan(_ g: UIPanGestureRecognizer) {
        let t  = g.translation(in: g.view)
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
        radius /= Float(g.scale); radius = radius.clamped(to: 0.1...30); g.scale = 1
    }

    // MARK: - Draw helpers

    private func drawPoints(enc: MTLRenderCommandEncoder,
                            mvp: simd_float4x4,
                            floats: [Float],
                            color: SIMD3<Float>,
                            size: Float) {
        let len = floats.count * MemoryLayout<Float>.size
        guard len > 0, let buf = device.makeBuffer(bytes: floats, length: len,
                                                   options: .storageModeShared) else { return }
        var u = ObjectUniforms(mvp: mvp, pointSize: size, color: color)
        memcpy(uniBuf.contents(), &u, MemoryLayout<ObjectUniforms>.size)
        enc.setRenderPipelineState(pointPipeline)
        enc.setDepthStencilState(depthState)
        enc.setVertexBuffer(uniBuf, offset: 0, index: 0)
        enc.setVertexBuffer(buf,    offset: 0, index: 1)
        enc.drawPrimitives(type: .point, vertexStart: 0, vertexCount: floats.count / 3)
    }

    private func drawAABB(enc: MTLRenderCommandEncoder,
                          mvp: simd_float4x4,
                          min mn: SIMD3<Float>,
                          max mx: SIMD3<Float>,
                          color: SIMD3<Float>) {
        let edges = makeAABBEdges(mn, mx)
        let len   = edges.count * MemoryLayout<SIMD3<Float>>.size
        guard len > 0, let buf = device.makeBuffer(bytes: edges, length: len,
                                                   options: .storageModeShared) else { return }
        var u = ObjectUniforms(mvp: mvp, pointSize: 1, color: simd_clamp(color, .zero, SIMD3(1,1,1)))
        memcpy(uniBuf.contents(), &u, MemoryLayout<ObjectUniforms>.size)
        enc.setRenderPipelineState(wirePipeline)
        enc.setDepthStencilState(depthState)
        enc.setVertexBuffer(uniBuf, offset: 0, index: 0)
        enc.setVertexBuffer(buf,    offset: 0, index: 1)
        enc.drawPrimitives(type: .line, vertexStart: 0, vertexCount: edges.count)
    }

    private func makeAABBEdges(_ mn: SIMD3<Float>, _ mx: SIMD3<Float>) -> [SIMD3<Float>] {
        let c = [
            SIMD3(mn.x, mn.y, mn.z), SIMD3(mx.x, mn.y, mn.z),
            SIMD3(mn.x, mx.y, mn.z), SIMD3(mx.x, mx.y, mn.z),
            SIMD3(mn.x, mn.y, mx.z), SIMD3(mx.x, mn.y, mx.z),
            SIMD3(mn.x, mx.y, mx.z), SIMD3(mx.x, mx.y, mx.z),
        ]
        let edges = [(0,1),(2,3),(4,5),(6,7),(0,2),(1,3),(4,6),(5,7),(0,4),(1,5),(2,6),(3,7)]
        return edges.flatMap { [c[$0.0], c[$0.1]] }
    }

    // MARK: - Camera math

    private func eye() -> SIMD3<Float> {
        center + SIMD3(radius*cos(phi)*sin(theta), radius*sin(phi), radius*cos(phi)*cos(theta))
    }

    private func makeMVP(viewportSize: CGSize) -> simd_float4x4 {
        let aspect = Float(viewportSize.width / max(1, viewportSize.height))
        let proj   = perspFov(fovY: 60 * .pi/180, aspect: aspect, near: 0.01, far: 100)
        let e = eye()
        let f = simd_normalize(center-e)
        let r = simd_normalize(simd_cross(f, SIMD3(0,1,0)))
        let u = simd_cross(r, f)
        let view = simd_float4x4(columns: (
            SIMD4(r.x,u.x,-f.x,0), SIMD4(r.y,u.y,-f.y,0), SIMD4(r.z,u.z,-f.z,0),
            SIMD4(-simd_dot(r,e),-simd_dot(u,e),simd_dot(f,e),1)))
        return proj * view
    }

    private func perspFov(fovY: Float, aspect: Float, near: Float, far: Float) -> simd_float4x4 {
        let y = 1/tan(fovY*0.5); let x = y/aspect; let z = far/(near-far)
        return simd_float4x4(columns: (SIMD4(x,0,0,0),SIMD4(0,y,0,0),SIMD4(0,0,z,-1),SIMD4(0,0,z*near,0)))
    }

    // MARK: - Pipeline

    private func buildPipelines() {
        guard let lib = device.makeDefaultLibrary() else { fatalError("Shaders.metal missing") }

        let pd = MTLRenderPipelineDescriptor()
        pd.vertexFunction                  = lib.makeFunction(name: "objectPointVertex")
        pd.fragmentFunction                = lib.makeFunction(name: "objectCircleFragment")
        pd.colorAttachments[0].pixelFormat = .bgra8Unorm
        pd.depthAttachmentPixelFormat      = .depth32Float
        pointPipeline = try! device.makeRenderPipelineState(descriptor: pd)

        let wd = MTLRenderPipelineDescriptor()
        wd.vertexFunction                  = lib.makeFunction(name: "wireVertex")
        wd.fragmentFunction                = lib.makeFunction(name: "wireFragment")
        wd.colorAttachments[0].pixelFormat = .bgra8Unorm
        wd.depthAttachmentPixelFormat      = .depth32Float
        wirePipeline = try! device.makeRenderPipelineState(descriptor: wd)

        let ds = MTLDepthStencilDescriptor()
        ds.depthCompareFunction = .less; ds.isDepthWriteEnabled = true
        depthState = device.makeDepthStencilState(descriptor: ds)!
        uniBuf     = device.makeBuffer(length: MemoryLayout<ObjectUniforms>.size,
                                       options: .storageModeShared)
    }
}

private extension Float {
    func clamped(to r: ClosedRange<Float>) -> Float { min(max(self, r.lowerBound), r.upperBound) }
}
