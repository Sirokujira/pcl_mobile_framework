import Metal
import MetalKit
import simd

private struct RawUniforms {
    var mvp:       simd_float4x4
    var pointSize: Float
    var _pad:      SIMD3<Float> = .zero
}

private struct HullUniforms {
    var mvp:       simd_float4x4
    var color:     SIMD4<Float>
    var pointSize: Float
    var _pad:      SIMD3<Float> = .zero
}

final class ConvexHullRenderer: NSObject, MTKViewDelegate {

    private let device:       MTLDevice
    private let commandQueue: MTLCommandQueue
    private var rawPipeline:  MTLRenderPipelineState!
    private var linePipeline: MTLRenderPipelineState!
    private var fillPipeline: MTLRenderPipelineState!
    private var depthState:   MTLDepthStencilState!
    private var rawUniBuf:    MTLBuffer!
    private var hullUniBuf:   MTLBuffer!

    private unowned let coordinator: ConvexHullCoordinator

    private var theta:  Float = 0.3
    private var phi:    Float = 0.2
    private var radius: Float = 4.0
    private var center  = SIMD3<Float>.zero
    private var prevPan: CGPoint = .zero

    init(coordinator: ConvexHullCoordinator) {
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

        let mvp = makeMVP(viewportSize: view.drawableSize)

        if let result = coordinator.currentResult() {
            // Raw cloud (dim background)
            if !result.rawData.isEmpty {
                drawRaw(enc: enc, mvp: mvp, floats: result.rawData)
            }
            // Filled polygon + outline per hull
            for hull in result.hulls {
                drawHullFill(enc: enc, mvp: mvp, hull: hull)
                drawHullLine(enc: enc, mvp: mvp, hull: hull)
            }
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

    private func drawRaw(enc: MTLRenderCommandEncoder, mvp: simd_float4x4, floats: [Float]) {
        let len = floats.count * MemoryLayout<Float>.size
        guard len > 0, let buf = device.makeBuffer(bytes: floats, length: len,
                                                   options: .storageModeShared) else { return }
        var u = RawUniforms(mvp: mvp, pointSize: 2.5)
        memcpy(rawUniBuf.contents(), &u, MemoryLayout<RawUniforms>.size)
        enc.setRenderPipelineState(rawPipeline)
        enc.setDepthStencilState(depthState)
        enc.setVertexBuffer(rawUniBuf, offset: 0, index: 0)
        enc.setVertexBuffer(buf,       offset: 0, index: 1)
        enc.drawPrimitives(type: .point, vertexStart: 0, vertexCount: floats.count / 3)
    }

    private func drawHullFill(enc: MTLRenderCommandEncoder, mvp: simd_float4x4, hull: PlaneHull) {
        guard hull.hullVertices.count >= 3 else { return }
        // Triangle fan: centroid, v0, v1, ..., vN, v0 (close)
        var fanVerts = [hull.centroid]
        fanVerts.append(contentsOf: hull.hullVertices)
        fanVerts.append(hull.hullVertices[0])

        let floats = fanVerts.flatMap { [$0.x, $0.y, $0.z] }
        let len    = floats.count * MemoryLayout<Float>.size
        guard let buf = device.makeBuffer(bytes: floats, length: len,
                                          options: .storageModeShared) else { return }
        var u = HullUniforms(mvp: mvp, color: SIMD4(hull.color, 1.0), pointSize: 1)
        memcpy(hullUniBuf.contents(), &u, MemoryLayout<HullUniforms>.size)
        enc.setRenderPipelineState(fillPipeline)
        enc.setDepthStencilState(depthState)
        enc.setVertexBuffer(hullUniBuf, offset: 0, index: 0)
        enc.setVertexBuffer(buf,        offset: 0, index: 1)
        enc.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: fanVerts.count)
    }

    private func drawHullLine(enc: MTLRenderCommandEncoder, mvp: simd_float4x4, hull: PlaneHull) {
        // Line strip: v0, v1, ..., vN, v0 (close the loop)
        var lineVerts = hull.hullVertices
        lineVerts.append(hull.hullVertices[0])
        let floats = lineVerts.flatMap { [$0.x, $0.y, $0.z] }
        let len    = floats.count * MemoryLayout<Float>.size
        guard let buf = device.makeBuffer(bytes: floats, length: len,
                                          options: .storageModeShared) else { return }
        var u = HullUniforms(mvp: mvp, color: SIMD4(hull.color, 1.0), pointSize: 1)
        memcpy(hullUniBuf.contents(), &u, MemoryLayout<HullUniforms>.size)
        enc.setRenderPipelineState(linePipeline)
        enc.setDepthStencilState(depthState)
        enc.setVertexBuffer(hullUniBuf, offset: 0, index: 0)
        enc.setVertexBuffer(buf,        offset: 0, index: 1)
        enc.drawPrimitives(type: .lineStrip, vertexStart: 0, vertexCount: lineVerts.count)
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

        func makePipeline(vert: String, frag: String, blend: Bool = false) -> MTLRenderPipelineState {
            let d = MTLRenderPipelineDescriptor()
            d.vertexFunction   = lib.makeFunction(name: vert)
            d.fragmentFunction = lib.makeFunction(name: frag)
            d.colorAttachments[0].pixelFormat = .bgra8Unorm
            d.depthAttachmentPixelFormat      = .depth32Float
            if blend {
                d.colorAttachments[0].isBlendingEnabled             = true
                d.colorAttachments[0].sourceRGBBlendFactor          = .sourceAlpha
                d.colorAttachments[0].destinationRGBBlendFactor     = .oneMinusSourceAlpha
                d.colorAttachments[0].sourceAlphaBlendFactor        = .one
                d.colorAttachments[0].destinationAlphaBlendFactor   = .oneMinusSourceAlpha
            }
            return try! device.makeRenderPipelineState(descriptor: d)
        }

        rawPipeline  = makePipeline(vert: "rawPointVertex",  frag: "rawCircleFragment")
        linePipeline = makePipeline(vert: "hullLineVertex",  frag: "hullLineFragment")
        fillPipeline = makePipeline(vert: "hullFillVertex",  frag: "hullFillFragment", blend: true)

        let ds = MTLDepthStencilDescriptor()
        ds.depthCompareFunction = .less; ds.isDepthWriteEnabled = true
        depthState = device.makeDepthStencilState(descriptor: ds)!
        rawUniBuf  = device.makeBuffer(length: MemoryLayout<RawUniforms>.size,  options: .storageModeShared)
        hullUniBuf = device.makeBuffer(length: MemoryLayout<HullUniforms>.size, options: .storageModeShared)
    }
}

private extension Float {
    func clamped(to r: ClosedRange<Float>) -> Float { min(max(self, r.lowerBound), r.upperBound) }
}
