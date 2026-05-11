// BodyMetalRenderer.swift — MetalBodyPCLSample
//
// Three-pass Metal renderer for body joint analysis:
//
//   Pass 1 — Joint point cloud
//     Joint positions colored by height (y coordinate → jet colormap).
//     Large point-sprite circles so each joint is clearly visible.
//
//   Pass 2 — Bounding box wireframe  (12 edges, .line primitive)
//     Shows the body's axis-aligned bounding box from PCLMobile boundsAndCentroid.
//     Teal color, slightly transparent.
//
//   Pass 3 — Normal arrows  (.line primitive, 2 vertices per joint)
//     Each arrow starts at the joint position and extends 0.12 m in the
//     direction of the PCL-estimated surface normal.
//     Purple arrows; arrow length is proportional to normal confidence.
//
//   Pass 4 — Centroid  (single large white point)
//     The body center of mass from PCLMobile boundsAndCentroid.

import ARKit
import Metal
import MetalKit
import simd

// Uniform layouts (must match Shaders.metal).
private struct PointUniforms {
    var mvp:       simd_float4x4
    var pointSize: Float
    var yMin:      Float   // for jet colormap normalisation
    var yMax:      Float
    var _pad:      Float = 0
}
private struct WireUniforms {
    var mvp:   simd_float4x4
    var color: SIMD4<Float>
}

final class BodyMetalRenderer: NSObject, MTKViewDelegate {

    // MARK: - Metal objects
    private let device:        MTLDevice
    private let commandQueue:  MTLCommandQueue
    private var pointPipeline: MTLRenderPipelineState!
    private var wirePipeline:  MTLRenderPipelineState!
    private var depthState:    MTLDepthStencilState!
    private var pointUniBuf:   MTLBuffer!
    private var wireUniBuf:    MTLBuffer!

    // MARK: - Data source
    private unowned let coordinator: BodyPCLCoordinator

    // MARK: - Orbit camera
    private var theta:  Float = 0.3
    private var phi:    Float = 0.15
    private var radius: Float = 3.0
    private var center  = SIMD3<Float>(0, 0, 0)
    private var prevPan: CGPoint = .zero

    init(coordinator: BodyPCLCoordinator) {
        self.coordinator = coordinator
        device       = MTLCreateSystemDefaultDevice()!
        commandQueue = device.makeCommandQueue()!
        super.init()
        buildPipelines()
    }

    // MARK: - MTKViewDelegate

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func draw(in view: MTKView) {
        // Read rendering settings under lock-free assumption (Bools/Bool reads are atomic on arm64).
        let showBBox     = coordinator.showBBox
        let showNormals  = coordinator.showNormals
        let showCentroid = coordinator.showCentroid
        guard let result   = coordinator.currentResult(),
              let drawable = view.currentDrawable,
              let passDesc = view.currentRenderPassDescriptor,
              let cmd      = commandQueue.makeCommandBuffer(),
              let enc      = cmd.makeRenderCommandEncoder(descriptor: passDesc)
        else { return }

        let vp   = view.drawableSize
        let mvp  = makeMVP(viewportSize: vp)
        let yMin = result.minBounds.y
        let yMax = result.maxBounds.y

        // ── Pass 1: joint points ───────────────────────────────────────
        let ptBuf = makeBuffer(result.joints)
        var pu = PointUniforms(mvp: mvp, pointSize: 18, yMin: yMin, yMax: yMax)
        memcpy(pointUniBuf.contents(), &pu, MemoryLayout<PointUniforms>.size)

        enc.setRenderPipelineState(pointPipeline)
        enc.setDepthStencilState(depthState)
        enc.setVertexBuffer(pointUniBuf, offset: 0, index: 0)
        enc.setVertexBuffer(ptBuf,       offset: 0, index: 1)
        enc.drawPrimitives(type: .point, vertexStart: 0, vertexCount: result.joints.count)

        // ── Pass 2: bounding box wireframe ─────────────────────────────
        if showBBox {
            let bboxVerts = makeBBoxEdges(result.minBounds, result.maxBounds)
            let bboxBuf   = makeBuffer(bboxVerts)
            var wu = WireUniforms(mvp: mvp, color: SIMD4<Float>(0.1, 0.9, 0.9, 0.8))
            memcpy(wireUniBuf.contents(), &wu, MemoryLayout<WireUniforms>.size)
            enc.setRenderPipelineState(wirePipeline)
            enc.setVertexBuffer(wireUniBuf, offset: 0, index: 0)
            enc.setVertexBuffer(bboxBuf,    offset: 0, index: 1)
            enc.drawPrimitives(type: .line, vertexStart: 0, vertexCount: bboxVerts.count)
        }

        // ── Pass 3: normal arrows ──────────────────────────────────────
        if showNormals {
            let arrowLen: Float = 0.12
            let arrowVerts: [SIMD3<Float>] = zip(result.joints, result.normals).flatMap { (p, n) in
                [p, p + simd_normalize(n) * arrowLen]
            }
            if !arrowVerts.isEmpty {
                let arrowBuf = makeBuffer(arrowVerts)
                var wu = WireUniforms(mvp: mvp, color: SIMD4<Float>(0.8, 0.2, 1.0, 0.9))
                memcpy(wireUniBuf.contents(), &wu, MemoryLayout<WireUniforms>.size)
                enc.setRenderPipelineState(wirePipeline)
                enc.setVertexBuffer(wireUniBuf, offset: 0, index: 0)
                enc.setVertexBuffer(arrowBuf,   offset: 0, index: 1)
                enc.drawPrimitives(type: .line, vertexStart: 0, vertexCount: arrowVerts.count)
            }
        }

        // ── Pass 4: centroid ───────────────────────────────────────────
        if showCentroid {
            let centBuf = makeBuffer([result.centroid])
            var pu2 = PointUniforms(mvp: mvp, pointSize: 30, yMin: result.centroid.y - 0.01,
                                    yMax: result.centroid.y + 0.01)
            // White centroid: override jet range so it maps to the middle (green)
            pu2.yMin = result.centroid.y - 0.5
            pu2.yMax = result.centroid.y + 0.5
            memcpy(pointUniBuf.contents(), &pu2, MemoryLayout<PointUniforms>.size)
            enc.setRenderPipelineState(pointPipeline)
            enc.setVertexBuffer(pointUniBuf, offset: 0, index: 0)
            enc.setVertexBuffer(centBuf,     offset: 0, index: 1)
            enc.drawPrimitives(type: .point, vertexStart: 0, vertexCount: 1)
        }

        enc.endEncoding()
        cmd.present(drawable)
        cmd.commit()
    }

    // MARK: - Gestures

    @objc func handlePan(_ g: UIPanGestureRecognizer) {
        let t = g.translation(in: g.view)
        let dx = Float(t.x - prevPan.x)
        let dy = Float(t.y - prevPan.y)
        if g.numberOfTouches == 1 {
            theta -= dx * 0.005
            phi   += dy * 0.005
            phi    = phi.clamped(to: -.pi/2 + 0.05 ... .pi/2 - 0.05)
        } else {
            let r = cameraRight(); let u = cameraUp()
            center -= r * dx * radius * 0.0008
            center += u * dy * radius * 0.0008
        }
        prevPan = t
        if g.state == .ended || g.state == .cancelled { prevPan = .zero }
    }

    @objc func handlePinch(_ g: UIPinchGestureRecognizer) {
        radius /= Float(g.scale)
        radius  = radius.clamped(to: 0.1 ... 20)
        g.scale = 1
    }

    // MARK: - Geometry helpers

    /// 24 vertices (12 edges × 2) describing the AABB wireframe.
    private func makeBBoxEdges(_ mn: SIMD3<Float>, _ mx: SIMD3<Float>) -> [SIMD3<Float>] {
        let c: [SIMD3<Float>] = [
            SIMD3(mn.x, mn.y, mn.z), SIMD3(mx.x, mn.y, mn.z),
            SIMD3(mn.x, mx.y, mn.z), SIMD3(mx.x, mx.y, mn.z),
            SIMD3(mn.x, mn.y, mx.z), SIMD3(mx.x, mn.y, mx.z),
            SIMD3(mn.x, mx.y, mx.z), SIMD3(mx.x, mx.y, mx.z),
        ]
        let edges = [(0,1),(2,3),(4,5),(6,7),
                     (0,2),(1,3),(4,6),(5,7),
                     (0,4),(1,5),(2,6),(3,7)]
        return edges.flatMap { [c[$0.0], c[$0.1]] }
    }

    /// Upload any [SIMD3<Float>] into a shared MTLBuffer.
    private func makeBuffer(_ v: [SIMD3<Float>]) -> MTLBuffer {
        v.withUnsafeBytes { bytes in
            device.makeBuffer(bytes: bytes.baseAddress!,
                              length: bytes.count,
                              options: .storageModeShared)!
        }
    }

    // MARK: - Camera maths

    private func makeMVP(viewportSize: CGSize) -> simd_float4x4 {
        let aspect = Float(viewportSize.width / max(1, viewportSize.height))
        let proj   = perspFov(fovY: 60 * .pi/180, aspect: aspect, near: 0.01, far: 100)
        let eye    = center + SIMD3<Float>(
            radius * cos(phi) * sin(theta),
            radius * sin(phi),
            radius * cos(phi) * cos(theta))
        return proj * lookAt(eye: eye, center: center, up: SIMD3(0,1,0))
    }

    private func cameraForward() -> SIMD3<Float> {
        let eye = center + SIMD3<Float>(
            radius * cos(phi) * sin(theta), radius * sin(phi), radius * cos(phi) * cos(theta))
        return simd_normalize(center - eye)
    }
    private func cameraRight() -> SIMD3<Float> { simd_normalize(simd_cross(cameraForward(), SIMD3(0,1,0))) }
    private func cameraUp()    -> SIMD3<Float> { simd_normalize(simd_cross(cameraRight(), cameraForward())) }

    private func perspFov(fovY: Float, aspect: Float, near: Float, far: Float) -> simd_float4x4 {
        let y = 1/tan(fovY*0.5); let x = y/aspect; let z = far/(near-far)
        return simd_float4x4(columns: (
            SIMD4(x,0,0,0), SIMD4(0,y,0,0), SIMD4(0,0,z,-1), SIMD4(0,0,z*near,0)))
    }
    private func lookAt(eye: SIMD3<Float>, center: SIMD3<Float>, up: SIMD3<Float>) -> simd_float4x4 {
        let f = simd_normalize(center-eye); let r = simd_normalize(simd_cross(f,up)); let u = simd_cross(r,f)
        return simd_float4x4(columns: (
            SIMD4(r.x,u.x,-f.x,0), SIMD4(r.y,u.y,-f.y,0), SIMD4(r.z,u.z,-f.z,0),
            SIMD4(-simd_dot(r,eye),-simd_dot(u,eye),simd_dot(f,eye),1)))
    }

    // MARK: - Pipeline setup

    private func buildPipelines() {
        guard let lib = device.makeDefaultLibrary() else {
            fatalError("Shaders.metal not in target")
        }

        let ptDesc = MTLRenderPipelineDescriptor()
        ptDesc.vertexFunction                  = lib.makeFunction(name: "bodyPointVertex")
        ptDesc.fragmentFunction                = lib.makeFunction(name: "circleFragment")
        ptDesc.colorAttachments[0].pixelFormat = .bgra8Unorm
        ptDesc.depthAttachmentPixelFormat      = .depth32Float
        pointPipeline = try! device.makeRenderPipelineState(descriptor: ptDesc)

        let wDesc = MTLRenderPipelineDescriptor()
        wDesc.vertexFunction                  = lib.makeFunction(name: "wireVertex")
        wDesc.fragmentFunction                = lib.makeFunction(name: "wireFragment")
        wDesc.colorAttachments[0].pixelFormat = .bgra8Unorm
        wDesc.depthAttachmentPixelFormat      = .depth32Float
        wirePipeline = try! device.makeRenderPipelineState(descriptor: wDesc)

        let ds = MTLDepthStencilDescriptor()
        ds.depthCompareFunction = .less; ds.isDepthWriteEnabled = true
        depthState = device.makeDepthStencilState(descriptor: ds)!

        pointUniBuf = device.makeBuffer(length: MemoryLayout<PointUniforms>.size, options: .storageModeShared)
        wireUniBuf  = device.makeBuffer(length: MemoryLayout<WireUniforms>.size,  options: .storageModeShared)
    }
}

private extension Float {
    func clamped(to r: ClosedRange<Float>) -> Float { min(max(self, r.lowerBound), r.upperBound) }
}
