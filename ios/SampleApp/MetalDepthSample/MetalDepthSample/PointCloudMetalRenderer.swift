// PointCloudMetalRenderer.swift
//
// MTKViewDelegate that renders the accumulated point cloud using Metal.
//
// Camera model: spherical orbit (theta/phi/radius) around an orbit center.
//   • 1-finger drag  → azimuth / elevation
//   • 2-finger drag  → pan (shifts orbit center in view plane)
//   • pinch          → dolly (zoom)
//
// The vertex shader receives world-space points as packed_float3 (12-byte
// stride) and a ViewerUniforms buffer containing the combined MVP matrix.

import ARKit
import Metal
import MetalKit
import simd

// Must match ViewerUniforms in Shaders.metal (std140 / Metal alignment).
private struct ViewerUniforms {
    var mvp:       simd_float4x4
    var pointSize: Float
    var _pad:      (Float, Float, Float) = (0, 0, 0)
}

final class PointCloudMetalRenderer: NSObject, MTKViewDelegate {

    // MARK: - Metal objects

    private let device:        MTLDevice
    private let commandQueue:  MTLCommandQueue
    private var pipeline:      MTLRenderPipelineState!
    private var depthState:    MTLDepthStencilState!
    private var uniformBuffer: MTLBuffer!

    // MARK: - AR data source

    private unowned let capturer: DepthCapturer

    // MARK: - Orbit camera

    private var theta:  Float = 0.4        // azimuth (rad)
    private var phi:    Float = 0.2        // elevation (rad)
    private var radius: Float = 2.5        // distance from center (m)
    private var center  = SIMD3<Float>(0, 0, 0)

    // Gesture bookkeeping
    private var prevPan: CGPoint = .zero

    // MARK: - Init

    init(capturer: DepthCapturer) {
        self.capturer     = capturer
        self.device       = capturer.device
        self.commandQueue = device.makeCommandQueue()!
        super.init()
        buildPipeline()
    }

    // MARK: - MTKViewDelegate

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func draw(in view: MTKView) {
        guard
            let drawable   = view.currentDrawable,
            let passDesc   = view.currentRenderPassDescriptor,
            let (ptBuf, n) = capturer.currentCloud()
        else { return }

        refreshUniforms(viewportSize: view.drawableSize)

        guard
            let cmd = commandQueue.makeCommandBuffer(),
            let enc = cmd.makeRenderCommandEncoder(descriptor: passDesc)
        else { return }

        enc.setRenderPipelineState(pipeline)
        enc.setDepthStencilState(depthState)
        enc.setVertexBuffer(uniformBuffer, offset: 0, index: 0)
        enc.setVertexBuffer(ptBuf,         offset: 0, index: 1)
        enc.drawPrimitives(type: .point, vertexStart: 0, vertexCount: n)
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
            // Orbit
            theta -= dx * 0.005
            phi   += dy * 0.005
            phi    = phi.clamped(to: -.pi / 2 + 0.05 ... .pi / 2 - 0.05)
        } else {
            // Pan in view plane
            let r = cameraRight()
            let u = cameraUp()
            let speed = radius * 0.0008
            center -= r * dx * speed
            center += u * dy * speed
        }

        prevPan = t
        if g.state == .ended || g.state == .cancelled { prevPan = .zero }
    }

    @objc func handlePinch(_ g: UIPinchGestureRecognizer) {
        radius /= Float(g.scale)
        radius  = radius.clamped(to: 0.05 ... 30)
        g.scale = 1
    }

    // MARK: - Private: pipeline

    private func buildPipeline() {
        guard let lib = device.makeDefaultLibrary() else {
            fatalError("Metal default library not found — is Shaders.metal in the target?")
        }

        let desc = MTLRenderPipelineDescriptor()
        desc.vertexFunction                        = lib.makeFunction(name: "pointCloudVertex")
        desc.fragmentFunction                      = lib.makeFunction(name: "circleFragment")
        desc.colorAttachments[0].pixelFormat       = .bgra8Unorm
        desc.depthAttachmentPixelFormat            = .depth32Float

        pipeline = try! device.makeRenderPipelineState(descriptor: desc)

        let ds = MTLDepthStencilDescriptor()
        ds.depthCompareFunction = .less
        ds.isDepthWriteEnabled  = true
        depthState = device.makeDepthStencilState(descriptor: ds)!

        uniformBuffer = device.makeBuffer(length: MemoryLayout<ViewerUniforms>.size,
                                          options: .storageModeShared)
    }

    // MARK: - Private: camera maths

    private func refreshUniforms(viewportSize: CGSize) {
        let aspect = Float(viewportSize.width / max(1, viewportSize.height))
        let proj   = perspectiveFov(fovY: 60 * .pi / 180, aspect: aspect,
                                    near: 0.01, far: 100)
        let eye    = orbitEye()
        let view   = lookAt(eye: eye, center: center, up: SIMD3<Float>(0, 1, 0))
        var u      = ViewerUniforms(mvp: proj * view, pointSize: 4)
        memcpy(uniformBuffer.contents(), &u, MemoryLayout<ViewerUniforms>.size)
    }

    private func orbitEye() -> SIMD3<Float> {
        center + SIMD3<Float>(
            radius * cos(phi) * sin(theta),
            radius * sin(phi),
            radius * cos(phi) * cos(theta)
        )
    }

    private func cameraForward() -> SIMD3<Float> {
        simd_normalize(center - orbitEye())
    }

    private func cameraRight() -> SIMD3<Float> {
        simd_normalize(simd_cross(cameraForward(), SIMD3<Float>(0, 1, 0)))
    }

    private func cameraUp() -> SIMD3<Float> {
        simd_normalize(simd_cross(cameraRight(), cameraForward()))
    }

    // Standard perspective matrix (Metal NDC: z ∈ [0,1]).
    private func perspectiveFov(fovY: Float, aspect: Float,
                                near: Float, far: Float) -> simd_float4x4 {
        let y = 1 / tan(fovY * 0.5)
        let x = y / aspect
        let z = far / (near - far)
        return simd_float4x4(columns: (
            SIMD4<Float>(x, 0,  0,  0),
            SIMD4<Float>(0, y,  0,  0),
            SIMD4<Float>(0, 0,  z, -1),
            SIMD4<Float>(0, 0,  z * near, 0)
        ))
    }

    // Column-major look-at view matrix.
    private func lookAt(eye: SIMD3<Float>, center: SIMD3<Float>,
                        up: SIMD3<Float>) -> simd_float4x4 {
        let f = simd_normalize(center - eye)
        let r = simd_normalize(simd_cross(f, up))
        let u = simd_cross(r, f)
        return simd_float4x4(columns: (
            SIMD4<Float>( r.x,  u.x, -f.x, 0),
            SIMD4<Float>( r.y,  u.y, -f.y, 0),
            SIMD4<Float>( r.z,  u.z, -f.z, 0),
            SIMD4<Float>(-simd_dot(r, eye), -simd_dot(u, eye), simd_dot(f, eye), 1)
        ))
    }
}

private extension Float {
    func clamped(to r: ClosedRange<Float>) -> Float { min(max(self, r.lowerBound), r.upperBound) }
}
