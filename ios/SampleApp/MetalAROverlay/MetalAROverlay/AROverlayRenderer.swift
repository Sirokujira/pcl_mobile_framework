// AROverlayRenderer.swift
//
// MTKViewDelegate + ARSessionDelegate that drives a two-pass Metal frame:
//
//   Pass 1 — Camera background
//     Renders the ARFrame.capturedImage (YCbCr biplanar) as a full-screen
//     RGB quad using cameraBackgroundVertex / cameraBackgroundFragment.
//     ARFrame.displayTransform corrects for device orientation and the
//     camera's aspect ratio.
//
//   Pass 2 — Depth / feature-point overlay
//     On LiDAR devices: unprojection is done in the vertex shader.
//       Each vertex ID maps to one pixel in the depth map.
//       The shader samples the depth texture, computes a camera-space
//       position, and projects it with the AR camera's projection matrix.
//     On non-LiDAR devices: world-space feature points are uploaded and
//       rendered with the same projection + view matrix path.
//
// Threading model:
//   • @Published properties are always updated via DispatchQueue.main.async.
//   • draw(in:) runs on the MTKView display-link thread; it snapshots
//     settings and the current ARFrame under a lightweight NSLock.
//   • ARSessionDelegate callbacks arrive on a background thread.
//
// Coordinate conventions (ARKit):
//   Camera space: +X right, +Y up, −Z forward (right-handed).
//   Depth values: positive metres along −Z (z_c = −depth).
//   Image row 0 is at the top → flip Y when unprojecting.
//
// Reference:
//   https://developer.apple.com/documentation/arkit/displaying-a-point-cloud-using-scene-depth

import ARKit
import Metal
import MetalKit
import simd

// ─── Uniform structs (must match Shaders.metal) ───────────────────────────────

private struct CameraUniforms {
    var displayTransform: simd_float3x3
}

private struct OverlayUniforms {
    var projectionMatrix: simd_float4x4
    var fx: Float;  var fy: Float
    var cx: Float;  var cy: Float
    var depthWidth:  Float
    var depthHeight: Float
    var maxDepth: Float
    var pointSize: Float
}

private struct FeatureUniforms {
    var viewProjection: simd_float4x4
    var pointSize: Float
    var _pad: (Float, Float, Float) = (0, 0, 0)
}

// Snapshot passed into draw() — all copied under the settings lock.
private struct RenderSettings {
    var showDepthOverlay: Bool  = true
    var maxDepth: Float         = 5.0
    var pointSize: Float        = 5.0
    var highConfOnly: Bool      = false
}

// ─── Renderer ─────────────────────────────────────────────────────────────────

final class AROverlayRenderer: NSObject, ObservableObject {

    // MARK: - Published (always written on main queue)

    @Published var statusMessage     = "Starting AR…"
    @Published var pointCount        = 0
    @Published var fps: Double       = 0
    @Published var hasLiDAR          = false

    // Writes from SwiftUI (main actor) propagate to _settings via didSet.
    @Published var showDepthOverlay: Bool  = true  { didSet { lockSettings { $0.showDepthOverlay = showDepthOverlay } } }
    @Published var maxDepth: Float         = 5.0   { didSet { lockSettings { $0.maxDepth         = maxDepth        } } }
    @Published var pointSize: Float        = 5.0   { didSet { lockSettings { $0.pointSize         = pointSize       } } }
    @Published var highConfidenceOnly: Bool = false { didSet { lockSettings { $0.highConfOnly      = highConfidenceOnly } } }

    // MARK: - Metal

    let device:       MTLDevice
    let commandQueue: MTLCommandQueue

    private var cameraPipeline:  MTLRenderPipelineState!
    private var overlayPipeline: MTLRenderPipelineState!
    private var featurePipeline: MTLRenderPipelineState!
    private var depthState:      MTLDepthStencilState!
    private var nodepthState:    MTLDepthStencilState!

    private var cameraUniformBuf:  MTLBuffer!
    private var overlayUniformBuf: MTLBuffer!
    private var featureUniformBuf: MTLBuffer!

    private var textureCache: CVMetalTextureCache!

    // MARK: - Thread-safe state

    private let lock = NSLock()

    private var _settings    = RenderSettings()
    private var _frame:       ARFrame?
    private var _featureBuf:  MTLBuffer?
    private var _featureCount = 0

    // MARK: - FPS

    private var lastDrawTime = Date()

    // MARK: - ARKit

    private let session = ARSession()

    // MARK: - Init

    override init() {
        guard let dev = MTLCreateSystemDefaultDevice() else {
            fatalError("Metal not supported")
        }
        device       = dev
        commandQueue = dev.makeCommandQueue()!
        super.init()

        CVMetalTextureCacheCreate(kCFAllocatorDefault, nil, device, nil, &textureCache)
        buildPipelines()
        startARSession()
    }

    // MARK: - Helpers

    private func lockSettings(_ f: (inout RenderSettings) -> Void) {
        lock.withLock { f(&_settings) }
    }
}

// MARK: - MTKViewDelegate

extension AROverlayRenderer: MTKViewDelegate {

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func draw(in view: MTKView) {
        // Snapshot thread-shared state under lock so the rest of draw() is lock-free.
        let (frame, settings, featureBuf, featureCount): (ARFrame?, RenderSettings, MTLBuffer?, Int) = lock.withLock {
            (_frame, _settings, _featureBuf, _featureCount)
        }

        guard
            let frame,
            let drawable = view.currentDrawable,
            let passDesc = view.currentRenderPassDescriptor,
            let cmd      = commandQueue.makeCommandBuffer(),
            let enc      = cmd.makeRenderCommandEncoder(descriptor: passDesc)
        else { return }

        let vp = view.drawableSize

        // ── Pass 1: camera background ────────────────────────────────────
        writeCameraUniforms(frame: frame, viewportSize: vp)
        drawCameraBackground(encoder: enc, frame: frame)

        // ── Pass 2: overlay ──────────────────────────────────────────────
        if settings.showDepthOverlay {
            if let sd = frame.sceneDepth {
                writeOverlayUniforms(frame: frame, sceneDepth: sd,
                                     viewportSize: vp, settings: settings)
                drawDepthOverlay(encoder: enc, sceneDepth: sd)
            } else {
                writeFeatureUniforms(frame: frame, viewportSize: vp, settings: settings)
                drawFeatureOverlay(encoder: enc, buf: featureBuf, count: featureCount)
            }
        }

        enc.endEncoding()
        cmd.present(drawable)
        cmd.commit()

        // FPS: update @Published on main queue (cheap post, no sync needed here).
        let now  = Date()
        let fps  = 1.0 / now.timeIntervalSince(lastDrawTime)
        lastDrawTime = now
        DispatchQueue.main.async { [weak self] in self?.fps = fps }
    }

    // MARK: - Camera background

    private func writeCameraUniforms(frame: ARFrame, viewportSize: CGSize) {
        let t = frame.displayTransform(for: .portrait, viewportSize: viewportSize)
        let m = simd_float3x3(columns: (
            SIMD3<Float>(Float(t.a),  Float(t.b),  0),
            SIMD3<Float>(Float(t.c),  Float(t.d),  0),
            SIMD3<Float>(Float(t.tx), Float(t.ty), 1)
        ))
        var u = CameraUniforms(displayTransform: m)
        memcpy(cameraUniformBuf.contents(), &u, MemoryLayout<CameraUniforms>.size)
    }

    private func drawCameraBackground(encoder: MTLRenderCommandEncoder,
                                       frame: ARFrame) {
        guard
            let yTex    = makeTexture(frame.capturedImage, format: .r8Unorm,  plane: 0),
            let cbcrTex = makeTexture(frame.capturedImage, format: .rg8Unorm, plane: 1)
        else { return }

        encoder.setRenderPipelineState(cameraPipeline)
        encoder.setDepthStencilState(nodepthState)
        encoder.setVertexBuffer(cameraUniformBuf, offset: 0, index: 0)
        encoder.setFragmentTexture(yTex,    index: 0)
        encoder.setFragmentTexture(cbcrTex, index: 1)
        encoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
    }

    // MARK: - Depth overlay

    private func writeOverlayUniforms(frame: ARFrame,
                                       sceneDepth: ARDepthData,
                                       viewportSize: CGSize,
                                       settings: RenderSettings) {
        let depthMap = sceneDepth.depthMap
        let dW = Float(CVPixelBufferGetWidth(depthMap))
        let dH = Float(CVPixelBufferGetHeight(depthMap))

        let imgSize = frame.camera.imageResolution
        let sx = dW / Float(imgSize.width)
        let sy = dH / Float(imgSize.height)
        let K  = frame.camera.intrinsics  // K[col][row]

        let proj = frame.camera.projectionMatrix(
            for: .portrait, viewportSize: viewportSize,
            zNear: 0.001, zFar: 20.0)

        var u = OverlayUniforms(
            projectionMatrix: proj,
            fx: K[0][0] * sx,  fy: K[1][1] * sy,
            cx: K[2][0] * sx,  cy: K[2][1] * sy,
            depthWidth: dW,    depthHeight: dH,
            maxDepth: settings.maxDepth,
            pointSize: settings.pointSize)
        memcpy(overlayUniformBuf.contents(), &u, MemoryLayout<OverlayUniforms>.size)
    }

    private func drawDepthOverlay(encoder: MTLRenderCommandEncoder,
                                   sceneDepth: ARDepthData) {
        guard let depthTex = makeTexture(sceneDepth.depthMap, format: .r32Float, plane: 0)
        else { return }

        let confTex = sceneDepth.confidenceMap.flatMap {
            makeTexture($0, format: .r8Uint, plane: 0)
        }

        let dW = CVPixelBufferGetWidth(sceneDepth.depthMap)
        let dH = CVPixelBufferGetHeight(sceneDepth.depthMap)

        encoder.setRenderPipelineState(overlayPipeline)
        encoder.setDepthStencilState(depthState)
        encoder.setVertexBuffer(overlayUniformBuf, offset: 0, index: 0)
        encoder.setVertexTexture(depthTex, index: 0)
        if let c = confTex { encoder.setVertexTexture(c, index: 1) }
        encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: dW * dH)

        DispatchQueue.main.async { [weak self] in self?.pointCount = dW * dH }
    }

    // MARK: - Feature-point overlay

    private func writeFeatureUniforms(frame: ARFrame,
                                       viewportSize: CGSize,
                                       settings: RenderSettings) {
        let proj = frame.camera.projectionMatrix(
            for: .portrait, viewportSize: viewportSize,
            zNear: 0.001, zFar: 50.0)
        let vp = proj * frame.camera.viewMatrix(for: .portrait)
        var u = FeatureUniforms(viewProjection: vp, pointSize: settings.pointSize)
        memcpy(featureUniformBuf.contents(), &u, MemoryLayout<FeatureUniforms>.size)
    }

    private func drawFeatureOverlay(encoder: MTLRenderCommandEncoder,
                                     buf: MTLBuffer?, count: Int) {
        guard let buf, count > 0 else { return }
        encoder.setRenderPipelineState(featurePipeline)
        encoder.setDepthStencilState(depthState)
        encoder.setVertexBuffer(featureUniformBuf, offset: 0, index: 0)
        encoder.setVertexBuffer(buf,               offset: 0, index: 1)
        encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: count)
    }

    // MARK: - Pipeline setup

    private func startARSession() {
        let config = ARWorldTrackingConfiguration()
        config.planeDetection = [.horizontal, .vertical]

        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics = .sceneDepth
            DispatchQueue.main.async { [weak self] in
                self?.hasLiDAR      = true
                self?.statusMessage = "LiDAR — depth overlay active"
            }
        } else {
            DispatchQueue.main.async { [weak self] in
                self?.statusMessage = "No LiDAR — showing feature points"
            }
        }
        session.delegate = self
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    private func buildPipelines() {
        guard let lib = device.makeDefaultLibrary() else {
            fatalError("Metal library not found — is Shaders.metal in the target?")
        }

        cameraPipeline  = makePipeline(lib: lib, vtx: "cameraBackgroundVertex",
                                       frag: "cameraBackgroundFragment", blend: false)
        overlayPipeline = makePipeline(lib: lib, vtx: "depthOverlayVertex",
                                       frag: "overlayPointFragment",     blend: true)
        featurePipeline = makePipeline(lib: lib, vtx: "featurePointVertex",
                                       frag: "overlayPointFragment",     blend: true)

        let noWrite = MTLDepthStencilDescriptor()
        noWrite.isDepthWriteEnabled  = false
        noWrite.depthCompareFunction = .always
        nodepthState = device.makeDepthStencilState(descriptor: noWrite)!

        let write = MTLDepthStencilDescriptor()
        write.isDepthWriteEnabled  = true
        write.depthCompareFunction = .less
        depthState = device.makeDepthStencilState(descriptor: write)!

        cameraUniformBuf  = device.makeBuffer(length: MemoryLayout<CameraUniforms>.size,  options: .storageModeShared)
        overlayUniformBuf = device.makeBuffer(length: MemoryLayout<OverlayUniforms>.size, options: .storageModeShared)
        featureUniformBuf = device.makeBuffer(length: MemoryLayout<FeatureUniforms>.size, options: .storageModeShared)
    }

    private func makePipeline(lib: MTLLibrary,
                               vtx: String, frag: String,
                               blend: Bool) -> MTLRenderPipelineState {
        let d = MTLRenderPipelineDescriptor()
        d.vertexFunction   = lib.makeFunction(name: vtx)
        d.fragmentFunction = lib.makeFunction(name: frag)
        let ca = d.colorAttachments[0]!
        ca.pixelFormat = .bgra8Unorm
        if blend {
            ca.isBlendingEnabled           = true
            ca.rgbBlendOperation           = .add
            ca.alphaBlendOperation         = .add
            ca.sourceRGBBlendFactor        = .sourceAlpha
            ca.destinationRGBBlendFactor   = .oneMinusSourceAlpha
            ca.sourceAlphaBlendFactor      = .one
            ca.destinationAlphaBlendFactor = .oneMinusSourceAlpha
        }
        d.depthAttachmentPixelFormat = .depth32Float
        return try! device.makeRenderPipelineState(descriptor: d)
    }

    // MARK: - CVMetalTextureCache helper

    private func makeTexture(_ pixelBuffer: CVPixelBuffer,
                              format: MTLPixelFormat,
                              plane: Int) -> MTLTexture? {
        let w = CVPixelBufferGetWidthOfPlane(pixelBuffer,  plane)
        let h = CVPixelBufferGetHeightOfPlane(pixelBuffer, plane)
        var cv: CVMetalTexture?
        guard CVMetalTextureCacheCreateTextureFromImage(
            kCFAllocatorDefault, textureCache, pixelBuffer, nil,
            format, w, h, plane, &cv) == kCVReturnSuccess, let cv
        else { return nil }
        return CVMetalTextureGetTexture(cv)
    }
}

// MARK: - ARSessionDelegate

extension AROverlayRenderer: ARSessionDelegate {

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        // Update feature-point buffer for non-LiDAR devices.
        var newBuf: MTLBuffer?
        var newCount = 0

        if frame.sceneDepth == nil,
           let fp = frame.rawFeaturePoints {
            let pts  = fp.points
            let size = pts.count * MemoryLayout<SIMD3<Float>>.stride
            newBuf   = device.makeBuffer(bytes: pts, length: size,
                                         options: .storageModeShared)
            newCount = pts.count
        }

        lock.withLock {
            _frame = frame
            if let b = newBuf {
                _featureBuf   = b
                _featureCount = newCount
            }
        }

        if newCount > 0 {
            DispatchQueue.main.async { [weak self] in
                self?.pointCount   = newCount
                self?.statusMessage = "\(newCount) feature pts"
            }
        }
    }

    func session(_ session: ARSession,
                 cameraDidChangeTrackingState camera: ARCamera) {
        let msg: String
        switch camera.trackingState {
        case .notAvailable:                    msg = "Tracking unavailable"
        case .limited(.initializing):         msg = "Initialising…"
        case .limited(.relocalizing):         msg = "Relocalising…"
        case .limited(.excessiveMotion):      msg = "Move camera more slowly"
        case .limited(.insufficientFeatures): msg = "Point at a textured surface"
        case .normal:                          msg = hasLiDAR ? "LiDAR active" : "Tracking normal"
        @unknown default:                      msg = "Unknown tracking state"
        }
        DispatchQueue.main.async { [weak self] in self?.statusMessage = msg }
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        DispatchQueue.main.async { [weak self] in
            self?.statusMessage = "AR error: \(error.localizedDescription)"
        }
    }
}

// MARK: - NSLock convenience

private extension NSLock {
    @discardableResult
    func withLock<T>(_ body: () throws -> T) rethrows -> T {
        lock(); defer { unlock() }
        return try body()
    }
}
