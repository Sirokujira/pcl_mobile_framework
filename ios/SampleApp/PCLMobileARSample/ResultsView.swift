// ResultsView.swift
//
// Sheet shown after Capture & Analyse completes.
// Displays stats, plane model, and a touch-navigable 3D SceneKit preview.

import SwiftUI
import SceneKit
import simd
import PCLMobile

struct ResultsView: View {

    let result: PCLPipelineResult
    let onSave: () -> Void

    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationStack {
            List {
                statsSection
                if let m = result.planeModel { planeSection(m) }
                previewSection
            }
            .navigationTitle("Capture Result")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Done") { dismiss() }
                }
                ToolbarItem(placement: .confirmationAction) {
                    Button("Save PCD") { onSave(); dismiss() }
                        .disabled(result.filteredPoints.isEmpty)
                }
            }
        }
    }

    // MARK: - Sections

    private var statsSection: some View {
        Section("Point Cloud") {
            LabeledContent("Filtered points") {
                Text("\(result.filteredPoints.count)")
                    .foregroundStyle(.cyan)
            }
            LabeledContent("Plane inliers") {
                Text("\(result.planePoints.count)")
                    .foregroundStyle(.orange)
            }
            LabeledContent("Processing time") {
                Text(String(format: "%.0f ms", result.durationMs))
                    .foregroundStyle(.secondary)
            }
        }
    }

    private func planeSection(_ m: PlaneModel) -> some View {
        let inlierPct = m.inputCount > 0
            ? Int(100 * Double(m.inlierCount) / Double(m.inputCount))
            : 0
        return Section("Plane Model") {
            LabeledContent("Normal") {
                Text(String(format: "(%.3f, %.3f, %.3f)", m.a, m.b, m.c))
                    .font(.caption.monospaced())
                    .foregroundStyle(.orange)
            }
            LabeledContent("Distance") {
                Text(String(format: "%.3f m", abs(m.d)))
                    .font(.caption.monospaced())
            }
            LabeledContent("Inliers") {
                Text("\(m.inlierCount) / \(m.inputCount)  (\(inlierPct)%)")
                    .foregroundStyle(inlierPct > 50 ? .green : .secondary)
            }
        }
    }

    private var previewSection: some View {
        Section("3D Preview") {
            PointCloudPreview(result: result)
                .frame(height: 300)
                .listRowInsets(EdgeInsets())
        }
    }
}

// MARK: - SceneKit 3D Preview

private struct PointCloudPreview: UIViewRepresentable {

    let result: PCLPipelineResult

    func makeUIView(context: Context) -> SCNView {
        let scnView = SCNView()
        scnView.allowsCameraControl      = true
        scnView.autoenablesDefaultLighting = true
        scnView.backgroundColor          = .black
        scnView.antialiasingMode         = .multisampling4X

        let scene = SCNScene()
        scnView.scene = scene

        if !result.filteredPoints.isEmpty {
            let geo = PointCloudNode.geometry(
                points: result.filteredPoints,
                color:  UIColor(red: 0, green: 0.85, blue: 1, alpha: 1),
                pointSize: 4)
            scene.rootNode.addChildNode(SCNNode(geometry: geo))
        }

        if !result.planePoints.isEmpty {
            let geo = PointCloudNode.geometry(
                points: result.planePoints,
                color:  UIColor(red: 1, green: 0.55, blue: 0, alpha: 1),
                pointSize: 5)
            scene.rootNode.addChildNode(SCNNode(geometry: geo))
        }

        scnView.pointOfView = makeCamera(for: scene)
        return scnView
    }

    func updateUIView(_ uiView: SCNView, context: Context) {}

    // Place a camera that frames all rendered points.
    private func makeCamera(for scene: SCNScene) -> SCNNode {
        let center = result.centroid ?? simd_float3(0, 0, 0)

        // Bounding radius from centroid.
        let radius = result.filteredPoints.reduce(Float(0)) { acc, p in
            max(acc, simd_distance(p, center))
        }
        let dist = max(radius * 2.5, 0.5)

        let cam  = SCNCamera()
        cam.automaticallyAdjustsZRange = true

        let node = SCNNode()
        node.camera       = cam
        node.simdPosition = center + simd_float3(0, dist * 0.4, dist)
        node.look(at: SCNVector3(center.x, center.y, center.z))
        scene.rootNode.addChildNode(node)
        return node
    }
}
