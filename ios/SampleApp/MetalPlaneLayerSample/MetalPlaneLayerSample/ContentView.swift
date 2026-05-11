// ContentView.swift — MetalPlaneLayerSample
//
// LiDAR depth capture → PCLMobile iterative plane segmentation → Metal layer viz.
//
// PCLMobile pipeline (triggered by "Segment" button):
//   1. voxelGridDownsampled(leaf: 0.03)
//   2. statisticalOutlierRemoval(meanK: 20, stddevMulThresh: 1.5)
//   3. segmentPlane × 3 passes: each pass finds the dominant plane in the
//      remaining point cloud and splits it out as a new coloured layer.
//      Points not assigned to any plane are shown as grey.
//
// The plane distance check is done in Swift (not PCL) by computing
//   dist = |dot(normal, p) + d| / |normal|
// for each point, so we can separate inliers from outliers without
// needing a dedicated PCLMobile "extractOutliers" API.

import SwiftUI

struct ContentView: View {

    @StateObject private var coordinator = PlaneLayerCoordinator()

    var body: some View {
        ZStack(alignment: .bottom) {
            PlaneMetalViewContainer(coordinator: coordinator)
                .ignoresSafeArea()
            VStack(spacing: 0) {
                statsPanel
                actionPanel
            }
        }
        .overlay(alignment: .top) { statusCapsule }
    }

    private var statsPanel: some View {
        HStack(spacing: 16) {
            statCell("Raw pts",   value: "\(coordinator.rawCount)",        color: .white)
            statCell("Filtered",  value: "\(coordinator.filteredCount)",   color: .cyan)
            statCell("Layers",    value: "\(coordinator.layerCount)",      color: .orange)
            statCell("PCL ms",    value: coordinator.durationLabel,        color: .yellow)
            Spacer()
        }
        .padding(.horizontal, 16).padding(.vertical, 8)
        .background(.regularMaterial)
    }

    private var actionPanel: some View {
        HStack(spacing: 10) {
            Button {
                coordinator.segment()
            } label: {
                Label(coordinator.isProcessing ? "Processing…" : "Segment Planes",
                      systemImage: coordinator.isProcessing ? "arrow.triangle.2.circlepath" : "square.3.layers.3d")
                    .frame(maxWidth: .infinity)
            }
            .buttonStyle(.borderedProminent)
            .disabled(coordinator.isProcessing || coordinator.rawCount < 100)

            Button(role: .destructive) { coordinator.clear() } label: {
                Image(systemName: "trash")
            }
            .buttonStyle(.bordered)
            .disabled(coordinator.rawCount == 0)
        }
        .padding()
        .background(.regularMaterial)
    }

    private var statusCapsule: some View {
        Text(coordinator.statusMessage)
            .font(.caption)
            .padding(.horizontal, 12).padding(.vertical, 5)
            .background(.regularMaterial, in: Capsule())
            .padding(.top, 52)
    }

    private func statCell(_ label: String, value: String, color: Color) -> some View {
        VStack(spacing: 2) {
            Text(label).font(.caption2).foregroundStyle(.secondary)
            Text(value).font(.caption.monospacedDigit().weight(.semibold)).foregroundStyle(color)
        }
    }
}
