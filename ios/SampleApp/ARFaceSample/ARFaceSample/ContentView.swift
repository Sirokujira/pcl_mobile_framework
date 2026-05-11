// ContentView.swift — ARFaceSample
//
// Layout:
//   • Full-screen ARSCNView (front camera, face mesh visible)
//   • Bottom panel with top-5 active blend shape bars
//   • Not-supported banner on non-TrueDepth devices

import SwiftUI

struct ContentView: View {

    @StateObject private var coordinator = FaceCoordinator()

    var body: some View {
        ZStack(alignment: .bottom) {
            if coordinator.isSupported {
                ARFaceViewContainer(coordinator: coordinator)
                    .ignoresSafeArea()
            } else {
                unsupportedView
            }

            if coordinator.isSupported {
                blendPanel
            }
        }
        .overlay(alignment: .top) {
            if coordinator.isSupported { statusCapsule }
        }
    }

    // MARK: - Sub-views

    private var unsupportedView: some View {
        VStack(spacing: 16) {
            Image(systemName: "faceid").font(.system(size: 60)).foregroundStyle(.secondary)
            Text("Face tracking requires a\nTrueDepth front camera\n(iPhone X+ / iPad Pro 2020+)")
                .multilineTextAlignment(.center)
                .foregroundStyle(.secondary)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(Color(.systemBackground))
    }

    private var blendPanel: some View {
        VStack(alignment: .leading, spacing: 6) {
            Text("Blend Shapes")
                .font(.caption2.weight(.semibold))
                .foregroundStyle(.secondary)
            ForEach(coordinator.topBlendShapes, id: \.name) { bs in
                blendRow(bs)
            }
        }
        .padding(12)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 12))
        .padding(.horizontal, 16)
        .padding(.bottom, 8)
    }

    private func blendRow(_ bs: FaceCoordinator.BlendShape) -> some View {
        HStack(spacing: 8) {
            Text(bs.name)
                .font(.caption.monospaced())
                .frame(width: 140, alignment: .leading)
            GeometryReader { geo in
                ZStack(alignment: .leading) {
                    RoundedRectangle(cornerRadius: 3)
                        .fill(Color(.systemFill))
                    RoundedRectangle(cornerRadius: 3)
                        .fill(barColor(bs.value))
                        .frame(width: geo.size.width * CGFloat(bs.value))
                }
            }
            .frame(height: 10)
            Text(String(format: "%.2f", bs.value))
                .font(.caption.monospacedDigit())
                .foregroundStyle(.secondary)
                .frame(width: 34)
        }
    }

    private func barColor(_ v: Float) -> Color {
        v > 0.7 ? .red : v > 0.4 ? .orange : .cyan
    }

    private var statusCapsule: some View {
        Text(coordinator.statusMessage)
            .font(.caption)
            .padding(.horizontal, 12)
            .padding(.vertical, 5)
            .background(.regularMaterial, in: Capsule())
            .padding(.top, 52)
    }
}
