// ContentView.swift — MetalNormalSample
//
// Surface normal visualization:
//   Scan with LiDAR → tap "Estimate Normals" → PCLMobile computes
//   a normal vector for each point → Metal colors each point by
//   its normal direction:
//
//     |nx| → Red   (surfaces facing left/right)
//     |ny| → Green (surfaces facing up/down — floor, ceiling)
//     |nz| → Blue  (surfaces facing toward/away camera — walls)
//
// Mixed surfaces produce mixed colors.  Horizontal floors appear green,
// vertical walls appear red or blue depending on orientation, ceilings green.

import SwiftUI

struct ContentView: View {

    @StateObject private var coordinator = NormalCoordinator()

    var body: some View {
        ZStack(alignment: .bottom) {
            NormalMetalViewContainer(coordinator: coordinator)
                .ignoresSafeArea()
            VStack(spacing: 0) { statsPanel; actionPanel }
        }
        .overlay(alignment: .top) { statusCapsule }
        .overlay(alignment: .bottomLeading) { legend }
    }

    private var statsPanel: some View {
        HStack(spacing: 16) {
            statCell("Raw pts",  value: "\(coordinator.rawCount)",   color: .white)
            statCell("Normal pts", value: "\(coordinator.normalCount)", color: .cyan)
            statCell("PCL ms",   value: coordinator.durationLabel,   color: .yellow)
            Spacer()
        }
        .padding(.horizontal, 16).padding(.vertical, 8)
        .background(.regularMaterial)
    }

    private var actionPanel: some View {
        HStack(spacing: 10) {
            Button {
                coordinator.estimateNormals()
            } label: {
                Label(coordinator.isProcessing ? "Estimating…" : "Estimate Normals",
                      systemImage: coordinator.isProcessing ? "arrow.triangle.2.circlepath" : "arrow.up.and.down.and.arrow.left.and.right")
                    .frame(maxWidth: .infinity)
            }
            .buttonStyle(.borderedProminent)
            .disabled(coordinator.isProcessing || coordinator.rawCount < 50)

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

    // Color legend (bottom-left corner)
    private var legend: some View {
        VStack(alignment: .leading, spacing: 4) {
            legendRow("Red",   "Horizontal surface (left/right)")
            legendRow("Green", "Vertical surface (floor/ceiling)")
            legendRow("Blue",  "Depth-facing surface (front/back)")
        }
        .padding(10)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 8))
        .padding(.leading, 12)
        .padding(.bottom, 130)
    }

    private func legendRow(_ color: String, _ desc: String) -> some View {
        HStack(spacing: 6) {
            Circle().fill(Color(color.lowercased())).frame(width: 10, height: 10)
            Text(desc).font(.caption2).foregroundStyle(.secondary)
        }
    }

    private func statCell(_ label: String, value: String, color: Color) -> some View {
        VStack(spacing: 2) {
            Text(label).font(.caption2).foregroundStyle(.secondary)
            Text(value).font(.caption.monospacedDigit().weight(.semibold)).foregroundStyle(color)
        }
    }
}
