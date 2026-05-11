// ContentView.swift — PCLConvexHullSample
//
// Workflow:
//   1. Scan flat surfaces (floor, walls, tables) with LiDAR
//   2. Tap "Compute Hulls" — for each plane:
//        RANSAC plane fit → project inliers → PCL convexHull() → polygon boundary
//   3. Metal viewer: dim raw cloud + filled translucent polygon + bright outline per plane
//   4. Legend shows: plane label, hull vertex count, approximate surface area

import SwiftUI

struct ContentView: View {

    @StateObject private var coordinator = ConvexHullCoordinator()

    var body: some View {
        ZStack(alignment: .bottom) {
            ConvexHullViewContainer(coordinator: coordinator)
                .ignoresSafeArea()
            VStack(spacing: 0) { statsPanel; actionPanel }
        }
        .overlay(alignment: .top) { statusCapsule }
        .overlay(alignment: .bottomLeading) { legend }
    }

    private var statsPanel: some View {
        HStack(spacing: 16) {
            statCell("Raw pts",  value: "\(coordinator.rawCount)",     color: .white)
            statCell("Planes",   value: "\(coordinator.hullCount)",    color: .cyan)
            statCell("PCL ms",   value: coordinator.durationLabel,     color: .yellow)
            Spacer()
        }
        .padding(.horizontal, 16).padding(.vertical, 8)
        .background(.regularMaterial)
    }

    private var actionPanel: some View {
        HStack(spacing: 10) {
            Button {
                coordinator.computeHulls()
            } label: {
                Label(coordinator.isProcessing ? "Computing…" : "Compute Hulls",
                      systemImage: coordinator.isProcessing
                          ? "arrow.triangle.2.circlepath"
                          : "skew")
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

    private var legend: some View {
        VStack(alignment: .leading, spacing: 6) {
            if coordinator.hullInfos.isEmpty {
                Text("No hulls yet").font(.caption2).foregroundStyle(.secondary)
            } else {
                ForEach(coordinator.hullInfos) { info in
                    HStack(spacing: 6) {
                        RoundedRectangle(cornerRadius: 2)
                            .fill(Color(red:   Double(info.color.x),
                                        green: Double(info.color.y),
                                        blue:  Double(info.color.z)))
                            .frame(width: 14, height: 8)
                        VStack(alignment: .leading, spacing: 1) {
                            Text(info.label).font(.caption2.bold())
                            Text(String(format: "%.2f m²  ·  %d pts", info.area, info.pts))
                                .font(.caption2).foregroundStyle(.secondary)
                        }
                    }
                }
            }
        }
        .padding(10)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 8))
        .padding(.leading, 12)
        .padding(.bottom, 130)
    }

    private func statCell(_ label: String, value: String, color: Color) -> some View {
        VStack(spacing: 2) {
            Text(label).font(.caption2).foregroundStyle(.secondary)
            Text(value).font(.caption.monospacedDigit().weight(.semibold)).foregroundStyle(color)
        }
    }
}
