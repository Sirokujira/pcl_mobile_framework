// ContentView.swift — PCLObjectMeasureSample
//
// Workflow:
//   1. Scan floor + objects with LiDAR
//   2. Tap "Measure" — PCL pipeline:
//        voxelGrid(4cm) + SOR → detect floor (RANSAC) → remove floor →
//        Swift voxel-BFS clustering → boundsAndCentroid() per cluster
//   3. Metal viewer shows: floor=grey, each object=distinct colour + AABB wireframe
//   4. Bottom sheet lists W×H×D for each detected object

import SwiftUI

struct ContentView: View {

    @StateObject private var coordinator = ObjectMeasureCoordinator()

    var body: some View {
        ZStack(alignment: .bottom) {
            ObjectMetalViewContainer(coordinator: coordinator)
                .ignoresSafeArea()
            VStack(spacing: 0) {
                statsPanel
                if !coordinator.clusterInfos.isEmpty { clusterList }
                actionPanel
            }
        }
        .overlay(alignment: .top) { statusCapsule }
    }

    // MARK: - Stats

    private var statsPanel: some View {
        HStack(spacing: 16) {
            statCell("Raw",     value: "\(coordinator.rawCount)",     color: .white)
            statCell("Floor",   value: "\(coordinator.floorCount)",   color: .gray)
            statCell("Objects", value: "\(coordinator.clusterCount)", color: .orange)
            statCell("PCL ms",  value: coordinator.durationLabel,     color: .yellow)
            Spacer()
        }
        .padding(.horizontal, 16).padding(.vertical, 8)
        .background(.regularMaterial)
    }

    // MARK: - Object measurement list

    private var clusterList: some View {
        ScrollView(.horizontal, showsIndicators: false) {
            HStack(spacing: 10) {
                ForEach(coordinator.clusterInfos) { info in
                    clusterCard(info)
                }
            }
            .padding(.horizontal, 12).padding(.vertical, 8)
        }
        .background(.ultraThinMaterial)
    }

    private func clusterCard(_ info: ObjectMeasureCoordinator.ClusterInfo) -> some View {
        VStack(alignment: .leading, spacing: 3) {
            HStack(spacing: 5) {
                Circle()
                    .fill(Color(red: Double(info.color.x),
                                green: Double(info.color.y),
                                blue: Double(info.color.z)))
                    .frame(width: 8, height: 8)
                Text(info.label).font(.caption.bold())
            }
            Text(String(format: "W %.2fm", info.w)).font(.caption2.monospacedDigit())
            Text(String(format: "H %.2fm", info.h)).font(.caption2.monospacedDigit())
            Text(String(format: "D %.2fm", info.d)).font(.caption2.monospacedDigit())
        }
        .padding(8)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 8))
    }

    // MARK: - Actions

    private var actionPanel: some View {
        HStack(spacing: 10) {
            Button {
                coordinator.measure()
            } label: {
                Label(coordinator.isProcessing ? "Analysing…" : "Measure Objects",
                      systemImage: coordinator.isProcessing
                          ? "arrow.triangle.2.circlepath"
                          : "ruler")
                    .frame(maxWidth: .infinity)
            }
            .buttonStyle(.borderedProminent)
            .disabled(coordinator.isProcessing || coordinator.rawCount < 200)

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
