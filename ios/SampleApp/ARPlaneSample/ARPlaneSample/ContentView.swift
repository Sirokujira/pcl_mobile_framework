// ContentView.swift — ARPlaneSample
//
// SwiftUI wrapper that shows:
//   • Full-screen ARSCNView with detected planes rendered as coloured meshes
//   • Stats: plane count, placed object count, current classification
//   • Tap anywhere on a detected plane to place a coloured cube

import SwiftUI

struct ContentView: View {

    @StateObject private var coordinator = PlaneCoordinator()

    var body: some View {
        ZStack(alignment: .bottom) {
            ARPlaneViewContainer(coordinator: coordinator)
                .ignoresSafeArea()

            VStack(spacing: 0) {
                statsRow
                legend
                hintRow
            }
        }
        .overlay(alignment: .top) { statusCapsule }
    }

    private var statsRow: some View {
        HStack(spacing: 20) {
            statCell("Planes",   value: "\(coordinator.planeCount)",  color: .cyan)
            statCell("Objects",  value: "\(coordinator.objectCount)", color: .orange)
            Spacer()
            Button(role: .destructive) { coordinator.clearObjects() } label: {
                Label("Clear", systemImage: "trash")
            }
            .buttonStyle(.bordered)
            .disabled(coordinator.objectCount == 0)
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 8)
        .background(.regularMaterial)
    }

    // Color legend for plane classification
    private var legend: some View {
        ScrollView(.horizontal, showsIndicators: false) {
            HStack(spacing: 8) {
                ForEach(PlaneCoordinator.Classification.allCases, id: \.self) { c in
                    HStack(spacing: 4) {
                        Circle().fill(c.color).frame(width: 8, height: 8)
                        Text(c.label).font(.caption2).foregroundStyle(.secondary)
                    }
                }
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 4)
        }
        .background(.regularMaterial)
    }

    private var hintRow: some View {
        Text("Tap a detected plane to place an object")
            .font(.caption)
            .foregroundStyle(.secondary)
            .frame(maxWidth: .infinity)
            .padding(.vertical, 6)
            .background(.regularMaterial)
    }

    private var statusCapsule: some View {
        Text(coordinator.statusMessage)
            .font(.caption)
            .padding(.horizontal, 12)
            .padding(.vertical, 5)
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
