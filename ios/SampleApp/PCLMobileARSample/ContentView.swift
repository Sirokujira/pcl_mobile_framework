// ContentView.swift

import SwiftUI

struct ContentView: View {
    @StateObject private var coordinator = ARPointCloudCoordinator()

    var body: some View {
        ZStack(alignment: .bottom) {
            ARViewContainer(coordinator: coordinator)
                .ignoresSafeArea()

            VStack(alignment: .leading, spacing: 8) {
                statRow(label: "Raw points (this frame)",
                        value: coordinator.lastRawCount)
                statRow(label: "After voxel grid (\(String(format: "%.3f", coordinator.voxelLeaf))m)",
                        value: coordinator.lastDownsampledCount)
                statRow(label: "After passthrough z[\(String(format: "%.2f", coordinator.zMin))…\(String(format: "%.2f", coordinator.zMax))]",
                        value: coordinator.lastFilteredCount)
                if let saved = coordinator.lastSavedURL {
                    Text("Saved: \(saved.lastPathComponent)")
                        .font(.footnote.monospaced())
                        .foregroundStyle(.green)
                }
                if let err = coordinator.lastError {
                    Text("⚠️ \(err)")
                        .font(.footnote)
                        .foregroundStyle(.red)
                }

                HStack {
                    Button("Capture & process") { coordinator.captureNow() }
                        .buttonStyle(.borderedProminent)
                    Button("Save PCD") { coordinator.savePCD() }
                        .buttonStyle(.bordered)
                        .disabled(coordinator.lastFilteredCount == 0)
                }
            }
            .padding()
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
            .padding()
        }
    }

    private func statRow(label: String, value: Int) -> some View {
        HStack {
            Text(label)
            Spacer()
            Text("\(value) pts").monospacedDigit().bold()
        }
        .font(.subheadline)
    }
}
