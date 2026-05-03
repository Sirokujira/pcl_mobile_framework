// ContentView.swift

import SwiftUI

struct ContentView: View {
    @StateObject private var lab = PointCloudLab()

    var body: some View {
        NavigationStack {
            Form {
                Section("Source") {
                    Stepper(value: $lab.sourcePointCount,
                            in: 1_000...500_000, step: 5_000) {
                        Text("Synthetic points: \(lab.sourcePointCount)")
                            .monospacedDigit()
                    }
                }

                Section("Voxel grid") {
                    HStack {
                        Text("Leaf size (m)")
                        Spacer()
                        Text(String(format: "%.3f", lab.voxelLeaf))
                            .monospacedDigit()
                    }
                    Slider(value: $lab.voxelLeaf, in: 0.01...0.20, step: 0.005)
                }

                Section("Pass-through filter") {
                    Picker("Axis", selection: $lab.passAxis) {
                        Text("x").tag("x")
                        Text("y").tag("y")
                        Text("z").tag("z")
                    }
                    .pickerStyle(.segmented)

                    HStack {
                        Text("min")
                        Spacer()
                        Text(String(format: "%.2f", lab.passMin))
                            .monospacedDigit()
                    }
                    Slider(value: $lab.passMin, in: 0...1)

                    HStack {
                        Text("max")
                        Spacer()
                        Text(String(format: "%.2f", lab.passMax))
                            .monospacedDigit()
                    }
                    Slider(value: $lab.passMax, in: 0...1)
                }

                Section("Run") {
                    Button {
                        lab.runPipeline()
                    } label: {
                        Label("Run pipeline", systemImage: "play.fill")
                    }
                    .buttonStyle(.borderedProminent)

                    if lab.lastDurationMS > 0 {
                        HStack {
                            Text("Total time")
                            Spacer()
                            Text("\(Int(lab.lastDurationMS)) ms")
                                .monospacedDigit()
                        }
                    }
                }

                if lab.sourceCount > 0 {
                    Section("Results") {
                        statRow("Source",       lab.sourceCount)
                        statRow("After voxel",  lab.downsampledCount)
                        statRow("After passthrough", lab.filteredCount)
                        statRow("Reload from PCD",   lab.roundTripCount)

                        if let url = lab.saveURL {
                            VStack(alignment: .leading, spacing: 4) {
                                Text("Saved PCD")
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                                Text(url.lastPathComponent)
                                    .font(.footnote.monospaced())
                                    .lineLimit(1)
                                    .truncationMode(.middle)
                                Button("Delete saved file",
                                       role: .destructive) {
                                    lab.deleteSavedFile()
                                }
                                .buttonStyle(.bordered)
                            }
                        }
                    }
                }

                if let err = lab.lastError {
                    Section {
                        Text(err)
                            .font(.footnote)
                            .foregroundStyle(.red)
                    } header: {
                        Text("Error").foregroundStyle(.red)
                    }
                }
            }
            .navigationTitle("PCLMobile demo")
        }
    }

    private func statRow(_ label: String, _ value: Int) -> some View {
        HStack {
            Text(label)
            Spacer()
            Text("\(value) pts")
                .monospacedDigit()
                .bold()
        }
    }
}
