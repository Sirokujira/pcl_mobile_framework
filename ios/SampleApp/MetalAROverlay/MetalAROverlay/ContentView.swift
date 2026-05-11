// ContentView.swift — MetalAROverlay
//
// Full-screen MTKView renders:
//   Pass 1: camera YCbCr feed as RGB background
//   Pass 2: depth/feature points overlaid in AR world space
//
// Confidence filter and max-depth sliders are exposed in an overlay sheet.

import SwiftUI

struct ContentView: View {

    @StateObject private var renderer = AROverlayRenderer()
    @State private var showSettings   = false

    var body: some View {
        ZStack(alignment: .bottom) {
            ARMetalViewContainer(renderer: renderer)
                .ignoresSafeArea()

            VStack(spacing: 0) {
                statsRow
                actionRow
            }
        }
        .overlay(alignment: .top) { statusCapsule }
        .sheet(isPresented: $showSettings) { settingsSheet }
    }

    // MARK: - Sub-views

    private var statsRow: some View {
        HStack(spacing: 20) {
            statCell("Mode",
                     value: renderer.hasLiDAR ? "LiDAR" : "Feature",
                     color: renderer.hasLiDAR ? .green : .yellow)
            statCell("Depth pts", value: "\(renderer.pointCount)", color: .cyan)
            statCell("FPS",       value: String(format: "%.0f", renderer.fps), color: .white)
            Spacer()
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 8)
        .background(.regularMaterial)
    }

    private var actionRow: some View {
        HStack {
            Toggle("Show depth", isOn: $renderer.showDepthOverlay)
                .toggleStyle(.button)
                .font(.caption)
            Spacer()
            Button { showSettings = true } label: {
                Image(systemName: "slider.horizontal.3")
            }
            .buttonStyle(.bordered)
        }
        .padding()
        .background(.regularMaterial)
    }

    private var statusCapsule: some View {
        Text(renderer.statusMessage)
            .font(.caption)
            .padding(.horizontal, 12)
            .padding(.vertical, 5)
            .background(.regularMaterial, in: Capsule())
            .padding(.top, 52)
    }

    // MARK: - Settings sheet

    private var settingsSheet: some View {
        NavigationStack {
            Form {
                Section("Depth filter") {
                    LabeledContent("Max depth: \(String(format: "%.1f", renderer.maxDepth)) m") {
                        Slider(value: $renderer.maxDepth, in: 0.5 ... 8.0, step: 0.5)
                    }
                    LabeledContent("Point size: \(String(format: "%.1f", renderer.pointSize))") {
                        Slider(value: $renderer.pointSize, in: 1.0 ... 10.0, step: 0.5)
                    }
                    Toggle("High-confidence only", isOn: $renderer.highConfidenceOnly)
                }
            }
            .navigationTitle("Settings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .confirmationAction) {
                    Button("Done") { showSettings = false }
                }
            }
        }
        .presentationDetents([.medium])
    }

    private func statCell(_ label: String, value: String, color: Color) -> some View {
        VStack(spacing: 2) {
            Text(label).font(.caption2).foregroundStyle(.secondary)
            Text(value)
                .font(.caption.monospacedDigit().weight(.semibold))
                .foregroundStyle(color)
        }
    }
}
