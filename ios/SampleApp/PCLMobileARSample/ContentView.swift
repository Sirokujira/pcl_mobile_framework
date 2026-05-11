// ContentView.swift
//
// SwiftUI overlay sitting above the ARSCNView.
//
// Layout:
//   ┌─────────────────────────────────┐
//   │  status bar (top, capsule)      │
//   │                                 │
//   │        AR world view            │
//   │                                 │
//   │  stats row  │  plane info row   │ ← .regularMaterial panel
//   │  error row                      │
//   │  save confirmation row          │
//   │  ┌──────────────────┐ [💾][🗑]  │ ← action buttons
//   └─────────────────────────────────┘

import SwiftUI

struct ContentView: View {

    @StateObject private var coordinator = ARPointCloudCoordinator()

    var body: some View {
        ZStack(alignment: .bottom) {
            // Full-screen AR view
            ARViewContainer(coordinator: coordinator)
                .ignoresSafeArea()

            // Bottom overlay panel
            VStack(spacing: 0) {
                statsRow
                if let info = coordinator.planeInfo    { planeInfoRow(info) }
                if let err  = coordinator.errorMessage { errorRow(err) }
                if let url  = coordinator.lastSavedURL { savedRow(url) }
                actionButtons
            }
        }
        // Status bar at the top
        .overlay(alignment: .top) { statusBar }
        .sheet(isPresented: $coordinator.showResultsSheet) {
            if let result = coordinator.lastResult {
                ResultsView(result: result) { coordinator.savePCD() }
            }
        }
    }

    // MARK: - Sub-views

    private var statsRow: some View {
        HStack(spacing: 20) {
            statCell("Raw",      count: coordinator.rawCount,      accent: .white)
            statCell("Filtered", count: coordinator.filteredCount,  accent: .cyan)
            statCell("Plane",    count: coordinator.planeCount,     accent: .orange)
            Spacer()
            if coordinator.durationMs > 0 {
                Text(String(format: "%.0f ms", coordinator.durationMs))
                    .font(.caption.monospacedDigit())
                    .foregroundStyle(.secondary)
            }
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 8)
        .background(.regularMaterial)
    }

    private func planeInfoRow(_ info: String) -> some View {
        HStack {
            Image(systemName: "square.3.layers.3d")
                .foregroundStyle(.orange)
            Text(info)
                .font(.caption.monospaced())
                .foregroundStyle(.orange)
            Spacer()
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 4)
        .background(.regularMaterial)
    }

    private func errorRow(_ err: String) -> some View {
        HStack {
            Image(systemName: "exclamationmark.triangle")
                .foregroundStyle(.red)
            Text(err)
                .font(.caption)
                .foregroundStyle(.red)
            Spacer()
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 4)
        .background(.regularMaterial)
    }

    private func savedRow(_ url: URL) -> some View {
        HStack {
            Image(systemName: "checkmark.circle")
                .foregroundStyle(.green)
            Text(url.lastPathComponent)
                .font(.caption.monospaced())
                .foregroundStyle(.green)
            Spacer()
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 4)
        .background(.regularMaterial)
    }

    private var actionButtons: some View {
        HStack(spacing: 10) {
            Button {
                coordinator.captureAndProcess()
            } label: {
                Label(
                    coordinator.isProcessing ? "Processing…" : "Capture & Analyse",
                    systemImage: coordinator.isProcessing
                        ? "arrow.triangle.2.circlepath"
                        : "wand.and.rays"
                )
                .frame(maxWidth: .infinity)
            }
            .buttonStyle(.borderedProminent)
            .disabled(coordinator.isProcessing || coordinator.rawCount < 5)

            Button {
                coordinator.savePCD()
            } label: {
                Image(systemName: "square.and.arrow.down")
            }
            .buttonStyle(.bordered)
            .disabled(coordinator.filteredCount == 0)

            Button(role: .destructive) {
                coordinator.clear()
            } label: {
                Image(systemName: "trash")
            }
            .buttonStyle(.bordered)
            .disabled(coordinator.rawCount == 0 && coordinator.filteredCount == 0)
        }
        .padding()
        .background(.regularMaterial)
    }

    private var statusBar: some View {
        Text(coordinator.statusMessage)
            .font(.caption)
            .padding(.horizontal, 12)
            .padding(.vertical, 5)
            .background(.regularMaterial, in: Capsule())
            .padding(.top, 52)
    }

    // MARK: - Helpers

    private func statCell(_ label: String, count: Int, accent: Color) -> some View {
        VStack(spacing: 2) {
            Text(label)
                .font(.caption2)
                .foregroundStyle(.secondary)
            Text("\(count)")
                .font(.caption.monospacedDigit().weight(.semibold))
                .foregroundStyle(accent)
        }
    }
}
