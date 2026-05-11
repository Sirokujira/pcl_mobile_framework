// ContentView.swift — PCLScanExportSample
//
// Workflow:
//   1. Scan room with LiDAR (raw points accumulate automatically)
//   2. Tap "Filter" → PCL voxelGrid(3cm) + SOR → filtered cloud shown in Metal viewer
//   3. Tap "Export PCD" → PCLMobile writes ASCII .pcd → iOS share sheet
//
// The exported file can be opened in MeshLab, CloudCompare, Open3D, or any
// PCL-aware tool on desktop.

import SwiftUI

struct ContentView: View {

    @StateObject private var coordinator = ScanExportCoordinator()
    @State private var showShare = false

    var body: some View {
        ZStack(alignment: .bottom) {
            ScanMetalViewContainer(coordinator: coordinator)
                .ignoresSafeArea()
            VStack(spacing: 0) { statsPanel; actionPanel }
        }
        .overlay(alignment: .top) { statusCapsule }
        .sheet(isPresented: $showShare) {
            if let url = coordinator.exportURL {
                ActivityView(url: url)
            }
        }
        .onChange(of: coordinator.exportURL) { _, url in
            if url != nil { showShare = true }
        }
    }

    private var statsPanel: some View {
        HStack(spacing: 16) {
            statCell("Raw",       value: "\(coordinator.rawCount)",      color: .white)
            statCell("Filtered",  value: "\(coordinator.filteredCount)", color: .cyan)
            statCell("PCL ms",    value: coordinator.durationLabel,      color: .yellow)
            statCell("File KB",   value: coordinator.exportSizeKB > 0 ? "\(coordinator.exportSizeKB)" : "—",
                     color: .green)
            Spacer()
        }
        .padding(.horizontal, 16).padding(.vertical, 8)
        .background(.regularMaterial)
    }

    private var actionPanel: some View {
        HStack(spacing: 10) {
            Button {
                coordinator.filter()
            } label: {
                Label(coordinator.isProcessing ? "Filtering…" : "Filter",
                      systemImage: "wand.and.stars")
                    .frame(maxWidth: .infinity)
            }
            .buttonStyle(.borderedProminent)
            .disabled(coordinator.isProcessing || coordinator.rawCount < 100)

            Button {
                coordinator.export()
            } label: {
                Label("Export PCD", systemImage: "square.and.arrow.up")
                    .frame(maxWidth: .infinity)
            }
            .buttonStyle(.bordered)
            .tint(.green)
            .disabled(coordinator.isProcessing || coordinator.filteredCount == 0)

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

// MARK: - UIActivityViewController wrapper

struct ActivityView: UIViewControllerRepresentable {
    let url: URL
    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: [url], applicationActivities: nil)
    }
    func updateUIViewController(_ vc: UIActivityViewController, context: Context) {}
}
