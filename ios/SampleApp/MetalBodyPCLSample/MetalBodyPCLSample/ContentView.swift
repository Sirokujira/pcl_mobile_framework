// ContentView.swift — MetalBodyPCLSample
//
// Human body motion analysis combining:
//   ARKit  — ARBodyTrackingConfiguration → skeleton joint world positions
//   PCLMobile — boundsAndCentroid + estimateNormals on the joint point cloud
//   Metal  — orbit viewer showing joints, bounding box, normal arrows

import SwiftUI

struct ContentView: View {

    @StateObject private var coordinator = BodyPCLCoordinator()

    var body: some View {
        ZStack(alignment: .bottom) {
            if coordinator.isSupported {
                BodyMetalViewContainer(coordinator: coordinator)
                    .ignoresSafeArea()
            } else {
                unsupportedView
            }

            if coordinator.isSupported {
                VStack(spacing: 0) {
                    statsPanel
                    pclPanel
                    hintBar
                }
            }
        }
        .overlay(alignment: .top) {
            if coordinator.isSupported { statusCapsule }
        }
    }

    // MARK: - Sub-views

    private var unsupportedView: some View {
        VStack(spacing: 16) {
            Image(systemName: "figure.walk").font(.system(size: 60)).foregroundStyle(.secondary)
            Text("Body tracking + PCL analysis\nrequires an A12 Bionic chip\n(iPhone XS / iPad Pro 2018+)")
                .multilineTextAlignment(.center).foregroundStyle(.secondary)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(Color(.systemBackground))
    }

    private var statsPanel: some View {
        HStack(spacing: 16) {
            statCell("Joints",  value: "\(coordinator.jointCount)",   color: .cyan)
            statCell("Height",  value: coordinator.heightLabel,        color: .green)
            statCell("Width",   value: coordinator.widthLabel,         color: .orange)
            statCell("PCL ms",  value: coordinator.pclDurationLabel,   color: .yellow)
            Spacer()
        }
        .padding(.horizontal, 16).padding(.vertical, 8)
        .background(.regularMaterial)
    }

    private var pclPanel: some View {
        HStack(spacing: 10) {
            Toggle("BBox",    isOn: $coordinator.showBBox)
                .toggleStyle(.button).font(.caption2)
            Toggle("Normals", isOn: $coordinator.showNormals)
                .toggleStyle(.button).font(.caption2)
            Toggle("Centroid",isOn: $coordinator.showCentroid)
                .toggleStyle(.button).font(.caption2)
            Spacer()
            Text(coordinator.orientationLabel)
                .font(.caption.monospaced())
                .foregroundStyle(.secondary)
        }
        .padding(.horizontal, 16).padding(.vertical, 6)
        .background(.regularMaterial)
    }

    private var hintBar: some View {
        Text("Drag: orbit  •  Pinch: zoom  •  Stand 2–3 m from camera")
            .font(.caption2).foregroundStyle(.secondary)
            .frame(maxWidth: .infinity).padding(.vertical, 5)
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
