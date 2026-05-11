// ContentView.swift — ARBodySample
//
// Full-body skeleton tracking overlay.
// Requires ARBodyTrackingConfiguration (A12 Bionic or later, iOS 13+).
// On unsupported devices, shows a clear "not supported" message.

import SwiftUI

struct ContentView: View {

    @StateObject private var coordinator = BodyCoordinator()

    var body: some View {
        ZStack(alignment: .bottom) {
            if coordinator.isSupported {
                ARBodyViewContainer(coordinator: coordinator)
                    .ignoresSafeArea()
            } else {
                unsupportedView
            }

            if coordinator.isSupported {
                bottomPanel
            }
        }
        .overlay(alignment: .top) {
            if coordinator.isSupported { statusCapsule }
        }
    }

    private var unsupportedView: some View {
        VStack(spacing: 16) {
            Image(systemName: "figure.walk").font(.system(size: 60)).foregroundStyle(.secondary)
            Text("Body tracking requires an A12 Bionic\nchip (iPhone XS / iPad Pro 2018+)\nwith iOS 13 or later.")
                .multilineTextAlignment(.center)
                .foregroundStyle(.secondary)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(Color(.systemBackground))
    }

    private var bottomPanel: some View {
        HStack(spacing: 20) {
            statCell("Bodies",  value: "\(coordinator.bodyCount)",     color: .green)
            statCell("Joints",  value: "\(coordinator.trackedJoints)", color: .cyan)
            statCell("Quality", value: coordinator.trackingQuality,    color: .orange)
            Spacer()
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 8)
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
