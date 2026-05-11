// ContentView.swift
//
// Layout:
//   ┌───────────────────────────────┐
//   │  status capsule (top)         │
//   │                               │
//   │     Metal orbit viewer        │  ← MTKView (drag/pinch)
//   │                               │
//   │  stats ──────────── [Clear]   │  ← .regularMaterial panel
//   └───────────────────────────────┘

import SwiftUI

struct ContentView: View {

    @StateObject private var capturer = DepthCapturer()

    var body: some View {
        ZStack(alignment: .bottom) {
            MetalViewContainer(capturer: capturer)
                .ignoresSafeArea()

            VStack(spacing: 0) {
                statsRow
                actionRow
            }
        }
        .overlay(alignment: .top) { statusCapsule }
    }

    private var statsRow: some View {
        HStack(spacing: 20) {
            statCell("Points", value: "\(capturer.pointCount)", color: .cyan)
            statCell("Frames", value: "\(capturer.frameCount)",  color: .white)
            Spacer()
            if capturer.hasLiDAR {
                Label("LiDAR", systemImage: "sensor.tag.radiowaves.forward")
                    .font(.caption2)
                    .foregroundStyle(.green)
            } else {
                Label("Feature Pts", systemImage: "dot.radiowaves.left.and.right")
                    .font(.caption2)
                    .foregroundStyle(.yellow)
            }
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 8)
        .background(.regularMaterial)
    }

    private var actionRow: some View {
        HStack {
            Text("Drag: orbit  •  Pinch: zoom  •  2-finger: pan")
                .font(.caption2)
                .foregroundStyle(.secondary)
            Spacer()
            Button(role: .destructive) { capturer.clear() } label: {
                Label("Clear", systemImage: "trash")
            }
            .buttonStyle(.bordered)
            .disabled(capturer.pointCount == 0)
        }
        .padding()
        .background(.regularMaterial)
    }

    private var statusCapsule: some View {
        Text(capturer.statusMessage)
            .font(.caption)
            .padding(.horizontal, 12)
            .padding(.vertical, 5)
            .background(.regularMaterial, in: Capsule())
            .padding(.top, 52)
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
