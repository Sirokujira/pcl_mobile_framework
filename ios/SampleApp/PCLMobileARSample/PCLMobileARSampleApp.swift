// PCLMobileARSampleApp.swift
//
// SwiftUI entry point for the on-device test harness. Designed to run on a
// physical iPhone / iPad — the simulator does not provide a real ARKit
// session, so most of this app is a no-op there.

import SwiftUI

@main
struct PCLMobileARSampleApp: App {
    var body: some Scene {
        WindowGroup {
            ContentView()
        }
    }
}
