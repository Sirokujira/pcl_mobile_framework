// ARKitIntegrationTests.swift
//
// On-device ARKit ↔ PCLMobile integration tests. The simulator does NOT
// expose real ARKit feature points, so these tests are skipped there.
//
// To run on your iPhone:
//   1. Plug the device in.
//   2. In Xcode, select the device + the `PCLMobileARSampleTests` scheme.
//   3. Cmd-U.
//
// Each test waits up to a few seconds for ARKit to converge on a tracking
// state and then asserts that PCLMobile can ingest the resulting point
// cloud without throwing.

import XCTest
import ARKit
@testable import PCLMobile

final class ARKitIntegrationTests: XCTestCase, ARSessionDelegate {

    private var session: ARSession?
    private var arrived: XCTestExpectation?
    private var lastFrame: ARFrame?

    override func setUpWithError() throws {
        // Hard-skip on Simulator -- ARKit only works on a real device.
        try XCTSkipIf(isRunningOnSimulator(),
                      "ARKitIntegrationTests require a physical device.")
        try XCTSkipUnless(ARWorldTrackingConfiguration.isSupported,
                          "ARWorldTrackingConfiguration unsupported on this device.")
    }

    override func tearDownWithError() throws {
        session?.pause()
        session = nil
    }

    func testCaptureFeaturePointsAndDownsample() throws {
        let session = ARSession()
        session.delegate = self
        self.session = session

        let configuration = ARWorldTrackingConfiguration()
        session.run(configuration, options: [.resetTracking, .removeExistingAnchors])

        let exp = expectation(description: "ARFrame with feature points")
        self.arrived = exp
        wait(for: [exp], timeout: 10)

        guard let frame = lastFrame,
              let raw = frame.rawFeaturePoints, !raw.points.isEmpty else {
            throw XCTSkip("no feature points available; aim the camera at a textured scene.")
        }

        // Flatten ARKit's [SIMD3<Float>] into [x,y,z,...].
        var packed = [Float](); packed.reserveCapacity(raw.points.count * 3)
        for p in raw.points { packed.append(p.x); packed.append(p.y); packed.append(p.z) }

        let cloud = try packed.withUnsafeBufferPointer { buf in
            try PCLMPointCloud.make(packedXYZ: buf.baseAddress!,
                                    count: UInt(raw.points.count))
        }
        XCTAssertEqual(cloud.pointCount, UInt(raw.points.count))

        let down = try cloud.voxelGridDownsampled(leaf: 0.02)
        XCTAssertLessThanOrEqual(down.pointCount, cloud.pointCount,
                                  "downsample should never grow the cloud")
    }

    // MARK: - ARSessionDelegate
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        lastFrame = frame
        if frame.rawFeaturePoints?.points.isEmpty == false {
            arrived?.fulfill()
            arrived = nil
        }
    }

    // MARK: - Helpers
    private func isRunningOnSimulator() -> Bool {
        #if targetEnvironment(simulator)
        return true
        #else
        return false
        #endif
    }
}
