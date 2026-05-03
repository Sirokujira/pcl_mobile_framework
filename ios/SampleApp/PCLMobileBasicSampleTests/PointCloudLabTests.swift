// PointCloudLabTests.swift
//
// End-to-end test that drives PointCloudLab as a real app would: configure
// parameters → runPipeline() → verify the published counts. Runs on the
// iOS Simulator without any special host setup.

import XCTest
@testable import PCLMobileBasicSample

@MainActor
final class PointCloudLabTests: XCTestCase {

    func testPipelineProducesShrinkingCounts() throws {
        let lab = PointCloudLab()
        lab.sourcePointCount = 5_000
        lab.voxelLeaf        = 0.05
        lab.passAxis         = "z"
        lab.passMin          = 0.25
        lab.passMax          = 0.75

        lab.runPipeline()

        XCTAssertNil(lab.lastError, "pipeline failed: \(lab.lastError ?? "")")
        XCTAssertEqual(lab.sourceCount, 5_000)

        // Voxel grid downsamples; output must be ≤ source.
        XCTAssertLessThanOrEqual(lab.downsampledCount, lab.sourceCount)

        // Pass-through filter further reduces (or keeps) the count.
        XCTAssertLessThanOrEqual(lab.filteredCount, lab.downsampledCount)

        // Round-trip via PCD should preserve the count exactly.
        XCTAssertEqual(lab.roundTripCount, lab.filteredCount,
                       "PCD round-trip must preserve the cloud size")

        // The save URL should resolve to an existing file.
        if let url = lab.saveURL {
            XCTAssertTrue(FileManager.default.fileExists(atPath: url.path),
                          "expected the saved PCD at \(url.path)")
            // Clean up so consecutive test runs don't pile up files.
            try? FileManager.default.removeItem(at: url)
        } else {
            XCTFail("expected a saved PCD URL")
        }
    }

    func testPipelineRecordsTiming() {
        let lab = PointCloudLab()
        lab.sourcePointCount = 1_000
        lab.runPipeline()
        XCTAssertGreaterThan(lab.lastDurationMS, 0)
    }
}
