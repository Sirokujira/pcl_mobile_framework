// PCLMobileTests.swift
//
// Pure-PCLMobile unit tests. These don't require ARKit and can run on the
// iOS Simulator (or, in fact, on macOS once the XCFramework is built for
// macOS). The tests synthesize point clouds in memory so they exercise
// `make(packedXYZ:count:)`, `voxelGridDownsampled(leaf:)`,
// `passThroughFiltered(axis:min:max:)` and `write(pcdAt:)`.

import XCTest
@testable import PCLMobile  // for symbol visibility from the umbrella header

final class PCLMobileTests: XCTestCase {

    // MARK: - Construction

    func testMakeFromPackedXYZ() throws {
        let packed: [Float] = [
            0, 0, 0,
            1, 1, 1,
            2, 2, 2,
        ]
        let cloud = try packed.withUnsafeBufferPointer { buf in
            try PCLMPointCloud.make(packedXYZ: buf.baseAddress!,
                                    count: 3)
        }
        XCTAssertEqual(cloud.pointCount, 3)
        XCTAssertEqual(cloud.width, 3)
        XCTAssertEqual(cloud.height, 1)
    }

    func testMakeWithEmptyBuffer() throws {
        let cloud = try PCLMPointCloud.make(packedXYZ: nil, count: 0)
        XCTAssertEqual(cloud.pointCount, 0)
    }

    // MARK: - Voxel grid

    func testVoxelGridShrinksDuplicates() throws {
        // 1000 random points clustered inside a 1cm cube. After a 5cm
        // voxel grid downsample, every point should collapse to 1.
        let count = 1000
        var packed = [Float]()
        packed.reserveCapacity(count * 3)
        for _ in 0..<count {
            packed.append(Float.random(in: 0...0.01))
            packed.append(Float.random(in: 0...0.01))
            packed.append(Float.random(in: 0...0.01))
        }
        let cloud = try packed.withUnsafeBufferPointer { buf in
            try PCLMPointCloud.make(packedXYZ: buf.baseAddress!,
                                    count: UInt(count))
        }
        let down = try cloud.voxelGridDownsampled(leaf: 0.05)
        XCTAssertEqual(cloud.pointCount, UInt(count))
        XCTAssertEqual(down.pointCount, 1, "expected 1 voxel for tightly-clustered points")
    }

    func testVoxelGridRejectsZeroLeaf() {
        let cloud = (try? PCLMPointCloud.make(packedXYZ: nil, count: 0)) ??
                    PCLMPointCloud()
        XCTAssertThrowsError(try cloud.voxelGridDownsampled(leaf: 0))
    }

    // MARK: - Pass-through filter

    func testPassThroughKeepsZRange() throws {
        // 11 points along the z-axis at 0.0, 0.1, ..., 1.0.
        var packed = [Float]()
        for i in 0...10 {
            packed.append(0)
            packed.append(0)
            packed.append(Float(i) * 0.1)
        }
        let cloud = try packed.withUnsafeBufferPointer { buf in
            try PCLMPointCloud.make(packedXYZ: buf.baseAddress!, count: 11)
        }
        let kept = try cloud.passThroughFiltered(axis: "z",
                                                 min: 0.25,
                                                 max: 0.75)
        // Should keep points at z in [0.25, 0.75]: 0.3, 0.4, 0.5, 0.6, 0.7.
        XCTAssertEqual(kept.pointCount, 5,
                       "expected 5 points kept, got \(kept.pointCount)")
    }

    func testPassThroughRejectsBadAxis() throws {
        let cloud = try PCLMPointCloud.make(packedXYZ: nil, count: 0)
        XCTAssertThrowsError(try cloud.passThroughFiltered(axis: "",
                                                            min: 0,
                                                            max: 1))
    }

    // MARK: - Round-trip via PCD

    func testRoundTripPCD() throws {
        let packed: [Float] = [
            0, 0, 0,
            0.5, 0.5, 0.5,
            1, 1, 1,
        ]
        let cloud = try packed.withUnsafeBufferPointer { buf in
            try PCLMPointCloud.make(packedXYZ: buf.baseAddress!, count: 3)
        }

        let url = FileManager.default.temporaryDirectory
            .appendingPathComponent("pcl-mobile-roundtrip.pcd")
        defer { try? FileManager.default.removeItem(at: url) }

        try cloud.write(pcdAt: url.path)
        XCTAssertTrue(FileManager.default.fileExists(atPath: url.path))

        let reloaded = try PCLMPointCloud.load(pcdAt: url.path)
        XCTAssertEqual(reloaded.pointCount, 3)

        // Verify the points round-tripped with float precision.
        var roundTripBuf = [Float](repeating: 0, count: 9)
        var actual: UInt = 0
        try roundTripBuf.withUnsafeMutableBufferPointer { mbuf in
            try reloaded.copyPackedXYZ(into: mbuf.baseAddress!,
                                       capacity: 3,
                                       actualCount: &actual)
        }
        XCTAssertEqual(actual, 3)
        for i in 0..<9 {
            XCTAssertEqual(roundTripBuf[i], packed[i], accuracy: 1e-5)
        }
    }
}
