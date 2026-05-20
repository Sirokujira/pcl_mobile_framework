// swift-tools-version:5.9
//
// Swift Package Manager manifest for PCLMobile.
//
// The published manifest references an XCFramework attached to a GitHub
// Release. To work against a freshly-built local XCFramework instead,
// flip `useLocalXCFramework` to `true` below.
//
// `xcframeworkChecksum` must match the value printed by
// `scripts/make_xcframework.sh` after each release build.

import PackageDescription

// Set to `true` while iterating locally; the manifest will then point at
// build/ios/xcframework/PCLMobile.xcframework instead of the released zip.
let useLocalXCFramework = false

let xcframeworkURL =
    "https://github.com/Sirokujira/pcl_mobile_framework/releases/download/v0.1.2/PCLMobile.xcframework.zip" // x-release-please-version
let xcframeworkChecksum =
    "0e083d194a1ef7cc9b4f0076ca94b52d7f94ae122d177ec6424d9e9ed47d5dd1"

let binaryTarget: Target = useLocalXCFramework
    ? .binaryTarget(
        name: "PCLMobile",
        path: "build/ios/xcframework/PCLMobile.xcframework"
      )
    : .binaryTarget(
        name: "PCLMobile",
        url: xcframeworkURL,
        checksum: xcframeworkChecksum
      )

let package = Package(
    name: "PCLMobile",
    platforms: [
        .iOS(.v13),
    ],
    products: [
        .library(name: "PCLMobile", targets: ["PCLMobile"]),
    ],
    targets: [
        binaryTarget,
    ]
)
