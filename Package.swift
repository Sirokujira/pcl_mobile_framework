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
    "https://github.com/Sirokujira/pcl_mobile_framework/releases/download/v0.1.0/PCLMobile.xcframework.zip"
let xcframeworkChecksum =
    "REPLACE_WITH_SHA256_FROM_make_xcframework.sh"

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
