# PCLMobile (iOS framework wrapper)

This directory contains the iOS-side wrapper that turns the cross-compiled
PCL/Boost/Eigen/FLANN/Qhull static libraries into a single
`PCLMobile.xcframework` consumers can drop into their app.

## Contents

```
iOSWrapper/
├── CMakeLists.txt              # Generates PCLMobile.framework via Xcode
├── module.modulemap            # Clang module definition (umbrella header)
├── PCLMobile.podspec           # CocoaPods spec (-> XCFramework)
└── Sources/
    ├── Info.plist              # Framework Info.plist (template)
    ├── include/PCLMobile/
    │   ├── PCLMobile.h         # Umbrella header (Objective-C)
    │   └── PCLMPointCloud.h    # Public API
    ├── PCLMobile.mm            # Version constants + error domain
    └── PCLMPointCloud.mm       # Objective-C++ implementation
```

`Sources/include/PCLMobile/` is the only header set exposed to consumers.
The PCL/Boost/Eigen/FLANN/Qhull C++ headers stay strictly inside the .mm
files, so apps can keep their files as plain Objective-C or Swift without
flipping to Objective-C++.

## Build

The wrapper is meant to be driven by `scripts/build_ios.sh` at the repo
root, which runs the cross-compile of PCL/Boost/etc. for every required
slice (device arm64 + simulator arm64 + simulator x86_64) and then asks
CMake to build this framework against each slice.

`scripts/make_xcframework.sh` packages those per-slice frameworks into a
single `build/PCLMobile.xcframework`.

## Public API (Swift)

```swift
import PCLMobile

let cloud = try PointCloud.load(pcdAt: "/var/mobile/lamppost.pcd")
let smaller = try cloud.voxelGridDownsampled(leaf: 0.01)
let cropped = try smaller.passThroughFiltered(axis: "z", min: 0.0, max: 1.5)
print("\(cropped.pointCount) points")
```

## Public API (Objective-C)

```objc
@import PCLMobile;

NSError *error;
PCLMPointCloud *cloud = [PCLMPointCloud cloudFromPCDFile:path error:&error];
PCLMPointCloud *smaller = [cloud voxelGridDownsampleWithLeaf:0.01 error:&error];
NSLog(@"%lu points", (unsigned long)smaller.pointCount);
```

## Distribution

`PCLMobile.xcframework.zip` is attached to every GitHub Release; pick the
package manager you prefer.

### Swift Package Manager (recommended)

`Package.swift` lives at the repo root and declares a `binaryTarget`
pointing at the released zip. Consumers add:

```swift
.package(url: "https://github.com/Sirokujira/pcl_mobile_framework.git",
         from: "0.1.0"),
```

then list `"PCLMobile"` in their target's `dependencies`.

### CocoaPods

```ruby
pod 'PCLMobile', '~> 0.1'
```

`PCLMobile.podspec` references the same release zip. Run
`scripts/lint_podspec.sh` before publishing to Trunk; see top-level
`MODERNIZATION_PLAN.md` for the full publish workflow.

### Carthage

`PCLMobile.json` next to this README is a Carthage *binary project
specification*. Add a single line to your `Cartfile`:

```
binary "https://raw.githubusercontent.com/Sirokujira/pcl_mobile_framework/main/iOSWrapper/PCLMobile.json"
```

then `carthage update --use-xcframeworks --platform iOS`. Carthage drops
the resolved `PCLMobile.xcframework` into `Carthage/Build/`; drag it into
your Xcode target's "Frameworks, Libraries, and Embedded Content" with
**Embed & Sign**.

### Manual XCFramework drop-in (zero-tooling fallback)

If you'd rather not adopt any package manager:

1. Download `PCLMobile.xcframework.zip` from the repo's
   [Releases](https://github.com/Sirokujira/pcl_mobile_framework/releases) page.
2. Unzip and drag `PCLMobile.xcframework` into your Xcode project.
3. In **Target → General → Frameworks, Libraries, and Embedded Content**,
   set the embed mode to **Embed & Sign**.
4. **Build Settings → Other Linker Flags** add `-lc++`.
5. **Build Settings → Build Library for Distribution → No** (Apps).

Done. You can now `import PCLMobile` from Swift or `@import PCLMobile;`
from Objective-C.

### Cutting a new release

Maintainers run a single script that builds the XCFramework, recomputes
checksums, patches `Package.swift` / `PCLMobile.podspec` /
`PCLMobile.json`, then optionally tags and publishes the GitHub Release:

```bash
./scripts/release.sh 0.2.0           # patch manifests only
./scripts/release.sh 0.2.0 --publish # also tag + gh release create
```
