# pcl_mobile_framework

Cross-compile a curated subset of [PCL](https://pointclouds.org/) plus its
dependencies (Boost, Eigen, FLANN, Qhull) into shippable mobile libraries:

* **iOS** — `PCLMobile.xcframework` (consumable via Swift Package Manager,
  CocoaPods or Carthage).
* **Android** — `pclmobile` AAR (consumable via Gradle / Maven).

## Repository layout

```
pcl_mobile_framework/
├── CMakeLists.txt                 # Top-level CMake superbuild
├── Package.swift                  # Swift Package Manager manifest (iOS)
├── LICENSE
├── README.md                      # ← you are here
├── MODERNIZATION_PLAN.md          # Where this repo is heading
│
├── cmake/
│   ├── SetupSuperbuild.cmake      # Per-tree paths
│   ├── ProjectVariables.cmake     # Options & ccache wiring
│   ├── external/                  # ExternalProject_Add per dependency
│   │   ├── eigen.cmake
│   │   ├── boost.cmake
│   │   ├── flann.cmake
│   │   ├── qhull.cmake
│   │   └── pcl.cmake
│   └── toolchains/
│       ├── ios.toolchain.cmake    # The single iOS toolchain
│       └── pcl-try-run-results.cmake
│
├── scripts/
│   ├── build_android.sh           # All ABIs in one go
│   ├── build_ios.sh               # All slices + XCFramework in one go
│   └── make_xcframework.sh        # Standalone XCFramework merger
│
├── iOSWrapper/                    # PCLMobile.framework sources
│   ├── CMakeLists.txt             #   Builds the framework via Xcode
│   ├── module.modulemap
│   ├── PCLMobile.podspec
│   └── Sources/
│       ├── Info.plist
│       ├── PCLMobile.mm
│       ├── PCLMPointCloud.mm
│       └── include/PCLMobile/{PCLMobile.h,PCLMPointCloud.h}
│
├── AndroidWrapper/aar/            # pclmobile AAR + sample app
│   ├── settings.gradle.kts
│   ├── build.gradle.kts
│   ├── pclmobile/                 # The AAR module
│   │   ├── build.gradle.kts
│   │   ├── CMakeLists.txt
│   │   └── src/main/{java,cpp,res,AndroidManifest.xml}
│   └── app/                       # Sample / smoke-test app
│
├── strip-frameworks.sh            # App-Store hook (Realm, Apache-2.0)
└── .deprecated/                   # Pre-modernization files (history only)
```

## Requirements

| | Recommended | Minimum |
|---|---|---|
| CMake          | 3.31+ | 3.24 |
| Android Studio | Hedgehog (2023.1.1) or newer | Iguana w/ AGP 8.5 |
| Android NDK    | r29 | r26 |
| Android SDK    | API 34 | API 24 (`minSdk`) |
| Xcode          | 15.x  | 15.0 |
| iOS deployment | 13.0+ | 13.0 |

The toolchain script auto-detects Xcode/SDK paths via `xcrun`, so any modern
Xcode install on macOS 13+ will work.

## Building

### Android (AAR)

```bash
export ANDROID_NDK_HOME=$HOME/Library/Android/sdk/ndk/29.0.14206865

# 1. Cross-compile PCL/Boost/FLANN/Qhull/Eigen for every required ABI.
#    Output is staged under AndroidWrapper/aar/pclmobile/libs/<ABI>/.
./scripts/build_android.sh

# 2. Build the AAR.
cd AndroidWrapper/aar
sh ./gradlew :pclmobile:assembleRelease

# 3. (optional) Publish to a Maven repository.
sh ./gradlew :pclmobile:publishReleasePublicationToMavenLocal
```

Or open `AndroidWrapper/aar/` in Android Studio (Hedgehog or later) and run
the `:pclmobile:assembleRelease` task from the Gradle pane.

### iOS (XCFramework)

```bash
# 1. Cross-compile every slice (device arm64, simulator arm64, simulator x86_64).
#    Slices to build can be limited via IOS_SLICES="OS64 SIMULATORARM64".
./scripts/build_ios.sh

# 2. The XCFramework (and a zipped + checksummed copy) ends up at
#    build/ios/xcframework/PCLMobile.xcframework{,.zip}
```

If you only need to play with the Objective-C++ wrapper (without the
underlying PCL build), open `iOSWrapper/CMakeLists.txt` directly in Xcode by
running `cmake -S iOSWrapper -B iOSWrapper/build.ios -G Xcode -DPLATFORM=OS64
-DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/ios.toolchain.cmake` and opening the
generated `PCLMobile.xcodeproj`.

## Consuming the iOS XCFramework

### Swift Package Manager

```swift
.package(url: "https://github.com/Sirokujira/pcl_mobile_framework.git", from: "0.1.0"),
```

```swift
import PCLMobile

let cloud   = try PointCloud.load(pcdAt: path)
let smaller = try cloud.voxelGridDownsampled(leaf: 0.01)
let cropped = try smaller.passThroughFiltered(axis: "z", min: 0.0, max: 1.5)
print(cropped.pointCount)
```

### CocoaPods

```ruby
pod 'PCLMobile', '~> 0.1'
```

### Carthage

Add a single line to your `Cartfile`:

```
binary "https://raw.githubusercontent.com/Sirokujira/pcl_mobile_framework/main/iOSWrapper/PCLMobile.json"
```

then run `carthage update --use-xcframeworks --platform iOS`. Drag the
resolved `Carthage/Build/PCLMobile.xcframework` into your Xcode target.

### Manual XCFramework drop-in

If you'd rather not adopt a package manager: grab
`PCLMobile.xcframework.zip` from the
[Releases page](https://github.com/Sirokujira/pcl_mobile_framework/releases),
unzip, drag the `.xcframework` into Xcode, set Embed mode to
**Embed & Sign**, and add `-lc++` to *Other Linker Flags*. See
[iOSWrapper/README.md](./iOSWrapper/README.md) for full instructions.

### Maintainer release workflow

`scripts/release.sh <version> [--publish]` rebuilds the XCFramework,
patches every manifest (`Package.swift`, `PCLMobile.podspec`,
`PCLMobile.json`) with the new version + checksum, and (optionally) tags
the repo and uploads the zip via `gh release create`. See
[iOSWrapper/README.md](./iOSWrapper/README.md) for details.

Manual iOS release:

```sh
cd ~/Github/pcl_mobile_framework
./scripts/build_ios.sh
./scripts/lint_podspec.sh
git tag v0.1.0
git push origin v0.1.0
gh release create v0.1.0 \
  build/ios/xcframework/PCLMobile.xcframework.zip \
  --title "PCLMobile 0.1.0" --notes "Initial release"
pod trunk push iOSWrapper/PCLMobile.podspec --allow-warnings
```

Android package release for the same tag:

```sh
cd ~/Github/pcl_mobile_framework
ANDROID_NDK_VERSION=29.0.14206865 \
ANDROID_CMAKE_VERSION=3.31.6 \
PCLMOBILE_GITHUB_REPOSITORY="Sirokujira/pcl_mobile_framework" \
MAVEN_USERNAME="$GITHUB_ACTOR" \
MAVEN_PASSWORD="$GITHUB_TOKEN" \
./scripts/package_android.sh 0.1.0 \
  --create-release \
  --upload-release \
  --publish-github-packages
```

The Android release workflow also runs automatically when a GitHub Release is
published. It rebuilds the multi-ABI AAR, uploads it to the same release, and
publishes `io.github.sirokujira:pclmobile:<version>` to GitHub Packages.

Android Maven publish options:

```sh
cd ~/Github/pcl_mobile_framework

# Local Maven cache: ~/.m2/repository/io/github/sirokujira/pclmobile/
ANDROID_CMAKE_VERSION=3.31.6 \
./scripts/package_android.sh 0.1.0 --maven-local

# Local repository under AndroidWrapper/aar/pclmobile/build/repo/
ANDROID_CMAKE_VERSION=3.31.6 \
./scripts/package_android.sh 0.1.0 --local-repo

# GitHub Packages
ANDROID_CMAKE_VERSION=3.31.6 \
PCLMOBILE_GITHUB_REPOSITORY="Sirokujira/pcl_mobile_framework" \
MAVEN_USERNAME="$GITHUB_ACTOR" \
MAVEN_PASSWORD="$GITHUB_TOKEN" \
./scripts/package_android.sh 0.1.0 --publish-github-packages

# Another Maven-compatible repository
ANDROID_CMAKE_VERSION=3.31.6 \
MAVEN_USERNAME="$MAVEN_USERNAME" \
MAVEN_PASSWORD="$MAVEN_PASSWORD" \
./scripts/package_android.sh 0.1.0 \
  --maven-repository-url "https://maven.example.com/releases"
```

## Publishing the Android AAR

Provider-side GitHub Packages release:

```sh
cd ~/Github/pcl_mobile_framework
ANDROID_NDK_VERSION=29.0.14206865 \
ANDROID_CMAKE_VERSION=3.31.6 \
PCLMOBILE_GITHUB_REPOSITORY="Sirokujira/pcl_mobile_framework" \
MAVEN_USERNAME="$GITHUB_ACTOR" \
MAVEN_PASSWORD="$GITHUB_TOKEN" \
./scripts/package_android.sh 0.1.0 \
  --create-release \
  --upload-release \
  --publish-github-packages
```

For GitHub Actions, run the `Android Package Release` workflow with:

```text
version: 0.1.0
create_release: true
upload_release: true
publish_github_packages: true
```

After publishing, the provider owns:

```text
GitHub Release asset: build/android/distributions/pclmobile-0.1.0.aar
GitHub Release checksum: build/android/distributions/pclmobile-0.1.0.aar.sha256
GitHub Packages coordinate: io.github.sirokujira:pclmobile:0.1.0
```

## Consuming the Android AAR

The following `repositories` and `dependencies` block is for app projects that
use the package after the provider has published it to GitHub Packages.

`build.gradle.kts`:

```kotlin
repositories {
    google()
    mavenCentral()
    maven {
        url = uri("https://maven.pkg.github.com/Sirokujira/pcl_mobile_framework")
        credentials {
            username = providers.gradleProperty("gpr.user")
                .orElse(providers.environmentVariable("GITHUB_ACTOR"))
                .get()
            password = providers.gradleProperty("gpr.key")
                .orElse(providers.environmentVariable("GITHUB_TOKEN"))
                .get()
        }
    }
}

dependencies {
    implementation("io.github.sirokujira:pclmobile:0.1.0")
}
```

Or, from a local repo generated by `./scripts/package_android.sh 0.1.0 --local-repo`:

```kotlin
repositories {
    google()
    mavenCentral()
    maven {
        url = uri("/path/to/pcl_mobile_framework/AndroidWrapper/aar/pclmobile/build/repo")
    }
}

dependencies {
    implementation("io.github.sirokujira:pclmobile:0.1.0")
}
```

Or, as a manually downloaded AAR from GitHub Release:

```kotlin
dependencies {
    implementation(files("libs/pclmobile-0.1.0.aar"))
}
```

## Status

This repository is in the middle of a major modernization (see
[MODERNIZATION_PLAN.md](./MODERNIZATION_PLAN.md)). Phases A and B from that
plan are landed; phases C (CI on GitHub Actions, releases) and D
(documentation polish, integration tests) are still in progress.

## License

Apache License 2.0. See [`LICENSE`](./LICENSE).

`strip-frameworks.sh` is from Realm Inc. and retains its original Apache-2.0
license header.
