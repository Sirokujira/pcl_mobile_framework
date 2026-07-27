# .deprecated/

Files retained for historical reference only. They are not part of the build
and will be deleted once the modernized layout is proven on real hardware.
See [MODERNIZATION_PLAN.md](../MODERNIZATION_PLAN.md) for context.

Most of these arrived with this repository's 2019 import commit `98c6c50`.
Eight did not — the six `AndroidWrapper/aar/` originals plus
`AndroidWrapper/.circleci/config.yml.original` and
`AndroidWrapper/bitrise_android.yml.original`, which this repository wrote
itself in `90b905e` (2019-06-07) and `e21dc8a` (2020-01-03).

Of the imported ones, some have an ancestor in the predecessor project,
`Sirokujira/pcl-superbuild@83cc231` (tagged here as `pcl-superbuild-origin`) —
`setup-superbuild.cmake.original`, `setup-project-variables.cmake.original`,
`makeFramework.sh.original`, the `toolchains/` files — while others (the
`build_ios_*` / `build_android` drivers, `makeFramework2.sh.original`,
`xamarinObjevtiveSharpie.sh`, the whole `AndroidWrapper/` tree) were never
committed to pcl-superbuild at all. The table below is the replacement side of
the derivation map in [docs/LINEAGE.md](../docs/LINEAGE.md), which marks the
distinction with †.

Each `*.original` is the file as it stood **when it was retired**, which for
several is a later state than the 2019 import — this repository kept editing
them until 2020. `build_android.sh.original`, `makeFramework2.sh.original` and
the four `build_ios_*.original` all differ from their `98c6c50` blobs; use
`git --no-replace-objects show 98c6c50:<path>` for the imported state.

## Why each file moved here

### Build scripts
| File | Replaced by | Why |
|---|---|---|
| `build_android.sh.original` / `build_android.bat.original` | `scripts/build_android.sh` | NDK r16b + Clang 3.6 era; broken shell continuation. |
| `build_ios_device_framework.sh.original` | `scripts/build_ios.sh` | Hardcoded armv7/armv7s/arm64e options; targets unsupported on modern iOS. |
| `build_ios_simulator_framework.sh.original` | `scripts/build_ios.sh` | Same. Plus references `iOSWrapper/build.sim64/` which no project ever produced. |
| `build_ios_universal_binary.sh.original` | `scripts/build_ios.sh` | Used `lipo` to glue device + simulator into a fat lib. Apple deprecated this in favour of XCFramework. |
| `build_ios_universal_framework.sh.original` | `scripts/make_xcframework.sh` | Same. |
| `makeFramework.sh.original` / `makeFramework2.sh.original` | `scripts/make_xcframework.sh` | 763 lines of nearly-duplicate fat-framework logic with mismatched variable names. |
| `xamarinObjevtiveSharpie.sh` | (none) | Xamarin reached EOL in May 2024; no longer relevant. |

### CMake setup / toolchains
| File | Replaced by | Why |
|---|---|---|
| `setup-superbuild.cmake.original` | `cmake/SetupSuperbuild.cmake` | Renamed + slimmed. |
| `setup-project-variables.cmake.original` | `cmake/ProjectVariables.cmake` | Dropped armv7/armv7s/arm64e/i386 options and the per-arch try-run cache files. |
| `toolchains/iOS_Device_*.cmake` (4 files) | `cmake/toolchains/ios.toolchain.cmake` | Each was a copy-pasted clone differing only in `IOS_ARCH`. |
| `toolchains/iOS_Simulator_*.cmake` (3 files) | `cmake/toolchains/ios.toolchain.cmake` | Same. |
| `toolchains/Toolchain-iPhone*.cmake` (3 files) | `cmake/toolchains/ios.toolchain.cmake` | OpenCV-derived `common-ios-toolchain.cmake` shim. |
| `toolchains/iOS.cmake` / `iOS_xcode.cmake` / `iOS.toolchain.cmake` | `cmake/toolchains/ios.toolchain.cmake` | Three unrelated toolchains in parallel. |
| `toolchains/iOS_Simulation.toolchain.cmake` | `cmake/toolchains/ios.toolchain.cmake` | Old cristeab/ios-cmake fork. |
| `toolchains/common-ios-toolchain.cmake` | `cmake/toolchains/ios.toolchain.cmake` | OpenCV import; uses the now-removed `CMakeForceCompiler`. |
| `toolchains/vtk-try-run-results.cmake` | (none) | VTK was never enabled (`fetch_vtk()` was always commented out). |

### CI / installers
| File | Replaced by | Why |
|---|---|---|
| `appveyor.yml.original` / `appveyor/` | (planned) GitHub Actions | VS2015 image + MinGW + NDK r16b, all unobtainable today. |
| `AndroidWrapper/.circleci/config.yml.original` | (planned) GitHub Actions | `circleci/android:api-26-alpha` image; AGP 3.x. |
| `AndroidWrapper/bitrise_android.yml.original` | (planned) GitHub Actions | Same. |
| `AndroidWrapper/InstallPointCloudLibrary.sh` / `.bat` | `scripts/build_android.sh` (rebuild from source) | Bash syntax broken (PowerShell-style `$VAR = '...'`) and the AppVeyor artifact URLs no longer resolve. |

### AAR (Groovy → Kotlin DSL)
| File | Replaced by | Why |
|---|---|---|
| `AndroidWrapper/aar/build.gradle.original` | `AndroidWrapper/aar/build.gradle.kts` | AGP 3.4.1 / Gradle 5.1.1 / `jcenter()` / Kotlin 1.3 → AGP 8.5 / Gradle 8.7 / Kotlin 1.9. |
| `AndroidWrapper/aar/settings.gradle.original` | `AndroidWrapper/aar/settings.gradle.kts` | Kotlin DSL + `dependencyResolutionManagement`. |
| `AndroidWrapper/aar/pclmobile/build.gradle.original` | `AndroidWrapper/aar/pclmobile/build.gradle.kts` | AndroidX, `arm64-v8a` added, `maven-publish`, namespace property. |
| `AndroidWrapper/aar/app/build.gradle.original` | `AndroidWrapper/aar/app/build.gradle.kts` | Same. |
| `AndroidWrapper/aar/app/CMakeLists.txt.original` | (deleted, unused) | Only built a stub `stringFromJNI` that `MainActivity` never called. |
| `AndroidWrapper/aar/app/src/main/cpp/native-lib.cpp.original` | (deleted, unused) | Same. |
| `AndroidWrapper/CMakeLists.txt.original` | `AndroidWrapper/aar/pclmobile/CMakeLists.txt` | Two CMakeLists were defining the same JNI library; consolidated into the AAR module. |
