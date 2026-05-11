# Flutter pclMobile sample

This folder contains a Flutter plugin sample for calling `pclMobile` on both
Android and iOS through a `MethodChannel`.

The sample intentionally uses the existing native wrappers:

- Android: `com.sirokujira.pclmobile.pclmobileJNILib`
- iOS: `PCLMobile.framework` / `PointCloud`

## Files

```text
lib/pcl_mobile.dart              # Dart facade and sample PCD generator
lib/pcl_mobile_flutter.dart      # Public export file
android/src/main/kotlin/...      # Android MethodChannel bridge
ios/Classes/...                  # iOS MethodChannel bridge
example/lib/main.dart            # Minimal Flutter UI smoke sample
pubspec.yaml                     # Example app dependencies
```

## Android dependency

The plugin Android target consumes the local Maven artifact from:

```text
AndroidWrapper/aar/pclmobile/build/repo
```

Build or publish the Android AAR before running the Flutter Android example:

```sh
cd /path/to/pcl_mobile_framework
ANDROID_ABIS="arm64-v8a armeabi-v7a x86_64" \
ANDROID_NDK_VERSION=29.0.14206865 \
ANDROID_CMAKE_VERSION=3.31.6 \
./scripts/package_android.sh 0.1.0 --local-repo
```

The plugin already wires that repository and dependency in:

```text
android/build.gradle
```

## iOS dependency

The plugin iOS podspec consumes the local XCFramework from:

```text
build/ios/xcframework/PCLMobile.xcframework
```

Build the iOS XCFramework before running the Flutter iOS example:

```sh
cd /path/to/pcl_mobile_framework
./scripts/build_ios.sh
```

## Covered calls

The sample covers the same minimal workflow on both platforms:

- Generate a small ASCII PCD file in the app temporary directory.
- `load(path)`
- `getCloudPoints()`
- `computeCentroidAndBounds()`
- `filterVoxelGrid(x, y, z)`
- `getFilteredPoints()`

For larger point clouds, keep heavy PCL calls away from the UI thread and avoid
returning huge point arrays through `MethodChannel`. Use a file-backed or FFI
path for high-volume point transfer later.

## Run

With Flutter installed:

```sh
cd FlutterWrapper/pcl_mobile_sample/example
flutter create . --platforms=android,ios
flutter pub get
flutter run
```

`flutter create .` generates only the host Android/iOS example project files.
The plugin implementation lives one directory above, under
`FlutterWrapper/pcl_mobile_sample`.
