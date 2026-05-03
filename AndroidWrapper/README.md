# AndroidWrapper

[![Android AAR](https://github.com/Sirokujira/pcl_mobile_framework/actions/workflows/android.yml/badge.svg)](https://github.com/Sirokujira/pcl_mobile_framework/actions/workflows/android.yml)
[![Android Package Release](https://github.com/Sirokujira/pcl_mobile_framework/actions/workflows/android-release.yml/badge.svg)](https://github.com/Sirokujira/pcl_mobile_framework/actions/workflows/android-release.yml)

Android AAR wrapper for `pclmobile`.

## GitHub Actions

Android CI uses two workflows:

- [`Android AAR`](https://github.com/Sirokujira/pcl_mobile_framework/actions/workflows/android.yml)
  verifies the Android wrapper for `arm64-v8a`, `armeabi-v7a`, and `x86_64`.
- [`Android Package Release`](https://github.com/Sirokujira/pcl_mobile_framework/actions/workflows/android-release.yml)
  builds the provider-side release package, uploads the GitHub Release assets,
  and publishes `io.github.sirokujira:pclmobile:<version>` to GitHub Packages.

For manual provider-side publishing, run `Android Package Release` with:

```text
version: 0.1.0
create_release: true
upload_release: true
publish_github_packages: true
```

## Local build

Prerequisites:

- JDK 17
- Android SDK with `ANDROID_HOME` set
- Android NDK `29.0.14206865` or `ANDROID_NDK_VERSION` set to the installed NDK version
- Android SDK CMake `3.31.6` or `ANDROID_CMAKE_VERSION` set to the installed version

Build the release AAR:

```sh
cd AndroidWrapper/aar
ANDROID_ABIS="arm64-v8a armeabi-v7a x86_64" \
ANDROID_NDK_VERSION=29.0.14206865 \
ANDROID_CMAKE_VERSION=3.31.6 \
sh ./gradlew --no-daemon :pclmobile:assembleRelease
```

Build one ABI at a time:

```sh
cd AndroidWrapper/aar
ANDROID_ABIS=arm64-v8a sh ./gradlew --no-daemon :pclmobile:assembleRelease
ANDROID_ABIS=armeabi-v7a sh ./gradlew --no-daemon :pclmobile:assembleRelease
ANDROID_ABIS=x86_64 sh ./gradlew --no-daemon :pclmobile:assembleRelease
```

Generated AAR:

```text
AndroidWrapper/aar/pclmobile/build/outputs/aar/pclmobile-release.aar
```

Check AAR contents:

```sh
unzip -l AndroidWrapper/aar/pclmobile/build/outputs/aar/pclmobile-release.aar
```

## Java / Kotlin call samples

Copyable call samples are generated under:

```text
AndroidWrapper/aar/samples/java/com/pcl_mobile/samples/PclMobileJavaSample.java
AndroidWrapper/aar/samples/kotlin/com/pcl_mobile/samples/PclMobileKotlinSample.kt
```

Both samples write a small PCD file and call PCL-backed `pclmobileJNILib`
functions for:

- PCD I/O: `load(...)`, `getCloudPoints()`
- common geometry: `computeCentroidAndBounds(...)`
- filtering: `filterVoxelGrid(...)`, `filterStatisticalOutlierRemoval(...)`,
  `filterRadiusOutlierRemoval(...)`, `filterCropBox(...)`
- feature estimation: `estimateNormals(...)`
- model fitting / segmentation: `segmentPlane(...)`, `extractPlaneInliers(...)`,
  `segmentSphere(...)`, `extractEuclideanClusters(...)`,
  `projectInliersToPlane(...)`
- search structures: `nearestKSearch(...)`, `octreeRadiusSearch(...)`
- surface / geometry: `computeConvexHull(...)`
- registration: `alignToTranslatedCopyICP(...)`

Point arrays are packed as `x, y, z` float triples. Normal arrays are packed as
`normal_x, normal_y, normal_z, curvature`. Neighbor search arrays are packed as
`x, y, z, squared_distance`. ICP arrays are packed as `has_converged`,
`fitness_score`, then a row-major 4x4 transform matrix.

## Publish to Maven local

```sh
cd AndroidWrapper/aar
ANDROID_ABIS="arm64-v8a armeabi-v7a x86_64" \
ANDROID_NDK_VERSION=29.0.14206865 \
ANDROID_CMAKE_VERSION=3.31.6 \
PCLMOBILE_VERSION=0.1.0 \
sh ./gradlew --no-daemon :pclmobile:publishReleasePublicationToMavenLocal
```

Local Maven output:

```text
~/.m2/repository/io/github/sirokujira/pclmobile/
```

## Package for release

Build the staged native libraries and release AAR from the repository root:

```sh
cd ~/Github/pcl_mobile_framework
ANDROID_NDK_VERSION=29.0.14206865 \
ANDROID_CMAKE_VERSION=3.31.6 \
./scripts/package_android.sh 0.1.0
```

Attach the AAR to the same GitHub Release as the iOS XCFramework:

```sh
cd ~/Github/pcl_mobile_framework
ANDROID_NDK_VERSION=29.0.14206865 \
ANDROID_CMAKE_VERSION=3.31.6 \
./scripts/package_android.sh 0.1.0 --upload-release
```

Create the GitHub Release when needed, upload a versioned AAR, and publish to
GitHub Packages:

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

Publish to a local Maven repository under the module build directory:

```sh
cd ~/Github/pcl_mobile_framework
ANDROID_NDK_VERSION=29.0.14206865 \
ANDROID_CMAKE_VERSION=3.31.6 \
./scripts/package_android.sh 0.1.0 --local-repo
```

Publish to Maven local:

```sh
cd ~/Github/pcl_mobile_framework
ANDROID_NDK_VERSION=29.0.14206865 \
ANDROID_CMAKE_VERSION=3.31.6 \
./scripts/package_android.sh 0.1.0 --maven-local
```

Publish to GitHub Packages:

```sh
cd ~/Github/pcl_mobile_framework
ANDROID_NDK_VERSION=29.0.14206865 \
ANDROID_CMAKE_VERSION=3.31.6 \
PCLMOBILE_GITHUB_REPOSITORY="Sirokujira/pcl_mobile_framework" \
MAVEN_USERNAME="$GITHUB_ACTOR" \
MAVEN_PASSWORD="$GITHUB_TOKEN" \
./scripts/package_android.sh 0.1.0 --publish-github-packages
```

Publish to another Maven-compatible repository:

```sh
cd ~/Github/pcl_mobile_framework
ANDROID_NDK_VERSION=29.0.14206865 \
ANDROID_CMAKE_VERSION=3.31.6 \
MAVEN_USERNAME="$MAVEN_USERNAME" \
MAVEN_PASSWORD="$MAVEN_PASSWORD" \
./scripts/package_android.sh 0.1.0 \
  --maven-repository-url "https://maven.example.com/releases"
```

When a GitHub Release is published, `.github/workflows/android-release.yml`
rebuilds the same multi-ABI AAR, uploads it to that release, and publishes the
matching Maven package to GitHub Packages.

The script also writes a versioned release asset and checksum:

```text
build/android/distributions/pclmobile-0.1.0.aar
build/android/distributions/pclmobile-0.1.0.aar.sha256
```

## Provider-side publishing

Use this command when this repository publishes the Android AAR to GitHub
Release and GitHub Packages:

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

The provider publishes these outputs:

```text
GitHub Release asset: build/android/distributions/pclmobile-0.1.0.aar
GitHub Release checksum: build/android/distributions/pclmobile-0.1.0.aar.sha256
GitHub Packages coordinate: io.github.sirokujira:pclmobile:0.1.0
```

## Consuming from Gradle

The following Gradle snippets are for app projects that depend on the published
package.

GitHub Packages:

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

Local Maven repository under this repo, produced by `--local-repo`:

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

Manual AAR drop-in:

```kotlin
dependencies {
    implementation(files("libs/pclmobile-0.1.0.aar"))
}
```

## Environment variables

- `ANDROID_HOME`: Android SDK path.
- `ANDROID_NDK_VERSION`: NDK version used by Gradle, defaulting to `29.0.14206865`.
- `ANDROID_CMAKE_VERSION`: Android SDK CMake version used by Gradle, defaulting to `3.31.6`.
- `ANDROID_ABIS`: Space-separated ABI list. Defaults to `arm64-v8a armeabi-v7a x86_64`.
- `ANDROID_CLEAN_AFTER_STAGE`: set to `ON` to remove each ABI superbuild tree after staging.
- `PCLMOBILE_VERSION`: Maven version for the generated AAR.
- `MAVEN_REPOSITORY_URL`: remote Maven repository URL, for example GitHub Packages.
- `MAVEN_USERNAME` / `MAVEN_PASSWORD`: remote Maven credentials.
- `PCLMOBILE_GITHUB_REPOSITORY`: `owner/repo` for GitHub Packages and Release uploads.
- `PCLMOBILE_RELEASE_TAG`: release tag, defaulting to `v<PCLMOBILE_VERSION>`.

## Maven Central

Maven Central publishing and signing are intentionally not implemented yet.
TODO: add release-only signing, Sonatype/Maven Central credentials from CI secrets, and a separate publish workflow after artifact metadata is finalized.
