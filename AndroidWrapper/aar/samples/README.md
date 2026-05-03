# pclmobile call samples

This folder contains copyable sample code for calling `pclmobile` from Android
Java and Kotlin code.

## Java

```text
samples/java/com/pcl_mobile/samples/PclMobileJavaSample.java
```

Use from an Android Activity or worker thread:

```java
PclMobileJavaSample.Result result = PclMobileJavaSample.runVoxelGrid(getCacheDir());
```

## Kotlin

```text
samples/kotlin/com/pcl_mobile/samples/PclMobileKotlinSample.kt
```

Use from an Android Activity, ViewModel, or worker thread:

```kotlin
val result = PclMobileKotlinSample.runVoxelGrid(cacheDir)
```

## Covered pclmobile calls

The Java and Kotlin samples cover these PCL-backed calls:

- `load(...)`: PCD I/O, backed by `pcl::io::loadPCDFile`.
- `computeCentroidAndBounds(...)`: common geometry statistics, backed by
  `pcl::compute3DCentroid` and `pcl::getMinMax3D`.
- `filterVoxelGrid(...)`: downsampling, backed by `pcl::VoxelGrid`.
- `filterStatisticalOutlierRemoval(...)`: noisy point filtering, backed by
  `pcl::StatisticalOutlierRemoval`.
- `filterRadiusOutlierRemoval(...)`: neighborhood-density filtering, backed by
  `pcl::RadiusOutlierRemoval`.
- `filterCropBox(...)`: axis-aligned region filtering, backed by
  `pcl::CropBox`.
- `estimateNormals(...)`: feature estimation, backed by
  `pcl::NormalEstimation`.
- `segmentPlane(...)`: model fitting / segmentation, backed by
  `pcl::SACSegmentation` with a plane model and RANSAC.
- `segmentSphere(...)`: model fitting / segmentation, backed by
  `pcl::SACSegmentation` with a sphere model and RANSAC.
- `extractPlaneInliers(...)`: model inlier extraction, backed by
  `pcl::ExtractIndices`.
- `projectInliersToPlane(...)`: model projection, backed by
  `pcl::ProjectInliers`.
- `nearestKSearch(...)`: nearest-neighbor search, backed by
  `pcl::KdTreeFLANN`.
- `octreeRadiusSearch(...)`: spatial radius search, backed by
  `pcl::octree::OctreePointCloudSearch`.
- `extractEuclideanClusters(...)`: cluster segmentation, backed by
  `pcl::EuclideanClusterExtraction`.
- `computeConvexHull(...)`: surface/geometry reconstruction, backed by
  `pcl::ConvexHull`.
- `alignToTranslatedCopyICP(...)`: registration, backed by
  `pcl::IterativeClosestPoint`.

## Real-device instrumented sample

The app module also contains a runnable device test:

```text
app/src/androidTest/java/com/pcl_mobile/PclMobileDeviceSampleTest.java
```

Run it on a connected Android device:

```sh
cd AndroidWrapper/aar
ANDROID_HOME="${ANDROID_HOME:-$HOME/Library/Android/sdk}" \
ANDROID_ABIS=arm64-v8a \
ANDROID_NDK_VERSION="${ANDROID_NDK_VERSION:-29.0.14206865}" \
ANDROID_CMAKE_VERSION="${ANDROID_CMAKE_VERSION:-3.31.6}" \
sh ./gradlew --no-daemon :app:connectedDebugAndroidTest
```

Success means the test passes and logcat contains `PclMobileDeviceSampleTest:
device sample passed`, with non-zero counts for the generated PCD, VoxelGrid
result, normals, model segmentation, search, hull, projection, filter, and ICP
calls.

## Notes

- These samples call the current Java wrapper class:
  `com.sirokujira.pclmobile.pclmobileJNILib`.
- Run the examples away from the UI thread if the input point cloud is large.
- Point arrays are packed as `x, y, z, x, y, z, ...`.
- Neighbor search arrays are packed as `x, y, z, squared_distance, ...`.
- Normal arrays are packed as `normal_x, normal_y, normal_z, curvature, ...`.
- Plane model arrays are packed as `a, b, c, d, inlier_count, input_count`.
- Sphere model arrays are packed as `center_x, center_y, center_z, radius,
  inlier_count, input_count`.
- Centroid/bounds arrays are packed as `centroid_x, centroid_y, centroid_z,
  min_x, min_y, min_z, max_x, max_y, max_z, point_count`.
- ICP arrays are packed as `has_converged, fitness_score, row-major 4x4 matrix`.
- Cluster arrays contain one cluster size per element.
