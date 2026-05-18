/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.sirokujira.pclmobile;

/**
 * Java wrapper for the pclmobile native library.
 *
 * <p>The wrapper keeps one native source point cloud and one filtered point
 * cloud in process memory. Call {@link #load(String)} before running filters,
 * segmentation, search, surface, or registration operations. Methods returning
 * {@code float[]} use compact tuple packing to avoid Java object allocation in
 * the JNI boundary.</p>
 *
 * <p>Tuple formats:</p>
 * <ul>
 *     <li>Point clouds: {@code x, y, z, x, y, z, ...}</li>
 *     <li>Normals: {@code normal_x, normal_y, normal_z, curvature, ...}</li>
 *     <li>PFH descriptors: 125-bin histogram per input point</li>
 *     <li>FPFH descriptors: 33-bin histogram per input point</li>
 *     <li>VFH descriptors: one 308-bin histogram for the active cloud</li>
 *     <li>ESF descriptors: one 640-bin histogram for the active cloud</li>
 *     <li>GASD descriptors: one 512-bin histogram for the active cloud</li>
 *     <li>CRH descriptors: one 90-bin histogram for the active cloud</li>
 *     <li>CVFH descriptors: 308-bin histogram per stable cluster</li>
 *     <li>OUR-CVFH descriptors: 308-bin histogram per stable cluster orientation</li>
 *     <li>Intensity spin descriptors: 20-bin histogram per input point</li>
 *     <li>3D shape context descriptors: 1980-bin descriptor per input point</li>
 *     <li>Unique shape context descriptors: 1960-bin descriptor per input point</li>
 *     <li>Spin image descriptors: 153-bin histogram per input point</li>
 *     <li>GRSD descriptors: one or more 21-bin histograms for the active cloud</li>
 *     <li>SHOT descriptors: 352-bin histogram per input point</li>
 *     <li>Moment invariants: {@code j1, j2, j3, ...}</li>
 *     <li>RSD descriptors: {@code r_min, r_max, ...}</li>
 *     <li>Principal curvatures: {@code principal_curvature_x/y/z, pc1, pc2, ...}</li>
 *     <li>Boundary points: {@code x, y, z, boundary_flag, ...}</li>
     *     <li>Plane model: {@code a, b, c, d, inlier_count, input_count}</li>
     *     <li>Sphere model: {@code center_x, center_y, center_z, radius, inlier_count, input_count}</li>
     *     <li>SAC inlier indices: native point indices as {@code int[]}</li>
     *     <li>Nearest/radius/voxel search: {@code x, y, z, squared_distance, ...}</li>
     *     <li>Search indices: {@code point_index, squared_distance, ...}</li>
 *     <li>SIFT/Harris/SUSAN/Trajkovic keypoints: {@code x, y, z, response, ...}</li>
 *     <li>Range image points: {@code x, y, z, range, ...}</li>
 *     <li>Hull and smoothed clouds: {@code x, y, z, x, y, z, ...}</li>
 *     <li>Centroid/bounds: {@code centroid_xyz, min_xyz, max_xyz, point_count}</li>
 *     <li>Covariance: {@code centroid_xyz, row-major 3x3 covariance, point_count}</li>
 *     <li>PCA axes: {@code mean_xyz, eigenvalues_xyz, row-major 3x3 eigenvectors, point_count}</li>
 *     <li>Distances: {@code squared_distance, squared_distance, ...}</li>
     *     <li>Rigid transform matrix: row-major 4x4 matrix</li>
     *     <li>ICP: {@code has_converged, fitness_score, row-major 4x4 matrix}</li>
     *     <li>NDT: {@code has_converged, fitness_score, row-major 4x4 matrix}</li>
 * </ul>
 */
public final class pclmobileJNILib {
    public static final int SACMODEL_PLANE = 0;
    public static final int SACMODEL_LINE = 1;
    public static final int SACMODEL_CIRCLE2D = 2;
    public static final int SACMODEL_CIRCLE3D = 3;
    public static final int SACMODEL_SPHERE = 4;
    public static final int SACMODEL_CYLINDER = 5;
    public static final int SACMODEL_CONE = 6;
    public static final int SACMODEL_TORUS = 7;
    public static final int SACMODEL_PARALLEL_LINE = 8;
    public static final int SACMODEL_PERPENDICULAR_PLANE = 9;
    public static final int SACMODEL_PARALLEL_LINES = 10;
    public static final int SACMODEL_NORMAL_PLANE = 11;
    public static final int SACMODEL_NORMAL_SPHERE = 12;
    public static final int SACMODEL_REGISTRATION = 13;
    public static final int SACMODEL_REGISTRATION_2D = 14;
    public static final int SACMODEL_PARALLEL_PLANE = 15;
    public static final int SACMODEL_NORMAL_PARALLEL_PLANE = 16;
    public static final int SACMODEL_STICK = 17;
    public static final int SACMODEL_ELLIPSE3D = 18;
    public static final int SAC_RANSAC = 0;
    public static final int SAC_LMEDS = 1;
    public static final int SAC_MSAC = 2;
    public static final int SAC_RRANSAC = 3;
    public static final int SAC_RMSAC = 4;
    public static final int SAC_MLESAC = 5;
    public static final int SAC_PROSAC = 6;
    public static final int HARRIS_RESPONSE_HARRIS = 1;
    public static final int HARRIS_RESPONSE_NOBLE = 2;
    public static final int HARRIS_RESPONSE_LOWE = 3;
    public static final int HARRIS_RESPONSE_TOMASI = 4;
    public static final int HARRIS_RESPONSE_CURVATURE = 5;
    public static final int TRAJKOVIC_METHOD_FOUR_CORNERS = 0;
    public static final int TRAJKOVIC_METHOD_EIGHT_CORNERS = 1;
    public static final int MORPH_OPEN = 0;
    public static final int MORPH_CLOSE = 1;
    public static final int MORPH_DILATE = 2;
    public static final int MORPH_ERODE = 3;

    static {
        System.loadLibrary("native-lib");
    }

    private pclmobileJNILib() {
    }

    /**
     * Reserved hook for future native renderer initialization.
     *
     * @param width the current view width in pixels
     * @param height the current view height in pixels
     */
    public static native void init(int width, int height);

    /**
     * Reserved hook for future native renderer frame updates.
     */
    public static native void step();

    /**
     * Loads a PCD file into the native source cloud and clears the filtered cloud.
     *
     * @param filename absolute or app-private path to an ASCII/Binary PCD file readable by PCL
     */
    public static native void load(String filename);

    /**
     * Loads a PLY file into the native source cloud and clears the filtered cloud.
     */
    public static native void loadPLY(String filename);

    /**
     * Replaces the native source cloud from packed {@code x, y, z} triples.
     *
     * <p>Trailing values are ignored when {@code packedXYZ.length} is not divisible by 3.</p>
     */
    public static native void setCloudPoints(float[] packedXYZ);

    /**
     * Returns the loaded source cloud packed as {@code x, y, z} triples.
     */
    public static native float[] getCloudPoints();

    /**
     * Returns the most recent filtered cloud packed as {@code x, y, z} triples.
     */
    public static native float[] getFilteredPoints();

    /**
     * Returns the active cloud point count.
     */
    public static native int getPointCount();

    /**
     * Returns the active cloud width.
     */
    public static native int getWidth();

    /**
     * Returns the active cloud height.
     */
    public static native int getHeight();

    /**
     * Returns {@code point_count, width, height} for the active cloud.
     */
    public static native int[] getCloudShape();

    /**
     * Returns {@code point_count, width, height} for the source cloud.
     */
    public static native int[] getSourceCloudShape();

    /**
     * Returns {@code point_count, width, height} for the filtered cloud.
     */
    public static native int[] getFilteredCloudShape();

    /**
     * Writes the active cloud to an ASCII PCD file.
     */
    public static native boolean writeActivePCDFile(String filename);

    /**
     * Writes the source cloud to an ASCII PCD file.
     */
    public static native boolean writeSourcePCDFile(String filename);

    /**
     * Writes the filtered cloud to an ASCII PCD file.
     */
    public static native boolean writeFilteredPCDFile(String filename);

    /**
     * Writes the active cloud to an ASCII PLY file.
     */
    public static native boolean writeActivePLYFile(String filename);

    /**
     * Writes the source cloud to an ASCII PLY file.
     */
    public static native boolean writeSourcePLYFile(String filename);

    /**
     * Writes the filtered cloud to an ASCII PLY file.
     */
    public static native boolean writeFilteredPLYFile(String filename);

    /**
     * Computes centroid, min bounds, max bounds, and point count for the active cloud.
     *
     * <p>The active cloud is the filtered cloud when non-empty, otherwise the source cloud.</p>
     */
    public static native float[] computeCentroidAndBounds();

    /**
     * Computes normalized covariance for the active cloud.
     *
     * @return {@code centroid_x, centroid_y, centroid_z, row-major 3x3 covariance, point_count}
     */
    public static native float[] computeCovarianceMatrix();

    /**
     * Computes PCA mean, eigenvalues, and eigenvectors for the active cloud.
     *
     * @return {@code mean_xyz, eigenvalues_xyz, row-major 3x3 eigenvectors, point_count}
     */
    public static native float[] computePrincipalAxes();

    /**
     * Computes moment-of-inertia descriptors plus AABB/OBB for the active cloud.
     *
     * @return moments, eccentricities, AABB min/max, OBB min/max/position, row-major OBB rotation, point_count
     */
    public static native float[] computeMomentOfInertiaAndOBB();

    /**
     * Computes squared Euclidean distance from each active-cloud point to a query point.
     *
     * @return one squared distance per active-cloud point
     */
    public static native float[] computeSquaredDistancesToPoint(float x, float y, float z);

    /**
     * Finds the active-cloud point farthest from the computed centroid.
     *
     * @return {@code centroid_xyz, farthest_point_xyz, distance, point_count}
     */
    public static native float[] computeMaxDistanceFromCentroid();

    /**
     * Subtracts the active-cloud centroid from each point.
     *
     * @return demeaned points packed as {@code x, y, z} triples
     */
    public static native float[] demeanActiveCloud();

    /**
     * Estimates normals for the active cloud using KdTree-backed neighborhood search.
     *
     * @param kSearch number of nearest neighbors used for each normal estimate
     * @return {@code normal_x, normal_y, normal_z, curvature} tuples
     */
    public static native float[] estimateNormals(int kSearch);

    /**
     * Estimates normals for the active cloud using all neighbors within {@code radiusSearch}.
     *
     * @return {@code normal_x, normal_y, normal_z, curvature} tuples
     */
    public static native float[] estimateNormalsRadius(double radiusSearch);

    /**
     * Estimates normals for the active cloud with PCL NormalEstimationOMP.
     *
     * @return {@code normal_x, normal_y, normal_z, curvature} tuples
     */
    public static native float[] estimateNormalsOMP(int kSearch, int numberOfThreads);

    /**
     * Refines estimated normals with PCL NormalRefinement.
     *
     * @return {@code normal_x, normal_y, normal_z, curvature} tuples
     */
    public static native float[] refineNormals(
            int normalKSearch,
            int refinementKSearch,
            int maxIterations,
            double convergenceThreshold);

    /**
     * Computes Point Feature Histograms for the active cloud.
     *
     * @param normalKSearch nearest neighbors used to estimate normals before descriptor computation
     * @param featureRadius radius used for each PFH descriptor neighborhood
     * @return 125 histogram values per descriptor
     */
    public static native float[] computePFHFeatures(int normalKSearch, double featureRadius);

    /**
     * Computes Fast Point Feature Histograms for the active cloud.
     *
     * @param normalKSearch nearest neighbors used to estimate normals before descriptor computation
     * @param featureRadius radius used for each FPFH descriptor neighborhood
     * @return 33 histogram values per descriptor
     */
    public static native float[] computeFPFHFeatures(int normalKSearch, double featureRadius);

    /**
     * Computes FPFH descriptors for the active cloud with PCL FPFHEstimationOMP.
     *
     * @return 33 histogram values per descriptor
     */
    public static native float[] computeFPFHFeaturesOMP(
            int normalKSearch,
            double featureRadius,
            int numberOfThreads);

    /**
     * Computes a Viewpoint Feature Histogram for the active cloud.
     *
     * @param normalKSearch nearest neighbors used to estimate normals before descriptor computation
     * @return one 308-bin histogram for the active cloud
     */
    public static native float[] computeVFHFeatures(int normalKSearch);

    /**
     * Computes an Ensemble of Shape Functions descriptor for the active cloud.
     *
     * @return one 640-bin histogram for the active cloud
     */
    public static native float[] computeESFDescriptor();

    /**
     * Computes a Globally Aligned Spatial Distribution descriptor for the active cloud.
     *
     * @return one 512-bin histogram for the active cloud
     */
    public static native float[] computeGASDDescriptor();

    /**
     * Computes a Camera Roll Histogram descriptor for the active cloud.
     *
     * @return one 90-bin histogram for the active cloud
     */
    public static native float[] computeCRHDescriptor(
            int normalKSearch,
            float viewpointX,
            float viewpointY,
            float viewpointZ);

    /**
     * Computes Clustered Viewpoint Feature Histograms for the active cloud.
     *
     * <p>Pass {@code 0.0} or {@code 0} for optional numeric parameters to keep PCL defaults,
     * except {@code normalKSearch}, which must be positive.</p>
     *
     * @return 308 histogram values per stable cluster descriptor
     */
    public static native float[] computeCVFHFeatures(
            int normalKSearch,
            double clusterTolerance,
            double epsAngleThreshold,
            double curvatureThreshold,
            int minPoints,
            boolean normalizeBins);

    /**
     * Computes Oriented, Unique and Repeatable CVFH descriptors for the active cloud.
     *
     * <p>Pass {@code 0.0} or {@code 0} for optional numeric parameters to keep PCL defaults,
     * except {@code normalKSearch}, which must be positive.</p>
     *
     * @return 308 histogram values per stable cluster orientation descriptor
     */
    public static native float[] computeOURCVFHFeatures(
            int normalKSearch,
            double clusterTolerance,
            double epsAngleThreshold,
            double curvatureThreshold,
            int minPoints,
            boolean normalizeBins,
            double refineClusters,
            double axisRatio,
            double minAxisValue);

    /**
     * Computes intensity-domain spin image descriptors for the active cloud.
     *
     * <p>The wrapper derives an intensity field from XYZ distance and uses a fixed
     * 4x5 bin layout.</p>
     *
     * @return 20 histogram values per descriptor
     */
    public static native float[] computeIntensitySpinFeatures(
            double radiusSearch,
            double smoothingBandwidth);

    /**
     * Computes IntensityGradient descriptors for the active cloud.
     *
     * <p>The wrapper derives an intensity field from XYZ distance before estimating gradients.</p>
     *
     * @return {@code gradient_x, gradient_y, gradient_z} tuples
     */
    public static native float[] computeIntensityGradientFeatures(
            int normalKSearch,
            double radiusSearch,
            int numberOfThreads);

    /**
     * Computes RIFT descriptors for the active cloud.
     *
     * <p>The wrapper derives an intensity field from XYZ distance and uses the fixed
     * PCL {@code Histogram<32>} layout: 4 distance bins by 8 gradient bins.</p>
     *
     * @return 32 histogram values per descriptor
     */
    public static native float[] computeRIFTFeatures(
            int normalKSearch,
            double gradientRadius,
            double featureRadius);

    /**
     * Computes RoPS descriptors for the active cloud.
     *
     * <p>The wrapper estimates normals, builds a GP3 triangle mesh, and uses the fixed
     * PCL {@code Histogram<135>} layout.</p>
     *
     * @return 135 histogram values per descriptor
     */
    public static native float[] computeROPSFeatures(
            int normalKSearch,
            double supportRadius,
            double meshSearchRadius,
            double mu,
            int maximumNearestNeighbors);

    /**
     * Computes PCL CPPF pair features for two active-cloud point indices.
     *
     * <p>The wrapper estimates normals from the active cloud and uses neutral synthetic colors.</p>
     *
     * @return {@code f1..f10}, or an empty array when the indices are invalid
     */
    public static native float[] computeCPPFPairFeature(
            int firstIndex,
            int secondIndex,
            int normalKSearch);

    /**
     * Computes 3D shape context descriptors for the active cloud.
     *
     * @return 1980 descriptor values per input point
     */
    public static native float[] computeShapeContext3DFeatures(
            int normalKSearch,
            double searchRadius,
            double minRadius,
            double pointDensityRadius,
            boolean random);

    /**
     * Computes Unique Shape Context descriptors for the active cloud.
     *
     * @return 1960 descriptor values per input point
     */
    public static native float[] computeUniqueShapeContextFeatures(
            double searchRadius,
            double minRadius,
            double pointDensityRadius,
            double localRadius);

    /**
     * Computes Point Pair Feature descriptors for the active cloud.
     *
     * <p>PPF produces one descriptor per point pair. To avoid excessive memory use, returns an empty
     * array when the active cloud has more than {@code maxPointCount} points.</p>
     *
     * @return {@code f1, f2, f3, f4, alpha_m} tuples
     */
    public static native float[] computePPFFeatures(int normalKSearch, int maxPointCount);

    /**
     * Computes Normal Based Signature descriptors for the active cloud.
     *
     * <p>The wrapper returns {@code pcl::NormalBasedSignature12} values, so N' and M' are fixed to
     * {@code 4} and {@code 3} while {@code n} and {@code m} control the source transform grid.</p>
     *
     * @return 12 descriptor values per input point
     */
    public static native float[] computeNormalBasedSignatureFeatures(
            int normalKSearch,
            double searchRadius,
            double scale,
            int n,
            int m);

    /**
     * Computes Spin Image descriptors for the active cloud.
     *
     * <p>The wrapper uses {@code pcl::Histogram<153>}; image widths larger than PCL can pack into
     * 153 bins are clamped by PCL.</p>
     *
     * @return 153 histogram values per descriptor
     */
    public static native float[] computeSpinImageFeatures(
            int normalKSearch,
            double featureRadius,
            int imageWidth,
            double supportAngleCos,
            int minPointCount);

    /**
     * Computes Global Radius-based Surface Descriptors for the active cloud.
     *
     * @return 21 histogram values per descriptor
     */
    public static native float[] computeGRSDDescriptor(
            int normalKSearch,
            double radiusSearch,
            double planeRadius,
            int subdivisions);

    /**
     * Computes moment invariants for the active cloud.
     *
     * @return {@code j1, j2, j3} tuples
     */
    public static native float[] computeMomentInvariants(double radiusSearch);

    /**
     * Computes Radius-based Surface Descriptor features for the active cloud.
     *
     * @return {@code r_min, r_max} tuples
     */
    public static native float[] computeRSDFeatures(
            int normalKSearch,
            double radiusSearch,
            double planeRadius,
            int subdivisions);

    /**
     * Computes principal curvatures for the active cloud.
     *
     * @param normalKSearch nearest neighbors used to estimate normals before curvature computation
     * @param curvatureKSearch nearest neighbors used for principal curvature estimation
     * @return {@code principal_curvature_x, principal_curvature_y, principal_curvature_z, pc1, pc2} tuples
     */
    public static native float[] computePrincipalCurvatures(int normalKSearch, int curvatureKSearch);

    /**
     * Computes Signature of Histograms of Orientations descriptors for the active cloud.
     *
     * @return 352 histogram values per descriptor
     */
    public static native float[] computeSHOTFeatures(int normalKSearch, double featureRadius);

    /**
     * Computes SHOT descriptors for the active cloud with PCL SHOTEstimationOMP.
     *
     * @return 352 histogram values per descriptor
     */
    public static native float[] computeSHOTFeaturesOMP(
            int normalKSearch,
            double featureRadius,
            int numberOfThreads);

    /**
     * Computes SHOT local reference frames for the active cloud.
     *
     * @return 9 reference-frame matrix values per input point
     */
    public static native float[] computeSHOTLocalReferenceFrames(
            double radiusSearch,
            boolean useOmp,
            int numberOfThreads);

    /**
     * Computes BOARD local reference frames for the active cloud.
     *
     * @return 9 reference-frame matrix values per input point
     */
    public static native float[] computeBOARDLocalReferenceFrames(
            int normalKSearch,
            double radiusSearch,
            double tangentRadius,
            boolean findHoles,
            double marginThreshold);

    /**
     * Computes FLARE local reference frames for the active cloud.
     *
     * @return 9 reference-frame matrix values per input point
     */
    public static native float[] computeFLARELocalReferenceFrames(
            int normalKSearch,
            double radiusSearch,
            double tangentRadius,
            double marginThreshold,
            int minNeighborsForNormalAxis,
            int minNeighborsForTangentAxis);

    /**
     * Computes PCL BoundaryEstimation flags for the active cloud.
     *
     * @return {@code x, y, z, boundary_flag} tuples
     */
    public static native float[] computeBoundaryPoints(
            int normalKSearch,
            double radiusSearch,
            double angleThresholdDegrees);

    /**
     * Computes Difference of Normals vectors for the active cloud.
     *
     * @return {@code normal_x, normal_y, normal_z, curvature} tuples
     */
    public static native float[] computeDifferenceOfNormals(double smallRadius, double largeRadius);

    /**
     * Fits a plane model to the active cloud using RANSAC sample consensus.
     *
     * @param distanceThreshold maximum point-to-model distance for inliers
     * @param maxIterations maximum RANSAC iterations
     * @return {@code a, b, c, d, inlier_count, input_count}, or an empty array on failure
     */
    public static native float[] segmentPlane(double distanceThreshold, int maxIterations);

    /**
     * Fits a sphere model to the active cloud using RANSAC sample consensus.
     *
     * @param distanceThreshold maximum point-to-model distance for inliers
     * @param maxIterations maximum RANSAC iterations
     * @return {@code center_x, center_y, center_z, radius, inlier_count, input_count}, or an empty array
     */
    public static native float[] segmentSphere(double distanceThreshold, int maxIterations);

    /**
     * Fits a PCL SAC model to the active cloud using RANSAC sample consensus.
     *
     * <p>{@code modelType} uses the {@code SACMODEL_*} constants exposed by this class.</p>
     *
     * @return model coefficients followed by {@code inlier_count, input_count}, or an empty array
     */
    public static native float[] segmentSACModel(int modelType, double distanceThreshold, int maxIterations);

    /**
     * Fits a PCL SAC model to the active cloud with a selected sample-consensus method.
     *
     * <p>{@code modelType} uses {@code SACMODEL_*}; {@code methodType} uses {@code SAC_*}.</p>
     *
     * @return model coefficients followed by {@code inlier_count, input_count}, or an empty array
     */
    public static native float[] segmentSACModelWithMethod(
            int modelType,
            int methodType,
            double distanceThreshold,
            int maxIterations);

    /**
     * Returns active-cloud point indices that inlie a fitted PCL SAC model.
     *
     * <p>{@code modelType} uses the {@code SACMODEL_*} constants exposed by this class.</p>
     */
    public static native int[] segmentSACModelInlierIndices(
            int modelType,
            double distanceThreshold,
            int maxIterations);

    /**
     * Runs nearest-neighbor search against the active cloud.
     *
     * @return {@code x, y, z, squared_distance} tuples for found neighbors
     */
    public static native float[] nearestKSearch(float x, float y, float z, int k);

    /**
     * Runs nearest-neighbor search and returns active-cloud indices.
     *
     * @return {@code point_index, squared_distance} tuples for found neighbors
     */
    public static native float[] nearestKSearchIndices(float x, float y, float z, int k);

    /**
     * Runs KdTree radius search against the active cloud.
     *
     * @return {@code x, y, z, squared_distance} tuples for found neighbors
     */
    public static native float[] radiusSearch(float x, float y, float z, double radius);

    /**
     * Runs KdTree radius search and returns active-cloud indices.
     *
     * @return {@code point_index, squared_distance} tuples for found neighbors
     */
    public static native float[] radiusSearchIndices(float x, float y, float z, double radius);

    /**
     * Runs KdTree radius search with PCL's {@code max_nn} result bound.
     *
     * <p>Pass {@code 0} or a negative value for {@code maxNeighbors} to keep PCL's unbounded behavior.</p>
     *
     * @return {@code x, y, z, squared_distance} tuples for found neighbors
     */
    public static native float[] radiusSearchLimited(
            float x, float y, float z, double radius, int maxNeighbors);

    /**
     * Runs bounded KdTree radius search and returns active-cloud indices.
     *
     * @return {@code point_index, squared_distance} tuples for found neighbors
     */
    public static native float[] radiusSearchIndicesLimited(
            float x, float y, float z, double radius, int maxNeighbors);

    /**
     * Runs octree nearest-neighbor search against the active cloud.
     *
     * @param resolution octree voxel resolution
     * @return {@code x, y, z, squared_distance} tuples for found neighbors
     */
    public static native float[] octreeNearestKSearch(float x, float y, float z, double resolution, int k);

    /**
     * Runs octree nearest-neighbor search and returns active-cloud indices.
     *
     * @return {@code point_index, squared_distance} tuples for found neighbors
     */
    public static native float[] octreeNearestKSearchIndices(float x, float y, float z, double resolution, int k);

    /**
     * Runs octree radius search against the active cloud.
     *
     * @param resolution octree voxel resolution
     * @param radius search radius around {@code x/y/z}
     * @return {@code x, y, z, squared_distance} tuples for found neighbors
     */
    public static native float[] octreeRadiusSearch(float x, float y, float z, double resolution, double radius);

    /**
     * Runs octree radius search and returns active-cloud indices.
     *
     * @return {@code point_index, squared_distance} tuples for found neighbors
     */
    public static native float[] octreeRadiusSearchIndices(float x, float y, float z, double resolution, double radius);

    /**
     * Runs octree radius search with PCL's {@code max_nn} result bound.
     *
     * <p>Pass {@code 0} or a negative value for {@code maxNeighbors} to keep PCL's unbounded behavior.</p>
     *
     * @return {@code x, y, z, squared_distance} tuples for found neighbors
     */
    public static native float[] octreeRadiusSearchLimited(
            float x, float y, float z, double resolution, double radius, int maxNeighbors);

    /**
     * Runs bounded octree radius search and returns active-cloud indices.
     *
     * @return {@code point_index, squared_distance} tuples for found neighbors
     */
    public static native float[] octreeRadiusSearchIndicesLimited(
            float x, float y, float z, double resolution, double radius, int maxNeighbors);

    /**
     * Returns active-cloud points that share the query point's octree voxel.
     *
     * @param resolution octree voxel resolution
     * @return {@code x, y, z, squared_distance_from_query} tuples for found points
     */
    public static native float[] octreeVoxelSearch(float x, float y, float z, double resolution);

    /**
     * Returns active-cloud indices that share the query point's octree voxel.
     *
     * @return {@code point_index, squared_distance_from_query} tuples for found points
     */
    public static native float[] octreeVoxelSearchIndices(float x, float y, float z, double resolution);

    /**
     * Runs PCL octree approximate nearest-neighbor search against the active cloud.
     *
     * @return one {@code x, y, z, squared_distance} tuple, or an empty array when unavailable
     */
    public static native float[] octreeApproxNearestSearch(float x, float y, float z, double resolution);

    /**
     * Runs octree approximate nearest-neighbor search and returns the active-cloud index.
     *
     * @return one {@code point_index, squared_distance} tuple, or an empty array when unavailable
     */
    public static native float[] octreeApproxNearestSearchIndex(float x, float y, float z, double resolution);

    /**
     * Extracts Euclidean clusters from the active cloud.
     *
     * @return one cluster size per element
     */
    public static native float[] extractEuclideanClusters(double tolerance, int minClusterSize, int maxClusterSize);

    /**
     * Extracts RegionGrowing clusters from the active cloud.
     *
     * @return one cluster size per element
     */
    public static native float[] extractRegionGrowingClusters(
            int normalKSearch,
            int numberOfNeighbours,
            int minClusterSize,
            int maxClusterSize,
            double smoothnessThresholdDegrees,
            double curvatureThreshold);

    /**
     * Extracts ConditionalEuclideanClustering clusters from the active cloud.
     *
     * <p>The Java-facing condition keeps neighboring points when their z-value difference is at most
     * {@code maxZDelta}.</p>
     *
     * @return one cluster size per element
     */
    public static native float[] extractConditionalEuclideanClusters(
            double tolerance,
            int minClusterSize,
            int maxClusterSize,
            double maxZDelta);

    /**
     * Runs PCL ExtractPolygonalPrismData using a packed planar hull and stores the selected points.
     *
     * <p>The planar hull must contain at least three {@code x, y, z} point tuples. Set {@code negative}
     * to keep points outside the prism instead of inside it.</p>
     *
     * @return selected points packed as {@code x, y, z} triples
     */
    public static native float[] extractPolygonalPrismData(
            float[] packedPlanarHullXYZ,
            double heightMin,
            double heightMax,
            float viewPointX,
            float viewPointY,
            float viewPointZ,
            boolean negative);

    /**
     * Runs PCL ProgressiveMorphologicalFilter and stores ground or non-ground points.
     *
     * <p>Set {@code negative} to keep non-ground points instead of ground points.</p>
     */
    public static native void extractProgressiveMorphologicalGround(
            int maxWindowSize,
            double slope,
            double initialDistance,
            double maxDistance,
            double cellSize,
            double base,
            boolean exponential,
            boolean negative);

    /**
     * Runs ProgressiveMorphologicalFilter and returns the selected points.
     */
    public static float[] progressiveMorphologicalGround(
            int maxWindowSize,
            double slope,
            double initialDistance,
            double maxDistance,
            double cellSize,
            double base,
            boolean exponential,
            boolean negative) {
        extractProgressiveMorphologicalGround(
                maxWindowSize, slope, initialDistance, maxDistance, cellSize, base, exponential, negative);
        return getFilteredPoints();
    }

    /**
     * Runs PCL ApproximateProgressiveMorphologicalFilter and stores ground or non-ground points.
     *
     * <p>Set {@code negative} to keep non-ground points instead of ground points.</p>
     */
    public static native void extractApproximateProgressiveMorphologicalGround(
            int maxWindowSize,
            double slope,
            double initialDistance,
            double maxDistance,
            double cellSize,
            double base,
            boolean exponential,
            int numberOfThreads,
            boolean negative);

    /**
     * Runs ApproximateProgressiveMorphologicalFilter and returns the selected points.
     */
    public static float[] approximateProgressiveMorphologicalGround(
            int maxWindowSize,
            double slope,
            double initialDistance,
            double maxDistance,
            double cellSize,
            double base,
            boolean exponential,
            int numberOfThreads,
            boolean negative) {
        extractApproximateProgressiveMorphologicalGround(
                maxWindowSize, slope, initialDistance, maxDistance, cellSize, base,
                exponential, numberOfThreads, negative);
        return getFilteredPoints();
    }

    /**
     * Extracts the largest Euclidean cluster from the active cloud into the filtered cloud.
     */
    public static native void extractLargestEuclideanCluster(
            double tolerance,
            int minClusterSize,
            int maxClusterSize);

    /**
     * Extracts the largest Euclidean cluster and returns the filtered points.
     */
    public static float[] extractLargestEuclideanClusterPoints(
            double tolerance,
            int minClusterSize,
            int maxClusterSize) {
        extractLargestEuclideanCluster(tolerance, minClusterSize, maxClusterSize);
        return getFilteredPoints();
    }

    /**
     * Runs MinCutSegmentation using packed foreground seed points and stores the largest foreground cluster.
     */
    public static native void extractMinCutForeground(
            float[] packedForegroundXYZ,
            double sigma,
            double radius,
            double sourceWeight,
            int numberOfNeighbours);

    /**
     * Runs MinCutSegmentation and returns the filtered foreground points.
     */
    public static float[] extractMinCutForegroundPoints(
            float[] packedForegroundXYZ,
            double sigma,
            double radius,
            double sourceWeight,
            int numberOfNeighbours) {
        extractMinCutForeground(packedForegroundXYZ, sigma, radius, sourceWeight, numberOfNeighbours);
        return getFilteredPoints();
    }

    /**
     * Runs MinCutSegmentation and returns {@code output_count, input_count, foreground_seed_count}.
     */
    public static native float[] extractMinCutForegroundStats(
            float[] packedForegroundXYZ,
            double sigma,
            double radius,
            double sourceWeight,
            int numberOfNeighbours);

    /**
     * Segments points from the active cloud that differ from {@code packedTargetXYZ}.
     *
     * @return difference points packed as {@code x, y, z} triples
     */
    public static native float[] segmentDifferencesAgainstTarget(
            float[] packedTargetXYZ,
            double distanceThreshold);

    /**
     * Computes Intrinsic Shape Signatures 3D keypoints for the active cloud.
     *
     * @return keypoint positions packed as {@code x, y, z} triples
     */
    public static native float[] computeISSKeypoints(
            double salientRadius,
            double nonMaxRadius,
            double threshold21,
            double threshold32,
            int minNeighbors);

    /**
     * Computes SIFT keypoints for the active cloud.
     *
     * <p>The wrapper stores XYZ points only, while PCL SIFT expects an intensity field.
     * This method derives intensity from distance to the origin before invoking PCL
     * {@code SIFTKeypoint}, so results depend on the cloud coordinate frame.</p>
     *
     * @return {@code x, y, z, scale} tuples
     */
    public static native float[] computeSIFTKeypoints(
            double minScale,
            int nrOctaves,
            int nrScalesPerOctave,
            double minContrast);

    /**
     * Computes Harris 3D keypoints for the active cloud.
     *
     * @param responseMethod one of the {@code HARRIS_RESPONSE_*} constants
     * @return {@code x, y, z, response} tuples
     */
    public static native float[] computeHarrisKeypoints(
            int responseMethod,
            double radius,
            double threshold,
            boolean nonMaxSuppression,
            boolean refine);

    /**
     * Computes Harris 2D keypoints for the active organized cloud.
     *
     * <p>The wrapper derives an intensity field from XYZ distance before invoking
     * PCL {@code HarrisKeypoint2D}. {@code responseMethod} accepts
     * {@code HARRIS_RESPONSE_HARRIS}, {@code HARRIS_RESPONSE_NOBLE},
     * {@code HARRIS_RESPONSE_LOWE}, or {@code HARRIS_RESPONSE_TOMASI}.</p>
     *
     * @return {@code x, y, z, response} tuples
     */
    public static native float[] computeHarris2DKeypoints(
            int responseMethod,
            int windowWidth,
            int windowHeight,
            int minDistance,
            double threshold,
            boolean nonMaxSuppression,
            boolean refine);

    /**
     * Computes SUSAN keypoints for the active cloud.
     *
     * <p>The wrapper derives an intensity field from XYZ distance before invoking
     * PCL {@code SUSANKeypoint}.</p>
     *
     * @return {@code x, y, z, response} tuples
     */
    public static native float[] computeSUSANKeypoints(
            double radius,
            double distanceThreshold,
            double angularThreshold,
            double intensityThreshold,
            boolean nonMaxSuppression,
            boolean geometricValidation);

    /**
     * Computes Trajkovic 3D keypoints for the active organized cloud.
     *
     * @param method one of the {@code TRAJKOVIC_METHOD_*} constants
     * @return {@code x, y, z, response} tuples
     */
    public static native float[] computeTrajkovicKeypoints(
            int method,
            int windowSize,
            double firstThreshold,
            double secondThreshold,
            int normalKSearch);

    /**
     * Computes Trajkovic 2D keypoints for the active organized cloud.
     *
     * <p>The wrapper derives an intensity field from XYZ distance before invoking
     * PCL {@code TrajkovicKeypoint2D}.</p>
     *
     * @param method one of the {@code TRAJKOVIC_METHOD_*} constants
     * @return {@code x, y, z, response} tuples
     */
    public static native float[] computeTrajkovic2DKeypoints(
            int method,
            int windowSize,
            double firstThreshold,
            double secondThreshold);

    /**
     * Computes BRISK 2D keypoints for the active organized cloud.
     *
     * <p>The wrapper derives an intensity field from XYZ distance before invoking
     * PCL {@code BriskKeypoint2D}.</p>
     *
     * @return {@code x, y, z, scale} tuples
     */
    public static native float[] computeBRISK2DKeypoints(
            int threshold,
            int octaves,
            boolean removeInvalid3DKeypoints);

    /**
     * Computes AGAST 2D keypoints for the active organized cloud.
     *
     * <p>The wrapper derives an intensity field from XYZ distance before invoking
     * PCL {@code AgastKeypoint2D}.</p>
     *
     * @return {@code u, v} image-coordinate pairs
     */
    public static native float[] computeAGAST2DKeypoints(
            double threshold,
            double maxDataValue,
            boolean nonMaxSuppression,
            int maxKeypoints);

    /**
     * Builds a PCL {@code RangeImage} from the active cloud and returns finite range pixels.
     *
     * @return {@code x, y, z, range} tuples for finite range-image points
     */
    public static native float[] computeRangeImageFromActiveCloud(
            float angularResolutionDegrees,
            float maxAngleWidthDegrees,
            float maxAngleHeightDegrees,
            float sensorX,
            float sensorY,
            float sensorZ,
            float minRange);

    /**
     * Reconstructs a convex hull from the active cloud.
     *
     * @return hull points packed as {@code x, y, z} triples
     */
    public static native float[] computeConvexHull();

    /**
     * Reconstructs a concave hull from the active cloud.
     *
     * @param alpha alpha shape value used by PCL ConcaveHull
     * @return hull points packed as {@code x, y, z} triples
     */
    public static native float[] computeConcaveHull(double alpha);

    /**
     * Projects plane inliers from the active cloud onto the fitted plane.
     *
     * @return projected points packed as {@code x, y, z} triples
     */
    public static native float[] projectInliersToPlane(double distanceThreshold, int maxIterations);

    /**
     * Smooths the active cloud with Moving Least Squares.
     *
     * @param searchRadius neighborhood radius used for local surface fitting
     * @return smoothed points packed as {@code x, y, z} triples
     */
    public static native float[] smoothMovingLeastSquares(double searchRadius);

    /**
     * Smooths the active cloud with PCL SurfelSmoothing.
     *
     * @return {@code x, y, z, normal_x, normal_y, normal_z, curvature} tuples
     */
    public static native float[] smoothSurfelSmoothing(int normalKSearch, double scale);

    /**
     * Reconstructs triangle vertex indices with PCL GreedyProjectionTriangulation.
     *
     * <p>The wrapper estimates normals from the active cloud and converts points to {@code PointNormal}.
     * Angle values are radians.</p>
     *
     * @return triangle vertex index triples
     */
    public static native float[] reconstructGreedyProjectionTriangles(
            int normalKSearch,
            double searchRadius,
            double mu,
            int maximumNearestNeighbors,
            double maximumSurfaceAngle,
            double minimumAngle,
            double maximumAngle,
            boolean normalConsistency);

    /**
     * Reconstructs a mesh from the active cloud with PCL GridProjection.
     *
     * <p>The returned array is packed as {@code vertex_count, polygon_count},
     * followed by {@code x, y, z} vertex triples, then variable-length polygons
     * as {@code vertex_count, vertex_index...}.</p>
     */
    public static native float[] reconstructGridProjectionMesh(
            int normalKSearch,
            double resolution,
            int paddingSize,
            int nearestNeighborCount,
            int maxBinarySearchLevel);

    /**
     * Reconstructs a mesh from the active cloud with PCL MarchingCubesHoppe.
     *
     * <p>The returned array uses the same layout as {@link #reconstructGridProjectionMesh}.</p>
     */
    public static native float[] reconstructMarchingCubesHoppeMesh(
            int normalKSearch,
            int resolutionX,
            int resolutionY,
            int resolutionZ,
            double percentageExtendGrid,
            double isoLevel,
            double distanceIgnore);

    /**
     * Reconstructs a mesh from the active cloud with PCL MarchingCubesRBF.
     *
     * <p>The returned array uses the same layout as {@link #reconstructGridProjectionMesh}.</p>
     */
    public static native float[] reconstructMarchingCubesRBFMesh(
            int normalKSearch,
            int resolutionX,
            int resolutionY,
            int resolutionZ,
            double offSurfaceDisplacement,
            double percentageExtendGrid,
            double isoLevel);

    /**
     * Reconstructs polygons from an organized active cloud with PCL OrganizedFastMesh.
     *
     * <p>{@code triangulationType} maps to PCL values: 0 right cut, 1 left cut,
     * 2 adaptive cut, 3 quad mesh. The returned array is a sequence of variable-length
     * polygons packed as {@code vertex_count, vertex_index...}. Returns an empty array
     * when the active cloud is not organized.</p>
     */
    public static native float[] reconstructOrganizedFastMeshPolygons(
            int triangulationType,
            int trianglePixelSize,
            double maxEdgeLengthA,
            double maxEdgeLengthB,
            double maxEdgeLengthC,
            double angleTolerance,
            double distanceTolerance,
            boolean distanceDependent,
            boolean useDepthAsDistance,
            boolean storeShadowedFaces);

    /**
     * Aligns the active cloud to a translated copy with ICP.
     *
     * <p>The aligned points are stored as the filtered cloud.</p>
     *
     * @return {@code has_converged, fitness_score, row-major 4x4 matrix}
     */
    public static native float[] alignToTranslatedCopyICP(float tx, float ty, float tz, int maxIterations);

    /**
     * Estimates a rigid transform from the active cloud to {@code packedTargetXYZ} using PCL SVD.
     *
     * <p>The source and target clouds must contain the same number of {@code x, y, z} point tuples.
     * Returns an empty array when either cloud is empty or the tuple counts differ.</p>
     *
     * @return row-major 4x4 matrix
     */
    public static native float[] estimateRigidTransformSVD(float[] packedTargetXYZ);

    /**
     * Estimates a similarity transform from the active cloud to {@code packedTargetXYZ} using PCL SVD with scale.
     *
     * <p>The source and target clouds must contain the same number of {@code x, y, z} point tuples.
     * Returns an empty array when either cloud is empty or the tuple counts differ.</p>
     *
     * @return row-major 4x4 matrix
     */
    public static native float[] estimateRigidTransformSVDScale(float[] packedTargetXYZ);

    /**
     * Estimates a rigid transform from exactly three active-cloud points to {@code packedTargetXYZ}.
     *
     * <p>The source and target clouds must each contain exactly three {@code x, y, z} point tuples.
     * Returns an empty array when the tuple counts are not exactly three.</p>
     *
     * @return row-major 4x4 matrix
     */
    public static native float[] estimateRigidTransform3Point(float[] packedTargetXYZ);

    /**
     * Estimates a rigid transform from the active cloud to {@code packedTargetXYZ} using dual quaternions.
     *
     * <p>The source and target clouds must contain the same number of {@code x, y, z} point tuples.
     * Returns an empty array when either cloud is empty or the tuple counts differ.</p>
     *
     * @return row-major 4x4 matrix
     */
    public static native float[] estimateRigidTransformDualQuaternion(float[] packedTargetXYZ);

    /**
     * Estimates a rigid transform from the active cloud to {@code packedTargetXYZ} using Levenberg-Marquardt.
     *
     * <p>The source and target clouds must contain the same number of {@code x, y, z} point tuples.
     * Returns an empty array when either cloud is empty or the tuple counts differ.</p>
     *
     * @return row-major 4x4 matrix
     */
    public static native float[] estimateRigidTransformLM(float[] packedTargetXYZ);

    /**
     * Estimates a 2D rigid transform from the active cloud to {@code packedTargetXYZ}.
     *
     * <p>The source and target clouds must contain the same number of {@code x, y, z} point tuples.
     * Returns an empty array when either cloud is empty or the tuple counts differ.</p>
     *
     * @return row-major 4x4 matrix
     */
    public static native float[] estimateRigidTransform2D(float[] packedTargetXYZ);

    /**
     * Applies a row-major 4x4 transform to the active cloud with PCL {@code transformPointCloud}.
     *
     * <p>The transformed points are stored as the filtered cloud and returned as {@code x, y, z} triples.
     * Arrays shorter than 16 values are treated as an identity transform.</p>
     */
    public static native float[] transformActiveCloud(float[] rowMajor4x4);

    /**
     * Translates the active cloud with PCL {@code transformPointCloud}.
     *
     * <p>The transformed points are stored as the filtered cloud and returned as {@code x, y, z} triples.</p>
     */
    public static native float[] translateActiveCloud(float tx, float ty, float tz);

    /**
     * Aligns the active cloud to {@code packedTargetXYZ} with PCL IterativeClosestPoint.
     *
     * <p>The target is packed as {@code x, y, z} triples. The aligned points are stored as the filtered
     * cloud. Pass {@code 0.0} for optional epsilon/distance values to keep PCL defaults.</p>
     *
     * @return {@code has_converged, fitness_score, row-major 4x4 matrix}
     */
    public static native float[] alignToTargetICP(
            float[] packedTargetXYZ,
            int maxIterations,
            double maxCorrespondenceDistance,
            double transformationEpsilon,
            double euclideanFitnessEpsilon);

    /**
     * Aligns the active cloud to {@code packedTargetXYZ} with PCL GeneralizedIterativeClosestPoint.
     *
     * <p>The target is packed as {@code x, y, z} triples. The aligned points are stored as the filtered
     * cloud. Pass {@code 0.0} or {@code 0} for optional values to keep PCL defaults.</p>
     *
     * @return {@code has_converged, fitness_score, row-major 4x4 matrix}
     */
    public static native float[] alignToTargetGICP(
            float[] packedTargetXYZ,
            int maxIterations,
            double maxCorrespondenceDistance,
            double transformationEpsilon,
            double rotationEpsilon,
            int maxOptimizerIterations);

    /**
     * Aligns the active cloud to {@code packedTargetXYZ} with PCL IterativeClosestPointNonLinear.
     *
     * @return {@code has_converged, fitness_score, row-major 4x4 matrix}
     */
    public static native float[] alignToTargetICPNonLinear(
            float[] packedTargetXYZ,
            int maxIterations,
            double maxCorrespondenceDistance,
            double transformationEpsilon,
            double euclideanFitnessEpsilon);

    /**
     * Aligns the active cloud to {@code packedTargetXYZ} with PCL NormalDistributionsTransform.
     *
     * <p>Pass {@code 0.0} or {@code 0} for optional values to keep PCL defaults, except
     * {@code resolution}, which must be positive.</p>
     *
     * @return {@code has_converged, fitness_score, row-major 4x4 matrix}
     */
    public static native float[] alignToTargetNDT(
            float[] packedTargetXYZ,
            int maxIterations,
            double resolution,
            double stepSize,
            double transformationEpsilon,
            int minPointsPerVoxel);

    //region Compatibility category samples

    /**
     * Compatibility feature sample. Loads {@code filename} when supplied and runs normal estimation.
     */
    public static native void feature1(String filename);

    /**
     * Applies a pass-through filter on one axis.
     *
     * @param filename axis name, normally {@code "x"}, {@code "y"}, or {@code "z"}
     * @param min minimum accepted value
     * @param max maximum accepted value
     */
    public static native void filterAxis(String filename, double min, double max);

    /**
     * Removes points inside an axis range with PassThrough negative mode.
     */
    public static native void filterAxisOutside(String filename, double min, double max);

    /**
     * Applies PassThrough and returns the filtered points.
     */
    public static float[] passThroughFiltered(String axis, double min, double max) {
        filterAxis(axis, min, max);
        return getFilteredPoints();
    }

    /**
     * Applies PassThrough negative mode and returns the filtered points.
     */
    public static float[] passThroughOutsideFiltered(String axis, double min, double max) {
        filterAxisOutside(axis, min, max);
        return getFilteredPoints();
    }

    /**
     * Applies PCL ConditionalRemoval on one active-cloud axis range.
     */
    public static native void filterConditionalAxisRange(
            String axis,
            double min,
            double max,
            boolean keepOrganized);

    /**
     * Applies ConditionalRemoval and returns the filtered points.
     */
    public static float[] conditionalAxisRange(String axis, double min, double max, boolean keepOrganized) {
        filterConditionalAxisRange(axis, min, max, keepOrganized);
        return getFilteredPoints();
    }

    /**
     * Applies PassThrough with PCL negative, keep-organized, and replacement-value options.
     */
    public static native void filterPassThroughAdvanced(
            String axis,
            double min,
            double max,
            boolean negative,
            boolean keepOrganized,
            float userFilterValue);

    /**
     * Applies advanced PassThrough options and returns the filtered points.
     */
    public static float[] passThroughAdvanced(
            String axis,
            double min,
            double max,
            boolean negative,
            boolean keepOrganized,
            float userFilterValue) {
        filterPassThroughAdvanced(axis, min, max, negative, keepOrganized, userFilterValue);
        return getFilteredPoints();
    }

    /**
     * Downsamples the source cloud with a VoxelGrid filter.
     *
     * @param x leaf size for x
     * @param y leaf size for y
     * @param z leaf size for z
     */
    public static native void filterVoxelGrid(double x, double y, double z);

    /**
     * Downsamples the source cloud with equal VoxelGrid leaf sizes.
     */
    public static void filterVoxelGrid(double leafSize) {
        filterVoxelGrid(leafSize, leafSize, leafSize);
    }

    /**
     * Downsamples with VoxelGrid and drops voxels below the minimum point count.
     */
    public static native void filterVoxelGridMinimumPoints(
            double x,
            double y,
            double z,
            int minimumPointsPerVoxel);

    /**
     * Applies VoxelGrid minimum-points filtering and returns the filtered points.
     */
    public static float[] voxelGridMinimumPointsDownsample(
            double x,
            double y,
            double z,
            int minimumPointsPerVoxel) {
        filterVoxelGridMinimumPoints(x, y, z, minimumPointsPerVoxel);
        return getFilteredPoints();
    }

    /**
     * Downsamples the source cloud with VoxelGridCovariance.
     */
    public static native void filterVoxelGridCovariance(
            double x,
            double y,
            double z,
            int minPointsPerVoxel,
            double minCovarEigvalueMult);

    /**
     * Applies VoxelGridCovariance and returns the filtered points.
     */
    public static float[] voxelGridCovarianceDownsample(
            double x,
            double y,
            double z,
            int minPointsPerVoxel,
            double minCovarEigvalueMult) {
        filterVoxelGridCovariance(x, y, z, minPointsPerVoxel, minCovarEigvalueMult);
        return getFilteredPoints();
    }

    /**
     * Computes occluded voxels with PCL VoxelGridOcclusionEstimation.
     *
     * <p>Returns an empty array if the number of occluded voxels exceeds {@code maxVoxelCount}.</p>
     *
     * @return {@code i, j, k, centroid_x, centroid_y, centroid_z} tuples
     */
    public static native float[] computeVoxelGridOccludedVoxels(
            double x,
            double y,
            double z,
            int maxVoxelCount);

    /**
     * Applies VoxelGrid and returns the filtered points.
     */
    public static float[] voxelGridDownsample(double leafSize) {
        filterVoxelGrid(leafSize);
        return getFilteredPoints();
    }

    /**
     * Downsamples the source cloud with an ApproximateVoxelGrid filter.
     */
    public static native void filterApproximateVoxelGrid(double x, double y, double z);

    /**
     * Applies ApproximateVoxelGrid and returns the filtered points.
     */
    public static float[] approximateVoxelGridDownsample(double x, double y, double z) {
        filterApproximateVoxelGrid(x, y, z);
        return getFilteredPoints();
    }

    /**
     * Downsamples the source cloud with UniformSampling.
     */
    public static native void filterUniformSampling(double radius);

    /**
     * Applies UniformSampling and returns the filtered points.
     */
    public static float[] uniformSample(double radius) {
        filterUniformSampling(radius);
        return getFilteredPoints();
    }

    /**
     * Downsamples the source cloud with GridMinimum's 2D minimum-z grid.
     */
    public static native void filterGridMinimum(double resolution);

    /**
     * Applies GridMinimum and returns the filtered points.
     */
    public static float[] gridMinimum(double resolution) {
        filterGridMinimum(resolution);
        return getFilteredPoints();
    }

    /**
     * Keeps local z maxima inside the configured radius.
     */
    public static native void filterLocalMaximum(double radius);

    /**
     * Applies LocalMaximum and returns the filtered points.
     */
    public static float[] localMaximum(double radius) {
        filterLocalMaximum(radius);
        return getFilteredPoints();
    }

    /**
     * Applies MedianFilter to organized source-cloud points.
     */
    public static native void filterMedian(int windowSize, double maxAllowedMovement);

    /**
     * Applies MedianFilter and returns the filtered points.
     */
    public static float[] medianFilter(int windowSize, double maxAllowedMovement) {
        filterMedian(windowSize, maxAllowedMovement);
        return getFilteredPoints();
    }

    /**
     * Randomly samples up to {@code sample} source-cloud points with a deterministic seed.
     */
    public static native void filterRandomSample(int sample, int seed);

    /**
     * Applies RandomSample and returns the filtered points.
     */
    public static float[] randomSample(int sample, int seed) {
        filterRandomSample(sample, seed);
        return getFilteredPoints();
    }

    /**
     * Samples up to {@code sample} source-cloud points with FarthestPointSampling.
     */
    public static native void filterFarthestPointSampling(int sample, int seed);

    /**
     * Applies FarthestPointSampling and returns the filtered points.
     */
    public static float[] farthestPointSample(int sample, int seed) {
        filterFarthestPointSampling(sample, seed);
        return getFilteredPoints();
    }

    /**
     * Samples source-cloud points across normal-direction bins.
     */
    public static native void filterNormalSpaceSampling(
            int sample,
            int seed,
            int binsX,
            int binsY,
            int binsZ,
            int normalKSearch);

    /**
     * Applies NormalSpaceSampling and returns the filtered points.
     */
    public static float[] normalSpaceSample(
            int sample,
            int seed,
            int binsX,
            int binsY,
            int binsZ,
            int normalKSearch) {
        filterNormalSpaceSampling(sample, seed, binsX, binsY, binsZ, normalKSearch);
        return getFilteredPoints();
    }

    /**
     * Samples the active cloud with PCL SamplingSurfaceNormal.
     *
     * <p>The wrapper converts active XYZ points to {@code PointNormal} and returns sampled points with
     * normals and curvature.</p>
     *
     * @return {@code x, y, z, normal_x, normal_y, normal_z, curvature} tuples
     */
    public static native float[] sampleSurfaceNormals(int sample, int seed, double ratio);

    /**
     * Samples the active cloud with PCL CovarianceSampling.
     */
    public static native void filterCovarianceSampling(int samples, int normalKSearch);

    /**
     * Applies CovarianceSampling and returns the filtered points.
     */
    public static float[] covarianceSample(int samples, int normalKSearch) {
        filterCovarianceSampling(samples, normalKSearch);
        return getFilteredPoints();
    }

    /**
     * Computes CovarianceSampling condition number metadata.
     *
     * @return {@code condition_number, input_count, requested_samples}
     */
    public static native float[] computeCovarianceSamplingConditionNumber(int samples, int normalKSearch);

    /**
     * Smooths an organized active cloud with PCL FastBilateralFilter.
     *
     * <p>Unorganized clouds clear the filtered cloud and return no points.</p>
     */
    public static native void filterFastBilateral(double sigmaS, double sigmaR);

    /**
     * Applies FastBilateralFilter and returns the filtered points.
     */
    public static float[] fastBilateral(double sigmaS, double sigmaR) {
        filterFastBilateral(sigmaS, sigmaR);
        return getFilteredPoints();
    }

    /**
     * Smooths an organized active cloud with PCL FastBilateralFilterOMP.
     */
    public static native void filterFastBilateralOMP(double sigmaS, double sigmaR, int numberOfThreads);

    /**
     * Applies FastBilateralFilterOMP and returns the filtered points.
     */
    public static float[] fastBilateralOMP(double sigmaS, double sigmaR, int numberOfThreads) {
        filterFastBilateralOMP(sigmaS, sigmaR, numberOfThreads);
        return getFilteredPoints();
    }

    /**
     * Smooths the active cloud with PCL Convolution3D and a Gaussian kernel.
     */
    public static native void filterConvolution3DGaussian(
            double sigma,
            double radius,
            double sigmaCoefficient,
            int numberOfThreads);

    /**
     * Applies Convolution3D Gaussian smoothing and returns the filtered points.
     */
    public static float[] convolution3DGaussian(
            double sigma,
            double radius,
            double sigmaCoefficient,
            int numberOfThreads) {
        filterConvolution3DGaussian(sigma, radius, sigmaCoefficient, numberOfThreads);
        return getFilteredPoints();
    }

    /**
     * Keeps active-cloud points on the positive side of a PCL PlaneClipper3D plane.
     *
     * <p>The plane is {@code a*x + b*y + c*z + d >= 0}; set {@code negative} to keep the opposite side.</p>
     */
    public static native void filterPlaneClipper(
            double a,
            double b,
            double c,
            double d,
            boolean negative);

    /**
     * Applies PlaneClipper3D and returns the filtered points.
     */
    public static float[] planeClipper(double a, double b, double c, double d, boolean negative) {
        filterPlaneClipper(a, b, c, d, negative);
        return getFilteredPoints();
    }

    /**
     * Removes NaN points from the active cloud into the filtered cloud.
     */
    public static native void removeNaNFromActiveCloud();

    /**
     * Removes NaN points from the active cloud and returns the filtered points.
     */
    public static float[] removeNaNPoints() {
        removeNaNFromActiveCloud();
        return getFilteredPoints();
    }

    /**
     * Removes statistical outliers from the source cloud.
     */
    public static native void filterStatisticalOutlierRemoval(int meanK, double stddevMulThresh);

    /**
     * Removes statistical outliers and returns the filtered points.
     */
    public static float[] statisticalOutlierRemoval(int meanK, double stddevMulThresh) {
        filterStatisticalOutlierRemoval(meanK, stddevMulThresh);
        return getFilteredPoints();
    }

    /**
     * Removes points that do not have enough neighbors inside {@code radius}.
     */
    public static native void filterRadiusOutlierRemoval(double radius, int minNeighbors);

    /**
     * Removes radius outliers and returns the filtered points.
     */
    public static float[] radiusOutlierRemoval(double radius, int minNeighbors) {
        filterRadiusOutlierRemoval(radius, minNeighbors);
        return getFilteredPoints();
    }

    /**
     * Removes edge-discontinuity shadow points using normals estimated from the active cloud.
     */
    public static native void filterShadowPoints(int normalKSearch, double threshold);

    /**
     * Applies ShadowPoints and returns the filtered points.
     */
    public static float[] shadowPointRemoval(int normalKSearch, double threshold) {
        filterShadowPoints(normalKSearch, threshold);
        return getFilteredPoints();
    }

    /**
     * Keeps source-cloud points inside an axis-aligned 3D box.
     */
    public static native void filterCropBox(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ);

    /**
     * Applies CropBox and returns the filtered points.
     */
    public static float[] cropBox(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ) {
        filterCropBox(minX, minY, minZ, maxX, maxY, maxZ);
        return getFilteredPoints();
    }

    /**
     * Keeps source-cloud points inside a translated/rotated CropBox.
     */
    public static native void filterCropBoxTransformed(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ,
            double translationX, double translationY, double translationZ,
            double rotationX, double rotationY, double rotationZ);

    /**
     * Applies transformed CropBox and returns the filtered points.
     */
    public static float[] cropBoxTransformed(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ,
            double translationX, double translationY, double translationZ,
            double rotationX, double rotationY, double rotationZ) {
        filterCropBoxTransformed(
                minX, minY, minZ, maxX, maxY, maxZ,
                translationX, translationY, translationZ,
                rotationX, rotationY, rotationZ);
        return getFilteredPoints();
    }

    /**
     * Keeps active-cloud points inside a camera frustum.
     *
     * <p>{@code rowMajorCameraPose} is a 4x4 row-major matrix. Arrays shorter than 16 values use identity.</p>
     */
    public static native void filterFrustumCulling(
            double horizontalFov,
            double verticalFov,
            double nearPlaneDistance,
            double farPlaneDistance,
            float[] rowMajorCameraPose);

    /**
     * Applies FrustumCulling and returns the filtered points.
     */
    public static float[] frustumCulling(
            double horizontalFov,
            double verticalFov,
            double nearPlaneDistance,
            double farPlaneDistance,
            float[] rowMajorCameraPose) {
        filterFrustumCulling(
                horizontalFov, verticalFov, nearPlaneDistance, farPlaneDistance, rowMajorCameraPose);
        return getFilteredPoints();
    }

    /**
     * Filters active-cloud points against explicit SAC model coefficients.
     *
     * <p>For a plane, pass {@code a, b, c, d}. Set {@code negative} to keep outliers instead of inliers.</p>
     */
    public static native void filterModelOutlierRemoval(
            int modelType,
            float[] modelCoefficients,
            double threshold,
            boolean negative);

    /**
     * Applies ModelOutlierRemoval and returns the filtered points.
     */
    public static float[] modelOutlierRemoval(
            int modelType,
            float[] modelCoefficients,
            double threshold,
            boolean negative) {
        filterModelOutlierRemoval(modelType, modelCoefficients, threshold, negative);
        return getFilteredPoints();
    }

    /**
     * Applies a PCL morphological operation to the active cloud z dimension.
     */
    public static native void filterMorphological(double resolution, int morphologicalOperator);

    /**
     * Applies a morphological operation and returns the filtered points.
     */
    public static float[] morphologicalFilter(double resolution, int morphologicalOperator) {
        filterMorphological(resolution, morphologicalOperator);
        return getFilteredPoints();
    }

    /**
     * Filters active-cloud points with PCL CropHull in 2D polygon mode.
     *
     * <p>{@code packedHullXYZ} must contain at least three {@code x, y, z} point tuples. Set
     * {@code negative} to keep points outside the hull instead of inside it.</p>
     */
    public static native void filterCropHull2D(float[] packedHullXYZ, boolean negative);

    /**
     * Applies CropHull 2D polygon filtering and returns the filtered points.
     */
    public static float[] cropHull2D(float[] packedHullXYZ, boolean negative) {
        filterCropHull2D(packedHullXYZ, negative);
        return getFilteredPoints();
    }

    /**
     * Extracts active-cloud points by index with PCL ExtractIndices.
     */
    public static native void filterExtractIndices(int[] indices, boolean negative);

    /**
     * Extracts indexed points and returns the filtered points.
     */
    public static float[] extractIndexedPoints(int[] indices) {
        filterExtractIndices(indices, false);
        return getFilteredPoints();
    }

    /**
     * Removes indexed points and returns the filtered points.
     */
    public static float[] removeIndexedPoints(int[] indices) {
        filterExtractIndices(indices, true);
        return getFilteredPoints();
    }

    /**
     * Extracts fitted plane inliers into the filtered cloud.
     */
    public static native void extractPlaneInliers(double distanceThreshold, int maxIterations);

    /**
     * Extracts fitted plane inliers and returns the filtered points.
     */
    public static float[] extractPlaneInlierPoints(double distanceThreshold, int maxIterations) {
        extractPlaneInliers(distanceThreshold, maxIterations);
        return getFilteredPoints();
    }

    /**
     * Extracts fitted SAC model inliers into the filtered cloud.
     */
    public static native void extractModelInliers(int modelType, double distanceThreshold, int maxIterations);

    /**
     * Extracts fitted SAC model inliers and returns the filtered points.
     */
    public static float[] extractModelInlierPoints(int modelType, double distanceThreshold, int maxIterations) {
        extractModelInliers(modelType, distanceThreshold, maxIterations);
        return getFilteredPoints();
    }

    /**
     * Extracts points outside a fitted SAC model into the filtered cloud.
     */
    public static native void extractModelOutliers(int modelType, double distanceThreshold, int maxIterations);

    /**
     * Extracts points outside a fitted SAC model and returns the filtered points.
     */
    public static float[] extractModelOutlierPoints(int modelType, double distanceThreshold, int maxIterations) {
        extractModelOutliers(modelType, distanceThreshold, maxIterations);
        return getFilteredPoints();
    }

    /**
     * Compatibility geometry sample. Loads {@code filename} when supplied and computes centroid/bounds.
     */
    public static native void geometry1(String filename);

    /**
     * Compatibility KdTree sample. Loads {@code filename} when supplied and runs nearest-neighbor search.
     */
    public static native void kdtree1(String filename);

    /**
     * Compatibility keypoint sample. Loads {@code filename} when supplied and computes ISS/SIFT keypoints.
     */
    public static native void keypoint1(String filename);

    /**
     * Compatibility octree sample. Loads {@code filename} when supplied and runs radius search.
     */
    public static native void octree1(String filename);

    /**
     * Compatibility people sample for XYZ-only clouds. Loads {@code filename} and runs cluster extraction.
     */
    public static native void people1(String filename);

    /**
     * Compatibility range-image sample for XYZ-only clouds. Loads {@code filename} and computes bounds.
     */
    public static native void rangeimages1(String filename);

    /**
     * Compatibility recognition sample. Loads {@code filename} and runs cluster extraction.
     */
    public static native void recognition1(String filename);

    /**
     * Compatibility registration sample. Loads {@code filename} and runs ICP against a translated copy.
     */
    public static native void registration1(String filename);

    /**
     * Compatibility sample-consensus sample. Loads {@code filename} and fits a plane model.
     */
    public static native void sampleconsensus1(String filename);

    /**
     * Compatibility segmentation sample. Loads {@code filename} and runs plane segmentation.
     */
    public static native void segmentation1(String filename);

    /**
     * Compatibility stereo sample for XYZ-only clouds. Loads {@code filename} and runs radius filtering.
     */
    public static native void stereo1(String filename);

    /**
     * Compatibility surface sample. Loads {@code filename} and reconstructs a convex hull.
     */
    public static native void surface1(String filename);

    /**
     * Compatibility tracking sample for XYZ-only clouds. Loads {@code filename} and runs ICP.
     */
    public static native void tracking1(String filename);
    //endregion
}
