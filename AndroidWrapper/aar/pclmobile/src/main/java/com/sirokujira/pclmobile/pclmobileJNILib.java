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
 *     <li>Plane model: {@code a, b, c, d, inlier_count, input_count}</li>
 *     <li>Sphere model: {@code center_x, center_y, center_z, radius, inlier_count, input_count}</li>
 *     <li>Nearest/radius search: {@code x, y, z, squared_distance, ...}</li>
 *     <li>Centroid/bounds: {@code centroid_xyz, min_xyz, max_xyz, point_count}</li>
 *     <li>ICP: {@code has_converged, fitness_score, row-major 4x4 matrix}</li>
 * </ul>
 */
public final class pclmobileJNILib {

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
     * Returns the loaded source cloud packed as {@code x, y, z} triples.
     */
    public static native float[] getCloudPoints();

    /**
     * Returns the most recent filtered cloud packed as {@code x, y, z} triples.
     */
    public static native float[] getFilteredPoints();

    /**
     * Computes centroid, min bounds, max bounds, and point count for the active cloud.
     *
     * <p>The active cloud is the filtered cloud when non-empty, otherwise the source cloud.</p>
     */
    public static native float[] computeCentroidAndBounds();

    /**
     * Estimates normals for the active cloud using KdTree-backed neighborhood search.
     *
     * @param kSearch number of nearest neighbors used for each normal estimate
     * @return {@code normal_x, normal_y, normal_z, curvature} tuples
     */
    public static native float[] estimateNormals(int kSearch);

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
     * Runs nearest-neighbor search against the active cloud.
     *
     * @return {@code x, y, z, squared_distance} tuples for found neighbors
     */
    public static native float[] nearestKSearch(float x, float y, float z, int k);

    /**
     * Runs octree radius search against the active cloud.
     *
     * @param resolution octree voxel resolution
     * @param radius search radius around {@code x/y/z}
     * @return {@code x, y, z, squared_distance} tuples for found neighbors
     */
    public static native float[] octreeRadiusSearch(float x, float y, float z, double resolution, double radius);

    /**
     * Extracts Euclidean clusters from the active cloud.
     *
     * @return one cluster size per element
     */
    public static native float[] extractEuclideanClusters(double tolerance, int minClusterSize, int maxClusterSize);

    /**
     * Reconstructs a convex hull from the active cloud.
     *
     * @return hull points packed as {@code x, y, z} triples
     */
    public static native float[] computeConvexHull();

    /**
     * Projects plane inliers from the active cloud onto the fitted plane.
     *
     * @return projected points packed as {@code x, y, z} triples
     */
    public static native float[] projectInliersToPlane(double distanceThreshold, int maxIterations);

    /**
     * Aligns the active cloud to a translated copy with ICP.
     *
     * @return {@code has_converged, fitness_score, row-major 4x4 matrix}
     */
    public static native float[] alignToTranslatedCopyICP(float tx, float ty, float tz, int maxIterations);

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
     * Downsamples the source cloud with a VoxelGrid filter.
     *
     * @param x leaf size for x
     * @param y leaf size for y
     * @param z leaf size for z
     */
    public static native void filterVoxelGrid(double x, double y, double z);

    /**
     * Removes statistical outliers from the source cloud.
     */
    public static native void filterStatisticalOutlierRemoval(int meanK, double stddevMulThresh);

    /**
     * Removes points that do not have enough neighbors inside {@code radius}.
     */
    public static native void filterRadiusOutlierRemoval(double radius, int minNeighbors);

    /**
     * Keeps source-cloud points inside an axis-aligned 3D box.
     */
    public static native void filterCropBox(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ);

    /**
     * Extracts fitted plane inliers into the filtered cloud.
     */
    public static native void extractPlaneInliers(double distanceThreshold, int maxIterations);

    /**
     * Compatibility geometry sample. Loads {@code filename} when supplied and computes centroid/bounds.
     */
    public static native void geometry1(String filename);

    /**
     * Compatibility KdTree sample. Loads {@code filename} when supplied and runs nearest-neighbor search.
     */
    public static native void kdtree1(String filename);

    /**
     * Compatibility keypoint sample. Loads {@code filename} when supplied and runs a KdTree probe.
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
