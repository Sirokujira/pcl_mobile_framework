package com.pcl_mobile.samples;

import com.sirokujira.pclmobile.pclmobileJNILib;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

/**
 * Minimal Java call sample for the pclmobile AAR.
 *
 * Call this from a worker thread when using a larger point cloud.
 */
public final class PclMobileJavaSample {
    private static final double VOXEL_LEAF_SIZE = 0.14;

    private PclMobileJavaSample() {
    }

    public static Result runVoxelGrid(File outputDir) throws IOException {
        File pcdFile = writeSamplePcd(outputDir);

        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        float[] rawPoints = pclmobileJNILib.getCloudPoints();

        pclmobileJNILib.filterVoxelGrid(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
        float[] voxelGridPoints = pclmobileJNILib.getFilteredPoints();

        float[] centroidAndBounds = pclmobileJNILib.computeCentroidAndBounds();
        float[] normals = pclmobileJNILib.estimateNormals(16);
        float[] planeModel = pclmobileJNILib.segmentPlane(0.03, 100);
        float[] sphereModel = pclmobileJNILib.segmentSphere(0.05, 100);
        float[] nearestNeighbors = pclmobileJNILib.nearestKSearch(0.0f, 0.0f, 0.0f, 8);
        float[] octreeNeighbors = pclmobileJNILib.octreeRadiusSearch(0.0f, 0.0f, 0.0f, 0.10, 0.28);
        float[] clusterSizes = pclmobileJNILib.extractEuclideanClusters(0.18, 20, 5000);
        float[] convexHullPoints = pclmobileJNILib.computeConvexHull();
        float[] projectedPlanePoints = pclmobileJNILib.projectInliersToPlane(0.03, 100);
        float[] icpResult = pclmobileJNILib.alignToTranslatedCopyICP(0.05f, -0.03f, 0.02f, 35);

        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        pclmobileJNILib.filterStatisticalOutlierRemoval(20, 1.0);
        float[] statisticalInliers = pclmobileJNILib.getFilteredPoints();

        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        pclmobileJNILib.filterRadiusOutlierRemoval(0.18, 3);
        float[] radiusInliers = pclmobileJNILib.getFilteredPoints();

        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        pclmobileJNILib.filterCropBox(-0.60, -0.50, -0.40, 0.85, 0.55, 0.75);
        float[] cropBoxPoints = pclmobileJNILib.getFilteredPoints();

        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        pclmobileJNILib.extractPlaneInliers(0.03, 100);
        float[] planeInliers = pclmobileJNILib.getFilteredPoints();

        if (rawPoints.length == 0) {
            throw new IllegalStateException("pclmobile returned no raw points");
        }
        if (voxelGridPoints.length == 0) {
            throw new IllegalStateException("pclmobile returned no VoxelGrid points");
        }

        return new Result(
                pcdFile,
                rawPoints,
                voxelGridPoints,
                centroidAndBounds,
                normals,
                planeModel,
                sphereModel,
                nearestNeighbors,
                octreeNeighbors,
                clusterSizes,
                convexHullPoints,
                projectedPlanePoints,
                icpResult,
                statisticalInliers,
                radiusInliers,
                cropBoxPoints,
                planeInliers);
    }

    private static File writeSamplePcd(File outputDir) throws IOException {
        File pcdFile = new File(outputDir, "pclmobile_java_sample.pcd");
        StringBuilder points = new StringBuilder();
        int pointCount = 0;

        for (int ix = 0; ix < 48; ix++) {
            double x = -1.15 + ix * 0.05;
            for (int iy = 0; iy < 32; iy++) {
                double y = -0.78 + iy * 0.05;
                double z = 0.32 * Math.sin(x * 3.0) + 0.20 * Math.cos(y * 4.4);
                appendPoint(points, x, y, z);
                pointCount++;
            }
        }

        for (int i = 0; i < 220; i++) {
            double t = i * 0.25;
            double radius = 0.18 + (i % 8) * 0.012;
            double x = 0.44 + Math.cos(t) * radius;
            double y = -0.10 + Math.sin(t) * radius;
            double z = -0.36 + i * 0.005;
            appendPoint(points, x, y, z);
            pointCount++;
        }

        String pcd = ""
                + "# .PCD v0.7 - Point Cloud Data file format\n"
                + "VERSION 0.7\n"
                + "FIELDS x y z\n"
                + "SIZE 4 4 4\n"
                + "TYPE F F F\n"
                + "COUNT 1 1 1\n"
                + "WIDTH " + pointCount + "\n"
                + "HEIGHT 1\n"
                + "VIEWPOINT 0 0 0 1 0 0 0\n"
                + "POINTS " + pointCount + "\n"
                + "DATA ascii\n"
                + points;

        try (FileOutputStream stream = new FileOutputStream(pcdFile)) {
            stream.write(pcd.getBytes(StandardCharsets.UTF_8));
        }
        return pcdFile;
    }

    private static void appendPoint(StringBuilder points, double x, double y, double z) {
        points.append(String.format(Locale.US, "%.4f %.4f %.4f%n", x, y, z));
    }

    public static final class Result {
        public final File pcdFile;
        public final float[] rawPoints;
        public final float[] voxelGridPoints;
        public final float[] centroidAndBounds;
        public final float[] normals;
        public final float[] planeModel;
        public final float[] sphereModel;
        public final float[] nearestNeighbors;
        public final float[] octreeNeighbors;
        public final float[] clusterSizes;
        public final float[] convexHullPoints;
        public final float[] projectedPlanePoints;
        public final float[] icpResult;
        public final float[] statisticalInliers;
        public final float[] radiusInliers;
        public final float[] cropBoxPoints;
        public final float[] planeInliers;
        public final int rawPointCount;
        public final int voxelGridPointCount;
        public final int normalCount;
        public final int nearestNeighborCount;
        public final int octreeNeighborCount;
        public final int clusterCount;
        public final int convexHullPointCount;
        public final int projectedPlanePointCount;
        public final int statisticalInlierCount;
        public final int radiusInlierCount;
        public final int cropBoxPointCount;
        public final int planeInlierCount;
        public final boolean icpConverged;
        public final float icpFitness;

        private Result(
                File pcdFile,
                float[] rawPoints,
                float[] voxelGridPoints,
                float[] centroidAndBounds,
                float[] normals,
                float[] planeModel,
                float[] sphereModel,
                float[] nearestNeighbors,
                float[] octreeNeighbors,
                float[] clusterSizes,
                float[] convexHullPoints,
                float[] projectedPlanePoints,
                float[] icpResult,
                float[] statisticalInliers,
                float[] radiusInliers,
                float[] cropBoxPoints,
                float[] planeInliers) {
            this.pcdFile = pcdFile;
            this.rawPoints = rawPoints;
            this.voxelGridPoints = voxelGridPoints;
            this.centroidAndBounds = centroidAndBounds;
            this.normals = normals;
            this.planeModel = planeModel;
            this.sphereModel = sphereModel;
            this.nearestNeighbors = nearestNeighbors;
            this.octreeNeighbors = octreeNeighbors;
            this.clusterSizes = clusterSizes;
            this.convexHullPoints = convexHullPoints;
            this.projectedPlanePoints = projectedPlanePoints;
            this.icpResult = icpResult;
            this.statisticalInliers = statisticalInliers;
            this.radiusInliers = radiusInliers;
            this.cropBoxPoints = cropBoxPoints;
            this.planeInliers = planeInliers;
            this.rawPointCount = rawPoints.length / 3;
            this.voxelGridPointCount = voxelGridPoints.length / 3;
            this.normalCount = normals.length / 4;
            this.nearestNeighborCount = nearestNeighbors.length / 4;
            this.octreeNeighborCount = octreeNeighbors.length / 4;
            this.clusterCount = clusterSizes.length;
            this.convexHullPointCount = convexHullPoints.length / 3;
            this.projectedPlanePointCount = projectedPlanePoints.length / 3;
            this.statisticalInlierCount = statisticalInliers.length / 3;
            this.radiusInlierCount = radiusInliers.length / 3;
            this.cropBoxPointCount = cropBoxPoints.length / 3;
            this.planeInlierCount = planeInliers.length / 3;
            this.icpConverged = icpResult.length > 0 && icpResult[0] == 1.0f;
            this.icpFitness = icpResult.length > 1 ? icpResult[1] : Float.NaN;
        }
    }
}
