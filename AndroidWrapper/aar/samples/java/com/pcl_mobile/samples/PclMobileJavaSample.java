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

        float[] covarianceMatrix = pclmobileJNILib.computeCovarianceMatrix();
        float[] principalAxes = pclmobileJNILib.computePrincipalAxes();
        float[] momentOfInertiaAndObb = pclmobileJNILib.computeMomentOfInertiaAndOBB();
        float[] squaredDistancesToOrigin = pclmobileJNILib.computeSquaredDistancesToPoint(0.0f, 0.0f, 0.0f);
        float[] maxDistanceFromCentroid = pclmobileJNILib.computeMaxDistanceFromCentroid();
        float[] demeanedPoints = pclmobileJNILib.demeanActiveCloud();
        float[] translatedPoints = pclmobileJNILib.translateActiveCloud(0.04f, -0.02f, 0.03f);
        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        float[] rigidTransform = pclmobileJNILib.estimateRigidTransformSVD(translatedPoints);
        float[] transformedPoints = pclmobileJNILib.transformActiveCloud(rigidTransform);
        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        float[] targetIcpResult = pclmobileJNILib.alignToTargetICP(translatedPoints, 35, 0.20, 1.0e-8, 1.0e-8);
        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        float[] targetGicpResult = pclmobileJNILib.alignToTargetGICP(translatedPoints, 35, 0.20, 1.0e-8, 1.0e-8, 20);

        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        pclmobileJNILib.filterVoxelGrid(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
        float[] voxelGridPoints = pclmobileJNILib.getFilteredPoints();

        float[] centroidAndBounds = pclmobileJNILib.computeCentroidAndBounds();
        float[] normals = pclmobileJNILib.estimateNormals(16);
        float[] pfhFeatures = pclmobileJNILib.computePFHFeatures(16, 0.18);
        float[] fpfhFeatures = pclmobileJNILib.computeFPFHFeatures(16, 0.18);
        float[] vfhFeatures = pclmobileJNILib.computeVFHFeatures(16);
        float[] esfDescriptor = pclmobileJNILib.computeESFDescriptor();
        float[] gasdDescriptor = pclmobileJNILib.computeGASDDescriptor();
        float[] momentInvariants = pclmobileJNILib.computeMomentInvariants(0.18);
        float[] rsdFeatures = pclmobileJNILib.computeRSDFeatures(16, 0.18, 0.06, 5);
        float[] shotFeatures = pclmobileJNILib.computeSHOTFeatures(16, 0.18);
        float[] boundaryPoints = pclmobileJNILib.computeBoundaryPoints(16, 0.18, 90.0);
        float[] differenceOfNormals = pclmobileJNILib.computeDifferenceOfNormals(0.08, 0.20);
        float[] planeModel = pclmobileJNILib.segmentPlane(0.03, 100);
        float[] sphereModel = pclmobileJNILib.segmentSphere(0.05, 100);
        float[] nearestNeighbors = pclmobileJNILib.nearestKSearch(0.0f, 0.0f, 0.0f, 8);
        float[] limitedRadiusNeighbors = pclmobileJNILib.radiusSearchLimited(0.0f, 0.0f, 0.0f, 0.28, 6);
        float[] octreeNeighbors = pclmobileJNILib.octreeRadiusSearch(0.0f, 0.0f, 0.0f, 0.10, 0.28);
        float[] limitedOctreeNeighbors = pclmobileJNILib.octreeRadiusSearchLimited(0.0f, 0.0f, 0.0f, 0.10, 0.28, 6);
        float[] approximateOctreeNeighbor = pclmobileJNILib.octreeApproxNearestSearch(0.0f, 0.0f, 0.0f, 0.10);
        float[] clusterSizes = pclmobileJNILib.extractEuclideanClusters(0.18, 20, 5000);
        float[] issKeypoints = pclmobileJNILib.computeISSKeypoints(0.12, 0.08, 0.975, 0.975, 5);
        float[] siftKeypoints = pclmobileJNILib.computeSIFTKeypoints(0.02, 3, 4, 0.001);
        float[] harrisKeypoints = pclmobileJNILib.computeHarrisKeypoints(
                pclmobileJNILib.HARRIS_RESPONSE_HARRIS, 0.05, 0.0001, true, true);
        float[] rangeImagePoints = pclmobileJNILib.computeRangeImageFromActiveCloud(
                1.0f, 360.0f, 180.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        float[] convexHullPoints = pclmobileJNILib.computeConvexHull();
        float[] concaveHullPoints = pclmobileJNILib.computeConcaveHull(0.18);
        float[] projectedPlanePoints = pclmobileJNILib.projectInliersToPlane(0.03, 100);
        float[] mlsPoints = pclmobileJNILib.smoothMovingLeastSquares(0.12);
        float[] icpResult = pclmobileJNILib.alignToTranslatedCopyICP(0.05f, -0.03f, 0.02f, 35);

        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        pclmobileJNILib.filterAxisOutside("z", -0.10, 0.10);
        float[] passThroughOutsidePoints = pclmobileJNILib.getFilteredPoints();

        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        pclmobileJNILib.filterGridMinimum(0.10);
        float[] gridMinimumPoints = pclmobileJNILib.getFilteredPoints();

        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        pclmobileJNILib.filterNormalSpaceSampling(96, 17, 4, 4, 4, 16);
        float[] normalSpacePoints = pclmobileJNILib.getFilteredPoints();

        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        pclmobileJNILib.removeNaNFromActiveCloud();
        float[] finitePoints = pclmobileJNILib.getFilteredPoints();

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

        pclmobileJNILib.load(pcdFile.getAbsolutePath());
        pclmobileJNILib.extractModelOutliers(pclmobileJNILib.SACMODEL_PLANE, 0.03, 100);
        float[] planeOutliers = pclmobileJNILib.getFilteredPoints();

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
                covarianceMatrix,
                principalAxes,
                momentOfInertiaAndObb,
                squaredDistancesToOrigin,
                maxDistanceFromCentroid,
                demeanedPoints,
                translatedPoints,
                rigidTransform,
                transformedPoints,
                targetIcpResult,
                targetGicpResult,
                centroidAndBounds,
                normals,
                pfhFeatures,
                fpfhFeatures,
                vfhFeatures,
                esfDescriptor,
                gasdDescriptor,
                momentInvariants,
                rsdFeatures,
                shotFeatures,
                boundaryPoints,
                differenceOfNormals,
                planeModel,
                sphereModel,
                nearestNeighbors,
                limitedRadiusNeighbors,
                octreeNeighbors,
                limitedOctreeNeighbors,
                approximateOctreeNeighbor,
                clusterSizes,
                issKeypoints,
                siftKeypoints,
                harrisKeypoints,
                rangeImagePoints,
                convexHullPoints,
                concaveHullPoints,
                projectedPlanePoints,
                mlsPoints,
                icpResult,
                passThroughOutsidePoints,
                gridMinimumPoints,
                normalSpacePoints,
                finitePoints,
                statisticalInliers,
                radiusInliers,
                cropBoxPoints,
                planeInliers,
                planeOutliers);
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
        public final float[] covarianceMatrix;
        public final float[] principalAxes;
        public final float[] momentOfInertiaAndObb;
        public final float[] squaredDistancesToOrigin;
        public final float[] maxDistanceFromCentroid;
        public final float[] demeanedPoints;
        public final float[] translatedPoints;
        public final float[] rigidTransform;
        public final float[] transformedPoints;
        public final float[] targetIcpResult;
        public final float[] targetGicpResult;
        public final float[] centroidAndBounds;
        public final float[] normals;
        public final float[] pfhFeatures;
        public final float[] fpfhFeatures;
        public final float[] vfhFeatures;
        public final float[] esfDescriptor;
        public final float[] gasdDescriptor;
        public final float[] momentInvariants;
        public final float[] rsdFeatures;
        public final float[] shotFeatures;
        public final float[] boundaryPoints;
        public final float[] differenceOfNormals;
        public final float[] planeModel;
        public final float[] sphereModel;
        public final float[] nearestNeighbors;
        public final float[] limitedRadiusNeighbors;
        public final float[] octreeNeighbors;
        public final float[] limitedOctreeNeighbors;
        public final float[] approximateOctreeNeighbor;
        public final float[] clusterSizes;
        public final float[] issKeypoints;
        public final float[] siftKeypoints;
        public final float[] harrisKeypoints;
        public final float[] rangeImagePoints;
        public final float[] convexHullPoints;
        public final float[] concaveHullPoints;
        public final float[] projectedPlanePoints;
        public final float[] mlsPoints;
        public final float[] icpResult;
        public final float[] passThroughOutsidePoints;
        public final float[] gridMinimumPoints;
        public final float[] normalSpacePoints;
        public final float[] finitePoints;
        public final float[] statisticalInliers;
        public final float[] radiusInliers;
        public final float[] cropBoxPoints;
        public final float[] planeInliers;
        public final float[] planeOutliers;
        public final int rawPointCount;
        public final int voxelGridPointCount;
        public final int squaredDistanceCount;
        public final int demeanedPointCount;
        public final int translatedPointCount;
        public final int transformedPointCount;
        public final int normalCount;
        public final int pfhDescriptorCount;
        public final int fpfhDescriptorCount;
        public final int vfhDescriptorCount;
        public final int esfDescriptorCount;
        public final int gasdDescriptorCount;
        public final int momentInvariantCount;
        public final int rsdFeatureCount;
        public final int shotDescriptorCount;
        public final int boundaryPointCount;
        public final int differenceOfNormalsCount;
        public final int nearestNeighborCount;
        public final int limitedRadiusNeighborCount;
        public final int octreeNeighborCount;
        public final int limitedOctreeNeighborCount;
        public final int approximateOctreeNeighborCount;
        public final int clusterCount;
        public final int issKeypointCount;
        public final int siftKeypointCount;
        public final int harrisKeypointCount;
        public final int rangeImagePointCount;
        public final int convexHullPointCount;
        public final int concaveHullPointCount;
        public final int projectedPlanePointCount;
        public final int mlsPointCount;
        public final int passThroughOutsidePointCount;
        public final int gridMinimumPointCount;
        public final int normalSpacePointCount;
        public final int finitePointCount;
        public final int statisticalInlierCount;
        public final int radiusInlierCount;
        public final int cropBoxPointCount;
        public final int planeInlierCount;
        public final int planeOutlierCount;
        public final boolean targetIcpConverged;
        public final float targetIcpFitness;
        public final boolean targetGicpConverged;
        public final float targetGicpFitness;
        public final boolean icpConverged;
        public final float icpFitness;

        private Result(
                File pcdFile,
                float[] rawPoints,
                float[] voxelGridPoints,
                float[] covarianceMatrix,
                float[] principalAxes,
                float[] momentOfInertiaAndObb,
                float[] squaredDistancesToOrigin,
                float[] maxDistanceFromCentroid,
                float[] demeanedPoints,
                float[] translatedPoints,
                float[] rigidTransform,
                float[] transformedPoints,
                float[] targetIcpResult,
                float[] targetGicpResult,
                float[] centroidAndBounds,
                float[] normals,
                float[] pfhFeatures,
                float[] fpfhFeatures,
                float[] vfhFeatures,
                float[] esfDescriptor,
                float[] gasdDescriptor,
                float[] momentInvariants,
                float[] rsdFeatures,
                float[] shotFeatures,
                float[] boundaryPoints,
                float[] differenceOfNormals,
                float[] planeModel,
                float[] sphereModel,
                float[] nearestNeighbors,
                float[] limitedRadiusNeighbors,
                float[] octreeNeighbors,
                float[] limitedOctreeNeighbors,
                float[] approximateOctreeNeighbor,
                float[] clusterSizes,
                float[] issKeypoints,
                float[] siftKeypoints,
                float[] harrisKeypoints,
                float[] rangeImagePoints,
                float[] convexHullPoints,
                float[] concaveHullPoints,
                float[] projectedPlanePoints,
                float[] mlsPoints,
                float[] icpResult,
                float[] passThroughOutsidePoints,
                float[] gridMinimumPoints,
                float[] normalSpacePoints,
                float[] finitePoints,
                float[] statisticalInliers,
                float[] radiusInliers,
                float[] cropBoxPoints,
                float[] planeInliers,
                float[] planeOutliers) {
            this.pcdFile = pcdFile;
            this.rawPoints = rawPoints;
            this.voxelGridPoints = voxelGridPoints;
            this.covarianceMatrix = covarianceMatrix;
            this.principalAxes = principalAxes;
            this.momentOfInertiaAndObb = momentOfInertiaAndObb;
            this.squaredDistancesToOrigin = squaredDistancesToOrigin;
            this.maxDistanceFromCentroid = maxDistanceFromCentroid;
            this.demeanedPoints = demeanedPoints;
            this.translatedPoints = translatedPoints;
            this.rigidTransform = rigidTransform;
            this.transformedPoints = transformedPoints;
            this.targetIcpResult = targetIcpResult;
            this.targetGicpResult = targetGicpResult;
            this.centroidAndBounds = centroidAndBounds;
            this.normals = normals;
            this.pfhFeatures = pfhFeatures;
            this.fpfhFeatures = fpfhFeatures;
            this.vfhFeatures = vfhFeatures;
            this.esfDescriptor = esfDescriptor;
            this.gasdDescriptor = gasdDescriptor;
            this.momentInvariants = momentInvariants;
            this.rsdFeatures = rsdFeatures;
            this.shotFeatures = shotFeatures;
            this.boundaryPoints = boundaryPoints;
            this.differenceOfNormals = differenceOfNormals;
            this.planeModel = planeModel;
            this.sphereModel = sphereModel;
            this.nearestNeighbors = nearestNeighbors;
            this.limitedRadiusNeighbors = limitedRadiusNeighbors;
            this.octreeNeighbors = octreeNeighbors;
            this.limitedOctreeNeighbors = limitedOctreeNeighbors;
            this.approximateOctreeNeighbor = approximateOctreeNeighbor;
            this.clusterSizes = clusterSizes;
            this.issKeypoints = issKeypoints;
            this.siftKeypoints = siftKeypoints;
            this.harrisKeypoints = harrisKeypoints;
            this.rangeImagePoints = rangeImagePoints;
            this.convexHullPoints = convexHullPoints;
            this.concaveHullPoints = concaveHullPoints;
            this.projectedPlanePoints = projectedPlanePoints;
            this.mlsPoints = mlsPoints;
            this.icpResult = icpResult;
            this.passThroughOutsidePoints = passThroughOutsidePoints;
            this.gridMinimumPoints = gridMinimumPoints;
            this.normalSpacePoints = normalSpacePoints;
            this.finitePoints = finitePoints;
            this.statisticalInliers = statisticalInliers;
            this.radiusInliers = radiusInliers;
            this.cropBoxPoints = cropBoxPoints;
            this.planeInliers = planeInliers;
            this.planeOutliers = planeOutliers;
            this.rawPointCount = rawPoints.length / 3;
            this.voxelGridPointCount = voxelGridPoints.length / 3;
            this.squaredDistanceCount = squaredDistancesToOrigin.length;
            this.demeanedPointCount = demeanedPoints.length / 3;
            this.translatedPointCount = translatedPoints.length / 3;
            this.transformedPointCount = transformedPoints.length / 3;
            this.normalCount = normals.length / 4;
            this.pfhDescriptorCount = pfhFeatures.length / 125;
            this.fpfhDescriptorCount = fpfhFeatures.length / 33;
            this.vfhDescriptorCount = vfhFeatures.length / 308;
            this.esfDescriptorCount = esfDescriptor.length / 640;
            this.gasdDescriptorCount = gasdDescriptor.length / 512;
            this.momentInvariantCount = momentInvariants.length / 3;
            this.rsdFeatureCount = rsdFeatures.length / 2;
            this.shotDescriptorCount = shotFeatures.length / 352;
            this.boundaryPointCount = boundaryPoints.length / 4;
            this.differenceOfNormalsCount = differenceOfNormals.length / 4;
            this.nearestNeighborCount = nearestNeighbors.length / 4;
            this.limitedRadiusNeighborCount = limitedRadiusNeighbors.length / 4;
            this.octreeNeighborCount = octreeNeighbors.length / 4;
            this.limitedOctreeNeighborCount = limitedOctreeNeighbors.length / 4;
            this.approximateOctreeNeighborCount = approximateOctreeNeighbor.length / 4;
            this.clusterCount = clusterSizes.length;
            this.issKeypointCount = issKeypoints.length / 3;
            this.siftKeypointCount = siftKeypoints.length / 4;
            this.harrisKeypointCount = harrisKeypoints.length / 4;
            this.rangeImagePointCount = rangeImagePoints.length / 4;
            this.convexHullPointCount = convexHullPoints.length / 3;
            this.concaveHullPointCount = concaveHullPoints.length / 3;
            this.projectedPlanePointCount = projectedPlanePoints.length / 3;
            this.mlsPointCount = mlsPoints.length / 3;
            this.passThroughOutsidePointCount = passThroughOutsidePoints.length / 3;
            this.gridMinimumPointCount = gridMinimumPoints.length / 3;
            this.normalSpacePointCount = normalSpacePoints.length / 3;
            this.finitePointCount = finitePoints.length / 3;
            this.statisticalInlierCount = statisticalInliers.length / 3;
            this.radiusInlierCount = radiusInliers.length / 3;
            this.cropBoxPointCount = cropBoxPoints.length / 3;
            this.planeInlierCount = planeInliers.length / 3;
            this.planeOutlierCount = planeOutliers.length / 3;
            this.targetIcpConverged = targetIcpResult.length > 0 && targetIcpResult[0] == 1.0f;
            this.targetIcpFitness = targetIcpResult.length > 1 ? targetIcpResult[1] : Float.NaN;
            this.targetGicpConverged = targetGicpResult.length > 0 && targetGicpResult[0] == 1.0f;
            this.targetGicpFitness = targetGicpResult.length > 1 ? targetGicpResult[1] : Float.NaN;
            this.icpConverged = icpResult.length > 0 && icpResult[0] == 1.0f;
            this.icpFitness = icpResult.length > 1 ? icpResult[1] : Float.NaN;
        }
    }
}
