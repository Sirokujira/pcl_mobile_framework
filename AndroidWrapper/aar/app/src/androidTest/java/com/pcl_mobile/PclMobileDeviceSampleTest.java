package com.pcl_mobile;

import android.content.Context;
import android.util.Log;

import androidx.test.ext.junit.runners.AndroidJUnit4;
import androidx.test.platform.app.InstrumentationRegistry;

import com.sirokujira.pclmobile.pclmobileJNILib;

import org.junit.Test;
import org.junit.runner.RunWith;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

@RunWith(AndroidJUnit4.class)
public class PclMobileDeviceSampleTest {
    private static final String TAG = "PclMobileDeviceSampleTest";
    private static final double VOXEL_LEAF_SIZE = 0.14;

    @Test
    public void runPclMobilePipelineOnDevice() throws Exception {
        Context context = InstrumentationRegistry.getInstrumentation().getTargetContext();
        File samplePcd = writeSamplePcd(context.getCacheDir());

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        float[] rawPoints = pclmobileJNILib.getCloudPoints();
        assertPointTriples("raw point cloud", rawPoints);
        int rawCount = rawPoints.length / 3;

        float[] covarianceMatrix = pclmobileJNILib.computeCovarianceMatrix();
        assertEquals("covariance tuple should contain centroid, 3x3 matrix, and count",
                13, covarianceMatrix.length);
        assertEquals("covariance count should match raw point count",
                rawCount, Math.round(covarianceMatrix[12]));

        float[] principalAxes = pclmobileJNILib.computePrincipalAxes();
        assertEquals("PCA tuple should contain mean, eigenvalues, eigenvectors, and count",
                16, principalAxes.length);
        assertEquals("PCA count should match raw point count",
                rawCount, Math.round(principalAxes[15]));

        float[] momentOfInertiaAndObb = pclmobileJNILib.computeMomentOfInertiaAndOBB();
        assertTrue("moment/OBB tuple should include descriptors and bounds",
                momentOfInertiaAndObb.length >= 28);
        assertEquals("moment/OBB count should match raw point count",
                rawCount, Math.round(momentOfInertiaAndObb[momentOfInertiaAndObb.length - 1]));

        float[] squaredDistancesToOrigin = pclmobileJNILib.computeSquaredDistancesToPoint(0.0f, 0.0f, 0.0f);
        assertEquals("squared distance count should match raw point count",
                rawCount, squaredDistancesToOrigin.length);
        assertTrue("squared distances should be non-negative", squaredDistancesToOrigin[0] >= 0.0f);

        float[] maxDistanceFromCentroid = pclmobileJNILib.computeMaxDistanceFromCentroid();
        assertEquals("max-distance tuple should contain centroid, point, distance, and count",
                8, maxDistanceFromCentroid.length);
        assertEquals("max-distance count should match raw point count",
                rawCount, Math.round(maxDistanceFromCentroid[7]));
        assertTrue("max distance should be positive", maxDistanceFromCentroid[6] > 0.0f);

        float[] demeanedPoints = pclmobileJNILib.demeanActiveCloud();
        assertEquals("demeaned cloud should preserve raw tuple count", rawPoints.length, demeanedPoints.length);

        float[] translatedPoints = pclmobileJNILib.translateActiveCloud(0.04f, -0.02f, 0.03f);
        assertPointTriples("translated points", translatedPoints);
        assertEquals("translated cloud should preserve raw tuple count", rawPoints.length, translatedPoints.length);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        float[] rigidTransform = pclmobileJNILib.estimateRigidTransformSVD(translatedPoints);
        assertEquals("SVD rigid transform should be a row-major 4x4 matrix", 16, rigidTransform.length);

        float[] transformedPoints = pclmobileJNILib.transformActiveCloud(rigidTransform);
        assertPointTriples("transformed points", transformedPoints);
        assertEquals("transformed cloud should preserve raw tuple count", rawPoints.length, transformedPoints.length);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        float[] targetIcpResult = pclmobileJNILib.alignToTargetICP(translatedPoints, 35, 0.20, 1.0e-8, 1.0e-8);
        assertEquals("target ICP result should contain convergence, fitness, and a 4x4 matrix",
                18, targetIcpResult.length);
        assertTrue("target ICP should converge on a translated target", targetIcpResult[0] == 1.0f);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        float[] targetGicpResult = pclmobileJNILib.alignToTargetGICP(translatedPoints, 35, 0.20, 1.0e-8, 1.0e-8, 20);
        assertEquals("target GICP result should contain convergence, fitness, and a 4x4 matrix",
                18, targetGicpResult.length);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        pclmobileJNILib.filterVoxelGrid(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
        float[] voxelGridPoints = pclmobileJNILib.getFilteredPoints();
        assertPointTriples("voxel-grid point cloud", voxelGridPoints);
        assertTrue("VoxelGrid should downsample this sample",
                voxelGridPoints.length < rawPoints.length);

        float[] centroidAndBounds = pclmobileJNILib.computeCentroidAndBounds();
        assertTrue("centroid/bounds should contain 10 values", centroidAndBounds.length == 10);

        float[] normals = pclmobileJNILib.estimateNormals(16);
        assertTrue("normals should contain x/y/z/curvature tuples", normals.length >= 4);
        assertTrue("normal tuple packing", normals.length % 4 == 0);

        float[] shotFeatures = pclmobileJNILib.computeSHOTFeatures(16, 0.18);
        assertTrue("SHOT descriptors should be 352-bin tuples", shotFeatures.length % 352 == 0);

        float[] boundaryPoints = pclmobileJNILib.computeBoundaryPoints(16, 0.18, 90.0);
        assertTrue("boundary results should be x/y/z/flag tuples", boundaryPoints.length % 4 == 0);

        float[] differenceOfNormals = pclmobileJNILib.computeDifferenceOfNormals(0.08, 0.20);
        assertTrue("DoN should contain normal/curvature tuples", differenceOfNormals.length % 4 == 0);

        float[] planeModel = pclmobileJNILib.segmentPlane(0.03, 100);
        assertTrue("plane model should contain coefficients and counts", planeModel.length >= 6);

        float[] sphereModel = pclmobileJNILib.segmentSphere(0.05, 100);
        assertTrue("sphere model should contain coefficients and counts", sphereModel.length >= 6);

        float[] nearestNeighbors = pclmobileJNILib.nearestKSearch(0.0f, 0.0f, 0.0f, 8);
        assertTrue("nearest search should return x/y/z/distance tuples", nearestNeighbors.length >= 4);
        assertTrue("nearest tuple packing", nearestNeighbors.length % 4 == 0);

        float[] octreeNeighbors = pclmobileJNILib.octreeRadiusSearch(0.0f, 0.0f, 0.0f, 0.10, 0.28);
        assertTrue("octree radius search should return x/y/z/distance tuples", octreeNeighbors.length >= 4);
        assertTrue("octree tuple packing", octreeNeighbors.length % 4 == 0);

        float[] clusterSizes = pclmobileJNILib.extractEuclideanClusters(0.18, 20, 5000);
        assertTrue("cluster extraction should find at least one cluster", clusterSizes.length >= 1);

        float[] convexHullPoints = pclmobileJNILib.computeConvexHull();
        assertPointTriples("convex hull", convexHullPoints);

        float[] concaveHullPoints = pclmobileJNILib.computeConcaveHull(0.18);
        assertPointTriples("concave hull", concaveHullPoints);

        float[] projectedPlanePoints = pclmobileJNILib.projectInliersToPlane(0.03, 100);
        assertPointTriples("projected plane", projectedPlanePoints);

        float[] mlsPoints = pclmobileJNILib.smoothMovingLeastSquares(0.12);
        assertPointTriples("moving least squares", mlsPoints);

        float[] icpResult = pclmobileJNILib.alignToTranslatedCopyICP(0.05f, -0.03f, 0.02f, 35);
        assertTrue("ICP result should contain convergence, fitness, and a 4x4 matrix",
                icpResult.length == 18);
        assertTrue("ICP should converge on a translated copy", icpResult[0] == 1.0f);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        pclmobileJNILib.filterAxisOutside("z", -0.10, 0.10);
        float[] passThroughOutsidePoints = pclmobileJNILib.getFilteredPoints();
        assertPointTriples("pass-through outside points", passThroughOutsidePoints);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        pclmobileJNILib.filterGridMinimum(0.10);
        float[] gridMinimumPoints = pclmobileJNILib.getFilteredPoints();
        assertPointTriples("grid-minimum points", gridMinimumPoints);
        assertTrue("GridMinimum should not increase this sample",
                gridMinimumPoints.length <= rawPoints.length);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        pclmobileJNILib.filterNormalSpaceSampling(128, 17, 4, 4, 4, 16);
        float[] normalSpacePoints = pclmobileJNILib.getFilteredPoints();
        assertPointTriples("normal-space sampled points", normalSpacePoints);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        pclmobileJNILib.removeNaNFromActiveCloud();
        float[] finitePoints = pclmobileJNILib.getFilteredPoints();
        assertPointTriples("finite points", finitePoints);
        assertEquals("NaN removal should keep every finite sample point",
                rawPoints.length, finitePoints.length);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        pclmobileJNILib.filterStatisticalOutlierRemoval(20, 1.0);
        float[] statisticalInliers = pclmobileJNILib.getFilteredPoints();
        assertPointTriples("statistical inliers", statisticalInliers);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        pclmobileJNILib.filterRadiusOutlierRemoval(0.18, 3);
        float[] radiusInliers = pclmobileJNILib.getFilteredPoints();
        assertPointTriples("radius inliers", radiusInliers);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        pclmobileJNILib.filterCropBox(-0.60, -0.50, -0.40, 0.85, 0.55, 0.75);
        float[] cropBoxPoints = pclmobileJNILib.getFilteredPoints();
        assertPointTriples("crop-box points", cropBoxPoints);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        pclmobileJNILib.extractPlaneInliers(0.03, 100);
        float[] planeInliers = pclmobileJNILib.getFilteredPoints();
        assertPointTriples("plane inliers", planeInliers);

        pclmobileJNILib.load(samplePcd.getAbsolutePath());
        pclmobileJNILib.extractModelOutliers(pclmobileJNILib.SACMODEL_PLANE, 0.03, 100);
        float[] planeOutliers = pclmobileJNILib.getFilteredPoints();
        assertPointTriples("plane outliers", planeOutliers);

        runCompatibilityCategorySamples(samplePcd);

        int voxelCount = voxelGridPoints.length / 3;
        int reduction = Math.round((1.0f - (voxelCount / (float) rawCount)) * 100.0f);
        Log.i(TAG, "device sample passed: raw=" + rawCount
                + " voxel=" + voxelCount
                + " reduction=" + reduction + "%"
                + " covariance=" + covarianceMatrix.length
                + " pca=" + principalAxes.length
                + " momentObb=" + momentOfInertiaAndObb.length
                + " distances=" + squaredDistancesToOrigin.length
                + " maxDistance=" + maxDistanceFromCentroid[6]
                + " demeaned=" + (demeanedPoints.length / 3)
                + " translated=" + (translatedPoints.length / 3)
                + " rigidTransform=" + rigidTransform.length
                + " transformed=" + (transformedPoints.length / 3)
                + " normals=" + (normals.length / 4)
                + " shot=" + (shotFeatures.length / 352)
                + " boundary=" + (boundaryPoints.length / 4)
                + " don=" + (differenceOfNormals.length / 4)
                + " planeInliers=" + Math.round(planeModel[4])
                + " sphereInliers=" + Math.round(sphereModel[4])
                + " nearest=" + (nearestNeighbors.length / 4)
                + " octree=" + (octreeNeighbors.length / 4)
                + " clusters=" + clusterSizes.length
                + " hull=" + (convexHullPoints.length / 3)
                + " concaveHull=" + (concaveHullPoints.length / 3)
                + " projectedPlane=" + (projectedPlanePoints.length / 3)
                + " mls=" + (mlsPoints.length / 3)
                + " passThroughOutside=" + (passThroughOutsidePoints.length / 3)
                + " gridMinimum=" + (gridMinimumPoints.length / 3)
                + " normalSpace=" + (normalSpacePoints.length / 3)
                + " finite=" + (finitePoints.length / 3)
                + " sor=" + (statisticalInliers.length / 3)
                + " radius=" + (radiusInliers.length / 3)
                + " cropBox=" + (cropBoxPoints.length / 3)
                + " extractedPlane=" + (planeInliers.length / 3)
                + " planeOutliers=" + (planeOutliers.length / 3)
                + " icpFitness=" + icpResult[1]
                + " targetIcpFitness=" + targetIcpResult[1]
                + " targetGicpFitness=" + targetGicpResult[1]
                + " file=" + samplePcd.getAbsolutePath());
    }

    private static void runCompatibilityCategorySamples(File samplePcd) {
        String path = samplePcd.getAbsolutePath();
        pclmobileJNILib.feature1(path);
        pclmobileJNILib.geometry1(path);
        pclmobileJNILib.kdtree1(path);
        pclmobileJNILib.keypoint1(path);
        pclmobileJNILib.octree1(path);
        pclmobileJNILib.people1(path);
        pclmobileJNILib.rangeimages1(path);
        pclmobileJNILib.recognition1(path);
        pclmobileJNILib.registration1(path);
        pclmobileJNILib.sampleconsensus1(path);
        pclmobileJNILib.segmentation1(path);
        pclmobileJNILib.stereo1(path);
        pclmobileJNILib.surface1(path);
        pclmobileJNILib.tracking1(path);
    }

    private static void assertPointTriples(String label, float[] points) {
        assertTrue(label + " should not be empty", points.length >= 3);
        assertTrue(label + " should be packed as x/y/z triples", points.length % 3 == 0);
    }

    private static File writeSamplePcd(File outputDir) throws IOException {
        File output = new File(outputDir, "pclmobile_device_test_sample.pcd");
        StringBuilder points = new StringBuilder();
        int pointCount = 0;

        for (int ix = 0; ix < 62; ix++) {
            double x = -1.45 + ix * 0.047;
            for (int iy = 0; iy < 42; iy++) {
                double y = -0.98 + iy * 0.047;
                double base = 0.30 * Math.sin(x * 3.0) + 0.22 * Math.cos(y * 4.2);
                double bump = Math.exp(-((x - 0.36) * (x - 0.36) + (y + 0.16) * (y + 0.16)) * 12.0);
                double groove = Math.exp(-((x + 0.58) * (x + 0.58) + (y - 0.34) * (y - 0.34)) * 18.0);
                appendPoint(points, x, y, base + bump * 0.82 - groove * 0.35);
                pointCount++;
            }
        }

        for (int i = 0; i < 420; i++) {
            double t = i * 0.23;
            double radius = 0.20 + (i % 11) * 0.010;
            double x = 0.46 + Math.cos(t) * radius;
            double y = -0.16 + Math.sin(t) * radius;
            double z = -0.48 + i * 0.0042;
            appendPoint(points, x, y, z);
            pointCount++;
        }

        String pcd = "# .PCD v0.7 - Point Cloud Data file format\n"
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

        try (FileOutputStream stream = new FileOutputStream(output)) {
            stream.write(pcd.getBytes(StandardCharsets.UTF_8));
        }
        return output;
    }

    private static void appendPoint(StringBuilder points, double x, double y, double z) {
        points.append(String.format(Locale.US, "%.4f %.4f %.4f%n", x, y, z));
    }
}
