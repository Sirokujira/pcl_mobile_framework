package com.pcl_mobile.samples

import com.sirokujira.pclmobile.pclmobileJNILib
import java.io.File
import java.util.Locale
import kotlin.math.cos
import kotlin.math.sin

/**
 * Minimal Kotlin call sample for the pclmobile AAR.
 *
 * Call this from a worker thread or coroutine dispatcher when using a larger
 * point cloud.
 */
object PclMobileKotlinSample {
    private const val VOXEL_LEAF_SIZE = 0.14

    @JvmStatic
    fun runVoxelGrid(outputDir: File): Result {
        val pcdFile = writeSamplePcd(outputDir)

        pclmobileJNILib.load(pcdFile.absolutePath)
        val rawPoints = pclmobileJNILib.getCloudPoints()

        pclmobileJNILib.filterVoxelGrid(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE)
        val voxelGridPoints = pclmobileJNILib.getFilteredPoints()

        val centroidAndBounds = pclmobileJNILib.computeCentroidAndBounds()
        val normals = pclmobileJNILib.estimateNormals(16)
        val planeModel = pclmobileJNILib.segmentPlane(0.03, 100)
        val sphereModel = pclmobileJNILib.segmentSphere(0.05, 100)
        val nearestNeighbors = pclmobileJNILib.nearestKSearch(0.0f, 0.0f, 0.0f, 8)
        val octreeNeighbors = pclmobileJNILib.octreeRadiusSearch(0.0f, 0.0f, 0.0f, 0.10, 0.28)
        val clusterSizes = pclmobileJNILib.extractEuclideanClusters(0.18, 20, 5_000)
        val convexHullPoints = pclmobileJNILib.computeConvexHull()
        val concaveHullPoints = pclmobileJNILib.computeConcaveHull(0.18)
        val projectedPlanePoints = pclmobileJNILib.projectInliersToPlane(0.03, 100)
        val mlsPoints = pclmobileJNILib.smoothMovingLeastSquares(0.12)
        val icpResult = pclmobileJNILib.alignToTranslatedCopyICP(0.05f, -0.03f, 0.02f, 35)

        pclmobileJNILib.load(pcdFile.absolutePath)
        pclmobileJNILib.filterStatisticalOutlierRemoval(20, 1.0)
        val statisticalInliers = pclmobileJNILib.getFilteredPoints()

        pclmobileJNILib.load(pcdFile.absolutePath)
        pclmobileJNILib.filterRadiusOutlierRemoval(0.18, 3)
        val radiusInliers = pclmobileJNILib.getFilteredPoints()

        pclmobileJNILib.load(pcdFile.absolutePath)
        pclmobileJNILib.filterCropBox(-0.60, -0.50, -0.40, 0.85, 0.55, 0.75)
        val cropBoxPoints = pclmobileJNILib.getFilteredPoints()

        pclmobileJNILib.load(pcdFile.absolutePath)
        pclmobileJNILib.extractPlaneInliers(0.03, 100)
        val planeInliers = pclmobileJNILib.getFilteredPoints()

        check(rawPoints.isNotEmpty()) { "pclmobile returned no raw points" }
        check(voxelGridPoints.isNotEmpty()) { "pclmobile returned no VoxelGrid points" }

        return Result(
            pcdFile = pcdFile,
            rawPoints = rawPoints,
            voxelGridPoints = voxelGridPoints,
            centroidAndBounds = centroidAndBounds,
            normals = normals,
            planeModel = planeModel,
            sphereModel = sphereModel,
            nearestNeighbors = nearestNeighbors,
            octreeNeighbors = octreeNeighbors,
            clusterSizes = clusterSizes,
            convexHullPoints = convexHullPoints,
            concaveHullPoints = concaveHullPoints,
            projectedPlanePoints = projectedPlanePoints,
            mlsPoints = mlsPoints,
            icpResult = icpResult,
            statisticalInliers = statisticalInliers,
            radiusInliers = radiusInliers,
            cropBoxPoints = cropBoxPoints,
            planeInliers = planeInliers,
        )
    }

    private fun writeSamplePcd(outputDir: File): File {
        val pcdFile = File(outputDir, "pclmobile_kotlin_sample.pcd")
        val points = StringBuilder()
        var pointCount = 0

        for (ix in 0 until 48) {
            val x = -1.15 + ix * 0.05
            for (iy in 0 until 32) {
                val y = -0.78 + iy * 0.05
                val z = 0.32 * sin(x * 3.0) + 0.20 * cos(y * 4.4)
                appendPoint(points, x, y, z)
                pointCount++
            }
        }

        for (i in 0 until 220) {
            val t = i * 0.25
            val radius = 0.18 + (i % 8) * 0.012
            val x = 0.44 + cos(t) * radius
            val y = -0.10 + sin(t) * radius
            val z = -0.36 + i * 0.005
            appendPoint(points, x, y, z)
            pointCount++
        }

        pcdFile.writeText(
            buildString {
                append("# .PCD v0.7 - Point Cloud Data file format\n")
                append("VERSION 0.7\n")
                append("FIELDS x y z\n")
                append("SIZE 4 4 4\n")
                append("TYPE F F F\n")
                append("COUNT 1 1 1\n")
                append("WIDTH ").append(pointCount).append('\n')
                append("HEIGHT 1\n")
                append("VIEWPOINT 0 0 0 1 0 0 0\n")
                append("POINTS ").append(pointCount).append('\n')
                append("DATA ascii\n")
                append(points)
            },
            Charsets.UTF_8,
        )
        return pcdFile
    }

    private fun appendPoint(points: StringBuilder, x: Double, y: Double, z: Double) {
        points.append(String.format(Locale.US, "%.4f %.4f %.4f%n", x, y, z))
    }

    data class Result(
        val pcdFile: File,
        val rawPoints: FloatArray,
        val voxelGridPoints: FloatArray,
        val centroidAndBounds: FloatArray,
        val normals: FloatArray,
        val planeModel: FloatArray,
        val sphereModel: FloatArray,
        val nearestNeighbors: FloatArray,
        val octreeNeighbors: FloatArray,
        val clusterSizes: FloatArray,
        val convexHullPoints: FloatArray,
        val concaveHullPoints: FloatArray,
        val projectedPlanePoints: FloatArray,
        val mlsPoints: FloatArray,
        val icpResult: FloatArray,
        val statisticalInliers: FloatArray,
        val radiusInliers: FloatArray,
        val cropBoxPoints: FloatArray,
        val planeInliers: FloatArray,
    ) {
        val rawPointCount: Int = rawPoints.size / 3
        val voxelGridPointCount: Int = voxelGridPoints.size / 3
        val normalCount: Int = normals.size / 4
        val nearestNeighborCount: Int = nearestNeighbors.size / 4
        val octreeNeighborCount: Int = octreeNeighbors.size / 4
        val clusterCount: Int = clusterSizes.size
        val convexHullPointCount: Int = convexHullPoints.size / 3
        val concaveHullPointCount: Int = concaveHullPoints.size / 3
        val projectedPlanePointCount: Int = projectedPlanePoints.size / 3
        val mlsPointCount: Int = mlsPoints.size / 3
        val statisticalInlierCount: Int = statisticalInliers.size / 3
        val radiusInlierCount: Int = radiusInliers.size / 3
        val cropBoxPointCount: Int = cropBoxPoints.size / 3
        val planeInlierCount: Int = planeInliers.size / 3
        val icpConverged: Boolean = icpResult.firstOrNull() == 1.0f
        val icpFitness: Float = icpResult.getOrNull(1) ?: Float.NaN
    }
}
