#include <jni.h>

#include "pcl_mobile_arrays.h"
#include "pcl_mobile_context.h"
#include "pcl_mobile_features.h"
#include "pcl_mobile_filters.h"
#include "pcl_mobile_io.h"
#include "pcl_mobile_log.h"
#include "pcl_mobile_registration.h"
#include "pcl_mobile_search.h"
#include "pcl_mobile_segmentation.h"
#include "pcl_mobile_surface.h"

namespace {

void loadIfProvided(JNIEnv* env, jstring filename)
{
    std::string path = pclmobile::jstringToString(env, filename);
    if (!path.empty()) {
        pclmobile::loadPCDFile(path);
    }
}

void logPointCount(const char* name, const char* operation, std::size_t float_count)
{
    LOGI("%s compatibility sample: %s points=%zu", name, operation, float_count / 3);
}

void logTupleCount(const char* name, const char* operation, std::size_t float_count, std::size_t tuple_size)
{
    LOGI("%s compatibility sample: %s tuples=%zu", name, operation,
         tuple_size == 0 ? 0 : float_count / tuple_size);
}

jintArray makeCloudShapeArray(JNIEnv* env, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source)
{
    jint values[3] = {
            static_cast<jint>(source->points.size()),
            static_cast<jint>(source->width),
            static_cast<jint>(source->height),
    };
    jintArray result = env->NewIntArray(3);
    if (result != nullptr) {
        env->SetIntArrayRegion(result, 0, 3, values);
    }
    return result;
}

} // namespace

extern "C" {

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_init(
        JNIEnv* env, jobject obj, jint width, jint height)
{
    (void) env;
    (void) obj;
    (void) width;
    (void) height;
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_step(JNIEnv* env, jobject obj)
{
    (void) env;
    (void) obj;
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_load(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    pclmobile::loadPCDFile(pclmobile::jstringToString(env, filename));
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_loadPLY(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    pclmobile::loadPLYFile(pclmobile::jstringToString(env, filename));
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_setCloudPoints(
        JNIEnv* env, jclass clazz, jfloatArray packedXYZ)
{
    (void) clazz;
    if (packedXYZ == nullptr) {
        pclmobile::setCloudFromPackedXYZ({});
        return;
    }

    const jsize length = env->GetArrayLength(packedXYZ);
    std::vector<jfloat> values(static_cast<std::size_t>(length));
    if (length > 0) {
        env->GetFloatArrayRegion(packedXYZ, 0, length, values.data());
    }
    pclmobile::setCloudFromPackedXYZ(values);
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_getCloudPoints(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makePointArray(env, pclmobile::cloud());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_getFilteredPoints(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makePointArray(env, pclmobile::filteredCloud());
}

JNIEXPORT jint JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_getPointCount(
        JNIEnv* env, jclass clazz)
{
    (void) env;
    (void) clazz;
    return static_cast<jint>(pclmobile::activeCloud()->points.size());
}

JNIEXPORT jint JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_getWidth(
        JNIEnv* env, jclass clazz)
{
    (void) env;
    (void) clazz;
    return static_cast<jint>(pclmobile::activeCloud()->width);
}

JNIEXPORT jint JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_getHeight(
        JNIEnv* env, jclass clazz)
{
    (void) env;
    (void) clazz;
    return static_cast<jint>(pclmobile::activeCloud()->height);
}

JNIEXPORT jintArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_getCloudShape(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return makeCloudShapeArray(env, pclmobile::activeCloud());
}

JNIEXPORT jintArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_getSourceCloudShape(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return makeCloudShapeArray(env, pclmobile::cloud());
}

JNIEXPORT jintArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_getFilteredCloudShape(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return makeCloudShapeArray(env, pclmobile::filteredCloud());
}

JNIEXPORT jboolean JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_writeActivePCDFile(
        JNIEnv* env, jclass clazz, jstring filename)
{
    (void) clazz;
    return pclmobile::writePCDFileASCII(pclmobile::jstringToString(env, filename), pclmobile::activeCloud())
            ? JNI_TRUE
            : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_writeSourcePCDFile(
        JNIEnv* env, jclass clazz, jstring filename)
{
    (void) clazz;
    return pclmobile::writePCDFileASCII(pclmobile::jstringToString(env, filename), pclmobile::cloud())
            ? JNI_TRUE
            : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_writeFilteredPCDFile(
        JNIEnv* env, jclass clazz, jstring filename)
{
    (void) clazz;
    return pclmobile::writePCDFileASCII(pclmobile::jstringToString(env, filename), pclmobile::filteredCloud())
            ? JNI_TRUE
            : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_writeActivePLYFile(
        JNIEnv* env, jclass clazz, jstring filename)
{
    (void) clazz;
    return pclmobile::writePLYFileASCII(pclmobile::jstringToString(env, filename), pclmobile::activeCloud())
            ? JNI_TRUE
            : JNI_FALSE;
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeCentroidAndBounds(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeCentroidAndBounds());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_estimateNormals(
        JNIEnv* env, jclass clazz, jint kSearch)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::estimateNormals(kSearch));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_estimateNormalsRadius(
        JNIEnv* env, jclass clazz, jdouble radiusSearch)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::estimateNormalsRadius(radiusSearch));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeFPFHFeatures(
        JNIEnv* env, jclass clazz, jint normalKSearch, jdouble featureRadius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::computeFPFHFeatures(normalKSearch, featureRadius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computePrincipalCurvatures(
        JNIEnv* env, jclass clazz, jint normalKSearch, jint curvatureKSearch)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::computePrincipalCurvatures(normalKSearch, curvatureKSearch));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_segmentPlane(
        JNIEnv* env, jclass clazz, jdouble distanceThreshold, jint maxIterations)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::segmentPlaneModel(distanceThreshold, maxIterations));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_segmentSphere(
        JNIEnv* env, jclass clazz, jdouble distanceThreshold, jint maxIterations)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::segmentSphereModel(distanceThreshold, maxIterations));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_segmentSACModel(
        JNIEnv* env, jclass clazz, jint modelType, jdouble distanceThreshold, jint maxIterations)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::segmentSACModel(modelType, distanceThreshold, maxIterations));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_nearestKSearch(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jint k)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::nearestKSearch(x, y, z, k));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_radiusSearch(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble radius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::radiusSearch(x, y, z, radius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeNearestKSearch(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble resolution, jint k)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::octreeNearestKSearch(x, y, z, resolution, k));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeRadiusSearch(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble resolution, jdouble radius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::octreeRadiusSearch(x, y, z, resolution, radius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeVoxelSearch(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble resolution)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::octreeVoxelSearch(x, y, z, resolution));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractEuclideanClusters(
        JNIEnv* env, jclass clazz, jdouble tolerance, jint minClusterSize, jint maxClusterSize)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::extractEuclideanClusters(tolerance, minClusterSize, maxClusterSize));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeConvexHull(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makePointArray(env, pclmobile::computeConvexHull());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeConcaveHull(
        JNIEnv* env, jclass clazz, jdouble alpha)
{
    (void) clazz;
    return pclmobile::makePointArray(env, pclmobile::computeConcaveHull(alpha));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_projectInliersToPlane(
        JNIEnv* env, jclass clazz, jdouble distanceThreshold, jint maxIterations)
{
    (void) clazz;
    return pclmobile::makePointArray(env, pclmobile::projectInliersToPlane(distanceThreshold, maxIterations));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_smoothMovingLeastSquares(
        JNIEnv* env, jclass clazz, jdouble searchRadius)
{
    (void) clazz;
    return pclmobile::makePointArray(env, pclmobile::smoothMovingLeastSquares(searchRadius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_alignToTranslatedCopyICP(
        JNIEnv* env, jclass clazz, jfloat tx, jfloat ty, jfloat tz, jint maxIterations)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::alignToTranslatedCopyICP(tx, ty, tz, maxIterations));
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_feature1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    logTupleCount("feature1", "NormalEstimation", pclmobile::estimateNormals(16).size(), 4);
    logTupleCount("feature1", "FPFHEstimation",
                  pclmobile::computeFPFHFeatures(16, 0.18).size(), 33);
    logTupleCount("feature1", "PrincipalCurvaturesEstimation",
                  pclmobile::computePrincipalCurvatures(16, 16).size(), 5);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterAxis(
        JNIEnv* env, jobject obj, jstring axis, jdouble minValue, jdouble maxValue)
{
    (void) obj;
    pclmobile::filterAxis(pclmobile::jstringToString(env, axis), minValue, maxValue);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterVoxelGrid(
        JNIEnv* env, jobject obj, jdouble x, jdouble y, jdouble z)
{
    (void) env;
    (void) obj;
    pclmobile::filterVoxelGrid(x, y, z);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterApproximateVoxelGrid(
        JNIEnv* env, jclass clazz, jdouble x, jdouble y, jdouble z)
{
    (void) env;
    (void) clazz;
    pclmobile::filterApproximateVoxelGrid(x, y, z);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterUniformSampling(
        JNIEnv* env, jclass clazz, jdouble radius)
{
    (void) env;
    (void) clazz;
    pclmobile::filterUniformSampling(radius);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterRandomSample(
        JNIEnv* env, jclass clazz, jint sample, jint seed)
{
    (void) env;
    (void) clazz;
    pclmobile::filterRandomSample(sample, seed);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterFarthestPointSampling(
        JNIEnv* env, jclass clazz, jint sample, jint seed)
{
    (void) env;
    (void) clazz;
    pclmobile::filterFarthestPointSampling(sample, seed);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterStatisticalOutlierRemoval(
        JNIEnv* env, jclass clazz, jint meanK, jdouble stddevMulThresh)
{
    (void) env;
    (void) clazz;
    pclmobile::filterStatisticalOutlierRemoval(meanK, stddevMulThresh);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterRadiusOutlierRemoval(
        JNIEnv* env, jclass clazz, jdouble radius, jint minNeighbors)
{
    (void) env;
    (void) clazz;
    pclmobile::filterRadiusOutlierRemoval(radius, minNeighbors);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterCropBox(
        JNIEnv* env,
        jclass clazz,
        jdouble minX,
        jdouble minY,
        jdouble minZ,
        jdouble maxX,
        jdouble maxY,
        jdouble maxZ)
{
    (void) env;
    (void) clazz;
    pclmobile::filterCropBox(minX, minY, minZ, maxX, maxY, maxZ);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractPlaneInliers(
        JNIEnv* env, jclass clazz, jdouble distanceThreshold, jint maxIterations)
{
    (void) env;
    (void) clazz;
    pclmobile::extractPlaneInliers(distanceThreshold, maxIterations);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractModelInliers(
        JNIEnv* env, jclass clazz, jint modelType, jdouble distanceThreshold, jint maxIterations)
{
    (void) env;
    (void) clazz;
    pclmobile::extractModelInliers(modelType, distanceThreshold, maxIterations);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_geometry1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    LOGI("geometry1 compatibility sample: computeCentroidAndBounds values=%zu",
         pclmobile::computeCentroidAndBounds().size());
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_kdtree1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    logTupleCount("kdtree1", "KdTreeFLANN nearestKSearch",
                  pclmobile::nearestKSearch(0.0f, 0.0f, 0.0f, 8).size(), 4);
    logTupleCount("kdtree1", "KdTreeFLANN radiusSearch",
                  pclmobile::radiusSearch(0.0f, 0.0f, 0.0f, 0.28).size(), 4);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_keypoint1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    logTupleCount("keypoint1", "KdTree probe", pclmobile::nearestKSearch(0.0f, 0.0f, 0.0f, 8).size(), 4);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octree1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    logTupleCount("octree1", "OctreePointCloudSearch nearestKSearch",
                  pclmobile::octreeNearestKSearch(0.0f, 0.0f, 0.0f, 0.10, 8).size(), 4);
    logTupleCount("octree1", "OctreePointCloudSearch radiusSearch",
                  pclmobile::octreeRadiusSearch(0.0f, 0.0f, 0.0f, 0.10, 0.28).size(), 4);
    logTupleCount("octree1", "OctreePointCloudSearch voxelSearch",
                  pclmobile::octreeVoxelSearch(0.0f, 0.0f, 0.0f, 0.10).size(), 4);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_people1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    LOGI("people1 compatibility sample: EuclideanClusterExtraction clusters=%zu",
         pclmobile::extractEuclideanClusters(0.18, 20, 5000).size());
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_rangeimages1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    LOGI("rangeimages1 compatibility sample: computeCentroidAndBounds values=%zu",
         pclmobile::computeCentroidAndBounds().size());
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_recognition1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    LOGI("recognition1 compatibility sample: EuclideanClusterExtraction clusters=%zu",
         pclmobile::extractEuclideanClusters(0.18, 20, 5000).size());
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_registration1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    LOGI("registration1 compatibility sample: ICP values=%zu",
         pclmobile::alignToTranslatedCopyICP(0.05f, -0.03f, 0.02f, 35).size());
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_sampleconsensus1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    LOGI("sampleconsensus1 compatibility sample: segmentPlane values=%zu",
         pclmobile::segmentPlaneModel(0.03, 100).size());
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_segmentation1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    LOGI("segmentation1 compatibility sample: segmentPlane values=%zu clusters=%zu",
         pclmobile::segmentPlaneModel(0.03, 100).size(),
         pclmobile::extractEuclideanClusters(0.18, 20, 5000).size());
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_stereo1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    pclmobile::filterRadiusOutlierRemoval(0.18, 3);
    logPointCount("stereo1", "RadiusOutlierRemoval", pclmobile::filteredCloud()->points.size() * 3);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_surface1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    logPointCount("surface1", "ConvexHull", pclmobile::computeConvexHull()->points.size() * 3);
    logPointCount("surface1", "ConcaveHull", pclmobile::computeConcaveHull(0.18)->points.size() * 3);
    logPointCount("surface1", "MovingLeastSquares", pclmobile::smoothMovingLeastSquares(0.12)->points.size() * 3);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_tracking1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    LOGI("tracking1 compatibility sample: ICP values=%zu",
         pclmobile::alignToTranslatedCopyICP(0.05f, -0.03f, 0.02f, 35).size());
}

} // extern "C"
