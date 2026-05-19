#include <jni.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pcl_mobile_arrays.h"
#include "pcl_mobile_context.h"
#include "pcl_mobile_features.h"
#include "pcl_mobile_filters.h"
#include "pcl_mobile_io.h"
#include "pcl_mobile_keypoints.h"
#include "pcl_mobile_log.h"
#include "pcl_mobile_range_image.h"
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

jintArray makeIntArray(JNIEnv* env, const std::vector<int>& values)
{
    jintArray result = env->NewIntArray(static_cast<jsize>(values.size()));
    if (result != nullptr && !values.empty()) {
        std::vector<jint> jni_values;
        jni_values.reserve(values.size());
        for (int value : values) {
            jni_values.push_back(static_cast<jint>(value));
        }
        env->SetIntArrayRegion(
                result,
                0,
                static_cast<jsize>(jni_values.size()),
                jni_values.data());
    }
    return result;
}

std::vector<jfloat> readFloatArray(JNIEnv* env, jfloatArray values)
{
    if (values == nullptr) {
        return {};
    }

    const jsize length = env->GetArrayLength(values);
    std::vector<jfloat> result(static_cast<std::size_t>(length));
    if (length > 0) {
        env->GetFloatArrayRegion(values, 0, length, result.data());
    }
    return result;
}

std::vector<int> readIntArray(JNIEnv* env, jintArray values)
{
    if (values == nullptr) {
        return {};
    }

    const jsize length = env->GetArrayLength(values);
    std::vector<jint> jni_values(static_cast<std::size_t>(length));
    if (length > 0) {
        env->GetIntArrayRegion(values, 0, length, jni_values.data());
    }

    std::vector<int> result;
    result.reserve(jni_values.size());
    for (jint value : jni_values) {
        result.push_back(static_cast<int>(value));
    }
    return result;
}

std::vector<int> segmentModelInlierIndices(int model_type, double distance_threshold, int max_iterations)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    if (!pclmobile::segmentModel(model_type, distance_threshold, max_iterations, coefficients, inliers)) {
        return {};
    }
    return inliers->indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractLargestEuclideanCluster(
        double tolerance,
        int min_cluster_size,
        int max_cluster_size)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = pclmobile::activeCloud();
    pclmobile::clearFilteredCloud();
    if (input->empty() || tolerance <= 0.0) {
        return pclmobile::filteredCloud();
    }

    const int min_size = min_cluster_size > 0 ? min_cluster_size : 1;
    const int max_size = max_cluster_size >= min_size
            ? max_cluster_size
            : static_cast<int>(input->points.size());

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(tolerance);
    extraction.setMinClusterSize(min_size);
    extraction.setMaxClusterSize(max_size);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(input);
    extraction.extract(cluster_indices);

    if (cluster_indices.empty()) {
        LOGI("EuclideanClusterExtraction largest: input=%zu clusters=0 tolerance=%.3f",
             input->points.size(), tolerance);
        return pclmobile::filteredCloud();
    }

    const pcl::PointIndices* largest_cluster = &cluster_indices.front();
    for (const auto& cluster : cluster_indices) {
        if (cluster.indices.size() > largest_cluster->indices.size()) {
            largest_cluster = &cluster;
        }
    }

    pclmobile::filteredCloud()->points.reserve(largest_cluster->indices.size());
    for (int index : largest_cluster->indices) {
        if (index >= 0 && static_cast<std::size_t>(index) < input->points.size()) {
            pclmobile::filteredCloud()->points.push_back(input->points[static_cast<std::size_t>(index)]);
        }
    }
    pclmobile::filteredCloud()->width = static_cast<std::uint32_t>(pclmobile::filteredCloud()->points.size());
    pclmobile::filteredCloud()->height = 1;
    pclmobile::filteredCloud()->is_dense = input->is_dense;
    LOGI("EuclideanClusterExtraction largest: input=%zu clusters=%zu output=%zu tolerance=%.3f",
         input->points.size(),
         cluster_indices.size(),
         pclmobile::filteredCloud()->points.size(),
         tolerance);
    return pclmobile::filteredCloud();
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

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_loadOBJ(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    pclmobile::loadOBJFile(pclmobile::jstringToString(env, filename));
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_loadIFS(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    pclmobile::loadIFSFile(pclmobile::jstringToString(env, filename));
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_loadAuto(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    pclmobile::loadAutoFile(pclmobile::jstringToString(env, filename));
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_setCloudPoints(
        JNIEnv* env, jclass clazz, jfloatArray packedXYZ)
{
    (void) clazz;
    pclmobile::setCloudFromPackedXYZ(readFloatArray(env, packedXYZ));
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

JNIEXPORT jboolean JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_writeSourcePLYFile(
        JNIEnv* env, jclass clazz, jstring filename)
{
    (void) clazz;
    return pclmobile::writePLYFileASCII(pclmobile::jstringToString(env, filename), pclmobile::cloud())
            ? JNI_TRUE
            : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_writeFilteredPLYFile(
        JNIEnv* env, jclass clazz, jstring filename)
{
    (void) clazz;
    return pclmobile::writePLYFileASCII(pclmobile::jstringToString(env, filename), pclmobile::filteredCloud())
            ? JNI_TRUE
            : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_writeActiveIFSFile(
        JNIEnv* env, jclass clazz, jstring filename)
{
    (void) clazz;
    return pclmobile::writeIFSFile(pclmobile::jstringToString(env, filename), pclmobile::activeCloud())
            ? JNI_TRUE
            : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_writeSourceIFSFile(
        JNIEnv* env, jclass clazz, jstring filename)
{
    (void) clazz;
    return pclmobile::writeIFSFile(pclmobile::jstringToString(env, filename), pclmobile::cloud())
            ? JNI_TRUE
            : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_writeFilteredIFSFile(
        JNIEnv* env, jclass clazz, jstring filename)
{
    (void) clazz;
    return pclmobile::writeIFSFile(pclmobile::jstringToString(env, filename), pclmobile::filteredCloud())
            ? JNI_TRUE
            : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_writeActiveAutoFile(
        JNIEnv* env, jclass clazz, jstring filename)
{
    (void) clazz;
    return pclmobile::writeAutoFile(pclmobile::jstringToString(env, filename), pclmobile::activeCloud())
            ? JNI_TRUE
            : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_writeSourceAutoFile(
        JNIEnv* env, jclass clazz, jstring filename)
{
    (void) clazz;
    return pclmobile::writeAutoFile(pclmobile::jstringToString(env, filename), pclmobile::cloud())
            ? JNI_TRUE
            : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_writeFilteredAutoFile(
        JNIEnv* env, jclass clazz, jstring filename)
{
    (void) clazz;
    return pclmobile::writeAutoFile(pclmobile::jstringToString(env, filename), pclmobile::filteredCloud())
            ? JNI_TRUE
            : JNI_FALSE;
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeCentroidAndBounds(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeCentroidAndBounds());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeCovarianceMatrix(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeCovarianceMatrix());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeMeanAndCovarianceMatrix(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeMeanAndCovarianceMatrix());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computePrincipalAxes(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computePrincipalAxes());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeCentroidAndOBB(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeCentroidAndOBB());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeMomentOfInertiaAndOBB(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeMomentOfInertiaAndOBB());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeSquaredDistancesToPoint(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeSquaredDistancesToPoint(x, y, z));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_calculateActivePolygonArea(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::calculateActivePolygonArea());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeMaxDistanceFromCentroid(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeMaxDistanceFromCentroid());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_demeanActiveCloud(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makePointArray(env, pclmobile::demeanActiveCloud());
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

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_estimateNormalsOMP(
        JNIEnv* env, jclass clazz, jint kSearch, jint numberOfThreads)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::estimateNormalsOMP(kSearch, numberOfThreads));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_estimateLinearLeastSquaresNormals(
        JNIEnv* env,
        jclass clazz,
        jdouble normalSmoothingSize,
        jboolean depthDependentSmoothing,
        jdouble maxDepthChangeFactor)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::estimateLinearLeastSquaresNormals(
                    normalSmoothingSize,
                    depthDependentSmoothing == JNI_TRUE,
                    maxDepthChangeFactor));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_refineNormals(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jint refinementKSearch,
        jint maxIterations,
        jdouble convergenceThreshold)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::refineNormals(
                    normalKSearch,
                    refinementKSearch,
                    maxIterations,
                    convergenceThreshold));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeFPFHFeatures(
        JNIEnv* env, jclass clazz, jint normalKSearch, jdouble featureRadius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::computeFPFHFeatures(normalKSearch, featureRadius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeFPFHFeaturesOMP(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble featureRadius,
        jint numberOfThreads)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeFPFHFeaturesOMP(
                    normalKSearch,
                    featureRadius,
                    numberOfThreads));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computePFHFeatures(
        JNIEnv* env, jclass clazz, jint normalKSearch, jdouble featureRadius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::computePFHFeatures(normalKSearch, featureRadius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computePFHRGBFeatures(
        JNIEnv* env, jclass clazz, jint normalKSearch, jdouble featureRadius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::computePFHRGBFeatures(normalKSearch, featureRadius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeVFHFeatures(
        JNIEnv* env, jclass clazz, jint normalKSearch)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeVFHFeatures(normalKSearch));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeESFDescriptor(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeESFDescriptor());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeGASDDescriptor(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeGASDDescriptor());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeCRHDescriptor(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jfloat viewpointX,
        jfloat viewpointY,
        jfloat viewpointZ)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeCRHDescriptor(normalKSearch, viewpointX, viewpointY, viewpointZ));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeCVFHFeatures(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble clusterTolerance,
        jdouble epsAngleThreshold,
        jdouble curvatureThreshold,
        jint minPoints,
        jboolean normalizeBins)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeCVFHFeatures(
                    normalKSearch,
                    clusterTolerance,
                    epsAngleThreshold,
                    curvatureThreshold,
                    minPoints,
                    normalizeBins == JNI_TRUE));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeOURCVFHFeatures(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble clusterTolerance,
        jdouble epsAngleThreshold,
        jdouble curvatureThreshold,
        jint minPoints,
        jboolean normalizeBins,
        jdouble refineClusters,
        jdouble axisRatio,
        jdouble minAxisValue)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeOURCVFHFeatures(
                    normalKSearch,
                    clusterTolerance,
                    epsAngleThreshold,
                    curvatureThreshold,
                    minPoints,
                    normalizeBins == JNI_TRUE,
                    refineClusters,
                    axisRatio,
                    minAxisValue));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeIntensitySpinFeatures(
        JNIEnv* env,
        jclass clazz,
        jdouble radiusSearch,
        jdouble smoothingBandwidth)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeIntensitySpinFeatures(radiusSearch, smoothingBandwidth));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeIntensityGradientFeatures(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble radiusSearch,
        jint numberOfThreads)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeIntensityGradientFeatures(
                    normalKSearch,
                    radiusSearch,
                    numberOfThreads));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeRIFTFeatures(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble gradientRadius,
        jdouble featureRadius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeRIFTFeatures(
                    normalKSearch,
                    gradientRadius,
                    featureRadius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeROPSFeatures(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble supportRadius,
        jdouble meshSearchRadius,
        jdouble mu,
        jint maximumNearestNeighbors)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeROPSFeatures(
                    normalKSearch,
                    supportRadius,
                    meshSearchRadius,
                    mu,
                    maximumNearestNeighbors));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeShapeContext3DFeatures(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble searchRadius,
        jdouble minRadius,
        jdouble pointDensityRadius,
        jboolean random)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeShapeContext3DFeatures(
                    normalKSearch,
                    searchRadius,
                    minRadius,
                    pointDensityRadius,
                    random == JNI_TRUE));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeUniqueShapeContextFeatures(
        JNIEnv* env,
        jclass clazz,
        jdouble searchRadius,
        jdouble minRadius,
        jdouble pointDensityRadius,
        jdouble localRadius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeUniqueShapeContextFeatures(
                    searchRadius,
                    minRadius,
                    pointDensityRadius,
                    localRadius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computePPFFeatures(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jint maxPointCount)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computePPFFeatures(normalKSearch, maxPointCount));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computePPFRGBFeatures(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jint maxPointCount)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computePPFRGBFeatures(normalKSearch, maxPointCount));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeCPPFPairFeature(
        JNIEnv* env,
        jclass clazz,
        jint firstIndex,
        jint secondIndex,
        jint normalKSearch)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeCPPFPairFeatureValues(firstIndex, secondIndex, normalKSearch));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeNormalBasedSignatureFeatures(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble searchRadius,
        jdouble scale,
        jint n,
        jint m)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeNormalBasedSignatureFeatures(
                    normalKSearch,
                    searchRadius,
                    scale,
                    n,
                    m));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeSpinImageFeatures(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble featureRadius,
        jint imageWidth,
        jdouble supportAngleCos,
        jint minPointCount)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeSpinImageFeatures(
                    normalKSearch,
                    featureRadius,
                    imageWidth,
                    supportAngleCos,
                    minPointCount));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeGRSDDescriptor(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble radiusSearch,
        jdouble planeRadius,
        jint subdivisions)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeGRSDDescriptor(normalKSearch, radiusSearch, planeRadius, subdivisions));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeMomentInvariants(
        JNIEnv* env, jclass clazz, jdouble radiusSearch)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeMomentInvariants(radiusSearch));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeRSDFeatures(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble radiusSearch,
        jdouble planeRadius,
        jint subdivisions)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeRSDFeatures(
                    normalKSearch,
                    radiusSearch,
                    planeRadius,
                    subdivisions));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computePrincipalCurvatures(
        JNIEnv* env, jclass clazz, jint normalKSearch, jint curvatureKSearch)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::computePrincipalCurvatures(normalKSearch, curvatureKSearch));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeSHOTFeatures(
        JNIEnv* env, jclass clazz, jint normalKSearch, jdouble featureRadius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::computeSHOTFeatures(normalKSearch, featureRadius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeSHOTFeaturesOMP(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble featureRadius,
        jint numberOfThreads)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeSHOTFeaturesOMP(normalKSearch, featureRadius, numberOfThreads));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeSHOTLocalReferenceFrames(
        JNIEnv* env,
        jclass clazz,
        jdouble radiusSearch,
        jboolean useOmp,
        jint numberOfThreads)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeSHOTLocalReferenceFrames(
                    radiusSearch,
                    useOmp == JNI_TRUE,
                    numberOfThreads));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeBOARDLocalReferenceFrames(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble radiusSearch,
        jdouble tangentRadius,
        jboolean findHoles,
        jdouble marginThreshold)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeBOARDLocalReferenceFrames(
                    normalKSearch,
                    radiusSearch,
                    tangentRadius,
                    findHoles == JNI_TRUE,
                    marginThreshold));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeFLARELocalReferenceFrames(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble radiusSearch,
        jdouble tangentRadius,
        jdouble marginThreshold,
        jint minNeighborsForNormalAxis,
        jint minNeighborsForTangentAxis)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeFLARELocalReferenceFrames(
                    normalKSearch,
                    radiusSearch,
                    tangentRadius,
                    marginThreshold,
                    minNeighborsForNormalAxis,
                    minNeighborsForTangentAxis));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeBoundaryPoints(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble radiusSearch,
        jdouble angleThresholdDegrees)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeBoundaryPoints(
                    normalKSearch,
                    radiusSearch,
                    angleThresholdDegrees));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeDifferenceOfNormals(
        JNIEnv* env, jclass clazz, jdouble smallRadius, jdouble largeRadius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::computeDifferenceOfNormals(smallRadius, largeRadius));
}

JNIEXPORT jintArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractStatisticalMultiscaleInterestRegionIndices(
        JNIEnv* env,
        jclass clazz,
        jdouble firstScale,
        jdouble secondScale,
        jdouble thirdScale,
        jint maxPointCount)
{
    (void) clazz;
    return makeIntArray(
            env,
            pclmobile::extractStatisticalMultiscaleInterestRegionIndices(
                    firstScale,
                    secondScale,
                    thirdScale,
                    maxPointCount));
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

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_segmentSACModelWithMethod(
        JNIEnv* env,
        jclass clazz,
        jint modelType,
        jint methodType,
        jdouble distanceThreshold,
        jint maxIterations)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::segmentSACModelWithMethod(
                    modelType,
                    methodType,
                    distanceThreshold,
                    maxIterations));
}

JNIEXPORT jintArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_segmentSACModelInlierIndices(
        JNIEnv* env, jclass clazz, jint modelType, jdouble distanceThreshold, jint maxIterations)
{
    (void) clazz;
    return makeIntArray(env, segmentModelInlierIndices(modelType, distanceThreshold, maxIterations));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_nearestKSearch(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jint k)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::nearestKSearch(x, y, z, k));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_nearestKSearchIndices(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jint k)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::nearestKSearchIndices(x, y, z, k));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_radiusSearch(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble radius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::radiusSearch(x, y, z, radius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_radiusSearchIndices(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble radius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::radiusSearchIndices(x, y, z, radius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_radiusSearchLimited(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble radius, jint maxNeighbors)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::radiusSearchLimited(x, y, z, radius, maxNeighbors));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_radiusSearchIndicesLimited(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble radius, jint maxNeighbors)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::radiusSearchIndicesLimited(x, y, z, radius, maxNeighbors));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeNearestKSearch(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble resolution, jint k)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::octreeNearestKSearch(x, y, z, resolution, k));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeNearestKSearchIndices(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble resolution, jint k)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::octreeNearestKSearchIndices(x, y, z, resolution, k));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeRadiusSearch(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble resolution, jdouble radius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::octreeRadiusSearch(x, y, z, resolution, radius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeRadiusSearchIndices(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble resolution, jdouble radius)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::octreeRadiusSearchIndices(x, y, z, resolution, radius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeRadiusSearchLimited(
        JNIEnv* env,
        jclass clazz,
        jfloat x,
        jfloat y,
        jfloat z,
        jdouble resolution,
        jdouble radius,
        jint maxNeighbors)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::octreeRadiusSearchLimited(
                    x, y, z, resolution, radius, maxNeighbors));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeRadiusSearchIndicesLimited(
        JNIEnv* env,
        jclass clazz,
        jfloat x,
        jfloat y,
        jfloat z,
        jdouble resolution,
        jdouble radius,
        jint maxNeighbors)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::octreeRadiusSearchIndicesLimited(
                    x, y, z, resolution, radius, maxNeighbors));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeVoxelSearch(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble resolution)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::octreeVoxelSearch(x, y, z, resolution));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeVoxelSearchIndices(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble resolution)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::octreeVoxelSearchIndices(x, y, z, resolution));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeApproxNearestSearch(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble resolution)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::octreeApproxNearestSearch(x, y, z, resolution));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octreeApproxNearestSearchIndex(
        JNIEnv* env, jclass clazz, jfloat x, jfloat y, jfloat z, jdouble resolution)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::octreeApproxNearestSearchIndex(x, y, z, resolution));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractEuclideanClusters(
        JNIEnv* env, jclass clazz, jdouble tolerance, jint minClusterSize, jint maxClusterSize)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::extractEuclideanClusters(tolerance, minClusterSize, maxClusterSize));
}

JNIEXPORT jintArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractEuclideanClusterIndices(
        JNIEnv* env, jclass clazz, jdouble tolerance, jint minClusterSize, jint maxClusterSize)
{
    (void) clazz;
    return makeIntArray(
            env, pclmobile::extractEuclideanClusterIndices(tolerance, minClusterSize, maxClusterSize));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractRegionGrowingClusters(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jint numberOfNeighbours,
        jint minClusterSize,
        jint maxClusterSize,
        jdouble smoothnessThresholdDegrees,
        jdouble curvatureThreshold)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::extractRegionGrowingClusters(
                    normalKSearch,
                    numberOfNeighbours,
                    minClusterSize,
                    maxClusterSize,
                    smoothnessThresholdDegrees,
                    curvatureThreshold));
}

JNIEXPORT jintArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractRegionGrowingClusterIndices(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jint numberOfNeighbours,
        jint minClusterSize,
        jint maxClusterSize,
        jdouble smoothnessThresholdDegrees,
        jdouble curvatureThreshold)
{
    (void) clazz;
    return makeIntArray(
            env,
            pclmobile::extractRegionGrowingClusterIndices(
                    normalKSearch,
                    numberOfNeighbours,
                    minClusterSize,
                    maxClusterSize,
                    smoothnessThresholdDegrees,
                    curvatureThreshold));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractConditionalEuclideanClusters(
        JNIEnv* env,
        jclass clazz,
        jdouble tolerance,
        jint minClusterSize,
        jint maxClusterSize,
        jdouble maxZDelta)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::extractConditionalEuclideanClusters(
                    tolerance, minClusterSize, maxClusterSize, maxZDelta));
}

JNIEXPORT jintArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractConditionalEuclideanClusterIndices(
        JNIEnv* env,
        jclass clazz,
        jdouble tolerance,
        jint minClusterSize,
        jint maxClusterSize,
        jdouble maxZDelta)
{
    (void) clazz;
    return makeIntArray(
            env,
            pclmobile::extractConditionalEuclideanClusterIndices(
                    tolerance, minClusterSize, maxClusterSize, maxZDelta));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractPolygonalPrismData(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedPlanarHullXYZ,
        jdouble heightMin,
        jdouble heightMax,
        jfloat viewPointX,
        jfloat viewPointY,
        jfloat viewPointZ,
        jboolean negative)
{
    (void) clazz;
    return pclmobile::makePointArray(
            env,
            pclmobile::extractPolygonalPrismData(
                    readFloatArray(env, packedPlanarHullXYZ),
                    heightMin,
                    heightMax,
                    viewPointX,
                    viewPointY,
                    viewPointZ,
                    negative == JNI_TRUE));
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractProgressiveMorphologicalGround(
        JNIEnv* env,
        jclass clazz,
        jint maxWindowSize,
        jdouble slope,
        jdouble initialDistance,
        jdouble maxDistance,
        jdouble cellSize,
        jdouble base,
        jboolean exponential,
        jboolean negative)
{
    (void) env;
    (void) clazz;
    pclmobile::extractProgressiveMorphologicalGround(
            maxWindowSize,
            slope,
            initialDistance,
            maxDistance,
            cellSize,
            base,
            exponential == JNI_TRUE,
            negative == JNI_TRUE);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractApproximateProgressiveMorphologicalGround(
        JNIEnv* env,
        jclass clazz,
        jint maxWindowSize,
        jdouble slope,
        jdouble initialDistance,
        jdouble maxDistance,
        jdouble cellSize,
        jdouble base,
        jboolean exponential,
        jint numberOfThreads,
        jboolean negative)
{
    (void) env;
    (void) clazz;
    pclmobile::extractApproximateProgressiveMorphologicalGround(
            maxWindowSize,
            slope,
            initialDistance,
            maxDistance,
            cellSize,
            base,
            exponential == JNI_TRUE,
            numberOfThreads,
            negative == JNI_TRUE);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractLargestEuclideanCluster(
        JNIEnv* env, jclass clazz, jdouble tolerance, jint minClusterSize, jint maxClusterSize)
{
    (void) env;
    (void) clazz;
    extractLargestEuclideanCluster(tolerance, minClusterSize, maxClusterSize);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractMinCutForeground(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedForegroundXYZ,
        jdouble sigma,
        jdouble radius,
        jdouble sourceWeight,
        jint numberOfNeighbours)
{
    (void) clazz;
    pclmobile::extractMinCutForeground(
            readFloatArray(env, packedForegroundXYZ),
            sigma,
            radius,
            sourceWeight,
            numberOfNeighbours);
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractMinCutForegroundStats(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedForegroundXYZ,
        jdouble sigma,
        jdouble radius,
        jdouble sourceWeight,
        jint numberOfNeighbours)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::extractMinCutForegroundStats(
                    readFloatArray(env, packedForegroundXYZ),
                    sigma,
                    radius,
                    sourceWeight,
                    numberOfNeighbours));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_segmentDifferencesAgainstTarget(
        JNIEnv* env, jclass clazz, jfloatArray packedTargetXYZ, jdouble distanceThreshold)
{
    (void) clazz;
    return pclmobile::makePointArray(
            env,
            pclmobile::segmentDifferencesAgainstTarget(
                    readFloatArray(env, packedTargetXYZ),
                    distanceThreshold));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeISSKeypoints(
        JNIEnv* env,
        jclass clazz,
        jdouble salientRadius,
        jdouble nonMaxRadius,
        jdouble threshold21,
        jdouble threshold32,
        jint minNeighbors)
{
    (void) clazz;
    return pclmobile::makePointArray(
            env,
            pclmobile::computeISSKeypoints(
                    salientRadius,
                    nonMaxRadius,
                    threshold21,
                    threshold32,
                    minNeighbors));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeSIFTKeypoints(
        JNIEnv* env,
        jclass clazz,
        jdouble minScale,
        jint nrOctaves,
        jint nrScalesPerOctave,
        jdouble minContrast)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeSIFTKeypoints(
                    minScale,
                    nrOctaves,
                    nrScalesPerOctave,
                    minContrast));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeHarrisKeypoints(
        JNIEnv* env,
        jclass clazz,
        jint responseMethod,
        jdouble radius,
        jdouble threshold,
        jboolean nonMaxSuppression,
        jboolean refine)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeHarrisKeypoints(
                    responseMethod,
                    radius,
                    threshold,
                    nonMaxSuppression == JNI_TRUE,
                    refine == JNI_TRUE));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeHarris6DKeypoints(
        JNIEnv* env,
        jclass clazz,
        jdouble radius,
        jdouble threshold,
        jboolean nonMaxSuppression,
        jboolean refine,
        jint numberOfThreads)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeHarris6DKeypoints(
                    radius,
                    threshold,
                    nonMaxSuppression == JNI_TRUE,
                    refine == JNI_TRUE,
                    numberOfThreads));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeHarris2DKeypoints(
        JNIEnv* env,
        jclass clazz,
        jint responseMethod,
        jint windowWidth,
        jint windowHeight,
        jint minDistance,
        jdouble threshold,
        jboolean nonMaxSuppression,
        jboolean refine)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeHarris2DKeypoints(
                    responseMethod,
                    windowWidth,
                    windowHeight,
                    minDistance,
                    threshold,
                    nonMaxSuppression == JNI_TRUE,
                    refine == JNI_TRUE));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeSUSANKeypoints(
        JNIEnv* env,
        jclass clazz,
        jdouble radius,
        jdouble distanceThreshold,
        jdouble angularThreshold,
        jdouble intensityThreshold,
        jboolean nonMaxSuppression,
        jboolean geometricValidation)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeSUSANKeypoints(
                    radius,
                    distanceThreshold,
                    angularThreshold,
                    intensityThreshold,
                    nonMaxSuppression == JNI_TRUE,
                    geometricValidation == JNI_TRUE));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeTrajkovicKeypoints(
        JNIEnv* env,
        jclass clazz,
        jint method,
        jint windowSize,
        jdouble firstThreshold,
        jdouble secondThreshold,
        jint normalKSearch)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeTrajkovicKeypoints(
                    method,
                    windowSize,
                    firstThreshold,
                    secondThreshold,
                    normalKSearch));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeTrajkovic2DKeypoints(
        JNIEnv* env,
        jclass clazz,
        jint method,
        jint windowSize,
        jdouble firstThreshold,
        jdouble secondThreshold)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeTrajkovic2DKeypoints(
                    method,
                    windowSize,
                    firstThreshold,
                    secondThreshold));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeBRISK2DKeypoints(
        JNIEnv* env,
        jclass clazz,
        jint threshold,
        jint octaves,
        jboolean removeInvalid3DKeypoints)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeBRISK2DKeypoints(
                    threshold,
                    octaves,
                    removeInvalid3DKeypoints == JNI_TRUE));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeAGAST2DKeypoints(
        JNIEnv* env,
        jclass clazz,
        jdouble threshold,
        jdouble maxDataValue,
        jboolean nonMaxSuppression,
        jint maxKeypoints)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeAGAST2DKeypoints(
                    threshold,
                    maxDataValue,
                    nonMaxSuppression == JNI_TRUE,
                    maxKeypoints));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeUniformSamplingKeypoints(
        JNIEnv* env, jclass clazz, jdouble radius)
{
    (void) clazz;
    return pclmobile::makePointArray(env, pclmobile::computeUniformSamplingKeypoints(radius));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeSmoothedSurfacesKeypoints(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble inputScale,
        jdouble firstSmoothedScale,
        jdouble secondSmoothedScale,
        jdouble neighborhoodConstant)
{
    (void) clazz;
    return pclmobile::makePointArray(
            env,
            pclmobile::computeSmoothedSurfacesKeypoints(
                    normalKSearch,
                    inputScale,
                    firstSmoothedScale,
                    secondSmoothedScale,
                    neighborhoodConstant));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeRangeImageFromActiveCloud(
        JNIEnv* env,
        jclass clazz,
        jfloat angularResolutionDegrees,
        jfloat maxAngleWidthDegrees,
        jfloat maxAngleHeightDegrees,
        jfloat sensorX,
        jfloat sensorY,
        jfloat sensorZ,
        jfloat minRange)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeRangeImageFromActiveCloud(
                    angularResolutionDegrees,
                    maxAngleWidthDegrees,
                    maxAngleHeightDegrees,
                    sensorX,
                    sensorY,
                    sensorZ,
                    minRange));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeSphericalRangeImageFromActiveCloud(
        JNIEnv* env,
        jclass clazz,
        jfloat angularResolutionDegrees,
        jfloat maxAngleWidthDegrees,
        jfloat maxAngleHeightDegrees,
        jfloat sensorX,
        jfloat sensorY,
        jfloat sensorZ,
        jfloat minRange)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeSphericalRangeImageFromActiveCloud(
                    angularResolutionDegrees,
                    maxAngleWidthDegrees,
                    maxAngleHeightDegrees,
                    sensorX,
                    sensorY,
                    sensorZ,
                    minRange));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computePlanarRangeImageFromActiveCloud(
        JNIEnv* env,
        jclass clazz,
        jint imageWidth,
        jint imageHeight,
        jfloat centerX,
        jfloat centerY,
        jfloat focalLengthX,
        jfloat focalLengthY,
        jfloat sensorX,
        jfloat sensorY,
        jfloat sensorZ,
        jfloat minRange)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computePlanarRangeImageFromActiveCloud(
                    imageWidth,
                    imageHeight,
                    centerX,
                    centerY,
                    focalLengthX,
                    focalLengthY,
                    sensorX,
                    sensorY,
                    sensorZ,
                    minRange));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeRangeImageBorderDescriptionsFromActiveCloud(
        JNIEnv* env,
        jclass clazz,
        jfloat angularResolutionDegrees,
        jfloat maxAngleWidthDegrees,
        jfloat maxAngleHeightDegrees,
        jfloat sensorX,
        jfloat sensorY,
        jfloat sensorZ,
        jfloat minRange,
        jint maxNoOfThreads,
        jint pixelRadiusBorders)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeRangeImageBorderDescriptionsFromActiveCloud(
                    angularResolutionDegrees,
                    maxAngleWidthDegrees,
                    maxAngleHeightDegrees,
                    sensorX,
                    sensorY,
                    sensorZ,
                    minRange,
                    maxNoOfThreads,
                    pixelRadiusBorders));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeNARFDescriptorsFromActiveCloud(
        JNIEnv* env,
        jclass clazz,
        jfloat angularResolutionDegrees,
        jfloat maxAngleWidthDegrees,
        jfloat maxAngleHeightDegrees,
        jfloat sensorX,
        jfloat sensorY,
        jfloat sensorZ,
        jfloat minRange,
        jfloat supportSize,
        jboolean rotationInvariant,
        jint maxDescriptorCount)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeNARFDescriptorsFromActiveCloud(
                    angularResolutionDegrees,
                    maxAngleWidthDegrees,
                    maxAngleHeightDegrees,
                    sensorX,
                    sensorY,
                    sensorZ,
                    minRange,
                    supportSize,
                    rotationInvariant == JNI_TRUE,
                    maxDescriptorCount));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeNARFKeypointsFromActiveCloud(
        JNIEnv* env,
        jclass clazz,
        jfloat angularResolutionDegrees,
        jfloat maxAngleWidthDegrees,
        jfloat maxAngleHeightDegrees,
        jfloat sensorX,
        jfloat sensorY,
        jfloat sensorZ,
        jfloat minRange,
        jfloat supportSize,
        jint maxKeypointCount,
        jfloat minInterestValue,
        jboolean nonMaximumSuppression)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeNARFKeypointsFromActiveCloud(
                    angularResolutionDegrees,
                    maxAngleWidthDegrees,
                    maxAngleHeightDegrees,
                    sensorX,
                    sensorY,
                    sensorZ,
                    minRange,
                    supportSize,
                    maxKeypointCount,
                    minInterestValue,
                    nonMaximumSuppression == JNI_TRUE));
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

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeConvexHullMesh(
        JNIEnv* env, jclass clazz)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeConvexHullMesh());
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeConcaveHullMesh(
        JNIEnv* env, jclass clazz, jdouble alpha)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::computeConcaveHullMesh(alpha));
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

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_smoothSurfelSmoothing(
        JNIEnv* env, jclass clazz, jint normalKSearch, jdouble scale)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::smoothSurfelSmoothing(normalKSearch, scale));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_reconstructGreedyProjectionTriangles(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble searchRadius,
        jdouble mu,
        jint maximumNearestNeighbors,
        jdouble maximumSurfaceAngle,
        jdouble minimumAngle,
        jdouble maximumAngle,
        jboolean normalConsistency)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::reconstructGreedyProjectionTriangles(
                    normalKSearch,
                    searchRadius,
                    mu,
                    maximumNearestNeighbors,
                    maximumSurfaceAngle,
                    minimumAngle,
                    maximumAngle,
                    normalConsistency == JNI_TRUE));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_reconstructGridProjectionMesh(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jdouble resolution,
        jint paddingSize,
        jint nearestNeighborCount,
        jint maxBinarySearchLevel)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::reconstructGridProjectionMesh(
                    normalKSearch,
                    resolution,
                    paddingSize,
                    nearestNeighborCount,
                    maxBinarySearchLevel));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_reconstructMarchingCubesHoppeMesh(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jint resolutionX,
        jint resolutionY,
        jint resolutionZ,
        jdouble percentageExtendGrid,
        jdouble isoLevel,
        jdouble distanceIgnore)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::reconstructMarchingCubesHoppeMesh(
                    normalKSearch,
                    resolutionX,
                    resolutionY,
                    resolutionZ,
                    percentageExtendGrid,
                    isoLevel,
                    distanceIgnore));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_reconstructMarchingCubesRBFMesh(
        JNIEnv* env,
        jclass clazz,
        jint normalKSearch,
        jint resolutionX,
        jint resolutionY,
        jint resolutionZ,
        jdouble offSurfaceDisplacement,
        jdouble percentageExtendGrid,
        jdouble isoLevel)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::reconstructMarchingCubesRBFMesh(
                    normalKSearch,
                    resolutionX,
                    resolutionY,
                    resolutionZ,
                    offSurfaceDisplacement,
                    percentageExtendGrid,
                    isoLevel));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_reconstructOrganizedFastMeshPolygons(
        JNIEnv* env,
        jclass clazz,
        jint triangulationType,
        jint trianglePixelSize,
        jdouble maxEdgeLengthA,
        jdouble maxEdgeLengthB,
        jdouble maxEdgeLengthC,
        jdouble angleTolerance,
        jdouble distanceTolerance,
        jboolean distanceDependent,
        jboolean useDepthAsDistance,
        jboolean storeShadowedFaces)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::reconstructOrganizedFastMeshPolygons(
                    triangulationType,
                    trianglePixelSize,
                    maxEdgeLengthA,
                    maxEdgeLengthB,
                    maxEdgeLengthC,
                    angleTolerance,
                    distanceTolerance,
                    distanceDependent == JNI_TRUE,
                    useDepthAsDistance == JNI_TRUE,
                    storeShadowedFaces == JNI_TRUE));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_alignToTranslatedCopyICP(
        JNIEnv* env, jclass clazz, jfloat tx, jfloat ty, jfloat tz, jint maxIterations)
{
    (void) clazz;
    return pclmobile::makeFloatArray(env, pclmobile::alignToTranslatedCopyICP(tx, ty, tz, maxIterations));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_estimateRigidTransformSVD(
        JNIEnv* env, jclass clazz, jfloatArray packedTargetXYZ)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::estimateRigidTransformSVD(readFloatArray(env, packedTargetXYZ)));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_estimateRigidTransformSVDScale(
        JNIEnv* env, jclass clazz, jfloatArray packedTargetXYZ)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::estimateRigidTransformSVDScale(readFloatArray(env, packedTargetXYZ)));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_estimateRigidTransform3Point(
        JNIEnv* env, jclass clazz, jfloatArray packedTargetXYZ)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::estimateRigidTransform3Point(readFloatArray(env, packedTargetXYZ)));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_estimateRigidTransformDualQuaternion(
        JNIEnv* env, jclass clazz, jfloatArray packedTargetXYZ)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::estimateRigidTransformDualQuaternion(readFloatArray(env, packedTargetXYZ)));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_estimateRigidTransformLM(
        JNIEnv* env, jclass clazz, jfloatArray packedTargetXYZ)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::estimateRigidTransformLM(readFloatArray(env, packedTargetXYZ)));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_estimateRigidTransform2D(
        JNIEnv* env, jclass clazz, jfloatArray packedTargetXYZ)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::estimateRigidTransform2D(readFloatArray(env, packedTargetXYZ)));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_findCorrespondences(
        JNIEnv* env, jclass clazz, jfloatArray packedTargetXYZ, jdouble maxDistance)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::findCorrespondences(
                    readFloatArray(env, packedTargetXYZ),
                    maxDistance,
                    false));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_findReciprocalCorrespondences(
        JNIEnv* env, jclass clazz, jfloatArray packedTargetXYZ, jdouble maxDistance)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::findCorrespondences(
                    readFloatArray(env, packedTargetXYZ),
                    maxDistance,
                    true));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_rejectCorrespondencesDistance(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedCorrespondences,
        jfloatArray packedTargetXYZ,
        jdouble maxDistance)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::rejectCorrespondencesDistance(
                    readFloatArray(env, packedCorrespondences),
                    readFloatArray(env, packedTargetXYZ),
                    maxDistance));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_rejectCorrespondencesOneToOne(
        JNIEnv* env, jclass clazz, jfloatArray packedCorrespondences)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::rejectCorrespondencesOneToOne(readFloatArray(env, packedCorrespondences)));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_rejectCorrespondencesMedianDistance(
        JNIEnv* env, jclass clazz, jfloatArray packedCorrespondences, jdouble medianFactor)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::rejectCorrespondencesMedianDistance(
                    readFloatArray(env, packedCorrespondences),
                    medianFactor));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_rejectCorrespondencesTrimmed(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedCorrespondences,
        jdouble overlapRatio,
        jint minCorrespondences)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::rejectCorrespondencesTrimmed(
                    readFloatArray(env, packedCorrespondences),
                    overlapRatio,
                    minCorrespondences));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_rejectCorrespondencesVarTrimmed(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedCorrespondences,
        jdouble minRatio,
        jdouble maxRatio)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::rejectCorrespondencesVarTrimmed(
                    readFloatArray(env, packedCorrespondences),
                    minRatio,
                    maxRatio));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_rejectCorrespondencesSampleConsensus(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedCorrespondences,
        jfloatArray packedTargetXYZ,
        jdouble inlierThreshold,
        jint maxIterations,
        jboolean refineModel)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::rejectCorrespondencesSampleConsensus(
                    readFloatArray(env, packedCorrespondences),
                    readFloatArray(env, packedTargetXYZ),
                    inlierThreshold,
                    maxIterations,
                    refineModel == JNI_TRUE));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_rejectCorrespondencesPoly(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedCorrespondences,
        jfloatArray packedTargetXYZ,
        jint cardinality,
        jdouble similarityThreshold,
        jint iterations)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::rejectCorrespondencesPoly(
                    readFloatArray(env, packedCorrespondences),
                    readFloatArray(env, packedTargetXYZ),
                    cardinality,
                    similarityThreshold,
                    iterations));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_validateTransformEuclidean(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedTargetXYZ,
        jfloatArray rowMajor4x4,
        jdouble maxRange,
        jdouble threshold)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::validateTransformEuclidean(
                    readFloatArray(env, packedTargetXYZ),
                    readFloatArray(env, rowMajor4x4),
                    maxRange,
                    threshold));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_transformActiveCloud(
        JNIEnv* env, jclass clazz, jfloatArray rowMajor4x4)
{
    (void) clazz;
    return pclmobile::makePointArray(env, pclmobile::transformActiveCloud(readFloatArray(env, rowMajor4x4)));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_translateActiveCloud(
        JNIEnv* env, jclass clazz, jfloat tx, jfloat ty, jfloat tz)
{
    (void) clazz;
    return pclmobile::makePointArray(env, pclmobile::translateActiveCloud(tx, ty, tz));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_alignToTargetICP(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedTargetXYZ,
        jint maxIterations,
        jdouble maxCorrespondenceDistance,
        jdouble transformationEpsilon,
        jdouble euclideanFitnessEpsilon)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::alignToTargetICP(
                    readFloatArray(env, packedTargetXYZ),
                    maxIterations,
                    maxCorrespondenceDistance,
                    transformationEpsilon,
                    euclideanFitnessEpsilon));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_alignToTargetGICP(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedTargetXYZ,
        jint maxIterations,
        jdouble maxCorrespondenceDistance,
        jdouble transformationEpsilon,
        jdouble rotationEpsilon,
        jint maxOptimizerIterations)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::alignToTargetGICP(
                    readFloatArray(env, packedTargetXYZ),
                    maxIterations,
                    maxCorrespondenceDistance,
                    transformationEpsilon,
                    rotationEpsilon,
                    maxOptimizerIterations));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_alignToTargetICPNonLinear(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedTargetXYZ,
        jint maxIterations,
        jdouble maxCorrespondenceDistance,
        jdouble transformationEpsilon,
        jdouble euclideanFitnessEpsilon)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::alignToTargetICPNonLinear(
                    readFloatArray(env, packedTargetXYZ),
                    maxIterations,
                    maxCorrespondenceDistance,
                    transformationEpsilon,
                    euclideanFitnessEpsilon));
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_alignToTargetNDT(
        JNIEnv* env,
        jclass clazz,
        jfloatArray packedTargetXYZ,
        jint maxIterations,
        jdouble resolution,
        jdouble stepSize,
        jdouble transformationEpsilon,
        jint minPointsPerVoxel)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::alignToTargetNDT(
                    readFloatArray(env, packedTargetXYZ),
                    maxIterations,
                    resolution,
                    stepSize,
                    transformationEpsilon,
                    minPointsPerVoxel));
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_feature1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    logTupleCount("feature1", "NormalEstimation", pclmobile::estimateNormals(16).size(), 4);
    logTupleCount("feature1", "NormalEstimationOMP",
                  pclmobile::estimateNormalsOMP(16, 0).size(), 4);
    logTupleCount("feature1", "NormalRefinement",
                  pclmobile::refineNormals(16, 16, 10, 0.00001).size(), 4);
    logTupleCount("feature1", "PFHEstimation",
                  pclmobile::computePFHFeatures(16, 0.18).size(), 125);
    logTupleCount("feature1", "FPFHEstimation",
                  pclmobile::computeFPFHFeatures(16, 0.18).size(), 33);
    logTupleCount("feature1", "FPFHEstimationOMP",
                  pclmobile::computeFPFHFeaturesOMP(16, 0.18, 0).size(), 33);
    logTupleCount("feature1", "VFHEstimation",
                  pclmobile::computeVFHFeatures(16).size(), 308);
    logTupleCount("feature1", "ESFEstimation",
                  pclmobile::computeESFDescriptor().size(), 640);
    logTupleCount("feature1", "GASDEstimation",
                  pclmobile::computeGASDDescriptor().size(), 512);
    logTupleCount("feature1", "CRHEstimation",
                  pclmobile::computeCRHDescriptor(16, 0.0f, 0.0f, 0.0f).size(), 90);
    logTupleCount("feature1", "CVFHEstimation",
                  pclmobile::computeCVFHFeatures(16, 0.0, 0.0, 0.03, 20, true).size(), 308);
    logTupleCount("feature1", "OURCVFHEstimation",
                  pclmobile::computeOURCVFHFeatures(16, 0.0, 0.0, 0.03, 20, true, 1.0, 0.8, 0.925).size(), 308);
    logTupleCount("feature1", "IntensitySpinEstimation",
                  pclmobile::computeIntensitySpinFeatures(0.18, 1.0).size(), 20);
    logTupleCount("feature1", "IntensityGradientEstimation",
                  pclmobile::computeIntensityGradientFeatures(16, 0.18, 0).size(), 3);
    logTupleCount("feature1", "RIFTEstimation",
                  pclmobile::computeRIFTFeatures(16, 0.18, 0.24).size(), 32);
    logTupleCount("feature1", "ROPSEstimation",
                  pclmobile::computeROPSFeatures(16, 0.18, 0.18, 2.5, 100).size(), 135);
    logTupleCount("feature1", "ShapeContext3DEstimation",
                  pclmobile::computeShapeContext3DFeatures(16, 0.18, 0.02, 0.04, false).size(), 1980);
    logTupleCount("feature1", "UniqueShapeContext",
                  pclmobile::computeUniqueShapeContextFeatures(0.18, 0.02, 0.04, 0.18).size(), 1960);
    logTupleCount("feature1", "PPFEstimation",
                  pclmobile::computePPFFeatures(16, 256).size(), 5);
    logTupleCount("feature1", "computeCPPFPairFeature",
                  pclmobile::computeCPPFPairFeatureValues(0, 1, 16).size(), 10);
    logTupleCount("feature1", "NormalBasedSignatureEstimation",
                  pclmobile::computeNormalBasedSignatureFeatures(16, 0.18, 0.06, 36, 8).size(), 12);
    logTupleCount("feature1", "SpinImageEstimation",
                  pclmobile::computeSpinImageFeatures(16, 0.18, 8, 0.0, 0).size(), 153);
    logTupleCount("feature1", "GRSDEstimation",
                  pclmobile::computeGRSDDescriptor(16, 0.18, 0.06, 5).size(), 21);
    logTupleCount("feature1", "MomentInvariantsEstimation",
                  pclmobile::computeMomentInvariants(0.18).size(), 3);
    logTupleCount("feature1", "RSDEstimation",
                  pclmobile::computeRSDFeatures(16, 0.18, 0.06, 5).size(), 2);
    logTupleCount("feature1", "PrincipalCurvaturesEstimation",
                  pclmobile::computePrincipalCurvatures(16, 16).size(), 5);
    logTupleCount("feature1", "SHOTEstimation",
                  pclmobile::computeSHOTFeatures(16, 0.18).size(), 352);
    logTupleCount("feature1", "SHOTEstimationOMP",
                  pclmobile::computeSHOTFeaturesOMP(16, 0.18, 0).size(), 352);
    logTupleCount("feature1", "SHOTLocalReferenceFrameEstimation",
                  pclmobile::computeSHOTLocalReferenceFrames(0.18, true, 0).size(), 9);
    logTupleCount("feature1", "BOARDLocalReferenceFrameEstimation",
                  pclmobile::computeBOARDLocalReferenceFrames(16, 0.18, 0.0, true, 0.85).size(), 9);
    logTupleCount("feature1", "FLARELocalReferenceFrameEstimation",
                  pclmobile::computeFLARELocalReferenceFrames(16, 0.18, 0.0, 0.85, 5, 5).size(), 9);
    logTupleCount("feature1", "BoundaryEstimation",
                  pclmobile::computeBoundaryPoints(16, 0.18, 90.0).size(), 4);
    logTupleCount("feature1", "DifferenceOfNormalsEstimation",
                  pclmobile::computeDifferenceOfNormals(0.08, 0.20).size(), 4);
    LOGI("feature1 compatibility sample: StatisticalMultiscaleInterestRegionExtraction values=%zu",
         pclmobile::extractStatisticalMultiscaleInterestRegionIndices(0.05, 0.10, 0.20, 64).size());
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterAxis(
        JNIEnv* env, jobject obj, jstring axis, jdouble minValue, jdouble maxValue)
{
    (void) obj;
    pclmobile::filterAxis(pclmobile::jstringToString(env, axis), minValue, maxValue);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterAxisOutside(
        JNIEnv* env, jobject obj, jstring axis, jdouble minValue, jdouble maxValue)
{
    (void) obj;
    pclmobile::filterAxisOutside(pclmobile::jstringToString(env, axis), minValue, maxValue);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterConditionalAxisRange(
        JNIEnv* env,
        jclass clazz,
        jstring axis,
        jdouble minValue,
        jdouble maxValue,
        jboolean keepOrganized)
{
    (void) clazz;
    pclmobile::filterConditionalAxisRange(
            pclmobile::jstringToString(env, axis),
            minValue,
            maxValue,
            keepOrganized == JNI_TRUE);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterPassThroughAdvanced(
        JNIEnv* env,
        jclass clazz,
        jstring axis,
        jdouble minValue,
        jdouble maxValue,
        jboolean negative,
        jboolean keepOrganized,
        jfloat userFilterValue)
{
    (void) clazz;
    pclmobile::filterPassThroughAdvanced(
            pclmobile::jstringToString(env, axis),
            minValue,
            maxValue,
            negative == JNI_TRUE,
            keepOrganized == JNI_TRUE,
            userFilterValue);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterVoxelGrid(
        JNIEnv* env, jobject obj, jdouble x, jdouble y, jdouble z)
{
    (void) env;
    (void) obj;
    pclmobile::filterVoxelGrid(x, y, z);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterVoxelGridMinimumPoints(
        JNIEnv* env, jclass clazz, jdouble x, jdouble y, jdouble z, jint minimumPointsPerVoxel)
{
    (void) env;
    (void) clazz;
    pclmobile::filterVoxelGridMinimumPoints(x, y, z, minimumPointsPerVoxel);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterVoxelGridCovariance(
        JNIEnv* env,
        jclass clazz,
        jdouble x,
        jdouble y,
        jdouble z,
        jint minPointsPerVoxel,
        jdouble minCovarEigvalueMult)
{
    (void) env;
    (void) clazz;
    pclmobile::filterVoxelGridCovariance(x, y, z, minPointsPerVoxel, minCovarEigvalueMult);
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeVoxelGridOccludedVoxels(
        JNIEnv* env,
        jclass clazz,
        jdouble x,
        jdouble y,
        jdouble z,
        jint maxVoxelCount)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::computeVoxelGridOccludedVoxels(x, y, z, maxVoxelCount));
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

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterGridMinimum(
        JNIEnv* env, jclass clazz, jdouble resolution)
{
    (void) env;
    (void) clazz;
    pclmobile::filterGridMinimum(resolution);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterLocalMaximum(
        JNIEnv* env, jclass clazz, jdouble radius)
{
    (void) env;
    (void) clazz;
    pclmobile::filterLocalMaximum(radius);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterMedian(
        JNIEnv* env, jclass clazz, jint windowSize, jdouble maxAllowedMovement)
{
    (void) env;
    (void) clazz;
    pclmobile::filterMedian(windowSize, maxAllowedMovement);
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

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterNormalSpaceSampling(
        JNIEnv* env,
        jclass clazz,
        jint sample,
        jint seed,
        jint binsX,
        jint binsY,
        jint binsZ,
        jint normalKSearch)
{
    (void) env;
    (void) clazz;
    pclmobile::filterNormalSpaceSampling(sample, seed, binsX, binsY, binsZ, normalKSearch);
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_sampleSurfaceNormals(
        JNIEnv* env,
        jclass clazz,
        jint sample,
        jint seed,
        jdouble ratio)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env,
            pclmobile::sampleSurfaceNormals(sample, seed, ratio));
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterCovarianceSampling(
        JNIEnv* env, jclass clazz, jint samples, jint normalKSearch)
{
    (void) env;
    (void) clazz;
    pclmobile::filterCovarianceSampling(samples, normalKSearch);
}

JNIEXPORT jfloatArray JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_computeCovarianceSamplingConditionNumber(
        JNIEnv* env, jclass clazz, jint samples, jint normalKSearch)
{
    (void) clazz;
    return pclmobile::makeFloatArray(
            env, pclmobile::computeCovarianceSamplingConditionNumber(samples, normalKSearch));
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterFastBilateral(
        JNIEnv* env, jclass clazz, jdouble sigmaS, jdouble sigmaR)
{
    (void) env;
    (void) clazz;
    pclmobile::filterFastBilateral(sigmaS, sigmaR);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterFastBilateralOMP(
        JNIEnv* env, jclass clazz, jdouble sigmaS, jdouble sigmaR, jint numberOfThreads)
{
    (void) env;
    (void) clazz;
    pclmobile::filterFastBilateralOMP(sigmaS, sigmaR, numberOfThreads);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterConvolution3DGaussian(
        JNIEnv* env,
        jclass clazz,
        jdouble sigma,
        jdouble radius,
        jdouble sigmaCoefficient,
        jint numberOfThreads)
{
    (void) env;
    (void) clazz;
    pclmobile::filterConvolution3DGaussian(sigma, radius, sigmaCoefficient, numberOfThreads);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterPlaneClipper(
        JNIEnv* env,
        jclass clazz,
        jdouble a,
        jdouble b,
        jdouble c,
        jdouble d,
        jboolean negative)
{
    (void) env;
    (void) clazz;
    pclmobile::filterPlaneClipper(a, b, c, d, negative == JNI_TRUE);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_removeNaNFromActiveCloud(
        JNIEnv* env, jclass clazz)
{
    (void) env;
    (void) clazz;
    pclmobile::removeNaNFromActiveCloud();
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

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterShadowPoints(
        JNIEnv* env, jclass clazz, jint normalKSearch, jdouble threshold)
{
    (void) env;
    (void) clazz;
    pclmobile::filterShadowPoints(normalKSearch, threshold);
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

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterBoxClipper(
        JNIEnv* env,
        jclass clazz,
        jdouble minX,
        jdouble minY,
        jdouble minZ,
        jdouble maxX,
        jdouble maxY,
        jdouble maxZ,
        jboolean negative)
{
    (void) env;
    (void) clazz;
    pclmobile::filterBoxClipper(minX, minY, minZ, maxX, maxY, maxZ, negative == JNI_TRUE);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterCropBoxTransformed(
        JNIEnv* env,
        jclass clazz,
        jdouble minX,
        jdouble minY,
        jdouble minZ,
        jdouble maxX,
        jdouble maxY,
        jdouble maxZ,
        jdouble translationX,
        jdouble translationY,
        jdouble translationZ,
        jdouble rotationX,
        jdouble rotationY,
        jdouble rotationZ)
{
    (void) env;
    (void) clazz;
    pclmobile::filterCropBoxTransformed(
            minX, minY, minZ,
            maxX, maxY, maxZ,
            translationX, translationY, translationZ,
            rotationX, rotationY, rotationZ);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterFrustumCulling(
        JNIEnv* env,
        jclass clazz,
        jdouble horizontalFov,
        jdouble verticalFov,
        jdouble nearPlaneDistance,
        jdouble farPlaneDistance,
        jfloatArray rowMajorCameraPose)
{
    (void) clazz;
    pclmobile::filterFrustumCulling(
            horizontalFov,
            verticalFov,
            nearPlaneDistance,
            farPlaneDistance,
            readFloatArray(env, rowMajorCameraPose));
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterModelOutlierRemoval(
        JNIEnv* env,
        jclass clazz,
        jint modelType,
        jfloatArray modelCoefficients,
        jdouble threshold,
        jboolean negative)
{
    (void) clazz;
    pclmobile::filterModelOutlierRemoval(
            modelType,
            readFloatArray(env, modelCoefficients),
            threshold,
            negative == JNI_TRUE);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterMorphological(
        JNIEnv* env, jclass clazz, jdouble resolution, jint morphologicalOperator)
{
    (void) env;
    (void) clazz;
    pclmobile::filterMorphological(resolution, morphologicalOperator);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterProjectInliers(
        JNIEnv* env,
        jclass clazz,
        jint modelType,
        jfloatArray modelCoefficients,
        jboolean copyAllData)
{
    (void) clazz;
    pclmobile::filterProjectInliers(
            modelType,
            readFloatArray(env, modelCoefficients),
            copyAllData == JNI_TRUE);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterCropHull2D(
        JNIEnv* env, jclass clazz, jfloatArray packedHullXYZ, jboolean negative)
{
    (void) clazz;
    pclmobile::filterCropHull2D(readFloatArray(env, packedHullXYZ), negative == JNI_TRUE);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterExtractIndices(
        JNIEnv* env, jclass clazz, jintArray indices, jboolean negative)
{
    (void) clazz;
    pclmobile::filterExtractIndices(readIntArray(env, indices), negative == JNI_TRUE);
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

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_extractModelOutliers(
        JNIEnv* env, jclass clazz, jint modelType, jdouble distanceThreshold, jint maxIterations)
{
    (void) env;
    (void) clazz;
    pclmobile::extractModelOutliers(modelType, distanceThreshold, maxIterations);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_geometry1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    LOGI("geometry1 compatibility sample: computeCentroidAndBounds values=%zu",
         pclmobile::computeCentroidAndBounds().size());
    LOGI("geometry1 compatibility sample: computeCovarianceMatrix values=%zu",
         pclmobile::computeCovarianceMatrix().size());
    LOGI("geometry1 compatibility sample: computeMeanAndCovarianceMatrix values=%zu",
         pclmobile::computeMeanAndCovarianceMatrix().size());
    LOGI("geometry1 compatibility sample: computePrincipalAxes values=%zu",
         pclmobile::computePrincipalAxes().size());
    LOGI("geometry1 compatibility sample: computeCentroidAndOBB values=%zu",
         pclmobile::computeCentroidAndOBB().size());
    LOGI("geometry1 compatibility sample: computeMomentOfInertiaAndOBB values=%zu",
         pclmobile::computeMomentOfInertiaAndOBB().size());
    LOGI("geometry1 compatibility sample: computeSquaredDistancesToPoint values=%zu",
         pclmobile::computeSquaredDistancesToPoint(0.0f, 0.0f, 0.0f).size());
    LOGI("geometry1 compatibility sample: calculatePolygonArea values=%zu",
         pclmobile::calculateActivePolygonArea().size());
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
    logTupleCount("kdtree1", "KdTreeFLANN radiusSearch max_nn",
                  pclmobile::radiusSearchLimited(0.0f, 0.0f, 0.0f, 0.28, 4).size(), 4);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_keypoint1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    logPointCount("keypoint1", "ISSKeypoint3D",
                  pclmobile::computeISSKeypoints(0.12, 0.08, 0.975, 0.975, 5)->points.size() * 3);
    logTupleCount("keypoint1", "SIFTKeypoint",
                  pclmobile::computeSIFTKeypoints(0.02, 3, 4, 0.001).size(), 4);
    logTupleCount("keypoint1", "HarrisKeypoint3D",
                  pclmobile::computeHarrisKeypoints(1, 0.05, 0.0001, true, true).size(), 4);
    logTupleCount("keypoint1", "HarrisKeypoint2D",
                  pclmobile::computeHarris2DKeypoints(1, 3, 3, 5, 0.0001, true, false).size(), 4);
    logTupleCount("keypoint1", "SUSANKeypoint",
                  pclmobile::computeSUSANKeypoints(0.05, 0.001, 0.0001, 7.0, true, false).size(), 4);
    logTupleCount("keypoint1", "TrajkovicKeypoint3D",
                  pclmobile::computeTrajkovicKeypoints(0, 3, 0.00046, 0.03589, 16).size(), 4);
    logTupleCount("keypoint1", "TrajkovicKeypoint2D",
                  pclmobile::computeTrajkovic2DKeypoints(0, 3, 0.1, 100.0).size(), 4);
    logTupleCount("keypoint1", "BriskKeypoint2D",
                  pclmobile::computeBRISK2DKeypoints(60, 4, true).size(), 4);
    logTupleCount("keypoint1", "AgastKeypoint2D",
                  pclmobile::computeAGAST2DKeypoints(30.0, 255.0, true, 0).size(), 2);
    logPointCount("keypoint1", "UniformSampling",
                  pclmobile::computeUniformSamplingKeypoints(0.05)->points.size() * 3);
    logPointCount("keypoint1", "SmoothedSurfacesKeypoint",
                  pclmobile::computeSmoothedSurfacesKeypoints(16, 0.09, 0.03, 0.06, 0.5)->points.size() * 3);
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
    logTupleCount("octree1", "OctreePointCloudSearch radiusSearch max_nn",
                  pclmobile::octreeRadiusSearchLimited(0.0f, 0.0f, 0.0f, 0.10, 0.28, 4).size(), 4);
    logTupleCount("octree1", "OctreePointCloudSearch voxelSearch",
                  pclmobile::octreeVoxelSearch(0.0f, 0.0f, 0.0f, 0.10).size(), 4);
    logTupleCount("octree1", "OctreePointCloudSearch approxNearestSearch",
                  pclmobile::octreeApproxNearestSearch(0.0f, 0.0f, 0.0f, 0.10).size(), 4);
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
    logTupleCount("rangeimages1", "RangeImage createFromPointCloud",
                  pclmobile::computeRangeImageFromActiveCloud(
                          1.0f, 360.0f, 180.0f, 0.0f, 0.0f, 0.0f, 0.0f).size(),
                  4);
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
    LOGI("sampleconsensus1 compatibility sample: SAC MSAC values=%zu",
         pclmobile::segmentSACModelWithMethod(0, 2, 0.03, 100).size());
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_segmentation1(
        JNIEnv* env, jobject obj, jstring filename)
{
    (void) obj;
    loadIfProvided(env, filename);
    LOGI("segmentation1 compatibility sample: segmentPlane values=%zu clusters=%zu",
         pclmobile::segmentPlaneModel(0.03, 100).size(),
         pclmobile::extractEuclideanClusters(0.18, 20, 5000).size());
    logPointCount("segmentation1", "ProgressiveMorphologicalFilter",
                  pclmobile::extractProgressiveMorphologicalGround(
                          33, 0.7, 0.15, 10.0, 1.0, 2.0, true, false)->points.size() * 3);
    logPointCount("segmentation1", "ApproximateProgressiveMorphologicalFilter",
                  pclmobile::extractApproximateProgressiveMorphologicalGround(
                          33, 0.7, 0.15, 10.0, 1.0, 2.0, true, 0, false)->points.size() * 3);
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
    logTupleCount("surface1", "SurfelSmoothing", pclmobile::smoothSurfelSmoothing(16, 0.03).size(), 7);
    logTupleCount("surface1", "GreedyProjectionTriangulation",
                  pclmobile::reconstructGreedyProjectionTriangles(
                          16, 0.18, 2.5, 100, 0.78539816339, 0.1745329252, 2.09439510239, false).size(), 3);
    LOGI("surface1 compatibility sample: GridProjection values=%zu",
         pclmobile::reconstructGridProjectionMesh(16, 0.05, 2, 30, 8).size());
    LOGI("surface1 compatibility sample: MarchingCubesHoppe values=%zu",
         pclmobile::reconstructMarchingCubesHoppeMesh(16, 16, 16, 16, 0.0, 0.0, -1.0).size());
    LOGI("surface1 compatibility sample: MarchingCubesRBF values=%zu",
         pclmobile::reconstructMarchingCubesRBFMesh(16, 16, 16, 16, 0.01, 0.0, 0.0).size());
    LOGI("surface1 compatibility sample: ConvexHull mesh values=%zu",
         pclmobile::computeConvexHullMesh().size());
    LOGI("surface1 compatibility sample: ConcaveHull mesh values=%zu",
         pclmobile::computeConcaveHullMesh(0.18).size());
    LOGI("surface1 compatibility sample: OrganizedFastMesh values=%zu",
         pclmobile::reconstructOrganizedFastMeshPolygons(
                 2, 1, 0.0, 0.0, 0.0, -1.0, -1.0, false, false, false).size());
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
