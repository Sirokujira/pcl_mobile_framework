#include "pcl_mobile_surface.h"

#include <cmath>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>

#include "pcl_mobile_arrays.h"
#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"
#include "pcl_mobile_segmentation.h"

namespace pclmobile {

namespace {

void appendMatrix3(std::vector<jfloat>& values, const Eigen::Matrix3f& matrix)
{
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            values.push_back(matrix(row, col));
        }
    }
}

void appendVector3(std::vector<jfloat>& values, const Eigen::Vector3f& vector)
{
    values.push_back(vector.x());
    values.push_back(vector.y());
    values.push_back(vector.z());
}

} // namespace

std::vector<jfloat> computeCentroidAndBounds()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input, centroid);
    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
    pcl::getMinMax3D(*input, min_point, max_point);

    std::vector<jfloat> values;
    values.reserve(10);
    values.push_back(centroid.x());
    values.push_back(centroid.y());
    values.push_back(centroid.z());
    appendPoint(values, min_point);
    appendPoint(values, max_point);
    values.push_back(static_cast<jfloat>(input->points.size()));
    LOGI("computeCentroidAndBounds: input=%zu centroid=(%.3f, %.3f, %.3f)",
         input->points.size(), centroid.x(), centroid.y(), centroid.z());
    return values;
}

std::vector<jfloat> computeCovarianceMatrix()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*input, centroid, covariance);

    std::vector<jfloat> values;
    values.reserve(13);
    values.push_back(centroid.x());
    values.push_back(centroid.y());
    values.push_back(centroid.z());
    appendMatrix3(values, covariance);
    values.push_back(static_cast<jfloat>(input->points.size()));
    LOGI("computeCovarianceMatrix: input=%zu", input->points.size());
    return values;
}

std::vector<jfloat> computePrincipalAxes()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(input);

    std::vector<jfloat> values;
    values.reserve(16);
    const Eigen::Vector4f mean = pca.getMean();
    values.push_back(mean.x());
    values.push_back(mean.y());
    values.push_back(mean.z());
    appendVector3(values, pca.getEigenValues());
    appendMatrix3(values, pca.getEigenVectors());
    values.push_back(static_cast<jfloat>(input->points.size()));
    LOGI("PCA computed principal axes: input=%zu", input->points.size());
    return values;
}

std::vector<jfloat> computeMomentOfInertiaAndOBB()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> estimation;
    estimation.setInputCloud(input);
    estimation.compute();

    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    pcl::PointXYZ min_aabb;
    pcl::PointXYZ max_aabb;
    pcl::PointXYZ min_obb;
    pcl::PointXYZ max_obb;
    pcl::PointXYZ position_obb;
    Eigen::Matrix3f rotational_matrix_obb;

    estimation.getMomentOfInertia(moment_of_inertia);
    estimation.getEccentricity(eccentricity);
    estimation.getAABB(min_aabb, max_aabb);
    estimation.getOBB(min_obb, max_obb, position_obb, rotational_matrix_obb);

    std::vector<jfloat> values;
    values.reserve(32);
    for (float value : moment_of_inertia) {
        values.push_back(value);
    }
    for (float value : eccentricity) {
        values.push_back(value);
    }
    appendPoint(values, min_aabb);
    appendPoint(values, max_aabb);
    appendPoint(values, min_obb);
    appendPoint(values, max_obb);
    appendPoint(values, position_obb);
    appendMatrix3(values, rotational_matrix_obb);
    values.push_back(static_cast<jfloat>(input->points.size()));
    LOGI("MomentOfInertiaEstimation computed descriptors: input=%zu moments=%zu eccentricities=%zu",
         input->points.size(), moment_of_inertia.size(), eccentricity.size());
    return values;
}

std::vector<jfloat> computeSquaredDistancesToPoint(float x, float y, float z)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    const pcl::PointXYZ query(x, y, z);
    std::vector<jfloat> values;
    values.reserve(input->points.size());
    for (const auto& point : input->points) {
        values.push_back(pcl::squaredEuclideanDistance(point, query));
    }
    LOGI("computeSquaredDistancesToPoint: input=%zu query=(%.3f, %.3f, %.3f)",
         input->points.size(), x, y, z);
    return values;
}

std::vector<jfloat> computeMaxDistanceFromCentroid()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input, centroid);
    Eigen::Vector4f max_point;
    pcl::getMaxDistance(*input, centroid, max_point);

    const float dx = max_point.x() - centroid.x();
    const float dy = max_point.y() - centroid.y();
    const float dz = max_point.z() - centroid.z();
    std::vector<jfloat> values;
    values.reserve(8);
    values.push_back(centroid.x());
    values.push_back(centroid.y());
    values.push_back(centroid.z());
    values.push_back(max_point.x());
    values.push_back(max_point.y());
    values.push_back(max_point.z());
    values.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
    values.push_back(static_cast<jfloat>(input->points.size()));
    LOGI("getMaxDistance: input=%zu distance=%.6f", input->points.size(), values[6]);
    return values;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr demeanActiveCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr demeaned(new pcl::PointCloud<pcl::PointXYZ>);
    if (input->empty()) {
        return demeaned;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input, centroid);
    pcl::demeanPointCloud(*input, centroid, *demeaned);
    LOGI("demeanPointCloud: input=%zu output=%zu", input->points.size(), demeaned->points.size());
    return demeaned;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr computeConvexHull()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
    if (input->empty()) {
        return hull;
    }

    pcl::ConvexHull<pcl::PointXYZ> convex_hull;
    convex_hull.setInputCloud(input);
    convex_hull.reconstruct(*hull);
    LOGI("ConvexHull reconstructed points: input=%zu hull=%zu dimension=%d",
         input->points.size(), hull->points.size(), convex_hull.getDimension());
    return hull;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr computeConcaveHull(double alpha)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
    if (input->empty() || alpha <= 0.0) {
        return hull;
    }

    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
    concave_hull.setInputCloud(input);
    concave_hull.setAlpha(alpha);
    concave_hull.reconstruct(*hull);
    LOGI("ConcaveHull reconstructed points: input=%zu hull=%zu alpha=%.3f dimension=%d",
         input->points.size(), hull->points.size(), alpha, concave_hull.getDimension());
    return hull;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr projectInliersToPlane(double distance_threshold, int max_iterations)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    if (!segmentPlane(distance_threshold, max_iterations, coefficients, inliers)) {
        return projected;
    }

    pcl::ProjectInliers<pcl::PointXYZ> projection;
    projection.setModelType(pcl::SACMODEL_PLANE);
    projection.setInputCloud(input);
    projection.setIndices(inliers);
    projection.setModelCoefficients(coefficients);
    projection.filter(*projected);
    LOGI("ProjectInliers plane: input=%zu projected=%zu", input->points.size(), projected->points.size());
    return projected;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr smoothMovingLeastSquares(double search_radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed(new pcl::PointCloud<pcl::PointXYZ>);
    if (input->empty() || search_radius <= 0.0) {
        return smoothed;
    }

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(input);
    mls.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    mls.setSearchRadius(search_radius);
    mls.process(*smoothed);
    LOGI("MovingLeastSquares smoothed points: input=%zu output=%zu radius=%.3f",
         input->points.size(), smoothed->points.size(), search_radius);
    return smoothed;
}

} // namespace pclmobile
