#include "pcl_mobile_registration.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transformation_estimation_3point.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>

#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"

namespace pclmobile {

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromPackedXYZ(const std::vector<jfloat>& packed_xyz)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    result->points.reserve(packed_xyz.size() / 3);
    for (std::size_t i = 0; i + 2 < packed_xyz.size(); i += 3) {
        result->points.emplace_back(packed_xyz[i], packed_xyz[i + 1], packed_xyz[i + 2]);
    }
    result->width = static_cast<std::uint32_t>(result->points.size());
    result->height = 1;
    result->is_dense = false;
    return result;
}

std::vector<jfloat> matrixToRowMajorTuple(const Eigen::Matrix4f& matrix)
{
    std::vector<jfloat> values;
    values.reserve(16);
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            values.push_back(matrix(row, col));
        }
    }
    return values;
}

Eigen::Matrix4f matrixFromRowMajorTuple(const std::vector<jfloat>& row_major_matrix)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    if (row_major_matrix.size() < 16) {
        return matrix;
    }

    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            matrix(row, col) = row_major_matrix[static_cast<std::size_t>(row * 4 + col)];
        }
    }
    return matrix;
}

template <typename RegistrationT>
std::vector<jfloat> registrationResultTuple(RegistrationT& registration)
{
    std::vector<jfloat> values;
    values.reserve(18);
    values.push_back(registration.hasConverged() ? 1.0f : 0.0f);
    values.push_back(static_cast<jfloat>(registration.getFitnessScore()));

    std::vector<jfloat> matrix_values = matrixToRowMajorTuple(registration.getFinalTransformation());
    values.insert(values.end(), matrix_values.begin(), matrix_values.end());
    return values;
}

} // namespace

std::vector<jfloat> estimateRigidTransformSVD(const std::vector<jfloat>& packed_target_xyz)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || source->points.size() != target->points.size()) {
        LOGE("TransformationEstimationSVD refused input: source=%zu target=%zu",
             source->points.size(), target->points.size());
        return {};
    }

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> estimation;
    estimation.estimateRigidTransformation(*source, *target, transform);
    LOGI("TransformationEstimationSVD: source=%zu target=%zu", source->points.size(), target->points.size());
    return matrixToRowMajorTuple(transform);
}

std::vector<jfloat> estimateRigidTransformSVDScale(const std::vector<jfloat>& packed_target_xyz)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || source->points.size() != target->points.size()) {
        LOGE("TransformationEstimationSVDScale refused input: source=%zu target=%zu",
             source->points.size(), target->points.size());
        return {};
    }

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimationSVDScale<pcl::PointXYZ, pcl::PointXYZ> estimation;
    estimation.estimateRigidTransformation(*source, *target, transform);
    LOGI("TransformationEstimationSVDScale: source=%zu target=%zu",
         source->points.size(), target->points.size());
    return matrixToRowMajorTuple(transform);
}

std::vector<jfloat> estimateRigidTransform3Point(const std::vector<jfloat>& packed_target_xyz)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->points.size() != 3 || target->points.size() != 3) {
        LOGE("TransformationEstimation3Point refused input: source=%zu target=%zu",
             source->points.size(), target->points.size());
        return {};
    }

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimation3Point<pcl::PointXYZ, pcl::PointXYZ> estimation;
    estimation.estimateRigidTransformation(*source, *target, transform);
    LOGI("TransformationEstimation3Point: source=%zu target=%zu",
         source->points.size(), target->points.size());
    return matrixToRowMajorTuple(transform);
}

std::vector<jfloat> estimateRigidTransformDualQuaternion(const std::vector<jfloat>& packed_target_xyz)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || source->points.size() != target->points.size()) {
        LOGE("TransformationEstimationDualQuaternion refused input: source=%zu target=%zu",
             source->points.size(), target->points.size());
        return {};
    }

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimationDualQuaternion<pcl::PointXYZ, pcl::PointXYZ> estimation;
    estimation.estimateRigidTransformation(*source, *target, transform);
    LOGI("TransformationEstimationDualQuaternion: source=%zu target=%zu",
         source->points.size(), target->points.size());
    return matrixToRowMajorTuple(transform);
}

std::vector<jfloat> estimateRigidTransformLM(const std::vector<jfloat>& packed_target_xyz)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || source->points.size() != target->points.size()) {
        LOGE("TransformationEstimationLM refused input: source=%zu target=%zu",
             source->points.size(), target->points.size());
        return {};
    }

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> estimation;
    estimation.estimateRigidTransformation(*source, *target, transform);
    LOGI("TransformationEstimationLM: source=%zu target=%zu",
         source->points.size(), target->points.size());
    return matrixToRowMajorTuple(transform);
}

std::vector<jfloat> estimateRigidTransform2D(const std::vector<jfloat>& packed_target_xyz)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || source->points.size() != target->points.size()) {
        LOGE("TransformationEstimation2D refused input: source=%zu target=%zu",
             source->points.size(), target->points.size());
        return {};
    }

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ> estimation;
    estimation.estimateRigidTransformation(*source, *target, transform);
    LOGI("TransformationEstimation2D: source=%zu target=%zu",
         source->points.size(), target->points.size());
    return matrixToRowMajorTuple(transform);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformActiveCloud(const std::vector<jfloat>& row_major_matrix)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    if (source->empty()) {
        clearFilteredCloud();
        return filteredCloud();
    }

    Eigen::Matrix4f transform = matrixFromRowMajorTuple(row_major_matrix);
    pcl::transformPointCloud(*source, *filteredCloud(), transform);
    LOGI("transformPointCloud: input=%zu output=%zu", source->points.size(), filteredCloud()->points.size());
    return filteredCloud();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr translateActiveCloud(float tx, float ty, float tz)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = tx;
    transform(1, 3) = ty;
    transform(2, 3) = tz;
    return transformActiveCloud(matrixToRowMajorTuple(transform));
}

std::vector<jfloat> alignToTargetICP(
        const std::vector<jfloat>& packed_target_xyz,
        int max_iterations,
        double max_correspondence_distance,
        double transformation_epsilon,
        double euclidean_fitness_epsilon)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || target->empty()) {
        clearFilteredCloud();
        return {};
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>* icp =
            new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>();
    icp->setInputSource(source);
    icp->setInputTarget(target);
    icp->setMaximumIterations(max_iterations);
    if (max_correspondence_distance > 0.0) {
        icp->setMaxCorrespondenceDistance(max_correspondence_distance);
    }
    if (transformation_epsilon > 0.0) {
        icp->setTransformationEpsilon(transformation_epsilon);
    }
    if (euclidean_fitness_epsilon > 0.0) {
        icp->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    }
    icp->align(*filteredCloud());

    LOGI("IterativeClosestPoint target: source=%zu target=%zu output=%zu converged=%d fitness=%.6f",
         source->points.size(), target->points.size(), filteredCloud()->points.size(),
         icp->hasConverged() ? 1 : 0, icp->getFitnessScore());
    return registrationResultTuple(*icp);
}

std::vector<jfloat> alignToTargetGICP(
        const std::vector<jfloat>& packed_target_xyz,
        int max_iterations,
        double max_correspondence_distance,
        double transformation_epsilon,
        double rotation_epsilon,
        int max_optimizer_iterations)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || target->empty()) {
        clearFilteredCloud();
        return {};
    }

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>* gicp =
            new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>();
    gicp->setInputSource(source);
    gicp->setInputTarget(target);
    gicp->setMaximumIterations(max_iterations);
    if (max_correspondence_distance > 0.0) {
        gicp->setMaxCorrespondenceDistance(max_correspondence_distance);
    }
    if (transformation_epsilon > 0.0) {
        gicp->setTransformationEpsilon(transformation_epsilon);
    }
    if (rotation_epsilon > 0.0) {
        gicp->setRotationEpsilon(rotation_epsilon);
    }
    if (max_optimizer_iterations > 0) {
        gicp->setMaximumOptimizerIterations(max_optimizer_iterations);
    }
    gicp->align(*filteredCloud());

    LOGI("GeneralizedIterativeClosestPoint target: source=%zu target=%zu output=%zu converged=%d fitness=%.6f",
         source->points.size(), target->points.size(), filteredCloud()->points.size(),
         gicp->hasConverged() ? 1 : 0, gicp->getFitnessScore());
    return registrationResultTuple(*gicp);
}

std::vector<jfloat> alignToTargetICPNonLinear(
        const std::vector<jfloat>& packed_target_xyz,
        int max_iterations,
        double max_correspondence_distance,
        double transformation_epsilon,
        double euclidean_fitness_epsilon)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || target->empty()) {
        clearFilteredCloud();
        return {};
    }

    pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>* icp =
            new pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>();
    icp->setInputSource(source);
    icp->setInputTarget(target);
    icp->setMaximumIterations(max_iterations);
    if (max_correspondence_distance > 0.0) {
        icp->setMaxCorrespondenceDistance(max_correspondence_distance);
    }
    if (transformation_epsilon > 0.0) {
        icp->setTransformationEpsilon(transformation_epsilon);
    }
    if (euclidean_fitness_epsilon > 0.0) {
        icp->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    }
    icp->align(*filteredCloud());

    LOGI("IterativeClosestPointNonLinear target: source=%zu target=%zu output=%zu converged=%d fitness=%.6f",
         source->points.size(), target->points.size(), filteredCloud()->points.size(),
         icp->hasConverged() ? 1 : 0, icp->getFitnessScore());
    return registrationResultTuple(*icp);
}

std::vector<jfloat> alignToTargetNDT(
        const std::vector<jfloat>& packed_target_xyz,
        int max_iterations,
        double resolution,
        double step_size,
        double transformation_epsilon,
        int min_points_per_voxel)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || target->empty() || resolution <= 0.0) {
        clearFilteredCloud();
        return {};
    }

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>* ndt =
            new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>();
    ndt->setInputSource(source);
    ndt->setInputTarget(target);
    ndt->setMaximumIterations(max_iterations);
    ndt->setResolution(static_cast<float>(resolution));
    if (step_size > 0.0) {
        ndt->setStepSize(step_size);
    }
    if (transformation_epsilon > 0.0) {
        ndt->setTransformationEpsilon(transformation_epsilon);
    }
    if (min_points_per_voxel > 0) {
        ndt->setMinPointPerVoxel(static_cast<unsigned int>(min_points_per_voxel));
    }
    ndt->align(*filteredCloud());

    LOGI("NormalDistributionsTransform target: source=%zu target=%zu output=%zu converged=%d fitness=%.6f resolution=%.3f",
         source->points.size(), target->points.size(), filteredCloud()->points.size(),
         ndt->hasConverged() ? 1 : 0, ndt->getFitnessScore(), resolution);
    return registrationResultTuple(*ndt);
}

std::vector<jfloat> alignToTranslatedCopyICP(float tx, float ty, float tz, int max_iterations)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    if (source->empty()) {
        clearFilteredCloud();
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << tx, ty, tz;
    pcl::transformPointCloud(*source, *target, transform);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    // Keep each ICP object alive for the process lifetime. Reusing one ICP
    // instance across calls corrupts the correspondence-estimation state on the
    // tested Android build, while destroying it after JNI return trips Scudo in
    // the same cleanup path.
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>* icp =
            new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>();
    icp->setInputSource(source);
    icp->setInputTarget(target);
    icp->setMaximumIterations(max_iterations);
    icp->align(aligned);
    *filteredCloud() = aligned;

    LOGI("IterativeClosestPoint: input=%zu converged=%d fitness=%.6f translation=(%.3f, %.3f, %.3f)",
         source->points.size(), icp->hasConverged() ? 1 : 0, icp->getFitnessScore(), tx, ty, tz);
    return registrationResultTuple(*icp);
}

} // namespace pclmobile
