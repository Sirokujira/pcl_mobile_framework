#include "pcl_mobile_registration.h"

#include <cmath>
#include <limits>

#include <pcl/common/transforms.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
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
#include <pcl/registration/transformation_validation_euclidean.h>

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

std::vector<jfloat> packCorrespondences(const pcl::Correspondences& correspondences)
{
    std::vector<jfloat> values;
    values.reserve(correspondences.size() * 3);
    for (const pcl::Correspondence& correspondence : correspondences) {
        values.push_back(static_cast<jfloat>(correspondence.index_query));
        values.push_back(static_cast<jfloat>(correspondence.index_match));
        values.push_back(static_cast<jfloat>(correspondence.distance));
    }
    return values;
}

pcl::Correspondences unpackCorrespondences(
        const std::vector<jfloat>& packed_correspondences,
        int source_size = -1,
        int target_size = -1)
{
    pcl::Correspondences correspondences;
    correspondences.reserve(packed_correspondences.size() / 3);
    for (std::size_t i = 0; i + 2 < packed_correspondences.size(); i += 3) {
        const int index_query = static_cast<int>(packed_correspondences[i]);
        const int index_match = static_cast<int>(packed_correspondences[i + 1]);
        const float distance = packed_correspondences[i + 2];
        if (index_query < 0 || index_match < 0 || !std::isfinite(distance)) {
            continue;
        }
        if ((source_size >= 0 && index_query >= source_size) ||
                (target_size >= 0 && index_match >= target_size)) {
            continue;
        }
        correspondences.emplace_back(index_query, index_match, distance);
    }
    return correspondences;
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

std::vector<jfloat> findCorrespondences(
        const std::vector<jfloat>& packed_target_xyz,
        double max_distance,
        bool reciprocal)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || target->empty()) {
        return {};
    }

    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> estimation;
    estimation.setInputSource(source);
    estimation.setInputTarget(target);

    pcl::Correspondences correspondences;
    const double search_distance =
            max_distance > 0.0 ? max_distance : std::numeric_limits<double>::max();
    if (reciprocal) {
        estimation.determineReciprocalCorrespondences(correspondences, search_distance);
    } else {
        estimation.determineCorrespondences(correspondences, search_distance);
    }

    LOGI("CorrespondenceEstimation%s: source=%zu target=%zu correspondences=%zu max_distance=%.6f",
         reciprocal ? " reciprocal" : "", source->points.size(), target->points.size(),
         correspondences.size(), search_distance);
    return packCorrespondences(correspondences);
}

std::vector<jfloat> rejectCorrespondencesDistance(
        const std::vector<jfloat>& packed_correspondences,
        const std::vector<jfloat>& packed_target_xyz,
        double max_distance)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || target->empty() || max_distance <= 0.0) {
        return {};
    }

    pcl::Correspondences correspondences = unpackCorrespondences(
            packed_correspondences,
            static_cast<int>(source->points.size()),
            static_cast<int>(target->points.size()));
    if (correspondences.empty()) {
        return {};
    }

    pcl::registration::CorrespondenceRejectorDistance rejector;
    rejector.setInputSource<pcl::PointXYZ>(source);
    rejector.setInputTarget<pcl::PointXYZ>(target);
    rejector.setMaximumDistance(static_cast<float>(max_distance));

    pcl::Correspondences remaining;
    rejector.getRemainingCorrespondences(correspondences, remaining);
    LOGI("CorrespondenceRejectorDistance: input=%zu output=%zu max_distance=%.6f",
         correspondences.size(), remaining.size(), max_distance);
    return packCorrespondences(remaining);
}

std::vector<jfloat> rejectCorrespondencesOneToOne(const std::vector<jfloat>& packed_correspondences)
{
    pcl::Correspondences correspondences = unpackCorrespondences(packed_correspondences);
    if (correspondences.empty()) {
        return {};
    }

    pcl::registration::CorrespondenceRejectorOneToOne rejector;
    pcl::Correspondences remaining;
    rejector.getRemainingCorrespondences(correspondences, remaining);
    LOGI("CorrespondenceRejectorOneToOne: input=%zu output=%zu",
         correspondences.size(), remaining.size());
    return packCorrespondences(remaining);
}

std::vector<jfloat> rejectCorrespondencesMedianDistance(
        const std::vector<jfloat>& packed_correspondences,
        double median_factor)
{
    pcl::Correspondences correspondences = unpackCorrespondences(packed_correspondences);
    if (correspondences.empty()) {
        return {};
    }

    pcl::registration::CorrespondenceRejectorMedianDistance rejector;
    if (median_factor > 0.0) {
        rejector.setMedianFactor(median_factor);
    }

    pcl::Correspondences remaining;
    rejector.getRemainingCorrespondences(correspondences, remaining);
    LOGI("CorrespondenceRejectorMedianDistance: input=%zu output=%zu factor=%.3f median=%.6f",
         correspondences.size(), remaining.size(), rejector.getMedianFactor(),
         rejector.getMedianDistance());
    return packCorrespondences(remaining);
}

std::vector<jfloat> rejectCorrespondencesTrimmed(
        const std::vector<jfloat>& packed_correspondences,
        double overlap_ratio,
        int min_correspondences)
{
    pcl::Correspondences correspondences = unpackCorrespondences(packed_correspondences);
    if (correspondences.empty()) {
        return {};
    }

    pcl::registration::CorrespondenceRejectorTrimmed rejector;
    if (overlap_ratio >= 0.0) {
        rejector.setOverlapRatio(static_cast<float>(overlap_ratio));
    }
    if (min_correspondences > 0) {
        rejector.setMinCorrespondences(static_cast<unsigned int>(min_correspondences));
    }

    pcl::Correspondences remaining;
    rejector.getRemainingCorrespondences(correspondences, remaining);
    LOGI("CorrespondenceRejectorTrimmed: input=%zu output=%zu overlap=%.3f min=%u",
         correspondences.size(), remaining.size(), rejector.getOverlapRatio(),
         rejector.getMinCorrespondences());
    return packCorrespondences(remaining);
}

std::vector<jfloat> rejectCorrespondencesVarTrimmed(
        const std::vector<jfloat>& packed_correspondences,
        double min_ratio,
        double max_ratio)
{
    pcl::Correspondences correspondences = unpackCorrespondences(packed_correspondences);
    if (correspondences.empty()) {
        return {};
    }

    pcl::registration::CorrespondenceRejectorVarTrimmed rejector;
    if (min_ratio >= 0.0 && min_ratio <= 1.0) {
        rejector.setMinRatio(min_ratio);
    }
    if (max_ratio >= 0.0 && max_ratio <= 1.0) {
        rejector.setMaxRatio(max_ratio);
    }

    pcl::Correspondences remaining;
    rejector.getRemainingCorrespondences(correspondences, remaining);
    LOGI("CorrespondenceRejectorVarTrimmed: input=%zu output=%zu trim=%.3f distance=%.6f min=%.3f max=%.3f",
         correspondences.size(), remaining.size(), rejector.getTrimFactor(),
         rejector.getTrimmedDistance(), rejector.getMinRatio(), rejector.getMaxRatio());
    return packCorrespondences(remaining);
}

std::vector<jfloat> rejectCorrespondencesSampleConsensus(
        const std::vector<jfloat>& packed_correspondences,
        const std::vector<jfloat>& packed_target_xyz,
        double inlier_threshold,
        int max_iterations,
        bool refine_model)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || target->empty()) {
        return {};
    }

    pcl::Correspondences correspondences = unpackCorrespondences(
            packed_correspondences,
            static_cast<int>(source->points.size()),
            static_cast<int>(target->points.size()));
    if (correspondences.empty()) {
        return {};
    }

    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
    rejector.setInputSource(source);
    rejector.setInputTarget(target);
    if (inlier_threshold > 0.0) {
        rejector.setInlierThreshold(inlier_threshold);
    }
    if (max_iterations > 0) {
        rejector.setMaximumIterations(max_iterations);
    }
    rejector.setRefineModel(refine_model);

    pcl::Correspondences remaining;
    rejector.getRemainingCorrespondences(correspondences, remaining);
    LOGI("CorrespondenceRejectorSampleConsensus: input=%zu output=%zu threshold=%.6f iterations=%d refine=%d",
         correspondences.size(), remaining.size(), rejector.getInlierThreshold(),
         rejector.getMaximumIterations(), refine_model ? 1 : 0);
    return packCorrespondences(remaining);
}

std::vector<jfloat> rejectCorrespondencesPoly(
        const std::vector<jfloat>& packed_correspondences,
        const std::vector<jfloat>& packed_target_xyz,
        int cardinality,
        double similarity_threshold,
        int iterations)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || target->empty()) {
        return {};
    }

    pcl::Correspondences correspondences = unpackCorrespondences(
            packed_correspondences,
            static_cast<int>(source->points.size()),
            static_cast<int>(target->points.size()));
    if (correspondences.empty()) {
        return {};
    }

    pcl::registration::CorrespondenceRejectorPoly<pcl::PointXYZ, pcl::PointXYZ> rejector;
    rejector.setInputSource(source);
    rejector.setInputTarget(target);
    if (cardinality >= 2) {
        rejector.setCardinality(cardinality);
    }
    if (similarity_threshold > 0.0 && similarity_threshold < 1.0) {
        rejector.setSimilarityThreshold(static_cast<float>(similarity_threshold));
    }
    if (iterations > 0) {
        rejector.setIterations(iterations);
    }

    pcl::Correspondences remaining;
    rejector.getRemainingCorrespondences(correspondences, remaining);
    LOGI("CorrespondenceRejectorPoly: input=%zu output=%zu cardinality=%d similarity=%.3f iterations=%d",
         correspondences.size(), remaining.size(), rejector.getCardinality(),
         rejector.getSimilarityThreshold(), rejector.getIterations());
    return packCorrespondences(remaining);
}

std::vector<jfloat> validateTransformEuclidean(
        const std::vector<jfloat>& packed_target_xyz,
        const std::vector<jfloat>& row_major_matrix,
        double max_range,
        double threshold)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (source->empty() || target->empty()) {
        return {};
    }

    pcl::registration::TransformationValidationEuclidean<pcl::PointXYZ, pcl::PointXYZ> validation;
    if (max_range > 0.0) {
        validation.setMaxRange(max_range);
    }

    const Eigen::Matrix4f transform = matrixFromRowMajorTuple(row_major_matrix);
    const double score = validation.validateTransformation(source, target, transform);
    bool valid = true;
    if (threshold > 0.0) {
        validation.setThreshold(threshold);
        valid = validation.isValid(source, target, transform);
    }

    LOGI("TransformationValidationEuclidean: source=%zu target=%zu score=%.6f valid=%d",
         source->points.size(), target->points.size(), score, valid ? 1 : 0);
    return {
            static_cast<jfloat>(score),
            valid ? 1.0f : 0.0f,
    };
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

pcl::PointCloud<pcl::PointXYZ>::Ptr transformActiveCloudQuaternion(
        float tx,
        float ty,
        float tz,
        float qx,
        float qy,
        float qz,
        float qw)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    if (source->empty()) {
        clearFilteredCloud();
        return filteredCloud();
    }

    Eigen::Vector3f offset(tx, ty, tz);
    Eigen::Quaternionf rotation(qw, qx, qy, qz);
    if (rotation.norm() <= std::numeric_limits<float>::epsilon()) {
        rotation = Eigen::Quaternionf::Identity();
    } else {
        rotation.normalize();
    }

    pcl::transformPointCloud(*source, *filteredCloud(), offset, rotation, true);
    LOGI("transformPointCloud quaternion: input=%zu output=%zu", source->points.size(), filteredCloud()->points.size());
    return filteredCloud();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformActiveCloudIndices(
        const std::vector<int>& indices,
        const std::vector<jfloat>& row_major_matrix)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    if (source->empty()) {
        clearFilteredCloud();
        return filteredCloud();
    }

    pcl::Indices selected;
    selected.reserve(indices.size());
    for (int index : indices) {
        if (index >= 0 && static_cast<std::size_t>(index) < source->points.size()) {
            selected.push_back(index);
        }
    }
    if (selected.empty()) {
        clearFilteredCloud();
        return filteredCloud();
    }

    Eigen::Matrix4f transform = matrixFromRowMajorTuple(row_major_matrix);
    pcl::transformPointCloud(*source, selected, *filteredCloud(), transform, true);
    LOGI("transformPointCloud indices: input=%zu selected=%zu output=%zu",
         source->points.size(), selected.size(), filteredCloud()->points.size());
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
