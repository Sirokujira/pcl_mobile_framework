#include "pcl_mobile_segmentation.h"

#include <algorithm>

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/segment_differences.h>

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

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        int k_search)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    if (input->empty() || k_search <= 0) {
        return normals;
    }

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(input);
    normal_estimation.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    normal_estimation.setKSearch(k_search);
    normal_estimation.compute(*normals);
    return normals;
}

float degreesToRadians(double degrees)
{
    return static_cast<float>(degrees * 3.14159265358979323846 / 180.0);
}

} // namespace

bool segmentPlane(double distance_threshold,
                  int max_iterations,
                  pcl::ModelCoefficients::Ptr coefficients,
                  pcl::PointIndices::Ptr inliers)
{
    return segmentModel(pcl::SACMODEL_PLANE, distance_threshold, max_iterations, coefficients, inliers);
}

bool segmentModel(int model_type,
                  double distance_threshold,
                  int max_iterations,
                  pcl::ModelCoefficients::Ptr coefficients,
                  pcl::PointIndices::Ptr inliers)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return false;
    }

    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(model_type);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(max_iterations);
    segmentation.setDistanceThreshold(distance_threshold);
    segmentation.setInputCloud(input);
    segmentation.segment(*inliers, *coefficients);
    LOGI("SACSegmentation model=%d: input=%zu inliers=%zu distance=%.3f maxIterations=%d",
         model_type, input->points.size(), inliers->indices.size(), distance_threshold, max_iterations);
    return !inliers->indices.empty() && coefficients->values.size() >= 4;
}

std::vector<jfloat> segmentPlaneModel(double distance_threshold, int max_iterations)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    if (!segmentPlane(distance_threshold, max_iterations, coefficients, inliers)) {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    std::vector<jfloat> values;
    values.reserve(6);
    for (int i = 0; i < 4; i++) {
        values.push_back(coefficients->values[i]);
    }
    values.push_back(static_cast<jfloat>(inliers->indices.size()));
    values.push_back(static_cast<jfloat>(input->points.size()));
    return values;
}

std::vector<jfloat> segmentSphereModel(double distance_threshold, int max_iterations)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    if (!segmentModel(pcl::SACMODEL_SPHERE, distance_threshold, max_iterations, coefficients, inliers)
            || coefficients->values.size() < 4) {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    std::vector<jfloat> values;
    values.reserve(6);
    for (int i = 0; i < 4; i++) {
        values.push_back(coefficients->values[i]);
    }
    values.push_back(static_cast<jfloat>(inliers->indices.size()));
    values.push_back(static_cast<jfloat>(input->points.size()));
    return values;
}

std::vector<jfloat> segmentSACModel(int model_type, double distance_threshold, int max_iterations)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    if (!segmentModel(model_type, distance_threshold, max_iterations, coefficients, inliers)) {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    std::vector<jfloat> values;
    values.reserve(coefficients->values.size() + 2);
    for (float coefficient : coefficients->values) {
        values.push_back(coefficient);
    }
    values.push_back(static_cast<jfloat>(inliers->indices.size()));
    values.push_back(static_cast<jfloat>(input->points.size()));
    return values;
}

std::vector<jfloat> extractEuclideanClusters(double tolerance, int min_cluster_size, int max_cluster_size)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(tolerance);
    extraction.setMinClusterSize(min_cluster_size);
    extraction.setMaxClusterSize(max_cluster_size);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(input);
    extraction.extract(cluster_indices);

    std::vector<jfloat> values;
    values.reserve(cluster_indices.size());
    for (const auto& indices : cluster_indices) {
        values.push_back(static_cast<jfloat>(indices.indices.size()));
    }
    LOGI("EuclideanClusterExtraction: input=%zu clusters=%zu tolerance=%.3f min=%d max=%d",
         input->points.size(), cluster_indices.size(), tolerance, min_cluster_size, max_cluster_size);
    return values;
}

std::vector<jfloat> extractRegionGrowingClusters(
        int normal_k_search,
        int number_of_neighbours,
        int min_cluster_size,
        int max_cluster_size,
        double smoothness_threshold_degrees,
        double curvature_threshold)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || normal_k_search <= 0 || number_of_neighbours <= 0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search);
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> region_growing;
    region_growing.setInputCloud(input);
    region_growing.setInputNormals(normals);
    region_growing.setSearchMethod(tree);
    region_growing.setNumberOfNeighbours(static_cast<unsigned int>(number_of_neighbours));
    region_growing.setMinClusterSize(static_cast<pcl::uindex_t>(std::max(min_cluster_size, 1)));
    region_growing.setMaxClusterSize(static_cast<pcl::uindex_t>(std::max(max_cluster_size, min_cluster_size)));
    region_growing.setSmoothnessThreshold(degreesToRadians(smoothness_threshold_degrees));
    region_growing.setCurvatureThreshold(static_cast<float>(curvature_threshold));

    std::vector<pcl::PointIndices> clusters;
    region_growing.extract(clusters);

    std::vector<jfloat> values;
    values.reserve(clusters.size());
    for (const auto& cluster : clusters) {
        values.push_back(static_cast<jfloat>(cluster.indices.size()));
    }
    LOGI("RegionGrowing: input=%zu clusters=%zu normal_k=%d neighbours=%d",
         input->points.size(), clusters.size(), normal_k_search, number_of_neighbours);
    return values;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr segmentDifferencesAgainstTarget(
        const std::vector<jfloat>& packed_target_xyz,
        double distance_threshold)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloudFromPackedXYZ(packed_target_xyz);
    if (input->empty() || target->empty() || distance_threshold < 0.0) {
        clearFilteredCloud();
        return filteredCloud();
    }

    pcl::SegmentDifferences<pcl::PointXYZ> differences;
    differences.setInputCloud(input);
    differences.setTargetCloud(target);
    differences.setSearchMethod(pcl::search::Search<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    differences.setDistanceThreshold(distance_threshold * distance_threshold);
    differences.segment(*filteredCloud());

    LOGI("SegmentDifferences: input=%zu target=%zu output=%zu distance=%.3f",
         input->points.size(), target->points.size(), filteredCloud()->points.size(), distance_threshold);
    return filteredCloud();
}

} // namespace pclmobile
