#include "pcl_mobile_segmentation.h"

#include <algorithm>
#include <cmath>

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
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

void setFilteredCloudFromIndices(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        const pcl::Indices& indices,
        bool negative)
{
    clearFilteredCloud();
    if (input->empty()) {
        return;
    }

    std::vector<bool> selected(input->points.size(), false);
    std::size_t valid_selected_count = 0;
    for (int index : indices) {
        if (index >= 0 && static_cast<std::size_t>(index) < selected.size()) {
            std::size_t point_index = static_cast<std::size_t>(index);
            if (!selected[point_index]) {
                selected[point_index] = true;
                ++valid_selected_count;
            }
        }
    }

    filteredCloud()->points.reserve(negative ? input->points.size() - valid_selected_count : valid_selected_count);
    for (std::size_t i = 0; i < input->points.size(); ++i) {
        if (selected[i] != negative) {
            filteredCloud()->points.push_back(input->points[i]);
        }
    }
    filteredCloud()->width = static_cast<std::uint32_t>(filteredCloud()->points.size());
    filteredCloud()->height = 1;
    filteredCloud()->is_dense = input->is_dense;
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

std::vector<jfloat> extractConditionalEuclideanClusters(
        double tolerance,
        int min_cluster_size,
        int max_cluster_size,
        double max_z_delta)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || tolerance <= 0.0 || max_z_delta < 0.0) {
        return {};
    }

    pcl::ConditionalEuclideanClustering<pcl::PointXYZ> clustering;
    clustering.setInputCloud(input);
    clustering.setClusterTolerance(static_cast<float>(tolerance));
    clustering.setMinClusterSize(static_cast<pcl::uindex_t>(std::max(min_cluster_size, 1)));
    clustering.setMaxClusterSize(static_cast<pcl::uindex_t>(std::max(max_cluster_size, min_cluster_size)));
    clustering.setConditionFunction(
            [max_z_delta](const pcl::PointXYZ& a, const pcl::PointXYZ& b, float squared_distance) {
                (void) squared_distance;
                return std::fabs(a.z - b.z) <= max_z_delta;
            });

    std::vector<pcl::PointIndices> clusters;
    clustering.segment(clusters);

    std::vector<jfloat> values;
    values.reserve(clusters.size());
    for (const auto& cluster : clusters) {
        values.push_back(static_cast<jfloat>(cluster.indices.size()));
    }
    LOGI("ConditionalEuclideanClustering: input=%zu clusters=%zu tolerance=%.3f maxZDelta=%.3f",
         input->points.size(), clusters.size(), tolerance, max_z_delta);
    return values;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractPolygonalPrismData(
        const std::vector<jfloat>& packed_planar_hull_xyz,
        double height_min,
        double height_max,
        float view_point_x,
        float view_point_y,
        float view_point_z,
        bool negative)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar_hull = cloudFromPackedXYZ(packed_planar_hull_xyz);
    if (input->empty() || planar_hull->points.size() < 3 || height_min > height_max) {
        clearFilteredCloud();
        return filteredCloud();
    }

    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud(input);
    prism.setInputPlanarHull(planar_hull);
    prism.setHeightLimits(height_min, height_max);
    prism.setViewPoint(view_point_x, view_point_y, view_point_z);

    pcl::PointIndices prism_indices;
    prism.segment(prism_indices);
    setFilteredCloudFromIndices(input, prism_indices.indices, negative);
    LOGI("ExtractPolygonalPrismData: input=%zu hull=%zu indices=%zu output=%zu height=[%.3f, %.3f] negative=%d",
         input->points.size(), planar_hull->points.size(), prism_indices.indices.size(),
         filteredCloud()->points.size(), height_min, height_max, negative ? 1 : 0);
    return filteredCloud();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractProgressiveMorphologicalGround(
        int max_window_size,
        double slope,
        double initial_distance,
        double max_distance,
        double cell_size,
        double base,
        bool exponential,
        bool negative)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || max_window_size <= 0 || cell_size <= 0.0 || base <= 0.0) {
        clearFilteredCloud();
        return filteredCloud();
    }

    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> filter;
    filter.setInputCloud(input);
    filter.setMaxWindowSize(max_window_size);
    filter.setSlope(static_cast<float>(slope));
    filter.setInitialDistance(static_cast<float>(initial_distance));
    filter.setMaxDistance(static_cast<float>(max_distance));
    filter.setCellSize(static_cast<float>(cell_size));
    filter.setBase(static_cast<float>(base));
    filter.setExponential(exponential);

    pcl::Indices ground;
    filter.extract(ground);
    setFilteredCloudFromIndices(input, ground, negative);
    LOGI("ProgressiveMorphologicalFilter: input=%zu ground=%zu output=%zu maxWindow=%d negative=%d",
         input->points.size(), ground.size(), filteredCloud()->points.size(), max_window_size, negative ? 1 : 0);
    return filteredCloud();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractApproximateProgressiveMorphologicalGround(
        int max_window_size,
        double slope,
        double initial_distance,
        double max_distance,
        double cell_size,
        double base,
        bool exponential,
        int number_of_threads,
        bool negative)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || max_window_size <= 0 || cell_size <= 0.0 || base <= 0.0) {
        clearFilteredCloud();
        return filteredCloud();
    }

    pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZ> filter;
    filter.setInputCloud(input);
    filter.setMaxWindowSize(max_window_size);
    filter.setSlope(static_cast<float>(slope));
    filter.setInitialDistance(static_cast<float>(initial_distance));
    filter.setMaxDistance(static_cast<float>(max_distance));
    filter.setCellSize(static_cast<float>(cell_size));
    filter.setBase(static_cast<float>(base));
    filter.setExponential(exponential);
    filter.setNumberOfThreads(static_cast<unsigned int>(std::max(number_of_threads, 0)));

    pcl::Indices ground;
    filter.extract(ground);
    setFilteredCloudFromIndices(input, ground, negative);
    LOGI("ApproximateProgressiveMorphologicalFilter: input=%zu ground=%zu output=%zu maxWindow=%d threads=%d negative=%d",
         input->points.size(), ground.size(), filteredCloud()->points.size(), max_window_size,
         number_of_threads, negative ? 1 : 0);
    return filteredCloud();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractMinCutForeground(
        const std::vector<jfloat>& packed_foreground_xyz,
        double sigma,
        double radius,
        double source_weight,
        int number_of_neighbours)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr foreground = cloudFromPackedXYZ(packed_foreground_xyz);
    if (input->empty()
            || foreground->empty()
            || sigma <= 0.0
            || radius <= 0.0
            || source_weight <= 0.0
            || number_of_neighbours <= 0) {
        clearFilteredCloud();
        return filteredCloud();
    }

    pcl::MinCutSegmentation<pcl::PointXYZ> min_cut;
    min_cut.setInputCloud(input);
    min_cut.setForegroundPoints(foreground);
    min_cut.setSigma(sigma);
    min_cut.setRadius(radius);
    min_cut.setSourceWeight(source_weight);
    min_cut.setNumberOfNeighbours(static_cast<unsigned int>(number_of_neighbours));

    std::vector<pcl::PointIndices> clusters;
    min_cut.extract(clusters);
    clearFilteredCloud();
    if (!clusters.empty()) {
        const pcl::PointIndices* largest_cluster = &clusters.front();
        for (const auto& cluster : clusters) {
            if (cluster.indices.size() > largest_cluster->indices.size()) {
                largest_cluster = &cluster;
            }
        }
        filteredCloud()->points.reserve(largest_cluster->indices.size());
        for (int index : largest_cluster->indices) {
            if (index >= 0 && static_cast<std::size_t>(index) < input->points.size()) {
                filteredCloud()->points.push_back(input->points[static_cast<std::size_t>(index)]);
            }
        }
        filteredCloud()->width = static_cast<std::uint32_t>(filteredCloud()->points.size());
        filteredCloud()->height = 1;
        filteredCloud()->is_dense = input->is_dense;
    }

    LOGI("MinCutSegmentation: input=%zu foreground=%zu clusters=%zu output=%zu flow=%.6f",
         input->points.size(), foreground->points.size(), clusters.size(), filteredCloud()->points.size(),
         min_cut.getMaxFlow());
    return filteredCloud();
}

std::vector<jfloat> extractMinCutForegroundStats(
        const std::vector<jfloat>& packed_foreground_xyz,
        double sigma,
        double radius,
        double source_weight,
        int number_of_neighbours)
{
    const std::size_t input_count = activeCloud()->points.size();
    const std::size_t foreground_count = packed_foreground_xyz.size() / 3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr foreground = extractMinCutForeground(
            packed_foreground_xyz, sigma, radius, source_weight, number_of_neighbours);
    return {
            static_cast<jfloat>(foreground->points.size()),
            static_cast<jfloat>(input_count),
            static_cast<jfloat>(foreground_count),
    };
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
