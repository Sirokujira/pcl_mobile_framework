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

std::vector<int> packClusterIndices(const std::vector<pcl::PointIndices>& clusters)
{
    std::vector<int> values;
    std::size_t value_count = 1 + clusters.size();
    for (const auto& cluster : clusters) {
        value_count += cluster.indices.size();
    }
    values.reserve(value_count);
    values.push_back(static_cast<int>(clusters.size()));
    for (const auto& cluster : clusters) {
        values.push_back(static_cast<int>(cluster.indices.size()));
        values.insert(values.end(), cluster.indices.begin(), cluster.indices.end());
    }
    return values;
}

const pcl::PointIndices* largestCluster(const std::vector<pcl::PointIndices>& clusters)
{
    if (clusters.empty()) {
        return nullptr;
    }

    const pcl::PointIndices* largest_cluster = &clusters.front();
    for (const auto& cluster : clusters) {
        if (cluster.indices.size() > largest_cluster->indices.size()) {
            largest_cluster = &cluster;
        }
    }
    return largest_cluster;
}

std::vector<int> maybeInvertIndices(
        const pcl::Indices& indices,
        std::size_t point_count,
        bool negative)
{
    if (!negative) {
        return indices;
    }

    std::vector<bool> selected(point_count, false);
    for (int index : indices) {
        if (index >= 0 && static_cast<std::size_t>(index) < point_count) {
            selected[static_cast<std::size_t>(index)] = true;
        }
    }

    std::vector<int> inverted;
    inverted.reserve(point_count);
    for (std::size_t i = 0; i < point_count; ++i) {
        if (!selected[i]) {
            inverted.push_back(static_cast<int>(i));
        }
    }
    return inverted;
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
    return segmentModelWithMethod(
            model_type,
            pcl::SAC_RANSAC,
            distance_threshold,
            max_iterations,
            coefficients,
            inliers);
}

bool segmentModelWithMethod(int model_type,
                            int method_type,
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
    segmentation.setMethodType(method_type);
    segmentation.setMaxIterations(max_iterations);
    segmentation.setDistanceThreshold(distance_threshold);
    segmentation.setInputCloud(input);
    segmentation.segment(*inliers, *coefficients);
    LOGI("SACSegmentation model=%d method=%d: input=%zu inliers=%zu distance=%.3f maxIterations=%d",
         model_type, method_type, input->points.size(), inliers->indices.size(), distance_threshold, max_iterations);
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
    return segmentSACModelWithMethod(model_type, pcl::SAC_RANSAC, distance_threshold, max_iterations);
}

std::vector<jfloat> segmentSACModelWithMethod(
        int model_type,
        int method_type,
        double distance_threshold,
        int max_iterations)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    if (!segmentModelWithMethod(
                model_type,
                method_type,
                distance_threshold,
                max_iterations,
                coefficients,
                inliers)) {
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

std::vector<int> segmentSACModelOutlierIndices(int model_type, double distance_threshold, int max_iterations)
{
    return segmentSACModelOutlierIndicesWithMethod(
            model_type,
            pcl::SAC_RANSAC,
            distance_threshold,
            max_iterations);
}

std::vector<int> segmentSACModelOutlierIndicesWithMethod(
        int model_type,
        int method_type,
        double distance_threshold,
        int max_iterations)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    if (!segmentModelWithMethod(
                model_type,
                method_type,
                distance_threshold,
                max_iterations,
                coefficients,
                inliers)) {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    std::vector<int> outliers = maybeInvertIndices(inliers->indices, input->points.size(), true);
    LOGI("SACSegmentation outlier indices: input=%zu inliers=%zu outliers=%zu model=%d method=%d",
         input->points.size(), inliers->indices.size(), outliers.size(), model_type, method_type);
    return outliers;
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

std::vector<int> extractEuclideanClusterIndices(double tolerance, int min_cluster_size, int max_cluster_size)
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

    LOGI("EuclideanClusterExtraction indices: input=%zu clusters=%zu tolerance=%.3f min=%d max=%d",
         input->points.size(), cluster_indices.size(), tolerance, min_cluster_size, max_cluster_size);
    return packClusterIndices(cluster_indices);
}

std::vector<int> extractLargestEuclideanClusterIndices(
        double tolerance,
        int min_cluster_size,
        int max_cluster_size)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || tolerance <= 0.0) {
        return {};
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

    const pcl::PointIndices* largest_cluster = largestCluster(cluster_indices);
    if (largest_cluster == nullptr) {
        LOGI("EuclideanClusterExtraction largest indices: input=%zu clusters=0 tolerance=%.3f",
             input->points.size(), tolerance);
        return {};
    }

    LOGI("EuclideanClusterExtraction largest indices: input=%zu clusters=%zu indices=%zu tolerance=%.3f",
         input->points.size(), cluster_indices.size(), largest_cluster->indices.size(), tolerance);
    return largest_cluster->indices;
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

std::vector<int> extractRegionGrowingClusterIndices(
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

    LOGI("RegionGrowing indices: input=%zu clusters=%zu normal_k=%d neighbours=%d",
         input->points.size(), clusters.size(), normal_k_search, number_of_neighbours);
    return packClusterIndices(clusters);
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

std::vector<int> extractConditionalEuclideanClusterIndices(
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

    LOGI("ConditionalEuclideanClustering indices: input=%zu clusters=%zu tolerance=%.3f maxZDelta=%.3f",
         input->points.size(), clusters.size(), tolerance, max_z_delta);
    return packClusterIndices(clusters);
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

std::vector<int> extractPolygonalPrismDataIndices(
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
        return {};
    }

    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud(input);
    prism.setInputPlanarHull(planar_hull);
    prism.setHeightLimits(height_min, height_max);
    prism.setViewPoint(view_point_x, view_point_y, view_point_z);

    pcl::PointIndices prism_indices;
    prism.segment(prism_indices);
    std::vector<int> result = maybeInvertIndices(prism_indices.indices, input->points.size(), negative);
    LOGI("ExtractPolygonalPrismData indices: input=%zu hull=%zu indices=%zu returned=%zu negative=%d",
         input->points.size(), planar_hull->points.size(), prism_indices.indices.size(),
         result.size(), negative ? 1 : 0);
    return result;
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

std::vector<int> extractProgressiveMorphologicalGroundIndices(
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
        return {};
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
    std::vector<int> result = maybeInvertIndices(ground, input->points.size(), negative);
    LOGI("ProgressiveMorphologicalFilter indices: input=%zu ground=%zu returned=%zu negative=%d",
         input->points.size(), ground.size(), result.size(), negative ? 1 : 0);
    return result;
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

std::vector<int> extractApproximateProgressiveMorphologicalGroundIndices(
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
        return {};
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
    std::vector<int> result = maybeInvertIndices(ground, input->points.size(), negative);
    LOGI("ApproximateProgressiveMorphologicalFilter indices: input=%zu ground=%zu returned=%zu threads=%d negative=%d",
         input->points.size(), ground.size(), result.size(), number_of_threads, negative ? 1 : 0);
    return result;
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

std::vector<int> extractMinCutForegroundIndices(
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
        return {};
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
    const pcl::PointIndices* largest_cluster = largestCluster(clusters);
    if (largest_cluster == nullptr) {
        LOGI("MinCutSegmentation largest indices: input=%zu foreground=%zu clusters=0 flow=%.6f",
             input->points.size(), foreground->points.size(), min_cut.getMaxFlow());
        return {};
    }

    LOGI("MinCutSegmentation largest indices: input=%zu foreground=%zu clusters=%zu indices=%zu flow=%.6f",
         input->points.size(), foreground->points.size(), clusters.size(),
         largest_cluster->indices.size(), min_cut.getMaxFlow());
    return largest_cluster->indices;
}

std::vector<int> extractMinCutForegroundClusterIndices(
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
        return {};
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
    LOGI("MinCutSegmentation cluster indices: input=%zu foreground=%zu clusters=%zu flow=%.6f",
         input->points.size(), foreground->points.size(), clusters.size(), min_cut.getMaxFlow());
    return packClusterIndices(clusters);
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
