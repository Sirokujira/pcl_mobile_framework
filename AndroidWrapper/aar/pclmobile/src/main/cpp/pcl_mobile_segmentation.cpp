#include "pcl_mobile_segmentation.h"

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"

namespace pclmobile {

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

} // namespace pclmobile
