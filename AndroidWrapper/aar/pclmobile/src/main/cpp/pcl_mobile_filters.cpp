#include "pcl_mobile_filters.h"

#include <algorithm>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/farthest_point_sampling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>

#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"
#include "pcl_mobile_segmentation.h"

namespace pclmobile {

namespace {

constexpr int kSacModelPlane = 0;

} // namespace

void filterAxis(const std::string& axis, double min_value, double max_value)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud());
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(min_value, max_value);
    pass.setNegative(true);
    pass.filter(*filteredCloud());
}

void filterVoxelGrid(double x, double y, double z)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud());
    voxel_grid.setLeafSize(x, y, z);
    voxel_grid.filter(*filteredCloud());
    LOGI("VoxelGrid filtered points: input=%zu output=%zu leaf=(%.3f, %.3f, %.3f)",
         cloud()->points.size(), filteredCloud()->points.size(), x, y, z);
}

void filterApproximateVoxelGrid(double x, double y, double z)
{
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud());
    voxel_grid.setLeafSize(x, y, z);
    voxel_grid.filter(*filteredCloud());
    LOGI("ApproximateVoxelGrid filtered points: input=%zu output=%zu leaf=(%.3f, %.3f, %.3f)",
         cloud()->points.size(), filteredCloud()->points.size(), x, y, z);
}

void filterUniformSampling(double radius)
{
    pcl::UniformSampling<pcl::PointXYZ> sampling;
    sampling.setInputCloud(cloud());
    sampling.setRadiusSearch(radius);
    sampling.filter(*filteredCloud());
    LOGI("UniformSampling filtered points: input=%zu output=%zu radius=%.3f",
         cloud()->points.size(), filteredCloud()->points.size(), radius);
}

void filterRandomSample(int sample, int seed)
{
    pcl::RandomSample<pcl::PointXYZ> random_sample;
    random_sample.setInputCloud(cloud());
    random_sample.setSample(static_cast<unsigned int>(std::max(sample, 0)));
    random_sample.setSeed(static_cast<unsigned int>(seed));
    random_sample.filter(*filteredCloud());
    LOGI("RandomSample filtered points: input=%zu output=%zu sample=%d seed=%d",
         cloud()->points.size(), filteredCloud()->points.size(), sample, seed);
}

void filterFarthestPointSampling(int sample, int seed)
{
    pcl::FarthestPointSampling<pcl::PointXYZ> sampling;
    sampling.setInputCloud(cloud());
    sampling.setSample(static_cast<std::size_t>(std::max(sample, 0)));
    sampling.setSeed(static_cast<unsigned int>(seed));
    sampling.filter(*filteredCloud());
    LOGI("FarthestPointSampling filtered points: input=%zu output=%zu sample=%d seed=%d",
         cloud()->points.size(), filteredCloud()->points.size(), sample, seed);
}

void filterStatisticalOutlierRemoval(int mean_k, double stddev_mul_thresh)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> removal;
    removal.setInputCloud(cloud());
    removal.setMeanK(mean_k);
    removal.setStddevMulThresh(stddev_mul_thresh);
    removal.filter(*filteredCloud());
    LOGI("StatisticalOutlierRemoval filtered points: input=%zu output=%zu meanK=%d stddev=%.3f",
         cloud()->points.size(), filteredCloud()->points.size(), mean_k, stddev_mul_thresh);
}

void filterRadiusOutlierRemoval(double radius, int min_neighbors)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> removal;
    removal.setInputCloud(cloud());
    removal.setRadiusSearch(radius);
    removal.setMinNeighborsInRadius(min_neighbors);
    removal.filter(*filteredCloud());
    LOGI("RadiusOutlierRemoval filtered points: input=%zu output=%zu radius=%.3f minNeighbors=%d",
         cloud()->points.size(), filteredCloud()->points.size(), radius, min_neighbors);
}

void filterCropBox(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z)
{
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud());
    crop_box.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0f));
    crop_box.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0f));
    crop_box.filter(*filteredCloud());
    LOGI("CropBox filtered points: input=%zu output=%zu min=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f)",
         cloud()->points.size(), filteredCloud()->points.size(), min_x, min_y, min_z, max_x, max_y, max_z);
}

void extractPlaneInliers(double distance_threshold, int max_iterations)
{
    extractModelInliers(kSacModelPlane, distance_threshold, max_iterations);
}

void extractModelInliers(int model_type, double distance_threshold, int max_iterations)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (!segmentModel(model_type, distance_threshold, max_iterations, coefficients, inliers)) {
        clearFilteredCloud();
        return;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*filteredCloud());
    LOGI("ExtractIndices model=%d inliers: input=%zu output=%zu",
         model_type, input->points.size(), filteredCloud()->points.size());
}

} // namespace pclmobile
