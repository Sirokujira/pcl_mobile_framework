#include "pcl_mobile_filters.h"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"
#include "pcl_mobile_segmentation.h"

namespace pclmobile {

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
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (!segmentPlane(distance_threshold, max_iterations, coefficients, inliers)) {
        clearFilteredCloud();
        return;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*filteredCloud());
    LOGI("ExtractIndices plane inliers: input=%zu output=%zu",
         input->points.size(), filteredCloud()->points.size());
}

} // namespace pclmobile
