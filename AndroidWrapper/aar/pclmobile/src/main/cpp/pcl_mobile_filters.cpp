#include "pcl_mobile_filters.h"

#include <algorithm>
#include <vector>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/farthest_point_sampling.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

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
    pass.setNegative(false);
    pass.filter(*filteredCloud());
}

void filterAxisOutside(const std::string& axis, double min_value, double max_value)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud());
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(min_value, max_value);
    pass.setNegative(true);
    pass.filter(*filteredCloud());
}

void filterConditionalAxisRange(
        const std::string& axis, double min_value, double max_value, bool keep_organized)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    if (input->empty()) {
        clearFilteredCloud();
        return;
    }

    pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZ>);
    condition->addComparison(
            pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
                    new pcl::FieldComparison<pcl::PointXYZ>(
                            axis, pcl::ComparisonOps::GE, min_value)));
    condition->addComparison(
            pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
                    new pcl::FieldComparison<pcl::PointXYZ>(
                            axis, pcl::ComparisonOps::LE, max_value)));

    pcl::ConditionalRemoval<pcl::PointXYZ> removal;
    removal.setCondition(condition);
    removal.setInputCloud(input);
    removal.setKeepOrganized(keep_organized);
    removal.filter(*filteredCloud());
    LOGI("ConditionalRemoval filtered points: input=%zu output=%zu axis=%s range=(%.3f, %.3f) keepOrganized=%d",
         input->points.size(), filteredCloud()->points.size(), axis.c_str(), min_value, max_value,
         keep_organized ? 1 : 0);
}

void filterPassThroughAdvanced(
        const std::string& axis,
        double min_value,
        double max_value,
        bool negative,
        bool keep_organized,
        float user_filter_value)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(activeCloud());
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(min_value, max_value);
    pass.setNegative(negative);
    pass.setKeepOrganized(keep_organized);
    pass.setUserFilterValue(user_filter_value);
    pass.filter(*filteredCloud());
    LOGI("PassThrough advanced filtered points: input=%zu output=%zu axis=%s range=(%.3f, %.3f) negative=%d keepOrganized=%d",
         activeCloud()->points.size(), filteredCloud()->points.size(), axis.c_str(), min_value, max_value,
         negative ? 1 : 0, keep_organized ? 1 : 0);
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

void filterVoxelGridMinimumPoints(double x, double y, double z, int minimum_points_per_voxel)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud());
    voxel_grid.setLeafSize(x, y, z);
    voxel_grid.setMinimumPointsNumberPerVoxel(
            static_cast<unsigned int>(std::max(minimum_points_per_voxel, 0)));
    voxel_grid.filter(*filteredCloud());
    LOGI("VoxelGrid minimum-points filtered points: input=%zu output=%zu leaf=(%.3f, %.3f, %.3f) minPoints=%d",
         cloud()->points.size(), filteredCloud()->points.size(), x, y, z, minimum_points_per_voxel);
}

void filterVoxelGridCovariance(
        double x,
        double y,
        double z,
        int min_points_per_voxel,
        double min_covar_eigvalue_mult)
{
    pcl::VoxelGridCovariance<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud());
    voxel_grid.setLeafSize(x, y, z);
    voxel_grid.setMinPointPerVoxel(min_points_per_voxel);
    if (min_covar_eigvalue_mult > 0.0) {
        voxel_grid.setCovEigValueInflationRatio(min_covar_eigvalue_mult);
    }
    voxel_grid.filter(*filteredCloud());
    LOGI("VoxelGridCovariance filtered points: input=%zu output=%zu leaf=(%.3f, %.3f, %.3f) minPoints=%d",
         cloud()->points.size(), filteredCloud()->points.size(), x, y, z, min_points_per_voxel);
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

void filterGridMinimum(double resolution)
{
    pcl::GridMinimum<pcl::PointXYZ> grid_minimum(static_cast<float>(resolution));
    grid_minimum.setInputCloud(cloud());
    grid_minimum.filter(*filteredCloud());
    LOGI("GridMinimum filtered points: input=%zu output=%zu resolution=%.3f",
         cloud()->points.size(), filteredCloud()->points.size(), resolution);
}

void filterLocalMaximum(double radius)
{
    pcl::LocalMaximum<pcl::PointXYZ> local_maximum;
    local_maximum.setInputCloud(cloud());
    local_maximum.setRadius(static_cast<float>(radius));
    local_maximum.filter(*filteredCloud());
    LOGI("LocalMaximum filtered points: input=%zu output=%zu radius=%.3f",
         cloud()->points.size(), filteredCloud()->points.size(), radius);
}

void filterMedian(int window_size, double max_allowed_movement)
{
    pcl::MedianFilter<pcl::PointXYZ> median_filter;
    median_filter.setInputCloud(cloud());
    median_filter.setWindowSize(window_size);
    if (max_allowed_movement > 0.0) {
        median_filter.setMaxAllowedMovement(static_cast<float>(max_allowed_movement));
    }
    median_filter.filter(*filteredCloud());
    LOGI("MedianFilter filtered points: input=%zu output=%zu window=%d maxMovement=%.3f",
         cloud()->points.size(), filteredCloud()->points.size(), window_size, max_allowed_movement);
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

void filterNormalSpaceSampling(int sample, int seed, int bins_x, int bins_y, int bins_z, int normal_k_search)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = cloud();
    if (input->empty() || sample <= 0 || bins_x <= 0 || bins_y <= 0 || bins_z <= 0 || normal_k_search <= 0) {
        clearFilteredCloud();
        return;
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(input);
    normal_estimation.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    normal_estimation.setKSearch(normal_k_search);
    normal_estimation.compute(*normals);

    pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> sampling;
    sampling.setInputCloud(input);
    sampling.setNormals(normals);
    sampling.setSample(static_cast<unsigned int>(sample));
    sampling.setSeed(static_cast<unsigned int>(seed));
    sampling.setBins(
            static_cast<unsigned int>(bins_x),
            static_cast<unsigned int>(bins_y),
            static_cast<unsigned int>(bins_z));
    sampling.filter(*filteredCloud());
    LOGI("NormalSpaceSampling filtered points: input=%zu output=%zu sample=%d bins=(%d,%d,%d) normal_k=%d",
         input->points.size(), filteredCloud()->points.size(), sample, bins_x, bins_y, bins_z, normal_k_search);
}

void removeNaNFromActiveCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    if (input->empty()) {
        clearFilteredCloud();
        return;
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*input, *filteredCloud(), indices);
    LOGI("removeNaNFromPointCloud filtered points: input=%zu output=%zu",
         input->points.size(), filteredCloud()->points.size());
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

void filterCropBoxTransformed(
        double min_x,
        double min_y,
        double min_z,
        double max_x,
        double max_y,
        double max_z,
        double translation_x,
        double translation_y,
        double translation_z,
        double rotation_x,
        double rotation_y,
        double rotation_z)
{
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud());
    crop_box.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0f));
    crop_box.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0f));
    crop_box.setTranslation(Eigen::Vector3f(translation_x, translation_y, translation_z));
    crop_box.setRotation(Eigen::Vector3f(rotation_x, rotation_y, rotation_z));
    crop_box.filter(*filteredCloud());
    LOGI("CropBox transformed filtered points: input=%zu output=%zu min=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f)",
         cloud()->points.size(), filteredCloud()->points.size(), min_x, min_y, min_z, max_x, max_y, max_z);
}

void filterExtractIndices(const std::vector<int>& indices, bool negative)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>(*activeCloud()));
    if (input->empty()) {
        clearFilteredCloud();
        return;
    }

    pcl::PointIndices::Ptr point_indices(new pcl::PointIndices);
    point_indices->indices.reserve(indices.size());
    for (int index : indices) {
        if (index >= 0 && static_cast<std::size_t>(index) < input->points.size()) {
            point_indices->indices.push_back(index);
        }
    }

    if (point_indices->indices.empty()) {
        if (negative) {
            *filteredCloud() = *input;
        } else {
            clearFilteredCloud();
        }
        LOGI("ExtractIndices explicit indices: input=%zu requested=%zu valid=0 output=%zu negative=%d",
             input->points.size(), indices.size(), filteredCloud()->points.size(), negative ? 1 : 0);
        return;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(point_indices);
    extract.setNegative(negative);
    extract.filter(*filteredCloud());
    LOGI("ExtractIndices explicit indices: input=%zu requested=%zu valid=%zu output=%zu negative=%d",
         input->points.size(), indices.size(), point_indices->indices.size(),
         filteredCloud()->points.size(), negative ? 1 : 0);
}

void extractPlaneInliers(double distance_threshold, int max_iterations)
{
    extractModelInliers(kSacModelPlane, distance_threshold, max_iterations);
}

void extractModelInliers(int model_type, double distance_threshold, int max_iterations)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (!segmentModel(model_type, distance_threshold, max_iterations, coefficients, inliers)) {
        clearFilteredCloud();
        return;
    }

    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*filteredCloud());
    LOGI("ExtractIndices model=%d inliers: input=%zu output=%zu",
         model_type, input->points.size(), filteredCloud()->points.size());
}

void extractModelOutliers(int model_type, double distance_threshold, int max_iterations)
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
    extract.setNegative(true);
    extract.filter(*filteredCloud());
    LOGI("ExtractIndices model=%d outliers: input=%zu output=%zu",
         model_type, input->points.size(), filteredCloud()->points.size());
}

} // namespace pclmobile
