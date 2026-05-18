#ifndef PCL_MOBILE_FILTERS_H
#define PCL_MOBILE_FILTERS_H

#include <string>
#include <vector>

namespace pclmobile {

void filterAxis(const std::string& axis, double min_value, double max_value);
void filterAxisOutside(const std::string& axis, double min_value, double max_value);
void filterConditionalAxisRange(
        const std::string& axis, double min_value, double max_value, bool keep_organized);
void filterPassThroughAdvanced(
        const std::string& axis,
        double min_value,
        double max_value,
        bool negative,
        bool keep_organized,
        float user_filter_value);
void filterVoxelGrid(double x, double y, double z);
void filterVoxelGridMinimumPoints(double x, double y, double z, int minimum_points_per_voxel);
void filterVoxelGridCovariance(double x, double y, double z, int min_points_per_voxel, double min_covar_eigvalue_mult);
std::vector<float> computeVoxelGridOccludedVoxels(double x, double y, double z, int max_voxel_count);
void filterApproximateVoxelGrid(double x, double y, double z);
void filterUniformSampling(double radius);
void filterGridMinimum(double resolution);
void filterLocalMaximum(double radius);
void filterMedian(int window_size, double max_allowed_movement);
void filterRandomSample(int sample, int seed);
void filterFarthestPointSampling(int sample, int seed);
void filterNormalSpaceSampling(int sample, int seed, int bins_x, int bins_y, int bins_z, int normal_k_search);
void filterCovarianceSampling(int samples, int normal_k_search);
std::vector<float> computeCovarianceSamplingConditionNumber(int samples, int normal_k_search);
void filterFastBilateral(double sigma_s, double sigma_r);
void removeNaNFromActiveCloud();
void filterStatisticalOutlierRemoval(int mean_k, double stddev_mul_thresh);
void filterRadiusOutlierRemoval(double radius, int min_neighbors);
void filterShadowPoints(int normal_k_search, double threshold);
void filterCropBox(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z);
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
        double rotation_z);
void filterFrustumCulling(
        double horizontal_fov,
        double vertical_fov,
        double near_plane_distance,
        double far_plane_distance,
        const std::vector<float>& row_major_camera_pose);
void filterModelOutlierRemoval(
        int model_type,
        const std::vector<float>& model_coefficients,
        double threshold,
        bool negative);
void filterMorphological(double resolution, int morphological_operator);
void filterCropHull2D(const std::vector<float>& packed_hull_xyz, bool negative);
void filterExtractIndices(const std::vector<int>& indices, bool negative);
void extractPlaneInliers(double distance_threshold, int max_iterations);
void extractModelInliers(int model_type, double distance_threshold, int max_iterations);
void extractModelOutliers(int model_type, double distance_threshold, int max_iterations);

} // namespace pclmobile

#endif // PCL_MOBILE_FILTERS_H
