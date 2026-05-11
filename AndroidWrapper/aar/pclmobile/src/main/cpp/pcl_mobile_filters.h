#ifndef PCL_MOBILE_FILTERS_H
#define PCL_MOBILE_FILTERS_H

#include <string>

namespace pclmobile {

void filterAxis(const std::string& axis, double min_value, double max_value);
void filterVoxelGrid(double x, double y, double z);
void filterApproximateVoxelGrid(double x, double y, double z);
void filterUniformSampling(double radius);
void filterRandomSample(int sample, int seed);
void filterFarthestPointSampling(int sample, int seed);
void filterStatisticalOutlierRemoval(int mean_k, double stddev_mul_thresh);
void filterRadiusOutlierRemoval(double radius, int min_neighbors);
void filterCropBox(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z);
void extractPlaneInliers(double distance_threshold, int max_iterations);
void extractModelInliers(int model_type, double distance_threshold, int max_iterations);

} // namespace pclmobile

#endif // PCL_MOBILE_FILTERS_H
