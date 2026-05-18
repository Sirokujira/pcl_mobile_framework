#ifndef PCL_MOBILE_SEGMENTATION_H
#define PCL_MOBILE_SEGMENTATION_H

#include <jni.h>

#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

namespace pclmobile {

bool segmentPlane(double distance_threshold,
                  int max_iterations,
                  pcl::ModelCoefficients::Ptr coefficients,
                  pcl::PointIndices::Ptr inliers);
bool segmentModel(int model_type,
                  double distance_threshold,
                  int max_iterations,
                  pcl::ModelCoefficients::Ptr coefficients,
                  pcl::PointIndices::Ptr inliers);
bool segmentModelWithMethod(int model_type,
                            int method_type,
                            double distance_threshold,
                            int max_iterations,
                            pcl::ModelCoefficients::Ptr coefficients,
                            pcl::PointIndices::Ptr inliers);
std::vector<jfloat> segmentPlaneModel(double distance_threshold, int max_iterations);
std::vector<jfloat> segmentSphereModel(double distance_threshold, int max_iterations);
std::vector<jfloat> segmentSACModel(int model_type, double distance_threshold, int max_iterations);
std::vector<jfloat> segmentSACModelWithMethod(
        int model_type,
        int method_type,
        double distance_threshold,
        int max_iterations);
std::vector<jfloat> extractEuclideanClusters(double tolerance, int min_cluster_size, int max_cluster_size);
std::vector<jfloat> extractRegionGrowingClusters(int normal_k_search,
                                                 int number_of_neighbours,
                                                 int min_cluster_size,
                                                 int max_cluster_size,
                                                 double smoothness_threshold_degrees,
                                                 double curvature_threshold);
std::vector<jfloat> extractConditionalEuclideanClusters(double tolerance,
                                                        int min_cluster_size,
                                                        int max_cluster_size,
                                                        double max_z_delta);
pcl::PointCloud<pcl::PointXYZ>::Ptr extractPolygonalPrismData(
        const std::vector<jfloat>& packed_planar_hull_xyz,
        double height_min,
        double height_max,
        float view_point_x,
        float view_point_y,
        float view_point_z,
        bool negative);
pcl::PointCloud<pcl::PointXYZ>::Ptr extractProgressiveMorphologicalGround(
        int max_window_size,
        double slope,
        double initial_distance,
        double max_distance,
        double cell_size,
        double base,
        bool exponential,
        bool negative);
pcl::PointCloud<pcl::PointXYZ>::Ptr extractApproximateProgressiveMorphologicalGround(
        int max_window_size,
        double slope,
        double initial_distance,
        double max_distance,
        double cell_size,
        double base,
        bool exponential,
        int number_of_threads,
        bool negative);
pcl::PointCloud<pcl::PointXYZ>::Ptr extractMinCutForeground(
        const std::vector<jfloat>& packed_foreground_xyz,
        double sigma,
        double radius,
        double source_weight,
        int number_of_neighbours);
std::vector<jfloat> extractMinCutForegroundStats(
        const std::vector<jfloat>& packed_foreground_xyz,
        double sigma,
        double radius,
        double source_weight,
        int number_of_neighbours);
pcl::PointCloud<pcl::PointXYZ>::Ptr segmentDifferencesAgainstTarget(
        const std::vector<jfloat>& packed_target_xyz,
        double distance_threshold);

} // namespace pclmobile

#endif // PCL_MOBILE_SEGMENTATION_H
