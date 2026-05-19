#ifndef PCL_MOBILE_KEYPOINTS_H
#define PCL_MOBILE_KEYPOINTS_H

#include <jni.h>

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pclmobile {

pcl::PointCloud<pcl::PointXYZ>::Ptr computeISSKeypoints(
        double salient_radius,
        double non_max_radius,
        double threshold21,
        double threshold32,
        int min_neighbors);

std::vector<jfloat> computeSIFTKeypoints(
        double min_scale,
        int nr_octaves,
        int nr_scales_per_octave,
        double min_contrast);
std::vector<jfloat> computeHarrisKeypoints(
        int response_method,
        double radius,
        double threshold,
        bool non_max_suppression,
        bool refine);
std::vector<jfloat> computeHarris6DKeypoints(
        double radius,
        double threshold,
        bool non_max_suppression,
        bool refine,
        int number_of_threads);
std::vector<jfloat> computeHarris2DKeypoints(
        int response_method,
        int window_width,
        int window_height,
        int min_distance,
        double threshold,
        bool non_max_suppression,
        bool refine);
std::vector<jfloat> computeSUSANKeypoints(
        double radius,
        double distance_threshold,
        double angular_threshold,
        double intensity_threshold,
        bool non_max_suppression,
        bool geometric_validation);
std::vector<jfloat> computeTrajkovicKeypoints(
        int method,
        int window_size,
        double first_threshold,
        double second_threshold,
        int normal_k_search);
std::vector<jfloat> computeTrajkovic2DKeypoints(
        int method,
        int window_size,
        double first_threshold,
        double second_threshold);
std::vector<jfloat> computeBRISK2DKeypoints(
        int threshold,
        int octaves,
        bool remove_invalid_3d_keypoints);
std::vector<jfloat> computeAGAST2DKeypoints(
        double threshold,
        double max_data_value,
        bool non_max_suppression,
        int max_keypoints);
pcl::PointCloud<pcl::PointXYZ>::Ptr computeUniformSamplingKeypoints(double radius);

} // namespace pclmobile

#endif // PCL_MOBILE_KEYPOINTS_H
