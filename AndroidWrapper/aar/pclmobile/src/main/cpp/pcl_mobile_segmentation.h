#ifndef PCL_MOBILE_SEGMENTATION_H
#define PCL_MOBILE_SEGMENTATION_H

#include <jni.h>

#include <vector>

#include <pcl/ModelCoefficients.h>
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
std::vector<jfloat> segmentPlaneModel(double distance_threshold, int max_iterations);
std::vector<jfloat> segmentSphereModel(double distance_threshold, int max_iterations);
std::vector<jfloat> segmentSACModel(int model_type, double distance_threshold, int max_iterations);
std::vector<jfloat> extractEuclideanClusters(double tolerance, int min_cluster_size, int max_cluster_size);

} // namespace pclmobile

#endif // PCL_MOBILE_SEGMENTATION_H
