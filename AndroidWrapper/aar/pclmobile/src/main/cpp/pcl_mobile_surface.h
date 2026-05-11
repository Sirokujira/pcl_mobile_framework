#ifndef PCL_MOBILE_SURFACE_H
#define PCL_MOBILE_SURFACE_H

#include <jni.h>

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pclmobile {

std::vector<jfloat> computeCentroidAndBounds();
pcl::PointCloud<pcl::PointXYZ>::Ptr computeConvexHull();
pcl::PointCloud<pcl::PointXYZ>::Ptr computeConcaveHull(double alpha);
pcl::PointCloud<pcl::PointXYZ>::Ptr projectInliersToPlane(double distance_threshold, int max_iterations);
pcl::PointCloud<pcl::PointXYZ>::Ptr smoothMovingLeastSquares(double search_radius);

} // namespace pclmobile

#endif // PCL_MOBILE_SURFACE_H
