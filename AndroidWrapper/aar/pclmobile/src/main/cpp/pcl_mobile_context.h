#ifndef PCL_MOBILE_CONTEXT_H
#define PCL_MOBILE_CONTEXT_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pclmobile {

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud();
pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud();
pcl::PointCloud<pcl::PointXYZ>::Ptr activeCloud();
void clearFilteredCloud();

} // namespace pclmobile

#endif // PCL_MOBILE_CONTEXT_H
