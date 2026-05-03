#include "pcl_mobile_context.h"

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

} // namespace

namespace pclmobile {

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud()
{
    return g_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud()
{
    return g_cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr activeCloud()
{
    return g_cloud_filtered->empty() ? g_cloud : g_cloud_filtered;
}

void clearFilteredCloud()
{
    g_cloud_filtered->clear();
}

} // namespace pclmobile
