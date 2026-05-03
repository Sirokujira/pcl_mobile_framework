#include "pcl_mobile_features.h"

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"

namespace pclmobile {

std::vector<jfloat> estimateNormals(int k_search)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    if (input->empty()) {
        return {};
    }

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(input);
    normal_estimation.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    normal_estimation.setKSearch(k_search);
    normal_estimation.compute(*normals);

    std::vector<jfloat> values;
    values.reserve(normals->points.size() * 4);
    for (const auto& normal : normals->points) {
        values.push_back(normal.normal_x);
        values.push_back(normal.normal_y);
        values.push_back(normal.normal_z);
        values.push_back(normal.curvature);
    }
    LOGI("NormalEstimation computed normals: input=%zu normals=%zu k=%d",
         input->points.size(), normals->points.size(), k_search);
    return values;
}

} // namespace pclmobile
