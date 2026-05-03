#include "pcl_mobile_search.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>

#include "pcl_mobile_arrays.h"
#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"

namespace pclmobile {

std::vector<jfloat> nearestKSearch(float x, float y, float z, int k)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || k <= 0) {
        return {};
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(input);

    pcl::PointXYZ query;
    query.x = x;
    query.y = y;
    query.z = z;
    std::vector<int> indices(static_cast<std::size_t>(k));
    std::vector<float> squared_distances(static_cast<std::size_t>(k));
    const int found = tree.nearestKSearch(query, k, indices, squared_distances);

    std::vector<jfloat> values;
    values.reserve(static_cast<std::size_t>(found) * 4);
    for (int i = 0; i < found; i++) {
        appendPoint(values, input->points[indices[i]]);
        values.push_back(squared_distances[i]);
    }
    LOGI("KdTreeFLANN nearestKSearch: input=%zu found=%d k=%d", input->points.size(), found, k);
    return values;
}

std::vector<jfloat> octreeRadiusSearch(float x, float y, float z, double resolution, double radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(input);
    octree.addPointsFromInputCloud();

    pcl::PointXYZ query;
    query.x = x;
    query.y = y;
    query.z = z;
    std::vector<int> indices;
    std::vector<float> squared_distances;
    const int found = octree.radiusSearch(query, radius, indices, squared_distances);

    std::vector<jfloat> values;
    values.reserve(static_cast<std::size_t>(found) * 4);
    for (int i = 0; i < found; i++) {
        appendPoint(values, input->points[indices[i]]);
        values.push_back(squared_distances[i]);
    }
    LOGI("OctreePointCloudSearch radiusSearch: input=%zu found=%d resolution=%.3f radius=%.3f",
         input->points.size(), found, resolution, radius);
    return values;
}

} // namespace pclmobile
