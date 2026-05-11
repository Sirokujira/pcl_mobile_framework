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

std::vector<jfloat> radiusSearch(float x, float y, float z, double radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || radius <= 0.0) {
        return {};
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(input);

    pcl::PointXYZ query;
    query.x = x;
    query.y = y;
    query.z = z;
    std::vector<int> indices;
    std::vector<float> squared_distances;
    const int found = tree.radiusSearch(query, radius, indices, squared_distances);

    std::vector<jfloat> values;
    values.reserve(static_cast<std::size_t>(found) * 4);
    for (int i = 0; i < found; i++) {
        appendPoint(values, input->points[indices[i]]);
        values.push_back(squared_distances[i]);
    }
    LOGI("KdTreeFLANN radiusSearch: input=%zu found=%d radius=%.3f",
         input->points.size(), found, radius);
    return values;
}

std::vector<jfloat> octreeNearestKSearch(float x, float y, float z, double resolution, int k)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || resolution <= 0.0 || k <= 0) {
        return {};
    }

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(input);
    octree.addPointsFromInputCloud();

    pcl::PointXYZ query;
    query.x = x;
    query.y = y;
    query.z = z;
    std::vector<int> indices(static_cast<std::size_t>(k));
    std::vector<float> squared_distances(static_cast<std::size_t>(k));
    const int found = octree.nearestKSearch(query, k, indices, squared_distances);

    std::vector<jfloat> values;
    values.reserve(static_cast<std::size_t>(found) * 4);
    for (int i = 0; i < found; i++) {
        appendPoint(values, input->points[indices[i]]);
        values.push_back(squared_distances[i]);
    }
    LOGI("OctreePointCloudSearch nearestKSearch: input=%zu found=%d resolution=%.3f k=%d",
         input->points.size(), found, resolution, k);
    return values;
}

std::vector<jfloat> octreeRadiusSearch(float x, float y, float z, double resolution, double radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || resolution <= 0.0 || radius <= 0.0) {
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

std::vector<jfloat> octreeVoxelSearch(float x, float y, float z, double resolution)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || resolution <= 0.0) {
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
    const bool found = octree.voxelSearch(query, indices);

    std::vector<jfloat> values;
    values.reserve(indices.size() * 4);
    if (found) {
        for (int index : indices) {
            const pcl::PointXYZ& point = input->points[index];
            const float dx = point.x - query.x;
            const float dy = point.y - query.y;
            const float dz = point.z - query.z;
            appendPoint(values, point);
            values.push_back(dx * dx + dy * dy + dz * dz);
        }
    }
    LOGI("OctreePointCloudSearch voxelSearch: input=%zu found=%zu resolution=%.3f",
         input->points.size(), indices.size(), resolution);
    return values;
}

} // namespace pclmobile
