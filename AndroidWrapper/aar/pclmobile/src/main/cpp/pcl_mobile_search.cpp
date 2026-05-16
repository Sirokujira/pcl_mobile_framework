#include "pcl_mobile_search.h"

#include <algorithm>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>

#include "pcl_mobile_arrays.h"
#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"

namespace pclmobile {

namespace {

pcl::PointXYZ makeQueryPoint(float x, float y, float z)
{
    pcl::PointXYZ query;
    query.x = x;
    query.y = y;
    query.z = z;
    return query;
}

unsigned int sanitizeMaxNeighbors(int max_neighbors)
{
    return static_cast<unsigned int>(std::max(max_neighbors, 0));
}

std::vector<jfloat> packSearchResult(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        const std::vector<int>& indices,
        const std::vector<float>& squared_distances,
        int found)
{
    std::vector<jfloat> values;
    if (found <= 0) {
        return values;
    }

    const std::size_t count = std::min(
            static_cast<std::size_t>(found),
            std::min(indices.size(), squared_distances.size()));
    values.reserve(count * 4);
    for (std::size_t i = 0; i < count; i++) {
        const int index = indices[i];
        if (index < 0 || static_cast<std::size_t>(index) >= input->points.size()) {
            continue;
        }
        appendPoint(values, input->points[static_cast<std::size_t>(index)]);
        values.push_back(squared_distances[i]);
    }
    return values;
}

std::vector<jfloat> packIndexResult(
        const std::vector<int>& indices,
        const std::vector<float>& squared_distances,
        int found)
{
    std::vector<jfloat> values;
    if (found <= 0) {
        return values;
    }

    const std::size_t count = std::min(
            static_cast<std::size_t>(found),
            std::min(indices.size(), squared_distances.size()));
    values.reserve(count * 2);
    for (std::size_t i = 0; i < count; i++) {
        if (indices[i] < 0) {
            continue;
        }
        values.push_back(static_cast<jfloat>(indices[i]));
        values.push_back(squared_distances[i]);
    }
    return values;
}

std::vector<jfloat> radiusSearchInternal(
        float x, float y, float z, double radius, unsigned int max_neighbors)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || radius <= 0.0) {
        return {};
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(input);

    std::vector<int> indices;
    std::vector<float> squared_distances;
    const int found = tree.radiusSearch(
            makeQueryPoint(x, y, z), radius, indices, squared_distances, max_neighbors);

    LOGI("KdTreeFLANN radiusSearch: input=%zu found=%d radius=%.3f maxNeighbors=%u",
         input->points.size(), found, radius, max_neighbors);
    return packSearchResult(input, indices, squared_distances, found);
}

std::vector<jfloat> radiusSearchIndicesInternal(
        float x, float y, float z, double radius, unsigned int max_neighbors)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || radius <= 0.0) {
        return {};
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(input);

    std::vector<int> indices;
    std::vector<float> squared_distances;
    const int found = tree.radiusSearch(
            makeQueryPoint(x, y, z), radius, indices, squared_distances, max_neighbors);

    LOGI("KdTreeFLANN radiusSearch indices: input=%zu found=%d radius=%.3f maxNeighbors=%u",
         input->points.size(), found, radius, max_neighbors);
    return packIndexResult(indices, squared_distances, found);
}

std::vector<jfloat> octreeRadiusSearchInternal(
        float x, float y, float z, double resolution, double radius, pcl::uindex_t max_neighbors)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || resolution <= 0.0 || radius <= 0.0) {
        return {};
    }

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(input);
    octree.addPointsFromInputCloud();

    std::vector<int> indices;
    std::vector<float> squared_distances;
    const pcl::uindex_t found = octree.radiusSearch(
            makeQueryPoint(x, y, z), radius, indices, squared_distances, max_neighbors);

    LOGI("OctreePointCloudSearch radiusSearch: input=%zu found=%u resolution=%.3f radius=%.3f maxNeighbors=%u",
         input->points.size(), static_cast<unsigned int>(found), resolution, radius,
         static_cast<unsigned int>(max_neighbors));
    return packSearchResult(input, indices, squared_distances, static_cast<int>(found));
}

std::vector<jfloat> octreeRadiusSearchIndicesInternal(
        float x, float y, float z, double resolution, double radius, pcl::uindex_t max_neighbors)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || resolution <= 0.0 || radius <= 0.0) {
        return {};
    }

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(input);
    octree.addPointsFromInputCloud();

    std::vector<int> indices;
    std::vector<float> squared_distances;
    const pcl::uindex_t found = octree.radiusSearch(
            makeQueryPoint(x, y, z), radius, indices, squared_distances, max_neighbors);

    LOGI("OctreePointCloudSearch radiusSearch indices: input=%zu found=%u resolution=%.3f radius=%.3f maxNeighbors=%u",
         input->points.size(), static_cast<unsigned int>(found), resolution, radius,
         static_cast<unsigned int>(max_neighbors));
    return packIndexResult(indices, squared_distances, static_cast<int>(found));
}

} // namespace

std::vector<jfloat> nearestKSearch(float x, float y, float z, int k)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || k <= 0) {
        return {};
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(input);

    std::vector<int> indices(static_cast<std::size_t>(k));
    std::vector<float> squared_distances(static_cast<std::size_t>(k));
    const int found = tree.nearestKSearch(makeQueryPoint(x, y, z), k, indices, squared_distances);

    LOGI("KdTreeFLANN nearestKSearch: input=%zu found=%d k=%d", input->points.size(), found, k);
    return packSearchResult(input, indices, squared_distances, found);
}

std::vector<jfloat> nearestKSearchIndices(float x, float y, float z, int k)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || k <= 0) {
        return {};
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(input);

    std::vector<int> indices(static_cast<std::size_t>(k));
    std::vector<float> squared_distances(static_cast<std::size_t>(k));
    const int found = tree.nearestKSearch(makeQueryPoint(x, y, z), k, indices, squared_distances);

    LOGI("KdTreeFLANN nearestKSearch indices: input=%zu found=%d k=%d", input->points.size(), found, k);
    return packIndexResult(indices, squared_distances, found);
}

std::vector<jfloat> radiusSearch(float x, float y, float z, double radius)
{
    return radiusSearchInternal(x, y, z, radius, 0);
}

std::vector<jfloat> radiusSearchIndices(float x, float y, float z, double radius)
{
    return radiusSearchIndicesInternal(x, y, z, radius, 0);
}

std::vector<jfloat> radiusSearchLimited(float x, float y, float z, double radius, int max_neighbors)
{
    return radiusSearchInternal(x, y, z, radius, sanitizeMaxNeighbors(max_neighbors));
}

std::vector<jfloat> radiusSearchIndicesLimited(float x, float y, float z, double radius, int max_neighbors)
{
    return radiusSearchIndicesInternal(x, y, z, radius, sanitizeMaxNeighbors(max_neighbors));
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

    std::vector<int> indices(static_cast<std::size_t>(k));
    std::vector<float> squared_distances(static_cast<std::size_t>(k));
    const int found = octree.nearestKSearch(makeQueryPoint(x, y, z), k, indices, squared_distances);

    LOGI("OctreePointCloudSearch nearestKSearch: input=%zu found=%d resolution=%.3f k=%d",
         input->points.size(), found, resolution, k);
    return packSearchResult(input, indices, squared_distances, found);
}

std::vector<jfloat> octreeNearestKSearchIndices(float x, float y, float z, double resolution, int k)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || resolution <= 0.0 || k <= 0) {
        return {};
    }

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(input);
    octree.addPointsFromInputCloud();

    std::vector<int> indices(static_cast<std::size_t>(k));
    std::vector<float> squared_distances(static_cast<std::size_t>(k));
    const int found = octree.nearestKSearch(makeQueryPoint(x, y, z), k, indices, squared_distances);

    LOGI("OctreePointCloudSearch nearestKSearch indices: input=%zu found=%d resolution=%.3f k=%d",
         input->points.size(), found, resolution, k);
    return packIndexResult(indices, squared_distances, found);
}

std::vector<jfloat> octreeRadiusSearch(float x, float y, float z, double resolution, double radius)
{
    return octreeRadiusSearchInternal(x, y, z, resolution, radius, 0);
}

std::vector<jfloat> octreeRadiusSearchIndices(float x, float y, float z, double resolution, double radius)
{
    return octreeRadiusSearchIndicesInternal(x, y, z, resolution, radius, 0);
}

std::vector<jfloat> octreeRadiusSearchLimited(
        float x, float y, float z, double resolution, double radius, int max_neighbors)
{
    return octreeRadiusSearchInternal(
            x, y, z, resolution, radius, static_cast<pcl::uindex_t>(sanitizeMaxNeighbors(max_neighbors)));
}

std::vector<jfloat> octreeRadiusSearchIndicesLimited(
        float x, float y, float z, double resolution, double radius, int max_neighbors)
{
    return octreeRadiusSearchIndicesInternal(
            x, y, z, resolution, radius, static_cast<pcl::uindex_t>(sanitizeMaxNeighbors(max_neighbors)));
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

    const pcl::PointXYZ query = makeQueryPoint(x, y, z);
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

std::vector<jfloat> octreeVoxelSearchIndices(float x, float y, float z, double resolution)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || resolution <= 0.0) {
        return {};
    }

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(input);
    octree.addPointsFromInputCloud();

    const pcl::PointXYZ query = makeQueryPoint(x, y, z);
    std::vector<int> indices;
    const bool found = octree.voxelSearch(query, indices);

    std::vector<jfloat> values;
    values.reserve(indices.size() * 2);
    if (found) {
        for (int index : indices) {
            if (index < 0 || static_cast<std::size_t>(index) >= input->points.size()) {
                continue;
            }
            const pcl::PointXYZ& point = input->points[static_cast<std::size_t>(index)];
            const float dx = point.x - query.x;
            const float dy = point.y - query.y;
            const float dz = point.z - query.z;
            values.push_back(static_cast<jfloat>(index));
            values.push_back(dx * dx + dy * dy + dz * dz);
        }
    }
    LOGI("OctreePointCloudSearch voxelSearch indices: input=%zu found=%zu resolution=%.3f",
         input->points.size(), indices.size(), resolution);
    return values;
}

std::vector<jfloat> octreeApproxNearestSearch(float x, float y, float z, double resolution)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || resolution <= 0.0) {
        return {};
    }

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(input);
    octree.addPointsFromInputCloud();

    pcl::index_t index = -1;
    float squared_distance = 0.0f;
    octree.approxNearestSearch(makeQueryPoint(x, y, z), index, squared_distance);
    if (index < 0 || static_cast<std::size_t>(index) >= input->points.size()) {
        return {};
    }

    std::vector<jfloat> values;
    values.reserve(4);
    appendPoint(values, input->points[static_cast<std::size_t>(index)]);
    values.push_back(squared_distance);
    LOGI("OctreePointCloudSearch approxNearestSearch: input=%zu resolution=%.3f index=%d",
         input->points.size(), resolution, static_cast<int>(index));
    return values;
}

std::vector<jfloat> octreeApproxNearestSearchIndex(float x, float y, float z, double resolution)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || resolution <= 0.0) {
        return {};
    }

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(input);
    octree.addPointsFromInputCloud();

    pcl::index_t index = -1;
    float squared_distance = 0.0f;
    octree.approxNearestSearch(makeQueryPoint(x, y, z), index, squared_distance);
    if (index < 0 || static_cast<std::size_t>(index) >= input->points.size()) {
        return {};
    }

    LOGI("OctreePointCloudSearch approxNearestSearch index: input=%zu resolution=%.3f index=%d",
         input->points.size(), resolution, static_cast<int>(index));
    return {
            static_cast<jfloat>(index),
            squared_distance,
    };
}

} // namespace pclmobile
