// PCLMSearch.mm
//
// KD-tree and octree spatial-search methods. Ported from pcl_mobile_search.cpp.

#import "PCLMPointCloud_Internal.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>

// Packs (x, y, z, squared_distance) tuples for `found` results into NSData.
static NSData *packSearchResults(const CloudPtr &cloud,
                                 const std::vector<int> &indices,
                                 const std::vector<float> &sqDists,
                                 int found)
{
    NSMutableData *data = [NSMutableData dataWithLength:
                           static_cast<NSUInteger>(found) * 4 * sizeof(float)];
    float *ptr = static_cast<float *>(data.mutableBytes);
    for (int i = 0; i < found; ++i) {
        const auto &p = cloud->points[static_cast<std::size_t>(indices[i])];
        ptr[i * 4 + 0] = p.x;
        ptr[i * 4 + 1] = p.y;
        ptr[i * 4 + 2] = p.z;
        ptr[i * 4 + 3] = sqDists[i];
    }
    return data;
}

@implementation PCLMPointCloud (Search)

- (nullable NSData *)nearestKSearchAtX:(float)x
                                     y:(float)y
                                     z:(float)z
                                     k:(NSInteger)k
                                 error:(NSError **)error
{
    if (k <= 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"k must be > 0");
        return nil;
    }

    CloudPtr input = self.cloud;
    if (input->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(input);

    pcl::PointXYZ query;
    query.x = x; query.y = y; query.z = z;
    std::vector<int>   indices(static_cast<std::size_t>(k));
    std::vector<float> sqDists(static_cast<std::size_t>(k));
    const int found = tree.nearestKSearch(query, static_cast<int>(k),
                                          indices, sqDists);

    return packSearchResults(input, indices, sqDists, found);
}

- (nullable NSData *)octreeRadiusSearchAtX:(float)x
                                         y:(float)y
                                         z:(float)z
                                resolution:(double)resolution
                                    radius:(double)radius
                                     error:(NSError **)error
{
    if (resolution <= 0.0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"resolution must be > 0");
        return nil;
    }
    if (radius <= 0.0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"radius must be > 0");
        return nil;
    }

    CloudPtr input = self.cloud;
    if (input->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(input);
    octree.addPointsFromInputCloud();

    pcl::PointXYZ query;
    query.x = x; query.y = y; query.z = z;
    std::vector<int>   indices;
    std::vector<float> sqDists;
    const int found = octree.radiusSearch(query, radius, indices, sqDists);

    return packSearchResults(input, indices, sqDists, found);
}

@end
