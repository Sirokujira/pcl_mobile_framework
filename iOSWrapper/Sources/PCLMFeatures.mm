// PCLMFeatures.mm
//
// Surface-normal estimation ported from pcl_mobile_features.cpp.

#import "PCLMPointCloud_Internal.h"

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

@implementation PCLMPointCloud (Features)

- (nullable NSData *)estimateNormalsWithKSearch:(NSInteger)kSearch
                                          error:(NSError **)error
{
    if (kSearch <= 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"kSearch must be > 0");
        return nil;
    }

    CloudPtr input = self.cloud;
    if (input->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input);
    ne.setSearchMethod(
        pcl::search::KdTree<pcl::PointXYZ>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZ>));
    ne.setKSearch(static_cast<int>(kSearch));
    ne.compute(*normals);

    // Pack as float[4*N]: nx, ny, nz, curvature per normal.
    const NSUInteger n = normals->size();
    NSMutableData *data = [NSMutableData dataWithLength:n * 4 * sizeof(float)];
    float *ptr = static_cast<float *>(data.mutableBytes);
    for (NSUInteger i = 0; i < n; ++i) {
        const auto &p = normals->points[i];
        ptr[i * 4 + 0] = p.normal_x;
        ptr[i * 4 + 1] = p.normal_y;
        ptr[i * 4 + 2] = p.normal_z;
        ptr[i * 4 + 3] = p.curvature;
    }
    return data;
}

@end
