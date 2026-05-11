// PCLMSurface.mm
//
// Surface reconstruction: ConvexHull, ConcaveHull, MLS smoothing,
// ProjectInliers.

#import "PCLMPointCloud_Internal.h"

#include <pcl/filters/project_inliers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>

@implementation PCLMPointCloud (Surface)

- (nullable PCLMPointCloud *)convexHullWithError:(NSError **)error
{
    CloudPtr input = self.cloud;
    if (input->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    CloudPtr hull(new CloudT);
    pcl::ConvexHull<pcl::PointXYZ> ch;
    ch.setInputCloud(input);
    ch.reconstruct(*hull);
    return [[PCLMPointCloud alloc] initWithCloud:std::move(hull)];
}

- (nullable PCLMPointCloud *)projectInliersToPlaneWithDistanceThreshold:(double)distanceThreshold
                                                           maxIterations:(NSInteger)maxIterations
                                                                   error:(NSError **)error
{
    if (distanceThreshold <= 0.0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"distanceThreshold must be > 0");
        return nil;
    }
    if (maxIterations <= 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"maxIterations must be > 0");
        return nil;
    }

    CloudPtr input = self.cloud;
    if (input->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    if (!PCLMobileSACSegmentation(input, pcl::SACMODEL_PLANE, distanceThreshold,
                                  static_cast<int>(maxIterations), coeff, inliers)) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidPCDFormat,
                                               @"no plane found for projection");
        return nil;
    }

    CloudPtr projected(new CloudT);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(input);
    proj.setIndices(inliers);
    proj.setModelCoefficients(coeff);
    proj.filter(*projected);
    return [[PCLMPointCloud alloc] initWithCloud:std::move(projected)];
}

- (nullable PCLMPointCloud *)concaveHullWithAlpha:(double)alpha
                                            error:(NSError **)error
{
    if (alpha <= 0.0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"alpha must be > 0");
        return nil;
    }
    CloudPtr input = self.cloud;
    if (input->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    CloudPtr hull(new CloudT);
    pcl::ConcaveHull<pcl::PointXYZ> ch;
    ch.setInputCloud(input);
    ch.setAlpha(alpha);
    ch.reconstruct(*hull);
    return [[PCLMPointCloud alloc] initWithCloud:std::move(hull)];
}

- (nullable PCLMPointCloud *)movingLeastSquaresSmoothedWithSearchRadius:(double)searchRadius
                                                                  error:(NSError **)error
{
    if (searchRadius <= 0.0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"searchRadius must be > 0");
        return nil;
    }
    CloudPtr input = self.cloud;
    if (input->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr smoothed(new pcl::PointCloud<pcl::PointNormal>);

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(input);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(searchRadius);
    mls.process(*smoothed);

    // Extract XYZ from PointNormal output
    CloudPtr output(new CloudT);
    output->reserve(smoothed->size());
    for (const auto &pt : *smoothed) {
        output->emplace_back(pt.x, pt.y, pt.z);
    }
    return [[PCLMPointCloud alloc] initWithCloud:std::move(output)];
}

@end
