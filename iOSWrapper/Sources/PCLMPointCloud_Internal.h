// PCLMPointCloud_Internal.h
//
// C++ category giving implementation files access to the underlying
// pcl::PointCloud<pcl::PointXYZ> and a shared error-factory helper.
//
// ONLY include this from .mm (Objective-C++) translation units.

#import <PCLMobile/PCLMPointCloud.h>
#import <PCLMobile/PCLMGeometry.h>
#import <PCLMobile/PCLMSegmentation.h>
#import <PCLMobile/PCLMRegistration.h>

#include <memory>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using CloudT   = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = CloudT::Ptr;

// Shared error factory.
static inline NSError *PCLMobileMakeError(PCLMobileErrorCode code,
                                          NSString *description)
{
    return [NSError errorWithDomain:PCLMobileErrorDomain
                               code:code
                           userInfo:@{ NSLocalizedDescriptionKey: description }];
}

// Shared RANSAC helper used by Filters, Segmentation, and Surface.
static inline bool PCLMobileSACSegmentation(
    const CloudPtr &input,
    int modelType,
    double distanceThreshold,
    int maxIterations,
    pcl::ModelCoefficients::Ptr &coefficients,
    pcl::PointIndices::Ptr &inliers)
{
    if (input->empty()) return false;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(modelType);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(input);
    seg.segment(*inliers, *coefficients);
    return !inliers->indices.empty();
}

// Private category: exposes the C++ cloud pointer and a package-style
// designated initialiser so all .mm implementation files can create new
// PCLMPointCloud instances and read the underlying cloud without touching
// the anonymous class extension in PCLMPointCloud.mm.
@interface PCLMPointCloud (Internal)
- (nonnull instancetype)initWithCloud:(CloudPtr)cloud;
- (CloudPtr)cloud;
@end
