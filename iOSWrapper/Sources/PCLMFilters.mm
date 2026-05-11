// PCLMFilters.mm
//
// Outlier-removal and inlier-extraction filters ported from the Android
// pclmobile JNI wrapper (pcl_mobile_filters.cpp).

#import "PCLMPointCloud_Internal.h"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

@implementation PCLMPointCloud (Filters)

- (nullable PCLMPointCloud *)statisticalOutlierRemovalWithMeanK:(NSInteger)meanK
                                               stddevMulThresh:(double)stddevMulThresh
                                                         error:(NSError **)error
{
    if (meanK <= 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"meanK must be > 0");
        return nil;
    }
    CloudPtr filtered(new CloudT);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(self.cloud);
    sor.setMeanK(static_cast<int>(meanK));
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter(*filtered);
    return [[PCLMPointCloud alloc] initWithCloud:std::move(filtered)];
}

- (nullable PCLMPointCloud *)radiusOutlierRemovalWithRadius:(double)radius
                                               minNeighbors:(NSInteger)minNeighbors
                                                      error:(NSError **)error
{
    if (radius <= 0.0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"radius must be > 0");
        return nil;
    }
    if (minNeighbors < 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"minNeighbors must be >= 0");
        return nil;
    }
    CloudPtr filtered(new CloudT);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(self.cloud);
    ror.setRadiusSearch(radius);
    ror.setMinNeighborsInRadius(static_cast<int>(minNeighbors));
    ror.filter(*filtered);
    return [[PCLMPointCloud alloc] initWithCloud:std::move(filtered)];
}

- (nullable PCLMPointCloud *)cropBoxWithMinX:(double)minX
                                        minY:(double)minY
                                        minZ:(double)minZ
                                        maxX:(double)maxX
                                        maxY:(double)maxY
                                        maxZ:(double)maxZ
                                       error:(NSError **)error
{
    if (minX >= maxX || minY >= maxY || minZ >= maxZ) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"each min value must be < corresponding max value");
        return nil;
    }
    CloudPtr filtered(new CloudT);
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(self.cloud);
    crop.setMin(Eigen::Vector4f(static_cast<float>(minX),
                                static_cast<float>(minY),
                                static_cast<float>(minZ), 1.0f));
    crop.setMax(Eigen::Vector4f(static_cast<float>(maxX),
                                static_cast<float>(maxY),
                                static_cast<float>(maxZ), 1.0f));
    crop.filter(*filtered);
    return [[PCLMPointCloud alloc] initWithCloud:std::move(filtered)];
}

- (nullable PCLMPointCloud *)extractPlaneInliersWithDistanceThreshold:(double)distanceThreshold
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

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    if (!PCLMobileSACSegmentation(input, pcl::SACMODEL_PLANE, distanceThreshold,
                                  static_cast<int>(maxIterations), coefficients, inliers)) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidPCDFormat,
                                               @"no plane inliers found");
        return nil;
    }

    CloudPtr filtered(new CloudT);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*filtered);
    return [[PCLMPointCloud alloc] initWithCloud:std::move(filtered)];
}

@end
