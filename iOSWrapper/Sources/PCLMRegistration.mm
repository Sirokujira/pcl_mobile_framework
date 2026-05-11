// PCLMRegistration.mm
//
// PCLMICPResult implementation and the ICP alignment method on PCLMPointCloud.
// Ported from pcl_mobile_registration.cpp.

#import "PCLMPointCloud_Internal.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

// ---------------------------------------------------------------------------
// PCLMICPResult
// ---------------------------------------------------------------------------

@interface PCLMICPResult ()
- (instancetype)initWithConverged:(BOOL)converged
                     fitnessScore:(double)fitnessScore
                        transform:(Eigen::Matrix4f)matrix NS_DESIGNATED_INITIALIZER;
@end

@implementation PCLMICPResult {
    BOOL   _converged;
    double _fitnessScore;
    float  _matrix[16];
}

- (instancetype)initWithConverged:(BOOL)converged
                     fitnessScore:(double)fitnessScore
                        transform:(Eigen::Matrix4f)matrix
{
    self = [super init];
    if (self) {
        _converged    = converged;
        _fitnessScore = fitnessScore;
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                _matrix[row * 4 + col] = matrix(row, col);
            }
        }
    }
    return self;
}

- (BOOL)converged       { return _converged; }
- (double)fitnessScore  { return _fitnessScore; }

- (void)copyTransformIntoBuffer:(float *)buffer
{
    memcpy(buffer, _matrix, 16 * sizeof(float));
}

@end

// ---------------------------------------------------------------------------
// PCLMPointCloud (Registration)
// ---------------------------------------------------------------------------

@implementation PCLMPointCloud (Registration)

- (nullable PCLMICPResult *)alignToTranslatedCopyICPWithTx:(float)tx
                                                        ty:(float)ty
                                                        tz:(float)tz
                                             maxIterations:(NSInteger)maxIterations
                                                     error:(NSError **)error
{
    if (maxIterations <= 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"maxIterations must be > 0");
        return nil;
    }

    CloudPtr source = self.cloud;
    if (source->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    CloudPtr target(new CloudT);
    Eigen::Affine3f t = Eigen::Affine3f::Identity();
    t.translation() << tx, ty, tz;
    pcl::transformPointCloud(*source, *target, t);

    CloudT aligned;
    auto icp = std::make_unique<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
    icp->setInputSource(source);
    icp->setInputTarget(target);
    icp->setMaximumIterations(static_cast<int>(maxIterations));
    icp->align(aligned);

    return [[PCLMICPResult alloc] initWithConverged:icp->hasConverged()
                                       fitnessScore:icp->getFitnessScore()
                                          transform:icp->getFinalTransformation()];
}

- (nullable PCLMICPResult *)alignToCloud:(PCLMPointCloud *)targetCloud
                           maxIterations:(NSInteger)maxIterations
                                   error:(NSError **)error
{
    if (maxIterations <= 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"maxIterations must be > 0");
        return nil;
    }

    CloudPtr source = self.cloud;
    if (source->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"source cloud is empty");
        return nil;
    }

    CloudPtr target = targetCloud.cloud;
    if (target->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"target cloud is empty");
        return nil;
    }

    CloudT aligned;
    auto icp = std::make_unique<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
    icp->setInputSource(source);
    icp->setInputTarget(target);
    icp->setMaximumIterations(static_cast<int>(maxIterations));
    icp->align(aligned);

    if (!icp->hasConverged()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInternal,
                                               @"ICP did not converge");
        return nil;
    }

    return [[PCLMICPResult alloc] initWithConverged:YES
                                       fitnessScore:icp->getFitnessScore()
                                          transform:icp->getFinalTransformation()];
}

@end
