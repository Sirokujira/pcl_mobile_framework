// PCLMSegmentation.h
// Result types for RANSAC plane / sphere / cylinder model fitting.

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/**
 * Result of a RANSAC plane-model fit.
 *
 * The fitted plane satisfies `a·x + b·y + c·z + d = 0`.
 * `inlierCount` and `inputCount` let callers assess fit quality.
 */
NS_SWIFT_NAME(PlaneModel)
@interface PCLMPlaneModel : NSObject

@property (nonatomic, readonly) float a;
@property (nonatomic, readonly) float b;
@property (nonatomic, readonly) float c;
@property (nonatomic, readonly) float d;
@property (nonatomic, readonly) NSUInteger inlierCount;
@property (nonatomic, readonly) NSUInteger inputCount;

- (instancetype)init NS_UNAVAILABLE;

@end

/**
 * Result of a RANSAC sphere-model fit.
 *
 * The fitted sphere has centre `(centerX, centerY, centerZ)` and `radius`.
 */
NS_SWIFT_NAME(SphereModel)
@interface PCLMSphereModel : NSObject

@property (nonatomic, readonly) float centerX;
@property (nonatomic, readonly) float centerY;
@property (nonatomic, readonly) float centerZ;
@property (nonatomic, readonly) float radius;
@property (nonatomic, readonly) NSUInteger inlierCount;
@property (nonatomic, readonly) NSUInteger inputCount;

- (instancetype)init NS_UNAVAILABLE;

@end

/**
 * Result of a RANSAC cylinder-model fit via SACSegmentationFromNormals.
 *
 * The cylinder axis passes through `(pointX, pointY, pointZ)` in the
 * direction `(axisX, axisY, axisZ)` (unit vector) with the given `radius`.
 */
NS_SWIFT_NAME(CylinderModel)
@interface PCLMCylinderModel : NSObject

/// A point on the cylinder axis.
@property (nonatomic, readonly) float pointX;
@property (nonatomic, readonly) float pointY;
@property (nonatomic, readonly) float pointZ;

/// Unit direction vector of the cylinder axis.
@property (nonatomic, readonly) float axisX;
@property (nonatomic, readonly) float axisY;
@property (nonatomic, readonly) float axisZ;

@property (nonatomic, readonly) float radius;
@property (nonatomic, readonly) NSUInteger inlierCount;
@property (nonatomic, readonly) NSUInteger inputCount;

- (instancetype)init NS_UNAVAILABLE;

@end

NS_ASSUME_NONNULL_END
