// PCLMRegistration.h
// Result type for Iterative Closest Point (ICP) registration.

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/**
 * Result of an ICP alignment run.
 *
 * The 4×4 row-major transformation matrix that maps the source cloud onto
 * the target can be read out via `copyTransform(into:)`, which writes 16
 * `float` values into a caller-provided buffer.
 */
NS_SWIFT_NAME(ICPResult)
@interface PCLMICPResult : NSObject

/// Whether the ICP optimisation converged within the iteration budget.
@property (nonatomic, readonly) BOOL converged;

/// Sum of squared point-to-point distances after alignment (lower is better).
@property (nonatomic, readonly) double fitnessScore;

/**
 * Copies the 4×4 row-major transformation matrix into `buffer`.
 *
 * @param buffer  Must point to storage for at least 16 `float` values.
 */
- (void)copyTransformIntoBuffer:(float *)buffer
    NS_SWIFT_NAME(copyTransform(into:));

- (instancetype)init NS_UNAVAILABLE;

@end

NS_ASSUME_NONNULL_END
