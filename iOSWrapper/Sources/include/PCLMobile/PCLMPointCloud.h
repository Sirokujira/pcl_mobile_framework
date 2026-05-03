// PCLMPointCloud.h
//
// Public point cloud Objective-C interface. The implementation
// (PCLMPointCloud.mm) is the only file that needs to know about
// pcl::PointCloud<pcl::PointXYZ>; consumers see a plain Cocoa API.

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/**
 * Errors emitted by `PCLMPointCloud`.
 *
 * `domain` is `PCLMobileErrorDomain`. `userInfo[NSLocalizedDescriptionKey]`
 * carries a short human-readable summary suitable for logging.
 */
FOUNDATION_EXPORT NSErrorDomain const PCLMobileErrorDomain;

typedef NS_ERROR_ENUM(PCLMobileErrorDomain, PCLMobileErrorCode) {
    PCLMobileErrorCodeFileNotFound      = 1,
    PCLMobileErrorCodeInvalidPCDFormat  = 2,
    PCLMobileErrorCodeInvalidArgument   = 3,
    PCLMobileErrorCodeInternal          = 99,
};

/**
 * Immutable wrapper around an underlying `pcl::PointCloud<pcl::PointXYZ>`.
 *
 * Construct one with `+cloudFromPCDFile:error:` and treat it as a value
 * type — most operations return a new instance rather than mutating self.
 */
NS_SWIFT_NAME(PointCloud)
@interface PCLMPointCloud : NSObject

/// Number of points held by the underlying cloud.
@property (nonatomic, readonly) NSUInteger pointCount;

/// Width of the cloud (column count for organised clouds, point count otherwise).
@property (nonatomic, readonly) NSUInteger width;

/// Height of the cloud (row count for organised clouds, 1 otherwise).
@property (nonatomic, readonly) NSUInteger height;

/**
 * Load a cloud from a PCD file on disk.
 *
 * @param path   Absolute path to the .pcd file.
 * @param error  On failure, populated with a `PCLMobileError` describing the
 *               cause. May be NULL.
 * @return A new cloud, or nil on error.
 */
+ (nullable instancetype)cloudFromPCDFile:(NSString *)path
                                    error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(load(pcdAt:));

/**
 * Build a cloud from a tightly-packed float buffer.
 *
 * The buffer must contain `count * 3` floats laid out as
 * `x0, y0, z0, x1, y1, z1, ...`. This matches the memory layout of
 * `simd_float3` arrays exposed by ARKit (`ARFrame.rawFeaturePoints
 * .points`), so callers can pass them directly.
 *
 * @param packedXYZ  Pointer to `count * 3` consecutive floats.
 * @param count      Number of points (NOT number of floats).
 * @param error      Populated on failure.
 */
+ (nullable instancetype)cloudFromPackedXYZ:(const float *)packedXYZ
                                       count:(NSUInteger)count
                                       error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(make(packedXYZ:count:));

/**
 * Copy the cloud's points into a caller-provided float buffer in
 * `x0,y0,z0,x1,y1,z1,...` order.
 *
 * @param buffer        Destination, must hold at least `capacity * 3` floats.
 * @param capacity      Maximum number of points the buffer can hold.
 * @param outCount      On success, set to the number of points actually
 *                      written (≤ `capacity`). May be NULL.
 * @param error         Populated on failure.
 */
- (BOOL)copyPackedXYZIntoBuffer:(float *)buffer
                       capacity:(NSUInteger)capacity
                    actualCount:(NSUInteger * _Nullable)outCount
                          error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(copyPackedXYZ(into:capacity:actualCount:));

/**
 * Save the cloud to a PCD file (ASCII format).
 *
 * @param path   Destination path.
 * @param error  Populated on failure.
 */
- (BOOL)writePCDFileAtPath:(NSString *)path
                     error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(write(pcdAt:));

/**
 * Apply a voxel-grid downsample, returning the resulting cloud.
 *
 * @param leafSize  Voxel side length in metres. Must be positive.
 * @param error     Populated on failure.
 */
- (nullable PCLMPointCloud *)voxelGridDownsampleWithLeaf:(double)leafSize
                                                   error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(voxelGridDownsampled(leaf:));

/**
 * Apply a pass-through filter on a single axis.
 *
 * @param axis     "x", "y" or "z".
 * @param minValue Lower limit of the kept range.
 * @param maxValue Upper limit of the kept range.
 * @param error    Populated on failure.
 */
- (nullable PCLMPointCloud *)passThroughFilteredOnAxis:(NSString *)axis
                                              minValue:(double)minValue
                                              maxValue:(double)maxValue
                                                 error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(passThroughFiltered(axis:min:max:));

@end

NS_ASSUME_NONNULL_END
