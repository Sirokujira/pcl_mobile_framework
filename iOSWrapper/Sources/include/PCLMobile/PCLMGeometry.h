// PCLMGeometry.h
// Geometric summaries: AABB (boundsAndCentroid) + OBB (orientedBoundingBox).

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/**
 * Result of a centroid-and-bounds computation.
 * Min/max define the axis-aligned bounding box (AABB).
 */
NS_SWIFT_NAME(BoundsResult)
@interface PCLMBoundsResult : NSObject

@property (nonatomic, readonly) float centroidX;
@property (nonatomic, readonly) float centroidY;
@property (nonatomic, readonly) float centroidZ;

@property (nonatomic, readonly) float minX;
@property (nonatomic, readonly) float minY;
@property (nonatomic, readonly) float minZ;

@property (nonatomic, readonly) float maxX;
@property (nonatomic, readonly) float maxY;
@property (nonatomic, readonly) float maxZ;

@property (nonatomic, readonly) NSUInteger pointCount;

- (instancetype)init NS_UNAVAILABLE;

@end

/**
 * Result of an oriented-bounding-box (OBB) computation via
 * `pcl::MomentOfInertiaEstimation`.
 *
 * The OBB is defined by a centre position and three orthogonal eigenvectors
 * (the local X/Y/Z axes of the box).  `minOBB` and `maxOBB` are the corners
 * of the box expressed in that local frame (centred at `position`).
 *
 * In Swift, reconstruct the 8 world-space corners with:
 *   let pos   = SIMD3(result.positionX, result.positionY, result.positionZ)
 *   let axisX = SIMD3(result.axisXx, result.axisXy, result.axisXz)
 *   let axisY = SIMD3(result.axisYx, result.axisYy, result.axisYz)
 *   let axisZ = SIMD3(result.axisZx, result.axisZy, result.axisZz)
 *   // then for each (sx,sy,sz) in {±1}³:
 *   //   corner = pos + axisX*result.halfX*sx
 *   //                + axisY*result.halfY*sy
 *   //                + axisZ*result.halfZ*sz
 */
NS_SWIFT_NAME(OBBResult)
@interface PCLMOBBResult : NSObject

/// Centre of the OBB in the cloud's coordinate frame.
@property (nonatomic, readonly) float positionX;
@property (nonatomic, readonly) float positionY;
@property (nonatomic, readonly) float positionZ;

/// Local X axis (first eigenvector, largest moment of inertia direction).
@property (nonatomic, readonly) float axisXx;
@property (nonatomic, readonly) float axisXy;
@property (nonatomic, readonly) float axisXz;

/// Local Y axis (second eigenvector).
@property (nonatomic, readonly) float axisYx;
@property (nonatomic, readonly) float axisYy;
@property (nonatomic, readonly) float axisYz;

/// Local Z axis (third eigenvector).
@property (nonatomic, readonly) float axisZx;
@property (nonatomic, readonly) float axisZy;
@property (nonatomic, readonly) float axisZz;

/// Half-extents of the box along each local axis.
@property (nonatomic, readonly) float halfX;
@property (nonatomic, readonly) float halfY;
@property (nonatomic, readonly) float halfZ;

- (instancetype)init NS_UNAVAILABLE;

@end

NS_ASSUME_NONNULL_END
