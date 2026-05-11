// PCLMPointCloud.h
//
// Public point cloud Objective-C interface. The implementation
// (PCLMPointCloud.mm) is the only file that needs to know about
// pcl::PointCloud<pcl::PointXYZ>; consumers see a plain Cocoa API.

#import <Foundation/Foundation.h>
#import <PCLMobile/PCLMGeometry.h>
#import <PCLMobile/PCLMSegmentation.h>
#import <PCLMobile/PCLMRegistration.h>

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

// -------------------------------------------------------------------------
// MARK: - Outlier removal
// -------------------------------------------------------------------------

/**
 * Remove statistical outliers.
 *
 * For each point, compute the mean distance to its `meanK` nearest neighbours.
 * Points whose mean distance exceeds `stddevMulThresh` standard deviations
 * above the global mean are considered outliers and removed.
 *
 * @param meanK            Number of neighbours used per point (must be > 0).
 * @param stddevMulThresh  Standard-deviation multiplier for the threshold.
 * @param error            Populated on failure.
 */
- (nullable PCLMPointCloud *)statisticalOutlierRemovalWithMeanK:(NSInteger)meanK
                                               stddevMulThresh:(double)stddevMulThresh
                                                         error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(statisticalOutlierRemoval(meanK:stddevMulThresh:));

/**
 * Remove points that have fewer than `minNeighbors` neighbours within `radius`.
 *
 * @param radius       Search radius in metres.
 * @param minNeighbors Minimum required neighbour count (inclusive).
 * @param error        Populated on failure.
 */
- (nullable PCLMPointCloud *)radiusOutlierRemovalWithRadius:(double)radius
                                               minNeighbors:(NSInteger)minNeighbors
                                                      error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(radiusOutlierRemoval(radius:minNeighbors:));

/**
 * Keep only points inside an axis-aligned bounding box.
 *
 * @param minX  Lower x bound.
 * @param minY  Lower y bound.
 * @param minZ  Lower z bound.
 * @param maxX  Upper x bound.
 * @param maxY  Upper y bound.
 * @param maxZ  Upper z bound.
 * @param error Populated on failure.
 */
- (nullable PCLMPointCloud *)cropBoxWithMinX:(double)minX
                                        minY:(double)minY
                                        minZ:(double)minZ
                                        maxX:(double)maxX
                                        maxY:(double)maxY
                                        maxZ:(double)maxZ
                                       error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(cropBox(minX:minY:minZ:maxX:maxY:maxZ:));

/**
 * Extract the inliers of a RANSAC-fitted plane as a new cloud.
 *
 * @param distanceThreshold Maximum point-to-plane distance for inliers.
 * @param maxIterations     RANSAC iteration budget.
 * @param error             Populated on failure.
 */
- (nullable PCLMPointCloud *)extractPlaneInliersWithDistanceThreshold:(double)distanceThreshold
                                                         maxIterations:(NSInteger)maxIterations
                                                                 error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(extractPlaneInliers(distanceThreshold:maxIterations:));

// -------------------------------------------------------------------------
// MARK: - Features
// -------------------------------------------------------------------------

/**
 * Estimate surface normals via k-nearest-neighbour PCA.
 *
 * Returns an `NSData` buffer of `pointCount * 4` `float` values laid out as
 * `nx0, ny0, nz0, curvature0, nx1, ny1, nz1, curvature1, ...`.
 *
 * @param kSearch Number of neighbours per normal estimate (must be > 0).
 * @param error   Populated on failure.
 */
- (nullable NSData *)estimateNormalsWithKSearch:(NSInteger)kSearch
                                          error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(estimateNormals(kSearch:));

// -------------------------------------------------------------------------
// MARK: - Geometry
// -------------------------------------------------------------------------

/**
 * Compute the centroid and axis-aligned bounding box of the cloud.
 *
 * @param error Populated on failure (e.g. empty cloud).
 */
- (nullable PCLMBoundsResult *)boundsAndCentroidWithError:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(boundsAndCentroid());

/**
 * Compute an oriented bounding box (OBB) via moment-of-inertia analysis.
 *
 * The OBB is tighter than the AABB for rotated objects and provides the
 * local eigenvectors (principal axes) of the point distribution.
 *
 * @param error Populated on failure (e.g. empty cloud).
 */
- (nullable PCLMOBBResult *)orientedBoundingBoxWithError:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(orientedBoundingBox());

// -------------------------------------------------------------------------
// MARK: - Spatial search
// -------------------------------------------------------------------------

/**
 * Find the `k` nearest neighbours to a query point using a KD-tree.
 *
 * Returns an `NSData` buffer of `found * 4` `float` values laid out as
 * `x0, y0, z0, sqDist0, x1, y1, z1, sqDist1, ...`.
 *
 * @param x  Query x coordinate.
 * @param y  Query y coordinate.
 * @param z  Query z coordinate.
 * @param k  Number of neighbours to retrieve (must be > 0).
 * @param error Populated on failure.
 */
- (nullable NSData *)nearestKSearchAtX:(float)x
                                     y:(float)y
                                     z:(float)z
                                     k:(NSInteger)k
                                 error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(nearestKSearch(x:y:z:k:));

/**
 * Find all points within `radius` of a query point using an octree.
 *
 * Returns an `NSData` buffer of `found * 4` `float` values laid out as
 * `x0, y0, z0, sqDist0, x1, y1, z1, sqDist1, ...`.
 *
 * @param x          Query x coordinate.
 * @param y          Query y coordinate.
 * @param z          Query z coordinate.
 * @param resolution Octree voxel side length.
 * @param radius     Search radius in metres.
 * @param error      Populated on failure.
 */
- (nullable NSData *)octreeRadiusSearchAtX:(float)x
                                         y:(float)y
                                         z:(float)z
                                resolution:(double)resolution
                                    radius:(double)radius
                                     error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(octreeRadiusSearch(x:y:z:resolution:radius:));

// -------------------------------------------------------------------------
// MARK: - Segmentation
// -------------------------------------------------------------------------

/**
 * Fit a plane model to the cloud using RANSAC.
 *
 * @param distanceThreshold Maximum point-to-plane distance for inliers.
 * @param maxIterations     RANSAC iteration budget.
 * @param error             Populated on failure or when no plane is found.
 */
- (nullable PCLMPlaneModel *)segmentPlaneWithDistanceThreshold:(double)distanceThreshold
                                                  maxIterations:(NSInteger)maxIterations
                                                          error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(segmentPlane(distanceThreshold:maxIterations:));

/**
 * Fit a sphere model to the cloud using RANSAC.
 *
 * @param distanceThreshold Maximum point-to-sphere-surface distance for inliers.
 * @param maxIterations     RANSAC iteration budget.
 * @param error             Populated on failure or when no sphere is found.
 */
- (nullable PCLMSphereModel *)segmentSphereWithDistanceThreshold:(double)distanceThreshold
                                                    maxIterations:(NSInteger)maxIterations
                                                            error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(segmentSphere(distanceThreshold:maxIterations:));

/**
 * Extract Euclidean clusters from the cloud.
 *
 * Returns an `NSArray<NSNumber *>` where each element is the point count of
 * one cluster, sorted by cluster index.
 *
 * @param tolerance      Maximum point-to-point distance within a cluster.
 * @param minClusterSize Minimum cluster point count.
 * @param maxClusterSize Maximum cluster point count.
 * @param error          Populated on failure.
 */
- (nullable NSArray<NSNumber *> *)extractEuclideanClustersWithTolerance:(double)tolerance
                                                          minClusterSize:(NSInteger)minClusterSize
                                                          maxClusterSize:(NSInteger)maxClusterSize
                                                                   error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(extractEuclideanClusters(tolerance:minSize:maxSize:));

/**
 * Extract Euclidean clusters and return each cluster as a separate PointCloud.
 *
 * Unlike `extractEuclideanClusters(tolerance:minSize:maxSize:)` which only
 * returns counts, this method returns the actual point data for each cluster
 * so you can process or render them independently.
 *
 * @param tolerance      Maximum point-to-point distance within a cluster.
 * @param minClusterSize Minimum cluster point count.
 * @param maxClusterSize Maximum cluster point count.
 * @param error          Populated on failure.
 * @return Array of PointCloud objects, one per cluster, largest first.
 */
- (nullable NSArray<PCLMPointCloud *> *)extractEuclideanClusterCloudsWithTolerance:(double)tolerance
                                                                     minClusterSize:(NSInteger)minClusterSize
                                                                     maxClusterSize:(NSInteger)maxClusterSize
                                                                              error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(extractEuclideanClusterClouds(tolerance:minSize:maxSize:));

/**
 * Segment a cylinder from the cloud using RANSAC + surface normals.
 *
 * `normalData` must be the `NSData` returned by `estimateNormals(kSearch:)`.
 * `normalDistanceWeight` (typically 0.1) balances the contribution of normals
 * vs. point distances in the RANSAC objective.
 *
 * @param normalData            Normal buffer from estimateNormals(kSearch:).
 * @param distanceThreshold     Max point-to-cylinder-surface distance (metres).
 * @param normalDistanceWeight  Weight for the normal orientation residual [0,1].
 * @param maxIterations         RANSAC iteration budget.
 * @param error                 Populated on failure.
 */
- (nullable PCLMCylinderModel *)segmentCylinderWithNormalData:(NSData *)normalData
                                            distanceThreshold:(double)distanceThreshold
                                        normalDistanceWeight:(double)normalDistanceWeight
                                               maxIterations:(NSInteger)maxIterations
                                                       error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(segmentCylinder(normalData:distanceThreshold:normalDistanceWeight:maxIterations:));

/**
 * Segment the cloud into smoothness-based region-growing clusters.
 *
 * Regions grow from seed points by adding neighbours whose surface-normal
 * angle differs by less than `smoothnessThresholdDeg` (degrees) and whose
 * curvature is below `curvatureThreshold`.
 *
 * `normalData` must be the `NSData` returned by `estimateNormals(kSearch:)`.
 *
 * @param normalData              Normal buffer from estimateNormals(kSearch:).
 * @param minClusterSize          Discard regions smaller than this.
 * @param maxClusterSize          Cap region size at this value.
 * @param numberOfNeighbours      KNN neighbour count during region growth.
 * @param smoothnessThresholdDeg  Max allowed normal-angle difference (degrees).
 * @param curvatureThreshold      Max allowed curvature for seed selection.
 * @param error                   Populated on failure.
 * @return Array of PointCloud objects, one per region, largest first.
 */
- (nullable NSArray<PCLMPointCloud *> *)regionGrowingClustersWithNormalData:(NSData *)normalData
                                                             minClusterSize:(NSInteger)minClusterSize
                                                             maxClusterSize:(NSInteger)maxClusterSize
                                                        numberOfNeighbours:(NSInteger)numberOfNeighbours
                                                   smoothnessThresholdDeg:(double)smoothnessThresholdDeg
                                                       curvatureThreshold:(double)curvatureThreshold
                                                                     error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(regionGrowingClusters(normalData:minSize:maxSize:neighbours:smoothnessDeg:curvature:));

// -------------------------------------------------------------------------
// MARK: - Surface reconstruction
// -------------------------------------------------------------------------

/**
 * Reconstruct the convex hull of the cloud.
 *
 * @param error Populated on failure.
 */
- (nullable PCLMPointCloud *)convexHullWithError:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(convexHull());

/**
 * Reconstruct the concave hull (alpha shape) of the cloud.
 *
 * A concave hull follows the actual boundary more closely than a convex hull.
 * Smaller `alpha` values produce tighter (more concave) boundaries; larger
 * values approach the convex hull.  A good starting value is 0.1 for LiDAR
 * clouds in metres.
 *
 * @param alpha  Concavity parameter in metres (must be > 0).
 * @param error  Populated on failure.
 */
- (nullable PCLMPointCloud *)concaveHullWithAlpha:(double)alpha
                                            error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(concaveHull(alpha:));

/**
 * Smooth the cloud using Moving Least Squares (MLS) surface reconstruction.
 *
 * MLS projects each point onto a locally-fitted polynomial surface, reducing
 * LiDAR noise while preserving sharp edges better than simple averaging.
 *
 * @param searchRadius  Neighbourhood radius for local polynomial fitting (metres).
 * @param error         Populated on failure.
 */
- (nullable PCLMPointCloud *)movingLeastSquaresSmoothedWithSearchRadius:(double)searchRadius
                                                                   error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(movingLeastSquaresSmoothed(searchRadius:));

/**
 * Project plane inliers onto the fitted plane.
 *
 * @param distanceThreshold Maximum point-to-plane distance for inliers.
 * @param maxIterations     RANSAC iteration budget.
 * @param error             Populated on failure.
 */
- (nullable PCLMPointCloud *)projectInliersToPlaneWithDistanceThreshold:(double)distanceThreshold
                                                           maxIterations:(NSInteger)maxIterations
                                                                   error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(projectInliersToPlane(distanceThreshold:maxIterations:));

// -------------------------------------------------------------------------
// MARK: - Registration
// -------------------------------------------------------------------------

/**
 * Align the cloud to a translated copy of itself using ICP.
 *
 * This is primarily a correctness / smoke test for the ICP pipeline.
 *
 * @param tx            Translation x offset applied to the target copy.
 * @param ty            Translation y offset applied to the target copy.
 * @param tz            Translation z offset applied to the target copy.
 * @param maxIterations ICP iteration budget.
 * @param error         Populated on failure.
 */
- (nullable PCLMICPResult *)alignToTranslatedCopyICPWithTx:(float)tx
                                                        ty:(float)ty
                                                        tz:(float)tz
                                             maxIterations:(NSInteger)maxIterations
                                                     error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(alignToTranslatedCopyICP(tx:ty:tz:maxIterations:));

/**
 * Align this cloud (source) to another cloud (target) using ICP.
 *
 * Use this for real alignment tasks such as stitching two LiDAR scans
 * taken from different poses, or matching a scanned object to a reference.
 *
 * @param targetCloud   The target cloud to align to.
 * @param maxIterations ICP iteration budget.
 * @param error         Populated on failure or non-convergence.
 */
- (nullable PCLMICPResult *)alignToCloud:(PCLMPointCloud *)targetCloud
                           maxIterations:(NSInteger)maxIterations
                                   error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(alignToCloud(_:maxIterations:));

@end

NS_ASSUME_NONNULL_END
