// PCLMGeometry.mm
//
// AABB (boundsAndCentroid) + OBB (orientedBoundingBox) via
// pcl::MomentOfInertiaEstimation.

#import "PCLMPointCloud_Internal.h"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>

// ---------------------------------------------------------------------------
// PCLMBoundsResult
// ---------------------------------------------------------------------------

@interface PCLMBoundsResult ()
- (instancetype)initWithCentroidX:(float)cx centroidY:(float)cy centroidZ:(float)cz
                              minX:(float)minX minY:(float)minY minZ:(float)minZ
                              maxX:(float)maxX maxY:(float)maxY maxZ:(float)maxZ
                        pointCount:(NSUInteger)pointCount NS_DESIGNATED_INITIALIZER;
@end

@implementation PCLMBoundsResult {
    float _centroidX, _centroidY, _centroidZ;
    float _minX, _minY, _minZ;
    float _maxX, _maxY, _maxZ;
    NSUInteger _pointCount;
}

- (instancetype)initWithCentroidX:(float)cx centroidY:(float)cy centroidZ:(float)cz
                              minX:(float)minX minY:(float)minY minZ:(float)minZ
                              maxX:(float)maxX maxY:(float)maxY maxZ:(float)maxZ
                        pointCount:(NSUInteger)pointCount
{
    self = [super init];
    if (self) {
        _centroidX = cx; _centroidY = cy; _centroidZ = cz;
        _minX = minX;    _minY = minY;    _minZ = minZ;
        _maxX = maxX;    _maxY = maxY;    _maxZ = maxZ;
        _pointCount = pointCount;
    }
    return self;
}

- (float)centroidX { return _centroidX; }
- (float)centroidY { return _centroidY; }
- (float)centroidZ { return _centroidZ; }
- (float)minX      { return _minX; }
- (float)minY      { return _minY; }
- (float)minZ      { return _minZ; }
- (float)maxX      { return _maxX; }
- (float)maxY      { return _maxY; }
- (float)maxZ      { return _maxZ; }
- (NSUInteger)pointCount { return _pointCount; }

@end

// ---------------------------------------------------------------------------
// PCLMOBBResult
// ---------------------------------------------------------------------------

@interface PCLMOBBResult ()
- (instancetype)initWithPositionX:(float)px positionY:(float)py positionZ:(float)pz
                           axisXx:(float)axx axisXy:(float)axy axisXz:(float)axz
                           axisYx:(float)ayx axisYy:(float)ayy axisYz:(float)ayz
                           axisZx:(float)azx axisZy:(float)azy axisZz:(float)azz
                            halfX:(float)hx halfY:(float)hy halfZ:(float)hz
    NS_DESIGNATED_INITIALIZER;
@end

@implementation PCLMOBBResult {
    float _positionX, _positionY, _positionZ;
    float _axisXx, _axisXy, _axisXz;
    float _axisYx, _axisYy, _axisYz;
    float _axisZx, _axisZy, _axisZz;
    float _halfX, _halfY, _halfZ;
}

- (instancetype)initWithPositionX:(float)px positionY:(float)py positionZ:(float)pz
                           axisXx:(float)axx axisXy:(float)axy axisXz:(float)axz
                           axisYx:(float)ayx axisYy:(float)ayy axisYz:(float)ayz
                           axisZx:(float)azx axisZy:(float)azy axisZz:(float)azz
                            halfX:(float)hx halfY:(float)hy halfZ:(float)hz
{
    self = [super init];
    if (self) {
        _positionX = px; _positionY = py; _positionZ = pz;
        _axisXx = axx; _axisXy = axy; _axisXz = axz;
        _axisYx = ayx; _axisYy = ayy; _axisYz = ayz;
        _axisZx = azx; _axisZy = azy; _axisZz = azz;
        _halfX = hx; _halfY = hy; _halfZ = hz;
    }
    return self;
}

- (float)positionX { return _positionX; }
- (float)positionY { return _positionY; }
- (float)positionZ { return _positionZ; }
- (float)axisXx    { return _axisXx; }
- (float)axisXy    { return _axisXy; }
- (float)axisXz    { return _axisXz; }
- (float)axisYx    { return _axisYx; }
- (float)axisYy    { return _axisYy; }
- (float)axisYz    { return _axisYz; }
- (float)axisZx    { return _axisZx; }
- (float)axisZy    { return _axisZy; }
- (float)axisZz    { return _axisZz; }
- (float)halfX     { return _halfX; }
- (float)halfY     { return _halfY; }
- (float)halfZ     { return _halfZ; }

@end

// ---------------------------------------------------------------------------
// PCLMPointCloud (Geometry)
// ---------------------------------------------------------------------------

@implementation PCLMPointCloud (Geometry)

- (nullable PCLMBoundsResult *)boundsAndCentroidWithError:(NSError **)error
{
    CloudPtr input = self.cloud;
    if (input->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input, centroid);

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*input, minPt, maxPt);

    return [[PCLMBoundsResult alloc]
            initWithCentroidX:centroid.x()
                    centroidY:centroid.y()
                    centroidZ:centroid.z()
                         minX:minPt.x  minY:minPt.y  minZ:minPt.z
                         maxX:maxPt.x  maxY:maxPt.y  maxZ:maxPt.z
                   pointCount:static_cast<NSUInteger>(input->size())];
}

- (nullable PCLMOBBResult *)orientedBoundingBoxWithError:(NSError **)error
{
    CloudPtr input = self.cloud;
    if (input->size() < 4) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud needs at least 4 points for OBB");
        return nil;
    }

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moi;
    moi.setInputCloud(input);
    moi.compute();

    pcl::PointXYZ minOBB, maxOBB, posOBB;
    Eigen::Matrix3f rotOBB;
    moi.getOBB(minOBB, maxOBB, posOBB, rotOBB);

    // rotOBB columns are the local X, Y, Z axes
    return [[PCLMOBBResult alloc]
            initWithPositionX:posOBB.x positionY:posOBB.y positionZ:posOBB.z
                       axisXx:rotOBB(0,0) axisXy:rotOBB(1,0) axisXz:rotOBB(2,0)
                       axisYx:rotOBB(0,1) axisYy:rotOBB(1,1) axisYz:rotOBB(2,1)
                       axisZx:rotOBB(0,2) axisZy:rotOBB(1,2) axisZz:rotOBB(2,2)
                        halfX:(maxOBB.x - minOBB.x) * 0.5f
                        halfY:(maxOBB.y - minOBB.y) * 0.5f
                        halfZ:(maxOBB.z - minOBB.z) * 0.5f];
}

@end
