// PCLMSegmentation.mm
//
// PCLMPlaneModel / PCLMSphereModel implementations and RANSAC + Euclidean-
// cluster-extraction methods on PCLMPointCloud.
// Ported from pcl_mobile_segmentation.cpp.

#import "PCLMPointCloud_Internal.h"

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

// ---------------------------------------------------------------------------
// PCLMPlaneModel
// ---------------------------------------------------------------------------

@interface PCLMPlaneModel ()
- (instancetype)initWithA:(float)a b:(float)b c:(float)c d:(float)d
              inlierCount:(NSUInteger)inlierCount
              inputCount:(NSUInteger)inputCount NS_DESIGNATED_INITIALIZER;
@end

@implementation PCLMPlaneModel {
    float _a, _b, _c, _d;
    NSUInteger _inlierCount, _inputCount;
}

- (instancetype)initWithA:(float)a b:(float)b c:(float)c d:(float)d
              inlierCount:(NSUInteger)inlierCount inputCount:(NSUInteger)inputCount
{
    self = [super init];
    if (self) {
        _a = a; _b = b; _c = c; _d = d;
        _inlierCount = inlierCount;
        _inputCount  = inputCount;
    }
    return self;
}

- (float)a      { return _a; }
- (float)b      { return _b; }
- (float)c      { return _c; }
- (float)d      { return _d; }
- (NSUInteger)inlierCount { return _inlierCount; }
- (NSUInteger)inputCount  { return _inputCount; }

@end

// ---------------------------------------------------------------------------
// PCLMSphereModel
// ---------------------------------------------------------------------------

@interface PCLMSphereModel ()
- (instancetype)initWithCenterX:(float)cx centerY:(float)cy centerZ:(float)cz
                          radius:(float)r
                     inlierCount:(NSUInteger)inlierCount
                      inputCount:(NSUInteger)inputCount NS_DESIGNATED_INITIALIZER;
@end

@implementation PCLMSphereModel {
    float _centerX, _centerY, _centerZ, _radius;
    NSUInteger _inlierCount, _inputCount;
}

- (instancetype)initWithCenterX:(float)cx centerY:(float)cy centerZ:(float)cz
                          radius:(float)r
                     inlierCount:(NSUInteger)inlierCount inputCount:(NSUInteger)inputCount
{
    self = [super init];
    if (self) {
        _centerX = cx; _centerY = cy; _centerZ = cz; _radius = r;
        _inlierCount = inlierCount;
        _inputCount  = inputCount;
    }
    return self;
}

- (float)centerX { return _centerX; }
- (float)centerY { return _centerY; }
- (float)centerZ { return _centerZ; }
- (float)radius  { return _radius; }
- (NSUInteger)inlierCount { return _inlierCount; }
- (NSUInteger)inputCount  { return _inputCount; }

@end

// ---------------------------------------------------------------------------
// PCLMCylinderModel
// ---------------------------------------------------------------------------

@interface PCLMCylinderModel ()
- (instancetype)initWithPointX:(float)px pointY:(float)py pointZ:(float)pz
                         axisX:(float)ax axisY:(float)ay axisZ:(float)az
                         radius:(float)r
                    inlierCount:(NSUInteger)inlierCount
                     inputCount:(NSUInteger)inputCount NS_DESIGNATED_INITIALIZER;
@end

@implementation PCLMCylinderModel {
    float _pointX, _pointY, _pointZ;
    float _axisX, _axisY, _axisZ;
    float _radius;
    NSUInteger _inlierCount, _inputCount;
}

- (instancetype)initWithPointX:(float)px pointY:(float)py pointZ:(float)pz
                         axisX:(float)ax axisY:(float)ay axisZ:(float)az
                         radius:(float)r
                    inlierCount:(NSUInteger)inlierCount inputCount:(NSUInteger)inputCount
{
    self = [super init];
    if (self) {
        _pointX = px; _pointY = py; _pointZ = pz;
        _axisX = ax;  _axisY = ay;  _axisZ = az;
        _radius = r;
        _inlierCount = inlierCount;
        _inputCount  = inputCount;
    }
    return self;
}

- (float)pointX { return _pointX; }
- (float)pointY { return _pointY; }
- (float)pointZ { return _pointZ; }
- (float)axisX  { return _axisX; }
- (float)axisY  { return _axisY; }
- (float)axisZ  { return _axisZ; }
- (float)radius { return _radius; }
- (NSUInteger)inlierCount { return _inlierCount; }
- (NSUInteger)inputCount  { return _inputCount; }

@end

// ---------------------------------------------------------------------------
// PCLMPointCloud (Segmentation)
// ---------------------------------------------------------------------------

@implementation PCLMPointCloud (Segmentation)

- (nullable PCLMPlaneModel *)segmentPlaneWithDistanceThreshold:(double)distanceThreshold
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
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    if (!PCLMobileSACSegmentation(input, pcl::SACMODEL_PLANE, distanceThreshold,
                            static_cast<int>(maxIterations), coeff, inliers)
        || coeff->values.size() < 4)
    {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidPCDFormat,
                                               @"RANSAC plane fit failed — no inliers found");
        return nil;
    }

    return [[PCLMPlaneModel alloc]
            initWithA:coeff->values[0]
                    b:coeff->values[1]
                    c:coeff->values[2]
                    d:coeff->values[3]
          inlierCount:inliers->indices.size()
           inputCount:input->size()];
}

- (nullable PCLMSphereModel *)segmentSphereWithDistanceThreshold:(double)distanceThreshold
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
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    if (!PCLMobileSACSegmentation(input, pcl::SACMODEL_SPHERE, distanceThreshold,
                            static_cast<int>(maxIterations), coeff, inliers)
        || coeff->values.size() < 4)
    {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidPCDFormat,
                                               @"RANSAC sphere fit failed — no inliers found");
        return nil;
    }

    return [[PCLMSphereModel alloc]
            initWithCenterX:coeff->values[0]
                    centerY:coeff->values[1]
                    centerZ:coeff->values[2]
                      radius:coeff->values[3]
               inlierCount:inliers->indices.size()
                inputCount:input->size()];
}

- (nullable NSArray<NSNumber *> *)extractEuclideanClustersWithTolerance:(double)tolerance
                                                          minClusterSize:(NSInteger)minClusterSize
                                                          maxClusterSize:(NSInteger)maxClusterSize
                                                                   error:(NSError **)error
{
    if (tolerance <= 0.0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"tolerance must be > 0");
        return nil;
    }
    if (minClusterSize <= 0 || maxClusterSize < minClusterSize) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"minClusterSize must be > 0 and <= maxClusterSize");
        return nil;
    }

    CloudPtr input = self.cloud;
    if (input->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    ece.setClusterTolerance(tolerance);
    ece.setMinClusterSize(static_cast<int>(minClusterSize));
    ece.setMaxClusterSize(static_cast<int>(maxClusterSize));
    ece.setSearchMethod(tree);
    ece.setInputCloud(input);
    ece.extract(clusterIndices);

    NSMutableArray<NSNumber *> *result =
        [NSMutableArray arrayWithCapacity:clusterIndices.size()];
    for (const auto &ci : clusterIndices) {
        [result addObject:@(ci.indices.size())];
    }
    return result;
}

- (nullable NSArray<PCLMPointCloud *> *)extractEuclideanClusterCloudsWithTolerance:(double)tolerance
                                                                    minClusterSize:(NSInteger)minClusterSize
                                                                    maxClusterSize:(NSInteger)maxClusterSize
                                                                             error:(NSError **)error
{
    if (tolerance <= 0.0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"tolerance must be > 0");
        return nil;
    }
    if (minClusterSize <= 0 || maxClusterSize < minClusterSize) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"minClusterSize must be > 0 and <= maxClusterSize");
        return nil;
    }

    CloudPtr input = self.cloud;
    if (input->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    ece.setClusterTolerance(tolerance);
    ece.setMinClusterSize(static_cast<int>(minClusterSize));
    ece.setMaxClusterSize(static_cast<int>(maxClusterSize));
    ece.setSearchMethod(tree);
    ece.setInputCloud(input);
    ece.extract(clusterIndices);

    NSMutableArray<PCLMPointCloud *> *result =
        [NSMutableArray arrayWithCapacity:clusterIndices.size()];
    for (const auto &ci : clusterIndices) {
        CloudPtr clusterCloud(new CloudT);
        clusterCloud->reserve(ci.indices.size());
        for (int idx : ci.indices) {
            clusterCloud->push_back((*input)[idx]);
        }
        [result addObject:[[PCLMPointCloud alloc] initWithCloud:std::move(clusterCloud)]];
    }
    return result;
}

- (nullable PCLMCylinderModel *)segmentCylinderWithNormalData:(NSData *)normalData
                                            distanceThreshold:(double)distanceThreshold
                                        normalDistanceWeight:(double)normalDistanceWeight
                                               maxIterations:(NSInteger)maxIterations
                                                       error:(NSError **)error
{
    if (distanceThreshold <= 0.0 || maxIterations <= 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"distanceThreshold and maxIterations must be > 0");
        return nil;
    }

    CloudPtr input = self.cloud;
    if (input->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    const NSUInteger expectedBytes = input->size() * 4 * sizeof(float);
    if (normalData.length < expectedBytes) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"normalData size does not match point count");
        return nil;
    }

    // Reconstruct PointNormal cloud from packed buffer
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_normals->resize(input->size());
    const float *buf = reinterpret_cast<const float *>(normalData.bytes);
    for (size_t i = 0; i < input->size(); ++i) {
        (*cloud_normals)[i].x         = (*input)[i].x;
        (*cloud_normals)[i].y         = (*input)[i].y;
        (*cloud_normals)[i].z         = (*input)[i].z;
        (*cloud_normals)[i].normal_x  = buf[i * 4 + 0];
        (*cloud_normals)[i].normal_y  = buf[i * 4 + 1];
        (*cloud_normals)[i].normal_z  = buf[i * 4 + 2];
        (*cloud_normals)[i].curvature = buf[i * 4 + 3];
    }

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::PointNormal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(normalDistanceWeight);
    seg.setMaxIterations(static_cast<int>(maxIterations));
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud_normals);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.empty() || coeff->values.size() < 7) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidPCDFormat,
                                               @"RANSAC cylinder fit failed — no inliers found");
        return nil;
    }

    // Cylinder model: [px, py, pz, ax, ay, az, radius]
    return [[PCLMCylinderModel alloc]
            initWithPointX:coeff->values[0]
                    pointY:coeff->values[1]
                    pointZ:coeff->values[2]
                     axisX:coeff->values[3]
                     axisY:coeff->values[4]
                     axisZ:coeff->values[5]
                    radius:coeff->values[6]
               inlierCount:inliers->indices.size()
                inputCount:input->size()];
}

- (nullable NSArray<PCLMPointCloud *> *)regionGrowingClustersWithNormalData:(NSData *)normalData
                                                            minClusterSize:(NSInteger)minClusterSize
                                                            maxClusterSize:(NSInteger)maxClusterSize
                                                       numberOfNeighbours:(NSInteger)numberOfNeighbours
                                                  smoothnessThresholdDeg:(double)smoothnessThresholdDeg
                                                      curvatureThreshold:(double)curvatureThreshold
                                                                    error:(NSError **)error
{
    CloudPtr input = self.cloud;
    if (input->empty()) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"cloud is empty");
        return nil;
    }

    const NSUInteger expectedBytes = input->size() * 4 * sizeof(float);
    if (normalData.length < expectedBytes) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                               @"normalData size does not match point count");
        return nil;
    }

    // Build PointNormal cloud
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals->resize(input->size());
    const float *buf = reinterpret_cast<const float *>(normalData.bytes);
    for (size_t i = 0; i < input->size(); ++i) {
        (*normals)[i].normal_x  = buf[i * 4 + 0];
        (*normals)[i].normal_y  = buf[i * 4 + 1];
        (*normals)[i].normal_z  = buf[i * 4 + 2];
        (*normals)[i].curvature = buf[i * 4 + 3];
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
    rg.setMinClusterSize(static_cast<int>(minClusterSize));
    rg.setMaxClusterSize(static_cast<int>(maxClusterSize));
    rg.setSearchMethod(tree);
    rg.setNumberOfNeighbours(static_cast<int>(numberOfNeighbours));
    rg.setInputCloud(input);
    rg.setInputNormals(normals);
    rg.setSmoothnessThreshold(static_cast<float>(smoothnessThresholdDeg * M_PI / 180.0));
    rg.setCurvatureThreshold(static_cast<float>(curvatureThreshold));

    std::vector<pcl::PointIndices> clusterIndices;
    rg.extract(clusterIndices);

    NSMutableArray<PCLMPointCloud *> *result =
        [NSMutableArray arrayWithCapacity:clusterIndices.size()];
    for (const auto &ci : clusterIndices) {
        CloudPtr clusterCloud(new CloudT);
        clusterCloud->reserve(ci.indices.size());
        for (int idx : ci.indices) {
            clusterCloud->push_back((*input)[idx]);
        }
        [result addObject:[[PCLMPointCloud alloc] initWithCloud:std::move(clusterCloud)]];
    }
    return result;
}

@end
