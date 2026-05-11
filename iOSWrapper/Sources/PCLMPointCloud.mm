// PCLMPointCloud.mm
//
// Objective-C++ implementation of the PCLMPointCloud public interface. The
// .mm extension is what lets us mix the C++ PCL types with Objective-C; the
// public header (PCLMPointCloud.h) hides them.

#import "PCLMPointCloud_Internal.h"

#import <cmath>
#import <cstdint>
#import <memory>
#import <string>

#import <pcl/io/pcd_io.h>
#import <pcl/filters/passthrough.h>
#import <pcl/filters/voxel_grid.h>

@interface PCLMPointCloud () {
    CloudPtr _cloud;
}

- (instancetype)initWithCloud:(CloudPtr)cloud NS_DESIGNATED_INITIALIZER;

@end

@implementation PCLMPointCloud

- (instancetype)initWithCloud:(CloudPtr)cloud {
    self = [super init];
    if (self) {
        _cloud = std::move(cloud);
    }
    return self;
}

- (instancetype)init {
    return [self initWithCloud:CloudPtr(new CloudT)];
}

- (CloudPtr)cloud { return _cloud; }

#pragma mark - Public properties

- (NSUInteger)pointCount {
    return static_cast<NSUInteger>(_cloud ? _cloud->size() : 0);
}

- (NSUInteger)width  { return _cloud ? _cloud->width  : 0; }
- (NSUInteger)height { return _cloud ? _cloud->height : 0; }

#pragma mark - Public API

+ (nullable instancetype)cloudFromPCDFile:(NSString *)path
                                    error:(NSError * _Nullable * _Nullable)error {
    if (path.length == 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                       @"path must not be empty");
        return nil;
    }

    NSFileManager *fm = NSFileManager.defaultManager;
    if (![fm fileExistsAtPath:path]) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeFileNotFound,
                                       [NSString stringWithFormat:@"PCD file not found: %@", path]);
        return nil;
    }

    CloudPtr cloud(new CloudT);
    const std::string utf8Path(path.UTF8String ?: "");
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(utf8Path, *cloud) == -1) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidPCDFormat,
                                       [NSString stringWithFormat:@"failed to parse PCD: %@", path]);
        return nil;
    }
    return [[self alloc] initWithCloud:std::move(cloud)];
}

+ (nullable instancetype)cloudFromPackedXYZ:(const float *)packedXYZ
                                       count:(NSUInteger)count
                                       error:(NSError * _Nullable * _Nullable)error {
    if (packedXYZ == nullptr && count > 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                       @"packedXYZ must not be NULL when count > 0");
        return nil;
    }

    CloudPtr cloud(new CloudT);
    cloud->resize(count);
    cloud->width  = static_cast<std::uint32_t>(count);
    cloud->height = 1;
    cloud->is_dense = true;

    for (NSUInteger i = 0; i < count; ++i) {
        const float x = packedXYZ[i * 3 + 0];
        const float y = packedXYZ[i * 3 + 1];
        const float z = packedXYZ[i * 3 + 2];
        (*cloud)[i].x = x;
        (*cloud)[i].y = y;
        (*cloud)[i].z = z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            cloud->is_dense = false;
        }
    }

    return [[self alloc] initWithCloud:std::move(cloud)];
}

- (BOOL)copyPackedXYZIntoBuffer:(float *)buffer
                       capacity:(NSUInteger)capacity
                    actualCount:(NSUInteger * _Nullable)outCount
                          error:(NSError * _Nullable * _Nullable)error {
    if (buffer == nullptr && capacity > 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                       @"buffer must not be NULL");
        return NO;
    }

    const NSUInteger have = self.pointCount;
    const NSUInteger n = MIN(have, capacity);
    for (NSUInteger i = 0; i < n; ++i) {
        const auto &p = (*_cloud)[i];
        buffer[i * 3 + 0] = p.x;
        buffer[i * 3 + 1] = p.y;
        buffer[i * 3 + 2] = p.z;
    }
    if (outCount) *outCount = n;
    return YES;
}

- (BOOL)writePCDFileAtPath:(NSString *)path
                     error:(NSError * _Nullable * _Nullable)error {
    if (path.length == 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                       @"path must not be empty");
        return NO;
    }
    if (!_cloud) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInternal,
                                       @"underlying cloud is null");
        return NO;
    }

    const std::string utf8Path(path.UTF8String ?: "");
    // ASCII format keeps the file portable / inspectable. Switch to
    // savePCDFileBinary if file size becomes a concern.
    const int rc = pcl::io::savePCDFileASCII<pcl::PointXYZ>(utf8Path, *_cloud);
    if (rc != 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInternal,
                                       [NSString stringWithFormat:
                                        @"savePCDFileASCII failed (rc=%d) for %@",
                                        rc, path]);
        return NO;
    }
    return YES;
}

- (nullable PCLMPointCloud *)voxelGridDownsampleWithLeaf:(double)leafSize
                                                   error:(NSError * _Nullable * _Nullable)error {
    if (leafSize <= 0.0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                       @"leafSize must be positive");
        return nil;
    }

    CloudPtr filtered(new CloudT);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(_cloud);
    voxel.setLeafSize(static_cast<float>(leafSize),
                      static_cast<float>(leafSize),
                      static_cast<float>(leafSize));
    voxel.filter(*filtered);
    return [[PCLMPointCloud alloc] initWithCloud:std::move(filtered)];
}

- (nullable PCLMPointCloud *)passThroughFilteredOnAxis:(NSString *)axis
                                              minValue:(double)minValue
                                              maxValue:(double)maxValue
                                                 error:(NSError * _Nullable * _Nullable)error {
    if (axis.length == 0) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                       @"axis must be 'x', 'y' or 'z'");
        return nil;
    }
    if (minValue >= maxValue) {
        if (error) *error = PCLMobileMakeError(PCLMobileErrorCodeInvalidArgument,
                                       @"minValue must be < maxValue");
        return nil;
    }

    CloudPtr filtered(new CloudT);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(_cloud);
    pass.setFilterFieldName(std::string(axis.UTF8String ?: ""));
    pass.setFilterLimits(static_cast<float>(minValue),
                         static_cast<float>(maxValue));
    pass.filter(*filtered);
    return [[PCLMPointCloud alloc] initWithCloud:std::move(filtered)];
}

@end
