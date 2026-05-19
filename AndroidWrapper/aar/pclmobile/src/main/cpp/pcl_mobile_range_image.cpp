#include "pcl_mobile_range_image.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/range_image/range_image_spherical.h>

#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"

namespace pclmobile {

namespace {

constexpr float kPi = 3.14159265358979323846f;

float degreesToRadians(float degrees)
{
    return degrees * kPi / 180.0f;
}

std::vector<jfloat> packFiniteRangePoints(const pcl::RangeImage& range_image)
{
    std::vector<jfloat> values;
    values.reserve(range_image.points.size() * 4);
    for (const auto& point : range_image.points) {
        if (!std::isfinite(point.range)) {
            continue;
        }
        values.push_back(point.x);
        values.push_back(point.y);
        values.push_back(point.z);
        values.push_back(point.range);
    }
    return values;
}

std::vector<jfloat> packBorderDescriptions(const pcl::PointCloud<pcl::BorderDescription>& borders)
{
    std::vector<jfloat> values;
    values.reserve(borders.points.size() * 3);
    for (const auto& border : borders.points) {
        if (!border.traits.any()) {
            continue;
        }
        values.push_back(static_cast<jfloat>(border.x));
        values.push_back(static_cast<jfloat>(border.y));
        values.push_back(static_cast<jfloat>(border.traits.to_ulong()));
    }
    return values;
}

std::vector<jfloat> packNarfDescriptors(const pcl::PointCloud<pcl::Narf36>& descriptors)
{
    std::vector<jfloat> values;
    values.reserve(descriptors.points.size() * 42);
    for (const auto& descriptor : descriptors.points) {
        values.push_back(descriptor.x);
        values.push_back(descriptor.y);
        values.push_back(descriptor.z);
        values.push_back(descriptor.roll);
        values.push_back(descriptor.pitch);
        values.push_back(descriptor.yaw);
        for (float value : descriptor.descriptor) {
            values.push_back(value);
        }
    }
    return values;
}

std::vector<jfloat> packNarfKeypointIndices(
        const pcl::RangeImage& range_image,
        const pcl::PointCloud<int>& indices)
{
    std::vector<jfloat> values;
    values.reserve(indices.points.size() * 5);
    for (int index : indices.points) {
        if (index < 0 || static_cast<std::size_t>(index) >= range_image.points.size()) {
            continue;
        }
        const auto& point = range_image.points[static_cast<std::size_t>(index)];
        if (!std::isfinite(point.range)) {
            continue;
        }
        values.push_back(static_cast<jfloat>(index));
        values.push_back(point.x);
        values.push_back(point.y);
        values.push_back(point.z);
        values.push_back(point.range);
    }
    return values;
}

} // namespace

std::vector<jfloat> computeRangeImageFromActiveCloud(
        float angular_resolution_degrees,
        float max_angle_width_degrees,
        float max_angle_height_degrees,
        float sensor_x,
        float sensor_y,
        float sensor_z,
        float min_range)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()
            || angular_resolution_degrees <= 0.0f
            || max_angle_width_degrees <= 0.0f
            || max_angle_height_degrees <= 0.0f) {
        return {};
    }

    Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();
    sensor_pose.translation() = Eigen::Vector3f(sensor_x, sensor_y, sensor_z);

    pcl::RangeImage range_image;
    range_image.createFromPointCloud(
            *input,
            degreesToRadians(angular_resolution_degrees),
            degreesToRadians(max_angle_width_degrees),
            degreesToRadians(max_angle_height_degrees),
            sensor_pose,
            pcl::RangeImage::CAMERA_FRAME,
            0.0f,
            min_range);

    std::vector<jfloat> values = packFiniteRangePoints(range_image);

    LOGI("RangeImage createFromPointCloud: input=%zu width=%u height=%u finite=%zu",
         input->points.size(), range_image.width, range_image.height, values.size() / 4);
    return values;
}

std::vector<jfloat> computeSphericalRangeImageFromActiveCloud(
        float angular_resolution_degrees,
        float max_angle_width_degrees,
        float max_angle_height_degrees,
        float sensor_x,
        float sensor_y,
        float sensor_z,
        float min_range)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()
            || angular_resolution_degrees <= 0.0f
            || max_angle_width_degrees <= 0.0f
            || max_angle_height_degrees <= 0.0f) {
        return {};
    }

    Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();
    sensor_pose.translation() = Eigen::Vector3f(sensor_x, sensor_y, sensor_z);

    pcl::RangeImageSpherical range_image;
    range_image.createFromPointCloud(
            *input,
            degreesToRadians(angular_resolution_degrees),
            degreesToRadians(max_angle_width_degrees),
            degreesToRadians(max_angle_height_degrees),
            sensor_pose,
            pcl::RangeImage::CAMERA_FRAME,
            0.0f,
            min_range);

    std::vector<jfloat> values = packFiniteRangePoints(range_image);
    LOGI("RangeImageSpherical createFromPointCloud: input=%zu width=%u height=%u finite=%zu",
         input->points.size(), range_image.width, range_image.height, values.size() / 4);
    return values;
}

std::vector<jfloat> computePlanarRangeImageFromActiveCloud(
        int image_width,
        int image_height,
        float center_x,
        float center_y,
        float focal_length_x,
        float focal_length_y,
        float sensor_x,
        float sensor_y,
        float sensor_z,
        float min_range)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()
            || image_width <= 0
            || image_height <= 0
            || focal_length_x <= 0.0f
            || focal_length_y <= 0.0f) {
        return {};
    }

    Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();
    sensor_pose.translation() = Eigen::Vector3f(sensor_x, sensor_y, sensor_z);

    pcl::RangeImagePlanar range_image;
    range_image.createFromPointCloudWithFixedSize(
            *input,
            image_width,
            image_height,
            center_x,
            center_y,
            focal_length_x,
            focal_length_y,
            sensor_pose,
            pcl::RangeImage::CAMERA_FRAME,
            0.0f,
            min_range);

    std::vector<jfloat> values = packFiniteRangePoints(range_image);
    LOGI("RangeImagePlanar createFromPointCloudWithFixedSize: input=%zu width=%u height=%u finite=%zu",
         input->points.size(), range_image.width, range_image.height, values.size() / 4);
    return values;
}

std::vector<jfloat> computeRangeImageBorderDescriptionsFromActiveCloud(
        float angular_resolution_degrees,
        float max_angle_width_degrees,
        float max_angle_height_degrees,
        float sensor_x,
        float sensor_y,
        float sensor_z,
        float min_range,
        int max_no_of_threads,
        int pixel_radius_borders)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()
            || angular_resolution_degrees <= 0.0f
            || max_angle_width_degrees <= 0.0f
            || max_angle_height_degrees <= 0.0f) {
        return {};
    }

    Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();
    sensor_pose.translation() = Eigen::Vector3f(sensor_x, sensor_y, sensor_z);

    pcl::RangeImage range_image;
    range_image.createFromPointCloud(
            *input,
            degreesToRadians(angular_resolution_degrees),
            degreesToRadians(max_angle_width_degrees),
            degreesToRadians(max_angle_height_degrees),
            sensor_pose,
            pcl::RangeImage::CAMERA_FRAME,
            0.0f,
            min_range);

    pcl::RangeImageBorderExtractor border_extractor(&range_image);
    border_extractor.getParameters().max_no_of_threads = std::max(max_no_of_threads, 1);
    border_extractor.getParameters().pixel_radius_borders = std::max(pixel_radius_borders, 1);
    pcl::PointCloud<pcl::BorderDescription> borders;
    border_extractor.compute(borders);

    std::vector<jfloat> values = packBorderDescriptions(borders);
    LOGI("RangeImageBorderExtractor computed borders: input=%zu width=%u height=%u borders=%zu",
         input->points.size(), range_image.width, range_image.height, values.size() / 3);
    return values;
}

std::vector<jfloat> computeNARFDescriptorsFromActiveCloud(
        float angular_resolution_degrees,
        float max_angle_width_degrees,
        float max_angle_height_degrees,
        float sensor_x,
        float sensor_y,
        float sensor_z,
        float min_range,
        float support_size,
        bool rotation_invariant,
        int max_descriptor_count)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()
            || angular_resolution_degrees <= 0.0f
            || max_angle_width_degrees <= 0.0f
            || max_angle_height_degrees <= 0.0f
            || support_size <= 0.0f
            || max_descriptor_count <= 0) {
        return {};
    }

    Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();
    sensor_pose.translation() = Eigen::Vector3f(sensor_x, sensor_y, sensor_z);

    pcl::RangeImage range_image;
    range_image.createFromPointCloud(
            *input,
            degreesToRadians(angular_resolution_degrees),
            degreesToRadians(max_angle_width_degrees),
            degreesToRadians(max_angle_height_degrees),
            sensor_pose,
            pcl::RangeImage::CAMERA_FRAME,
            0.0f,
            min_range);

    pcl::Indices indices;
    indices.reserve(std::min<std::size_t>(
            range_image.points.size(),
            static_cast<std::size_t>(max_descriptor_count)));
    for (std::size_t i = 0; i < range_image.points.size()
            && indices.size() < static_cast<std::size_t>(max_descriptor_count); ++i) {
        if (std::isfinite(range_image.points[i].range)) {
            indices.push_back(static_cast<int>(i));
        }
    }

    if (indices.empty()) {
        return {};
    }

    pcl::PointCloud<pcl::Narf36> descriptors;
    pcl::NarfDescriptor narf(&range_image, &indices);
    narf.getParameters().support_size = support_size;
    narf.getParameters().rotation_invariant = rotation_invariant;
    narf.compute(descriptors);

    std::vector<jfloat> values = packNarfDescriptors(descriptors);
    LOGI("NarfDescriptor computed descriptors: input=%zu range=%zu indices=%zu descriptors=%zu support=%.3f",
         input->points.size(), range_image.points.size(), indices.size(), descriptors.points.size(), support_size);
    return values;
}

std::vector<jfloat> computeNARFKeypointsFromActiveCloud(
        float angular_resolution_degrees,
        float max_angle_width_degrees,
        float max_angle_height_degrees,
        float sensor_x,
        float sensor_y,
        float sensor_z,
        float min_range,
        float support_size,
        int max_keypoint_count,
        float min_interest_value,
        bool non_maximum_suppression)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()
            || angular_resolution_degrees <= 0.0f
            || max_angle_width_degrees <= 0.0f
            || max_angle_height_degrees <= 0.0f
            || support_size <= 0.0f) {
        return {};
    }

    Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();
    sensor_pose.translation() = Eigen::Vector3f(sensor_x, sensor_y, sensor_z);

    pcl::RangeImage range_image;
    range_image.createFromPointCloud(
            *input,
            degreesToRadians(angular_resolution_degrees),
            degreesToRadians(max_angle_width_degrees),
            degreesToRadians(max_angle_height_degrees),
            sensor_pose,
            pcl::RangeImage::CAMERA_FRAME,
            0.0f,
            min_range);

    pcl::RangeImageBorderExtractor border_extractor(&range_image);
    pcl::NarfKeypoint keypoint_detector(&border_extractor, support_size);
    keypoint_detector.getParameters().max_no_of_interest_points = max_keypoint_count > 0 ? max_keypoint_count : -1;
    if (min_interest_value > 0.0f) {
        keypoint_detector.getParameters().min_interest_value = min_interest_value;
    }
    keypoint_detector.getParameters().do_non_maximum_suppression = non_maximum_suppression;

    pcl::PointCloud<int> keypoint_indices;
    keypoint_detector.compute(keypoint_indices);

    std::vector<jfloat> values = packNarfKeypointIndices(range_image, keypoint_indices);
    LOGI("NarfKeypoint computed keypoints: input=%zu range=%zu keypoints=%zu support=%.3f",
         input->points.size(), range_image.points.size(), values.size() / 5, support_size);
    return values;
}

} // namespace pclmobile
