#include "pcl_mobile_range_image.h"

#include <cmath>

#include <Eigen/Geometry>
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

} // namespace pclmobile
