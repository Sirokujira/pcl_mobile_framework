#ifndef PCL_MOBILE_RANGE_IMAGE_H
#define PCL_MOBILE_RANGE_IMAGE_H

#include <jni.h>

#include <vector>

namespace pclmobile {

std::vector<jfloat> computeRangeImageFromActiveCloud(
        float angular_resolution_degrees,
        float max_angle_width_degrees,
        float max_angle_height_degrees,
        float sensor_x,
        float sensor_y,
        float sensor_z,
        float min_range);
std::vector<jfloat> computeSphericalRangeImageFromActiveCloud(
        float angular_resolution_degrees,
        float max_angle_width_degrees,
        float max_angle_height_degrees,
        float sensor_x,
        float sensor_y,
        float sensor_z,
        float min_range);
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
        float min_range);
std::vector<jfloat> computeRangeImageBorderDescriptionsFromActiveCloud(
        float angular_resolution_degrees,
        float max_angle_width_degrees,
        float max_angle_height_degrees,
        float sensor_x,
        float sensor_y,
        float sensor_z,
        float min_range,
        int max_no_of_threads,
        int pixel_radius_borders);

} // namespace pclmobile

#endif // PCL_MOBILE_RANGE_IMAGE_H
