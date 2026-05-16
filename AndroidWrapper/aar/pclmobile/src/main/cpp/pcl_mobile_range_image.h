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

} // namespace pclmobile

#endif // PCL_MOBILE_RANGE_IMAGE_H
