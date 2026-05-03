#include "pcl_mobile_arrays.h"

namespace pclmobile {

void appendPoint(std::vector<jfloat>& values, const pcl::PointXYZ& point)
{
    values.push_back(point.x);
    values.push_back(point.y);
    values.push_back(point.z);
}

jfloatArray makeFloatArray(JNIEnv* env, const std::vector<jfloat>& values)
{
    jfloatArray result = env->NewFloatArray(static_cast<jsize>(values.size()));
    if (result == nullptr || values.empty()) {
        return result;
    }
    env->SetFloatArrayRegion(result, 0, static_cast<jsize>(values.size()), values.data());
    return result;
}

jfloatArray makePointArray(JNIEnv* env, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source)
{
    std::vector<jfloat> values;
    values.reserve(source->points.size() * 3);
    for (const auto& point : source->points) {
        appendPoint(values, point);
    }
    return makeFloatArray(env, values);
}

} // namespace pclmobile
