#ifndef PCL_MOBILE_ARRAYS_H
#define PCL_MOBILE_ARRAYS_H

#include <jni.h>

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pclmobile {

void appendPoint(std::vector<jfloat>& values, const pcl::PointXYZ& point);
jfloatArray makeFloatArray(JNIEnv* env, const std::vector<jfloat>& values);
jfloatArray makePointArray(JNIEnv* env, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source);

} // namespace pclmobile

#endif // PCL_MOBILE_ARRAYS_H
