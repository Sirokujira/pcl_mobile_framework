#ifndef PCL_MOBILE_IO_H
#define PCL_MOBILE_IO_H

#include <jni.h>

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pclmobile {

std::string jstringToString(JNIEnv* env, jstring value);
void loadPCDFile(const std::string& filename);
void loadPLYFile(const std::string& filename);
void setCloudFromPackedXYZ(const std::vector<jfloat>& packed_xyz);
bool writePCDFileASCII(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source);
bool writePLYFileASCII(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source);

} // namespace pclmobile

#endif // PCL_MOBILE_IO_H
