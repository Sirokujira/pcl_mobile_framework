#ifndef PCL_MOBILE_REGISTRATION_H
#define PCL_MOBILE_REGISTRATION_H

#include <jni.h>

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pclmobile {

std::vector<jfloat> estimateRigidTransformSVD(const std::vector<jfloat>& packed_target_xyz);
pcl::PointCloud<pcl::PointXYZ>::Ptr transformActiveCloud(const std::vector<jfloat>& row_major_matrix);
pcl::PointCloud<pcl::PointXYZ>::Ptr translateActiveCloud(float tx, float ty, float tz);
std::vector<jfloat> alignToTargetICP(
        const std::vector<jfloat>& packed_target_xyz,
        int max_iterations,
        double max_correspondence_distance,
        double transformation_epsilon,
        double euclidean_fitness_epsilon);
std::vector<jfloat> alignToTargetGICP(
        const std::vector<jfloat>& packed_target_xyz,
        int max_iterations,
        double max_correspondence_distance,
        double transformation_epsilon,
        double rotation_epsilon,
        int max_optimizer_iterations);
std::vector<jfloat> alignToTranslatedCopyICP(float tx, float ty, float tz, int max_iterations);

} // namespace pclmobile

#endif // PCL_MOBILE_REGISTRATION_H
