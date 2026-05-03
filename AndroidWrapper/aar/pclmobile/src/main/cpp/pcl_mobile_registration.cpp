#include "pcl_mobile_registration.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"

namespace pclmobile {

std::vector<jfloat> alignToTranslatedCopyICP(float tx, float ty, float tz, int max_iterations)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source = activeCloud();
    if (source->empty()) {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << tx, ty, tz;
    pcl::transformPointCloud(*source, *target, transform);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    // Keep each ICP object alive for the process lifetime. Reusing one ICP
    // instance across calls corrupts the correspondence-estimation state on the
    // tested Android build, while destroying it after JNI return trips Scudo in
    // the same cleanup path.
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>* icp =
            new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>();
    icp->setInputSource(source);
    icp->setInputTarget(target);
    icp->setMaximumIterations(max_iterations);
    icp->align(aligned);

    Eigen::Matrix4f matrix = icp->getFinalTransformation();
    std::vector<jfloat> values;
    values.reserve(18);
    values.push_back(icp->hasConverged() ? 1.0f : 0.0f);
    values.push_back(static_cast<jfloat>(icp->getFitnessScore()));
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            values.push_back(matrix(row, col));
        }
    }
    LOGI("IterativeClosestPoint: input=%zu converged=%d fitness=%.6f translation=(%.3f, %.3f, %.3f)",
         source->points.size(), icp->hasConverged() ? 1 : 0, icp->getFitnessScore(), tx, ty, tz);
    return values;
}

} // namespace pclmobile
