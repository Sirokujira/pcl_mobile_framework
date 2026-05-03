#include "pcl_mobile_surface.h"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include "pcl_mobile_arrays.h"
#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"
#include "pcl_mobile_segmentation.h"

namespace pclmobile {

std::vector<jfloat> computeCentroidAndBounds()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input, centroid);
    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
    pcl::getMinMax3D(*input, min_point, max_point);

    std::vector<jfloat> values;
    values.reserve(10);
    values.push_back(centroid.x());
    values.push_back(centroid.y());
    values.push_back(centroid.z());
    appendPoint(values, min_point);
    appendPoint(values, max_point);
    values.push_back(static_cast<jfloat>(input->points.size()));
    LOGI("computeCentroidAndBounds: input=%zu centroid=(%.3f, %.3f, %.3f)",
         input->points.size(), centroid.x(), centroid.y(), centroid.z());
    return values;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr computeConvexHull()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
    if (input->empty()) {
        return hull;
    }

    pcl::ConvexHull<pcl::PointXYZ> convex_hull;
    convex_hull.setInputCloud(input);
    convex_hull.reconstruct(*hull);
    LOGI("ConvexHull reconstructed points: input=%zu hull=%zu dimension=%d",
         input->points.size(), hull->points.size(), convex_hull.getDimension());
    return hull;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr projectInliersToPlane(double distance_threshold, int max_iterations)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    if (!segmentPlane(distance_threshold, max_iterations, coefficients, inliers)) {
        return projected;
    }

    pcl::ProjectInliers<pcl::PointXYZ> projection;
    projection.setModelType(pcl::SACMODEL_PLANE);
    projection.setInputCloud(input);
    projection.setIndices(inliers);
    projection.setModelCoefficients(coefficients);
    projection.filter(*projected);
    LOGI("ProjectInliers plane: input=%zu projected=%zu", input->points.size(), projected->points.size());
    return projected;
}

} // namespace pclmobile
