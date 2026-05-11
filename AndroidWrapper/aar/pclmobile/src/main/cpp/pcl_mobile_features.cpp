#include "pcl_mobile_features.h"

#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/search/kdtree.h>

#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"

namespace pclmobile {

namespace {

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        int k_search,
        double radius_search)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    if (input->empty() || (k_search <= 0 && radius_search <= 0.0)) {
        return normals;
    }

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(input);
    normal_estimation.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    if (radius_search > 0.0) {
        normal_estimation.setRadiusSearch(radius_search);
    } else {
        normal_estimation.setKSearch(k_search);
    }
    normal_estimation.compute(*normals);
    return normals;
}

std::vector<jfloat> packNormals(const pcl::PointCloud<pcl::Normal>& normals)
{
    std::vector<jfloat> values;
    values.reserve(normals.points.size() * 4);
    for (const auto& normal : normals.points) {
        values.push_back(normal.normal_x);
        values.push_back(normal.normal_y);
        values.push_back(normal.normal_z);
        values.push_back(normal.curvature);
    }
    return values;
}

} // namespace

std::vector<jfloat> estimateNormals(int k_search)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, k_search, 0.0);
    std::vector<jfloat> values = packNormals(*normals);
    LOGI("NormalEstimation computed normals: input=%zu normals=%zu k=%d",
         input->points.size(), normals->points.size(), k_search);
    return values;
}

std::vector<jfloat> estimateNormalsRadius(double radius_search)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, 0, radius_search);
    std::vector<jfloat> values = packNormals(*normals);
    LOGI("NormalEstimation computed normals: input=%zu normals=%zu radius=%.3f",
         input->points.size(), normals->points.size(), radius_search);
    return values;
}

std::vector<jfloat> computeFPFHFeatures(int normal_k_search, double feature_radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || normal_k_search <= 0 || feature_radius <= 0.0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search, 0.0);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>);

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(input);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    fpfh.setRadiusSearch(feature_radius);
    fpfh.compute(*descriptors);

    std::vector<jfloat> values;
    values.reserve(descriptors->points.size() * 33);
    for (const auto& descriptor : descriptors->points) {
        for (float value : descriptor.histogram) {
            values.push_back(value);
        }
    }
    LOGI("FPFHEstimation computed descriptors: input=%zu descriptors=%zu normal_k=%d radius=%.3f",
         input->points.size(), descriptors->points.size(), normal_k_search, feature_radius);
    return values;
}

std::vector<jfloat> computePrincipalCurvatures(int normal_k_search, int curvature_k_search)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || normal_k_search <= 0 || curvature_k_search <= 0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search, 0.0);
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);

    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> estimation;
    estimation.setInputCloud(input);
    estimation.setInputNormals(normals);
    estimation.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    estimation.setKSearch(curvature_k_search);
    estimation.compute(*curvatures);

    std::vector<jfloat> values;
    values.reserve(curvatures->points.size() * 5);
    for (const auto& curvature : curvatures->points) {
        values.push_back(curvature.principal_curvature_x);
        values.push_back(curvature.principal_curvature_y);
        values.push_back(curvature.principal_curvature_z);
        values.push_back(curvature.pc1);
        values.push_back(curvature.pc2);
    }
    LOGI("PrincipalCurvaturesEstimation computed curvatures: input=%zu curvatures=%zu normal_k=%d curvature_k=%d",
         input->points.size(), curvatures->points.size(), normal_k_search, curvature_k_search);
    return values;
}

} // namespace pclmobile
