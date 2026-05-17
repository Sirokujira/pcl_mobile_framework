#include "pcl_mobile_features.h"

#include <algorithm>
#include <cmath>

#include <pcl/features/boundary.h>
#include <pcl/features/crh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/don.h>
#include <pcl/features/esf.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/gasd.h>
#include <pcl/features/grsd.h>
#include <pcl/features/moment_invariants.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/rsd.h>
#include <pcl/features/shot.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/vfh.h>
#include <pcl/common/centroid.h>
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

template <typename DescriptorT, std::size_t Size>
std::vector<jfloat> packHistogramDescriptors(const pcl::PointCloud<DescriptorT>& descriptors)
{
    std::vector<jfloat> values;
    values.reserve(descriptors.points.size() * Size);
    for (const auto& descriptor : descriptors.points) {
        for (float value : descriptor.histogram) {
            values.push_back(value);
        }
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

std::vector<jfloat> computePFHFeatures(int normal_k_search, double feature_radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || normal_k_search <= 0 || feature_radius <= 0.0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search, 0.0);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>);

    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(input);
    pfh.setInputNormals(normals);
    pfh.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    pfh.setRadiusSearch(feature_radius);
    pfh.compute(*descriptors);

    std::vector<jfloat> values;
    values.reserve(descriptors->points.size() * 125);
    for (const auto& descriptor : descriptors->points) {
        for (float value : descriptor.histogram) {
            values.push_back(value);
        }
    }
    LOGI("PFHEstimation computed descriptors: input=%zu descriptors=%zu normal_k=%d radius=%.3f",
         input->points.size(), descriptors->points.size(), normal_k_search, feature_radius);
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

std::vector<jfloat> computeVFHFeatures(int normal_k_search)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || normal_k_search <= 0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search, 0.0);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);

    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud(input);
    vfh.setInputNormals(normals);
    vfh.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    vfh.compute(*descriptors);

    std::vector<jfloat> values;
    values.reserve(descriptors->points.size() * 308);
    for (const auto& descriptor : descriptors->points) {
        for (float value : descriptor.histogram) {
            values.push_back(value);
        }
    }
    LOGI("VFHEstimation computed descriptors: input=%zu descriptors=%zu normal_k=%d",
         input->points.size(), descriptors->points.size(), normal_k_search);
    return values;
}

std::vector<jfloat> computeESFDescriptor()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptors(new pcl::PointCloud<pcl::ESFSignature640>);
    pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
    esf.setInputCloud(input);
    esf.compute(*descriptors);

    std::vector<jfloat> values;
    values.reserve(descriptors->points.size() * 640);
    for (const auto& descriptor : descriptors->points) {
        for (float value : descriptor.histogram) {
            values.push_back(value);
        }
    }
    LOGI("ESFEstimation computed descriptors: input=%zu descriptors=%zu",
         input->points.size(), descriptors->points.size());
    return values;
}

std::vector<jfloat> computeGASDDescriptor()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    pcl::PointCloud<pcl::GASDSignature512>::Ptr descriptors(new pcl::PointCloud<pcl::GASDSignature512>);
    pcl::GASDEstimation<pcl::PointXYZ, pcl::GASDSignature512> gasd;
    gasd.setInputCloud(input);
    gasd.compute(*descriptors);

    std::vector<jfloat> values;
    values.reserve(descriptors->points.size() * 512);
    for (const auto& descriptor : descriptors->points) {
        for (float value : descriptor.histogram) {
            values.push_back(value);
        }
    }
    LOGI("GASDEstimation computed descriptors: input=%zu descriptors=%zu",
         input->points.size(), descriptors->points.size());
    return values;
}

std::vector<jfloat> computeCRHDescriptor(int normal_k_search, float viewpoint_x, float viewpoint_y, float viewpoint_z)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || normal_k_search <= 0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search, 0.0);
    pcl::PointCloud<pcl::Histogram<90>>::Ptr descriptors(new pcl::PointCloud<pcl::Histogram<90>>);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input, centroid);

    pcl::CRHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<90>> crh;
    crh.setInputCloud(input);
    crh.setInputNormals(normals);
    crh.setCentroid(centroid);
    crh.setViewPoint(viewpoint_x, viewpoint_y, viewpoint_z);
    crh.compute(*descriptors);

    std::vector<jfloat> values = packHistogramDescriptors<pcl::Histogram<90>, 90>(*descriptors);
    LOGI("CRHEstimation computed descriptors: input=%zu descriptors=%zu normal_k=%d viewpoint=(%.3f, %.3f, %.3f)",
         input->points.size(), descriptors->points.size(), normal_k_search, viewpoint_x, viewpoint_y, viewpoint_z);
    return values;
}

std::vector<jfloat> computeCVFHFeatures(
        int normal_k_search,
        double cluster_tolerance,
        double eps_angle_threshold,
        double curvature_threshold,
        int min_points,
        bool normalize_bins)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || normal_k_search <= 0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search, 0.0);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);

    pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setInputCloud(input);
    cvfh.setInputNormals(normals);
    if (cluster_tolerance > 0.0) {
        cvfh.setClusterTolerance(static_cast<float>(cluster_tolerance));
    }
    if (eps_angle_threshold > 0.0) {
        cvfh.setEPSAngleThreshold(static_cast<float>(eps_angle_threshold));
    }
    if (curvature_threshold >= 0.0) {
        cvfh.setCurvatureThreshold(static_cast<float>(curvature_threshold));
    }
    if (min_points > 0) {
        cvfh.setMinPoints(static_cast<std::size_t>(min_points));
    }
    cvfh.setNormalizeBins(normalize_bins);
    cvfh.compute(*descriptors);

    std::vector<jfloat> values = packHistogramDescriptors<pcl::VFHSignature308, 308>(*descriptors);
    LOGI("CVFHEstimation computed descriptors: input=%zu descriptors=%zu normal_k=%d tolerance=%.3f minPoints=%d",
         input->points.size(), descriptors->points.size(), normal_k_search, cluster_tolerance, min_points);
    return values;
}

std::vector<jfloat> computeOURCVFHFeatures(
        int normal_k_search,
        double cluster_tolerance,
        double eps_angle_threshold,
        double curvature_threshold,
        int min_points,
        bool normalize_bins,
        double refine_clusters,
        double axis_ratio,
        double min_axis_value)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || normal_k_search <= 0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search, 0.0);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);

    pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> our_cvfh;
    our_cvfh.setInputCloud(input);
    our_cvfh.setInputNormals(normals);
    if (cluster_tolerance > 0.0) {
        our_cvfh.setClusterTolerance(static_cast<float>(cluster_tolerance));
    }
    if (eps_angle_threshold > 0.0) {
        our_cvfh.setEPSAngleThreshold(static_cast<float>(eps_angle_threshold));
    }
    if (curvature_threshold >= 0.0) {
        our_cvfh.setCurvatureThreshold(static_cast<float>(curvature_threshold));
    }
    if (min_points > 0) {
        our_cvfh.setMinPoints(static_cast<std::size_t>(min_points));
    }
    our_cvfh.setNormalizeBins(normalize_bins);
    if (refine_clusters > 0.0) {
        our_cvfh.setRefineClusters(static_cast<float>(refine_clusters));
    }
    if (axis_ratio > 0.0) {
        our_cvfh.setAxisRatio(static_cast<float>(axis_ratio));
    }
    if (min_axis_value > 0.0) {
        our_cvfh.setMinAxisValue(static_cast<float>(min_axis_value));
    }
    our_cvfh.compute(*descriptors);

    std::vector<jfloat> values = packHistogramDescriptors<pcl::VFHSignature308, 308>(*descriptors);
    LOGI("OURCVFHEstimation computed descriptors: input=%zu descriptors=%zu normal_k=%d tolerance=%.3f minPoints=%d",
         input->points.size(), descriptors->points.size(), normal_k_search, cluster_tolerance, min_points);
    return values;
}

std::vector<jfloat> computeSpinImageFeatures(
        int normal_k_search,
        double feature_radius,
        int image_width,
        double support_angle_cos,
        int min_point_count)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || normal_k_search <= 0 || feature_radius <= 0.0 || image_width <= 0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search, 0.0);
    pcl::PointCloud<pcl::Histogram<153>>::Ptr descriptors(new pcl::PointCloud<pcl::Histogram<153>>);

    pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153>> spin_image(
            static_cast<unsigned int>(image_width),
            support_angle_cos,
            static_cast<unsigned int>(std::max(min_point_count, 0)));
    spin_image.setInputCloud(input);
    spin_image.setInputNormals(normals);
    spin_image.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    spin_image.setRadiusSearch(feature_radius);
    spin_image.compute(*descriptors);

    std::vector<jfloat> values = packHistogramDescriptors<pcl::Histogram<153>, 153>(*descriptors);
    LOGI("SpinImageEstimation computed descriptors: input=%zu descriptors=%zu normal_k=%d radius=%.3f width=%d",
         input->points.size(), descriptors->points.size(), normal_k_search, feature_radius, image_width);
    return values;
}

std::vector<jfloat> computeGRSDDescriptor(
        int normal_k_search,
        double radius_search,
        double plane_radius,
        int subdivisions)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()
            || normal_k_search <= 0
            || radius_search <= 0.0
            || plane_radius <= 0.0
            || subdivisions <= 0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search, 0.0);
    pcl::PointCloud<pcl::GRSDSignature21>::Ptr descriptors(new pcl::PointCloud<pcl::GRSDSignature21>);

    pcl::GRSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::GRSDSignature21> grsd;
    grsd.setInputCloud(input);
    grsd.setInputNormals(normals);
    grsd.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    grsd.setRadiusSearch(radius_search);
    grsd.setPlaneRadius(plane_radius);
    grsd.setNrSubdivisions(subdivisions);
    grsd.compute(*descriptors);

    std::vector<jfloat> values = packHistogramDescriptors<pcl::GRSDSignature21, 21>(*descriptors);
    LOGI("GRSDEstimation computed descriptors: input=%zu descriptors=%zu normal_k=%d radius=%.3f",
         input->points.size(), descriptors->points.size(), normal_k_search, radius_search);
    return values;
}

std::vector<jfloat> computeMomentInvariants(double radius_search)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || radius_search <= 0.0) {
        return {};
    }

    pcl::PointCloud<pcl::MomentInvariants>::Ptr descriptors(new pcl::PointCloud<pcl::MomentInvariants>);
    pcl::MomentInvariantsEstimation<pcl::PointXYZ, pcl::MomentInvariants> estimation;
    estimation.setInputCloud(input);
    estimation.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    estimation.setRadiusSearch(radius_search);
    estimation.compute(*descriptors);

    std::vector<jfloat> values;
    values.reserve(descriptors->points.size() * 3);
    for (const auto& descriptor : descriptors->points) {
        values.push_back(descriptor.j1);
        values.push_back(descriptor.j2);
        values.push_back(descriptor.j3);
    }
    LOGI("MomentInvariantsEstimation computed descriptors: input=%zu descriptors=%zu radius=%.3f",
         input->points.size(), descriptors->points.size(), radius_search);
    return values;
}

std::vector<jfloat> computeRSDFeatures(
        int normal_k_search,
        double radius_search,
        double plane_radius,
        int subdivisions)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()
            || normal_k_search <= 0
            || radius_search <= 0.0
            || plane_radius <= 0.0
            || subdivisions <= 0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search, 0.0);
    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors(new pcl::PointCloud<pcl::PrincipalRadiiRSD>);

    pcl::RSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
    rsd.setInputCloud(input);
    rsd.setInputNormals(normals);
    rsd.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    rsd.setRadiusSearch(radius_search);
    rsd.setPlaneRadius(plane_radius);
    rsd.setNrSubdivisions(subdivisions);
    rsd.compute(*descriptors);

    std::vector<jfloat> values;
    values.reserve(descriptors->points.size() * 2);
    for (const auto& descriptor : descriptors->points) {
        values.push_back(descriptor.r_min);
        values.push_back(descriptor.r_max);
    }
    LOGI("RSDEstimation computed descriptors: input=%zu descriptors=%zu normal_k=%d radius=%.3f",
         input->points.size(), descriptors->points.size(), normal_k_search, radius_search);
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

std::vector<jfloat> computeSHOTFeatures(int normal_k_search, double feature_radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || normal_k_search <= 0 || feature_radius <= 0.0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search, 0.0);
    pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>);

    pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
    shot.setInputCloud(input);
    shot.setInputNormals(normals);
    shot.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    shot.setRadiusSearch(feature_radius);
    shot.compute(*descriptors);

    std::vector<jfloat> values;
    values.reserve(descriptors->points.size() * 352);
    for (const auto& descriptor : descriptors->points) {
        for (float value : descriptor.descriptor) {
            values.push_back(value);
        }
    }
    LOGI("SHOTEstimation computed descriptors: input=%zu descriptors=%zu normal_k=%d radius=%.3f",
         input->points.size(), descriptors->points.size(), normal_k_search, feature_radius);
    return values;
}

std::vector<jfloat> computeBoundaryPoints(
        int normal_k_search,
        double radius_search,
        double angle_threshold_degrees)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || normal_k_search <= 0 || radius_search <= 0.0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search, 0.0);
    pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);

    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> estimation;
    estimation.setInputCloud(input);
    estimation.setInputNormals(normals);
    estimation.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    estimation.setRadiusSearch(radius_search);
    if (angle_threshold_degrees > 0.0) {
        estimation.setAngleThreshold(static_cast<float>(angle_threshold_degrees * M_PI / 180.0));
    }
    estimation.compute(*boundaries);

    std::vector<jfloat> values;
    values.reserve(boundaries->points.size() * 4);
    const std::size_t count = std::min(input->points.size(), boundaries->points.size());
    for (std::size_t i = 0; i < count; i++) {
        values.push_back(input->points[i].x);
        values.push_back(input->points[i].y);
        values.push_back(input->points[i].z);
        values.push_back(static_cast<jfloat>(boundaries->points[i].boundary_point));
    }
    LOGI("BoundaryEstimation computed boundaries: input=%zu boundaries=%zu radius=%.3f",
         input->points.size(), boundaries->points.size(), radius_search);
    return values;
}

std::vector<jfloat> computeDifferenceOfNormals(double small_radius, double large_radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || small_radius <= 0.0 || large_radius <= small_radius) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr small_normals = computeNormals(input, 0, small_radius);
    pcl::PointCloud<pcl::Normal>::Ptr large_normals = computeNormals(input, 0, large_radius);
    pcl::PointCloud<pcl::Normal>::Ptr don_normals(new pcl::PointCloud<pcl::Normal>);

    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::Normal, pcl::Normal> don;
    don.setInputCloud(input);
    don.setNormalScaleSmall(small_normals);
    don.setNormalScaleLarge(large_normals);
    don.computeFeature(*don_normals);

    std::vector<jfloat> values = packNormals(*don_normals);
    LOGI("DifferenceOfNormalsEstimation computed normals: input=%zu output=%zu small=%.3f large=%.3f",
         input->points.size(), don_normals->points.size(), small_radius, large_radius);
    return values;
}

} // namespace pclmobile
