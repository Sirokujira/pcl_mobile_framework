#include "pcl_mobile_keypoints.h"

#include <cmath>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/trajkovic_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"

namespace pclmobile {

namespace {

pcl::PointCloud<pcl::PointXYZI>::Ptr makeIntensityCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
    output->points.reserve(input->points.size());
    for (const auto& point : input->points) {
        pcl::PointXYZI converted;
        converted.x = point.x;
        converted.y = point.y;
        converted.z = point.z;
        converted.intensity = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        output->points.push_back(converted);
    }
    if (input->width * input->height == input->points.size()) {
        output->width = input->width;
        output->height = input->height;
    } else {
        output->width = static_cast<std::uint32_t>(output->points.size());
        output->height = 1;
    }
    output->is_dense = input->is_dense;
    return output;
}

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        int k_search)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    if (input->empty() || k_search <= 0) {
        return normals;
    }

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(input);
    normal_estimation.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    normal_estimation.setKSearch(k_search);
    normal_estimation.compute(*normals);
    return normals;
}

} // namespace

pcl::PointCloud<pcl::PointXYZ>::Ptr computeISSKeypoints(
        double salient_radius,
        double non_max_radius,
        double threshold21,
        double threshold32,
        int min_neighbors)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
    if (input->empty() || salient_radius <= 0.0 || non_max_radius <= 0.0) {
        return keypoints;
    }

    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
    iss.setInputCloud(input);
    iss.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    iss.setSalientRadius(salient_radius);
    iss.setNonMaxRadius(non_max_radius);
    iss.setThreshold21(threshold21);
    iss.setThreshold32(threshold32);
    iss.setMinNeighbors(min_neighbors);
    iss.compute(*keypoints);

    LOGI("ISSKeypoint3D computed keypoints: input=%zu keypoints=%zu salient=%.3f nonMax=%.3f",
         input->points.size(), keypoints->points.size(), salient_radius, non_max_radius);
    return keypoints;
}

std::vector<jfloat> computeSIFTKeypoints(
        double min_scale,
        int nr_octaves,
        int nr_scales_per_octave,
        double min_contrast)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || min_scale <= 0.0 || nr_octaves <= 0 || nr_scales_per_octave <= 0) {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_input = makeIntensityCloud(input);
    pcl::PointCloud<pcl::PointWithScale> keypoints;
    pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointWithScale> sift;
    sift.setInputCloud(intensity_input);
    sift.setSearchMethod(pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>));
    sift.setScales(min_scale, nr_octaves, nr_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.compute(keypoints);

    std::vector<jfloat> values;
    values.reserve(keypoints.points.size() * 4);
    for (const auto& keypoint : keypoints.points) {
        values.push_back(keypoint.x);
        values.push_back(keypoint.y);
        values.push_back(keypoint.z);
        values.push_back(keypoint.scale);
    }
    LOGI("SIFTKeypoint computed keypoints: input=%zu keypoints=%zu minScale=%.3f octaves=%d scales=%d",
         input->points.size(), keypoints.points.size(), min_scale, nr_octaves, nr_scales_per_octave);
    return values;
}

std::vector<jfloat> computeHarrisKeypoints(
        int response_method,
        double radius,
        double threshold,
        bool non_max_suppression,
        bool refine)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || radius <= 0.0) {
        return {};
    }

    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris(
            static_cast<pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::ResponseMethod>(response_method),
            static_cast<float>(radius),
            static_cast<float>(threshold));
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
    harris.setInputCloud(input);
    harris.setNonMaxSupression(non_max_suppression);
    harris.setRefine(refine);
    harris.compute(*keypoints);

    std::vector<jfloat> values;
    values.reserve(keypoints->points.size() * 4);
    for (const auto& keypoint : keypoints->points) {
        values.push_back(keypoint.x);
        values.push_back(keypoint.y);
        values.push_back(keypoint.z);
        values.push_back(keypoint.intensity);
    }
    LOGI("HarrisKeypoint3D computed keypoints: input=%zu keypoints=%zu method=%d radius=%.3f",
         input->points.size(), keypoints->points.size(), response_method, radius);
    return values;
}

std::vector<jfloat> computeSUSANKeypoints(
        double radius,
        double distance_threshold,
        double angular_threshold,
        double intensity_threshold,
        bool non_max_suppression,
        bool geometric_validation)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || radius <= 0.0) {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_input = makeIntensityCloud(input);
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::SUSANKeypoint<pcl::PointXYZI, pcl::PointXYZI> susan(
            static_cast<float>(radius),
            static_cast<float>(distance_threshold),
            static_cast<float>(angular_threshold),
            static_cast<float>(intensity_threshold));
    susan.setInputCloud(intensity_input);
    susan.setNonMaxSupression(non_max_suppression);
    susan.setGeometricValidation(geometric_validation);
    susan.compute(*keypoints);

    std::vector<jfloat> values;
    values.reserve(keypoints->points.size() * 4);
    for (const auto& keypoint : keypoints->points) {
        values.push_back(keypoint.x);
        values.push_back(keypoint.y);
        values.push_back(keypoint.z);
        values.push_back(keypoint.intensity);
    }
    LOGI("SUSANKeypoint computed keypoints: input=%zu keypoints=%zu radius=%.3f nonMax=%d geometric=%d",
         input->points.size(), keypoints->points.size(), radius, non_max_suppression, geometric_validation);
    return values;
}

std::vector<jfloat> computeTrajkovicKeypoints(
        int method,
        int window_size,
        double first_threshold,
        double second_threshold,
        int normal_k_search)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || input->height <= 1 || window_size <= 0 || normal_k_search <= 0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(input, normal_k_search);
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::TrajkovicKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> trajkovic(
            method == 1
                    ? pcl::TrajkovicKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>::EIGHT_CORNERS
                    : pcl::TrajkovicKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>::FOUR_CORNERS,
            window_size,
            static_cast<float>(first_threshold),
            static_cast<float>(second_threshold));
    trajkovic.setInputCloud(input);
    trajkovic.setNormals(normals);
    trajkovic.compute(*keypoints);

    std::vector<jfloat> values;
    values.reserve(keypoints->points.size() * 4);
    for (const auto& keypoint : keypoints->points) {
        values.push_back(keypoint.x);
        values.push_back(keypoint.y);
        values.push_back(keypoint.z);
        values.push_back(keypoint.intensity);
    }
    LOGI("TrajkovicKeypoint3D computed keypoints: input=%zu keypoints=%zu method=%d window=%d normalK=%d",
         input->points.size(), keypoints->points.size(), method, window_size, normal_k_search);
    return values;
}

} // namespace pclmobile
