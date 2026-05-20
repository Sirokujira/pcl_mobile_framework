#include "pcl_mobile_surface.h"

#include <cmath>
#include <limits>

#include <pcl/common/centroid.h>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/common/distances.h>
#include <pcl/common/geometry.h>
#include <pcl/common/intersections.h>
#include <pcl/common/norms.h>
#include <pcl/common/pca.h>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/impl/grid_projection.hpp>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/impl/marching_cubes_hoppe.hpp>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/impl/marching_cubes_rbf.hpp>
#include <pcl/surface/mls.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/surface/surfel_smoothing.h>
#include <pcl/surface/impl/surfel_smoothing.hpp>

#include "pcl_mobile_arrays.h"
#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"
#include "pcl_mobile_segmentation.h"

namespace pclmobile {

namespace {

void appendMatrix3(std::vector<jfloat>& values, const Eigen::Matrix3f& matrix)
{
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            values.push_back(matrix(row, col));
        }
    }
}

void appendMatrix4(std::vector<jfloat>& values, const Eigen::Matrix4f& matrix)
{
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            values.push_back(matrix(row, col));
        }
    }
}

Eigen::Affine3f affineFromRowMajorMatrix(const std::vector<jfloat>& row_major_matrix)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    if (row_major_matrix.size() >= 16) {
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                matrix(row, col) = row_major_matrix[static_cast<std::size_t>(row * 4 + col)];
            }
        }
    }

    Eigen::Affine3f transform;
    transform.matrix() = matrix;
    return transform;
}

void appendVector3(std::vector<jfloat>& values, const Eigen::Vector3f& vector)
{
    values.push_back(vector.x());
    values.push_back(vector.y());
    values.push_back(vector.z());
}

bool makePlane4(const std::vector<jfloat>& values, Eigen::Vector4f& plane)
{
    if (values.size() < 4) {
        return false;
    }
    plane << values[0], values[1], values[2], values[3];
    return true;
}

bool makeLine6(const std::vector<jfloat>& values, Eigen::VectorXf& line)
{
    if (values.size() < 6) {
        return false;
    }
    line.resize(6);
    for (int i = 0; i < 6; ++i) {
        line[i] = values[static_cast<std::size_t>(i)];
    }
    return true;
}

bool makeVector3(const std::vector<jfloat>& values, Eigen::Vector3f& vector)
{
    if (values.size() < 3) {
        return false;
    }
    vector << values[0], values[1], values[2];
    return true;
}

pcl::PointCloud<pcl::Normal>::Ptr computeSurfaceNormals(
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

pcl::PointCloud<pcl::PointNormal>::Ptr computePointNormals(int k_search)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointNormal>::Ptr point_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals = computeSurfaceNormals(input, k_search);
    if (normals->points.size() != input->points.size()) {
        return point_normals;
    }

    point_normals->points.reserve(input->points.size());
    for (std::size_t i = 0; i < input->points.size(); ++i) {
        pcl::PointNormal point_normal;
        point_normal.x = input->points[i].x;
        point_normal.y = input->points[i].y;
        point_normal.z = input->points[i].z;
        point_normal.normal_x = normals->points[i].normal_x;
        point_normal.normal_y = normals->points[i].normal_y;
        point_normal.normal_z = normals->points[i].normal_z;
        point_normal.curvature = normals->points[i].curvature;
        point_normals->points.push_back(point_normal);
    }
    point_normals->width = input->width * input->height == input->points.size()
            ? input->width
            : static_cast<std::uint32_t>(point_normals->points.size());
    point_normals->height = input->width * input->height == input->points.size() ? input->height : 1;
    point_normals->is_dense = input->is_dense;
    return point_normals;
}

template <typename PointT>
std::vector<jfloat> packXYZMesh(
        const pcl::PointCloud<PointT>& vertices,
        const std::vector<pcl::Vertices>& polygons)
{
    std::size_t value_count = 2 + vertices.points.size() * 3;
    for (const auto& polygon : polygons) {
        value_count += polygon.vertices.size() + 1;
    }

    std::vector<jfloat> values;
    values.reserve(value_count);
    values.push_back(static_cast<jfloat>(vertices.points.size()));
    values.push_back(static_cast<jfloat>(polygons.size()));
    for (const auto& vertex : vertices.points) {
        values.push_back(vertex.x);
        values.push_back(vertex.y);
        values.push_back(vertex.z);
    }
    for (const auto& polygon : polygons) {
        values.push_back(static_cast<jfloat>(polygon.vertices.size()));
        for (std::uint32_t vertex : polygon.vertices) {
            values.push_back(static_cast<jfloat>(vertex));
        }
    }
    return values;
}

bool readNonNegativeCount(jfloat value, std::size_t& count)
{
    if (!std::isfinite(value) || value < 0.0f) {
        return false;
    }
    count = static_cast<std::size_t>(value);
    return static_cast<jfloat>(count) == value;
}

bool unpackXYZMesh(const std::vector<jfloat>& values, pcl::PolygonMesh& mesh)
{
    if (values.size() < 2) {
        return false;
    }

    std::size_t vertex_count = 0;
    std::size_t polygon_count = 0;
    if (!readNonNegativeCount(values[0], vertex_count)
            || !readNonNegativeCount(values[1], polygon_count)
            || values.size() < 2 + vertex_count * 3) {
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ> vertices;
    vertices.points.reserve(vertex_count);
    std::size_t offset = 2;
    for (std::size_t i = 0; i < vertex_count; ++i) {
        vertices.points.emplace_back(values[offset], values[offset + 1], values[offset + 2]);
        offset += 3;
    }
    vertices.width = static_cast<std::uint32_t>(vertices.points.size());
    vertices.height = 1;
    vertices.is_dense = false;
    pcl::toPCLPointCloud2(vertices, mesh.cloud);

    mesh.polygons.clear();
    mesh.polygons.reserve(polygon_count);
    for (std::size_t i = 0; i < polygon_count; ++i) {
        if (offset >= values.size()) {
            return false;
        }

        std::size_t polygon_size = 0;
        if (!readNonNegativeCount(values[offset], polygon_size)
                || offset + 1 + polygon_size > values.size()) {
            return false;
        }
        ++offset;

        pcl::Vertices polygon;
        polygon.vertices.reserve(polygon_size);
        for (std::size_t j = 0; j < polygon_size; ++j) {
            std::size_t vertex_index = 0;
            if (!readNonNegativeCount(values[offset + j], vertex_index)
                    || vertex_index >= vertex_count) {
                return false;
            }
            polygon.vertices.push_back(static_cast<std::uint32_t>(vertex_index));
        }
        offset += polygon_size;
        mesh.polygons.push_back(polygon);
    }
    return offset == values.size();
}

std::vector<jfloat> packPolygonMesh(const pcl::PolygonMesh& mesh)
{
    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(mesh.cloud, vertices);
    return packXYZMesh(vertices, mesh.polygons);
}

} // namespace

jfloat radiansToDegrees(float radians)
{
    return static_cast<jfloat>(pcl::rad2deg(radians));
}

jfloat degreesToRadians(float degrees)
{
    return static_cast<jfloat>(pcl::deg2rad(degrees));
}

jfloat normalizeAngleRadians(float radians)
{
    return static_cast<jfloat>(pcl::normAngle(radians));
}

jfloat angleBetweenVectors(
        const std::vector<jfloat>& vector_a_values,
        const std::vector<jfloat>& vector_b_values,
        bool in_degrees)
{
    Eigen::Vector3f vector_a;
    Eigen::Vector3f vector_b;
    if (!makeVector3(vector_a_values, vector_a) || !makeVector3(vector_b_values, vector_b)) {
        return std::numeric_limits<jfloat>::quiet_NaN();
    }
    return static_cast<jfloat>(pcl::getAngle3D(vector_a, vector_b, in_degrees));
}

jfloat squaredPointToLineDistance(
        float point_x,
        float point_y,
        float point_z,
        float line_x,
        float line_y,
        float line_z,
        float direction_x,
        float direction_y,
        float direction_z)
{
    Eigen::Vector4f point(point_x, point_y, point_z, 0.0f);
    Eigen::Vector4f line_point(line_x, line_y, line_z, 0.0f);
    Eigen::Vector4f line_direction(direction_x, direction_y, direction_z, 0.0f);
    if (line_direction.squaredNorm() <= 0.0f) {
        return std::numeric_limits<jfloat>::quiet_NaN();
    }
    return static_cast<jfloat>(pcl::sqrPointToLineDistance(point, line_point, line_direction));
}

jfloat pointDistance(
        float point_a_x,
        float point_a_y,
        float point_a_z,
        float point_b_x,
        float point_b_y,
        float point_b_z)
{
    const pcl::PointXYZ point_a(point_a_x, point_a_y, point_a_z);
    const pcl::PointXYZ point_b(point_b_x, point_b_y, point_b_z);
    return static_cast<jfloat>(pcl::geometry::distance(point_a, point_b));
}

jfloat squaredPointDistance(
        float point_a_x,
        float point_a_y,
        float point_a_z,
        float point_b_x,
        float point_b_y,
        float point_b_z)
{
    const pcl::PointXYZ point_a(point_a_x, point_a_y, point_a_z);
    const pcl::PointXYZ point_b(point_b_x, point_b_y, point_b_z);
    return static_cast<jfloat>(pcl::geometry::squaredDistance(point_a, point_b));
}

jboolean isFinitePoint(float point_x, float point_y, float point_z)
{
    const pcl::PointXYZ point(point_x, point_y, point_z);
    return pcl::isFinite(point) ? JNI_TRUE : JNI_FALSE;
}

jboolean isFiniteXYPoint(float point_x, float point_y)
{
    const pcl::PointXY point(point_x, point_y);
    return pcl::isXYFinite(point) ? JNI_TRUE : JNI_FALSE;
}

jboolean isFiniteNormal(float normal_x, float normal_y, float normal_z)
{
    pcl::Normal normal;
    normal.normal_x = normal_x;
    normal.normal_y = normal_y;
    normal.normal_z = normal_z;
    return pcl::isNormalFinite(normal) ? JNI_TRUE : JNI_FALSE;
}

std::vector<jfloat> projectPointOnPlane(
        float point_x,
        float point_y,
        float point_z,
        float origin_x,
        float origin_y,
        float origin_z,
        float normal_x,
        float normal_y,
        float normal_z)
{
    Eigen::Vector3f point(point_x, point_y, point_z);
    Eigen::Vector3f origin(origin_x, origin_y, origin_z);
    Eigen::Vector3f normal(normal_x, normal_y, normal_z);
    Eigen::Vector3f projected;
    pcl::geometry::project(point, origin, normal, projected);
    return {projected.x(), projected.y(), projected.z()};
}

std::vector<jfloat> projectedUnitVectorOnPlane(
        float point_x,
        float point_y,
        float point_z,
        float origin_x,
        float origin_y,
        float origin_z,
        float normal_x,
        float normal_y,
        float normal_z)
{
    Eigen::Vector3f point(point_x, point_y, point_z);
    Eigen::Vector3f origin(origin_x, origin_y, origin_z);
    Eigen::Vector3f normal(normal_x, normal_y, normal_z);
    Eigen::Vector3f unit = pcl::geometry::projectedAsUnitVector(point, origin, normal);
    return {unit.x(), unit.y(), unit.z()};
}

std::vector<jfloat> computeMeanStd(const std::vector<jfloat>& values)
{
    if (values.empty()) {
        return {};
    }

    std::vector<float> copied(values.begin(), values.end());
    double mean = 0.0;
    double stddev = 0.0;
    pcl::getMeanStd(copied, mean, stddev);
    return {
            static_cast<jfloat>(mean),
            static_cast<jfloat>(stddev),
            static_cast<jfloat>(copied.size()),
    };
}

jfloat computeMedian(const std::vector<jfloat>& values)
{
    if (values.empty()) {
        return std::numeric_limits<jfloat>::quiet_NaN();
    }

    std::vector<float> copied(values.begin(), values.end());
    return static_cast<jfloat>(pcl::computeMedian(copied.begin(), copied.end()));
}

std::vector<int> getPointsInBox(
        float min_x,
        float min_y,
        float min_z,
        float max_x,
        float max_y,
        float max_z)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    Eigen::Vector4f min_point(
            std::min(min_x, max_x),
            std::min(min_y, max_y),
            std::min(min_z, max_z),
            1.0f);
    Eigen::Vector4f max_point(
            std::max(min_x, max_x),
            std::max(min_y, max_y),
            std::max(min_z, max_z),
            1.0f);
    std::vector<int> indices;
    pcl::getPointsInBox(*input, min_point, max_point, indices);
    return indices;
}

std::vector<jfloat> transformPoint(
        float point_x,
        float point_y,
        float point_z,
        const std::vector<jfloat>& row_major_matrix)
{
    pcl::PointXYZ point(point_x, point_y, point_z);
    pcl::PointXYZ transformed = pcl::transformPoint(point, affineFromRowMajorMatrix(row_major_matrix));
    return {transformed.x, transformed.y, transformed.z};
}

std::vector<jfloat> getPrincipalTransformation()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    const double ratio = pcl::getPrincipalTransformation(*input, transform);

    std::vector<jfloat> values;
    values.reserve(17);
    values.push_back(static_cast<jfloat>(ratio));
    appendMatrix4(values, transform.matrix());
    return values;
}

jfloat selectNormDistance(
        const std::vector<jfloat>& values_a,
        const std::vector<jfloat>& values_b,
        int dimension,
        int norm_type)
{
    if (dimension <= 0
            || static_cast<std::size_t>(dimension) > values_a.size()
            || static_cast<std::size_t>(dimension) > values_b.size()
            || norm_type < pcl::L1
            || norm_type > pcl::HIK
            || norm_type == pcl::PF
            || norm_type == pcl::K) {
        return std::numeric_limits<jfloat>::quiet_NaN();
    }

    return static_cast<jfloat>(pcl::selectNorm(
            values_a.data(),
            values_b.data(),
            dimension,
            static_cast<pcl::NormType>(norm_type)));
}

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

std::vector<jfloat> computeCovarianceMatrix()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*input, centroid, covariance);

    std::vector<jfloat> values;
    values.reserve(13);
    values.push_back(centroid.x());
    values.push_back(centroid.y());
    values.push_back(centroid.z());
    appendMatrix3(values, covariance);
    values.push_back(static_cast<jfloat>(input->points.size()));
    LOGI("computeCovarianceMatrix: input=%zu", input->points.size());
    return values;
}

std::vector<jfloat> computeMeanAndCovarianceMatrix()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance;
    const unsigned int point_count = pcl::computeMeanAndCovarianceMatrix(*input, covariance, centroid);
    if (point_count == 0) {
        return {};
    }

    std::vector<jfloat> values;
    values.reserve(13);
    values.push_back(centroid.x());
    values.push_back(centroid.y());
    values.push_back(centroid.z());
    appendMatrix3(values, covariance);
    values.push_back(static_cast<jfloat>(point_count));
    LOGI("computeMeanAndCovarianceMatrix: input=%zu used=%u", input->points.size(), point_count);
    return values;
}

std::vector<jfloat> computePrincipalAxes()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(input);

    std::vector<jfloat> values;
    values.reserve(16);
    const Eigen::Vector4f mean = pca.getMean();
    values.push_back(mean.x());
    values.push_back(mean.y());
    values.push_back(mean.z());
    appendVector3(values, pca.getEigenValues());
    appendMatrix3(values, pca.getEigenVectors());
    values.push_back(static_cast<jfloat>(input->points.size()));
    LOGI("PCA computed principal axes: input=%zu", input->points.size());
    return values;
}

std::vector<jfloat> computeCentroidAndOBB()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    Eigen::Vector3f centroid;
    Eigen::Vector3f obb_center;
    Eigen::Vector3f obb_dimensions;
    Eigen::Matrix3f obb_rotation;
    const unsigned int point_count = pcl::computeCentroidAndOBB(
            *input,
            centroid,
            obb_center,
            obb_dimensions,
            obb_rotation);
    if (point_count == 0) {
        return {};
    }

    std::vector<jfloat> values;
    values.reserve(19);
    appendVector3(values, centroid);
    appendVector3(values, obb_center);
    appendVector3(values, obb_dimensions);
    appendMatrix3(values, obb_rotation);
    values.push_back(static_cast<jfloat>(point_count));
    LOGI("computeCentroidAndOBB: input=%zu used=%u", input->points.size(), point_count);
    return values;
}

std::vector<jfloat> computeMomentOfInertiaAndOBB()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> estimation;
    estimation.setInputCloud(input);
    estimation.compute();

    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    pcl::PointXYZ min_aabb;
    pcl::PointXYZ max_aabb;
    pcl::PointXYZ min_obb;
    pcl::PointXYZ max_obb;
    pcl::PointXYZ position_obb;
    Eigen::Matrix3f rotational_matrix_obb;

    estimation.getMomentOfInertia(moment_of_inertia);
    estimation.getEccentricity(eccentricity);
    estimation.getAABB(min_aabb, max_aabb);
    estimation.getOBB(min_obb, max_obb, position_obb, rotational_matrix_obb);

    std::vector<jfloat> values;
    values.reserve(32);
    for (float value : moment_of_inertia) {
        values.push_back(value);
    }
    for (float value : eccentricity) {
        values.push_back(value);
    }
    appendPoint(values, min_aabb);
    appendPoint(values, max_aabb);
    appendPoint(values, min_obb);
    appendPoint(values, max_obb);
    appendPoint(values, position_obb);
    appendMatrix3(values, rotational_matrix_obb);
    values.push_back(static_cast<jfloat>(input->points.size()));
    LOGI("MomentOfInertiaEstimation computed descriptors: input=%zu moments=%zu eccentricities=%zu",
         input->points.size(), moment_of_inertia.size(), eccentricity.size());
    return values;
}

std::vector<jfloat> computeSquaredDistancesToPoint(float x, float y, float z)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    const pcl::PointXYZ query(x, y, z);
    std::vector<jfloat> values;
    values.reserve(input->points.size());
    for (const auto& point : input->points) {
        values.push_back(pcl::squaredEuclideanDistance(point, query));
    }
    LOGI("computeSquaredDistancesToPoint: input=%zu query=(%.3f, %.3f, %.3f)",
         input->points.size(), x, y, z);
    return values;
}

std::vector<jfloat> computeMaxSegment()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
    const double distance = pcl::getMaxSegment(*input, min_point, max_point);

    std::vector<jfloat> values;
    values.reserve(8);
    appendPoint(values, min_point);
    appendPoint(values, max_point);
    values.push_back(static_cast<jfloat>(distance));
    values.push_back(static_cast<jfloat>(input->points.size()));
    LOGI("getMaxSegment: input=%zu distance=%.6f", input->points.size(), distance);
    return values;
}

std::vector<jfloat> calculateActivePolygonArea()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->points.size() < 3) {
        return {};
    }

    const float area = pcl::calculatePolygonArea(*input);
    std::vector<jfloat> values;
    values.reserve(2);
    values.push_back(area);
    values.push_back(static_cast<jfloat>(input->points.size()));
    LOGI("calculatePolygonArea: vertices=%zu area=%.6f", input->points.size(), area);
    return values;
}

std::vector<jfloat> intersectLines(
        const std::vector<jfloat>& line_a_values,
        const std::vector<jfloat>& line_b_values,
        double squared_epsilon)
{
    Eigen::VectorXf line_a;
    Eigen::VectorXf line_b;
    if (!makeLine6(line_a_values, line_a) || !makeLine6(line_b_values, line_b)) {
        return {};
    }

    Eigen::Vector4f point;
    if (!pcl::lineWithLineIntersection(line_a, line_b, point, squared_epsilon)) {
        return {};
    }

    std::vector<jfloat> values;
    values.reserve(3);
    values.push_back(point.x());
    values.push_back(point.y());
    values.push_back(point.z());
    LOGI("lineWithLineIntersection computed point=(%.3f, %.3f, %.3f)",
         point.x(), point.y(), point.z());
    return values;
}

std::vector<jfloat> intersectPlanes(
        const std::vector<jfloat>& plane_a_values,
        const std::vector<jfloat>& plane_b_values,
        double angular_tolerance)
{
    Eigen::Vector4f plane_a;
    Eigen::Vector4f plane_b;
    if (!makePlane4(plane_a_values, plane_a) || !makePlane4(plane_b_values, plane_b)) {
        return {};
    }

    Eigen::VectorXf line;
    if (!pcl::planeWithPlaneIntersection(plane_a, plane_b, line, angular_tolerance) || line.size() < 6) {
        return {};
    }

    std::vector<jfloat> values;
    values.reserve(6);
    for (int i = 0; i < 6; ++i) {
        values.push_back(line[i]);
    }
    LOGI("planeWithPlaneIntersection computed line point=(%.3f, %.3f, %.3f) direction=(%.3f, %.3f, %.3f)",
         values[0], values[1], values[2], values[3], values[4], values[5]);
    return values;
}

std::vector<jfloat> intersectThreePlanes(
        const std::vector<jfloat>& plane_a_values,
        const std::vector<jfloat>& plane_b_values,
        const std::vector<jfloat>& plane_c_values,
        double determinant_tolerance)
{
    Eigen::Vector4f plane_a;
    Eigen::Vector4f plane_b;
    Eigen::Vector4f plane_c;
    if (!makePlane4(plane_a_values, plane_a)
            || !makePlane4(plane_b_values, plane_b)
            || !makePlane4(plane_c_values, plane_c)) {
        return {};
    }

    Eigen::Vector3f point;
    if (!pcl::threePlanesIntersection(plane_a, plane_b, plane_c, point, determinant_tolerance)) {
        return {};
    }

    std::vector<jfloat> values;
    values.reserve(3);
    appendVector3(values, point);
    LOGI("threePlanesIntersection computed point=(%.3f, %.3f, %.3f)",
         point.x(), point.y(), point.z());
    return values;
}

std::vector<jfloat> computeMaxDistanceFromCentroid()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input, centroid);
    Eigen::Vector4f max_point;
    pcl::getMaxDistance(*input, centroid, max_point);

    const float dx = max_point.x() - centroid.x();
    const float dy = max_point.y() - centroid.y();
    const float dz = max_point.z() - centroid.z();
    std::vector<jfloat> values;
    values.reserve(8);
    values.push_back(centroid.x());
    values.push_back(centroid.y());
    values.push_back(centroid.z());
    values.push_back(max_point.x());
    values.push_back(max_point.y());
    values.push_back(max_point.z());
    values.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
    values.push_back(static_cast<jfloat>(input->points.size()));
    LOGI("getMaxDistance: input=%zu distance=%.6f", input->points.size(), values[6]);
    return values;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr demeanActiveCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr demeaned(new pcl::PointCloud<pcl::PointXYZ>);
    if (input->empty()) {
        return demeaned;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input, centroid);
    pcl::demeanPointCloud(*input, centroid, *demeaned);
    LOGI("demeanPointCloud: input=%zu output=%zu", input->points.size(), demeaned->points.size());
    return demeaned;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr computeConcaveHull(double alpha)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
    if (input->empty() || alpha <= 0.0) {
        return hull;
    }

    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
    concave_hull.setInputCloud(input);
    concave_hull.setAlpha(alpha);
    concave_hull.reconstruct(*hull);
    LOGI("ConcaveHull reconstructed points: input=%zu hull=%zu alpha=%.3f dimension=%d",
         input->points.size(), hull->points.size(), alpha, concave_hull.getDimension());
    return hull;
}

std::vector<jfloat> computeConvexHullMesh()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty()) {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ> hull;
    std::vector<pcl::Vertices> polygons;
    pcl::ConvexHull<pcl::PointXYZ> convex_hull;
    convex_hull.setInputCloud(input);
    convex_hull.reconstruct(hull, polygons);

    std::vector<jfloat> values = packXYZMesh(hull, polygons);
    LOGI("ConvexHull reconstructed mesh: input=%zu vertices=%zu polygons=%zu dimension=%d",
         input->points.size(), hull.points.size(), polygons.size(), convex_hull.getDimension());
    return values;
}

std::vector<jfloat> computeConcaveHullMesh(double alpha)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || alpha <= 0.0) {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ> hull;
    std::vector<pcl::Vertices> polygons;
    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
    concave_hull.setInputCloud(input);
    concave_hull.setAlpha(alpha);
    concave_hull.reconstruct(hull, polygons);

    std::vector<jfloat> values = packXYZMesh(hull, polygons);
    LOGI("ConcaveHull reconstructed mesh: input=%zu vertices=%zu polygons=%zu alpha=%.3f dimension=%d",
         input->points.size(), hull.points.size(), polygons.size(), alpha, concave_hull.getDimension());
    return values;
}

std::vector<jfloat> simplifyMeshRemoveUnusedVertices(const std::vector<jfloat>& packed_mesh)
{
    pcl::PolygonMesh input;
    if (!unpackXYZMesh(packed_mesh, input)) {
        return {};
    }

    pcl::PolygonMesh output;
    pcl::surface::SimplificationRemoveUnusedVertices simplification;
    simplification.simplify(input, output);

    std::vector<jfloat> values = packPolygonMesh(output);
    LOGI("SimplificationRemoveUnusedVertices simplified mesh: vertices=%u polygons=%zu output_values=%zu",
         input.cloud.width * input.cloud.height, input.polygons.size(), values.size());
    return values;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr smoothMovingLeastSquares(double search_radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed(new pcl::PointCloud<pcl::PointXYZ>);
    if (input->empty() || search_radius <= 0.0) {
        return smoothed;
    }

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(input);
    mls.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    mls.setSearchRadius(search_radius);
    mls.process(*smoothed);
    LOGI("MovingLeastSquares smoothed points: input=%zu output=%zu radius=%.3f",
         input->points.size(), smoothed->points.size(), search_radius);
    return smoothed;
}

std::vector<jfloat> smoothSurfelSmoothing(int normal_k_search, double scale)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || normal_k_search <= 0 || scale <= 0.0) {
        return {};
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals = computeSurfaceNormals(input, normal_k_search);
    if (normals->points.size() != input->points.size()) {
        return {};
    }

    pcl::SurfelSmoothing<pcl::PointXYZ, pcl::Normal> smoothing(static_cast<float>(scale));
    smoothing.setInputCloud(input);
    smoothing.setInputNormals(normals);
    smoothing.setSearchMethod(
            pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));

    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr smoothed_normals(new pcl::PointCloud<pcl::Normal>);
    smoothing.computeSmoothedCloud(smoothed_points, smoothed_normals);
    if (smoothed_points->points.size() != smoothed_normals->points.size()) {
        return {};
    }

    std::vector<jfloat> values;
    values.reserve(smoothed_points->points.size() * 7);
    for (std::size_t i = 0; i < smoothed_points->points.size(); ++i) {
        values.push_back(smoothed_points->points[i].x);
        values.push_back(smoothed_points->points[i].y);
        values.push_back(smoothed_points->points[i].z);
        values.push_back(smoothed_normals->points[i].normal_x);
        values.push_back(smoothed_normals->points[i].normal_y);
        values.push_back(smoothed_normals->points[i].normal_z);
        values.push_back(smoothed_normals->points[i].curvature);
    }
    LOGI("SurfelSmoothing smoothed points: input=%zu output=%zu normal_k=%d scale=%.3f",
         input->points.size(), smoothed_points->points.size(), normal_k_search, scale);
    return values;
}

std::vector<jfloat> reconstructGreedyProjectionTriangles(
        int normal_k_search,
        double search_radius,
        double mu,
        int maximum_nearest_neighbors,
        double maximum_surface_angle,
        double minimum_angle,
        double maximum_angle,
        bool normal_consistency)
{
    if (normal_k_search <= 0
            || search_radius <= 0.0
            || mu <= 0.0
            || maximum_nearest_neighbors <= 0
            || maximum_surface_angle <= 0.0
            || minimum_angle <= 0.0
            || maximum_angle <= minimum_angle) {
        return {};
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr point_normals = computePointNormals(normal_k_search);
    if (point_normals->empty()) {
        return {};
    }

    std::vector<pcl::Vertices> polygons;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    gp3.setInputCloud(point_normals);
    gp3.setSearchRadius(search_radius);
    gp3.setMu(mu);
    gp3.setMaximumNearestNeighbors(maximum_nearest_neighbors);
    gp3.setMaximumSurfaceAngle(maximum_surface_angle);
    gp3.setMinimumAngle(minimum_angle);
    gp3.setMaximumAngle(maximum_angle);
    gp3.setNormalConsistency(normal_consistency);
    gp3.reconstruct(polygons);

    std::vector<jfloat> values;
    values.reserve(polygons.size() * 3);
    for (const auto& polygon : polygons) {
        if (polygon.vertices.size() == 3) {
            values.push_back(static_cast<jfloat>(polygon.vertices[0]));
            values.push_back(static_cast<jfloat>(polygon.vertices[1]));
            values.push_back(static_cast<jfloat>(polygon.vertices[2]));
        }
    }
    LOGI("GreedyProjectionTriangulation reconstructed triangles: input=%zu polygons=%zu triangles=%zu radius=%.3f",
         point_normals->points.size(), polygons.size(), values.size() / 3, search_radius);
    return values;
}

std::vector<jfloat> reconstructGridProjectionMesh(
        int normal_k_search,
        double resolution,
        int padding_size,
        int nearest_neighbor_count,
        int max_binary_search_level)
{
    if (normal_k_search <= 0 || resolution <= 0.0) {
        return {};
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr point_normals = computePointNormals(normal_k_search);
    if (point_normals->empty()) {
        return {};
    }

    pcl::PointCloud<pcl::PointNormal> vertices;
    std::vector<pcl::Vertices> polygons;
    pcl::GridProjection<pcl::PointNormal> grid_projection(resolution);
    grid_projection.setInputCloud(point_normals);
    grid_projection.setSearchMethod(
            pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
    grid_projection.setPaddingSize(std::max(padding_size, 0));
    grid_projection.setNearestNeighborNum(std::max(nearest_neighbor_count, 1));
    grid_projection.setMaxBinarySearchLevel(std::max(max_binary_search_level, 1));
    grid_projection.reconstruct(vertices, polygons);

    std::size_t value_count = 2 + vertices.points.size() * 3;
    for (const auto& polygon : polygons) {
        value_count += polygon.vertices.size() + 1;
    }

    std::vector<jfloat> values;
    values.reserve(value_count);
    values.push_back(static_cast<jfloat>(vertices.points.size()));
    values.push_back(static_cast<jfloat>(polygons.size()));
    for (const auto& vertex : vertices.points) {
        values.push_back(vertex.x);
        values.push_back(vertex.y);
        values.push_back(vertex.z);
    }
    for (const auto& polygon : polygons) {
        values.push_back(static_cast<jfloat>(polygon.vertices.size()));
        for (std::uint32_t vertex : polygon.vertices) {
            values.push_back(static_cast<jfloat>(vertex));
        }
    }
    LOGI("GridProjection reconstructed mesh: input=%zu vertices=%zu polygons=%zu resolution=%.3f",
         point_normals->points.size(), vertices.points.size(), polygons.size(), resolution);
    return values;
}

std::vector<jfloat> reconstructMarchingCubesHoppeMesh(
        int normal_k_search,
        int resolution_x,
        int resolution_y,
        int resolution_z,
        double percentage_extend_grid,
        double iso_level,
        double distance_ignore)
{
    if (normal_k_search <= 0 || resolution_x <= 0 || resolution_y <= 0 || resolution_z <= 0) {
        return {};
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr point_normals = computePointNormals(normal_k_search);
    if (point_normals->empty()) {
        return {};
    }

    pcl::PointCloud<pcl::PointNormal> vertices;
    std::vector<pcl::Vertices> polygons;
    pcl::MarchingCubesHoppe<pcl::PointNormal> marching_cubes(
            static_cast<float>(distance_ignore),
            static_cast<float>(std::max(percentage_extend_grid, 0.0)),
            static_cast<float>(iso_level));
    marching_cubes.setInputCloud(point_normals);
    marching_cubes.setSearchMethod(
            pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
    marching_cubes.setGridResolution(resolution_x, resolution_y, resolution_z);
    marching_cubes.reconstruct(vertices, polygons);

    std::size_t value_count = 2 + vertices.points.size() * 3;
    for (const auto& polygon : polygons) {
        value_count += polygon.vertices.size() + 1;
    }

    std::vector<jfloat> values;
    values.reserve(value_count);
    values.push_back(static_cast<jfloat>(vertices.points.size()));
    values.push_back(static_cast<jfloat>(polygons.size()));
    for (const auto& vertex : vertices.points) {
        values.push_back(vertex.x);
        values.push_back(vertex.y);
        values.push_back(vertex.z);
    }
    for (const auto& polygon : polygons) {
        values.push_back(static_cast<jfloat>(polygon.vertices.size()));
        for (std::uint32_t vertex : polygon.vertices) {
            values.push_back(static_cast<jfloat>(vertex));
        }
    }
    LOGI("MarchingCubesHoppe reconstructed mesh: input=%zu vertices=%zu polygons=%zu resolution=%dx%dx%d",
         point_normals->points.size(),
         vertices.points.size(),
         polygons.size(),
         resolution_x,
         resolution_y,
         resolution_z);
    return values;
}

std::vector<jfloat> reconstructMarchingCubesRBFMesh(
        int normal_k_search,
        int resolution_x,
        int resolution_y,
        int resolution_z,
        double off_surface_displacement,
        double percentage_extend_grid,
        double iso_level)
{
    if (normal_k_search <= 0
            || resolution_x <= 0
            || resolution_y <= 0
            || resolution_z <= 0
            || off_surface_displacement <= 0.0) {
        return {};
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr point_normals = computePointNormals(normal_k_search);
    if (point_normals->empty()) {
        return {};
    }

    pcl::PointCloud<pcl::PointNormal> vertices;
    std::vector<pcl::Vertices> polygons;
    pcl::MarchingCubesRBF<pcl::PointNormal> marching_cubes(
            static_cast<float>(off_surface_displacement),
            static_cast<float>(std::max(percentage_extend_grid, 0.0)),
            static_cast<float>(iso_level));
    marching_cubes.setInputCloud(point_normals);
    marching_cubes.setSearchMethod(
            pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
    marching_cubes.setGridResolution(resolution_x, resolution_y, resolution_z);
    marching_cubes.reconstruct(vertices, polygons);

    std::size_t value_count = 2 + vertices.points.size() * 3;
    for (const auto& polygon : polygons) {
        value_count += polygon.vertices.size() + 1;
    }

    std::vector<jfloat> values;
    values.reserve(value_count);
    values.push_back(static_cast<jfloat>(vertices.points.size()));
    values.push_back(static_cast<jfloat>(polygons.size()));
    for (const auto& vertex : vertices.points) {
        values.push_back(vertex.x);
        values.push_back(vertex.y);
        values.push_back(vertex.z);
    }
    for (const auto& polygon : polygons) {
        values.push_back(static_cast<jfloat>(polygon.vertices.size()));
        for (std::uint32_t vertex : polygon.vertices) {
            values.push_back(static_cast<jfloat>(vertex));
        }
    }
    LOGI("MarchingCubesRBF reconstructed mesh: input=%zu vertices=%zu polygons=%zu resolution=%dx%dx%d",
         point_normals->points.size(),
         vertices.points.size(),
         polygons.size(),
         resolution_x,
         resolution_y,
         resolution_z);
    return values;
}

std::vector<jfloat> reconstructOrganizedFastMeshPolygons(
        int triangulation_type,
        int triangle_pixel_size,
        double max_edge_length_a,
        double max_edge_length_b,
        double max_edge_length_c,
        double angle_tolerance,
        double distance_tolerance,
        bool distance_dependent,
        bool use_depth_as_distance,
        bool store_shadowed_faces)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = activeCloud();
    if (input->empty() || input->height <= 1 || input->width * input->height != input->points.size()) {
        return {};
    }

    pcl::OrganizedFastMesh<pcl::PointXYZ> mesh;
    mesh.setInputCloud(input);
    const int bounded_type = std::max(0, std::min(triangulation_type, 3));
    mesh.setTriangulationType(
            static_cast<pcl::OrganizedFastMesh<pcl::PointXYZ>::TriangulationType>(bounded_type));
    mesh.setTrianglePixelSize(std::max(triangle_pixel_size, 1));
    if (max_edge_length_a > 0.0 || max_edge_length_b > 0.0 || max_edge_length_c > 0.0) {
        mesh.setMaxEdgeLength(
                static_cast<float>(std::max(max_edge_length_a, 0.0)),
                static_cast<float>(std::max(max_edge_length_b, 0.0)),
                static_cast<float>(std::max(max_edge_length_c, 0.0)));
    }
    mesh.setAngleTolerance(static_cast<float>(angle_tolerance));
    mesh.setDistanceTolerance(static_cast<float>(distance_tolerance), distance_dependent);
    mesh.useDepthAsDistance(use_depth_as_distance);
    mesh.storeShadowedFaces(store_shadowed_faces);

    std::vector<pcl::Vertices> polygons;
    mesh.reconstruct(polygons);

    std::size_t value_count = 0;
    for (const auto& polygon : polygons) {
        value_count += polygon.vertices.size() + 1;
    }

    std::vector<jfloat> values;
    values.reserve(value_count);
    for (const auto& polygon : polygons) {
        values.push_back(static_cast<jfloat>(polygon.vertices.size()));
        for (std::uint32_t vertex : polygon.vertices) {
            values.push_back(static_cast<jfloat>(vertex));
        }
    }
    LOGI("OrganizedFastMesh reconstructed polygons: input=%zu width=%u height=%u polygons=%zu",
         input->points.size(), input->width, input->height, polygons.size());
    return values;
}

} // namespace pclmobile
