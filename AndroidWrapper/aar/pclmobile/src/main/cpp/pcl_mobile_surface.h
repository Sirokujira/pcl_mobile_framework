#ifndef PCL_MOBILE_SURFACE_H
#define PCL_MOBILE_SURFACE_H

#include <jni.h>

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pclmobile {

jfloat radiansToDegrees(float radians);
jfloat degreesToRadians(float degrees);
jfloat normalizeAngleRadians(float radians);
jfloat angleBetweenVectors(
        const std::vector<jfloat>& vector_a_values,
        const std::vector<jfloat>& vector_b_values,
        bool in_degrees);
jfloat squaredPointToLineDistance(
        float point_x,
        float point_y,
        float point_z,
        float line_x,
        float line_y,
        float line_z,
        float direction_x,
        float direction_y,
        float direction_z);
jfloat pointDistance(
        float point_a_x,
        float point_a_y,
        float point_a_z,
        float point_b_x,
        float point_b_y,
        float point_b_z);
jfloat squaredPointDistance(
        float point_a_x,
        float point_a_y,
        float point_a_z,
        float point_b_x,
        float point_b_y,
        float point_b_z);
jboolean isFinitePoint(float point_x, float point_y, float point_z);
jboolean isFiniteXYPoint(float point_x, float point_y);
jboolean isFiniteNormal(float normal_x, float normal_y, float normal_z);
std::vector<jfloat> projectPointOnPlane(
        float point_x,
        float point_y,
        float point_z,
        float origin_x,
        float origin_y,
        float origin_z,
        float normal_x,
        float normal_y,
        float normal_z);
std::vector<jfloat> projectedUnitVectorOnPlane(
        float point_x,
        float point_y,
        float point_z,
        float origin_x,
        float origin_y,
        float origin_z,
        float normal_x,
        float normal_y,
        float normal_z);
std::vector<jfloat> computeMeanStd(const std::vector<jfloat>& values);
jfloat computeMedian(const std::vector<jfloat>& values);
std::vector<int> getPointsInBox(
        float min_x,
        float min_y,
        float min_z,
        float max_x,
        float max_y,
        float max_z);
std::vector<jfloat> transformPoint(
        float point_x,
        float point_y,
        float point_z,
        const std::vector<jfloat>& row_major_matrix);
std::vector<jfloat> getPrincipalTransformation();
jfloat selectNormDistance(
        const std::vector<jfloat>& values_a,
        const std::vector<jfloat>& values_b,
        int dimension,
        int norm_type);
std::vector<jfloat> computeCentroidAndBounds();
std::vector<jfloat> computeCovarianceMatrix();
std::vector<jfloat> computeMeanAndCovarianceMatrix();
std::vector<jfloat> computePrincipalAxes();
std::vector<jfloat> computeCentroidAndOBB();
std::vector<jfloat> computeMomentOfInertiaAndOBB();
std::vector<jfloat> computeSquaredDistancesToPoint(float x, float y, float z);
std::vector<jfloat> computeMaxSegment();
std::vector<jfloat> computeMaxDistanceFromCentroid();
std::vector<jfloat> calculateActivePolygonArea();
std::vector<jfloat> intersectLines(
        const std::vector<jfloat>& line_a_values,
        const std::vector<jfloat>& line_b_values,
        double squared_epsilon);
std::vector<jfloat> intersectPlanes(
        const std::vector<jfloat>& plane_a_values,
        const std::vector<jfloat>& plane_b_values,
        double angular_tolerance);
std::vector<jfloat> intersectThreePlanes(
        const std::vector<jfloat>& plane_a_values,
        const std::vector<jfloat>& plane_b_values,
        const std::vector<jfloat>& plane_c_values,
        double determinant_tolerance);
pcl::PointCloud<pcl::PointXYZ>::Ptr demeanActiveCloud();
pcl::PointCloud<pcl::PointXYZ>::Ptr computeConvexHull();
pcl::PointCloud<pcl::PointXYZ>::Ptr computeConcaveHull(double alpha);
std::vector<jfloat> computeConvexHullMesh();
std::vector<jfloat> computeConcaveHullMesh(double alpha);
std::vector<jfloat> simplifyMeshRemoveUnusedVertices(const std::vector<jfloat>& packed_mesh);
pcl::PointCloud<pcl::PointXYZ>::Ptr projectInliersToPlane(double distance_threshold, int max_iterations);
pcl::PointCloud<pcl::PointXYZ>::Ptr smoothMovingLeastSquares(double search_radius);
std::vector<jfloat> smoothSurfelSmoothing(int normal_k_search, double scale);
std::vector<jfloat> reconstructGreedyProjectionTriangles(
        int normal_k_search,
        double search_radius,
        double mu,
        int maximum_nearest_neighbors,
        double maximum_surface_angle,
        double minimum_angle,
        double maximum_angle,
        bool normal_consistency);
std::vector<jfloat> reconstructGridProjectionMesh(
        int normal_k_search,
        double resolution,
        int padding_size,
        int nearest_neighbor_count,
        int max_binary_search_level);
std::vector<jfloat> reconstructMarchingCubesHoppeMesh(
        int normal_k_search,
        int resolution_x,
        int resolution_y,
        int resolution_z,
        double percentage_extend_grid,
        double iso_level,
        double distance_ignore);
std::vector<jfloat> reconstructMarchingCubesRBFMesh(
        int normal_k_search,
        int resolution_x,
        int resolution_y,
        int resolution_z,
        double off_surface_displacement,
        double percentage_extend_grid,
        double iso_level);
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
        bool store_shadowed_faces);

} // namespace pclmobile

#endif // PCL_MOBILE_SURFACE_H
