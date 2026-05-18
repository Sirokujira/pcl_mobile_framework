#ifndef PCL_MOBILE_SURFACE_H
#define PCL_MOBILE_SURFACE_H

#include <jni.h>

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pclmobile {

std::vector<jfloat> computeCentroidAndBounds();
std::vector<jfloat> computeCovarianceMatrix();
std::vector<jfloat> computePrincipalAxes();
std::vector<jfloat> computeMomentOfInertiaAndOBB();
std::vector<jfloat> computeSquaredDistancesToPoint(float x, float y, float z);
std::vector<jfloat> computeMaxDistanceFromCentroid();
pcl::PointCloud<pcl::PointXYZ>::Ptr demeanActiveCloud();
pcl::PointCloud<pcl::PointXYZ>::Ptr computeConvexHull();
pcl::PointCloud<pcl::PointXYZ>::Ptr computeConcaveHull(double alpha);
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
