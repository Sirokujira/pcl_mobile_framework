#ifndef PCL_MOBILE_FEATURES_H
#define PCL_MOBILE_FEATURES_H

#include <jni.h>

#include <vector>

namespace pclmobile {

std::vector<jfloat> estimateNormals(int k_search);
std::vector<jfloat> estimateNormalsRadius(double radius_search);
std::vector<jfloat> estimateNormalsOMP(int k_search, int number_of_threads);
std::vector<jfloat> estimateLinearLeastSquaresNormals(
        double normal_smoothing_size,
        bool depth_dependent_smoothing,
        double max_depth_change_factor);
std::vector<jfloat> refineNormals(
        int normal_k_search,
        int refinement_k_search,
        int max_iterations,
        double convergence_threshold);
std::vector<jfloat> computePFHFeatures(int normal_k_search, double feature_radius);
std::vector<jfloat> computePFHRGBFeatures(int normal_k_search, double feature_radius);
std::vector<jfloat> computeFPFHFeatures(int normal_k_search, double feature_radius);
std::vector<jfloat> computeFPFHFeaturesOMP(int normal_k_search, double feature_radius, int number_of_threads);
std::vector<jfloat> computeVFHFeatures(int normal_k_search);
std::vector<jfloat> computeESFDescriptor();
std::vector<jfloat> computeGASDDescriptor();
std::vector<jfloat> computeCRHDescriptor(int normal_k_search, float viewpoint_x, float viewpoint_y, float viewpoint_z);
std::vector<jfloat> computeCVFHFeatures(
        int normal_k_search,
        double cluster_tolerance,
        double eps_angle_threshold,
        double curvature_threshold,
        int min_points,
        bool normalize_bins);
std::vector<jfloat> computeOURCVFHFeatures(
        int normal_k_search,
        double cluster_tolerance,
        double eps_angle_threshold,
        double curvature_threshold,
        int min_points,
        bool normalize_bins,
        double refine_clusters,
        double axis_ratio,
        double min_axis_value);
std::vector<jfloat> computeIntensitySpinFeatures(
        double radius_search,
        double smoothing_bandwidth);
std::vector<jfloat> computeIntensityGradientFeatures(
        int normal_k_search,
        double radius_search,
        int number_of_threads);
std::vector<jfloat> computeRIFTFeatures(
        int normal_k_search,
        double gradient_radius,
        double feature_radius);
std::vector<jfloat> computeROPSFeatures(
        int normal_k_search,
        double support_radius,
        double mesh_search_radius,
        double mu,
        int maximum_nearest_neighbors);
std::vector<jfloat> computeShapeContext3DFeatures(
        int normal_k_search,
        double search_radius,
        double min_radius,
        double point_density_radius,
        bool random);
std::vector<jfloat> computeUniqueShapeContextFeatures(
        double search_radius,
        double min_radius,
        double point_density_radius,
        double local_radius);
std::vector<jfloat> computePPFFeatures(int normal_k_search, int max_point_count);
std::vector<jfloat> computePPFRGBFeatures(int normal_k_search, int max_point_count);
std::vector<jfloat> computeCPPFPairFeatureValues(int first_index, int second_index, int normal_k_search);
std::vector<jfloat> computeNormalBasedSignatureFeatures(
        int normal_k_search,
        double search_radius,
        double scale,
        int n,
        int m);
std::vector<jfloat> computeSpinImageFeatures(
        int normal_k_search,
        double feature_radius,
        int image_width,
        double support_angle_cos,
        int min_point_count);
std::vector<jfloat> computeGRSDDescriptor(
        int normal_k_search,
        double radius_search,
        double plane_radius,
        int subdivisions);
std::vector<jfloat> computeMomentInvariants(double radius_search);
std::vector<jfloat> computeRSDFeatures(
        int normal_k_search,
        double radius_search,
        double plane_radius,
        int subdivisions);
std::vector<jfloat> computePrincipalCurvatures(int normal_k_search, int curvature_k_search);
std::vector<jfloat> computeSHOTFeatures(int normal_k_search, double feature_radius);
std::vector<jfloat> computeSHOTFeaturesOMP(int normal_k_search, double feature_radius, int number_of_threads);
std::vector<jfloat> computeSHOTLocalReferenceFrames(double radius_search, bool use_omp, int number_of_threads);
std::vector<jfloat> computeBOARDLocalReferenceFrames(
        int normal_k_search,
        double radius_search,
        double tangent_radius,
        bool find_holes,
        double margin_threshold);
std::vector<jfloat> computeFLARELocalReferenceFrames(
        int normal_k_search,
        double radius_search,
        double tangent_radius,
        double margin_threshold,
        int min_neighbors_for_normal_axis,
        int min_neighbors_for_tangent_axis);
std::vector<jfloat> computeBoundaryPoints(
        int normal_k_search,
        double radius_search,
        double angle_threshold_degrees);
std::vector<jfloat> computeDifferenceOfNormals(double small_radius, double large_radius);
std::vector<int> extractStatisticalMultiscaleInterestRegionIndices(
        double first_scale,
        double second_scale,
        double third_scale,
        int max_point_count);

} // namespace pclmobile

#endif // PCL_MOBILE_FEATURES_H
