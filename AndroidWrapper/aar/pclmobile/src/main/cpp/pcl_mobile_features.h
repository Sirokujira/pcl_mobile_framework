#ifndef PCL_MOBILE_FEATURES_H
#define PCL_MOBILE_FEATURES_H

#include <jni.h>

#include <vector>

namespace pclmobile {

std::vector<jfloat> estimateNormals(int k_search);
std::vector<jfloat> estimateNormalsRadius(double radius_search);
std::vector<jfloat> computePFHFeatures(int normal_k_search, double feature_radius);
std::vector<jfloat> computeFPFHFeatures(int normal_k_search, double feature_radius);
std::vector<jfloat> computeVFHFeatures(int normal_k_search);
std::vector<jfloat> computeESFDescriptor();
std::vector<jfloat> computeGASDDescriptor();
std::vector<jfloat> computeMomentInvariants(double radius_search);
std::vector<jfloat> computeRSDFeatures(
        int normal_k_search,
        double radius_search,
        double plane_radius,
        int subdivisions);
std::vector<jfloat> computePrincipalCurvatures(int normal_k_search, int curvature_k_search);
std::vector<jfloat> computeSHOTFeatures(int normal_k_search, double feature_radius);
std::vector<jfloat> computeBoundaryPoints(
        int normal_k_search,
        double radius_search,
        double angle_threshold_degrees);
std::vector<jfloat> computeDifferenceOfNormals(double small_radius, double large_radius);

} // namespace pclmobile

#endif // PCL_MOBILE_FEATURES_H
