#ifndef PCL_MOBILE_FEATURES_H
#define PCL_MOBILE_FEATURES_H

#include <jni.h>

#include <vector>

namespace pclmobile {

std::vector<jfloat> estimateNormals(int k_search);
std::vector<jfloat> estimateNormalsRadius(double radius_search);
std::vector<jfloat> computeFPFHFeatures(int normal_k_search, double feature_radius);
std::vector<jfloat> computePrincipalCurvatures(int normal_k_search, int curvature_k_search);

} // namespace pclmobile

#endif // PCL_MOBILE_FEATURES_H
