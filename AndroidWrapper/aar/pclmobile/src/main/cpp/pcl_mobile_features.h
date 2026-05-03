#ifndef PCL_MOBILE_FEATURES_H
#define PCL_MOBILE_FEATURES_H

#include <jni.h>

#include <vector>

namespace pclmobile {

std::vector<jfloat> estimateNormals(int k_search);

} // namespace pclmobile

#endif // PCL_MOBILE_FEATURES_H
