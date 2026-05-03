#ifndef PCL_MOBILE_REGISTRATION_H
#define PCL_MOBILE_REGISTRATION_H

#include <jni.h>

#include <vector>

namespace pclmobile {

std::vector<jfloat> alignToTranslatedCopyICP(float tx, float ty, float tz, int max_iterations);

} // namespace pclmobile

#endif // PCL_MOBILE_REGISTRATION_H
