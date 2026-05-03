#ifndef PCL_MOBILE_SEARCH_H
#define PCL_MOBILE_SEARCH_H

#include <jni.h>

#include <vector>

namespace pclmobile {

std::vector<jfloat> nearestKSearch(float x, float y, float z, int k);
std::vector<jfloat> octreeRadiusSearch(float x, float y, float z, double resolution, double radius);

} // namespace pclmobile

#endif // PCL_MOBILE_SEARCH_H
