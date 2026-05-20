#ifndef PCL_MOBILE_SEARCH_H
#define PCL_MOBILE_SEARCH_H

#include <jni.h>

#include <vector>

namespace pclmobile {

std::vector<jfloat> nearestKSearch(float x, float y, float z, int k);
std::vector<jfloat> nearestKSearchIndices(float x, float y, float z, int k);
std::vector<jfloat> radiusSearch(float x, float y, float z, double radius);
std::vector<jfloat> radiusSearchIndices(float x, float y, float z, double radius);
std::vector<jfloat> radiusSearchLimited(float x, float y, float z, double radius, int max_neighbors);
std::vector<jfloat> radiusSearchIndicesLimited(float x, float y, float z, double radius, int max_neighbors);
std::vector<jfloat> bruteForceNearestKSearch(float x, float y, float z, int k);
std::vector<jfloat> bruteForceNearestKSearchIndices(float x, float y, float z, int k);
std::vector<jfloat> bruteForceRadiusSearchLimited(float x, float y, float z, double radius, int max_neighbors);
std::vector<jfloat> bruteForceRadiusSearchIndicesLimited(
        float x, float y, float z, double radius, int max_neighbors);
std::vector<jfloat> octreeNearestKSearch(float x, float y, float z, double resolution, int k);
std::vector<jfloat> octreeNearestKSearchIndices(float x, float y, float z, double resolution, int k);
std::vector<jfloat> octreeRadiusSearch(float x, float y, float z, double resolution, double radius);
std::vector<jfloat> octreeRadiusSearchIndices(float x, float y, float z, double resolution, double radius);
std::vector<jfloat> octreeRadiusSearchLimited(
        float x, float y, float z, double resolution, double radius, int max_neighbors);
std::vector<jfloat> octreeRadiusSearchIndicesLimited(
        float x, float y, float z, double resolution, double radius, int max_neighbors);
std::vector<jfloat> octreeVoxelSearch(float x, float y, float z, double resolution);
std::vector<jfloat> octreeVoxelSearchIndices(float x, float y, float z, double resolution);
std::vector<jfloat> octreeApproxNearestSearch(float x, float y, float z, double resolution);
std::vector<jfloat> octreeApproxNearestSearchIndex(float x, float y, float z, double resolution);

} // namespace pclmobile

#endif // PCL_MOBILE_SEARCH_H
