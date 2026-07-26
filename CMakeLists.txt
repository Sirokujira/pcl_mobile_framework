# Top-level CMakeLists.txt for the pcl_mobile_framework superbuild.
#
# Direct descendant of Sirokujira/pcl-superbuild@83cc231 — CMakeLists.txt.
# See docs/LINEAGE.md.
#
# This is a "superbuild" in the CMake sense: it doesn't build any code
# itself, only orchestrates ExternalProject_Add of PCL and its dependencies
# for one cross-compilation target at a time.
#
#   * Android slices: driven by scripts/build_android.sh
#   * iOS slices:     driven by scripts/build_ios.sh
#
# See MODERNIZATION_PLAN.md for context.

cmake_minimum_required(VERSION 3.24 FATAL_ERROR)

# CMP0135: ExternalProject URL downloads honour the file's mtime, not the
# extraction time. Required by ExternalProject_Add caching.
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

project(pcl_mobile_superbuild
    VERSION 0.1.0
    DESCRIPTION "Cross-compile PCL and its dependencies for iOS / Android"
    LANGUAGES NONE
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(SetupSuperbuild)
include(ProjectVariables)

# Each ExternalProject_Add wrapper lives in its own file so the dependency
# tree is grep-friendly.
include(external/eigen)
include(external/boost)
include(external/lz4)
include(external/flann)
include(external/qhull)
include(external/pcl)

# Eigen is header-only and shared across slices.
install_eigen()

# For every requested slice, build LZ4 -> Boost/FLANN/Qhull, then PCL on top.
# (LZ4 is required by FLANN 1.9.2 -- see cmake/external/lz4.cmake.)
foreach(tag IN LISTS PCL_MOBILE_TARGETS)
    crosscompile_lz4(${tag})
    crosscompile_boost(${tag})
    crosscompile_flann(${tag})
    crosscompile_qhull(${tag})
    crosscompile_pcl(${tag})
endforeach()

message(STATUS "")
message(STATUS "pcl_mobile_framework superbuild configured.")
message(STATUS "  PCL_MOBILE_TARGETS = ${PCL_MOBILE_TARGETS}")
message(STATUS "  install_prefix     = ${install_prefix}")
message(STATUS "  Run: cmake --build <build dir> --parallel")
