# cmake/SetupSuperbuild.cmake
#
# Derived from Sirokujira/pcl-superbuild@83cc231 — setup-superbuild.cmake.
# See docs/LINEAGE.md.
#
# Renamed from setup-superbuild.cmake. Defines the per-tree paths used by all
# ExternalProject_Add calls under cmake/external/.

include_guard(GLOBAL)
include(ExternalProject)

set(base "${CMAKE_BINARY_DIR}/CMakeExternals" CACHE INTERNAL "Superbuild root")
set_property(DIRECTORY PROPERTY EP_BASE ${base})

macro(set_default_build_type build_type)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE ${build_type})
    endif()
    set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Build configuration type" FORCE)
endmacro()

set_default_build_type(Release)

set(build_type     ${CMAKE_BUILD_TYPE})
set(source_prefix  ${base}/Source  CACHE INTERNAL "Per-EP source dir")
set(build_prefix   ${base}/Build   CACHE INTERNAL "Per-EP build dir")
set(install_prefix ${base}/Install CACHE INTERNAL "Per-EP install dir")

message(STATUS "[superbuild] base           = ${base}")
message(STATUS "[superbuild] install_prefix = ${install_prefix}")
