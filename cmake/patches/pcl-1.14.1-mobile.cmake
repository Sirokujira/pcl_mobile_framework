set(_file "${PCL_SOURCE_DIR}/registration/include/pcl/registration/correspondence_rejection_features.h")

file(READ "${_file}" _content)
string(REPLACE
    "this->getClassName().c_str()"
    "\"CorrespondenceRejectorFeatures\""
    _patched
    "${_content}")

if(_patched STREQUAL _content)
    message(FATAL_ERROR "PCL patch did not match: ${_file}")
endif()

file(WRITE "${_file}" "${_patched}")

set(_surface_cmake "${PCL_SOURCE_DIR}/surface/CMakeLists.txt")
file(READ "${_surface_cmake}" _surface_content)
string(REPLACE "  src/poisson.cpp\n" "" _surface_patched "${_surface_content}")
string(REPLACE "  \${POISSON_SOURCES}\n" "" _surface_patched "${_surface_patched}")

if(_surface_patched STREQUAL _surface_content)
    message(FATAL_ERROR "PCL surface patch did not match: ${_surface_cmake}")
endif()

file(WRITE "${_surface_cmake}" "${_surface_patched}")
