# cmake/external/flann-patch.cmake
#
# Run as a `cmake -P` script during ExternalProject_Add's PATCH_COMMAND step.
# Removes flann 1.9.2's hard dependency on system `pkg-config` + `liblz4`
# discovery, so we can supply LZ4 paths via -D arguments instead.
#
# Idempotent: if the markers already exist, leaves the file alone.
#
# Usage:
#   cmake -DFLANN_CMAKELISTS=<path> -P flann-patch.cmake

if(NOT DEFINED FLANN_CMAKELISTS)
    message(FATAL_ERROR "Pass -DFLANN_CMAKELISTS=<absolute path>")
endif()
if(NOT EXISTS "${FLANN_CMAKELISTS}")
    message(FATAL_ERROR "Not a file: ${FLANN_CMAKELISTS}")
endif()

file(READ "${FLANN_CMAKELISTS}" _content)

if(_content MATCHES "PATCHED_BY_PCL_MOBILE_FRAMEWORK")
    message(STATUS "[flann-patch] already patched; nothing to do.")
    return()
endif()

string(REPLACE
    "find_package(PkgConfig REQUIRED)"
    "# PATCHED_BY_PCL_MOBILE_FRAMEWORK: replaced PkgConfig + pkg_check_modules
# with externally-supplied LZ4_INCLUDE_DIRS / LZ4_LINK_LIBRARIES.
# find_package(PkgConfig REQUIRED)"
    _content "${_content}")

string(REPLACE
    "pkg_check_modules(LZ4 REQUIRED liblz4)"
    "# pkg_check_modules(LZ4 REQUIRED liblz4)"
    _content "${_content}")

file(WRITE "${FLANN_CMAKELISTS}" "${_content}")
message(STATUS "[flann-patch] patched ${FLANN_CMAKELISTS}")
