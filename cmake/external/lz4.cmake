# cmake/external/lz4.cmake
#
# FLANN 1.9.2 hard-depends on LZ4 (its `serialization.h` does
# `#include <lz4.h>` and `#include <lz4hc.h>`). FLANN does NOT bundle LZ4,
# so we have to provide a cross-compiled liblz4 ourselves for every slice.
# This file does that.

include_guard(GLOBAL)
include(ExternalProject)

set(LZ4_VERSION 1.9.4)

if(NOT DEFINED LZ4_URL)
    set(LZ4_URL
        https://github.com/lz4/lz4/archive/refs/tags/v${LZ4_VERSION}.tar.gz)
endif()
if(NOT DEFINED LZ4_URL_HASH)
    # NOTE: placeholder; CMake prints the real SHA256 on first download
    # mismatch. Replace this constant with that value, or override at the
    # configure command line via -DLZ4_URL_HASH=SHA256=...
    set(LZ4_URL_HASH
        SHA256=0b0e3aa07c8c063ddf40b082bdf7e37a1562bda40a0ff5272957f3e987e0e54b)
endif()

function(_lz4_resolve_ndk OUT_VAR)
    set(_ndk "$ENV{ANDROID_NDK_HOME}")
    if(_ndk STREQUAL "")
        set(_ndk "$ENV{ANDROID_NDK}")
    endif()
    if(_ndk STREQUAL "")
        message(FATAL_ERROR "[lz4] ANDROID_NDK_HOME (or ANDROID_NDK) must be set.")
    endif()
    set(${OUT_VAR} "${_ndk}" PARENT_SCOPE)
endfunction()

function(crosscompile_lz4 TAG)
    set(_dest "${install_prefix}/lz4-${TAG}")
    set(_target_name lz4-${TAG})

    set(_extra_cmake_args)
    if(TAG MATCHES "^android-(.+)$")
        pcl_mobile_android_platform(_android_platform _android_api_level)
        _lz4_resolve_ndk(_ndk)
        list(APPEND _extra_cmake_args
            -DCMAKE_TOOLCHAIN_FILE=${_ndk}/build/cmake/android.toolchain.cmake
            -DANDROID_ABI=${CMAKE_MATCH_1}
            -DANDROID_PLATFORM=${_android_platform}
            -DANDROID_STL=c++_shared
        )
    elseif(TAG MATCHES "^(ios|iossim)-(.+)$")
        list(APPEND _extra_cmake_args
            -DCMAKE_TOOLCHAIN_FILE=${CMAKE_SOURCE_DIR}/cmake/toolchains/ios.toolchain.cmake
        )
        if(TAG STREQUAL "ios-arm64")
            list(APPEND _extra_cmake_args -DPLATFORM=OS64)
        elseif(TAG STREQUAL "iossim-arm64")
            list(APPEND _extra_cmake_args -DPLATFORM=SIMULATORARM64)
        elseif(TAG STREQUAL "iossim-x86_64")
            list(APPEND _extra_cmake_args -DPLATFORM=SIMULATOR64)
        endif()
    endif()

    # LZ4's CMakeLists lives at <SOURCE_DIR>/build/cmake (not at root).
    ExternalProject_Add(${_target_name}
        PREFIX            ${base}/${_target_name}
        URL               ${LZ4_URL}
        URL_HASH          ${LZ4_URL_HASH}
        SOURCE_SUBDIR     build/cmake
        CMAKE_ARGS
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5
            -DCMAKE_INSTALL_PREFIX=${_dest}
            -DCMAKE_BUILD_TYPE=Release
            -DBUILD_SHARED_LIBS=OFF
            -DBUILD_STATIC_LIBS=ON
            -DLZ4_BUILD_CLI=OFF
            -DLZ4_BUILD_LEGACY_LZ4C=OFF
            ${_extra_cmake_args}
        LOG_DOWNLOAD      ON
        LOG_CONFIGURE     ON
        LOG_BUILD         ON
    )
endfunction()
