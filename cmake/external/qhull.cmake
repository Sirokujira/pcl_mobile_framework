# cmake/external/qhull.cmake

include_guard(GLOBAL)
include(ExternalProject)

set(QHULL_VERSION 2020.2)

if(NOT DEFINED QHULL_URL)
    set(QHULL_URL
        https://github.com/qhull/qhull/archive/refs/tags/v8.0.2.tar.gz)
endif()
if(NOT DEFINED QHULL_URL_HASH)
    # NOTE: placeholder; replace with the real SHA256 once verified.
    set(QHULL_URL_HASH
        SHA256=8774e9a12c70b0180b95d6b0b563c5aa4bea8d5960c15e18ae3b6d2521d64f8b)
endif()

function(_qhull_resolve_ndk OUT_VAR)
    set(_ndk "$ENV{ANDROID_NDK_HOME}")
    if(_ndk STREQUAL "")
        set(_ndk "$ENV{ANDROID_NDK}")
    endif()
    if(_ndk STREQUAL "")
        message(FATAL_ERROR "[qhull] ANDROID_NDK_HOME (or ANDROID_NDK) must be set.")
    endif()
    set(${OUT_VAR} "${_ndk}" PARENT_SCOPE)
endfunction()

function(crosscompile_qhull TAG)
    set(_dest "${install_prefix}/qhull-${TAG}")
    set(_target_name qhull-${TAG})

    set(_extra_cmake_args)
    if(TAG MATCHES "^android-(.+)$")
        pcl_mobile_android_platform(_android_platform _android_api_level)
        _qhull_resolve_ndk(_ndk)
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

    ExternalProject_Add(${_target_name}
        PREFIX            ${base}/${_target_name}
        URL               ${QHULL_URL}
        URL_HASH          ${QHULL_URL_HASH}
        CMAKE_ARGS
            # qhull's CMakeLists.txt declares cmake_minimum_required(3.4),
            # which CMake 4 deprecates. This override keeps the configure
            # step quiet without patching the upstream tarball.
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5
            # qhull always builds qhull/qconvex/etc. CLI tools; on iOS
            # CMake auto-promotes them to MACOSX_BUNDLE which then trips
            # `install(TARGETS qhull ...)` for missing BUNDLE DESTINATION.
            # Force-default the property to OFF here as well as in the
            # toolchain (belt + suspenders against qhull's missing
            # BUILD_APPLICATIONS option).
            -DCMAKE_MACOSX_BUNDLE=OFF
            -DCMAKE_INSTALL_PREFIX=${_dest}
            -DCMAKE_BUILD_TYPE=Release
            -DBUILD_SHARED_LIBS=OFF
            -DBUILD_STATIC_LIBS=ON
            ${_extra_cmake_args}
        LOG_DOWNLOAD      ON
        LOG_CONFIGURE     ON
        LOG_BUILD         ON
    )
endfunction()
