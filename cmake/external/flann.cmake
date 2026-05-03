# cmake/external/flann.cmake

include_guard(GLOBAL)
include(ExternalProject)

set(FLANN_VERSION 1.9.2)

if(NOT DEFINED FLANN_URL)
    set(FLANN_URL
        https://github.com/flann-lib/flann/archive/refs/tags/${FLANN_VERSION}.tar.gz)
endif()
if(NOT DEFINED FLANN_URL_HASH)
    # Verified against CMake's actual download report.
    set(FLANN_URL_HASH
        SHA256=e26829bb0017f317d9cc45ab83ddcb8b16d75ada1ae07157006c1e7d601c8824)
endif()

function(_flann_resolve_ndk OUT_VAR)
    set(_ndk "$ENV{ANDROID_NDK_HOME}")
    if(_ndk STREQUAL "")
        set(_ndk "$ENV{ANDROID_NDK}")
    endif()
    if(_ndk STREQUAL "")
        message(FATAL_ERROR "[flann] ANDROID_NDK_HOME (or ANDROID_NDK) must be set.")
    endif()
    set(${OUT_VAR} "${_ndk}" PARENT_SCOPE)
endfunction()

function(crosscompile_flann TAG)
    set(_dest "${install_prefix}/flann-${TAG}")
    set(_target_name flann-${TAG})

    set(_extra_cmake_args)
    if(TAG MATCHES "^android-(.+)$")
        pcl_mobile_android_platform(_android_platform _android_api_level)
        _flann_resolve_ndk(_ndk)
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

    # Where the per-slice LZ4 cross-build lives (see crosscompile_lz4()).
    set(_lz4_dest "${install_prefix}/lz4-${TAG}")
    set(_lz4_include "${_lz4_dest}/include")
    set(_lz4_lib     "${_lz4_dest}/lib/liblz4.a")

    ExternalProject_Add(${_target_name}
        PREFIX            ${base}/${_target_name}
        URL               ${FLANN_URL}
        URL_HASH          ${FLANN_URL_HASH}
        # FLANN 1.9.2 hard-requires PkgConfig + system liblz4. We supply LZ4
        # ourselves (cmake/external/lz4.cmake), so strip those calls out of
        # FLANN's CMakeLists.txt before configuring.
        DEPENDS           lz4-${TAG}
        PATCH_COMMAND
            ${CMAKE_COMMAND}
                -DFLANN_CMAKELISTS=<SOURCE_DIR>/CMakeLists.txt
                -P ${CMAKE_SOURCE_DIR}/cmake/external/flann-patch.cmake
        CMAKE_ARGS
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5
            -DCMAKE_INSTALL_PREFIX=${_dest}
            -DCMAKE_BUILD_TYPE=Release
            -DBUILD_C_BINDINGS=OFF
            -DBUILD_PYTHON_BINDINGS=OFF
            -DBUILD_MATLAB_BINDINGS=OFF
            -DBUILD_EXAMPLES=OFF
            -DBUILD_TESTS=OFF
            -DBUILD_DOC=OFF
            -DBUILD_SHARED_LIBS=OFF
            -DBUILD_CUDA_LIB=OFF
            # LZ4 wiring -- consumed by the patched CMakeLists.txt.
            -DLZ4_INCLUDE_DIRS=${_lz4_include}
            -DLZ4_LIBRARIES=${_lz4_lib}
            -DLZ4_LINK_LIBRARIES=${_lz4_lib}
            ${_extra_cmake_args}
        LOG_DOWNLOAD      ON
        LOG_CONFIGURE     ON
        LOG_BUILD         ON
    )
endfunction()
