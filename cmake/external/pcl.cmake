# cmake/external/pcl.cmake
#
# Successor to fetch_pcl()/crosscompile_pcl() in Sirokujira/pcl-superbuild@83cc231 —
# external-project-macros.cmake. See docs/LINEAGE.md.
#
# Cross-compile PCL 1.14 against the slice-local Boost/Eigen/FLANN/Qhull.

include_guard(GLOBAL)
include(ExternalProject)

set(PCL_VERSION 1.14.1)

if(NOT DEFINED PCL_URL)
    set(PCL_URL
        https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-${PCL_VERSION}.tar.gz)
endif()
if(NOT DEFINED PCL_URL_HASH)
    # Verified against CMake's actual download report.
    set(PCL_URL_HASH
        SHA256=5dc5e09509644f703de9a3fb76d99ab2cc67ef53eaf5637db2c6c8b933b28af6)
endif()

# Top-level PCL components shipped on mobile. Visualization / surface_on_nurbs
# / people / gpu / cuda are excluded because they pull dependencies (VTK,
# OpenGL ES extras, CUDA) that don't make sense on mobile.
set(PCL_BUILD_MODULES
    common io kdtree search octree sample_consensus filters features
    keypoints registration segmentation surface tracking recognition ml
)

# PCL 1.14 module names (as used by `BUILD_<module>` cache vars). These are
# the ones we explicitly want OFF on mobile. Names must match exactly --
# unknown names get silently ignored, which is how we ended up shipping
# `BUILD_io_ply` etc. that did nothing.
set(PCL_DISABLE_MODULES
    visualization
    people
    outofcore
    stereo
    tools
    apps
    examples
)

function(_pcl_resolve_ndk OUT_VAR)
    set(_ndk "$ENV{ANDROID_NDK_HOME}")
    if(_ndk STREQUAL "")
        set(_ndk "$ENV{ANDROID_NDK}")
    endif()
    if(_ndk STREQUAL "")
        message(FATAL_ERROR "[pcl] ANDROID_NDK_HOME (or ANDROID_NDK) must be set.")
    endif()
    set(${OUT_VAR} "${_ndk}" PARENT_SCOPE)
endfunction()

function(_pcl_target_args TAG OUT_VAR)
    set(_args)
    if(TAG MATCHES "^android-(.+)$")
        pcl_mobile_android_platform(_android_platform _android_api_level)
        _pcl_resolve_ndk(_ndk)
        list(APPEND _args
            -DCMAKE_TOOLCHAIN_FILE=${_ndk}/build/cmake/android.toolchain.cmake
            -DANDROID_ABI=${CMAKE_MATCH_1}
            -DANDROID_PLATFORM=${_android_platform}
            -DANDROID_STL=c++_shared
        )
    elseif(TAG MATCHES "^(ios|iossim)-(.+)$")
        list(APPEND _args
            -DCMAKE_TOOLCHAIN_FILE=${CMAKE_SOURCE_DIR}/cmake/toolchains/ios.toolchain.cmake
        )
        if(TAG STREQUAL "ios-arm64")
            list(APPEND _args -DPLATFORM=OS64)
        elseif(TAG STREQUAL "iossim-arm64")
            list(APPEND _args -DPLATFORM=SIMULATORARM64)
        elseif(TAG STREQUAL "iossim-x86_64")
            list(APPEND _args -DPLATFORM=SIMULATOR64)
        endif()
    endif()
    set(${OUT_VAR} "${_args}" PARENT_SCOPE)
endfunction()

function(crosscompile_pcl TAG)
    set(_dest "${install_prefix}/pcl-${TAG}")
    set(_target_name pcl-${TAG})

    set(_boost_dest  "${install_prefix}/boost-${TAG}")
    set(_eigen_dest  "${install_prefix}/eigen")
    set(_flann_dest  "${install_prefix}/flann-${TAG}")
    set(_qhull_dest  "${install_prefix}/qhull-${TAG}")
    set(_lz4_dest    "${install_prefix}/lz4-${TAG}")

    _pcl_target_args(${TAG} _slice_args)

    # Module on/off flags. Use the canonical PCL_BUILD_MODULES /
    # PCL_DISABLE_MODULES lists -- nothing in here is invented.
    set(_disable_modules)
    foreach(mod IN LISTS PCL_DISABLE_MODULES)
        list(APPEND _disable_modules -DBUILD_${mod}=OFF)
    endforeach()

    set(_enable_modules)
    foreach(mod IN LISTS PCL_BUILD_MODULES)
        list(APPEND _enable_modules -DBUILD_${mod}=ON)
    endforeach()

    # PCL probes runtime-only behaviour with try_run() (e.g. posix_memalign,
    # backtrace) which can't execute when cross-compiling. Pre-seed those
    # cache entries from cmake/toolchains/pcl-try-run-results.cmake.
    set(_try_run_cache_args)
    set(_try_run_file ${CMAKE_SOURCE_DIR}/cmake/toolchains/pcl-try-run-results.cmake)
    if(EXISTS "${_try_run_file}")
        # ExternalProject_Add doesn't accept INITIAL_CACHE, so re-emit the
        # try_run results as -D arguments. Today the file only carries
        # HAVE_POSIX_MEMALIGN_EXITCODE; if more get added, extend here.
        list(APPEND _try_run_cache_args
            -DHAVE_POSIX_MEMALIGN_EXITCODE=0
            -DHAVE_POSIX_MEMALIGN_EXITCODE__TRYRUN_OUTPUT=
        )
    endif()

    # Make every dependency's *Config.cmake findable by PCL's find_package()
    # calls (Eigen3, FLANN, Boost, Qhull all live under their own slice
    # install prefix). CMAKE_PREFIX_PATH covers Boost/FLANN/Qhull;
    # Eigen3_DIR is the canonical override for Eigen3's config file.
    #
    # CMAKE_PREFIX_PATH is a CMake list (semicolon-separated). Inside
    # ExternalProject_Add CMAKE_ARGS we have to join with the LIST_SEPARATOR
    # we declare below ('|'); ExternalProject then puts the original ';'
    # back when invoking the sub-cmake.
    set(_prefix_path
        "${_eigen_dest}"
        "${_boost_dest}"
        "${_flann_dest}"
        "${_qhull_dest}"
        "${_lz4_dest}")
    list(JOIN _prefix_path "|" _prefix_path_joined)

    # Boost layout (b2 default):     ${_boost_dest}/{include,lib}
    # FLANN layout (cmake install):  ${_flann_dest}/{include,lib}
    # Qhull v8.0.2 ships several static libs; the reentrant variant is what
    # FindQhull.cmake (PCL-bundled) actually wants.
    set(_boost_inc "${_boost_dest}/include")
    set(_boost_lib "${_boost_dest}/lib")
    set(_flann_inc "${_flann_dest}/include")
    set(_flann_lib "${_flann_dest}/lib/libflann_cpp_s.a")
    set(_lz4_inc "${_lz4_dest}/include")
    set(_lz4_lib "${_lz4_dest}/lib/liblz4.a")
    set(_qhull_inc "${_qhull_dest}/include")
    set(_qhull_header "${_qhull_inc}/libqhull_r/libqhull_r.h")
    set(_qhull_lib "${_qhull_dest}/lib/libqhullstatic_r.a")

    ExternalProject_Add(${_target_name}
        PREFIX            ${base}/${_target_name}
        URL               ${PCL_URL}
        URL_HASH          ${PCL_URL_HASH}
        PATCH_COMMAND     ${CMAKE_COMMAND}
            -DPCL_SOURCE_DIR=<SOURCE_DIR>
            -P ${CMAKE_SOURCE_DIR}/cmake/patches/pcl-1.14.1-mobile.cmake
        DEPENDS
            eigen
            boost-${TAG}
            flann-${TAG}
            qhull-${TAG}
        # Use LIST_SEPARATOR so the semicolons inside CMAKE_PREFIX_PATH are
        # preserved (otherwise ExternalProject splits on `;` thinking each
        # piece is a separate flag).
        LIST_SEPARATOR    "|"
        CMAKE_ARGS
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5
            # CMP0144 NEW makes find_package() actually honor BOOST_ROOT,
            # FLANN_ROOT and QHULL_ROOT instead of ignoring them with a
            # backward-compat warning (CMake 3.27+ tightened this).
            -DCMAKE_POLICY_DEFAULT_CMP0144=NEW
            # CMP0167 OLD keeps the legacy FindBoost.cmake module path.
            # Boost installed via b2 does NOT ship BoostConfig.cmake, so
            # NEW (Config-only) cannot work for us.
            -DCMAKE_POLICY_DEFAULT_CMP0167=OLD
            -DCMAKE_INSTALL_PREFIX=${_dest}
            -DCMAKE_BUILD_TYPE=Release
            -DCMAKE_C_FLAGS=-I${_lz4_inc}
            -DCMAKE_CXX_FLAGS=-I${_lz4_inc}
            -DBUILD_SHARED_LIBS=OFF
            -DPCL_SHARED_LIBS=OFF
            -DBUILD_tests=OFF
            -DBUILD_examples=OFF
            -DBUILD_apps=OFF
            -DPCL_ENABLE_SSE=OFF
            -DWITH_OPENGL=OFF
            -DWITH_QT=OFF
            -DWITH_VTK=OFF
            -DWITH_PCAP=OFF
            -DWITH_PNG=OFF
            -DWITH_LIBUSB=OFF
            -DWITH_OPENNI=OFF
            -DWITH_OPENNI2=OFF
            -DWITH_ENSENSO=OFF
            -DWITH_DAVIDSDK=OFF
            -DWITH_DSSDK=OFF
            -DWITH_RSSDK=OFF
            -DWITH_RSSDK2=OFF
            -DWITH_CUDA=OFF
            -DWITH_QHULL=ON
            -DCMAKE_PREFIX_PATH=${_prefix_path_joined}
            # Eigen3 ----------------------------------------------------
            -DEigen3_DIR=${_eigen_dest}/share/eigen3/cmake
            -DEIGEN_INCLUDE_DIR=${_eigen_dest}/include/eigen3
            # Boost -- both upper- and mixed-case ROOT plus explicit
            # include/lib hints because the CMake 4 FindBoost stub doesn't
            # always derive them on its own. NO_BOOST_CMAKE forces module
            # mode (we have no BoostConfig.cmake).
            -DBoost_NO_BOOST_CMAKE=ON
            -DBoost_NO_SYSTEM_PATHS=ON
            -DBoost_USE_STATIC_LIBS=ON
            -DBOOST_ROOT=${_boost_dest}
            -DBoost_ROOT=${_boost_dest}
            -DBoost_INCLUDE_DIR=${_boost_inc}
            -DBoost_LIBRARY_DIRS=${_boost_lib}
            -DBOOST_INCLUDEDIR=${_boost_inc}
            -DBOOST_LIBRARYDIR=${_boost_lib}
            -DBoost_FILESYSTEM_LIBRARY_RELEASE=${_boost_lib}/libboost_filesystem.a
            -DBoost_FILESYSTEM_LIBRARY_DEBUG=${_boost_lib}/libboost_filesystem.a
            -DBoost_IOSTREAMS_LIBRARY_RELEASE=${_boost_lib}/libboost_iostreams.a
            -DBoost_IOSTREAMS_LIBRARY_DEBUG=${_boost_lib}/libboost_iostreams.a
            -DBoost_SYSTEM_LIBRARY_RELEASE=${_boost_lib}/libboost_system.a
            -DBoost_SYSTEM_LIBRARY_DEBUG=${_boost_lib}/libboost_system.a
            -DBoost_REGEX_LIBRARY_RELEASE=${_boost_lib}/libboost_regex.a
            -DBoost_REGEX_LIBRARY_DEBUG=${_boost_lib}/libboost_regex.a
            -DBoost_THREAD_LIBRARY_RELEASE=${_boost_lib}/libboost_thread.a
            -DBoost_THREAD_LIBRARY_DEBUG=${_boost_lib}/libboost_thread.a
            -DBoost_PROGRAM_OPTIONS_LIBRARY_RELEASE=${_boost_lib}/libboost_program_options.a
            -DBoost_PROGRAM_OPTIONS_LIBRARY_DEBUG=${_boost_lib}/libboost_program_options.a
            -DBoost_DATE_TIME_LIBRARY_RELEASE=${_boost_lib}/libboost_date_time.a
            -DBoost_DATE_TIME_LIBRARY_DEBUG=${_boost_lib}/libboost_date_time.a
            # FLANN -----------------------------------------------------
            -DPCL_FLANN_REQUIRED_TYPE=STATIC
            -DFLANN_ROOT=${_flann_dest}
            -DFlann_ROOT=${_flann_dest}
            -DFLANN_INCLUDE_DIR=${_flann_inc}
            -DFLANN_LIBRARY=${_flann_lib}
            -DFLANN_LIBRARY_STATIC=${_flann_lib}
            # Qhull -----------------------------------------------------
            -DPCL_QHULL_REQUIRED_TYPE=STATIC
            -DQHULL_ROOT=${_qhull_dest}
            -DQhull_ROOT=${_qhull_dest}
            -DQHULL_HEADER=${_qhull_header}
            -DQHULL_INCLUDE_DIR=${_qhull_inc}
            -DQHULL_LIBRARY=${_qhull_lib}
            -DQHULL_LIBRARY_STATIC=${_qhull_lib}
            ${_enable_modules}
            ${_disable_modules}
            ${_try_run_cache_args}
            ${_slice_args}
        LOG_DOWNLOAD      ON
        LOG_CONFIGURE     ON
        LOG_BUILD         ON
    )
endfunction()
