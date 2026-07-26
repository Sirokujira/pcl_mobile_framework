# Native side of the :pclmobile AAR module.
#
# Consolidates AndroidWrapper/CMakeLists.txt and the AAR module's own
# CMakeLists.txt, both inherited from Sirokujira/pcl-superbuild@83cc231. See docs/LINEAGE.md.
#
# This file used to declare 14 individual add_library(lib_pclN ...) targets.
# It is now a foreach() loop over the PCL/Boost components we depend on so
# that adding/removing a component is a one-line change.
#
# See ../../../MODERNIZATION_PLAN.md for the bigger picture.

cmake_minimum_required(VERSION 3.24)

project(pclmobile_jni LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

message(STATUS "[pclmobile] CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "[pclmobile] ANDROID_ABI               = ${ANDROID_ABI}")

# ---------------------------------------------------------------------------
# Pre-built dependencies (PCL/Boost/Eigen/FLANN/Qhull) are unpacked under
# ${CMAKE_CURRENT_SOURCE_DIR}/libs/<ANDROID_ABI>/<modulename>/{include,lib}.
# scripts/build_android.sh produces this layout from the cross-compiled output
# of the superbuild at the repo root.
# ---------------------------------------------------------------------------
set(PCL_PREBUILT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/libs/${ANDROID_ABI}")

set(PCL_ROOT   "${PCL_PREBUILT_ROOT}/pcl-android")
set(BOOST_ROOT "${PCL_PREBUILT_ROOT}/boost-android")
set(FLANN_ROOT "${PCL_PREBUILT_ROOT}/flann-android")
set(QHULL_ROOT "${PCL_PREBUILT_ROOT}/qhull-android")
set(LZ4_ROOT   "${PCL_PREBUILT_ROOT}/lz4-android")
set(EIGEN_ROOT "${PCL_PREBUILT_ROOT}/eigen")

set(PCL_INCLUDE_DIRS   "${PCL_ROOT}/include")
set(BOOST_INCLUDE_DIRS "${BOOST_ROOT}/include")
set(FLANN_INCLUDE_DIRS "${FLANN_ROOT}/include")
set(QHULL_INCLUDE_DIRS "${QHULL_ROOT}/include" "${QHULL_ROOT}/include/libqhull_r")
set(LZ4_INCLUDE_DIRS   "${LZ4_ROOT}/include")
set(EIGEN_INCLUDE_DIRS "${EIGEN_ROOT}/include/eigen3")

# ---------------------------------------------------------------------------
# Helper: declare an IMPORTED static library that lives in the prebuilt root.
# ---------------------------------------------------------------------------
function(pclmobile_import_static target file)
    if(TARGET ${target})
        return()
    endif()
    if(NOT EXISTS "${file}")
        message(FATAL_ERROR
            "[pclmobile] Missing prebuilt library for ${ANDROID_ABI}: ${file}\n"
            "Run scripts/build_android.sh before building the AAR.")
    endif()
    add_library(${target} STATIC IMPORTED)
    set_target_properties(${target} PROPERTIES IMPORTED_LOCATION "${file}")
endfunction()

function(pclmobile_require_dir label dir)
    if(NOT IS_DIRECTORY "${dir}")
        message(FATAL_ERROR
            "[pclmobile] Missing ${label} for ${ANDROID_ABI}: ${dir}\n"
            "Run scripts/build_android.sh before building the AAR.")
    endif()
endfunction()

pclmobile_require_dir("PCL prebuilt root" "${PCL_PREBUILT_ROOT}")
pclmobile_require_dir("PCL include directory" "${PCL_INCLUDE_DIRS}")
pclmobile_require_dir("Boost include directory" "${BOOST_INCLUDE_DIRS}")
pclmobile_require_dir("Eigen include directory" "${EIGEN_INCLUDE_DIRS}")
pclmobile_require_dir("FLANN include directory" "${FLANN_INCLUDE_DIRS}")
pclmobile_require_dir("Qhull include directory" "${QHULL_ROOT}/include")
pclmobile_require_dir("LZ4 include directory" "${LZ4_INCLUDE_DIRS}")

# PCL components used by the JNI wrapper implementation.
set(PCL_COMPONENTS
    recognition tracking segmentation registration keypoints surface
    features filters sample_consensus search kdtree octree io io_ply
    ml common
)
foreach(comp IN LISTS PCL_COMPONENTS)
    pclmobile_import_static(pcl::${comp} "${PCL_ROOT}/lib/libpcl_${comp}.a")
endforeach()

set(BOOST_COMPONENTS filesystem system thread program_options iostreams date_time regex)
foreach(comp IN LISTS BOOST_COMPONENTS)
    pclmobile_import_static(boost::${comp} "${BOOST_ROOT}/lib/libboost_${comp}.a")
endforeach()

pclmobile_import_static(flann::cpp_s "${FLANN_ROOT}/lib/libflann_cpp_s.a")
pclmobile_import_static(lz4::lz4 "${LZ4_ROOT}/lib/liblz4.a")
# Qhull and qhull_r are both shipped by recent PCL; link the reentrant variant.
pclmobile_import_static(qhull::r "${QHULL_ROOT}/lib/libqhullstatic_r.a")

if(ANDROID)
    if(ANDROID_ABI STREQUAL "arm64-v8a")
        set(PCLMOBILE_OMP_ARCH "aarch64")
    elseif(ANDROID_ABI STREQUAL "armeabi-v7a")
        set(PCLMOBILE_OMP_ARCH "arm")
    elseif(ANDROID_ABI STREQUAL "x86_64")
        set(PCLMOBILE_OMP_ARCH "x86_64")
    endif()

    if(PCLMOBILE_OMP_ARCH)
        file(GLOB PCLMOBILE_OMP_LIBS
            "${ANDROID_NDK}/toolchains/llvm/prebuilt/*/lib/clang/*/lib/linux/${PCLMOBILE_OMP_ARCH}/libomp.a"
        )
        if(PCLMOBILE_OMP_LIBS)
            list(SORT PCLMOBILE_OMP_LIBS)
            list(LENGTH PCLMOBILE_OMP_LIBS PCLMOBILE_OMP_LIB_COUNT)
            if(PCLMOBILE_OMP_LIB_COUNT GREATER 1)
                message(WARNING
                    "[pclmobile] Multiple libomp.a candidates found for ${ANDROID_ABI}; "
                    "using the latest sorted path.")
            endif()
            list(GET PCLMOBILE_OMP_LIBS -1 PCLMOBILE_OMP_LIB)
            pclmobile_import_static(openmp::omp "${PCLMOBILE_OMP_LIB}")
        else()
            message(FATAL_ERROR
                "[pclmobile] Missing Android NDK OpenMP runtime libomp.a "
                "for ${ANDROID_ABI} (${PCLMOBILE_OMP_ARCH}) under ${ANDROID_NDK}.")
        endif()
    else()
        message(FATAL_ERROR "[pclmobile] Unsupported Android ABI for OpenMP: ${ANDROID_ABI}")
    endif()
endif()

# ---------------------------------------------------------------------------
# The JNI shared object that gets packaged inside the AAR.
# ---------------------------------------------------------------------------
add_library(pclmobile-jni SHARED
    src/main/cpp/native-lib.cpp
    src/main/cpp/pcl_mobile_arrays.cpp
    src/main/cpp/pcl_mobile_context.cpp
    src/main/cpp/pcl_mobile_features.cpp
    src/main/cpp/pcl_mobile_filters.cpp
    src/main/cpp/pcl_mobile_io.cpp
    src/main/cpp/pcl_mobile_keypoints.cpp
    src/main/cpp/pcl_mobile_range_image.cpp
    src/main/cpp/pcl_mobile_registration.cpp
    src/main/cpp/pcl_mobile_search.cpp
    src/main/cpp/pcl_mobile_segmentation.cpp
    src/main/cpp/pcl_mobile_surface.cpp
)

target_include_directories(pclmobile-jni PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${BOOST_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
    ${QHULL_INCLUDE_DIRS}
    ${LZ4_INCLUDE_DIRS}
)

# Backwards-compatible alias so System.loadLibrary("native-lib") still works
# until the JNI side is renamed in a future release.
set_target_properties(pclmobile-jni PROPERTIES OUTPUT_NAME "native-lib")

find_library(log-lib log)

target_link_libraries(pclmobile-jni
    PRIVATE
        ${log-lib}
)
foreach(comp IN LISTS PCL_COMPONENTS)
    target_link_libraries(pclmobile-jni PRIVATE pcl::${comp})
endforeach()
foreach(comp IN LISTS BOOST_COMPONENTS)
    target_link_libraries(pclmobile-jni PRIVATE boost::${comp})
endforeach()
target_link_libraries(pclmobile-jni PRIVATE flann::cpp_s lz4::lz4 qhull::r)
if(TARGET openmp::omp)
    target_link_libraries(pclmobile-jni PRIVATE openmp::omp)
endif()
