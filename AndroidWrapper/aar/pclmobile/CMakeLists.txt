# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

### external Native Settings ###
# root path setting
message(STATUS ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS ${ANDROID_ABI})
#set (BASE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/libs)
set (BASE_ROOT "/Users/to/pcl_mobile_framework/AndroidWrapper/aar/pclmobile/libs")

set (PCL_ROOT ${BASE_ROOT}/${ANDROID_ABI}/pcl-android)
set (PCL_DIR ${BASE_ROOT}/${ANDROID_ABI}/pcl-android)
#find_package(PCL REQUIRED)
#message(STATUS ${PCL_FOUND})
#message(STATUS ${PCL_LIBRARIES})
#message(STATUS ${PCL_COMPONENTS})
#message(STATUS ${PCL_INCLUDE_DIRS})
#message(STATUS ${PCL_LIBRARY_DIRS})
#message(STATUS ${PCL_DEFINITIONS})

set (BOOST_ROOT ${BASE_ROOT}/${ANDROID_ABI}/boost-android)
set (BOOST_INCLUDEDIR ${BOOST_ROOT}/include)
set (FLANN_ROOT ${BASE_ROOT}/${ANDROID_ABI}/flann-android)
set (FLANN_USE_STATIC TRUE)
set (EIGEN_ROOT "${BASE_ROOT}/${ANDROID_ABI}/Eigen" "${BASE_ROOT}/${ANDROID_ABI}/unsupported/Eigen")
set (Eigen_DIR "${BASE_ROOT}/${ANDROID_ABI}/eigen")
set (Eigen3_DIR "${BASE_ROOT}/${ANDROID_ABI}/eigen")
set (EIGEN_INCLUDE_DIR "${BASE_ROOT}/${ANDROID_ABI}/eigen")
set (QHULL_ROOT "${BASE_ROOT}/${ANDROID_ABI}/qhull-android")

#PCLConfig.cmake
list(APPEND CMAKE_MODULE_PATH ${PCL_ROOT}/share/pcl-1.9)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PCL_ROOT}/share/pcl-1.9/Modules)

# boost
# find_package(boost REQUIRED)
set (Boost_INCLUDE_DIRS ${BOOST_ROOT}/include)
set (Boost_LIBRARIES_DIRS ${BOOST_ROOT}/lib)

# flann
# priority using pkg-config?
# find_package(FLANN REQUIRED)
set (Flann_INCLUDE_DIRS ${FLANN_ROOT}/include)
set (Flann_LIBRARIES_DIRS ${FLANN_ROOT}/lib)
include_directories(${Flann_INCLUDE_DIRS})

# Eigen
# Folder : Eigen, unsupported/Eigen
#find_package (Eigen3 3.3 REQUIRED NO_MODULE)
#add_executable (example example.cpp)
#target_link_libraries (example Eigen3::Eigen)
#set (EIGEN_INCLUDE_DIRS ${EIGEN_ANRDROID_ROOT}/Eigen)
# set (EIGEN_INCLUDE_DIRS ${EIGEN_ANRDROID_ROOT}/unsupported/Eigen)
include_directories(${EIGEN_ANRDROID_ROOT})
#include_directories(${EIGEN_INCLUDE_DIRS})

# QHull
# Folder : libqhull, libqhull_r, libqhullcpp
set (QHull_INCLUDE_DIRS ${QHull_ROOT}/libqhull ${QHull_ROOT}/libqhull)
set (QHull_LIBRARIES_DIRS ${QHull_ROOT}/lib)
# include_directories(${QHull_INCLUDE_DIRS})

# pcl
#find_package(PCL 1.9 REQUIRED)
set (PCL_INCLUDE_DIRS ${PCL_ROOT}/include)
set (PCL_LIBRARIES_DIRS ${PCL_ROOT}/lib)
# include_directories(${PCL_INCLUDE_DIRS})


# library folder path
link_directories (${Boost_LIBRARIES_DIRS} ${Flann_LIBRARIES_DIRS} ${Qhull_LIBRARIES_DIRS} ${PCL_LIBRARIES_DIRS})

###

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.

add_library( # Sets the name of the library.
        native-lib

        # Sets the library as a shared library.
        SHARED
        # STATIC

        # Provides a relative path to your source file(s).
        src/main/cpp/native-lib.cpp)

target_include_directories(native-lib PRIVATE
              ${Boost_INCLUDE_DIRS}
              ${FLANN_INCLUDE_DIRS}
              ${EIGEN_INCLUDE_DIRS}
              ${QHULL_INCLUDE_DIRS}
              ${PCL_INCLUDE_DIRS}
              # ${GLM_INCLUDE_DIRS}
              )

# Searches for a specified prebuilt library and stores the path as a
# variable. Because CMake includes system libraries in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library( # Sets the name of the path variable.
        log-lib

        # Specifies the name of the NDK library that
        # you want CMake to locate.
        log)

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in this
# build script, prebuilt third-party libraries, or system libraries.

add_library(lib_pcl STATIC IMPORTED)
add_library(lib_pcl2 STATIC IMPORTED)
add_library(lib_pcl3 STATIC IMPORTED)
add_library(lib_pcl4 STATIC IMPORTED)
add_library(lib_pcl5 STATIC IMPORTED)
add_library(lib_pcl6 STATIC IMPORTED)
add_library(lib_pcl7 STATIC IMPORTED)
add_library(lib_pcl8 STATIC IMPORTED)
add_library(lib_pcl9 STATIC IMPORTED)
add_library(lib_pcl10 STATIC IMPORTED)
add_library(lib_pcl11 STATIC IMPORTED)
add_library(lib_pcl12 STATIC IMPORTED)
add_library(lib_pcl13 STATIC IMPORTED)
add_library(lib_pcl14 STATIC IMPORTED)
set_target_properties(lib_pcl PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_common.a)
set_target_properties(lib_pcl2 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_keypoints.a)
set_target_properties(lib_pcl3 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_segmentation.a)
set_target_properties(lib_pcl4 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_features.a)
set_target_properties(lib_pcl5 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_octree.a)
set_target_properties(lib_pcl6 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_surface.a)
set_target_properties(lib_pcl7 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_filters.a)
set_target_properties(lib_pcl8 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_recognition.a)
set_target_properties(lib_pcl9 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_io.a)
set_target_properties(lib_pcl10 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_registration.a)
set_target_properties(lib_pcl11 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_io_ply.a)
set_target_properties(lib_pcl12 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_sample_consensus.a)
set_target_properties(lib_pcl13 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_kdtree.a)
set_target_properties(lib_pcl14 PROPERTIES IMPORTED_LOCATION ${PCL_ROOT}/lib/libpcl_search.a)
add_library(lib_boost STATIC IMPORTED)
set_target_properties(lib_boost PROPERTIES IMPORTED_LOCATION ${BOOST_ROOT}/lib/libboost_filesystem.a)
add_library(lib_boost2 STATIC IMPORTED)
add_library(lib_boost3 STATIC IMPORTED)
add_library(lib_boost4 STATIC IMPORTED)
add_library(lib_boost5 STATIC IMPORTED)
add_library(lib_boost6 STATIC IMPORTED)
set_target_properties(lib_boost2 PROPERTIES IMPORTED_LOCATION ${BOOST_ROOT}/lib/libboost_system.a)
set_target_properties(lib_boost3 PROPERTIES IMPORTED_LOCATION ${BOOST_ROOT}/lib/libboost_thread.a)
set_target_properties(lib_boost4 PROPERTIES IMPORTED_LOCATION ${BOOST_ROOT}/lib/libboost_program_options.a)
set_target_properties(lib_boost5 PROPERTIES IMPORTED_LOCATION ${BOOST_ROOT}/lib/libboost_iostreams.a)
set_target_properties(lib_boost6 PROPERTIES IMPORTED_LOCATION ${BOOST_ROOT}/lib/libboost_date_time.a)
add_library(lib_flann_s STATIC IMPORTED)
set_target_properties(lib_flann_s PROPERTIES IMPORTED_LOCATION ${FLANN_ROOT}/lib/libflann_cpp_s.a)

# add_definitions(${FLANN_DEFINITIONS} ${QHULL_DEFINITIONS})

target_link_libraries( # Specifies the target library.
        native-lib

        # external libraries
        # pcl_common pcl_keypoints pcl_segmentation pcl_features pcl_octree pcl_surface pcl_filters pcl_recognition pcl_io pcl_registration pcl_io_ply pcl_sample_consensus pcl_kdtree pcl_search
        # boost_filesystem boost_system boost_thread boost_program_options boost_iostreams boost_date_time
        # flann_cpp_s
        # pcl_common.a pcl_keypoints.a pcl_segmentation.a pcl_features.a pcl_octree.a pcl_surface.a pcl_filters.a pcl_recognition.a pcl_io.a pcl_registration.a pcl_io_ply.a pcl_sample_consensus.a pcl_kdtree.a pcl_search.a
        # boost_filesystem.a boost_system.a boost_thread.a boost_program_options.a boost_iostreams.a boost_date_time.a
        # flann_cpp_s.a
        lib_pcl
        lib_pcl2
        lib_pcl3
        lib_pcl4
        lib_pcl5
        lib_pcl6
        lib_pcl7
        lib_pcl8
        lib_pcl9
        lib_pcl10
        lib_pcl11
        lib_pcl12
        lib_pcl13
        lib_pcl14
        lib_boost
        lib_boost2
        lib_boost3
        lib_boost4
        lib_boost5
        lib_boost6
        lib_flann_s
        # ${BOOST_LIBRARIES}
        # ${FLANN_LIBRARIES}
        # ${QHULL_LIBRARIES}
        # ${PCL_LIBRARIES}
        # Eigen3::Eigen

        # Links the target library to the log library
        # included in the NDK.
        ${log-lib})