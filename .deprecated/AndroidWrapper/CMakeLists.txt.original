# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)


message(${CMAKE_SOURCE_DIR})
message(${CMAKE_CURRENT_SOURCE_DIR})
message(${PROJECT_SOURCE_DIR})

# root path setting
set (BASE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../jniLibs)
set (BOOST_ANDROID_ROOT ${BASE_ROOT}/${ANDROID_ABI}/boost-android)
set (Flann_ANRDROIT_ROOT ${BASE_ROOT}/${ANDROID_ABI}/flann-android)
set (EIGEN_ANRDROIT_ROOT "${BASE_ROOT}/${ANDROID_ABI}/Eigen" "${BASE_ROOT}/${ANDROID_ABI}/unsupported/Eigen")
set (QHull_ANRDROIT_ROOT ${BASE_ROOT}/${ANDROID_ABI}/libqhull)
set (PCL_ANRDROIT_ROOT ${BASE_ROOT}/${ANDROID_ABI}/pcl-android)

# boost
set (Boost_INCLUDE_DIRS ${BOOST_ANDROID_ROOT}/include)
set (Boost_LIBRARIES_DIRS ${BOOST_ANDROID_ROOT}/lib)
include_directories(${Boost_INCLUDE_DIRS})

# flann
set (Flann_INCLUDE_DIRS ${Flann_ANRDROIT_ROOT}/include)
set (Flann_LIBRARIES_DIRS ${Flann_ANRDROIT_ROOT}/lib)
include_directories(${Flann_INCLUDE_DIRS})

# Eigen
# Folder : Eigen, unsupported/Eigen
set (EIGEN_INCLUDE_DIRS ${EIGEN_ANRDROIT_ROOT}/eigen)
# set (EIGEN_INCLUDE_DIRS ${EIGEN_ANRDROIT_ROOT}/unsupported/Eigen)
include_directories(${EIGEN_INCLUDE_DIRS})

# QHull
# Folder : libqhull, libqhull_r, libqhullcpp
set (QHull_INCLUDE_DIRS ${QHull_ANRDROIT_ROOT}/libqhull)
set (QHull_LIBRARIES_DIRS ${QHull_ANRDROIT_ROOT}/lib)
include_directories(${QHull_INCLUDE_DIRS})

# pcl
set (PCL_INCLUDE_DIRS ${PCL_ANRDROIT_ROOT}/include)
set (PCL_LIBRARIES_DIRS ${PCL_ANRDROIT_ROOT}/lib)
include_directories(${PCL_INCLUDE_DIRS})

# library folder path
link_directories (${Boost_LIBRARIES_DIRS} ${Flann_LIBRARIES_DIRS} ${Qhull_LIBRARIES_DIRS} ${PCL_LIBRARIES_DIRS})

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.
set(pointcloud_java3_SRCS pointcloud_java3.cpp)
add_library(pointcloud_java3 SHARED ${pointcloud_java3_SRCS})


target_include_directories(pointcloud_java3 PRIVATE
              ${Boost_INCLUDE_DIRS}
              ${Flann_INCLUDE_DIRS}
              ${EIGEN_INCLUDE_DIRS}
              ${QHull_INCLUDE_DIRS}
              ${PCL_INCLUDE_DIRS}
              ${GLM_INCLUDE_DIRS}
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
              log )

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in this
# build script, prebuilt third-party libraries, or system libraries.

target_link_libraries( # Specifies the target library.
                       pointcloud_java3 

                       pcl_common pcl_keypoints pcl_segmentation pcl_features pcl_octree pcl_surface pcl_filters pcl_recognition pcl_tracking pcl_io pcl_registration pcl_io_ply pcl_sample_consensus pcl_kdtree pcl_search
                       boost_filesystem boost_system boost_thread boost_program_options boost_iostreams boost_date_time
                       flann_cpp_s

                       EGL GLESv2

                       # Links the target library to the log library
                       # included in the NDK.
                       ${log-lib} )
                       # log android)
