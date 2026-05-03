# cmake/ProjectVariables.cmake
#
# Renamed and slimmed-down successor to setup-project-variables.cmake.
# Drops the legacy iOS arch options (armv7/armv7s/arm64e/i386) and the
# per-arch try-run-result CMake files; ios.toolchain.cmake handles all of that.

include_guard(GLOBAL)

# Modern Python 3 + git probes. Both are required by ExternalProject_Add.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Git    REQUIRED)

option(BUILD_ANDROID       "Build for Android"        ON)
option(BUILD_IOS_DEVICE    "Build for iOS device"     OFF)
option(BUILD_IOS_SIMULATOR "Build for iOS simulator"  OFF)
set(PCL_MOBILE_ANDROID_PLATFORM "${ANDROID_PLATFORM}" CACHE STRING "Android API level passed to Android ExternalProject builds.")
if(PCL_MOBILE_ANDROID_PLATFORM STREQUAL "")
    set(PCL_MOBILE_ANDROID_PLATFORM "android-24" CACHE STRING "Android API level passed to Android ExternalProject builds." FORCE)
endif()
if(NOT PCL_MOBILE_ANDROID_PLATFORM MATCHES "^android-[0-9]+$")
    message(FATAL_ERROR
        "PCL_MOBILE_ANDROID_PLATFORM must look like android-24, got: "
        "${PCL_MOBILE_ANDROID_PLATFORM}")
endif()
string(REGEX REPLACE "^android-" "" PCL_MOBILE_ANDROID_API_LEVEL "${PCL_MOBILE_ANDROID_PLATFORM}")

function(pcl_mobile_android_platform OUT_PLATFORM OUT_API_LEVEL)
    set(${OUT_PLATFORM} "${PCL_MOBILE_ANDROID_PLATFORM}" PARENT_SCOPE)
    set(${OUT_API_LEVEL} "${PCL_MOBILE_ANDROID_API_LEVEL}" PARENT_SCOPE)
endfunction()

# ccache support (clang launcher).
option(CCACHE_ENABLE "Use ccache when available." ON)
find_program(CCACHE_EXE ccache)
if(CCACHE_EXE AND CCACHE_ENABLE)
    message(STATUS "[ccache] enabled (${CCACHE_EXE})")
    set(CMAKE_C_COMPILER_LAUNCHER   "${CCACHE_EXE}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_EXE}")
endif()

# Per-target tag used by every ExternalProject_Add macro under cmake/external/.
# scripts/build_android.sh and scripts/build_ios.sh set this explicitly via
# -DPCL_MOBILE_TAG=<android-arm64|android-armeabi-v7a|android-x86_64|ios-arm64|iossim-arm64|iossim-x86_64>.
if(NOT DEFINED PCL_MOBILE_TAG AND DEFINED ANDROID_ABI)
    set(PCL_MOBILE_TAG "android-${ANDROID_ABI}")
endif()
if(NOT DEFINED PCL_MOBILE_TAG AND DEFINED PLATFORM)
    if(PLATFORM STREQUAL "OS64")
        set(PCL_MOBILE_TAG "ios-arm64")
    elseif(PLATFORM STREQUAL "SIMULATORARM64")
        set(PCL_MOBILE_TAG "iossim-arm64")
    elseif(PLATFORM STREQUAL "SIMULATOR64")
        set(PCL_MOBILE_TAG "iossim-x86_64")
    endif()
endif()

if(NOT DEFINED PCL_MOBILE_TARGETS)
    if(DEFINED PCL_MOBILE_TAG)
        set(PCL_MOBILE_TARGETS ${PCL_MOBILE_TAG})
    else()
        message(FATAL_ERROR
            "Set PCL_MOBILE_TARGETS (or PCL_MOBILE_TAG) before configuring. "
            "Examples: -DPCL_MOBILE_TAG=android-arm64, -DPCL_MOBILE_TAG=ios-arm64.")
    endif()
endif()

message(STATUS "[ProjectVariables] PCL_MOBILE_TARGETS = ${PCL_MOBILE_TARGETS}")
