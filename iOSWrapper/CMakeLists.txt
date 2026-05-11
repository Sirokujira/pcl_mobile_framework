# iOS framework wrapper.
#
# Generates an Xcode project that builds PCLMobile.framework, an Objective-C++
# facade over the PCL/Boost/Eigen/FLANN/Qhull static libraries produced by the
# top-level superbuild.
#
# Usage:
#   cmake -S iOSWrapper -B iOSWrapper/build.ios \
#         -G Xcode \
#         -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/ios.toolchain.cmake \
#         -DPLATFORM=OS64 \
#         -DDEPLOYMENT_TARGET=13.0
#   cmake --build iOSWrapper/build.ios --config Release
#
# scripts/build_ios.sh wraps these incantations for each (platform, sdk) combo.
# scripts/make_xcframework.sh stitches the per-slice .frameworks into a single
# PCLMobile.xcframework.

cmake_minimum_required(VERSION 3.24 FATAL_ERROR)

project(PCLMobile
    VERSION 0.1.0
    LANGUAGES C CXX OBJC OBJCXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_OBJCXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ------------------------------------------------------------------
# Where the cross-compiled static libs live. Override with -DPCL_INSTALL_PREFIX
# from scripts/build_ios.sh; default expects the layout the superbuild
# produces under <repo>/build/CMakeExternals/Install/<module>-<tag>/...
# ------------------------------------------------------------------
if(NOT DEFINED PCL_INSTALL_PREFIX)
    set(PCL_INSTALL_PREFIX
        "${CMAKE_CURRENT_SOURCE_DIR}/../build/CMakeExternals/Install"
        CACHE PATH "Install prefix used by the superbuild")
endif()

if(NOT DEFINED PCL_TARGET_TAG)
    # Examples: ios-arm64, iossim-arm64, iossim-x86_64.
    if(PLATFORM STREQUAL "OS64")
        set(PCL_TARGET_TAG "ios-arm64")
    elseif(PLATFORM STREQUAL "SIMULATORARM64")
        set(PCL_TARGET_TAG "iossim-arm64")
    elseif(PLATFORM STREQUAL "SIMULATOR64")
        set(PCL_TARGET_TAG "iossim-x86_64")
    else()
        message(FATAL_ERROR
            "Set PLATFORM to one of: OS64, SIMULATORARM64, SIMULATOR64")
    endif()
endif()

set(PCL_DIR_FOR_TAG    "${PCL_INSTALL_PREFIX}/pcl-${PCL_TARGET_TAG}")
set(BOOST_DIR_FOR_TAG  "${PCL_INSTALL_PREFIX}/boost-${PCL_TARGET_TAG}")
set(FLANN_DIR_FOR_TAG  "${PCL_INSTALL_PREFIX}/flann-${PCL_TARGET_TAG}")
set(QHULL_DIR_FOR_TAG  "${PCL_INSTALL_PREFIX}/qhull-${PCL_TARGET_TAG}")
set(LZ4_DIR_FOR_TAG    "${PCL_INSTALL_PREFIX}/lz4-${PCL_TARGET_TAG}")
set(EIGEN_DIR          "${PCL_INSTALL_PREFIX}/eigen")

# ------------------------------------------------------------------
# Public API + Objective-C++ implementation.
# ------------------------------------------------------------------
set(PCLMOBILE_PUBLIC_HEADERS
    Sources/include/PCLMobile/PCLMobile.h
    Sources/include/PCLMobile/PCLMPointCloud.h
    Sources/include/PCLMobile/PCLMGeometry.h
    Sources/include/PCLMobile/PCLMSegmentation.h
    Sources/include/PCLMobile/PCLMRegistration.h
    Sources/include/PCLMobile/PCLMSurface.h
)

set(PCLMOBILE_SOURCES
    Sources/PCLMobile.mm
    Sources/PCLMPointCloud.mm
    Sources/PCLMFilters.mm
    Sources/PCLMFeatures.mm
    Sources/PCLMGeometry.mm
    Sources/PCLMSearch.mm
    Sources/PCLMSegmentation.mm
    Sources/PCLMSurface.mm
    Sources/PCLMRegistration.mm
)

add_library(PCLMobile STATIC
    ${PCLMOBILE_SOURCES}
    ${PCLMOBILE_PUBLIC_HEADERS}
)

target_include_directories(PCLMobile
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Sources/include>
    PRIVATE
        ${PCL_DIR_FOR_TAG}/include/pcl-1.14
        ${BOOST_DIR_FOR_TAG}/include
        ${FLANN_DIR_FOR_TAG}/include
        ${QHULL_DIR_FOR_TAG}/include
        ${EIGEN_DIR}/include/eigen3
)

# Don't expose PCL/Boost/Eigen/FLANN/Qhull headers to consumers: only the
# Objective-C umbrella header in Sources/include/PCLMobile/ is public.
#
# CMake 3.21+ supports STATIC libraries with FRAMEWORK = TRUE, which produces
# a static framework on iOS. We pin MACH_O_TYPE = staticlib so Xcode does not
# fall back to building a dylib (which would crash at app load time on iOS
# because static-only frameworks can't be code-signed as dynamic).
set_target_properties(PCLMobile PROPERTIES
    FRAMEWORK TRUE
    FRAMEWORK_VERSION A
    MACOSX_FRAMEWORK_IDENTIFIER io.github.sirokujira.PCLMobile
    MACOSX_FRAMEWORK_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_FRAMEWORK_SHORT_VERSION_STRING ${PROJECT_VERSION}
    MACOSX_FRAMEWORK_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Sources/Info.plist
    PUBLIC_HEADER "${PCLMOBILE_PUBLIC_HEADERS}"
    XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER io.github.sirokujira.PCLMobile
    XCODE_ATTRIBUTE_MACH_O_TYPE staticlib
    XCODE_ATTRIBUTE_DEFINES_MODULE YES
    XCODE_ATTRIBUTE_CLANG_ENABLE_MODULES YES
    XCODE_ATTRIBUTE_CLANG_ENABLE_OBJC_ARC YES
    XCODE_ATTRIBUTE_GCC_C_LANGUAGE_STANDARD gnu17
    XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++17"
    XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++"
    XCODE_ATTRIBUTE_ENABLE_BITCODE NO
    XCODE_ATTRIBUTE_SKIP_INSTALL NO
    XCODE_ATTRIBUTE_BUILD_LIBRARY_FOR_DISTRIBUTION YES
    XCODE_ATTRIBUTE_CODE_SIGNING_REQUIRED NO
    XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY ""
    XCODE_ATTRIBUTE_GCC_GENERATE_DEBUGGING_SYMBOLS YES
    XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT "dwarf-with-dsym"
    # Disable Clang's "-fmodules-autolink" so the resulting static archive
    # doesn't carry LC_LINKER_OPTION load commands like `-framework
    # UIUtilities`. iPhoneOS 18+ SDK pulls UIUtilities transitively through
    # Foundation's module map; CocoaPods Trunk's older lint environment
    # can't resolve it and bails. The wrapper itself doesn't need any
    # frameworks beyond Foundation, which we link explicitly below.
    XCODE_ATTRIBUTE_CLANG_MODULES_AUTOLINK NO
)

# Pull every static .a in the relevant install-tag folder. We rely on the
# superbuild to have installed them with predictable filenames.
file(GLOB PCL_STATIC   "${PCL_DIR_FOR_TAG}/lib/libpcl_*.a")
file(GLOB BOOST_STATIC "${BOOST_DIR_FOR_TAG}/lib/libboost_*.a")
file(GLOB FLANN_STATIC "${FLANN_DIR_FOR_TAG}/lib/libflann*.a")
file(GLOB QHULL_STATIC "${QHULL_DIR_FOR_TAG}/lib/libqhull*.a")
file(GLOB LZ4_STATIC   "${LZ4_DIR_FOR_TAG}/lib/liblz4*.a")

target_link_libraries(PCLMobile
    PRIVATE
        ${PCL_STATIC}
        ${BOOST_STATIC}
        ${FLANN_STATIC}
        ${QHULL_STATIC}
        ${LZ4_STATIC}
        "-framework Foundation"
)

# Copy the umbrella module map into the framework bundle so consumers see a
# proper Clang module rather than just a header collection.
set_source_files_properties(${PCLMOBILE_PUBLIC_HEADERS}
    PROPERTIES MACOSX_PACKAGE_LOCATION Headers
)

add_custom_command(TARGET PCLMobile POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            $<TARGET_BUNDLE_CONTENT_DIR:PCLMobile>/Modules
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/module.modulemap
            $<TARGET_BUNDLE_CONTENT_DIR:PCLMobile>/Modules/module.modulemap
    VERBATIM
)
