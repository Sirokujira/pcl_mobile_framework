# cmake/toolchains/ios.toolchain.cmake
#
# Collapses the 15-file toolchains/ directory of Sirokujira/pcl-superbuild@83cc231
# into a single PLATFORM-driven toolchain. See docs/LINEAGE.md.
#
# Single iOS / iOS-Simulator toolchain. Replaces the 18-file collection that
# used to live in toolchains/ (each ARM/x86 slice in its own copy-pasted
# CMake file). Inspired by leetal/ios-cmake v4 but trimmed down to the
# slices we actually ship.
#
# Required input:
#   PLATFORM            One of:
#                         OS64            -- iPhoneOS, arm64
#                         SIMULATORARM64  -- iPhoneSimulator, arm64 (Apple Silicon Macs)
#                         SIMULATOR64     -- iPhoneSimulator, x86_64 (Intel Macs)
#
# Optional input:
#   DEPLOYMENT_TARGET   Defaults to 13.0
#   ENABLE_BITCODE      Defaults to OFF (Bitcode was removed in Xcode 14+)
#   ENABLE_ARC          Defaults to ON
#
# Usage:
#   cmake -S iOSWrapper -B iOSWrapper/build.ios64 \
#         -G Xcode \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/ios.toolchain.cmake \
#         -DPLATFORM=OS64

if(DEFINED PCL_MOBILE_IOS_TOOLCHAIN_LOADED)
    return()
endif()
set(PCL_MOBILE_IOS_TOOLCHAIN_LOADED TRUE)

# ---------------------------------------------------------------------------
# Make our own input variables survive `try_compile` sub-builds. Without
# this, the FATAL_ERROR below fires every time CMake spawns a sub-cmake
# during compiler/ABI detection, because the sub-build starts from an
# empty cache and only sees what CMAKE_TRY_COMPILE_PLATFORM_VARIABLES
# forwards.
# ---------------------------------------------------------------------------
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES
    PLATFORM
    DEPLOYMENT_TARGET
    ENABLE_BITCODE
    ENABLE_ARC
)

# ---------------------------------------------------------------------------
# Validate inputs.
# ---------------------------------------------------------------------------
if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR
        "ios.toolchain.cmake: -DPLATFORM=<OS64|SIMULATORARM64|SIMULATOR64> required")
endif()

set(_supported OS64 SIMULATORARM64 SIMULATOR64)
list(FIND _supported "${PLATFORM}" _idx)
if(_idx EQUAL -1)
    message(FATAL_ERROR
        "ios.toolchain.cmake: PLATFORM=${PLATFORM} not supported. "
        "Choose one of: ${_supported}")
endif()

if(NOT DEFINED DEPLOYMENT_TARGET)
    set(DEPLOYMENT_TARGET "13.0")
endif()

if(NOT DEFINED ENABLE_BITCODE)
    set(ENABLE_BITCODE OFF)
endif()

if(NOT DEFINED ENABLE_ARC)
    set(ENABLE_ARC ON)
endif()

# Promote the inputs to cache entries so the very first configure round
# reaches the rest of the toolchain (and so any nested `cmake -P` script
# sees them too). FORCE keeps them in sync if the user re-runs cmake with
# different -DPLATFORM=... values.
set(PLATFORM          "${PLATFORM}"          CACHE STRING "iOS platform slice"      FORCE)
set(DEPLOYMENT_TARGET "${DEPLOYMENT_TARGET}" CACHE STRING "iOS deployment target"   FORCE)
set(ENABLE_BITCODE    "${ENABLE_BITCODE}"    CACHE BOOL   "Embed bitcode"           FORCE)
set(ENABLE_ARC        "${ENABLE_ARC}"        CACHE BOOL   "Objective-C ARC"         FORCE)

# ---------------------------------------------------------------------------
# Map PLATFORM -> SDK / arch.
# ---------------------------------------------------------------------------
if(PLATFORM STREQUAL "OS64")
    set(SDK_NAME       "iphoneos")
    set(ARCHS          "arm64")
    set(_SYSTEM_NAME   "iOS")
elseif(PLATFORM STREQUAL "SIMULATORARM64")
    set(SDK_NAME       "iphonesimulator")
    set(ARCHS          "arm64")
    set(_SYSTEM_NAME   "iOS")
elseif(PLATFORM STREQUAL "SIMULATOR64")
    set(SDK_NAME       "iphonesimulator")
    set(ARCHS          "x86_64")
    set(_SYSTEM_NAME   "iOS")
endif()

# ---------------------------------------------------------------------------
# Resolve SDK path via xcrun.
# ---------------------------------------------------------------------------
execute_process(
    COMMAND xcrun --sdk ${SDK_NAME} --show-sdk-path
    OUTPUT_VARIABLE CMAKE_OSX_SYSROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _xcrun_rc
)
if(NOT _xcrun_rc EQUAL 0 OR NOT IS_DIRECTORY "${CMAKE_OSX_SYSROOT}")
    message(FATAL_ERROR "xcrun couldn't locate ${SDK_NAME} SDK (rc=${_xcrun_rc})")
endif()

execute_process(
    COMMAND xcrun --sdk ${SDK_NAME} --find clang
    OUTPUT_VARIABLE _CC
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND xcrun --sdk ${SDK_NAME} --find clang++
    OUTPUT_VARIABLE _CXX
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND xcrun --sdk ${SDK_NAME} --find ar
    OUTPUT_VARIABLE _AR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND xcrun --sdk ${SDK_NAME} --find ranlib
    OUTPUT_VARIABLE _RANLIB
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# ---------------------------------------------------------------------------
# Tell CMake we are cross-compiling for iOS.
# ---------------------------------------------------------------------------
set(CMAKE_SYSTEM_NAME       iOS)
set(CMAKE_SYSTEM_PROCESSOR  ${ARCHS})
set(CMAKE_OSX_ARCHITECTURES ${ARCHS}      CACHE STRING "iOS architectures")
set(CMAKE_OSX_DEPLOYMENT_TARGET ${DEPLOYMENT_TARGET}
                                          CACHE STRING "iOS deployment target")
set(CMAKE_OSX_SYSROOT       ${CMAKE_OSX_SYSROOT}
                                          CACHE PATH   "iOS SDK path")
set(CMAKE_C_COMPILER        ${_CC}        CACHE FILEPATH "iOS C compiler")
set(CMAKE_CXX_COMPILER      ${_CXX}       CACHE FILEPATH "iOS C++ compiler")
set(CMAKE_AR                ${_AR}        CACHE FILEPATH "iOS ar")
set(CMAKE_RANLIB            ${_RANLIB}    CACHE FILEPATH "iOS ranlib")

# Skip platform compiler probes — they fail when cross-compiling.
set(CMAKE_C_COMPILER_WORKS   TRUE)
set(CMAKE_CXX_COMPILER_WORKS TRUE)

# Search for libraries/headers under the SDK only.
set(CMAKE_FIND_ROOT_PATH         ${CMAKE_OSX_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ---------------------------------------------------------------------------
# Compiler / linker flag stack.
# ---------------------------------------------------------------------------
if(SDK_NAME STREQUAL "iphoneos")
    set(_min_version "-miphoneos-version-min=${DEPLOYMENT_TARGET}")
else()
    set(_min_version "-mios-simulator-version-min=${DEPLOYMENT_TARGET}")
endif()

set(_common_flags "-arch ${ARCHS} -isysroot ${CMAKE_OSX_SYSROOT} ${_min_version}")

# `-fno-autolink` prevents clang from emitting `LC_LINKER_OPTION` load
# commands like `-framework UIUtilities` into the resulting `.o` files.
# Apple's iPhoneOS 26+ SDK pulls UIUtilities (a *private* framework) in
# transitively through Foundation's module map, and CocoaPods Trunk's
# lint environment can't resolve that framework — `pod trunk push` then
# fails when its sample app tries to link against our XCFramework.
# Disabling autolink at the source compilation step keeps the load
# commands out of every `.o` we ever ship, so consumers don't inherit
# the unresolvable reference.
set(_common_flags "${_common_flags} -fno-autolink")

if(ENABLE_BITCODE)
    set(_common_flags "${_common_flags} -fembed-bitcode")
    set(CMAKE_XCODE_ATTRIBUTE_ENABLE_BITCODE YES)
else()
    set(CMAKE_XCODE_ATTRIBUTE_ENABLE_BITCODE NO)
endif()

if(ENABLE_ARC)
    set(_objc_flags "-fobjc-arc")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_ENABLE_OBJC_ARC YES)
else()
    set(_objc_flags "")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_ENABLE_OBJC_ARC NO)
endif()

set(CMAKE_C_FLAGS_INIT     "${_common_flags}")
set(CMAKE_CXX_FLAGS_INIT   "${_common_flags} -stdlib=libc++")
set(CMAKE_OBJC_FLAGS_INIT  "${_common_flags} ${_objc_flags}")
set(CMAKE_OBJCXX_FLAGS_INIT "${_common_flags} ${_objc_flags} -stdlib=libc++")
set(CMAKE_EXE_LINKER_FLAGS_INIT    "${_common_flags}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_common_flags}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${_common_flags}")

set(CMAKE_XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET ${DEPLOYMENT_TARGET})
set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY  "libc++")
set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++17")
set(CMAKE_XCODE_ATTRIBUTE_GCC_C_LANGUAGE_STANDARD "gnu17")
set(CMAKE_XCODE_ATTRIBUTE_BUILD_LIBRARY_FOR_DISTRIBUTION YES)
set(CMAKE_XCODE_ATTRIBUTE_SKIP_INSTALL NO)

# When CMAKE_SYSTEM_NAME=iOS, CMake initialises new executable targets with
# MACOSX_BUNDLE=ON. That blows up third-party CMakeLists.txt files that
# install command-line tools without supplying BUNDLE DESTINATION (qhull
# v8.0.2 hits this at line 688). We don't ship any iOS app from this repo
# -- only static libraries -- so default executables back to plain CLI
# binaries. Consumers that *do* want a real app bundle can flip this on per
# target with `set_target_properties(<tgt> PROPERTIES MACOSX_BUNDLE TRUE)`.
set(CMAKE_MACOSX_BUNDLE OFF CACHE BOOL "" FORCE)

message(STATUS "[ios.toolchain]")
message(STATUS "  PLATFORM           = ${PLATFORM}")
message(STATUS "  SDK_NAME           = ${SDK_NAME}")
message(STATUS "  ARCHS              = ${ARCHS}")
message(STATUS "  DEPLOYMENT_TARGET  = ${DEPLOYMENT_TARGET}")
message(STATUS "  CMAKE_OSX_SYSROOT  = ${CMAKE_OSX_SYSROOT}")
message(STATUS "  ENABLE_BITCODE     = ${ENABLE_BITCODE}")
