#!/usr/bin/env bash
#
# scripts/build_ios.sh
#
# Cross-compiles the PCL stack for iOS device + simulator slices, then asks
# CMake to build PCLMobile.framework against each slice. Finally calls
# scripts/make_xcframework.sh to merge the slices.
#
# Optional env:
#   IOS_SLICES        -- space-separated, defaults to "OS64 SIMULATORARM64 SIMULATOR64"
#   DEPLOYMENT_TARGET -- defaults to 13.0
#   BUILD_DIR         -- defaults to <repo>/build/ios
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SLICES="${IOS_SLICES:-OS64 SIMULATORARM64 SIMULATOR64}"
DEPLOYMENT_TARGET="${DEPLOYMENT_TARGET:-13.0}"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build/ios}"
TOOLCHAIN="${REPO_ROOT}/cmake/toolchains/ios.toolchain.cmake"

if [[ "$(uname -s)" != "Darwin" ]]; then
    echo "ERROR: iOS slices can only be built on macOS." >&2
    exit 1
fi

if [[ ! -f "${TOOLCHAIN}" ]]; then
    echo "ERROR: ${TOOLCHAIN} missing." >&2
    exit 1
fi

slice_to_tag() {
    case "$1" in
        OS64)            echo "ios-arm64";;
        SIMULATORARM64)  echo "iossim-arm64";;
        SIMULATOR64)     echo "iossim-x86_64";;
        *) echo "ERROR: unknown PLATFORM=$1" >&2; exit 1;;
    esac
}

framework_config_dir_for_slice() {
    case "$1" in
        OS64)            echo "Release-iphoneos";;
        SIMULATORARM64)  echo "Release-iphonesimulator";;
        SIMULATOR64)     echo "Release-iphonesimulator";;
        *) echo "ERROR: unknown PLATFORM=$1" >&2; exit 1;;
    esac
}

merge_static_archives_into_framework() {
    local slice="$1"
    local tag="$2"
    local framework_build="$3"
    local install_prefix="$4"
    local cfg_dir
    cfg_dir="$(framework_config_dir_for_slice "${slice}")"

    local framework_binary="${framework_build}/${cfg_dir}/PCLMobile.framework/PCLMobile"
    if [[ ! -f "${framework_binary}" ]]; then
        echo "ERROR: missing framework binary: ${framework_binary}" >&2
        exit 1
    fi

    local old_nullglob
    old_nullglob="$(shopt -p nullglob || true)"
    shopt -s nullglob
    local archive_inputs=(
        "${framework_binary}"
        "${install_prefix}/pcl-${tag}/lib"/libpcl_*.a
        "${install_prefix}/boost-${tag}/lib"/libboost_*.a
        "${install_prefix}/flann-${tag}/lib"/libflann*.a
        "${install_prefix}/qhull-${tag}/lib"/libqhull*.a
        "${install_prefix}/lz4-${tag}/lib"/liblz4*.a
    )
    eval "${old_nullglob}"

    if [[ ${#archive_inputs[@]} -le 1 ]]; then
        echo "ERROR: no dependency static archives found under ${install_prefix} for ${tag}" >&2
        exit 1
    fi

    local merged_binary="${framework_binary}.merged.$$"
    rm -f "${merged_binary}"
    echo "=== build_ios: merging $((${#archive_inputs[@]} - 1)) dependency archives into ${framework_binary} ==="
    xcrun libtool -static -o "${merged_binary}" "${archive_inputs[@]}"
    mv "${merged_binary}" "${framework_binary}"

    # Belt-and-suspenders fallback: if any LC_LINKER_OPTION load commands
    # leaked through the `-fno-autolink` net (most often `-framework
    # UIUtilities` from the iPhoneOS 26+ Foundation module map), strip
    # them after merging. The script is a no-op when there's nothing to
    # remove; PCLMOBILE_STRIP_AUTOLINK=OFF skips it entirely.
    if [[ "${PCLMOBILE_STRIP_AUTOLINK:-ON}" != "OFF" \
       && "${PCLMOBILE_STRIP_AUTOLINK:-ON}" != "0"  \
       && "${PCLMOBILE_STRIP_AUTOLINK:-ON}" != "false" ]]; then
        if command -v python3 >/dev/null 2>&1; then
            python3 "${REPO_ROOT}/scripts/strip_autolink_options.py" \
                "${framework_binary}" \
                --match UIUtilities \
                --verbose || true
        fi
    fi
}

mkdir -p "${BUILD_DIR}"

# 1. Cross-compile PCL/Boost/etc for each slice.
for SLICE in ${SLICES}; do
    TAG="$(slice_to_tag "${SLICE}")"
    SUPER_BUILD="${BUILD_DIR}/${TAG}/superbuild"
    mkdir -p "${SUPER_BUILD}"
    echo
    echo "=== build_ios: superbuild for ${SLICE} (tag=${TAG}) ==="
    cmake -S "${REPO_ROOT}" -B "${SUPER_BUILD}" -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}" \
        -DPLATFORM="${SLICE}" \
        -DDEPLOYMENT_TARGET="${DEPLOYMENT_TARGET}" \
        -DBUILD_ANDROID=OFF \
        -DBUILD_IOS_DEVICE=ON \
        -DBUILD_IOS_SIMULATOR=ON \
        -DPCL_MOBILE_TAG="${TAG}"
    cmake --build "${SUPER_BUILD}" --parallel
done

# 2. Build PCLMobile.framework against each slice via Xcode.
for SLICE in ${SLICES}; do
    TAG="$(slice_to_tag "${SLICE}")"
    FRAMEWORK_BUILD="${BUILD_DIR}/${TAG}/wrapper"
    INSTALL_PREFIX="${BUILD_DIR}/${TAG}/superbuild/CMakeExternals/Install"
    echo
    echo "=== build_ios: wrapper framework for ${SLICE} (tag=${TAG}) ==="
    cmake -S "${REPO_ROOT}/iOSWrapper" -B "${FRAMEWORK_BUILD}" -G Xcode \
        -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}" \
        -DPLATFORM="${SLICE}" \
        -DDEPLOYMENT_TARGET="${DEPLOYMENT_TARGET}" \
        -DPCL_INSTALL_PREFIX="${INSTALL_PREFIX}" \
        -DPCL_TARGET_TAG="${TAG}"
    cmake --build "${FRAMEWORK_BUILD}" --config Release
    merge_static_archives_into_framework "${SLICE}" "${TAG}" "${FRAMEWORK_BUILD}" "${INSTALL_PREFIX}"
done

# 3. Stitch the slices into one XCFramework.
"${REPO_ROOT}/scripts/make_xcframework.sh" ${SLICES}
