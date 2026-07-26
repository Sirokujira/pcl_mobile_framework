#!/usr/bin/env bash
#
# scripts/build_android.sh
#
# Successor to build_android.sh / build_android.bat, which entered this
# repository with the 2019 import commit 98c6c50. They are not in
# Sirokujira/pcl-superbuild@83cc231 — nor anywhere in that repository's
# history. See docs/LINEAGE.md.
#
# Cross-compiles PCL/Boost/Eigen/FLANN/Qhull for the listed Android ABIs and
# stages the result under AndroidWrapper/aar/pclmobile/libs/<ABI>/ so that
# the AAR module can pick them up.
#
# Required env:
#   ANDROID_NDK_HOME (or ANDROID_NDK)  -- path to NDK r26 or newer.
#                                          If unset, the newest SDK-installed
#                                          NDK under ~/Library/Android/sdk is used.
#
# Optional env:
#   ANDROID_ABIS                       -- space-separated, defaults to all 3
#   ANDROID_PLATFORM                   -- defaults to android-24
#   BUILD_DIR                          -- defaults to <repo>/build/android
#   ANDROID_CLEAN_AFTER_STAGE          -- ON deletes each ABI build tree after
#                                          staging to keep multi-ABI builds small.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NDK="${ANDROID_NDK_HOME:-${ANDROID_NDK:-}}"

# When ANDROID_NDK_HOME isn't set, try the platform's default Android Studio
# install location. macOS keeps the SDK under ~/Library/Android/sdk; Linux
# uses ~/Android/Sdk; Windows users are expected to set ANDROID_NDK_HOME.
if [[ -z "${NDK}" && -n "${ANDROID_NDK_VERSION:-}" ]]; then
    for sdk_root in \
            "${ANDROID_HOME:-}" \
            "${ANDROID_SDK_ROOT:-}" \
            "${HOME}/Library/Android/sdk" \
            "${HOME}/Android/Sdk"; do
        candidate="${sdk_root}/ndk/${ANDROID_NDK_VERSION}"
        if [[ -d "${candidate}" ]]; then
            NDK="${candidate}"
            break
        fi
    done
fi

if [[ -z "${NDK}" ]]; then
    for candidate in \
            "${ANDROID_HOME:-}/ndk" \
            "${ANDROID_SDK_ROOT:-}/ndk" \
            "${HOME}/Library/Android/sdk/ndk" \
            "${HOME}/Android/Sdk/ndk"; do
        if [[ -d "${candidate}" ]]; then
            NDK="$(find "${candidate}" -mindepth 1 -maxdepth 1 -type d | sort | tail -n 1)"
            [[ -n "${NDK}" ]] && break
        fi
    done
fi

if [[ -z "${NDK}" ]]; then
    echo "ERROR: set ANDROID_NDK_HOME (or ANDROID_NDK) to your r26+ NDK path." >&2
    exit 1
fi

if [[ ! -f "${NDK}/build/cmake/android.toolchain.cmake" ]]; then
    echo "ERROR: ${NDK}/build/cmake/android.toolchain.cmake not found." >&2
    exit 1
fi
export ANDROID_NDK_HOME="${NDK}"

ABIS="${ANDROID_ABIS:-arm64-v8a armeabi-v7a x86_64}"
PLATFORM="${ANDROID_PLATFORM:-android-24}"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build/android}"
INSTALL_BASE="${BUILD_DIR}/install"
STAGE_DIR="${REPO_ROOT}/AndroidWrapper/aar/pclmobile/libs"
CLEAN_AFTER_STAGE="${ANDROID_CLEAN_AFTER_STAGE:-OFF}"
STAGE_ROOT_REAL=""

if [[ ! "${PLATFORM}" =~ ^android-[0-9]+$ ]]; then
    echo "ERROR: ANDROID_PLATFORM must look like android-24, got: ${PLATFORM}" >&2
    exit 64
fi

for ABI in ${ABIS}; do
    case "${ABI}" in
        arm64-v8a|armeabi-v7a|x86_64) ;;
        *)
            echo "ERROR: unsupported Android ABI '${ABI}'. Allowed: arm64-v8a armeabi-v7a x86_64" >&2
            exit 64
            ;;
    esac
done

export ANDROID_NDK_HOME="${NDK}"
export ANDROID_PLATFORM="${PLATFORM}"

mkdir -p "${BUILD_DIR}" "${STAGE_DIR}"
STAGE_ROOT_REAL="$(cd "${STAGE_DIR}" && pwd -P)"

for ABI in ${ABIS}; do
    echo
    echo "=== build_android: ABI=${ABI} ==="
    BUILD_SUBDIR="${BUILD_DIR}/${ABI}"
    INSTALL_SUBDIR="${INSTALL_BASE}/${ABI}"

    cmake -S "${REPO_ROOT}" -B "${BUILD_SUBDIR}" -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE="${NDK}/build/cmake/android.toolchain.cmake" \
        -DANDROID_ABI="${ABI}" \
        -DANDROID_PLATFORM="${PLATFORM}" \
        -DANDROID_STL=c++_shared \
        -DBUILD_ANDROID=ON \
        -DBUILD_IOS_DEVICE=OFF \
        -DBUILD_IOS_SIMULATOR=OFF \
        -DCMAKE_INSTALL_PREFIX="${INSTALL_SUBDIR}" \
        -DPCL_MOBILE_TAG="android-${ABI}"

    cmake --build "${BUILD_SUBDIR}" --parallel

    # Stage the artefacts where AndroidWrapper/aar/pclmobile/CMakeLists.txt
    # expects them (libs/<ABI>/<module>/{include,lib}).
    DEST="${STAGE_DIR}/${ABI}"
    TMP_DEST="${STAGE_DIR}/.${ABI}.tmp.$$"
    INSTALL_ROOT=""
    if [[ -d "${BUILD_SUBDIR}/CMakeExternals/Install" ]]; then
        INSTALL_ROOT="${BUILD_SUBDIR}/CMakeExternals/Install"
    elif [[ -d "${INSTALL_SUBDIR}" ]]; then
        INSTALL_ROOT="${INSTALL_SUBDIR}"
    fi

    if [[ -z "${INSTALL_ROOT}" ]]; then
        echo "ERROR: install root missing for ${ABI}" >&2
        exit 1
    fi

    case "$(cd "$(dirname "${DEST}")" && pwd -P)/$(basename "${DEST}")" in
        "${STAGE_ROOT_REAL}"/*) ;;
        *)
            echo "ERROR: refusing to stage outside ${STAGE_DIR}: ${DEST}" >&2
            exit 1
            ;;
    esac

    rm -rf "${TMP_DEST}"
    mkdir -p \
        "${TMP_DEST}/eigen" \
        "${TMP_DEST}/boost-android" \
        "${TMP_DEST}/flann-android" \
        "${TMP_DEST}/lz4-android" \
        "${TMP_DEST}/qhull-android" \
        "${TMP_DEST}/pcl-android"

    if [[ ! -d "${INSTALL_ROOT}/eigen/include" ]]; then
        echo "ERROR: missing Eigen include directory for ${ABI}: ${INSTALL_ROOT}/eigen/include" >&2
        rm -rf "${TMP_DEST}"
        exit 1
    fi
    cp -R "${INSTALL_ROOT}/eigen/include" "${TMP_DEST}/eigen/"
    for module in boost flann lz4 qhull pcl; do
        src="${INSTALL_ROOT}/${module}-android-${ABI}"
        dst="${TMP_DEST}/${module}-android"
        if [[ ! -d "${src}/include" || ! -d "${src}/lib" ]]; then
            echo "ERROR: missing ${module} include/lib directories for ${ABI}: ${src}" >&2
            rm -rf "${TMP_DEST}"
            exit 1
        fi
        cp -R "${src}/include" "${dst}/"
        mkdir -p "${dst}/lib"
        find "${src}/lib" -maxdepth 1 -type f -name '*.a' -exec cp {} "${dst}/lib/" \;
        if ! find "${dst}/lib" -maxdepth 1 -type f -name '*.a' -print -quit | grep -q .; then
            echo "ERROR: no static libraries staged for ${module} (${ABI}) from ${src}/lib" >&2
            rm -rf "${TMP_DEST}"
            exit 1
        fi
    done
    rm -rf "${DEST}"
    mv "${TMP_DEST}" "${DEST}"
    echo "[OK] staged ${ABI} -> ${DEST}"

    if [[ "${CLEAN_AFTER_STAGE}" == "ON" || "${CLEAN_AFTER_STAGE}" == "1" || "${CLEAN_AFTER_STAGE}" == "true" ]]; then
        echo "[clean] removing ${BUILD_SUBDIR}"
        rm -rf "${BUILD_SUBDIR}" "${INSTALL_SUBDIR}"
    fi
done

echo
echo "All ABIs built. Run sh ./gradlew :pclmobile:assembleRelease in AndroidWrapper/aar/."
