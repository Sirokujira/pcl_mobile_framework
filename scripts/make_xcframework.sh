#!/usr/bin/env bash
#
# scripts/make_xcframework.sh
#
# Combines the per-slice PCLMobile.framework builds produced by
# scripts/build_ios.sh into a single PCLMobile.xcframework that
# CocoaPods / SwiftPM / Carthage can consume.
#
# Usage:
#   scripts/make_xcframework.sh [SLICE ...]
#   # default slices: OS64 SIMULATORARM64 SIMULATOR64
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build/ios}"
OUT_DIR="${BUILD_DIR}/xcframework"
OUT_FRAMEWORK="${OUT_DIR}/PCLMobile.xcframework"

if [[ "$(uname -s)" != "Darwin" ]]; then
    echo "ERROR: XCFramework creation needs macOS / Xcode." >&2
    exit 1
fi

SLICES=("$@")
if [[ ${#SLICES[@]} -eq 0 ]]; then
    SLICES=(OS64 SIMULATORARM64 SIMULATOR64)
fi

slice_to_tag() {
    case "$1" in
        OS64)            echo "ios-arm64";;
        SIMULATORARM64)  echo "iossim-arm64";;
        SIMULATOR64)     echo "iossim-x86_64";;
        *) echo "ERROR: unknown slice=$1" >&2; exit 1;;
    esac
}

framework_for_slice() {
    local slice="$1"
    local tag
    tag="$(slice_to_tag "${slice}")"
    # CMake's Xcode generator drops the framework into Release-iphoneos /
    # Release-iphonesimulator depending on the slice.
    local cfg_dir
    case "${slice}" in
        OS64)            cfg_dir="Release-iphoneos";;
        SIMULATORARM64)  cfg_dir="Release-iphonesimulator";;
        SIMULATOR64)     cfg_dir="Release-iphonesimulator";;
    esac
    echo "${BUILD_DIR}/${tag}/wrapper/${cfg_dir}/PCLMobile.framework"
}

# Note: when both SIMULATORARM64 and SIMULATOR64 are produced, lipo their
# PCLMobile binaries into a single fat simulator slice, then hand it to
# xcodebuild together with the device slice. xcodebuild -create-xcframework
# refuses to combine two simulator-arch slices on its own.
#
# This covers the common Apple-Silicon-Mac case where you want both
# simulator architectures inside one .xcframework.

mkdir -p "${OUT_DIR}"
rm -rf "${OUT_FRAMEWORK}"

LIBRARY_ARGS=()

# Device slice (always at most one).
device_fw=""
for slice in "${SLICES[@]}"; do
    if [[ "${slice}" == "OS64" ]]; then
        device_fw="$(framework_for_slice "${slice}")"
        if [[ ! -d "${device_fw}" ]]; then
            echo "ERROR: missing ${device_fw} -- run scripts/build_ios.sh first." >&2
            exit 1
        fi
        LIBRARY_ARGS+=( -framework "${device_fw}" )
    fi
done

# Simulator slice(s).
sim_arm64_fw=""
sim_x86_64_fw=""
for slice in "${SLICES[@]}"; do
    case "${slice}" in
        SIMULATORARM64) sim_arm64_fw="$(framework_for_slice "${slice}")";;
        SIMULATOR64)    sim_x86_64_fw="$(framework_for_slice "${slice}")";;
    esac
done

if [[ -n "${sim_arm64_fw}" && -n "${sim_x86_64_fw}" ]]; then
    # Merge both simulator architectures into one .framework with `lipo`.
    MERGED_DIR="${BUILD_DIR}/sim-merged/PCLMobile.framework"
    rm -rf "${MERGED_DIR}"
    mkdir -p "$(dirname "${MERGED_DIR}")"
    cp -R "${sim_arm64_fw}" "${MERGED_DIR}"
    lipo -create \
        "${sim_arm64_fw}/PCLMobile" \
        "${sim_x86_64_fw}/PCLMobile" \
        -output "${MERGED_DIR}/PCLMobile"
    LIBRARY_ARGS+=( -framework "${MERGED_DIR}" )
elif [[ -n "${sim_arm64_fw}" ]]; then
    LIBRARY_ARGS+=( -framework "${sim_arm64_fw}" )
elif [[ -n "${sim_x86_64_fw}" ]]; then
    LIBRARY_ARGS+=( -framework "${sim_x86_64_fw}" )
fi

if [[ ${#LIBRARY_ARGS[@]} -eq 0 ]]; then
    echo "ERROR: no slices supplied." >&2
    exit 1
fi

echo "Creating ${OUT_FRAMEWORK} ..."
xcodebuild -create-xcframework "${LIBRARY_ARGS[@]}" -output "${OUT_FRAMEWORK}"

# Print a checksum that Package.swift can paste into binaryTarget(url:checksum:)
# once the XCFramework is uploaded to a GitHub Release.
ZIP="${OUT_FRAMEWORK}.zip"
rm -f "${ZIP}"
( cd "${OUT_DIR}" && zip -ryq "${ZIP##*/}" "$(basename "${OUT_FRAMEWORK}")" )
SUM="$(swift package compute-checksum "${ZIP}" 2>/dev/null || true)"
if [[ -n "${SUM}" ]]; then
    echo "[swift package compute-checksum] ${SUM}  ${ZIP##*/}"
else
    SUM="$(shasum -a 256 "${ZIP}" | awk '{print $1}')"
    echo "[shasum -a 256]                  ${SUM}  ${ZIP##*/}"
fi

echo "Done."
echo "  ${OUT_FRAMEWORK}"
echo "  ${ZIP}"

# ---------------------------------------------------------------------------
# Sanity-check the size. A "real" XCFramework with PCL + Boost + FLANN +
# Qhull + LZ4 merged in should be ~50–200 MB compressed. Anything smaller
# means scripts/build_ios.sh's merge_static_archives_into_framework step
# never ran (i.e. the framework binary still only contains the wrapper's
# own .mm files, ~1 MB).
#
# Override the threshold via XCFRAMEWORK_MIN_MB if you intentionally trim
# components.
# ---------------------------------------------------------------------------
ZIP_BYTES=$(stat -f '%z' "${ZIP}" 2>/dev/null || stat -c '%s' "${ZIP}" 2>/dev/null || echo 0)
ZIP_MB=$(( ZIP_BYTES / 1024 / 1024 ))
MIN_MB="${XCFRAMEWORK_MIN_MB:-30}"

echo "  size : ${ZIP_MB} MB"

if (( ZIP_MB < MIN_MB )); then
    cat >&2 <<EOF

⚠️  WARNING: ${ZIP##*/} is only ${ZIP_MB} MB.

A complete PCLMobile.xcframework with Boost/FLANN/Qhull/LZ4/PCL merged in
typically weighs 50–200 MB. Sizes below ${MIN_MB} MB usually mean
scripts/build_ios.sh's merge_static_archives_into_framework step did not
run -- the framework binary contains only the wrapper's own object files
and apps that link against it will fail with thousands of undefined
PCL/Boost/etc. symbols at runtime.

Inspect the framework binary directly to confirm:
  file  ${OUT_FRAMEWORK}/ios-arm64/PCLMobile.framework/PCLMobile
  ls -lh ${OUT_FRAMEWORK}/ios-arm64/PCLMobile.framework/PCLMobile

To rebuild from a clean state:
  rm -rf build/ios
  ./scripts/build_ios.sh

If you intentionally produce a thin XCFramework, override the threshold:
  XCFRAMEWORK_MIN_MB=1 ./scripts/make_xcframework.sh
EOF
    exit 1
fi
