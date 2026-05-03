#!/usr/bin/env bash
#
# scripts/package_android.sh
#
# Builds the staged Android native dependencies, produces a release AAR, and
# optionally uploads/publishes it.
#
# Usage:
#   scripts/package_android.sh <version>
#   scripts/package_android.sh <version> --upload-release
#   scripts/package_android.sh <version> --maven-local
#   scripts/package_android.sh <version> --local-repo
#   scripts/package_android.sh <version> --publish-maven
#
# Options:
#   --skip-native      Do not run scripts/build_android.sh before Gradle.
#   --upload-release  Upload the AAR to GitHub Release v<version>.
#   --maven-local     Publish to ~/.m2/repository.
#   --local-repo      Publish to AndroidWrapper/aar/pclmobile/build/repo.
#   --publish-maven   Publish to MAVEN_REPOSITORY_URL.
#
# Optional env:
#   PCLMOBILE_RELEASE_TAG  GitHub Release tag, defaults to v<version>.
set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <version> [--skip-native] [--upload-release] [--maven-local] [--local-repo] [--publish-maven]" >&2
    exit 64
fi

VERSION="$1"
shift

SKIP_NATIVE=false
UPLOAD_RELEASE=false
PUBLISH_MAVEN_LOCAL=false
PUBLISH_LOCAL_REPO=false
PUBLISH_REMOTE=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-native)
            SKIP_NATIVE=true
            ;;
        --upload-release)
            UPLOAD_RELEASE=true
            ;;
        --maven-local)
            PUBLISH_MAVEN_LOCAL=true
            ;;
        --local-repo)
            PUBLISH_LOCAL_REPO=true
            ;;
        --publish-maven)
            PUBLISH_REMOTE=true
            ;;
        *)
            echo "ERROR: unknown option: $1" >&2
            exit 64
            ;;
    esac
    shift
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GRADLE_DIR="${REPO_ROOT}/AndroidWrapper/aar"
AAR_PATH="${GRADLE_DIR}/pclmobile/build/outputs/aar/pclmobile-release.aar"
TAG="${PCLMOBILE_RELEASE_TAG:-v${VERSION}}"

export ANDROID_ABIS="${ANDROID_ABIS:-arm64-v8a armeabi-v7a x86_64}"
export ANDROID_CLEAN_AFTER_STAGE="${ANDROID_CLEAN_AFTER_STAGE:-ON}"
export PCLMOBILE_VERSION="${PCLMOBILE_VERSION:-${VERSION}}"

if ! ${SKIP_NATIVE}; then
    echo "==> Building Android native dependencies (${ANDROID_ABIS})"
    "${REPO_ROOT}/scripts/build_android.sh"
fi

cd "${GRADLE_DIR}"
if [[ -x ./gradlew ]]; then
    GRADLE=(./gradlew --no-daemon)
else
    GRADLE=(sh ./gradlew --no-daemon)
fi

echo "==> Building release AAR (${PCLMOBILE_VERSION})"
"${GRADLE[@]}" :pclmobile:assembleRelease

if [[ ! -f "${AAR_PATH}" ]]; then
    echo "ERROR: ${AAR_PATH} not produced." >&2
    exit 1
fi

echo "==> AAR: ${AAR_PATH}"
AAR_LIST="$(unzip -l "${AAR_PATH}")"
for ABI in ${ANDROID_ABIS}; do
    echo "${AAR_LIST}" | grep "jni/${ABI}/libnative-lib.so" >/dev/null || {
        echo "ERROR: AAR does not contain jni/${ABI}/libnative-lib.so." >&2
        exit 1
    }
    echo "${AAR_LIST}" | grep "jni/${ABI}/libc++_shared.so" >/dev/null || {
        echo "ERROR: AAR does not contain jni/${ABI}/libc++_shared.so." >&2
        exit 1
    }
done

if ${PUBLISH_MAVEN_LOCAL}; then
    echo "==> Publishing to Maven local"
    "${GRADLE[@]}" :pclmobile:publishReleasePublicationToMavenLocal
fi

if ${PUBLISH_LOCAL_REPO}; then
    echo "==> Publishing to local release repository"
    "${GRADLE[@]}" :pclmobile:publishReleasePublicationToLocalReleaseRepository
fi

if ${PUBLISH_REMOTE}; then
    if [[ -z "${MAVEN_REPOSITORY_URL:-}" ]]; then
        echo "ERROR: --publish-maven requires MAVEN_REPOSITORY_URL." >&2
        exit 64
    fi
    echo "==> Publishing to ${MAVEN_REPOSITORY_URL}"
    "${GRADLE[@]}" :pclmobile:publishReleasePublicationToRemoteRepository
fi

if ${UPLOAD_RELEASE}; then
    if ! command -v gh >/dev/null 2>&1; then
        echo "ERROR: --upload-release requires the GitHub CLI (gh)." >&2
        exit 1
    fi
    echo "==> Uploading AAR to GitHub Release ${TAG}"
    gh release upload "${TAG}" "${AAR_PATH}" --clobber
fi

echo "==> Android package complete."
