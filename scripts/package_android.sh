#!/usr/bin/env bash
#
# scripts/package_android.sh
#
# Builds the staged Android native dependencies, produces a release AAR, and
# optionally uploads/publishes it.
#
# Usage:
#   scripts/package_android.sh <version>
#   scripts/package_android.sh <version> --create-release --upload-release
#   scripts/package_android.sh <version> --upload-release
#   scripts/package_android.sh <version> --maven-local
#   scripts/package_android.sh <version> --local-repo
#   scripts/package_android.sh <version> --publish-maven
#   scripts/package_android.sh <version> --publish-github-packages
#
# Options:
#   --skip-native                  Do not run scripts/build_android.sh before Gradle.
#   --create-release               Create GitHub Release v<version> when missing.
#   --upload-release               Upload the versioned AAR to GitHub Release v<version>.
#   --maven-local                  Publish to ~/.m2/repository.
#   --local-repo                   Publish to AndroidWrapper/aar/pclmobile/build/repo.
#   --publish-maven                Publish to MAVEN_REPOSITORY_URL.
#   --publish-github-packages      Publish to GitHub Packages for the current repository.
#   --maven-repository-url <url>   Maven repository URL for --publish-maven.
#
# Optional env:
#   PCLMOBILE_RELEASE_TAG          GitHub Release tag, defaults to v<version>.
#   PCLMOBILE_GITHUB_REPOSITORY    owner/repo for GitHub Packages and Release.
#                                  Defaults to GITHUB_REPOSITORY, then origin URL.
set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <version> [--skip-native] [--create-release] [--upload-release] [--maven-local] [--local-repo] [--publish-maven] [--publish-github-packages] [--maven-repository-url <url>]" >&2
    exit 64
fi

VERSION="$1"
shift

SKIP_NATIVE=false
CREATE_RELEASE=false
UPLOAD_RELEASE=false
PUBLISH_MAVEN_LOCAL=false
PUBLISH_LOCAL_REPO=false
PUBLISH_REMOTE=false
PUBLISH_GITHUB_PACKAGES=false
MAVEN_REPOSITORY_URL_ARG=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-native)
            SKIP_NATIVE=true
            ;;
        --create-release)
            CREATE_RELEASE=true
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
        --publish-github-packages)
            PUBLISH_GITHUB_PACKAGES=true
            ;;
        --maven-repository-url)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: --maven-repository-url requires a URL value." >&2
                exit 64
            fi
            MAVEN_REPOSITORY_URL_ARG="$2"
            PUBLISH_REMOTE=true
            shift
            ;;
        --maven-repository-url=*)
            MAVEN_REPOSITORY_URL_ARG="${1#*=}"
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
DIST_DIR="${REPO_ROOT}/build/android/distributions"
TAG="${PCLMOBILE_RELEASE_TAG:-v${VERSION}}"

github_repository() {
    if [[ -n "${PCLMOBILE_GITHUB_REPOSITORY:-}" ]]; then
        echo "${PCLMOBILE_GITHUB_REPOSITORY}"
        return
    fi
    if [[ -n "${GITHUB_REPOSITORY:-}" ]]; then
        echo "${GITHUB_REPOSITORY}"
        return
    fi
    git -C "${REPO_ROOT}" remote get-url origin 2>/dev/null \
        | sed -E 's#^git@github.com:##; s#^https://github.com/##; s#\.git$##'
}

GITHUB_REPOSITORY_NAME="$(github_repository)"
VERSIONED_AAR_NAME="pclmobile-${VERSION}.aar"
VERSIONED_AAR_PATH="${DIST_DIR}/${VERSIONED_AAR_NAME}"
GH_RELEASE_ARGS=()
if [[ -n "${GITHUB_REPOSITORY_NAME}" ]]; then
    GH_RELEASE_ARGS=(--repo "${GITHUB_REPOSITORY_NAME}")
fi

export ANDROID_ABIS="${ANDROID_ABIS:-arm64-v8a armeabi-v7a x86_64}"
export ANDROID_CLEAN_AFTER_STAGE="${ANDROID_CLEAN_AFTER_STAGE:-ON}"
export PCLMOBILE_VERSION="${PCLMOBILE_VERSION:-${VERSION}}"

if ${PUBLISH_GITHUB_PACKAGES}; then
    if [[ -z "${GITHUB_REPOSITORY_NAME}" ]]; then
        echo "ERROR: --publish-github-packages could not resolve owner/repo." >&2
        echo "Set PCLMOBILE_GITHUB_REPOSITORY=owner/repo or GITHUB_REPOSITORY=owner/repo." >&2
        exit 64
    fi
    export MAVEN_REPOSITORY_URL="${MAVEN_REPOSITORY_URL:-https://maven.pkg.github.com/${GITHUB_REPOSITORY_NAME}}"
    export MAVEN_USERNAME="${MAVEN_USERNAME:-${GITHUB_ACTOR:-}}"
    if [[ -z "${MAVEN_PASSWORD:-}" && -n "${GITHUB_TOKEN:-}" ]]; then
        export MAVEN_PASSWORD="${GITHUB_TOKEN}"
    fi
    PUBLISH_REMOTE=true
fi

if [[ -n "${MAVEN_REPOSITORY_URL_ARG}" ]]; then
    export MAVEN_REPOSITORY_URL="${MAVEN_REPOSITORY_URL_ARG}"
fi

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

mkdir -p "${DIST_DIR}"
cp "${AAR_PATH}" "${VERSIONED_AAR_PATH}"
shasum -a 256 "${VERSIONED_AAR_PATH}" > "${VERSIONED_AAR_PATH}.sha256"
echo "==> Distribution AAR: ${VERSIONED_AAR_PATH}"
echo "==> SHA256: $(awk '{print $1}' "${VERSIONED_AAR_PATH}.sha256")"

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
        echo "ERROR: --publish-maven requires MAVEN_REPOSITORY_URL or --maven-repository-url." >&2
        exit 64
    fi
    if [[ "${MAVEN_REPOSITORY_URL}" == https://maven.pkg.github.com/* ]]; then
        if [[ -z "${MAVEN_USERNAME:-}" || -z "${MAVEN_PASSWORD:-}" ]]; then
            echo "ERROR: GitHub Packages publish requires MAVEN_USERNAME and MAVEN_PASSWORD." >&2
            echo "In GitHub Actions use github.actor and GITHUB_TOKEN with packages:write." >&2
            exit 64
        fi
    fi
    echo "==> Publishing to ${MAVEN_REPOSITORY_URL}"
    "${GRADLE[@]}" :pclmobile:publishReleasePublicationToRemoteRepository
fi

if ${CREATE_RELEASE} || ${UPLOAD_RELEASE}; then
    if ! command -v gh >/dev/null 2>&1; then
        echo "ERROR: --create-release/--upload-release requires the GitHub CLI (gh)." >&2
        exit 1
    fi
fi

if ${CREATE_RELEASE}; then
    if gh release view "${TAG}" "${GH_RELEASE_ARGS[@]}" >/dev/null 2>&1; then
        echo "==> GitHub Release ${TAG} already exists."
    else
        echo "==> Creating GitHub Release ${TAG}"
        gh release create "${TAG}" \
            --title "PCLMobile ${VERSION}" \
            --notes "Android AAR and mobile wrapper release ${VERSION}." \
            "${GH_RELEASE_ARGS[@]}"
    fi
fi

if ${UPLOAD_RELEASE}; then
    echo "==> Uploading AAR to GitHub Release ${TAG}"
    gh release upload "${TAG}" "${VERSIONED_AAR_PATH}" "${VERSIONED_AAR_PATH}.sha256" --clobber "${GH_RELEASE_ARGS[@]}"
fi

echo "==> Android package complete."
