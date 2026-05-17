#!/usr/bin/env bash
#
# scripts/release.sh — one-shot release pipeline for PCLMobile.
#
# What it does:
#   1. Build the XCFramework (calls scripts/build_ios.sh).
#   2. Re-zip and compute SHA256 + Swift PM checksum.
#   3. Patch Package.swift and PCLMobile.podspec with the new version
#      number, URL and checksum.
#   4. Optionally create a git tag and a GitHub Release with the zip
#      attached (controlled by --publish).
#
# Usage:
#   scripts/release.sh <version>                # rebuild + patch files only
#   scripts/release.sh <version> --publish      # also tag, push and gh-release
#
# Required env (only when --publish):
#   gh CLI authenticated (`gh auth status`).
#
# Examples:
#   scripts/release.sh 0.1.0
#   scripts/release.sh 0.1.0 --publish
set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <version> [--publish]" >&2
    exit 64
fi

VERSION="$1"
PUBLISH=false
if [[ "${2:-}" == "--publish" ]]; then
    PUBLISH=true
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

OWNER="${PCLMOBILE_OWNER:-Sirokujira}"
REPO="${PCLMOBILE_REPO:-pcl_mobile_framework}"
TAG="v${VERSION}"
ASSET_NAME="PCLMobile.xcframework.zip"
ASSET_URL="https://github.com/${OWNER}/${REPO}/releases/download/${TAG}/${ASSET_NAME}"

# ---------------------------------------------------------------------------
# 1. Build XCFramework
# ---------------------------------------------------------------------------
echo "==> Building XCFramework via scripts/build_ios.sh"
"${REPO_ROOT}/scripts/build_ios.sh"

XCFW_DIR="${REPO_ROOT}/build/ios/xcframework"
XCFW="${XCFW_DIR}/PCLMobile.xcframework"
ZIP="${XCFW_DIR}/${ASSET_NAME}"

if [[ ! -d "${XCFW}" ]]; then
    echo "ERROR: ${XCFW} not produced by build_ios.sh." >&2
    exit 1
fi

# build_ios.sh already produces a zip, but we recompute here so the
# release script is independently runnable (e.g. when only patching files).
echo "==> Re-zipping ${XCFW}"
rm -f "${ZIP}"
( cd "${XCFW_DIR}" && zip -ryq "${ASSET_NAME}" PCLMobile.xcframework )

# ---------------------------------------------------------------------------
# 2. Compute checksums
# ---------------------------------------------------------------------------
SHA256="$(shasum -a 256 "${ZIP}" | awk '{print $1}')"
echo "==> SHA256       : ${SHA256}"

# Swift PM uses its own format: try `swift package compute-checksum` if
# available, fall back to plain SHA256 (which Swift accepts since 5.6).
if command -v swift >/dev/null 2>&1; then
    SPM_CHECKSUM="$(swift package compute-checksum "${ZIP}" 2>/dev/null || true)"
fi
if [[ -z "${SPM_CHECKSUM:-}" ]]; then
    SPM_CHECKSUM="${SHA256}"
fi
echo "==> SPM checksum : ${SPM_CHECKSUM}"

# ---------------------------------------------------------------------------
# 3. Patch the three distribution manifests
# ---------------------------------------------------------------------------
echo "==> Patching Package.swift"
python3 - <<PY
import pathlib, re
p = pathlib.Path("Package.swift")
s = p.read_text()
s = re.sub(
    r'(let xcframeworkURL\s*=\s*\n?\s*")[^"]*(")',
    r'\g<1>${ASSET_URL}\g<2>',
    s, count=1)
s = re.sub(
    r'(let xcframeworkChecksum\s*=\s*\n?\s*")[^"]*(")',
    r'\g<1>${SPM_CHECKSUM}\g<2>',
    s, count=1)
p.write_text(s)
print("OK Package.swift")
PY

echo "==> Patching iOSWrapper/PCLMobile.podspec"
python3 - <<PY
import pathlib, re
p = pathlib.Path("iOSWrapper/PCLMobile.podspec")
s = p.read_text()
s = re.sub(
    r"(s\.version\s*=\s*')([^']+)(')",
    r"\g<1>${VERSION}\g<3>",
    s, count=1)
p.write_text(s)
print("OK PCLMobile.podspec")
PY

# ---------------------------------------------------------------------------
# 4. Optional publish
# ---------------------------------------------------------------------------
if ! ${PUBLISH}; then
    echo
    echo "Patched manifests for v${VERSION}."
    echo "Re-run with --publish to tag and create the GitHub Release."
    echo
    echo "Manual follow-up:"
    echo "  git add Package.swift iOSWrapper/PCLMobile.podspec"
    echo "  git commit -m 'release: PCLMobile ${VERSION}'"
    echo "  git tag ${TAG}"
    echo "  git push origin main ${TAG}"
    echo "  gh release create ${TAG} ${ZIP} --title 'PCLMobile ${VERSION}' --notes '...'"
    echo "  pod trunk push iOSWrapper/PCLMobile.podspec --allow-warnings"
    exit 0
fi

if ! command -v gh >/dev/null 2>&1; then
    echo "ERROR: --publish needs the GitHub CLI (gh). Install with: brew install gh" >&2
    exit 1
fi

echo "==> git commit / tag / push"
git add Package.swift iOSWrapper/PCLMobile.podspec
git diff --cached --stat
git commit -m "release: PCLMobile ${VERSION}"
git tag "${TAG}"
git push origin HEAD "${TAG}"

echo "==> gh release create ${TAG}"
gh release create "${TAG}" "${ZIP}" \
    --title "PCLMobile ${VERSION}" \
    --notes "Auto-generated release. SHA256: ${SHA256}"

echo
echo "✅ Released ${TAG}."
echo "Next: pod trunk push iOSWrapper/PCLMobile.podspec --allow-warnings"
