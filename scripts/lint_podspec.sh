#!/usr/bin/env bash
#
# scripts/lint_podspec.sh
#
# One-shot local CocoaPods validation for iOSWrapper/PCLMobile.podspec.
#
# What it does:
#   1. Verify Xcode + cocoapods are present.
#   2. Make sure the XCFramework exists (run scripts/build_ios.sh first if not).
#   3. Stage the XCFramework next to the podspec so `vendored_frameworks`
#      resolves locally (lint refuses to touch s.source.http unless we
#      flip it, so we work with the file layout instead).
#   4. Run `pod lib lint`. By default we run the **full** validation
#      (xcodebuild compile included) so that issues surfaced by
#      `pod trunk push` -- like the UIUtilities auto-link error -- are
#      caught here.
#   5. Clean up the staged XCFramework on exit.
#
# Modes (PODSPEC_LINT_MODE env var):
#   full   (default) -- runs xcodebuild against a sample app linking the
#                       framework. Slow but matches what `pod trunk push`
#                       does.
#   quick           -- adds --skip-import-validation --skip-tests, so the
#                       xcodebuild step is skipped. Useful for syntax-only
#                       checks while iterating on the podspec itself.
#
# Override with arbitrary flags via PODSPEC_LINT_FLAGS:
#   PODSPEC_LINT_FLAGS="--verbose --use-libraries" ./scripts/lint_podspec.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PODSPEC="${REPO_ROOT}/iOSWrapper/PCLMobile.podspec"
XCFW_SRC="${REPO_ROOT}/build/ios/xcframework/PCLMobile.xcframework"
XCFW_STAGED="${REPO_ROOT}/iOSWrapper/PCLMobile.xcframework"

MODE="${PODSPEC_LINT_MODE:-full}"
case "${MODE}" in
    full)  DEFAULT_FLAGS="--allow-warnings --verbose";;
    quick) DEFAULT_FLAGS="--allow-warnings --skip-import-validation --skip-tests";;
    *)     echo "ERROR: PODSPEC_LINT_MODE must be 'full' or 'quick'." >&2; exit 64;;
esac

LINT_FLAGS="${PODSPEC_LINT_FLAGS:-${DEFAULT_FLAGS}}"

# ---------------------------------------------------------------------------
# Sanity checks
# ---------------------------------------------------------------------------
if [[ "$(uname -s)" != "Darwin" ]]; then
    echo "ERROR: pod lib lint must run on macOS (Xcode required)." >&2
    exit 1
fi

if ! command -v pod >/dev/null 2>&1; then
    echo "ERROR: 'pod' not found. Install CocoaPods first:" >&2
    echo "  sudo gem install cocoapods" >&2
    exit 1
fi

if [[ ! -f "${PODSPEC}" ]]; then
    echo "ERROR: podspec missing at ${PODSPEC}" >&2
    exit 1
fi

if [[ ! -d "${XCFW_SRC}" ]]; then
    echo "ERROR: XCFramework not built. Run scripts/build_ios.sh first."  >&2
    echo "  expected: ${XCFW_SRC}"                                         >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Stage the XCFramework next to the podspec, lint, then unstage. Trap so
# the staged copy is wiped even if pod lint exits non-zero.
# ---------------------------------------------------------------------------
cleanup() {
    rm -rf "${XCFW_STAGED}"
}
trap cleanup EXIT

echo "==> Staging ${XCFW_SRC} -> ${XCFW_STAGED}"
rm -rf "${XCFW_STAGED}"
cp -R "${XCFW_SRC}" "${XCFW_STAGED}"

echo "==> Quick Ruby syntax check"
ruby -c "${PODSPEC}"

echo "==> pod lib lint  (mode=${MODE})"
echo "    flags: ${LINT_FLAGS}"
( cd "${REPO_ROOT}/iOSWrapper" && pod lib lint PCLMobile.podspec ${LINT_FLAGS} )

echo
echo "✅ podspec passed local lint."
echo "   Next: tag a release, push the XCFramework zip, then:"
echo "       pod trunk push iOSWrapper/PCLMobile.podspec --allow-warnings"
