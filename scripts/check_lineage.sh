#!/usr/bin/env bash
#
# scripts/check_lineage.sh
#
# Verifies every factual claim in docs/LINEAGE.md against the actual git
# objects. The lineage record has been wrong twice; this makes it impossible
# for it to rot silently.
#
# Needs the predecessor's objects, which reach this repository through the
# pcl-superbuild-origin tag (83cc231). A shallow clone will not do:
#
#   git fetch --unshallow                      # if shallow
#   git fetch origin 'refs/tags/*:refs/tags/*'
#
# The refs/replace graft is NOT required — every check below names its commits
# explicitly and passes --no-replace-objects so the result is the same with or
# without it.
set -uo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

PREDECESSOR=83cc231   # pcl-superbuild's last development commit (tag: pcl-superbuild-origin)
IMPORT=98c6c50        # this repository's initial commit
LINEAGE=docs/LINEAGE.md

g() { git --no-replace-objects "$@"; }

failures=0
checks=0

check() {
    local label=$1 expected=$2 actual=$3
    checks=$((checks + 1))
    if [ "$expected" = "$actual" ]; then
        printf '  ok    %-52s %s\n' "$label" "$actual"
    else
        printf '  FAIL  %-52s expected %s, got %s\n' "$label" "$expected" "$actual"
        failures=$((failures + 1))
    fi
}

# docs/LINEAGE.md must literally contain the figure it documents, so that
# editing the prose without re-checking the tree fails here.
documents() {
    local label=$1 needle=$2
    checks=$((checks + 1))
    if grep -qF -- "$needle" "$LINEAGE"; then
        printf '  ok    %-52s %s\n' "$label" "documented"
    else
        printf '  FAIL  %-52s %s not found in %s\n' "$label" "$needle" "$LINEAGE"
        failures=$((failures + 1))
    fi
}

if ! g rev-parse -q --verify "$PREDECESSOR^{commit}" >/dev/null; then
    echo "FATAL: $PREDECESSOR is not in this clone."
    echo "       git fetch origin 'refs/tags/*:refs/tags/*'   (and --unshallow if shallow)"
    exit 2
fi

echo "== tag =="
check "pcl-superbuild-origin points at the predecessor" \
      "$(g rev-parse "$PREDECESSOR")" \
      "$(g rev-parse 'pcl-superbuild-origin^{commit}')"

echo "== the 2019 import =="
check "files at $PREDECESSOR"        72 "$(g ls-tree -r --name-only $PREDECESSOR | wc -l | tr -d ' ')"
check "files at $IMPORT"             44 "$(g ls-tree -r --name-only $IMPORT     | wc -l | tr -d ' ')"
check "left behind by the import"    44 "$(g diff --diff-filter=D --name-only $PREDECESSOR $IMPORT | wc -l | tr -d ' ')"
check "arrived with the import"      16 "$(g diff --diff-filter=A --name-only $PREDECESSOR $IMPORT | wc -l | tr -d ' ')"
check "shared but already modified"  12 "$(g diff --diff-filter=M --name-only $PREDECESSOR $IMPORT | wc -l | tr -d ' ')"
check "iOSWrapper/ files left behind" 38 \
      "$(g diff --diff-filter=D --name-only $PREDECESSOR $IMPORT | grep -c '^iOSWrapper/')"
check "predecessor history length"  683 "$(g rev-list --count $PREDECESSOR)"
check "toolchains/ at $PREDECESSOR"  15 "$(g ls-tree -r --name-only $PREDECESSOR -- toolchains/ | wc -l | tr -d ' ')"

documents "72 is documented"  '72 files'
documents "44 is documented"  '`98c6c50` has 44'
documents "683 is documented" '683-commit history'

echo "== makeFramework.sh: a working directory, not a checkout =="
check "lines at $PREDECESSOR" 488 "$(g show $PREDECESSOR:makeFramework.sh | wc -l | tr -d ' ')"
check "lines at $IMPORT"      354 "$(g show $IMPORT:makeFramework.sh      | wc -l | tr -d ' ')"

imported_blob=$(g rev-parse "$IMPORT:makeFramework.sh")
matches=$(g rev-list $PREDECESSOR | while read -r c; do
              b=$(g rev-parse -q --verify "$c:makeFramework.sh" 2>/dev/null) || continue
              [ "$b" = "$imported_blob" ] && echo hit
          done | wc -l | tr -d ' ')
check "imported blob matches no predecessor commit" 0 "$matches"

echo "== files with no pcl-superbuild ancestor (the † rows) =="
# Each must be absent from the predecessor's entire history, on every ref.
for p in strip-frameworks.sh makeFramework2.sh xamarinObjevtiveSharpie.sh \
         build_android.sh build_android.bat \
         build_ios_device_framework.sh build_ios_simulator_framework.sh \
         build_ios_universal_binary.sh build_ios_universal_framework.sh \
         toolchains/iOS_Device_ARM64e.cmake AndroidWrapper; do
    check "$p is absent from the predecessor" 0 \
          "$(g log --format=%h $PREDECESSOR -- "$p" | wc -l | tr -d ' ')"
done

echo "== files the import did NOT invent =="
# appveyor/ is present at the predecessor and was carried over byte-identically;
# LINEAGE.md used to claim it had been deleted there.
check "appveyor/ tree unchanged by the import" \
      "$(g rev-parse "$PREDECESSOR:appveyor")" "$(g rev-parse "$IMPORT:appveyor")"
check "toolchains/iOS.toolchain.cmake was deleted at 63ee1a4" 63ee1a4 \
      "$(g log --format=%h --diff-filter=D -1 $PREDECESSOR -- toolchains/iOS.toolchain.cmake)"

echo "== the AAR module is native to this repository =="
check "pclmobile module first appears in e21dc8a" e21dc8a \
      "$(g log --format=%h --diff-filter=A -- 'AndroidWrapper/aar/pclmobile/*' | tail -1)"
check "AndroidWrapper/.circleci added by 90b905e" 90b905e \
      "$(g log --format=%h --diff-filter=A -- 'AndroidWrapper/.circleci/config.yml' | tail -1)"

echo "== byte-identity that the docs promise =="
check "cmake/toolchains/pcl-try-run-results.cmake unchanged" \
      "$(g rev-parse "$PREDECESSOR:toolchains/pcl-try-run-results.cmake")" \
      "$(git hash-object cmake/toolchains/pcl-try-run-results.cmake)"

echo "== .deprecated/ =="
late=0 diverged=0
while IFS= read -r f; do
    b=${f#.deprecated/}; b=${b%.original}
    if ! orig=$(g rev-parse -q --verify "$IMPORT:$b" 2>/dev/null); then
        late=$((late + 1))
        continue
    fi
    [ "$(git hash-object "$f")" = "$orig" ] || diverged=$((diverged + 1))
done < <(find .deprecated -name '*.original' | sort)
check "originals that post-date the import"  8 "$late"
check "originals edited after the import"    6 "$diverged"

echo "== provenance headers =="
# Files still living at a path the predecessor had, which carry no header.
# LINEAGE.md names exactly these three as deliberate omissions.
headerless=""
for p in $(g ls-tree -r --name-only $PREDECESSOR); do
    [ -f "$p" ] || continue
    grep -q "pcl-superbuild@83cc231" "$p" 2>/dev/null || headerless="$headerless $p"
done
check "headerless descendants" " .gitignore README.md iOSWrapper/module.modulemap" "$headerless"

for p in .gitignore README.md iOSWrapper/module.modulemap; do
    documents "$p is listed as an omission" "$p"
done

echo
if [ "$failures" -eq 0 ]; then
    echo "$checks checks passed — docs/LINEAGE.md matches the tree."
    exit 0
fi
echo "$failures of $checks checks FAILED. Either the tree changed or docs/LINEAGE.md is wrong."
exit 1
