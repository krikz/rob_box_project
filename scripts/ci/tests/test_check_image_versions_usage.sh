#!/usr/bin/env bash
# Unit test for scripts/ci/check_image_versions_usage.sh (ADR-0094 §3.2).
#
# Validates three behaviors:
#   1. Phantom *_TAG → script exits 1 and prints the phantom tag name.
#   2. Whitelisted phantom (IV_KNOWN_PHANTOMS) → exit 0, only WARN line.
#   3. Used *_TAG → exit 0, ✓ line.
#
# Strategy: build a tempdir with fake docker/<component>/ structure,
# point the script at it via positional args, assert exit + stderr.
#
# Run: bash scripts/ci/tests/test_check_image_versions_usage.sh
#
# ADR-0094 §7 unit test entry.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="${SCRIPT_DIR}/../check_image_versions_usage.sh"

if [ ! -x "$SCRIPT" ]; then
  echo "❌ $SCRIPT not executable"; exit 1
fi
echo "✅ script is executable"

# ------------------------------------------------------------------
# Test 1: phantom *_TAG → exit 1
# ------------------------------------------------------------------
TMP1=$(mktemp -d)
mkdir -p "$TMP1/docker/main"
cat > "$TMP1/docker/main/.image-versions.dev" <<'EOF'
# main staging
USED_TAG=dev-abc1234
PHANTOM_TAG=dev-abc1234
EOF
cat > "$TMP1/docker/main/docker-compose.yaml" <<'EOF'
services:
  foo:
    image: rob_box/foo:${USED_TAG}
EOF

set +e
out1=$(bash "$SCRIPT" "$TMP1/docker/main" 2>&1)
rc1=$?
set -e
echo "--- Test 1 output ---"
echo "$out1"
echo "---"
if [ "$rc1" -ne 1 ]; then
  echo "❌ Test 1 FAILED: expected exit 1, got $rc1"
  rm -rf "$TMP1"
  exit 1
fi
if ! echo "$out1" | grep -q 'PHANTOM_TAG.*PHANTOM'; then
  echo "❌ Test 1 FAILED: expected PHANTOM line for PHANTOM_TAG"
  rm -rf "$TMP1"
  exit 1
fi
if ! echo "$out1" | grep -q 'USED_TAG.*used'; then
  echo "❌ Test 1 FAILED: expected 'used' line for USED_TAG"
  rm -rf "$TMP1"
  exit 1
fi
echo "✅ Test 1 PASSED (phantom detected, used tag reported)"
rm -rf "$TMP1"

# ------------------------------------------------------------------
# Test 2: whitelisted phantom → exit 0, WARN
# ------------------------------------------------------------------
TMP2=$(mktemp -d)
mkdir -p "$TMP2/docker/main"
cat > "$TMP2/docker/main/.image-versions.dev" <<'EOF'
USED_TAG=dev-abc1234
EXPERIMENTAL_TAG=dev-abc1234
EOF
cat > "$TMP2/docker/main/docker-compose.yaml" <<'EOF'
services:
  foo:
    image: rob_box/foo:${USED_TAG}
EOF

set +e
out2=$(IV_KNOWN_PHANTOMS="EXPERIMENTAL_TAG:intentionally unused until #2425 done" \
       bash "$SCRIPT" "$TMP2/docker/main" 2>&1)
rc2=$?
set -e
echo "--- Test 2 output ---"
echo "$out2"
echo "---"
if [ "$rc2" -ne 0 ]; then
  echo "❌ Test 2 FAILED: expected exit 0 with whitelist, got $rc2"
  rm -rf "$TMP2"
  exit 1
fi
if ! echo "$out2" | grep -q 'EXPERIMENTAL_TAG.*whitelist'; then
  echo "❌ Test 2 FAILED: expected whitelist line for EXPERIMENTAL_TAG"
  rm -rf "$TMP2"
  exit 1
fi
echo "✅ Test 2 PASSED (whitelist honoured)"
rm -rf "$TMP2"

# ------------------------------------------------------------------
# Test 3: all used → exit 0, no warnings
# ------------------------------------------------------------------
TMP3=$(mktemp -d)
mkdir -p "$TMP3/docker/vision"
cat > "$TMP3/docker/vision/.image-versions.latest" <<'EOF'
A_TAG=latest-aaa
B_TAG=latest-bbb
EOF
cat > "$TMP3/docker/vision/docker-compose.yaml" <<'EOF'
services:
  a:
    image: rbx/a:${A_TAG}
  b:
    image: rbx/b:${B_TAG}
EOF

set +e
out3=$(bash "$SCRIPT" "$TMP3/docker/vision" 2>&1)
rc3=$?
set -e
echo "--- Test 3 output ---"
echo "$out3"
echo "---"
if [ "$rc3" -ne 0 ]; then
  echo "❌ Test 3 FAILED: expected exit 0, got $rc3"
  rm -rf "$TMP3"
  exit 1
fi
if echo "$out3" | grep -q 'PHANTOM'; then
  echo "❌ Test 3 FAILED: unexpected PHANTOM line for used tags"
  rm -rf "$TMP3"
  exit 1
fi
echo "✅ Test 3 PASSED (all tags used, no warnings)"
rm -rf "$TMP3"

# ------------------------------------------------------------------
# Test 4: empty component dir → exit 0 (no files)
# ------------------------------------------------------------------
TMP4=$(mktemp -d)
set +e
out4=$(bash "$SCRIPT" "$TMP4/does-not-exist" 2>&1)
rc4=$?
set -e
if [ "$rc4" -ne 0 ]; then
  echo "❌ Test 4 FAILED: missing dir should exit 0 (silently skipped), got $rc4"
  rm -rf "$TMP4"
  exit 1
fi
echo "✅ Test 4 PASSED (missing dir silently skipped)"
rm -rf "$TMP4"

# ------------------------------------------------------------------
# Test 5: comment lines and non-_*_TAG keys are ignored
# ------------------------------------------------------------------
TMP5=$(mktemp -d)
mkdir -p "$TMP5/docker/main"
cat > "$TMP5/docker/main/.image-versions.dev" <<'EOF'
# This is a comment
ROS_DISTRO=humble
IMAGE_REGISTRY=ghcr.io
DEPLOY_ENV=staging
FOO_TAG=dev-zzz
EOF
cat > "$TMP5/docker/main/docker-compose.yaml" <<'EOF'
services:
  foo:
    image: rbx/foo:${FOO_TAG}
EOF

set +e
out5=$(bash "$SCRIPT" "$TMP5/docker/main" 2>&1)
rc5=$?
set -e
echo "--- Test 5 output ---"
echo "$out5"
echo "---"
if [ "$rc5" -ne 0 ]; then
  echo "❌ Test 5 FAILED: expected exit 0, got $rc5"
  rm -rf "$TMP5"
  exit 1
fi
# ROS_DISTRO / IMAGE_REGISTRY / DEPLOY_ENV should NOT be checked
# (they don't end in *_TAG — only *_TAG-suffixed keys are validated)
if echo "$out5" | grep -q 'ROS_DISTRO'; then
  echo "❌ Test 5 FAILED: ROS_DISTRO should be skipped (not *_TAG)"
  rm -rf "$TMP5"
  exit 1
fi
if echo "$out5" | grep -q 'IMAGE_REGISTRY'; then
  echo "❌ Test 5 FAILED: IMAGE_REGISTRY should be skipped (not *_TAG)"
  rm -rf "$TMP5"
  exit 1
fi
echo "✅ Test 5 PASSED (non-*_TAG keys skipped)"
rm -rf "$TMP5"

echo ""
echo "======================================="
echo "All 5 tests PASSED (ADR-0094 §7)"
echo "======================================="