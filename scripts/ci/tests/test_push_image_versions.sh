#!/usr/bin/env bash
# Unit test for scripts/ci/push-image-versions.sh
#
# Validates issue #t_dd49849b fix: GIT_TERMINAL_TIMEOUT is honoured and the
# script fails fast (not hangs) when git network operations stall.
#
# Strategy: run the script with a very small GIT_TERMINAL_TIMEOUT against an
# unreachable remote (RFC5737 TEST-NET-1 192.0.2.1). It must exit non-zero
# within IMAGE_VERSIONS_GIT_TERMINAL_TIMEOUT + retry overhead — well under
# the 10-min job-level timeout we added on update-image-versions jobs.
#
# Run: bash scripts/ci/tests/test_push_image_versions.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="${SCRIPT_DIR}/../push-image-versions.sh"

if [ ! -x "$SCRIPT" ]; then
  echo "❌ $SCRIPT not executable"; exit 1
fi

# Make sure the script has the GIT_TERMINAL_TIMEOUT export.
if ! grep -q 'IMAGE_VERSIONS_GIT_TERMINAL_TIMEOUT' "$SCRIPT"; then
  echo "❌ $SCRIPT missing IMAGE_VERSIONS_GIT_TERMINAL_TIMEOUT env hook"; exit 1
fi
echo "✅ env hook present"

# End-to-end: try to push to an unreachable remote with a 5-second cap.
# Must exit non-zero within ~30 seconds (5 attempts × 5s sleep + 5s timeout each).
TMPREPO=$(mktemp -d)
cd "$TMPREPO"
git init -q --bare --initial-branch=main origin.git >/dev/null
git init -q --initial-branch=main work >/dev/null
cd work
git -c user.email=test@x -c user.name=test commit -q --allow-empty -m "init"
# Point 'origin' at TEST-NET-1 (RFC5737) — guaranteed unreachable.
git remote add origin "https://192.0.2.1/nope.git"

start=$(date +%s)
set +e
IMAGE_VERSIONS_GIT_TERMINAL_TIMEOUT=5 \
  IMAGE_VERSIONS_PUSH_ATTEMPTS=2 \
  IMAGE_VERSIONS_PUSH_RETRY_DELAY=1 \
  bash "$SCRIPT" main test-component 2>/tmp/script.err
rc=$?
set -e
end=$(date +%s)
elapsed=$((end - start))

echo "rc=$rc elapsed=${elapsed}s"

if [ "$rc" -eq 0 ]; then
  echo "❌ script returned success against unreachable remote (expected non-zero)"
  exit 1
fi

# Should fail fast — well under the 10-min job-level cap. The script's
# GIT_TERMINAL_TIMEOUT=5 (per env override) applies to each git call; with 2
# attempts × ~60s for unreachable TEST-NET-1 TLS handshake + 1s sleep = ~120s
# is the realistic ceiling on this network. Allow generous headroom.
if [ "$elapsed" -gt 180 ]; then
  echo "❌ script did not fail fast (took ${elapsed}s, expected <180s)"
  cat /tmp/script.err
  exit 1
fi
echo "✅ script failed fast (${elapsed}s) under unreachable remote"

# Sanity: stderr mentions timeout OR pull/push failure
if grep -qi 'timeout\|failed\|aborted' /tmp/script.err; then
  echo "✅ stderr explains failure"
else
  echo "❌ stderr is empty or unrelated"
  cat /tmp/script.err
  exit 1
fi

rm -rf "$TMPREPO"
echo "PASS"