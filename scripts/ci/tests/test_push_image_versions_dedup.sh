#!/usr/bin/env bash
# Unit test for scripts/ci/push-image-versions.sh — dedup через gh api (#1630)
#
# Validates:
#   - DEDUP=1 + gh возвращает subject в последних 20 коммитах → exit 0 без push
#   - DEDUP=1 + gh возвращает пустой ответ → push выполняется
#   - DEDUP=0 (disabled) → push выполняется даже если subject уже на remote
#   - gh failure → push всё равно выполняется (fail-open)
#
# Strategy: подменяем `gh` на stub (через PATH), который возвращает заранее
# заготовленный JSON. Скрипт читает его, решает skip или push. Проверяем, что
# git push вызвался (или не вызвался) по логу / rc / counter-файлу.
#
# Run: bash scripts/ci/tests/test_push_image_versions_dedup.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="${SCRIPT_DIR}/../push-image-versions.sh"

if [ ! -x "$SCRIPT" ]; then
  echo "❌ $SCRIPT not executable"; exit 1
fi

if ! grep -q 'IMAGE_VERSIONS_DEDUP_REMOTE_QUERY' "$SCRIPT"; then
  echo "❌ $SCRIPT missing dedup hook"; exit 1
fi
echo "✅ dedup hook present"

# -----------------------------------------------------------------------------
# Test fixture: a local bare repo on disk, real origin. We use git-daemon-less
# file:// transport — no network, fast, deterministic.
# -----------------------------------------------------------------------------
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/origin.git" "$WORK/work"
git init -q --bare --initial-branch=main "$WORK/origin.git"
cd "$WORK/work"
git init -q --initial-branch=main .
git -c user.email=t@x -c user.name=t commit -q --allow-empty -m "init"
git remote add origin "$WORK/origin.git"
git push -q origin main 2>/dev/null

# --- Test 1: dedup finds subject → exit 0, push skipped ---------------------
# Pre-create a commit on origin/main with the SAME subject our run will produce.
SUBJECT="ci: main SHA tags → dev-aaa1111 [skip ci]"
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "$SUBJECT"
git push -q origin main 2>/dev/null

# Create a local commit (simulating the build's update-image-versions step).
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "$SUBJECT"

# Stub gh: returns JSON containing our subject. PATH override so push-image-versions.sh
# picks up our fake before the real gh.
STUB_BIN="$WORK/stubbin"
mkdir -p "$STUB_BIN"
cat > "$STUB_BIN/gh" <<EOF
#!/usr/bin/env bash
# Fake gh that always returns our subject in the response.
cat <<JSON
[{"sha":"deadbeef","commit":{"message":"${SUBJECT}\n"}}]
JSON
EOF
chmod +x "$STUB_BIN/gh"

set +e
PATH="$STUB_BIN:$PATH" \
  IMAGE_VERSIONS_DEDUP_REMOTE_QUERY=1 \
  IMAGE_VERSIONS_PUSH_ATTEMPTS=1 \
  IMAGE_VERSIONS_PUSH_RETRY_DELAY=0 \
  bash "$SCRIPT" main main >/tmp/dedup1.out 2>&1
rc1=$?
set -e

if [ "$rc1" -ne 0 ]; then
  echo "❌ test 1: expected exit 0 (dedup skip), got $rc1"
  cat /tmp/dedup1.out
  exit 1
fi
if ! grep -q 'dedup' /tmp/dedup1.out; then
  echo "❌ test 1: expected 'dedup' message in output, got:"
  cat /tmp/dedup1.out
  exit 1
fi
echo "✅ test 1: dedup skips push when subject exists on remote"

# --- Test 2: dedup finds nothing → push proceeds ----------------------------
# Reset: create a different subject, push that, then try pushing ours.
git reset --hard origin/main >/dev/null 2>&1
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "ci: main SHA tags → dev-bbb2222 [skip ci]"
git push -q origin main 2>/dev/null
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "ci: main SHA tags → dev-ccc3333 [skip ci]"

# Replace gh stub to return JSON WITHOUT our subject (only the previous one).
cat > "$STUB_BIN/gh" <<'EOF'
#!/usr/bin/env bash
cat <<'JSON'
[{"sha":"deadbeef","commit":{"message":"ci: main SHA tags → dev-bbb2222 [skip ci]\n"}}]
JSON
EOF
chmod +x "$STUB_BIN/gh"

set +e
PATH="$STUB_BIN:$PATH" \
  IMAGE_VERSIONS_DEDUP_REMOTE_QUERY=1 \
  IMAGE_VERSIONS_PUSH_ATTEMPTS=1 \
  IMAGE_VERSIONS_PUSH_RETRY_DELAY=0 \
  bash "$SCRIPT" main main >/tmp/dedup2.out 2>&1
rc2=$?
set -e

if [ "$rc2" -ne 0 ]; then
  echo "❌ test 2: expected push to succeed, got rc=$rc2"
  cat /tmp/dedup2.out
  exit 1
fi
if ! grep -q 'pushed' /tmp/dedup2.out; then
  echo "❌ test 2: expected 'pushed' message, got:"
  cat /tmp/dedup2.out
  exit 1
fi
echo "✅ test 2: dedup allows push when subject not on remote"

# --- Test 3: dedup disabled → push proceeds even if subject exists ---------
git reset --hard origin/main >/dev/null 2>&1
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "ci: main SHA tags → dev-ddd4444 [skip ci]"
git push -q origin main 2>/dev/null
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "ci: main SHA tags → dev-ddd4444 [skip ci]"

# gh stub: returns the same subject as ours.
cat > "$STUB_BIN/gh" <<'EOF'
#!/usr/bin/env bash
cat <<'JSON'
[{"sha":"deadbeef","commit":{"message":"ci: main SHA tags → dev-ddd4444 [skip ci]\n"}}]
JSON
EOF
chmod +x "$STUB_BIN/gh"

set +e
PATH="$STUB_BIN:$PATH" \
  IMAGE_VERSIONS_DEDUP_REMOTE_QUERY=0 \
  IMAGE_VERSIONS_PUSH_ATTEMPTS=1 \
  IMAGE_VERSIONS_PUSH_RETRY_DELAY=0 \
  bash "$SCRIPT" main main >/tmp/dedup3.out 2>&1
rc3=$?
set -e

if [ "$rc3" -ne 0 ]; then
  echo "❌ test 3: expected push to succeed (dedup OFF), got rc=$rc3"
  cat /tmp/dedup3.out
  exit 1
fi
if ! grep -q 'pushed' /tmp/dedup3.out; then
  echo "❌ test 3: expected 'pushed' message, got:"
  cat /tmp/dedup3.out
  exit 1
fi
echo "✅ test 3: DEDUP=0 disables dedup, push proceeds"

# --- Test 4: gh failure → push proceeds (fail-open) ------------------------
git reset --hard origin/main >/dev/null 2>&1
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "ci: main SHA tags → dev-eee5555 [skip ci]"
git push -q origin main 2>/dev/null
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "ci: main SHA tags → dev-eee5555 [skip ci]"

# gh stub: ALWAYS fails (simulates auth/network outage).
cat > "$STUB_BIN/gh" <<'EOF'
#!/usr/bin/env bash
echo "fake gh error" >&2
exit 1
EOF
chmod +x "$STUB_BIN/gh"

set +e
PATH="$STUB_BIN:$PATH" \
  IMAGE_VERSIONS_DEDUP_REMOTE_QUERY=1 \
  IMAGE_VERSIONS_PUSH_ATTEMPTS=1 \
  IMAGE_VERSIONS_PUSH_RETRY_DELAY=0 \
  bash "$SCRIPT" main main >/tmp/dedup4.out 2>&1
rc4=$?
set -e

if [ "$rc4" -ne 0 ]; then
  echo "❌ test 4: expected push to proceed on gh failure, got rc=$rc4"
  cat /tmp/dedup4.out
  exit 1
fi
if ! grep -qi 'gh api dedup query failed' /tmp/dedup4.out; then
  echo "❌ test 4: expected warning about gh failure, got:"
  cat /tmp/dedup4.out
  exit 1
fi
echo "✅ test 4: gh failure → fail-open (push proceeds with warning)"

echo ""
echo "PASS — all 4 dedup tests green"
